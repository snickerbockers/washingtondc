/*******************************************************************************
 *
 *
 *    WashingtonDC Dreamcast Emulator
 *    Copyright (C) 2019 snickerbockers
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 ******************************************************************************/

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <errno.h>
#include <sys/stat.h>

#include "log.h"
#include "fifo.h"

#include "config_file.h"

#define CFG_NODE_KEY_LEN 256
#define CFG_NODE_VAL_LEN 256

#define CFG_FILE_NAME "wash.cfg"

struct cfg_node {
    struct fifo_node list_node;
    char key[CFG_NODE_KEY_LEN];
    char val[CFG_NODE_VAL_LEN];
};

enum cfg_parse_state {
    CFG_PARSE_PRE_KEY,
    CFG_PARSE_KEY,
    CFG_PARSE_PRE_VAL,
    CFG_PARSE_VAL,
    CFG_PARSE_POST_VAL,
    CFG_PARSE_ERROR
};

static struct cfg_state {
    enum cfg_parse_state state;
    unsigned key_len, val_len;
    char key[CFG_NODE_KEY_LEN];
    char val[CFG_NODE_VAL_LEN];
    unsigned line_count;
    struct fifo_head cfg_nodes;
    bool in_comment;
} cfg_state;

static void cfg_add_entry(void);
static void cfg_handle_newline(void);
static int cfg_parse_bool(char const *val, bool *outp);

#define CFG_PATH_LEN 4096

static void path_append(char *dst, char const *src, size_t dst_sz) {
    if (!src[0])
        return; // nothing to append

    // get the index of the null terminator
    unsigned zero_idx = 0;
    while (dst[zero_idx])
        zero_idx++;

    /*
     * If there's a trailing / on dst and a leading / on src then get rid of
     * the leading slash on src.
     *
     * If there is not a trailing / on dst and there is not a leading slash on
     * src then give dst a trailing /.
     */
    if (dst[zero_idx - 1] == '/' && src[0] == '/') {
        // remove leading / from src
        src = src + 1;
        if (!src[0])
            return; // nothing to append
    } else if (dst[zero_idx - 1] != '/' && src[0] != '/') {
        // add trailing / to dst
        if (zero_idx < dst_sz - 1) {
            dst[zero_idx++] = '/';
            dst[zero_idx] = '\0';
        } else {
            return; // out of space
        }
    }

    // there's no more space
    if (zero_idx >= dst_sz -1 )
        return;

    strncpy(dst + zero_idx, src, dst_sz - zero_idx);
    dst[dst_sz - 1] = '\0';
}

char const *cfg_get_default_dir(void) {
    static char path[CFG_PATH_LEN];
    char const *config_root = getenv("XDG_CONFIG_HOME");
    if (config_root) {
        strncpy(path, config_root, CFG_PATH_LEN);
        path[CFG_PATH_LEN - 1] = '\0';
    } else {
        char const *home_dir = getenv("HOME");
        if (home_dir) {
            strncpy(path, home_dir, CFG_PATH_LEN);
            path[CFG_PATH_LEN - 1] = '\0';
        } else {
            return NULL;
        }
        path_append(path, "/.config", CFG_PATH_LEN);
    }
    path_append(path, "washdc", CFG_PATH_LEN);
    return path;
}

char const *cfg_get_default_file(void) {
    static char path[CFG_PATH_LEN];
    char const *cfg_dir = cfg_get_default_dir();
    if (!cfg_dir)
        return NULL;
    strncpy(path, cfg_dir, CFG_PATH_LEN);
    path[CFG_PATH_LEN - 1] = '\0';
    path_append(path, "wash.cfg", CFG_PATH_LEN);
    return path;
}

static void cfg_create_default_config(void) {
    char const *cfg_file_path = cfg_get_default_file();
    char const *cfg_file_dir = cfg_get_default_dir();

    static char const *cfg_default =
        ";;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;"
        ";;;;;;;;;;\n"
        ";;\n"
        ";; AUTOMATICALLY GENERATED BY WASHINGTONDC\n"
        ";;\n"
        ";; This is WashingtonDC's config file.  Config settings consist of a "
        "config\n"
        ";; name followed by its value on the same line\n"
        ";; the semicolon (;) character can be used to create single-line "
        "comments\n"
        ";;\n"
        ";;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;"
        ";;;;;;;;;;\n"
        "\n"
        "; background color (use html hex syntax)\n"
        "ui.bgcolor #3d77c0\n"
        "\n"
        "; vsync\n"
        "; options are true (to enable) or false (to disable)\n"
        "win.vsync false\n"
        "\n"
        "; framelimit mode\n"
        "; options are:\n"
        ";     unlimited - run WashingtonDC as fast as possible\n"
        ";     spin - limit framerate with an idle loop "
        "(accurate but wastes CPU time)\n"
        ";     sleep - limit framerate by sleeping (less accurate but more "
        "efficient)\n"
        "win.framelimit_mode spin\n"
        "\n"
        /*
         * TODO: find a way to explain the naming convention for control
         * bindings to end-users
         */
        "wash.ctrl.toggle-overlay kbd.f2\n"
        "\n"
        "; mapping d-pad to the right joystick because I forgot to implement the\n"
        "; hat lol\n"
        "dc.ctrl.p1_1.dpad-up js0.axis4-\n"
        "dc.ctrl.p1_1.dpad-left js0.axis3-\n"
        "dc.ctrl.p1_1.dpad-down js0.axis4+\n"
        "dc.ctrl.p1_1.dpad-right js0.axis3+\n"
        "dc.ctrl.p1_1.stick-left  js0.axis0-\n"
        "dc.ctrl.p1_1.stick-right js0.axis0+\n"
        "dc.ctrl.p1_1.stick-up    js0.axis1+\n"
        "dc.ctrl.p1_1.stick-down  js0.axis1-\n"
        "dc.ctrl.p1_1.trig-l      js0.axis2\n"
        "dc.ctrl.p1_1.trig-r      js0.axis5\n"
        "dc.ctrl.p1_1.btn-a js0.btn0\n"
        "dc.ctrl.p1_1.btn-b js0.btn1\n"
        "dc.ctrl.p1_1.btn-x js0.btn2\n"
        "dc.ctrl.p1_1.btn-y js0.btn3\n"
        "dc.ctrl.p1_1.btn-start js0.btn7\n"
        "\n"
        "dc.ctrl.p1_2.dpad-up    kbd.w\n"
        "dc.ctrl.p1_2.dpad-left  kbd.a\n"
        "dc.ctrl.p1_2.dpad-down  kbd.s\n"
        "dc.ctrl.p1_2.dpad-right kbd.d\n"
        "dc.ctrl.p1_2.btn-a      kbd.keypad2\n"
        "dc.ctrl.p1_2.btn-b      kbd.keypad6\n"
        "dc.ctrl.p1_2.btn-x      kbd.keypad4\n"
        "dc.ctrl.p1_2.btn-y      kbd.keypad8\n"
        "dc.ctrl.p1_2.btn-start  kbd.space\n";

    LOG_INFO("Attempting to create configuration directory \"%s\"\n",
             cfg_file_dir);
    if (mkdir(cfg_file_dir, S_IRUSR | S_IWUSR | S_IXUSR) != 0) {
        if (errno == EEXIST) {
            LOG_INFO("The directory already exists, I'm going to assume that's "
                     "a good thing...\n");
        } else {
            LOG_ERROR("Unable to create %s: %s\n", cfg_file_dir, strerror(errno));
            return;
        }
    }

    FILE *cfg_file = fopen(cfg_file_path, "w");
    if (!cfg_file) {
        LOG_ERROR("unable to create %s: %s\n", cfg_file_path, strerror(errno));
        return;
    }

    if (fputs(cfg_default, cfg_file) == EOF) {
        LOG_ERROR("Unable to write default config to %s\n", cfg_file_path);
    }

    fclose(cfg_file);
}

void cfg_init(void) {
    memset(&cfg_state, 0, sizeof(cfg_state));
    cfg_state.state = CFG_PARSE_PRE_KEY;

    char const *cfg_file_path = cfg_get_default_file();
    char const *cfg_file_dir = cfg_get_default_dir();

    if (cfg_file_dir)
        LOG_INFO("cfg directory is \"%s\"\n", cfg_file_dir);
    else
        LOG_ERROR("unable to determine cfg directory\n");

    if (cfg_file_path)
        LOG_INFO("Using cfg file \"%s\"\n", cfg_file_path);
    else
        LOG_ERROR("Unable to determine location of cfg file\n");

    fifo_init(&cfg_state.cfg_nodes);

    FILE *cfg_file = fopen(cfg_file_path, "r");

    if (!cfg_file) {
        cfg_create_default_config();
        cfg_file = fopen(cfg_file_path, "r");
    }

    if (cfg_file) {
        LOG_INFO("Parsing configuration file %s\n", CFG_FILE_NAME);
        for (;;) {
            int ch = fgetc(cfg_file);
            if (ch == EOF)
                break;
            cfg_put_char(ch);
        }
        cfg_put_char('\n'); // in case the last line doesn't end with newline
        fclose(cfg_file);
    } else {
        LOG_INFO("Unable to open %s; does it even exist?\n", CFG_FILE_NAME);
    }
}

void cfg_cleanup(void) {
    struct fifo_node *curs;

    while ((curs = fifo_pop(&cfg_state.cfg_nodes)) != NULL) {
        struct cfg_node *node = &FIFO_DEREF(curs, struct cfg_node, list_node);
        free(node);
    }
}

void cfg_put_char(char ch) {
    /*
     * special case - a null terminator counts as a newline so that any data
     * which does not end in a newline can be flushed.
     */
    if (ch == '\0')
        ch = '\n';

    /*
     * Very simple preprocessor - replace comments with whitespace and
     * otherwise don't modify the parser state
     */
    if (ch == ';')
        cfg_state.in_comment = true;
    if (cfg_state.in_comment) {
        if (ch == '\n')
            cfg_state.in_comment = false;
        else
            ch = ' ';
    }

    switch (cfg_state.state) {
    case CFG_PARSE_PRE_KEY:
        if (ch == '\n') {
            cfg_handle_newline();
        } else if (!isspace(ch)) {
            cfg_state.state = CFG_PARSE_KEY;
            cfg_state.key_len = 1;
            cfg_state.key[0] = ch;
        }
        break;
    case CFG_PARSE_KEY:
        if (ch == '\n') {
            LOG_ERROR("*** CFG ERROR INCOMPLETE LINE %u ***\n", cfg_state.line_count);
            cfg_handle_newline();
        } else if (isspace(ch)) {
            cfg_state.state = CFG_PARSE_PRE_VAL;
            cfg_state.key[cfg_state.key_len] = '\0';
        } else if (cfg_state.key_len < CFG_NODE_KEY_LEN - 1) {
            cfg_state.key[cfg_state.key_len++] = ch;
        } else {
            LOG_WARN("CFG file dropped char from line %u; key length is "
                     "limited to %u characters\n",
                     cfg_state.line_count, CFG_NODE_KEY_LEN - 1);
        }
        break;
    case CFG_PARSE_PRE_VAL:
        if (ch == '\n') {
            LOG_ERROR("*** CFG ERROR INCOMPLETE LINE %u ***\n", cfg_state.line_count);
            cfg_handle_newline();
        } else if (!isspace(ch)) {
            cfg_state.state = CFG_PARSE_VAL;
            cfg_state.val_len = 1;
            cfg_state.val[0] = ch;
        }
        break;
    case CFG_PARSE_VAL:
        if (ch == '\n') {
            cfg_state.val[cfg_state.val_len] = '\0';
            cfg_add_entry();
            cfg_handle_newline();
        } else if (isspace(ch)) {
            cfg_state.state = CFG_PARSE_POST_VAL;
            cfg_state.val[cfg_state.val_len] = '\0';
        } else if (cfg_state.val_len < CFG_NODE_VAL_LEN - 1) {
            cfg_state.val[cfg_state.val_len++] = ch;
        } else {
            LOG_WARN("CFG file dropped char from line %u; value length is "
                     "limited to %u characters\n",
                     cfg_state.line_count, CFG_NODE_VAL_LEN - 1);
        }
        break;
    case CFG_PARSE_POST_VAL:
        if (ch == '\n') {
            cfg_add_entry();
            cfg_handle_newline();
        } else if (!isspace(ch)) {
            cfg_state.state = CFG_PARSE_ERROR;
            LOG_ERROR("*** CFG ERROR INVALID DATA LINE %u ***\n", cfg_state.line_count);
        }
        break;
    default:
    case CFG_PARSE_ERROR:
        if (ch == '\n')
            cfg_handle_newline();
        break;
    }
}

static void cfg_add_entry(void) {
    struct cfg_node *dst_node = NULL;
    struct fifo_node *curs;

    FIFO_FOREACH(cfg_state.cfg_nodes, curs) {
        struct cfg_node *node = &FIFO_DEREF(curs, struct cfg_node, list_node);
        if (strcmp(node->key, cfg_state.key) == 0) {
            dst_node = node;
            break;
        }
    }

    if (dst_node) {
        LOG_INFO("CFG overwriting existing config key \"%s\" at line %u\n",
                 cfg_state.key, cfg_state.line_count);
    } else {
        LOG_INFO("CFG allocating new config key \"%s\" at line %u\n",
                 cfg_state.key, cfg_state.line_count);
        dst_node = (struct cfg_node*)malloc(sizeof(struct cfg_node));
        memcpy(dst_node->key, cfg_state.key, sizeof(dst_node->key));
        fifo_push(&cfg_state.cfg_nodes, &dst_node->list_node);
    }

    if (dst_node)
        memcpy(dst_node->val, cfg_state.val, sizeof(dst_node->val));
    else
        LOG_ERROR("CFG file dropped line %u due to failed node allocation\n", cfg_state.line_count);
}

static void cfg_handle_newline(void) {
    cfg_state.state = CFG_PARSE_PRE_KEY;
    cfg_state.key_len = 0;
    cfg_state.val_len = 0;
    cfg_state.line_count++;
}

char const *cfg_get_node(char const *key) {
    struct fifo_node *curs;

    FIFO_FOREACH(cfg_state.cfg_nodes, curs) {
        struct cfg_node *node = &FIFO_DEREF(curs, struct cfg_node, list_node);
        if (strcmp(node->key, key) == 0)
            return node->val;
    }

    return NULL;
}

static int cfg_parse_bool(char const *valstr, bool *outp) {
    if (strcmp(valstr, "true") == 0 || strcmp(valstr, "1") == 0) {
        *outp = true;
        return 0;
    } else if (strcmp(valstr, "false") == 0 || strcmp(valstr, "0") == 0) {
        *outp = false;
        return 0;
    }
    return -1;
}

int cfg_get_bool(char const *key, bool *outp) {
    char const *nodestr = cfg_get_node(key);
    if (nodestr) {
        int success = cfg_parse_bool(nodestr, outp);
        if (success != 0)
            LOG_ERROR("error parsing config node \"%s\"\n", key);
        return success;
    }
    return -1;
}

static int cfg_parse_rgb(char const *valstr, int *red, int *green, int *blue) {
    if (strlen(valstr) != 7)
        return -1;

    if (valstr[0] != '#')
        return -1;

    int idx;
    unsigned digits[6];

    for (idx = 0; idx < 6; idx++) {
        char ch = valstr[idx + 1];
        if (ch >= '0' && ch <= '9') {
            digits[idx] = ch - '0';
        } else if (ch >= 'a' && ch <= 'f') {
            digits[idx] = ch - 'a' + 10;
        } else if (ch >= 'A' && ch <= 'F') {
            digits[idx] = ch - 'A' + 10;
        } else {
            LOG_ERROR("Bad color syntax \"%s\"\n", valstr);
            return -1;
        }
    }

    *red = digits[0] * 16 + digits[1];
    *green = digits[2] * 16 + digits[3];
    *blue = digits[4] * 16 + digits[5];

    return 0;
}

int cfg_get_rgb(char const *key, int *red, int *green, int *blue) {
    char const *nodestr = cfg_get_node(key);
    if (nodestr) {
        int success = cfg_parse_rgb(nodestr, red, green, blue);
        if (success != 0)
            LOG_ERROR("error parsing config node \"%s\"\n", key);
        return success;
    }
    return -1;
}
