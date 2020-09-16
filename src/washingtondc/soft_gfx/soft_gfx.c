/*******************************************************************************
 *
 * Copyright 2020 snickerbockers
 * snickerbockers@washemu.org
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#include <stdint.h>
#include <string.h>

#define GL3_PROTOTYPES 1
#include <GL/glew.h>
#include <GL/gl.h>

#include "washdc/gfx/gfx_all.h"
#include "washdc/gfx/def.h"
#include "../gfx_obj.h"
#include "../shader.h"

#include "soft_gfx.h"

static struct soft_gfx_callbacks const *switch_table;

static void soft_gfx_init(void);
static void soft_gfx_cleanup(void);
static void soft_gfx_exec_gfx_il(struct gfx_il_inst *cmd, unsigned n_cmd);

static void init_poly();

#define FB_WIDTH 640
#define FB_HEIGHT 480

// vertex position (x, y, z)
#define OUTPUT_SLOT_VERT_POS 0

// vertex texture coordinates (s, t)
#define OUTPUT_SLOT_VERT_ST 1

#define OUTPUT_SLOT_TRANS_MAT 2

#define OUTPUT_SLOT_TEX_MAT 3

// number of floats per vertex.
// that's 3 floats for the position and 2 for the texture coords
#define FB_VERT_LEN 5
#define FB_VERT_COUNT 4
static GLfloat fb_quad_verts[FB_VERT_LEN * FB_VERT_COUNT] = {
    // position            // texture coordinates
    -1.0f,  1.0f, 0.0f,    0.0f, 1.0f,
    -1.0f, -1.0f, 0.0f,    0.0f, 0.0f,
     1.0f, -1.0f, 0.0f,    1.0f, 0.0f,
     1.0f,  1.0f, 0.0f,    1.0f, 1.0f
};

#define FB_QUAD_IDX_COUNT 4
static GLuint fb_quad_idx[FB_QUAD_IDX_COUNT] = {
    1, 0, 2, 3
};

static GLfloat trans_mat[16] = {
    1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f
};

static GLfloat tex_mat[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f
};

/*
 * container for the poly's vertex array and its associated buffer objects.
 * this is created by fb_init_poly and never modified.
 *
 * The tex_obj, on the other hand, is modified frequently, as it is OpenGL's
 * view of our framebuffer.
 */
static struct fb_poly {
    GLuint vbo; // vertex buffer object
    GLuint vao; // vertex array object
    GLuint ebo; // element buffer object
} fb_poly;

static uint32_t fb[FB_WIDTH * FB_HEIGHT];
static GLuint fb_tex;
static struct shader fb_shader;

struct rend_if const soft_gfx_if = {
    .init = soft_gfx_init,
    .cleanup = soft_gfx_cleanup,
    .exec_gfx_il = soft_gfx_exec_gfx_il
};

static void soft_gfx_init(void) {
    glewExperimental = GL_TRUE;
    glewInit();

    static char const * const final_vert_glsl =
        "#extension GL_ARB_explicit_uniform_location : enable\n"

        "layout (location = 0) in vec3 vert_pos;\n"
        "layout (location = 1) in vec2 tex_coord;\n"
        "layout (location = 2) uniform mat4 trans_mat;\n"
        "layout (location = 3) uniform mat3 tex_mat;\n"

        "out vec2 st;\n"

        "void main() {\n"
        "    gl_Position = trans_mat * vec4(vert_pos.x, vert_pos.y, vert_pos.z, 1.0);\n"
        "    st = (tex_mat * vec3(tex_coord.x, tex_coord.y, 1.0)).xy;\n"
        "}\n";

    static char const * const final_frag_glsl =
        "in vec2 st;\n"
        "out vec4 color;\n"

        "uniform sampler2D fb_tex;\n"

        "void main() {\n"
        "    vec4 sample = texture(fb_tex, st);\n"
        "    color = sample;\n"
        "}\n";

    shader_load_vert(&fb_shader, final_vert_glsl);
    shader_load_frag(&fb_shader, final_frag_glsl);
    shader_link(&fb_shader);

    glGenTextures(1, &fb_tex);
    glBindTexture(GL_TEXTURE_2D, fb_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);

    memset(fb, 0, sizeof(fb));

    init_poly();
}

static void soft_gfx_cleanup(void) {
    glDeleteTextures(1, &fb_tex);
}

static void init_poly() {
    GLuint vbo, vao, ebo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 FB_VERT_LEN * FB_VERT_COUNT * sizeof(GLfloat),
                 fb_quad_verts, GL_STATIC_DRAW);
    glVertexAttribPointer(OUTPUT_SLOT_VERT_POS, 3, GL_FLOAT, GL_FALSE,
                          FB_VERT_LEN * sizeof(GLfloat),
                          (GLvoid*)0);
    glEnableVertexAttribArray(OUTPUT_SLOT_VERT_POS);
    glVertexAttribPointer(OUTPUT_SLOT_VERT_ST, 2, GL_FLOAT, GL_FALSE,
                          FB_VERT_LEN * sizeof(GLfloat),
                          (GLvoid*)(3 * sizeof(GLfloat)));
    glEnableVertexAttribArray(OUTPUT_SLOT_VERT_ST);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, FB_QUAD_IDX_COUNT * sizeof(GLuint),
                 fb_quad_idx, GL_STATIC_DRAW);

    glBindVertexArray(0);

    fb_poly.vbo = vbo;
    fb_poly.vao = vao;
    fb_poly.ebo = ebo;
}

void soft_gfx_set_callbacks(struct soft_gfx_callbacks const *callbacks) {
    switch_table = callbacks;
}

static void soft_gfx_obj_init(struct gfx_il_inst *cmd) {
    int obj_no = cmd->arg.init_obj.obj_no;
    size_t n_bytes = cmd->arg.init_obj.n_bytes;
    gfx_obj_init(obj_no, n_bytes);
}

static void soft_gfx_obj_write(struct gfx_il_inst *cmd) {
    int obj_no = cmd->arg.write_obj.obj_no;
    size_t n_bytes = cmd->arg.write_obj.n_bytes;
    void const *dat = cmd->arg.write_obj.dat;
    gfx_obj_write(obj_no, dat, n_bytes);
}

static void soft_gfx_obj_read(struct gfx_il_inst *cmd) {
    int obj_no = cmd->arg.read_obj.obj_no;
    size_t n_bytes = cmd->arg.read_obj.n_bytes;
    void *dat = cmd->arg.read_obj.dat;
    gfx_obj_read(obj_no, dat, n_bytes);
}

static void soft_gfx_obj_free(struct gfx_il_inst *cmd) {
    int obj_no = cmd->arg.free_obj.obj_no;
    gfx_obj_free(obj_no);
}

static void soft_gfx_post_fb(struct gfx_il_inst *cmd) {
    int obj_handle = cmd->arg.post_framebuffer.obj_handle;
    struct gfx_obj *obj = gfx_obj_get(obj_handle);
    bool do_flip = cmd->arg.post_framebuffer.vert_flip;

    if (obj->dat_len && obj->dat) {
        size_t n_bytes = obj->dat_len < sizeof(fb) ? obj->dat_len : sizeof(fb);
        if (do_flip) {
            unsigned src_width = cmd->arg.post_framebuffer.width;
            unsigned src_height = cmd->arg.post_framebuffer.height;

            unsigned copy_width = src_width < FB_WIDTH ? src_width : FB_WIDTH;
            unsigned copy_height = src_height < FB_HEIGHT ? src_height : FB_HEIGHT;

            unsigned row;
            for (row = 0; row < copy_height; row++) {
                uint32_t *dstp = fb + row * FB_WIDTH;
                char *srcp = ((char*)obj->dat) + 4 * (src_height - 1 - row) * src_width;
                memcpy(dstp, srcp, copy_width * 4);
            }
        } else {
            memcpy(fb, obj->dat, n_bytes);
        }
    }

    glViewport(0, 0, FB_WIDTH, FB_HEIGHT);
    glUseProgram(fb_shader.shader_prog_obj);

    glBindTexture(GL_TEXTURE_2D, fb_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, FB_WIDTH, FB_HEIGHT,
                 0, GL_RGBA, GL_UNSIGNED_BYTE, fb);

    glUniform1i(glGetUniformLocation(fb_shader.shader_prog_obj, "fb_tex"), 0);
    glUniformMatrix4fv(OUTPUT_SLOT_TRANS_MAT, 1, GL_TRUE, trans_mat);
    glUniformMatrix3fv(OUTPUT_SLOT_TEX_MAT, 1, GL_TRUE, tex_mat);

    glUseProgram(fb_shader.shader_prog_obj);
    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(fb_poly.vao);
    glDrawElements(GL_TRIANGLE_STRIP, FB_QUAD_IDX_COUNT, GL_UNSIGNED_INT, 0);

    switch_table->win_update();
}

static void soft_gfx_exec_gfx_il(struct gfx_il_inst *cmd, unsigned n_cmd) {
    while (n_cmd--) {
        switch (cmd->op) {
        case GFX_IL_BIND_TEX:
            printf("GFX_IL_BIND_TEX\n");
            break;
        case GFX_IL_UNBIND_TEX:
            printf("GFX_IL_UNBIND_TEX\n");
            break;
        case GFX_IL_BIND_RENDER_TARGET:
            printf("GFX_IL_BIND_RENDER_TARGET\n");
            break;
        case GFX_IL_UNBIND_RENDER_TARGET:
            printf("GFX_IL_UNBIND_RENDER_TARGET\n");
            break;
        case GFX_IL_BEGIN_REND:
            printf("GFX_IL_BEGIN_REND\n");
            break;
        case GFX_IL_END_REND:
            printf("GFX_IL_END_REND\n");
            break;
        case GFX_IL_CLEAR:
            printf("GFX_IL_CLEAR\n");
            break;
        case GFX_IL_SET_BLEND_ENABLE:
            printf("GFX_IL_SET_BLEND_ENABLE\n");
            break;
        case GFX_IL_SET_REND_PARAM:
            printf("GFX_IL_SET_REND_PARAM\n");
            break;
        case GFX_IL_SET_CLIP_RANGE:
            printf("GFX_IL_SET_CLIP_RANGE\n");
            break;
        case GFX_IL_DRAW_ARRAY:
            printf("GFX_IL_DRAW_ARRAY\n");
            break;
        case GFX_IL_INIT_OBJ:
            printf("GFX_IL_INIT_OBJ\n");
            soft_gfx_obj_init(cmd);
            break;
        case GFX_IL_WRITE_OBJ:
            printf("GFX_IL_WRITE_OBJ\n");
            soft_gfx_obj_write(cmd);
            break;
        case GFX_IL_READ_OBJ:
            printf("GFX_IL_READ_OBJ\n");
            soft_gfx_obj_read(cmd);
            break;
        case GFX_IL_FREE_OBJ:
            printf("GFX_IL_FREE_OBJ\n");
            soft_gfx_obj_free(cmd);
            break;
        case GFX_IL_POST_FRAMEBUFFER:
            printf("GFX_IL_POST_FRAMEBUFFER\n");
            soft_gfx_post_fb(cmd);
            break;
        case GFX_IL_GRAB_FRAMEBUFFER:
            printf("GFX_IL_GRAB_FRAMEBUFFER\n");
            break;
        case GFX_IL_BEGIN_DEPTH_SORT:
            printf("GFX_IL_BEGIN_DEPTH_SORT\n");
            break;
        case GFX_IL_END_DEPTH_SORT:
            printf("GFX_IL_END_DEPTH_SORT\n");
            break;
        default:
            fprintf(stderr, "ERROR: UNKNOWN GFX IL COMMAND %02X\n",
                    (unsigned)cmd->op);
        }
        cmd++;
    }
}
