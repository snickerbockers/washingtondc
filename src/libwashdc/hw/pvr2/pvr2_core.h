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

#ifndef PVR2_CORE_H_
#define PVR2_CORE_H_

#include <stdbool.h>
#include <stdint.h>

#include "pvr2_def.h"
#include "gfx/gfx.h" // for enum tex_filter
#include "dc_sched.h"

/*
 * On a real Dreamcast, the CPU creates in GPU VRAM a per-tile array which
 * contains pointers to lists of polygon data for each of the five polygon
 * groups.  This tile array is pointed to by the PVR2_REGION_BASE register,
 * and the pointers to the five polygon groups are offset by the
 * PVR2_PARAM_BASE registero.  When the STARTRENDER command is issued, the GPU
 * reads in each tile from the tile array (pointed to by PVR2_REGION_BASE), and
 * then for each tile it renders the polygon data pointed to by the 5 polygon
 * group pointers (after adding the PVR2_REGION_BASE register to those
 * pointers).
 *
 * The TA creates the polygon data but it has no knowledge of the tile array.
 * Instead it has its own control registers which point it to where in GPU
 * memory polygon data should be written.  These registers are configured by
 * the CPU in a way that ought to be consistent with what's in the tile array.
 *
 * The tile array allows tiles to be laid out in-memory in any order.  I'm not
 * 100% sure on this but I think that the TA assumes they're laid out in a
 * sensible row-major order (thus restricting the layout to row-major unless
 * the CPU wants to generate its own display lists without the TA's help).
 *
 * The PVR2_TA_VERTBUF_POS register points to where the TA should start writing
 * polygon data.  So it corresponds to the PVR2_REGION_BASE register, so it's
 * *hopefully* safe to use this as a key for tracking display lists.  So our
 * HLE strategy here is keep track of the last PVR2_MAX_FRAMES_IN_FLIGHT values
 * of PVR2_TA_VERTBUF_POS that were used, and replay those TAFIFO inputs
 * whenever we see a STARTRENDER command with a matching PVR2_REGION_BASE.  This
 * will be faster and easier to implement than a real LLE of the display list
 * format, albeit less accurate.
 *
 * Potential failure cases include:
 * * there are more than PVR2_MAX_FRAMES_IN_FLIGHT frames in flight - it is
 *   extremely unlikely that anybody would ever use more than two, but if they
 *   really want to fuck with me by doing this then they can.
 * * PVR2_TA_VERTBUF_POS doesn't match PVR2_REGION_BASE, but the pointers in
 *   the tile array still line up with where the TA put the data - this is very
 *   possible but thankfully it never seems to be the case in any of the logs I
 *   have looked at.
 * * software generated its own display lists without using the TA - very
 *   possible but I seriously doubt anything actually does this.  If this case
 *   is ever encountered then true low-level display list emulation is the only
 *   possible solution.
 * * The game queued up more data then I have room to buffer - this is very
 *   avoidable even in the worst-case scenario since a modern PC can easily have
 *   thousands of times more memory than the Dreamcast's VRAM.
 *
 * I'm also not entirely sure how list continuation will need to be handled as
 * a special case, but hopefully the answer is "it won't".
 *
 */
enum pvr2_display_list_command_tp {
    PVR2_DISPLAY_LIST_COMMAND_TP_HEADER,
    PVR2_DISPLAY_LIST_COMMAND_TP_END_OF_GROUP,
    PVR2_DISPLAY_LIST_COMMAND_TP_VERTEX,
    PVR2_DISPLAY_LIST_COMMAND_TP_QUAD,
    PVR2_DISPLAY_LIST_COMMAND_TP_USER_CLIP
};

struct pvr2_display_list_command_header {
    // current geometry type (either triangle strips or quads)
    enum pvr2_hdr_tp geo_tp;

    bool stride_sel;
    bool tex_enable;
    bool tex_twiddle;
    bool tex_vq_compression;
    bool tex_mipmap;
    unsigned tex_width_shift, tex_height_shift;
    enum tex_wrap_mode tex_wrap_mode[2];
    enum tex_inst tex_inst;
    enum tex_filter tex_filter;
    enum TexCtrlPixFmt pix_fmt;
    uint32_t tex_addr;

    enum pvr2_user_clip_mode user_clip_mode;

    /*
     * this is the upper 2-bits (for 8BPP) or 6 bits (for 4BPP) of every
     * palette address referenced by this texture.  It needs to be shifted left
     * by 2 or 6 bits and ORed with pixel values to get palette addresses.
     *
     * this field only holds meaning if tex_fmt is TEX_CTRL_PIX_FMT_4_BPP_PAL
     * or TEX_CTRL_PIX_FMT_8_BPP_PAL; otherwise it is meaningless.
     */
    unsigned tex_palette_start;

    enum Pvr2BlendFactor src_blend_factor, dst_blend_factor;

    bool enable_depth_writes;
    enum Pvr2DepthFunc depth_func;
};

struct pvr2_display_list_end_of_group {
    enum pvr2_poly_type poly_type;
};

struct pvr2_display_list_vertex {
    float pos[3];
    float tex_coord[2];
    float base_color[4];
    float offs_color[4];
    bool end_of_strip;
};

struct pvr2_display_list_quad {
    /*
     * four vertices consisting of 3-component poistions
     *and 2-component texture coordinates
     */
    float vert_pos[4][3];
    unsigned tex_coords_packed[3];
    bool degenerate;

    float base_color[4];
    float offs_color[4];
};

struct pvr2_display_list_user_clip {
    // these coordinates are in terms of tiles, not pixels
    unsigned x_min, y_min, x_max, y_max;
};

struct pvr2_display_list_command {
    enum pvr2_display_list_command_tp tp;
    union {
        struct pvr2_display_list_command_header hdr;
        struct pvr2_display_list_end_of_group end_of_group;
        struct pvr2_display_list_vertex vtx;
        struct pvr2_display_list_quad quad;
        struct pvr2_display_list_user_clip user_clip;
    };
};

struct pvr2_display_list_group {
    // if false, this polygon group is not used by the display list
    bool valid;

    unsigned n_cmds;

#define PVR2_DISPLAY_LIST_MAX_LEN (128*1024) // TODO: made up bullshit limit
    struct pvr2_display_list_command cmds[PVR2_DISPLAY_LIST_MAX_LEN];
};

typedef uint32_t pvr2_display_list_key;
struct pvr2_display_list {
    pvr2_display_list_key key;
    unsigned age_counter; // used for determining the least-recently used list
    bool valid;
    struct pvr2_display_list_group poly_groups[PVR2_POLY_TYPE_COUNT];
};

#define PVR2_MAX_FRAMES_IN_FLIGHT 4

struct pvr2_core_vert {
    float pos[3];
    float base_color[4];
    float offs_color[4];
    float tex_coord[2];
};

struct pvr2_core {
    // textures - this will change throught display list execution
    bool stride_sel;
    unsigned tex_width_shift, tex_height_shift;
    unsigned cur_poly_group;

    /*
     * minimum and maximum vertex depth per frame, used for mapping to OpenGL
     * clip coordinates
     */
    float clip_min, clip_max;

    /*
     * used to store the previous two verts when we're
     * rendering a triangle strip
     */
    struct pvr2_core_vert strip_vert_1;
    struct pvr2_core_vert strip_vert_2;
    unsigned strip_len; // number of verts in the current triangle strip

    // the 4-component color that gets sent to glClearColor
    float pvr2_bgcolor[4];

    // vertex buf containing vertices which have not yet been put into the gfx_il_inst_buf
    float *pvr2_core_vert_buf;
    unsigned pvr2_core_vert_buf_count;
    unsigned pvr2_core_vert_buf_start;

    // here's where we buffer gfx_il instructions
    struct gfx_il_inst *gfx_il_inst_buf;
    unsigned gfx_il_inst_buf_count;

    // reference alpha value for punch-through polygons
    unsigned pt_alpha_ref;

    unsigned next_frame_stamp;

    /*
     * DISPLAY LIST TRACKING
     */
    struct pvr2_display_list disp_lists[PVR2_MAX_FRAMES_IN_FLIGHT];
    unsigned disp_list_counter; // used to find least-recently used display list

    struct SchedEvent pvr2_render_complete_int_event;
    bool pvr2_render_complete_int_event_scheduled;
};

struct pvr2;

void pvr2_core_init(struct pvr2 *pvr2);
void pvr2_core_cleanup(struct pvr2 *pvr2);

struct pvr2_display_list_command *
pvr2_list_alloc_new_cmd(struct pvr2_display_list *listp,
                        enum pvr2_poly_type poly_tp);

unsigned pvr2_list_age(struct pvr2 const *pvr2,
                       struct pvr2_display_list const *listp);

/*
 * increment pvr2->ta.disp_list_counter.  If there's an integer overflow, then
 * the counter will be rolled back as far as possible and all display lists
 * will be adjusted accordingly.
 */
void pvr2_inc_age_counter(struct pvr2 *pvr2);

void
display_list_exec(struct pvr2 *pvr2, struct pvr2_display_list const *listp);

void pvr2_display_list_init(struct pvr2_display_list *list);

unsigned get_cur_frame_stamp(struct pvr2 *pvr2);

#endif