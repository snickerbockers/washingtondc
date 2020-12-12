/*******************************************************************************
 *
 *
 *    WashingtonDC Dreamcast Emulator
 *    Copyright (C) 2017-2020 snickerbockers
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

#ifndef PVR2_REG_H_
#define PVR2_REG_H_

#include <stddef.h>
#include <stdint.h>

#include "washdc/MemoryMap.h"
#include "washdc/types.h"

struct pvr2;

// vlck divider bit for the FB_R_CTRL register
#define PVR2_VCLK_DIV_SHIFT 23
#define PVR2_VCLK_DIV_MASK (1 << PVR2_VCLK_DIV_SHIFT)

// bit in the FB_R_CTRL register that causes each scanline to be sent twice
#define PVR2_LINE_DOUBLE_SHIFT 1
#define PVR2_LINE_DOUBLE_MASK (1 << PVR2_LINE_DOUBLE_SHIFT)

#define PVR2_PALETTE_RAM_FIRST 0x5f9000
#define PVR2_PALETTE_RAM_LAST  0x5f9fff
#define PVR2_PALETTE_RAM_LEN \
    (PVR2_PALETTE_RAM_LAST - PVR2_PALETTE_RAM_FIRST + 1)

uint8_t *pvr2_get_palette_ram(struct pvr2 *pvr2);

void pvr2_reg_init(struct pvr2 *pvr2);
void pvr2_reg_cleanup(struct pvr2 *pvr2);

double pvr2_reg_read_double(addr32_t addr, void *ctxt);
void pvr2_reg_write_double(addr32_t addr, double val, void *ctxt);
float pvr2_reg_read_float(addr32_t addr, void *ctxt);
void pvr2_reg_write_float(addr32_t addr, float val, void *ctxt);
uint32_t pvr2_reg_read_32(addr32_t addr, void *ctxt);
void pvr2_reg_write_32(addr32_t addr, uint32_t val, void *ctxt);
uint16_t pvr2_reg_read_16(addr32_t addr, void *ctxt);
void pvr2_reg_write_16(addr32_t addr, uint16_t val, void *ctxt);
uint8_t pvr2_reg_read_8(addr32_t addr, void *ctxt);
void pvr2_reg_write_8(addr32_t addr, uint8_t val, void *ctxt);

extern struct memory_interface pvr2_reg_intf;

enum palette_tp {
    PALETTE_TP_ARGB_1555,
    PALETTE_TP_RGB_565,
    PALETTE_TP_ARGB_4444,
    PALETTE_TP_ARGB_8888,

    PALETTE_TP_COUNT
};

uint32_t get_fb_r_ctrl(struct pvr2 *pvr2);
uint32_t get_fb_w_ctrl(struct pvr2 *pvr2);
uint32_t get_fb_w_linestride(struct pvr2 *pvr2);
uint32_t get_fb_r_sof1(struct pvr2 *pvr2);
uint32_t get_fb_r_sof2(struct pvr2 *pvr2);
uint32_t get_fb_r_size(struct pvr2 *pvr2);
uint32_t get_fb_w_sof1(struct pvr2 *pvr2);
uint32_t get_fb_w_sof2(struct pvr2 *pvr2);
uint32_t get_isp_backgnd_t(struct pvr2 *pvr2);
uint32_t get_isp_backgnd_d(struct pvr2 *pvr2);
uint32_t get_glob_tile_clip(struct pvr2 *pvr2);
uint32_t get_glob_tile_clip_x(struct pvr2 *pvr2);
uint32_t get_glob_tile_clip_y(struct pvr2 *pvr2);
uint32_t get_fb_x_clip(struct pvr2 *pvr2);
uint32_t get_fb_y_clip(struct pvr2 *pvr2);
unsigned get_fb_x_clip_min(struct pvr2 *pvr2);
unsigned get_fb_y_clip_min(struct pvr2 *pvr2);
unsigned get_fb_x_clip_max(struct pvr2 *pvr2);
unsigned get_fb_y_clip_max(struct pvr2 *pvr2);
enum palette_tp get_palette_tp(struct pvr2 *pvr2);
uint32_t get_ta_yuv_tex_base(struct pvr2 *pvr2);
uint32_t get_ta_yuv_tex_ctrl(struct pvr2 *pvr2);

#define PVR2_SB_PDSTAP  0
#define PVR2_SB_PDSTAR  1
#define PVR2_SB_PDLEN   2
#define PVR2_SB_PDDIR   3
#define PVR2_SB_PDTSEL  4
#define PVR2_SB_PDEN    5
#define PVR2_SB_PDST    6
#define PVR2_SB_PDAPRO 32

#define PVR2_REG_IDX(addr) (((addr) - ADDR_PVR2_FIRST) / sizeof(uint32_t))

#define PVR2_ID PVR2_REG_IDX(0x5f8000)
#define PVR2_REV PVR2_REG_IDX(0x5f8004)
#define PVR2_SOFTRESET PVR2_REG_IDX(0x5f8008)
#define PVR2_STARTRENDER PVR2_REG_IDX(0x5f8014)
#define PVR2_PARAM_BASE PVR2_REG_IDX(0x5f8020)
#define PVR2_REGION_BASE PVR2_REG_IDX(0x5f802c)
#define PVR2_SPAN_SORT_CFG PVR2_REG_IDX(0x5f8030)
#define PVR2_VO_BORDER_COL PVR2_REG_IDX(0x5f8040)
#define PVR2_FB_R_CTRL PVR2_REG_IDX(0x5f8044)
#define PVR2_FB_W_CTRL PVR2_REG_IDX(0x5f8048)
#define PVR2_FB_W_LINESTRIDE PVR2_REG_IDX(0x5f804c)
#define PVR2_FB_R_SOF1 PVR2_REG_IDX(0x5f8050)
#define PVR2_FB_R_SOF2 PVR2_REG_IDX(0x5f8054)
#define PVR2_FB_R_SIZE PVR2_REG_IDX(0x5f805c)
#define PVR2_FB_W_SOF1 PVR2_REG_IDX(0x5f8060)
#define PVR2_FB_W_SOF2 PVR2_REG_IDX(0x5f8064)
#define PVR2_FB_X_CLIP PVR2_REG_IDX(0x5f8068)
#define PVR2_FB_Y_CLIP PVR2_REG_IDX(0x5f806c)
#define PVR2_FPU_SHAD_SCALE PVR2_REG_IDX(0x5f8074)
#define PVR2_FPU_CULL_VAL PVR2_REG_IDX(0x5f8078)
#define PVR2_FPU_PARAM_CFG PVR2_REG_IDX(0x5f807c)
#define PVR2_HALF_OFFSET PVR2_REG_IDX(0x5f8080)
#define PVR2_FPU_PERP_VAL PVR2_REG_IDX(0x5f8084)
#define PVR2_ISP_BACKGND_D PVR2_REG_IDX(0x5f8088)
#define PVR2_ISP_BACKGND_T PVR2_REG_IDX(0x5f808c)

// unknown/undocumented register mislabeled as ISP_FEED_CFG in some docs
#define PVR2_UNKNOWN_8090 PVR2_REG_IDX(0x5f8090)

#define PVR2_ISP_FEED_CFG PVR2_REG_IDX(0x5f8098)
#define PVR2_FOG_CLAMP_MAX PVR2_REG_IDX(0x5f80bc)
#define PVR2_FOG_CLAMP_MIN PVR2_REG_IDX(0x5f80c0)
#define PVR2_TEXT_CONTROL PVR2_REG_IDX(0x5f80e4)
#define PVR2_SCALER_CTL PVR2_REG_IDX(0x5f80f4)
#define PVR2_FB_BURSTXTRL PVR2_REG_IDX(0x5f8110)
#define PVR2_Y_COEFF PVR2_REG_IDX(0x5f8118)
#define PVR2_SDRAM_REFRESH PVR2_REG_IDX(0x5f80a0)
#define PVR2_SDRAM_CFG PVR2_REG_IDX(0x5f80a8)
#define PVR2_FOG_COL_RAM PVR2_REG_IDX(0x5f80b0)
#define PVR2_FOG_COL_VERT PVR2_REG_IDX(0x5f80b4)
#define PVR2_FOG_DENSITY PVR2_REG_IDX(0x5f80b8)
#define PVR2_SPG_HBLANK_INT PVR2_REG_IDX(0x5f80c8)
#define PVR2_SPG_VBLANK_INT PVR2_REG_IDX(0x5f80cc)
#define PVR2_SPG_CONTROL PVR2_REG_IDX(0x5f80d0)
#define PVR2_SPG_HBLANK PVR2_REG_IDX(0x5f80d4)
#define PVR2_SPG_LOAD PVR2_REG_IDX(0x5f80d8)
#define PVR2_SPG_VBLANK PVR2_REG_IDX(0x5f80dc)
#define PVR2_SPG_WIDTH PVR2_REG_IDX(0x5f80e0)
#define PVR2_VO_CONTROL PVR2_REG_IDX(0x5f80e8)
#define PVR2_VO_STARTX PVR2_REG_IDX(0x5f80ec)
#define PVR2_VO_STARTY PVR2_REG_IDX(0x5f80f0)
#define PVR2_PALETTE_TP PVR2_REG_IDX(0x5f8108)
#define PVR2_SPG_STATUS PVR2_REG_IDX(0x5f810c)
#define PVR2_PT_ALPHA_REF PVR2_REG_IDX(0x5f811c)
#define PVR2_TA_OL_BASE PVR2_REG_IDX(0x5f8124)
#define PVR2_TA_VERTBUF_START PVR2_REG_IDX(0x5f8128)
#define PVR2_TA_ISP_LIMIT PVR2_REG_IDX(0x5f8130)
#define PVR2_TA_NEXT_OPB PVR2_REG_IDX(0x5f8134)
#define PVR2_TA_VERTBUF_POS PVR2_REG_IDX(0x5f8138)
#define PVR2_TA_OL_LIMIT PVR2_REG_IDX(0x5f812c)
#define PVR2_TA_GLOB_TILE_CLIP PVR2_REG_IDX(0x5f813c)
#define PVR2_TA_ALLOC_CTRL PVR2_REG_IDX(0x5f8140)
#define PVR2_TA_RESET PVR2_REG_IDX(0x5f8144)
#define PVR2_TA_YUV_TEX_BASE PVR2_REG_IDX(0x5f8148)
#define PVR2_TA_YUV_TEX_CTRL PVR2_REG_IDX(0x5f814c)
#define PVR2_TA_LIST_CONT     PVR2_REG_IDX(0x5f8160)
#define PVR2_TA_NEXT_OPB_INIT PVR2_REG_IDX(0x5f8164)

#endif
