/*******************************************************************************
 *
 *
 *    WashingtonDC Dreamcast Emulator
 *    Copyright (C) 2017 snickerbockers
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

#ifndef SH4_MEM_H_
#define SH4_MEM_H_

#include <assert.h>

#include "sh4_excp.h"
#include "mem_code.h"
#include "error.h"

struct Sh4;

enum VirtMemArea {
    SH4_AREA_P0 = 0,
    SH4_AREA_P1,
    SH4_AREA_P2,
    SH4_AREA_P3,
    SH4_AREA_P4
};

// Physical memory aread boundaries
#define SH4_AREA_P0_FIRST  0x00000000
#define SH4_AREA_P0_LAST   0x7fffffff
#define SH4_AREA_P1_FIRST  0x80000000
#define SH4_AREA_P1_LAST   0x9fffffff
#define SH4_AREA_P2_FIRST  0xa0000000
#define SH4_AREA_P2_LAST   0xbfffffff
#define SH4_AREA_P3_FIRST  0xc0000000
#define SH4_AREA_P3_LAST   0xdfffffff
#define SH4_AREA_P4_FIRST  0xe0000000
#define SH4_AREA_P4_LAST   0xffffffff

/*
 * SH4_P4_REGSTART is the addr of the first memory-mapped
 *     register in area 7
 * SH4_P4_REGEND is the first addr *after* the last memory-mapped
 *     register in the p4 area.
 * SH4_AREA7_REGSTART is the addr of the first memory-mapped
 *     register in area 7
 * SH4_AREA7_REGEND is the first addr *after* the last memory-mapped
 *     register in area 7
 */
#define SH4_P4_REGSTART    0xff000000
#define SH4_P4_REGEND      0xfff00008
#define SH4_AREA7_REGSTART 0x1f000000
#define SH4_AREA7_REGEND   0x1ff00008
static_assert((SH4_P4_REGEND - SH4_P4_REGSTART) ==
              (SH4_AREA7_REGEND - SH4_AREA7_REGSTART),
              "AREA7 is not the same size as the P4 area");

/* constants needed for opcache as ram */
#define SH4_LONGS_PER_OP_CACHE_LINE 8
#define SH4_OP_CACHE_LINE_SIZE (SH4_LONGS_PER_OP_CACHE_LINE * 4)
#define SH4_OC_RAM_AREA_SIZE (8 * 1024)

/*
 * TODO:
 * These functions do not signal errors in any way at all
 * If they detect an error, they will call RAISE_ERROR, which panics
 * WashingtonDC
 */
void sh4_write_mem_8(Sh4 *sh4, uint8_t val, addr32_t addr);
void sh4_write_mem_16(Sh4 *sh4, uint16_t val, addr32_t addr);
void sh4_write_mem_32(Sh4 *sh4, uint32_t val, addr32_t addr);
uint8_t sh4_read_mem_8(Sh4 *sh4, addr32_t addr);
uint16_t sh4_read_mem_16(Sh4 *sh4, addr32_t addr);
uint32_t sh4_read_mem_32(Sh4 *sh4, addr32_t addr);

/*
 * same as sh4_write_mem/sh4_read_mem, except they don't automatically raise
 * pending errors and they don't check for watchpoints
 */
int sh4_do_write_mem(Sh4 *sh4, void const *dat, addr32_t addr, unsigned len);
int sh4_do_read_mem(Sh4 *sh4, void *dat, addr32_t addr, unsigned len);

/*
 * generally you'll call these functions through do_read_mem/do_write_mem
 * instead of calling these functions directly
 */
int sh4_do_read_p4(Sh4 *sh4, void *dat, addr32_t addr, unsigned len);
int sh4_do_write_p4(Sh4 *sh4, void const *dat, addr32_t addr, unsigned len);

static inline int
sh4_read_mem(Sh4 *sh4, void *data, addr32_t addr, unsigned len) {
#ifdef ENABLE_DEBUGGER
    if (dc_debugger_enabled() && debug_is_r_watch(addr, len)) {
        sh4->aborted_operation = true;
        return MEM_ACCESS_EXC;
    }
#endif
    int ret;

    if ((ret = sh4_do_read_mem(sh4, data, addr, len)) == MEM_ACCESS_FAILURE)
        RAISE_ERROR(get_error_pending());
    return ret;
}

static inline int
sh4_write_mem(Sh4 *sh4, void const *data, addr32_t addr, unsigned len) {
#ifdef ENABLE_DEBUGGER
    if (dc_debugger_enabled() && debug_is_w_watch(addr, len)) {
        sh4->aborted_operation = true;
        return MEM_ACCESS_EXC;
    }
#endif

    int ret;

    if ((ret = sh4_do_write_mem(sh4, data, addr, len)) == MEM_ACCESS_FAILURE)
        RAISE_ERROR(get_error_pending());
    return ret;
}

#endif
