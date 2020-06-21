/*******************************************************************************
 *
 * Copyright 2017-2020 snickerbockers
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

#ifndef SH4_MEM_H_
#define SH4_MEM_H_

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "sh4_excp.h"
#include "mem_code.h"
#include "washdc/error.h"
#include "washdc/MemoryMap.h"

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

#define SH4_P4_ITLB_ADDR_ARRAY_FIRST   0xf2000000
#define SH4_P4_ITLB_ADDR_ARRAY_LAST    0xf2ffffff
#define SH4_P4_ITLB_DATA_ARRAY_1_FIRST 0xf3000000
#define SH4_P4_ITLB_DATA_ARRAY_1_LAST  0xf37fffff
#define SH4_P4_ITLB_DATA_ARRAY_2_FIRST 0xf3800000
#define SH4_P4_ITLB_DATA_ARRAY_2_LAST  0xf3ffffff
#define SH4_P4_UTLB_ADDR_ARRAY_FIRST   0xf6000000
#define SH4_P4_UTLB_ADDR_ARRAY_LAST    0xf6ffffff
#define SH4_P4_UTLB_DATA_ARRAY_1_FIRST 0xf7000000
#define SH4_P4_UTLB_DATA_ARRAY_1_LAST  0xf77fffff
#define SH4_P4_UTLB_DATA_ARRAY_2_FIRST 0xf7800000
#define SH4_P4_UTLB_DATA_ARRAY_2_LAST  0xf77fffff

/* constants needed for opcache as ram */
#define SH4_LONGS_PER_OP_CACHE_LINE 8
#define SH4_OP_CACHE_LINE_SIZE (SH4_LONGS_PER_OP_CACHE_LINE * 4)
#define SH4_OC_RAM_AREA_SIZE (8 * 1024)

void sh4_mem_init(Sh4 *sh4);
void sh4_mem_cleanup(Sh4 *sh4);

#define SH4_UTLB_LEN 64
#define SH4_ITLB_LEN 4

enum sh4_tlb_page_sz {
    SH4_TLB_PAGE_1KB = 0,
    SH4_TLB_PAGE_4KB = 1,
    SH4_TLB_PAGE_64KB = 2,
    SH4_TLB_PAGE_1MB = 3
};

struct sh4_utlb_ent {
    unsigned asid;
    unsigned vpn /* : 22 */;
    unsigned ppn /* : 19 */;

    /*
     * 2 bits
     * bit 0 is set if writable, cleared for read-only.
     * bit 1 is set if accessible in user-mode, cleared for privileged-only.
     */
    unsigned protection;

    /*
     * space attribute (three bits)
     *
     * Has somethhing to do with PCMIA, we don't actually care about this.
     */
    unsigned sa;

    enum sh4_tlb_page_sz sz;

    bool valid;
    bool shared;
    bool cacheable;
    bool dirty;
    bool wt; //write-through
    bool tc; // i sincerely hope i never need to understand what this is.
};

struct sh4_itlb_ent {
    unsigned asid;
    unsigned vpn /* : 22 */;
    unsigned ppn /* : 19 */;

    /*
     * only 1 bit, unlike for utlb
     * bit 0 is set if accessible in user-mode, cleared for privileged-only.
     *
     * I guess it doesn't make sense to have a writable bit in the itlb.
     */
    unsigned protection;

    /*
     * space attribute (three bits)
     *
     * Has somethhing to do with PCMIA, we don't actually care about this.
     */
    unsigned sa;

    enum sh4_tlb_page_sz sz;

    bool valid;
    bool shared;
    bool cacheable;
    bool tc; // i sincerely hope i never need to understand what this is.
};

struct sh4_mem {
    struct memory_map *map;

    struct sh4_utlb_ent utlb[SH4_UTLB_LEN];
    struct sh4_itlb_ent itlb[SH4_ITLB_LEN];
};

void sh4_set_mem_map(struct Sh4 *sh4, struct memory_map *map);

extern struct memory_interface sh4_p4_intf;

#ifdef ENABLE_MMU

enum sh4_utlb_translate_result {
    SH4_UTLB_SUCCESS = 0,
    SH4_UTLB_MISS = -1,
    SH4_UTLB_PROT_VIOL = -2,
    SH4_UTLB_INITIAL_WRITE = -3
};

enum sh4_itlb_translate_result {
    SH4_ITLB_SUCCESS = 0,
    SH4_ITLB_MISS = -1,
    SH4_ITLB_PROT_VIOL = -2
};

/*
 * translates *addr, if MMU is enabled and addr is in one of the areas where
 * address translation is possible.
 *
 * If the translation was successful, this function returns 0.
 * Else, it returns non-zero.
 *
 * These functions can modify the state of the sh4, so the lower-level
 * sh4_utlb_find_ent_associative and sh4_itlb_find_ent_associative should be
 * used instead if all you want to do is passively query the TLB without
 * modifying anything.
 */
enum sh4_utlb_translate_result
sh4_utlb_translate_address(struct Sh4 *sh4, uint32_t *addrp, bool write);
enum sh4_itlb_translate_result
sh4_itlb_translate_address(struct Sh4 *sh4, uint32_t *addr_p);

/*
 * These functions search the itlb or the utlb for the given entry and return a
 * pointer to it, or NULL if the entry was not found.
 */
struct sh4_utlb_ent *
sh4_utlb_find_ent_associative(struct Sh4 *sh4, uint32_t vpn);
struct sh4_itlb_ent *
sh4_itlb_find_ent_associative(struct Sh4 *sh4, uint32_t vpn);

/*
 * translate the given address based on the given tlb entry.
 * No error checking is performed; it is assumed that the caller
 * has already verified that the vpns match up.
 */
uint32_t sh4_itlb_ent_translate_addr(struct sh4_itlb_ent const *ent, uint32_t vpn);
uint32_t sh4_utlb_ent_translate_addr(struct sh4_utlb_ent const *ent, uint32_t vpn);

#endif

// invalidate the entirety of both the ITLB and the UTLB
void sh4_mmu_invalidate_tlb(struct Sh4 *sh4);

void sh4_mmu_do_ldtlb(struct Sh4 *sh4);

// excessive, but handy for debugging MMU stuff
// #define SUPER_VERBOSE_MEM_TRACE

#ifdef SUPER_VERBOSE_MEM_TRACE

#define SH4_MEM_TRACE(msg, ...)                                         \
    do {                                                                \
        LOG_DBG("SH4-MEM : ");                                          \
        LOG_DBG(msg, ##__VA_ARGS__);                                    \
    } while (0)

#else

#define SH4_MEM_TRACE(msg, ...) do { } while (0)

#endif

#endif
