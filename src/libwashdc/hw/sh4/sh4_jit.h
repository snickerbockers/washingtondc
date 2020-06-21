/*******************************************************************************
 *
 * Copyright 2018, 2019 snickerbockers
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

#ifndef SH4_JIT_H_
#define SH4_JIT_H_

#include <stdbool.h>

#include "washdc/cpu.h"
#include "washdc/types.h"
#include "sh4_inst.h"
#include "sh4_read_inst.h"
#include "jit/jit_il.h"
#include "jit/code_block.h"
#include "jit/optimize.h"
#include "jit/code_cache.h"

#ifdef JIT_PROFILE
#include "jit/jit_profile.h"
#endif

#ifdef ENABLE_JIT_X86_64
#include "jit/x86_64/code_block_x86_64.h"
#endif

struct InstOpcode;
struct il_code_block;
struct Sh4;

/*
 * call this at the beginning of every new block to reset the disassembler's
 * state to its default configuration.
 */
void sh4_jit_new_block(void);

struct sh4_jit_compile_ctx {
    unsigned last_inst_type;
    unsigned cycle_count;

    // only valid if have_reg_slot is true
    unsigned reg_slot;

    bool sz_bit : 1;
    bool pr_bit : 1;
    bool in_delay_slot : 1;
    bool dirty_fpscr : 1;
    bool have_reg_slot : 1;
};

#define SH4_JIT_HASH_MASK 0x1fffffff
#define SH4_JIT_HASH_PR_SHIFT 29
#define SH4_JIT_HASH_SZ_SHIFT 30
#define SH4_JIT_HASH_PR_MASK (1 << SH4_JIT_HASH_PR_SHIFT)
#define SH4_JIT_HASH_SZ_MASK (1 << SH4_JIT_HASH_SZ_SHIFT)

/*
 * code block hash values
 *
 * addr refers to the 32-bit PC address of the first instruction of the block
 * pr_bit refers to the PR bit in FPSCR
 * sz_bit refers to the SZ bit in FPSCR
 */
static inline jit_hash sh4_jit_hash(void *sh4, uint32_t addr,
                                    bool pr_bit, bool sz_bit) {
    return (addr & SH4_JIT_HASH_MASK) |
        (((jit_hash)pr_bit) << SH4_JIT_HASH_PR_SHIFT) |
        (((jit_hash)sz_bit) << SH4_JIT_HASH_SZ_SHIFT);
}

bool
sh4_jit_compile_inst(struct Sh4 *sh4, struct sh4_jit_compile_ctx *ctx,
                     struct il_code_block *block, cpu_inst_param inst,
                     unsigned pc);

static inline void
sh4_jit_il_code_block_compile(struct Sh4 *sh4, struct sh4_jit_compile_ctx *ctx,
                              struct jit_code_block *jit_blk,
                              struct il_code_block *block, addr32_t addr) {
    bool do_continue;

    sh4_jit_new_block();

    do {
        cpu_inst_param inst =
            memory_map_read_16(sh4->mem.map, addr & BIT_RANGE(0, 28));

#ifdef JIT_PROFILE
        uint16_t inst16 = inst;
        jit_profile_push_inst(&sh4->jit_profile, jit_blk->profile, &inst16);
#endif

        do_continue = sh4_jit_compile_inst(sh4, ctx, block, inst, addr);
        addr += 2;
    } while (do_continue);
}

#ifdef ENABLE_JIT_X86_64

#ifdef JIT_PROFILE
static void sh4_jit_profile_notify(void *cpu, struct jit_profile_per_block *blk_profile) {
    struct Sh4 *sh4 = (struct Sh4*)cpu;
    jit_profile_notify(&sh4->jit_profile, blk_profile);
}
#endif

static inline void
sh4_jit_compile_native(void *cpu, struct native_dispatch_meta const *meta,
                       struct jit_code_block *jit_blk, uint32_t pc) {
    struct Sh4 const *sh4 = (struct Sh4*)cpu;
    struct il_code_block il_blk;
    struct code_block_x86_64 *blk = &jit_blk->x86_64;
    struct sh4_jit_compile_ctx ctx = {
        .last_inst_type = SH4_GROUP_NONE,
        .cycle_count = 0,
        .sz_bit = sh4_fpscr_sz(sh4),
        .pr_bit = sh4_fpscr_pr(sh4),
        .in_delay_slot = false,
        .dirty_fpscr = false,
        .have_reg_slot = false
    };

    il_code_block_init(&il_blk);

#ifdef JIT_PROFILE
    il_blk.profile = jit_blk->profile;
#endif

    sh4_jit_il_code_block_compile(cpu, &ctx, jit_blk, &il_blk, pc);

    jit_optimize(&il_blk);

#ifdef JIT_PROFILE
    unsigned inst_no;
    for (inst_no = 0; inst_no < il_blk.inst_count; inst_no++) {
        jit_profile_push_il_inst(&sh4->jit_profile, jit_blk->profile,
                                 il_blk.inst_list + inst_no);
    }
#endif
    code_block_x86_64_compile(cpu, blk, &il_blk, meta,
                              ctx.cycle_count * SH4_CLOCK_SCALE);

#ifdef JIT_PROFILE
    ptrdiff_t wasted_bytes = 0;
    if (blk->native != blk->exec_mem_alloc_start)
        wasted_bytes = ((char*)blk->native) - ((char*)blk->exec_mem_alloc_start);
    jit_profile_set_native_insts(&sh4->jit_profile, jit_blk->profile,
                                 blk->bytes_used - wasted_bytes, blk->native);
#endif

    il_code_block_cleanup(&il_blk);
}
#endif

static inline void
sh4_jit_compile_intp(void *cpu, void *blk_ptr, uint32_t pc) {
    struct Sh4 const *sh4 = (struct Sh4*)cpu;
    struct il_code_block il_blk;
    struct jit_code_block *jit_blk = (struct jit_code_block*)blk_ptr;
    struct code_block_intp *blk = &jit_blk->intp;
    struct sh4_jit_compile_ctx ctx = {
        .last_inst_type = SH4_GROUP_NONE,
        .cycle_count = 0,
        .sz_bit = sh4_fpscr_sz(sh4),
        .pr_bit = sh4_fpscr_pr(sh4),
        .in_delay_slot = false,
        .dirty_fpscr = false,
        .have_reg_slot = false
    };

    il_code_block_init(&il_blk);

#ifdef JIT_PROFILE
    il_blk.profile = jit_blk->profile;
#endif

    sh4_jit_il_code_block_compile(cpu, &ctx, jit_blk, &il_blk, pc);

    jit_optimize(&il_blk);

#ifdef JIT_PROFILE
    unsigned inst_no;
    for (inst_no = 0; inst_no < il_blk.inst_count; inst_no++) {
        jit_profile_push_il_inst(&sh4->jit_profile, jit_blk->profile,
                                 il_blk.inst_list + inst_no);
    }
#endif

    code_block_intp_compile(cpu, blk, &il_blk, ctx.cycle_count * SH4_CLOCK_SCALE);
    il_code_block_cleanup(&il_blk);
}

/*
 * Since the SH4 doesn't own the jit context, this doesn't really do anything
 * except initialize the profiling context if that's enabled (because the
 * profiling context does actually belong to the SH4 even though the jit itself
 * does not).
 */
void sh4_jit_init(struct Sh4 *sh4);
void sh4_jit_cleanup(struct Sh4 *sh4);

#ifdef ENABLE_JIT_X86_64
void sh4_jit_set_native_dispatch_meta(struct native_dispatch_meta *meta);
#endif

/*
 * disassembly function that emits a function call to the instruction's
 * interpreter implementation.
 */
bool sh4_jit_fallback(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                      struct il_code_block *block, unsigned pc,
                      struct InstOpcode const *op, cpu_inst_param inst);

// disassemble the rts instruction
bool sh4_jit_rts(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                 struct il_code_block *block, unsigned pc,
                 struct InstOpcode const *op, cpu_inst_param inst);

// disassemble the rte instruction
bool sh4_jit_rte(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                 struct il_code_block *block, unsigned pc,
                 struct InstOpcode const *op, cpu_inst_param inst);

// disassemble the "braf rn" instruction.
bool sh4_jit_braf_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                     struct il_code_block *block, unsigned pc,
                     struct InstOpcode const *op, cpu_inst_param inst);

// disassembles the "bsrf rn" instruction"
bool sh4_jit_bsrf_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                     struct il_code_block *block, unsigned pc,
                     struct InstOpcode const *op, cpu_inst_param inst);

// disassembles the "bf" instruction
bool sh4_jit_bf(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                struct il_code_block *block, unsigned pc,
                struct InstOpcode const *op, cpu_inst_param inst);

// disassembles the "bt" instruction
bool sh4_jit_bt(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                struct il_code_block *block, unsigned pc,
                struct InstOpcode const *op, cpu_inst_param inst);

// disassembles the "bf/s" instruction
bool sh4_jit_bfs(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                 struct il_code_block *block, unsigned pc,
                 struct InstOpcode const *op, cpu_inst_param inst);

// disassembles the "bt/s" instruction
bool sh4_jit_bts(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                 struct il_code_block *block, unsigned pc,
                 struct InstOpcode const *op, cpu_inst_param inst);

// disassembles the "bra" instruction
bool sh4_jit_bra(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                 struct il_code_block *block, unsigned pc,
                 struct InstOpcode const *op, cpu_inst_param inst);

// disassembles the "bsr" instruction
bool sh4_jit_bsr(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                 struct il_code_block *block, unsigned pc,
                 struct InstOpcode const *op, cpu_inst_param inst);

// disassembles the "jmp @rn" instruction
bool sh4_jit_jmp_arn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                     struct il_code_block *block, unsigned pc,
                     struct InstOpcode const *op, cpu_inst_param inst);

// disassembles the "jsr @rn" instruction
bool sh4_jit_jsr_arn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                     struct il_code_block *block, unsigned pc,
                     struct InstOpcode const *op, cpu_inst_param inst);

// disassembles the "mov.w @(disp, pc), rn" instruction
bool
sh4_jit_movw_a_disp_pc_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst);

// disassembles the "mov.l @(disp, pc), rn" instruction
bool
sh4_jit_movl_a_disp_pc_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst);
bool
sh4_jit_mova_a_disp_pc_r0(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst);

bool sh4_jit_nop(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                 struct il_code_block *block, unsigned pc,
                 struct InstOpcode const *op, cpu_inst_param inst);
bool sh4_jit_ocbi_arn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                      struct il_code_block *block, unsigned pc,
                      struct InstOpcode const *op, cpu_inst_param inst);
bool sh4_jit_ocbp_arn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                      struct il_code_block *block, unsigned pc,
                      struct InstOpcode const *op, cpu_inst_param inst);
bool sh4_jit_ocbwb_arn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                       struct il_code_block *block, unsigned pc,
                       struct InstOpcode const *op, cpu_inst_param inst);

// ADD Rm, Rn
// 0011nnnnmmmm1100
bool sh4_jit_add_rm_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                       struct il_code_block *block, unsigned pc,
                       struct InstOpcode const *op, cpu_inst_param inst);

// ADD #imm, Rn
// 0111nnnniiiiiiii
bool sh4_jit_add_imm_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                        struct il_code_block *block, unsigned pc,
                        struct InstOpcode const *op, cpu_inst_param inst);

// XOR Rm, Rn
// 0010nnnnmmmm1010
bool sh4_jit_xor_rm_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                       struct il_code_block *block, unsigned pc,
                       struct InstOpcode const *op, cpu_inst_param inst);

// MOV Rm, Rn
// 0110nnnnmmmm0011
bool sh4_jit_mov_rm_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                       struct il_code_block *block, unsigned pc,
                       struct InstOpcode const *op, cpu_inst_param inst);

// AND Rm, Rn
// 0010nnnnmmmm1001
bool sh4_jit_and_rm_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                       struct il_code_block *block, unsigned pc,
                       struct InstOpcode const *op, cpu_inst_param inst);

// OR Rm, Rn
// 0010nnnnmmmm1011
bool sh4_jit_or_rm_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                      struct il_code_block *block, unsigned pc,
                      struct InstOpcode const *op, cpu_inst_param inst);

// SUB Rm, Rn
// 0011nnnnmmmm1000
bool sh4_jit_sub_rm_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                       struct il_code_block *block, unsigned pc,
                       struct InstOpcode const *op, cpu_inst_param inst);

// AND #imm, R0
// 11001001iiiiiiii
bool
sh4_inst_binary_andb_imm_r0(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                            struct il_code_block *block, unsigned pc,
                            struct InstOpcode const *op, cpu_inst_param inst);

// OR #imm, R0
// 11001011iiiiiiii
bool sh4_jit_or_imm8_r0(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                        struct il_code_block *block, unsigned pc,
                        struct InstOpcode const *op, cpu_inst_param inst);

// XOR #imm, R0
// 11001010iiiiiiii
bool sh4_jit_xor_imm8_r0(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst);

// TST Rm, Rn
// 0010nnnnmmmm1000
bool sh4_jit_tst_rm_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                       struct il_code_block *block, unsigned pc,
                       struct InstOpcode const *op, cpu_inst_param inst);

// TST #imm, R0
// 11001000iiiiiiii
bool sh4_jit_tst_imm8_r0(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst);

// MOV.B @(R0, Rm), Rn
// 0000nnnnmmmm1100
bool sh4_jit_movb_a_r0_rm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                             struct il_code_block *block, unsigned pc,
                             struct InstOpcode const *op, cpu_inst_param inst);

// MOV.L @(R0, Rm), Rn
// 0000nnnnmmmm1110
bool sh4_jit_movl_a_r0_rm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                             struct il_code_block *block, unsigned pc,
                             struct InstOpcode const *op, cpu_inst_param inst);

// MOV.L Rm, @(disp, Rn)
// 0001nnnnmmmmdddd
bool
sh4_jit_movl_rm_a_disp4_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                           struct il_code_block *block, unsigned pc,
                           struct InstOpcode const *op, cpu_inst_param inst);

// MOV.B @Rm, Rn
// 0110nnnnmmmm0000
bool sh4_jit_movb_arm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst);

// MOV.L @Rm, Rn
// 0110nnnnmmmm0010
bool sh4_jit_movl_arm_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst);

// MOV.B @Rm+, Rn
// 0110nnnnmmmm0100
bool sh4_jit_movb_armp_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst);

// MOV.W @Rm+, Rn
// 0110nnnnmmmm0101
bool
sh4_jit_movw_armp_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                     struct il_code_block *block, unsigned pc,
                     struct InstOpcode const *op, cpu_inst_param inst);
// MOV.L @Rm+, Rn
// 0110nnnnmmmm0110
bool sh4_jit_movl_armp_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst);

// LDS.L @Rm+, PR
// 0100mmmm00100110
bool sh4_jit_ldsl_armp_pr(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst);
// MOV.L @(disp, Rm), Rn
// 0101nnnnmmmmdddd
bool
sh4_jit_movl_a_disp4_rm_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                           struct il_code_block *block, unsigned pc,
                           struct InstOpcode const *op, cpu_inst_param inst);

// MOV #imm, Rn
// 1110nnnniiiiiiii
bool sh4_jit_mov_imm8_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst);

// SHLL16 Rn
// 0100nnnn00101000
bool sh4_jit_shll16_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                       struct il_code_block *block, unsigned pc,
                       struct InstOpcode const *op, cpu_inst_param inst);

// SHLL2 Rn
// 0100nnnn00001000
bool sh4_jit_shll2_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                      struct il_code_block *block, unsigned pc,
                      struct InstOpcode const *op, cpu_inst_param inst);

// SHLL8 Rn
// 0100nnnn00011000
bool sh4_jit_shll8_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                      struct il_code_block *block, unsigned pc,
                      struct InstOpcode const *op, cpu_inst_param inst);

// SHAR Rn
// 0100nnnn00100001
bool sh4_jit_shar_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                     struct il_code_block *block, unsigned pc,
                     struct InstOpcode const *op, cpu_inst_param inst);

// SHLR Rn
// 0100nnnn00000001
bool sh4_jit_shlr_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                     struct il_code_block *block, unsigned pc,
                     struct InstOpcode const *op, cpu_inst_param inst);

// SHAD Rm, Rn
// 0100nnnnmmmm1100
bool sh4_jit_shad_rm_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                        struct il_code_block *block, unsigned pc,
                        struct InstOpcode const *op, cpu_inst_param inst);

// SHLR2 Rn
// 0100nnnn00001001
bool sh4_jit_shlr2_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                      struct il_code_block *block, unsigned pc,
                      struct InstOpcode const *op, cpu_inst_param inst);

// SHLR8 Rn
// 0100nnnn00011001
bool sh4_jit_shlr8_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                      struct il_code_block *block, unsigned pc,
                      struct InstOpcode const *op, cpu_inst_param inst);

// SHLR16 Rn
// 0100nnnn00101001
bool sh4_jit_shlr16_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                       struct il_code_block *block, unsigned pc,
                       struct InstOpcode const *op, cpu_inst_param inst);

// SHLL Rn
// 0100nnnn00000000
bool sh4_jit_shll_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                     struct il_code_block *block, unsigned pc,
                     struct InstOpcode const *op, cpu_inst_param inst);

// SHAL Rn
// 0100nnnn00100000
bool sh4_jit_shal_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                     struct il_code_block *block, unsigned pc,
                     struct InstOpcode const *op, cpu_inst_param inst);

// SWAP.W Rm, Rn
// 0110nnnnmmmm1001
bool sh4_jit_swapw_rm_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst);

// CMP/HI Rm, Rn
// 0011nnnnmmmm0110
bool sh4_jit_cmphi_rm_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst);

// MULU.W Rm, Rn
// 0010nnnnmmmm1110
bool sh4_jit_muluw_rm_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst);

// STS MACL, Rn
// 0000nnnn00011010
bool sh4_jit_sts_macl_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst);

// STS MACL, Rn
// 0000nnnn00011010
bool sh4_jit_sts_macl_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst);

// MOV.B Rm, @-Rn
// 0010nnnnmmmm0100
bool sh4_jit_movb_rm_amrn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst);

// MOV.B Rm, @Rn
// 0010nnnnmmmm0000
bool sh4_jit_movb_rm_arn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst);

// MOV.L Rm, @-Rn
// 0010nnnnmmmm0110
bool sh4_jit_movl_rm_amrn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst);

// MOV.L Rm, @Rn
// 0010nnnnmmmm0010
bool sh4_jit_movl_rm_arn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst);

// CMP/EQ Rm, Rn
// 0011nnnnmmmm0000
bool sh4_jit_cmpeq_rm_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst);

// CMP/HS Rm, Rn
// 0011nnnnmmmm0010
bool sh4_jit_cmphs_rm_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst);

// CMP/GT Rm, Rn
// 0011nnnnmmmm0111
bool sh4_jit_cmpgt_rm_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst);

// CMP/GE Rm, Rn
// 0011nnnnmmmm0011
bool sh4_jit_cmpge_rm_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst);

// CMP/PL Rn
// 0100nnnn00010101
bool sh4_jit_cmppl_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                      struct il_code_block *block, unsigned pc,
                      struct InstOpcode const *op, cpu_inst_param inst);

// CMP/PZ Rn
// 0100nnnn00010001
bool sh4_jit_cmppz_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                      struct il_code_block *block, unsigned pc,
                      struct InstOpcode const *op, cpu_inst_param inst);

// NOT Rm, Rn
// 0110nnnnmmmm0111
bool sh4_jit_not_rm_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                       struct il_code_block *block, unsigned pc,
                       struct InstOpcode const *op, cpu_inst_param inst);

// DT Rn
// 0100nnnn00010000
bool sh4_jit_dt_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                   struct il_code_block *block, unsigned pc,
                   struct InstOpcode const *op, cpu_inst_param inst);

// CLRT
// 0000000000001000
bool sh4_jit_clrt(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                  struct il_code_block *block, unsigned pc,
                  struct InstOpcode const *op, cpu_inst_param inst);

// SETT
// 0000000000011000
bool sh4_jit_sett(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                  struct il_code_block *block, unsigned pc,
                  struct InstOpcode const *op, cpu_inst_param inst);

// MOVT Rn
// 0000nnnn00101001
bool sh4_jit_movt(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                  struct il_code_block *block, unsigned pc,
                  struct InstOpcode const *op, cpu_inst_param inst);

// MOV.L @(disp, GBR), R0
// 11000110dddddddd
bool
sh4_jit_movl_a_disp8_gbr_r0(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                            struct il_code_block *block, unsigned pc,
                            struct InstOpcode const *op, cpu_inst_param inst);

// STS.L PR, @-Rn
// 0100nnnn00100010
bool
sh4_jit_stsl_pr_amrn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                  struct il_code_block *block, unsigned pc,
                  struct InstOpcode const *op, cpu_inst_param inst);

// EXTU.B Rm, Rn
// 0110nnnnmmmm1100
bool
sh4_jit_extub_rm_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                    struct il_code_block *block, unsigned pc,
                    struct InstOpcode const *op, cpu_inst_param inst);

// LDS Rm, FPSCR
// 0100mmmm01101010
bool sh4_jit_lds_rm_fpscr(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst);

// LDS.L @Rm+, FPSCR
// 0100mmmm01100110
bool sh4_jit_ldsl_armp_fpscr(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                             struct il_code_block *block, unsigned pc,
                             struct InstOpcode const *op, cpu_inst_param inst);

// FSCHG
// 1111001111111101
bool sh4_jit_fschg(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                   struct il_code_block *block, unsigned pc,
                   struct InstOpcode const *op, cpu_inst_param inst);

// TODO: below FPU things
// needs to be implemented but can be put off until we have *real* FPU support:
// FRCHG
// probably does not need to be impelemnted
// STS FPSCR, Rn
// STS.L FPSCR, @-Rn

// FMOV.S FRm, @(R0, Rn)
// 1111nnnnmmmm0111
// FMOV DRm, @(R0, Rn)
// 1111nnnnmmm00111
// FMOV XDm, @(R0, Rn)
// 1111nnnnmmm10111
bool sh4_jit_fmov_fpu_a_r0_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                              struct il_code_block *block, unsigned pc,
                              struct InstOpcode const *op, cpu_inst_param inst);

// FMOV.S @Rm+, FRn
// 1111nnnnmmmm1001
// FMOV @Rm+, DRn
// 1111nnn0mmmm1001
// FMOV @Rm+, XDn
// 1111nnn1mmmm1001
bool sh4_jit_fmov_fpu_armp_fpu(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                               struct il_code_block *block, unsigned pc,
                               struct InstOpcode const *op, cpu_inst_param inst);

// FMOV.S @Rm, FRn
// 1111nnnnmmmm1000
// FMOV @Rm, DRn
// 1111nnn0mmmm1000
// FMOV @Rm, XDn
// 1111nnn1mmmm1000
bool sh4_jit_fmov_arm_fpu(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst);

// FMOV.S FRm, @-Rn
// 1111nnnnmmmm1011
// FMOV DRm, @-Rn
// 1111nnnnmmm01011
// FMOV XDm, @-Rn
// 1111nnnnmmm11011
bool sh4_jit_fmov_fpu_amrn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                           struct il_code_block *block, unsigned pc,
                           struct InstOpcode const *op, cpu_inst_param inst);

// FMUL FRm, FRn
// 1111nnnnmmmm0010
// FMUL DRm, DRn
// 1111nnn0mmm00010
bool sh4_jit_fmul_frm_frn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst);

// FCMP/GT FRm, FRn
// 1111nnnnmmmm0101
// FCMP/GT DRm, DRn
// 1111nnn0mmm00101
bool sh4_jit_fcmpgt_frm_frn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                            struct il_code_block *block, unsigned pc,
                            struct InstOpcode const *op, cpu_inst_param inst);

// FSUB FRm, FRn
// 1111nnnnmmmm0001
// FSUB DRm, DRn
// 1111nnn0mmm00001
bool sh4_jit_fsub_frm_frn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst);

// FADD FRm, FRn
// 1111nnnnmmmm0000
// FADD DRm, DRn
// 1111nnn0mmm00000
bool sh4_jit_fadd_frm_frn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst);

// FTRC FRm, FPUL
// 1111mmmm00111101
// FTRC DRm, FPUL
// 1111mmm000111101
bool sh4_jit_ftrc_frm_fpul(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                           struct il_code_block *block, unsigned pc,
                           struct InstOpcode const *op, cpu_inst_param inst);

// FMOV.S @(R0, Rm), FRn
// 1111nnnnmmmm0110
// FMOV @(R0, Rm), DRn
// 1111nnn0mmmm0110
// FMOV @(R0, Rm), XDn
// 1111nnn1mmmm0110
bool sh4_jit_fmovs_a_r0_rm_fpu(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                               struct il_code_block *block, unsigned pc,
                               struct InstOpcode const *op, cpu_inst_param inst);

// FMOV FRm, FRn
// 1111nnnnmmmm1100
// FMOV DRm, DRn
// 1111nnn0mmm01100
// FMOV XDm, DRn
// 1111nnn0mmm11100
// FMOV DRm, XDn
// 1111nnn1mmm01100
// FMOV XDm, XDn
// 1111nnn1mmm11100
bool sh4_jit_fmov_frm_frn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst);

#endif
