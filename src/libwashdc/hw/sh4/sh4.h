/*******************************************************************************
 *
 * Copyright 2016-2020 snickerbockers
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

#ifndef SH4_H_
#define SH4_H_

#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "washdc/error.h"
#include "washdc/types.h"
#include "sh4_inst.h"
#include "sh4_reg.h"
#include "sh4_mem.h"
#include "sh4_tmu.h"
#include "sh4_ocache.h"
#include "sh4_excp.h"
#include "sh4_scif.h"
#include "sh4_dmac.h"
#include "dc_sched.h"
#include "atomics.h"

#ifdef JIT_PROFILE
#include "jit/jit_profile.h"
#endif

/*
 * The clock-scale is here defined as the number of scheduler cyclers per sh4
 * cycle.
 *
 * To convert dc_sched cycles to sh4 cycles, divide by SH4_CLOCK_SCALE
 * To convert sh4 cycles to dc_sched_cycles, multiply by SH4_CLOCK_SCALE
 */
#define SH4_CLOCK_SCALE (SCHED_FREQUENCY / (200 * 1000 * 1000))

static_assert(SCHED_FREQUENCY % (200 * 1000 * 1000) == 0,
              "scheduler frequency does not cleanly divide by SH4 frequency");

/* Hitachi SuperH-4 interpreter */

#define SH4_N_FLOAT_REGS 16
#define SH4_N_DOUBLE_REGS 8

enum Sh4ExecState {
    SH4_EXEC_STATE_NORM,
    SH4_EXEC_STATE_SLEEP,
    SH4_EXEC_STATE_STANDBY
};
typedef enum Sh4ExecState Sh4ExecState;

struct Sh4 {
    struct dc_clock *clk;

    Sh4ExecState exec_state;

    reg32_t reg[SH4_REGISTER_COUNT];

    /*
     * If the CPU is executing a delayed branch instruction, then
     * delayed_branch will be true and delayed_branch_addr will point to the
     * address to branch to.  After executing one instruction, delayed_branch
     * will be set to false and the CPU will jump to delayed_branch_addr.
     *
     * If the branch instruction evaluates to false (ie, there is not a delayed
     * branch) then delayed_branch will never be set to true.  This means that
     * the interpreter will not raise any exceptions caused by executing a
     * branch instruction in a delay slot; this is an inaccuracy which may need
     * to be revisited in the future.
     */
    bool delayed_branch;
    addr32_t delayed_branch_addr;

    /*
     * if true, the PC will not be incremented between instructions
     * (interpreter-mode only).  The purpose of this is to make sure the PC
     * isn't incremented after a CPU exception has pointed the PC to an
     * exception handler.  This value will be automatically reset to false
     * between instructions.
     */
    bool dont_increment_pc;

    struct sh4_tmu tmu;

    /*
     * operand cache - this is really only here to be used as RAM
     * when the ORA bit is set in CCR
     */
    struct sh4_ocache ocache;

    struct sh4_intc intc;

    struct sh4_scif scif;

    struct sh4_dmac dmac;

    struct sh4_mem mem;

#ifdef JIT_PROFILE
    struct jit_profile_ctxt jit_profile;
#endif

    /*
     * pointer to place where memory-mapped registers are stored.
     * RegReadHandlers and RegWriteHandlers do not need to use this as long as
     * they are consistent.
     */
    uint8_t *reg_area;

    /*
     * this is used by sh4_count_inst_cycles to track the type of the last
     * instruction that was executed.  This is used to determine if the next
     * instruction to be executed should advance the cycle count, or if it
     * would have been executed by the second pipeline on a real sh4.
     */
     sh4_inst_group_t last_inst_type;
};

typedef struct Sh4 Sh4;

void sh4_init(Sh4 *sh4, struct dc_clock *clk);
void sh4_cleanup(Sh4 *sh4);

// reset all values to their power-on-reset values
void sh4_on_hard_reset(Sh4 *sh4);

// returns the program counter
reg32_t sh4_get_pc(Sh4 *sh4);

/*
 * call this function instead of setting the value directly to make sure
 * that any state changes are immediately processed.
 */
void sh4_set_fpscr(void *cpu, reg32_t new_val);

// clear the cause bits in the FPSCR reg
static inline void sh4_fpu_clear_cause(Sh4 *sh4) {
#ifndef SH4_FPU_FAST
    sh4->reg[SH4_REG_FPSCR] &= ~SH4_FPSCR_CAUSE_MASK;
#endif
}

// these four APIs are intended primarily for debuggers to use
void sh4_get_regs(Sh4 *sh4, reg32_t reg_out[SH4_REGISTER_COUNT]);
/* FpuReg sh4_get_fpu(Sh4 *sh4); */
void sh4_set_regs(Sh4 *sh4, reg32_t const reg_out[SH4_REGISTER_COUNT]);
void sh4_set_individual_reg(Sh4 *sh4, unsigned reg_no, reg32_t reg_val);
/* void sh4_set_fpu(Sh4 *sh4, FpuReg src); */

void sh4_bank_switch(Sh4 *sh4);
void sh4_bank_switch_maybe(Sh4 *sh4, reg32_t old_sr, reg32_t new_sr);

void sh4_fpu_bank_switch(Sh4 *sh4);
void sh4_fpu_bank_switch_maybe(Sh4 *sh4, reg32_t old_fpscr, reg32_t new_fpscr);

/*
 * return the index of the given general-purpose register.
 * This function takes bank-switching into account.
 */
static inline sh4_reg_idx_t sh4_gen_reg_idx(Sh4 *sh4, int reg_no) {
    assert(!(reg_no & ~0xf));

    return (sh4_reg_idx_t)(SH4_REG_R0 + reg_no);
}

/*
 * return a pointer to the given general-purpose register.
 * This function takes bank-switching into account.
 */
static inline reg32_t *sh4_gen_reg(Sh4 *sh4, int idx) {
    return sh4->reg + sh4_gen_reg_idx(sh4, idx);
}

/* return an index to the given banked general-purpose register */
static inline sh4_reg_idx_t sh4_bank_reg_idx(Sh4 *sh4, int idx) {
    assert(!(idx & ~0x7));

    return (sh4_reg_idx_t)(SH4_REG_R0_BANK + idx);
}

// return a pointer to the given banked general-purpose register
static inline reg32_t *sh4_bank_reg(Sh4 *sh4, int idx) {
    return sh4->reg + sh4_bank_reg_idx(sh4, idx);
}

static inline sh4_reg_idx_t sh4_bank0_reg_idx(Sh4 *sh4, int idx) {
    assert(!(idx & ~0x7));

    uint32_t sr = sh4->reg[SH4_REG_SR];
    bool sr_rb = sr & SH4_SR_RB_MASK;
    if (!(sr & SH4_SR_MD_MASK))
        sr_rb = false;

    if (sr_rb)
        return (sh4_reg_idx_t)(SH4_REG_R0_BANK + idx);
    return (sh4_reg_idx_t)(SH4_REG_R0 + idx);
}

static inline reg32_t *sh4_bank0_reg(Sh4 *sh4, int idx) {
    return sh4->reg + sh4_bank0_reg_idx(sh4, idx);
}

static inline sh4_reg_idx_t sh4_bank1_reg_idx(Sh4 *sh4, int idx) {
    assert(!(idx & ~0x7));

    uint32_t sr = sh4->reg[SH4_REG_SR];
    bool sr_rb = sr & SH4_SR_RB_MASK;
    if (!(sr & SH4_SR_MD_MASK))
        sr_rb = false;

    if (sr_rb)
        return (sh4_reg_idx_t)(SH4_REG_R0 + idx);
    return (sh4_reg_idx_t)(SH4_REG_R0_BANK + idx);
}

static inline reg32_t *sh4_bank1_reg(Sh4 *sh4, int idx) {
    return sh4->reg + sh4_bank1_reg_idx(sh4, idx);
}

/*
 * access single-precision floating-point register,
 * taking bank-switching into account
 */
static inline float *sh4_fpu_fr(Sh4 *sh4, unsigned reg_no) {
    assert(reg_no < SH4_N_FLOAT_REGS);

    return (float*)(sh4->reg + SH4_REG_FR0 + reg_no);
}

static inline float *sh4_fpu_xf(Sh4 *sh4, unsigned reg_no) {
    assert(reg_no < SH4_N_FLOAT_REGS);

    return (float*)(sh4->reg + SH4_REG_XF0 + reg_no);
}

static inline float *sh4_bank0_fpu_fr(Sh4 *sh4, unsigned reg_no) {
    if (sh4->reg[SH4_REG_FPSCR] & SH4_FPSCR_FR_MASK)
        return sh4_fpu_xf(sh4, reg_no);
    return sh4_fpu_fr(sh4, reg_no);
}

static inline float *sh4_bank1_fpu_fr(Sh4 *sh4, unsigned reg_no) {
    if (sh4->reg[SH4_REG_FPSCR] & SH4_FPSCR_FR_MASK)
        return sh4_fpu_fr(sh4, reg_no);
    return sh4_fpu_xf(sh4, reg_no);
}

/*
 * access double-precision floating-point register,
 * taking bank-switching into account
 */
static inline double *sh4_fpu_dr(Sh4 *sh4, unsigned reg_no) {
    assert(reg_no < SH4_N_DOUBLE_REGS);

    return (double*)(sh4->reg + SH4_REG_DR0 + (reg_no * 2));
}

static inline double *sh4_fpu_xd(Sh4 *sh4, unsigned reg_no) {
    assert(reg_no < SH4_N_DOUBLE_REGS);

    return (double*)(sh4->reg + SH4_REG_XD0 + (reg_no << 1));
}

/*
 * this function should be called every time sr has just been written to and
 * bits other than T/Q/M/S may have changed
 */
void sh4_on_sr_change(Sh4 *sh4, reg32_t old_sr);

/*
 * The purpose of this function is to do things which need to be performed
 * periodically, but not with any urgency or hard timing requirements.
 *
 * Currently, that means the only thing it does is check to see if the serial
 * server wants to communicate with the SCIF; in the future other tasks may be
 * added in here as well if I need them.
 */
static inline void sh4_periodic(Sh4 *sh4) {
    if (!washdc_atomic_flag_test_and_set(&sh4->scif.nothing_pending))
        sh4_scif_periodic(sh4);
}

dc_cycle_stamp_t sh4_get_cycles(struct Sh4 *sh4);

// Fetches the given instruction's metadata and returns it.
static inline InstOpcode const *sh4_decode_inst(cpu_inst_param inst) {
    return sh4_inst_lut[inst & 0xffff];
}

/*
 * return the number of cycles this instruction requires.  This is not the same
 * as the instruction's issue cycles due to the dual-issue pipeline of the sh4.
 */
static inline unsigned
sh4_count_inst_cycles(InstOpcode const *op, unsigned *last_inst_type_p) {
    unsigned last_inst_type = *last_inst_type_p;
    unsigned n_cycles;
    if ((last_inst_type == SH4_GROUP_NONE) ||
        ((op->group == SH4_GROUP_CO) ||
         (last_inst_type == SH4_GROUP_CO) ||
         ((last_inst_type == op->group) && (op->group != SH4_GROUP_MT)))) {
        // This instruction was not free
        n_cycles = op->issue;

        /*
         * no need to check for SH4_GROUP_CO here because we'll do that when we
         * check for last_inst_type==SH4_GROUP_CO next time we're in this if
         * statement
         */
        last_inst_type = op->group;
    } else {
        /*
         * cash in on the dual-issue pipeline's "free" instruction and set
         * last_inst_type to SH4_GROUP_NONE so that the next instruction is
         * not free.
         */
        n_cycles = 0;
        last_inst_type = SH4_GROUP_NONE;
    }
    *last_inst_type_p = last_inst_type;
    return n_cycles;
}

/*
 * In Little-Endian mode, the SH-4 swaps the upper and lower quads of
 * double-precision floating point.  The two quads are themselves still
 * little-endian.
 *
 * These functions should only be used by opcodes that need to interpret the
 * data in the register as a double.  Opcodes that merely need to move the
 * contents of a double-precision float register should use a simple binary copy
 * instead.
 */
static inline double sh4_read_double(struct Sh4 *sh4, unsigned dr_reg) {
#ifdef INVARIANTS
    if ((dr_reg % 2) || (dr_reg > 14))
        RAISE_ERROR(ERROR_INTEGRITY);
#endif
    double ret_val;
    double const *ptr = (double*)(sh4->reg + SH4_REG_DR0 + dr_reg);

    memcpy(&ret_val, ((uint32_t*)ptr) + 1, sizeof(uint32_t));
    memcpy(((uint32_t*)&ret_val) + 1, ptr, sizeof(uint32_t));

    return ret_val;
}

static inline void sh4_write_double(struct Sh4 *sh4, unsigned dr_reg, double val) {
#ifdef INVARIANTS
    if ((dr_reg % 2) || (dr_reg > 14))
        RAISE_ERROR(ERROR_INTEGRITY);
#endif
    double *ptr = (double*)(sh4->reg + SH4_REG_DR0 + dr_reg);

    memcpy(ptr, ((uint32_t*)&val) + 1, sizeof(uint32_t));
    memcpy(((uint32_t*)ptr) + 1, &val, sizeof(uint32_t));
}

uint32_t sh4_pc_next(struct Sh4 *sh4);

static inline bool sh4_fpscr_pr(struct Sh4 const *sh4) {
    return (bool)(sh4->reg[SH4_REG_FPSCR] & SH4_FPSCR_PR_MASK);
}

static inline bool sh4_fpscr_sz(struct Sh4 const *sh4) {
    return (bool)(sh4->reg[SH4_REG_FPSCR] & SH4_FPSCR_SZ_MASK);
}

#ifdef ENABLE_MMU
static inline bool sh4_mmu_at(struct Sh4 const *sh4) {
    return (bool)(sh4->reg[SH4_REG_MMUCR] & SH4_MMUCR_AT_MASK);
}

static inline bool sh4_fpu_enabled(struct Sh4 const *sh4) {
    return !(bool)(sh4->reg[SH4_REG_SR] & SH4_SR_FD_MASK);
}

#endif

#endif
