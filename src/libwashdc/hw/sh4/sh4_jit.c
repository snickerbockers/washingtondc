/*******************************************************************************
 *
 *
 *    WashingtonDC Dreamcast Emulator
 *    Copyright (C) 2018-2020 snickerbockers
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

#include "sh4asm_core/disas.h"

#include "jit/jit_il.h"
#include "jit/code_block.h"
#include "jit/jit_mem.h"

#ifdef JIT_PROFILE
#include "jit/jit_profile.h"
#endif

#include "log.h"
#include "sh4.h"
#include "sh4_read_inst.h"
#include "sh4_jit.h"

#ifdef ENABLE_JIT_X86_64
#include "jit/x86_64/native_dispatch.h"
#endif

#include "washdc/hostfile.h"

static jit_hash sh4_jit_hash_wrapper(void *sh4, uint32_t addr);

#ifdef ENABLE_JIT_X86_64
void sh4_jit_set_native_dispatch_meta(struct native_dispatch_meta *meta) {
#ifdef JIT_PROFILE
    meta->profile_notify = sh4_jit_profile_notify;
#endif
    meta->on_compile = sh4_jit_compile_native;
    meta->hash_func = sh4_jit_hash_wrapper;
}
#endif

enum reg_status {
    // the register resides in the sh4's reg array
    REG_STATUS_SH4,

    /*
     * the register resides in a slot, but it does not need to be written back
     * to the sh4's reg array because it has not been written to (yet).
     */
    REG_STATUS_SLOT_AND_SH4,

    /*
     * the register resides in a slot and the copy of the register in the sh4's
     * reg array is outdated.  The slot will need to be written back to the
     * sh4's reg array at some point before the current code block ends.
     */
    REG_STATUS_SLOT
};

struct residency {
    enum reg_status stat;
    int slot_no;

    /*
     * these track the value of inst_count (from the il_code_block) the last
     * time this slot was used by this register.  The idea is that the IL will
     * be able to use these to minimize the number of slots in use at any time
     * by writing slots back to the sh4 registers after they've been used for
     * the last time.  Currently that's not implemented, and slots are only
     * written back when they need to be.
     */
    unsigned last_write, last_read;
};

// this is a temporary space the il uses to map sh4 registers to slots
static struct residency reg_map[SH4_REGISTER_COUNT];

static void sh4_jit_set_sr(void *ctx, uint32_t new_sr_val);

static void res_associate_reg(unsigned reg_no, unsigned slot_no);
static void res_disassociate_reg(Sh4 *sh4, struct sh4_jit_compile_ctx *ctx,
                                 struct il_code_block *block, unsigned reg_no);

/*
 * this will load the given register into a slot if it is not already in a slot
 * and then return the index of the slot it resides in.
 *
 * the register will be marked as REG_STATUS_SLOT_AND_SH4 if it its status is
 * REG_STATUS_SH4.  Otherwise the reg status will be left alone.
 */
static unsigned reg_slot(Sh4 *sh4, struct sh4_jit_compile_ctx *ctx,
                         struct il_code_block *block, unsigned reg_no,
                         enum washdc_jit_slot_tp tp);

/*
 * return the slot index of a given register.  If the register is
 * REG_STATUS_SH4, then allocate a new slot for it, set the reg status to
 * REG_STATUS_SLOT and return the new slot.  If the reg status is
 * REG_STATUS_SLOT_AND_SH4, then the existing slot index will be returned but
 * the reg status will still be set to REG_STATUS_SLOT.
 *
 * This function will not load the register into the slot; instead it will set
 * the register residency to point to the slot without initializing the slot
 * contents.  This function is intended for situations in which the preexisting
 * contents of a given register are irrelevant because they will immediately be
 * overwritten.
 */
static unsigned
reg_slot_noload(Sh4 *sh4, struct il_code_block *block, unsigned reg_no);

#ifdef JIT_PROFILE
static void sh4_jit_profile_disas(washdc_hostfile out, uint32_t addr, void const *instp);
static void sh4_jit_profile_emit_fn(char ch);
#endif

/*
 * emit il ops to compute a jump hash and place it in hash_slot.
 *
 * jmp_addr_slot should be a slot which holds the address which is being
 * hashed.
 * fpscr_slot should be a slot which holds the fpscr.
 * hash_slot is the slot where the final hash value should be written to.
 *
 * the contents of jmp_addr_slot and fscpr_slot will not be modified by this
 * function.
 */
static void sh4_jit_hash_slot(struct Sh4 *sh4, struct il_code_block *block,
                              unsigned jmp_addr_slot, unsigned hash_slot,
                              unsigned fpscr_slot);

/*
 * this is like sh4_jit_hash_slot, except it's for situations in which the
 * values of fpscr's PR and SZ bits are anready know.
 */
static void
sh4_jit_hash_slot_known_fpscr(struct Sh4 *sh4,
                              struct sh4_jit_compile_ctx const *ctx,
                              struct il_code_block *block,
                              unsigned jmp_addr_slot, unsigned hash_slot);

static unsigned
get_regbase_slot(struct Sh4 *sh4, struct sh4_jit_compile_ctx *ctx,
                 struct il_code_block *block) {
    if (ctx->have_reg_slot)
        return ctx->reg_slot;
    ctx->reg_slot = alloc_slot(block, WASHDC_JIT_SLOT_HOST_PTR);
    jit_set_slot_host_ptr(block, ctx->reg_slot, sh4->reg);
    ctx->have_reg_slot = true;
    return ctx->reg_slot;
}

void sh4_jit_init(struct Sh4 *sh4) {
#ifdef JIT_PROFILE
    jit_profile_ctxt_init(&sh4->jit_profile, sizeof(uint16_t));
    sh4->jit_profile.disas = sh4_jit_profile_disas;
#endif
}

void sh4_jit_cleanup(struct Sh4 *sh4) {
#ifdef JIT_PROFILE
    washdc_hostfile outfile =
        washdc_hostfile_open("sh4_profile.txt",
                             WASHDC_HOSTFILE_WRITE | WASHDC_HOSTFILE_TEXT);
    if (outfile != WASHDC_HOSTFILE_INVALID) {
        jit_profile_print(&sh4->jit_profile, outfile);
        washdc_hostfile_close(outfile);
    } else {
        LOG_ERROR("Failure to open sh4_profile.txt for writing\n");
    }
    jit_profile_ctxt_cleanup(&sh4->jit_profile);
#endif
}

#ifdef JIT_PROFILE
static washdc_hostfile jit_profile_out;

static void
sh4_jit_profile_disas(washdc_hostfile out, uint32_t addr, void const *instp) {
    jit_profile_out = out;

    uint16_t inst;
    memcpy(&inst, instp, sizeof(inst));
    sh4asm_disas_inst(inst, sh4_jit_profile_emit_fn, addr);
    jit_profile_out = NULL;
}

static void sh4_jit_profile_emit_fn(char ch) {
    washdc_hostfile_putc(jit_profile_out, ch);
}
#endif

static void
res_drain_reg(Sh4 *sh4, struct sh4_jit_compile_ctx *ctx,
              struct il_code_block *block, unsigned reg_no) {
    struct residency *res = reg_map + reg_no;
    if (res->stat == REG_STATUS_SLOT) {
        unsigned regbase_slot = get_regbase_slot(sh4, ctx, block);
        switch (block->slots[res->slot_no].tp) {
        case WASHDC_JIT_SLOT_GEN:
            jit_store_slot_offset(block, res->slot_no, regbase_slot, reg_no);
            break;
        case WASHDC_JIT_SLOT_FLOAT:
            jit_store_float_slot_offset(block, res->slot_no,
                                        regbase_slot, reg_no);
            break;
        default:
            RAISE_ERROR(ERROR_UNIMPLEMENTED);
        }
        res->stat = REG_STATUS_SLOT_AND_SH4;
    }
}

// this function emits il ops to move all data in slots into registers.
static void
res_drain_all_regs(Sh4 *sh4, struct sh4_jit_compile_ctx *ctx,
                   struct il_code_block *block) {
    unsigned reg_no;
    for (reg_no = 0; reg_no < SH4_REGISTER_COUNT; reg_no++)
        res_drain_reg(sh4, ctx, block, reg_no);
}

/*
 * mark the given register as REG_STATUS_SH4.
 * This does not write it back to the reg array.
 */
static void res_invalidate_reg(struct il_code_block *block, unsigned reg_no) {
    struct residency *res = reg_map + reg_no;
    if (res->stat != REG_STATUS_SH4) {
        res->stat = REG_STATUS_SH4;
        free_slot(block, res->slot_no);
    }
}

/*
 * mark all registers as REG_STATUS_SH4.
 * This does not write them back to the reg array.
 */
static void res_invalidate_all_regs(struct il_code_block *block) {
    unsigned reg_no;
    for (reg_no = 0; reg_no < SH4_REGISTER_COUNT; reg_no++)
        if (reg_map[reg_no].stat != REG_STATUS_SH4)
            res_invalidate_reg(block, reg_no);
}

void sh4_jit_new_block(void) {
    unsigned reg_no;
    for (reg_no = 0; reg_no < SH4_REGISTER_COUNT; reg_no++) {
        reg_map[reg_no].slot_no = -1;
        reg_map[reg_no].stat = REG_STATUS_SH4;
        reg_map[reg_no].last_read = 0;
        reg_map[reg_no].last_write = 0;
    }
}

static void
sh4_jit_delay_slot(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                   struct il_code_block *block, unsigned pc) {
    cpu_inst_param inst = memory_map_read_16(sh4->mem.map, pc & BIT_RANGE(0, 28));
    struct InstOpcode const *inst_op = sh4_decode_inst(inst);
    if (inst_op->pc_relative) {
        error_set_feature("illegal slot exceptions in the jit");
        error_set_address(pc);
        RAISE_ERROR(ERROR_UNIMPLEMENTED);
    }

#ifdef JIT_PROFILE
        uint16_t inst16 = inst;
        jit_profile_push_inst(&sh4->jit_profile, block->profile, &inst16);
#endif

    ctx->in_delay_slot = true;

    if (!inst_op->disas(sh4, ctx, block, pc, inst_op, inst)) {
        /*
         * in theory, this will never happen because only branch instructions
         * can return true, and those all should have been filtered out by the
         * pc_relative check above.
         */
        LOG_ERROR("inst is 0x%04x\n", (unsigned)inst);
        RAISE_ERROR(ERROR_INTEGRITY);
    }
    ctx->in_delay_slot = false;
    unsigned old_cycle_count = ctx->cycle_count;
    ctx->cycle_count += sh4_count_inst_cycles(inst_op,
                                              &ctx->last_inst_type);
    if (old_cycle_count > ctx->cycle_count)
        LOG_ERROR("*** JIT DETECTED CYCLE COUNT OVERFLOW ***\n");
}

bool
sh4_jit_compile_inst(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                     struct il_code_block *block, cpu_inst_param inst,
                     unsigned pc) {
    if (sh4_ocache_in_ram_area(pc)) {
        /*
         * the jit hashes code by discarding the top three bits of the PC so
         * that those can be used for FPSCR flags.
         * This means that the Operand Cache's RAM area will overlap with
         * normal memory.  If this error is ever tripped, then an alternative
         * hashing algorithm will be required.
         *
         * AFAIK, this is the only region of on-chip memory where it's plausible
         * to think that executable code could be stored.  Even so, I'm not sure
         * if it's actually possible.
         */
        LOG_ERROR("**** ATTEMPT TO EXECUTE CODE FROM SH4 OPERAND CACHE ****\n");
        RAISE_ERROR(ERROR_UNIMPLEMENTED);
    }

    struct InstOpcode const *inst_op = sh4_decode_inst(inst);

    unsigned old_cycle_count = ctx->cycle_count;
    ctx->cycle_count += sh4_count_inst_cycles(inst_op,
                                              &ctx->last_inst_type);
    if (old_cycle_count > ctx->cycle_count)
        LOG_ERROR("*** JIT DETECTED CYCLE COUNT OVERFLOW ***\n");

    return inst_op->disas(sh4, ctx, block, pc, inst_op, inst);
}

bool
sh4_jit_fallback(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                 struct il_code_block *block, unsigned pc,
                 struct InstOpcode const *op, cpu_inst_param inst) {
    struct jit_inst il_inst;

    res_drain_all_regs(sh4, ctx, block);
    res_invalidate_all_regs(block);

    il_inst.op = JIT_OP_FALLBACK;
    il_inst.immed.fallback.fallback_fn = op->func;
    il_inst.immed.fallback.inst = inst;

    il_code_block_push_inst(block, &il_inst);

    return true;
}

bool sh4_jit_rts(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                 struct il_code_block *block, unsigned pc,
                 struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned jmp_addr_slot = reg_slot(sh4, ctx, block, SH4_REG_PR,
                                      WASHDC_JIT_SLOT_GEN);
    res_disassociate_reg(sh4, ctx, block, SH4_REG_PR);

    sh4_jit_delay_slot(sh4, ctx, block, pc + 2);

    unsigned hash_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    if (ctx->dirty_fpscr) {
        unsigned fpscr_slot = reg_slot(sh4, ctx, block, SH4_REG_FPSCR,
                                       WASHDC_JIT_SLOT_GEN);
        sh4_jit_hash_slot(sh4, block, jmp_addr_slot, hash_slot, fpscr_slot);
        free_slot(block, fpscr_slot);
    } else {
        sh4_jit_hash_slot_known_fpscr(sh4, ctx, block, jmp_addr_slot, hash_slot);
    }

    res_drain_all_regs(sh4, ctx, block);
    jit_jump(block, jmp_addr_slot, hash_slot);

    free_slot(block, hash_slot);
    free_slot(block, jmp_addr_slot);

    return false;
}

bool sh4_jit_rte(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                 struct il_code_block *block, unsigned pc,
                 struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned jmp_addr_slot = reg_slot(sh4, ctx, block, SH4_REG_SPC,
                                      WASHDC_JIT_SLOT_GEN);
    res_disassociate_reg(sh4, ctx, block, SH4_REG_SPC);

    /*
     * there are a few different ways editing the SR can cause side-effects (for
     * example by initiating a bank-switch) so we need to make sure everything
     * is committed to the reg array and we also need to make sure we reload any
     * registers referenced after the jit_restore_sr operation.
     */
    res_drain_all_regs(sh4, ctx, block);
    res_invalidate_all_regs(block);

    jit_call_func(block, sh4_jit_set_sr, reg_slot(sh4, ctx, block, SH4_REG_SSR,
                                                  WASHDC_JIT_SLOT_GEN));

    sh4_jit_delay_slot(sh4, ctx, block, pc + 2);

    unsigned hash_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    if (ctx->dirty_fpscr) {
        unsigned fpscr_slot = reg_slot(sh4, ctx, block, SH4_REG_FPSCR,
                                       WASHDC_JIT_SLOT_GEN);
        sh4_jit_hash_slot(sh4, block, jmp_addr_slot, hash_slot, fpscr_slot);
        free_slot(block, fpscr_slot);
    } else {
        sh4_jit_hash_slot_known_fpscr(sh4, ctx, block, jmp_addr_slot, hash_slot);
    }

    res_drain_all_regs(sh4, ctx, block);
    jit_jump(block, jmp_addr_slot, hash_slot);

    free_slot(block, hash_slot);
    free_slot(block, jmp_addr_slot);

    return false;
}

bool sh4_jit_braf_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                     struct il_code_block *block, unsigned pc,
                     struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_no = (inst >> 8) & 0xf;
    unsigned jump_offs = pc + 4;

    unsigned jmp_addr_slot = reg_slot(sh4, ctx, block, SH4_REG_R0 + reg_no,
                                      WASHDC_JIT_SLOT_GEN);
    res_disassociate_reg(sh4, ctx, block, SH4_REG_R0 + reg_no);
    jit_add_const32(block, jmp_addr_slot, jump_offs);

    sh4_jit_delay_slot(sh4, ctx, block, pc + 2);

    unsigned hash_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    if (ctx->dirty_fpscr) {
        unsigned fpscr_slot = reg_slot(sh4, ctx, block, SH4_REG_FPSCR,
                                       WASHDC_JIT_SLOT_GEN);
        sh4_jit_hash_slot(sh4, block, jmp_addr_slot, hash_slot, fpscr_slot);
        free_slot(block, fpscr_slot);
    } else {
        sh4_jit_hash_slot_known_fpscr(sh4, ctx, block, jmp_addr_slot, hash_slot);
    }

    res_drain_all_regs(sh4, ctx, block);
    jit_jump(block, jmp_addr_slot, hash_slot);

    free_slot(block, hash_slot);
    free_slot(block, jmp_addr_slot);

    return false;
}

bool sh4_jit_bsrf_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                     struct il_code_block *block, unsigned pc,
                     struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_no = (inst >> 8) & 0xf;
    unsigned jump_offs = pc + 4;

    unsigned addr_slot_no = reg_slot(sh4, ctx, block, SH4_REG_R0 + reg_no,
                                     WASHDC_JIT_SLOT_GEN);
    res_disassociate_reg(sh4, ctx, block, SH4_REG_R0 + reg_no);
    jit_add_const32(block, addr_slot_no, jump_offs);

    unsigned pr_slot_no = reg_slot_noload(sh4, block, SH4_REG_PR);
    jit_set_slot(block, pr_slot_no, pc + 4);

    sh4_jit_delay_slot(sh4, ctx, block, pc + 2);

    unsigned hash_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    if (ctx->dirty_fpscr) {
        unsigned fpscr_slot = reg_slot(sh4, ctx, block, SH4_REG_FPSCR,
                                       WASHDC_JIT_SLOT_GEN);
        sh4_jit_hash_slot(sh4, block, addr_slot_no, hash_slot, fpscr_slot);
        free_slot(block, fpscr_slot);
    } else {
        sh4_jit_hash_slot_known_fpscr(sh4, ctx, block, addr_slot_no, hash_slot);
    }

    res_drain_all_regs(sh4, ctx, block);
    jit_jump(block, addr_slot_no, hash_slot);

    free_slot(block, hash_slot);
    free_slot(block, addr_slot_no);

    return false;
}

bool sh4_jit_bf(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                struct il_code_block *block, unsigned pc,
                struct InstOpcode const *op, cpu_inst_param inst) {
    int jump_offs = (int)((int8_t)(inst & 0x00ff)) * 2 + 4;

    unsigned flag_slot = reg_slot(sh4, ctx, block, SH4_REG_SR,
                                  WASHDC_JIT_SLOT_GEN);
    res_disassociate_reg(sh4, ctx, block, SH4_REG_SR);

    unsigned jmp_addr_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    jit_set_slot(block, jmp_addr_slot, pc + jump_offs);

    jit_cset(block, flag_slot, 1, pc+2, jmp_addr_slot);
    unsigned hash_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    if (ctx->dirty_fpscr) {
        unsigned fpscr_slot = reg_slot(sh4, ctx, block, SH4_REG_FPSCR,
                                       WASHDC_JIT_SLOT_GEN);
        sh4_jit_hash_slot(sh4, block, jmp_addr_slot, hash_slot, fpscr_slot);
        free_slot(block, fpscr_slot);
    } else {
        sh4_jit_hash_slot_known_fpscr(sh4, ctx, block, jmp_addr_slot, hash_slot);
    }

    res_drain_all_regs(sh4, ctx, block);
    jit_jump(block, jmp_addr_slot, hash_slot);

    free_slot(block, hash_slot);
    free_slot(block, jmp_addr_slot);
    free_slot(block, flag_slot);

    return false;
}

bool sh4_jit_bt(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                struct il_code_block *block, unsigned pc,
                struct InstOpcode const *op, cpu_inst_param inst) {
    int jump_offs = (int)((int8_t)(inst & 0x00ff)) * 2 + 4;

    unsigned flag_slot =
        reg_slot(sh4, ctx, block, SH4_REG_SR, WASHDC_JIT_SLOT_GEN);
    res_disassociate_reg(sh4, ctx, block, SH4_REG_SR);

    unsigned jmp_addr_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    jit_set_slot(block, jmp_addr_slot, pc + jump_offs);

    jit_cset(block, flag_slot, 0, pc + 2, jmp_addr_slot);
    unsigned hash_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    if (ctx->dirty_fpscr) {
        unsigned fpscr_slot = reg_slot(sh4, ctx, block, SH4_REG_FPSCR,
                                       WASHDC_JIT_SLOT_GEN);
        sh4_jit_hash_slot(sh4, block, jmp_addr_slot, hash_slot, fpscr_slot);
        free_slot(block, fpscr_slot);
    } else {
        sh4_jit_hash_slot_known_fpscr(sh4, ctx, block, jmp_addr_slot, hash_slot);
    }

    res_drain_all_regs(sh4, ctx, block);
    jit_jump(block, jmp_addr_slot, hash_slot);

    free_slot(block, hash_slot);
    free_slot(block, jmp_addr_slot);
    free_slot(block, flag_slot);

    return false;
}

bool sh4_jit_bfs(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                 struct il_code_block *block, unsigned pc,
                 struct InstOpcode const *op, cpu_inst_param inst) {
    int jump_offs = (int)((int8_t)(inst & 0x00ff)) * 2 + 4;

    unsigned flag_slot = reg_slot(sh4, ctx, block, SH4_REG_SR,
                                  WASHDC_JIT_SLOT_GEN);
    res_disassociate_reg(sh4, ctx, block, SH4_REG_SR);

    sh4_jit_delay_slot(sh4, ctx, block, pc + 2);

    unsigned jmp_addr_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    jit_set_slot(block, jmp_addr_slot, pc + jump_offs);

    jit_cset(block, flag_slot, 1, pc + 4, jmp_addr_slot);
    unsigned hash_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    if (ctx->dirty_fpscr) {
        unsigned fpscr_slot = reg_slot(sh4, ctx, block, SH4_REG_FPSCR,
                                       WASHDC_JIT_SLOT_GEN);
        sh4_jit_hash_slot(sh4, block, jmp_addr_slot, hash_slot, fpscr_slot);
        free_slot(block, fpscr_slot);
    } else {
        sh4_jit_hash_slot_known_fpscr(sh4, ctx, block, jmp_addr_slot, hash_slot);
    }

    res_drain_all_regs(sh4, ctx, block);
    jit_jump(block, jmp_addr_slot, hash_slot);

    free_slot(block, hash_slot);
    free_slot(block, jmp_addr_slot);
    free_slot(block, flag_slot);

    return false;
}

bool sh4_jit_bts(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                 struct il_code_block *block, unsigned pc,
                 struct InstOpcode const *op, cpu_inst_param inst) {
    int jump_offs = (int)((int8_t)(inst & 0x00ff)) * 2 + 4;

    unsigned flag_slot = reg_slot(sh4, ctx, block, SH4_REG_SR,
                                  WASHDC_JIT_SLOT_GEN);
    res_disassociate_reg(sh4, ctx, block, SH4_REG_SR);

    sh4_jit_delay_slot(sh4, ctx, block, pc + 2);

    unsigned jmp_addr_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    jit_set_slot(block, jmp_addr_slot, pc + jump_offs);

    jit_cset(block, flag_slot, 0, pc + 4, jmp_addr_slot);
    unsigned hash_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    if (ctx->dirty_fpscr) {
        unsigned fpscr_slot = reg_slot(sh4, ctx, block, SH4_REG_FPSCR,
                                       WASHDC_JIT_SLOT_GEN);
        sh4_jit_hash_slot(sh4, block, jmp_addr_slot, hash_slot, fpscr_slot);
        free_slot(block, fpscr_slot);
    } else {
        sh4_jit_hash_slot_known_fpscr(sh4, ctx, block, jmp_addr_slot, hash_slot);
    }

    res_drain_all_regs(sh4, ctx, block);
    jit_jump(block, jmp_addr_slot, hash_slot);

    free_slot(block, hash_slot);
    free_slot(block, jmp_addr_slot);
    free_slot(block, flag_slot);

    return false;
}

bool sh4_jit_bra(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                 struct il_code_block *block, unsigned pc,
                 struct InstOpcode const *op, cpu_inst_param inst) {
    int32_t disp = inst & 0x0fff;
    if (disp & 0x0800)
        disp |= 0xfffff000;
    disp = disp * 2 + 4;

    sh4_jit_delay_slot(sh4, ctx, block, pc + 2);

    unsigned addr_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    jit_set_slot(block, addr_slot, pc + disp);

    unsigned hash_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    if (ctx->dirty_fpscr) {
        unsigned fpscr_slot = reg_slot(sh4, ctx, block, SH4_REG_FPSCR,
                                       WASHDC_JIT_SLOT_GEN);
        sh4_jit_hash_slot(sh4, block, addr_slot, hash_slot, fpscr_slot);
        free_slot(block, fpscr_slot);
    } else {
        sh4_jit_hash_slot_known_fpscr(sh4, ctx, block, addr_slot, hash_slot);
    }

    res_drain_all_regs(sh4, ctx, block);
    jit_jump(block, addr_slot, hash_slot);

    free_slot(block, hash_slot);
    free_slot(block, addr_slot);

    return false;
}

bool sh4_jit_bsr(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                 struct il_code_block *block, unsigned pc,
                 struct InstOpcode const *op, cpu_inst_param inst) {
    int32_t disp = inst & 0x0fff;
    if (disp & 0x0800)
        disp |= 0xfffff000;
    disp = disp * 2 + 4;

    unsigned pr_slot_no = reg_slot_noload(sh4, block, SH4_REG_PR);
    jit_set_slot(block, pr_slot_no, pc + 4);

    sh4_jit_delay_slot(sh4, ctx, block, pc + 2);

    unsigned addr_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    jit_set_slot(block, addr_slot, pc + disp);

    unsigned hash_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    if (ctx->dirty_fpscr) {
        unsigned fpscr_slot = reg_slot(sh4, ctx, block, SH4_REG_FPSCR,
                                       WASHDC_JIT_SLOT_GEN);
        sh4_jit_hash_slot(sh4, block, addr_slot, hash_slot, fpscr_slot);
        free_slot(block, fpscr_slot);
    } else {
        sh4_jit_hash_slot_known_fpscr(sh4, ctx, block, addr_slot, hash_slot);
    }

    res_drain_all_regs(sh4, ctx, block);
    jit_jump(block, addr_slot, hash_slot);

    free_slot(block, hash_slot);
    free_slot(block, addr_slot);

    return false;
}

bool sh4_jit_jmp_arn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                     struct il_code_block *block, unsigned pc,
                     struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_no = (inst >> 8) & 0xf;

    unsigned jmp_addr_slot = reg_slot(sh4, ctx, block, SH4_REG_R0 + reg_no,
                                      WASHDC_JIT_SLOT_GEN);
    res_disassociate_reg(sh4, ctx, block, SH4_REG_R0 + reg_no);

    sh4_jit_delay_slot(sh4, ctx, block, pc + 2);

    unsigned hash_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    if (ctx->dirty_fpscr) {
        unsigned fpscr_slot = reg_slot(sh4, ctx, block, SH4_REG_FPSCR,
                                       WASHDC_JIT_SLOT_GEN);
        sh4_jit_hash_slot(sh4, block, jmp_addr_slot, hash_slot, fpscr_slot);
        free_slot(block, fpscr_slot);
    } else {
        sh4_jit_hash_slot_known_fpscr(sh4, ctx, block, jmp_addr_slot, hash_slot);
    }

    res_drain_all_regs(sh4, ctx, block);
    jit_jump(block, jmp_addr_slot, hash_slot);

    free_slot(block, hash_slot);
    free_slot(block, jmp_addr_slot);

    return false;
}

bool sh4_jit_jsr_arn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                     struct il_code_block *block, unsigned pc,
                     struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_no = (inst >> 8) & 0xf;

    unsigned addr_slot_no = reg_slot(sh4, ctx, block, SH4_REG_R0 + reg_no,
                                     WASHDC_JIT_SLOT_GEN);
    res_disassociate_reg(sh4, ctx, block, SH4_REG_R0 + reg_no);

    unsigned pr_slot_no = reg_slot_noload(sh4, block, SH4_REG_PR);
    jit_set_slot(block, pr_slot_no, pc + 4);

    sh4_jit_delay_slot(sh4, ctx, block, pc + 2);

    unsigned hash_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    if (ctx->dirty_fpscr) {
        unsigned fpscr_slot = reg_slot(sh4, ctx, block, SH4_REG_FPSCR,
                                       WASHDC_JIT_SLOT_GEN);
        sh4_jit_hash_slot(sh4, block, addr_slot_no, hash_slot, fpscr_slot);
        free_slot(block, fpscr_slot);
    } else {
        sh4_jit_hash_slot_known_fpscr(sh4, ctx, block, addr_slot_no, hash_slot);
    }

    res_drain_all_regs(sh4, ctx, block);
    jit_jump(block, addr_slot_no, hash_slot);

    free_slot(block, hash_slot);
    free_slot(block, addr_slot_no);

    return false;
}

// disassembles the "mov.w @(disp, pc), rn" instruction
bool
sh4_jit_movw_a_disp_pc_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_no = ((inst >> 8) & 0xf) + SH4_REG_R0;
    unsigned disp = inst & 0xff;
    addr32_t addr = disp * 2 + pc + 4;

    unsigned slot_no = reg_slot_noload(sh4, block, reg_no);

    jit_mem_read_constaddr_16(sh4->mem.map, block, addr, slot_no);

    jit_sign_extend_16(block, slot_no);

    return true;
}

// disassembles the "mov.l @(disp, pc), rn" instruction
bool
sh4_jit_movl_a_disp_pc_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_no = (inst >> 8) & 0xf;
    unsigned disp = inst & 0xff;
    addr32_t addr = disp * 4 + (pc & ~3) + 4;

    unsigned slot_no = reg_slot_noload(sh4, block, reg_no);
    jit_mem_read_constaddr_32(sh4->mem.map, block, addr, slot_no);

    return true;
}

bool
sh4_jit_mova_a_disp_pc_r0(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned disp = inst & 0xff;
    addr32_t addr = disp * 4 + (pc & ~3) + 4;

    unsigned slot_no = reg_slot_noload(sh4, block, SH4_REG_R0);
    jit_set_slot(block, slot_no, addr);

    return true;
}

bool
sh4_jit_nop(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
            struct il_code_block *block, unsigned pc,
            struct InstOpcode const *op, cpu_inst_param inst) {
    return true;
}

bool sh4_jit_ocbi_arn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                      struct il_code_block *block, unsigned pc,
                      struct InstOpcode const *op, cpu_inst_param inst) {
    return true;
}

bool sh4_jit_ocbp_arn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                      struct il_code_block *block, unsigned pc,
                      struct InstOpcode const *op, cpu_inst_param inst) {
    return true;
}

bool sh4_jit_ocbwb_arn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                       struct il_code_block *block, unsigned pc,
                       struct InstOpcode const *op, cpu_inst_param inst) {
    return true;
}

// ADD Rm, Rn
// 0011nnnnmmmm1100
bool sh4_jit_add_rm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                       struct il_code_block *block, unsigned pc,
                       struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = (inst & 0x00f0) >> 4;
    unsigned reg_dst = (inst & 0x0f00) >> 8;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);

    jit_add(block, slot_src, slot_dst);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;

    return true;
}

// ADD #imm, Rn
// 0111nnnniiiiiiii
bool sh4_jit_add_imm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                        struct il_code_block *block, unsigned pc,
                        struct InstOpcode const *op, cpu_inst_param inst) {
    int32_t imm_val = (int32_t)(int8_t)(inst & 0xff);
    unsigned reg_dst = (inst & 0x0f00) >> 8;

    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);

    jit_add_const32(block, slot_dst, imm_val);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;

    return true;
}

// XOR Rm, Rn
// 0010nnnnmmmm1010
bool sh4_jit_xor_rm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                       struct il_code_block *block, unsigned pc,
                       struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = (inst & 0x00f0) >> 4;
    unsigned reg_dst = (inst & 0x0f00) >> 8;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);

    jit_xor(block, slot_src, slot_dst);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;

    return true;
}

// MOV Rm, Rn
// 0110nnnnmmmm0011
bool sh4_jit_mov_rm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                       struct il_code_block *block, unsigned pc,
                       struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = (inst & 0x00f0) >> 4;
    unsigned reg_dst = (inst & 0x0f00) >> 8;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);

    jit_mov(block, slot_src, slot_dst);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;

    return true;
}

// AND Rm, Rn
// 0010nnnnmmmm1001
bool sh4_jit_and_rm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                       struct il_code_block *block, unsigned pc,
                       struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = (inst & 0x00f0) >> 4;
    unsigned reg_dst = (inst & 0x0f00) >> 8;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);

    jit_and(block, slot_src, slot_dst);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;

    return true;
}

// OR Rm, Rn
// 0010nnnnmmmm1011
bool sh4_jit_or_rm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                      struct il_code_block *block, unsigned pc,
                      struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = (inst & 0x00f0) >> 4;
    unsigned reg_dst = (inst & 0x0f00) >> 8;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);

    jit_or(block, slot_src, slot_dst);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;

    return true;
}

// SUB Rm, Rn
// 0011nnnnmmmm1000
bool sh4_jit_sub_rm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                       struct il_code_block *block, unsigned pc,
                       struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = (inst & 0x00f0) >> 4;
    unsigned reg_dst = (inst & 0x0f00) >> 8;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);

    jit_sub(block, slot_src, slot_dst);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;

    return true;
}

// AND #imm, R0
// 11001001iiiiiiii
bool
sh4_inst_binary_andb_imm_r0(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                            struct il_code_block *block, unsigned pc,
                            struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned imm_val = inst & 0xff;
    unsigned slot_no = reg_slot(sh4, ctx, block, SH4_REG_R0, WASHDC_JIT_SLOT_GEN);

    jit_and_const32(block, slot_no, imm_val);
    reg_map[SH4_REG_R0].stat = REG_STATUS_SLOT;

    return true;
}

// OR #imm, R0
// 11001011iiiiiiii
bool sh4_jit_or_imm8_r0(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                        struct il_code_block *block, unsigned pc,
                        struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned imm_val = inst & 0xff;
    unsigned slot_no = reg_slot(sh4, ctx, block, SH4_REG_R0, WASHDC_JIT_SLOT_GEN);

    jit_or_const32(block, slot_no, imm_val);
    reg_map[SH4_REG_R0].stat = REG_STATUS_SLOT;

    return true;
}

// XOR #imm, R0
// 11001010iiiiiiii
bool sh4_jit_xor_imm8_r0(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned imm_val = inst & 0xff;
    unsigned slot_no = reg_slot(sh4, ctx, block, SH4_REG_R0, WASHDC_JIT_SLOT_GEN);

    jit_xor_const32(block, slot_no, imm_val);

    reg_map[SH4_REG_R0].stat = REG_STATUS_SLOT;

    return true;
}

// TST Rm, Rn
// 0010nnnnmmmm1000
bool sh4_jit_tst_rm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                       struct il_code_block *block, unsigned pc,
                       struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_dst = ((inst & 0x0f00) >> 8) + SH4_REG_R0;
    unsigned reg_src = ((inst & 0x00f0) >> 4) + SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);
    unsigned slot_sr = reg_slot(sh4, ctx, block, SH4_REG_SR, WASHDC_JIT_SLOT_GEN);

    res_disassociate_reg(sh4, ctx, block, reg_dst);
    jit_and(block, slot_src, slot_dst);

    jit_slot_to_bool_inv(block, slot_dst);

    jit_and_const32(block, slot_sr, ~1);
    jit_or(block, slot_dst, slot_sr);

    reg_map[SH4_REG_SR].stat = REG_STATUS_SLOT;

    free_slot(block, slot_dst);

    return true;
}

// TST #imm, R0
// 11001000iiiiiiii
bool sh4_jit_tst_imm8_r0(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned slot_r0 = reg_slot(sh4, ctx, block, SH4_REG_R0, WASHDC_JIT_SLOT_GEN);
    unsigned slot_sr = reg_slot(sh4, ctx, block, SH4_REG_SR, WASHDC_JIT_SLOT_GEN);

    res_disassociate_reg(sh4, ctx, block, SH4_REG_R0);
    jit_and_const32(block, slot_r0, inst & 0xff);

    jit_slot_to_bool_inv(block, slot_r0);

    jit_and_const32(block, slot_sr, ~1);
    jit_or(block, slot_r0, slot_sr);

    reg_map[SH4_REG_SR].stat = REG_STATUS_SLOT;

    free_slot(block, slot_r0);

    return true;
}

// MOV.B @(R0, Rm), Rn
// 0000nnnnmmmm1100
bool sh4_jit_movb_a_r0_rm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                             struct il_code_block *block, unsigned pc,
                             struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = ((inst & 0x00f0) >> 4) + SH4_REG_R0;
    unsigned reg_dst = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);
    unsigned slot_r0 = reg_slot(sh4, ctx, block, SH4_REG_R0, WASHDC_JIT_SLOT_GEN);

    unsigned slot_srcaddr = alloc_slot(block, WASHDC_JIT_SLOT_GEN);

    jit_mov(block, slot_src, slot_srcaddr);
    jit_add(block, slot_r0, slot_srcaddr);

    jit_read_8_slot(block, sh4->mem.map, slot_srcaddr, slot_dst);
    jit_sign_extend_8(block, slot_dst);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;

    free_slot(block, slot_srcaddr);

    return true;
}

// MOV.L @(R0, Rm), Rn
// 0000nnnnmmmm1110
bool sh4_jit_movl_a_r0_rm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                             struct il_code_block *block, unsigned pc,
                             struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = ((inst & 0x00f0) >> 4) + SH4_REG_R0;
    unsigned reg_dst = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);
    unsigned slot_r0 = reg_slot(sh4, ctx, block, SH4_REG_R0, WASHDC_JIT_SLOT_GEN);

    unsigned slot_srcaddr = alloc_slot(block, WASHDC_JIT_SLOT_GEN);

    jit_mov(block, slot_src, slot_srcaddr);
    jit_add(block, slot_r0, slot_srcaddr);

    jit_read_32_slot(block, sh4->mem.map, slot_srcaddr, slot_dst);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;

    free_slot(block, slot_srcaddr);

    return true;
}

// MOV.L Rm, @(disp, Rn)
// 0001nnnnmmmmdddd
bool
sh4_jit_movl_rm_a_disp4_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                           struct il_code_block *block, unsigned pc,
                           struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned disp4 = (inst & 0xf) << 2;
    unsigned reg_src = ((inst & 0x00f0) >> 4) + SH4_REG_R0;
    unsigned reg_dst = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);

    unsigned slot_dstaddr = alloc_slot(block, WASHDC_JIT_SLOT_GEN);

    jit_mov(block, slot_dst, slot_dstaddr);
    jit_add_const32(block, slot_dstaddr, disp4);

    jit_write_32_slot(block, sh4->mem.map, slot_src, slot_dstaddr);

    free_slot(block, slot_dstaddr);

    return true;
}

// MOV.B @Rm, Rn
// 0110nnnnmmmm0000
bool sh4_jit_movb_arm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = ((inst & 0x00f0) >> 4) + SH4_REG_R0;
    unsigned reg_dst = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);

    jit_read_8_slot(block, sh4->mem.map, slot_src, slot_dst);
    jit_sign_extend_8(block, slot_dst);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;

    return true;
}

// MOV.L @Rm, Rn
// 0110nnnnmmmm0010
bool sh4_jit_movl_arm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = ((inst & 0x00f0) >> 4) + SH4_REG_R0;
    unsigned reg_dst = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);

    jit_read_32_slot(block, sh4->mem.map, slot_src, slot_dst);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;

    return true;
}

// MOV.L Rm, @Rn
// 0010nnnnmmmm0010
bool sh4_jit_movl_rm_arn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = ((inst & 0x00f0) >> 4) + SH4_REG_R0;
    unsigned reg_dst = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);

    jit_write_32_slot(block, sh4->mem.map, slot_src, slot_dst);

    reg_map[reg_src].stat = REG_STATUS_SLOT;

    return true;
}

// MOV.L @(disp, Rm), Rn
// 0101nnnnmmmmdddd
bool
sh4_jit_movl_a_disp4_rm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                           struct il_code_block *block, unsigned pc,
                           struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned disp = (inst & 0xf) << 2;
    unsigned reg_src = ((inst & 0x00f0) >> 4) + SH4_REG_R0;
    unsigned reg_dst = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    res_disassociate_reg(sh4, ctx, block, reg_src);
    jit_add_const32(block, slot_src, disp);

    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);

    jit_read_32_slot(block, sh4->mem.map, slot_src, slot_dst);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;

    free_slot(block, slot_src);

    return true;
}

// MOV.L @(disp, GBR), R0
// 11000110dddddddd
bool
sh4_jit_movl_a_disp8_gbr_r0(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                            struct il_code_block *block, unsigned pc,
                            struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned disp = (inst & 0xff) << 2;
    unsigned reg_src = SH4_REG_GBR;
    unsigned reg_dst = SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    res_disassociate_reg(sh4, ctx, block, reg_src);
    jit_add_const32(block, slot_src, disp);

    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);

    jit_read_32_slot(block, sh4->mem.map, slot_src, slot_dst);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;

    free_slot(block, slot_src);

    return true;
}

// MOV.B @Rm+, Rn
// 0110nnnnmmmm0100
bool sh4_jit_movb_armp_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = ((inst >> 4) & 0xf) + SH4_REG_R0;
    unsigned reg_dst = ((inst >> 8) & 0xf) + SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);

    jit_read_8_slot(block, sh4->mem.map, slot_src, slot_dst);
    jit_sign_extend_8(block, slot_dst);
    if (reg_src != reg_dst)
        jit_add_const32(block, slot_src, 1);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;
    reg_map[reg_src].stat = REG_STATUS_SLOT;

    return true;
}

// MOV.W Rm, @-Rn
// 0010nnnnmmmm0101
bool sh4_jit_movw_rm_amrn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = ((inst >> 4) & 0xf) + SH4_REG_R0;
    unsigned reg_dst = ((inst >> 8) & 0xf) + SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);

    jit_add_const32(block, slot_dst, -2);
    jit_write_16_slot(block, sh4->mem.map, slot_src, slot_dst);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;

    // TODO: is this necessary given that reg_src did not change?
    reg_map[reg_src].stat = REG_STATUS_SLOT;

    return true;
}

// MOV.W @Rm+, Rn
// 0110nnnnmmmm0101
bool sh4_jit_movw_armp_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = ((inst >> 4) & 0xf) + SH4_REG_R0;
    unsigned reg_dst = ((inst >> 8) & 0xf) + SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);

    jit_read_16_slot(block, sh4->mem.map, slot_src, slot_dst);
    jit_sign_extend_16(block, slot_dst);
    if (reg_src != reg_dst)
        jit_add_const32(block, slot_src, 2);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;
    reg_map[reg_src].stat = REG_STATUS_SLOT;

    return true;
}

// MOV.L @Rm+, Rn
// 0110nnnnmmmm0110
bool sh4_jit_movl_armp_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = ((inst & 0x00f0) >> 4) + SH4_REG_R0;
    unsigned reg_dst = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);

    jit_read_32_slot(block, sh4->mem.map, slot_src, slot_dst);
    if (reg_src != reg_dst)
        jit_add_const32(block, slot_src, 4);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;
    reg_map[reg_src].stat = REG_STATUS_SLOT;

    return true;
}

// MOV.B Rm, @-Rn
// 0010nnnnmmmm0100
bool sh4_jit_movb_rm_amrn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = ((inst & 0x00f0) >> 4) + SH4_REG_R0;
    unsigned reg_dst = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);

    jit_add_const32(block, slot_dst, -1);
    jit_write_8_slot(block, sh4->mem.map, slot_src, slot_dst);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;

    // TODO: is this necessary given that reg_src did not change?
    reg_map[reg_src].stat = REG_STATUS_SLOT;

    return true;
}

// MOV.B Rm, @Rn
// 0010nnnnmmmm0000
bool sh4_jit_movb_rm_arn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = ((inst & 0x00f0) >> 4) + SH4_REG_R0;
    unsigned reg_dst = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);

    jit_write_8_slot(block, sh4->mem.map, slot_src, slot_dst);

    return true;
}

// MOV.L Rm, @-Rn
// 0010nnnnmmmm0110
bool sh4_jit_movl_rm_amrn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = ((inst & 0x00f0) >> 4) + SH4_REG_R0;
    unsigned reg_dst = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);

    jit_add_const32(block, slot_dst, -4);
    jit_write_32_slot(block, sh4->mem.map, slot_src, slot_dst);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;
    reg_map[reg_src].stat = REG_STATUS_SLOT;

    return true;
}

// LDS.L @Rm+, PR
// 0100mmmm00100110
bool sh4_jit_ldsl_armp_pr(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned addr_reg = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned addr_slot = reg_slot(sh4, ctx, block, addr_reg, WASHDC_JIT_SLOT_GEN);
    unsigned pr_slot = reg_slot(sh4, ctx, block, SH4_REG_PR, WASHDC_JIT_SLOT_GEN);

    jit_read_32_slot(block, sh4->mem.map, addr_slot, pr_slot);
    jit_add_const32(block, addr_slot, 4);

    reg_map[SH4_REG_PR].stat = REG_STATUS_SLOT;
    reg_map[addr_reg].stat = REG_STATUS_SLOT;

    return true;
}

// MOV #imm, Rn
// 1110nnnniiiiiiii
bool sh4_jit_mov_imm8_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst) {
    int32_t imm32 = (int32_t)((int8_t)(inst & 0xff));
    unsigned dst_reg = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned dst_slot = reg_slot_noload(sh4, block, dst_reg);
    jit_set_slot(block, dst_slot, imm32);

    reg_map[dst_reg].stat = REG_STATUS_SLOT;

    return true;
}

// SHLL16 Rn
// 0100nnnn00101000
bool sh4_jit_shll16_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                       struct il_code_block *block, unsigned pc,
                       struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_no = ((inst & 0x0f00) >> 8) + SH4_REG_R0;
    unsigned slot_no = reg_slot(sh4, ctx, block, reg_no, WASHDC_JIT_SLOT_GEN);
    jit_shll(block, slot_no, 16);

    reg_map[reg_no].stat = REG_STATUS_SLOT;

    return true;
}

// SHLL2 Rn
// 0100nnnn00001000
bool sh4_jit_shll2_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                      struct il_code_block *block, unsigned pc,
                      struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_no = ((inst & 0x0f00) >> 8) + SH4_REG_R0;
    unsigned slot_no = reg_slot(sh4, ctx, block, reg_no, WASHDC_JIT_SLOT_GEN);
    jit_shll(block, slot_no, 2);

    reg_map[reg_no].stat = REG_STATUS_SLOT;

    return true;
}

// SHLL8 Rn
// 0100nnnn00011000
bool sh4_jit_shll8_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                      struct il_code_block *block, unsigned pc,
                      struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_no = ((inst & 0x0f00) >> 8) + SH4_REG_R0;
    unsigned slot_no = reg_slot(sh4, ctx, block, reg_no, WASHDC_JIT_SLOT_GEN);
    jit_shll(block, slot_no, 8);

    reg_map[reg_no].stat = REG_STATUS_SLOT;

    return true;
}

// SHAR Rn
// 0100nnnn00100001
bool sh4_jit_shar_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                     struct il_code_block *block, unsigned pc,
                     struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_no = ((inst & 0x0f00) >> 8) + SH4_REG_R0;
    unsigned slot_no = reg_slot(sh4, ctx, block, reg_no, WASHDC_JIT_SLOT_GEN);
    unsigned tmp_cpy = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    unsigned sr_slot = reg_slot(sh4, ctx, block, SH4_REG_SR, WASHDC_JIT_SLOT_GEN);

    // set the T-bit in SR from the shift-out.a
    jit_mov(block, slot_no, tmp_cpy);
    jit_and_const32(block, tmp_cpy, 1);
    jit_and_const32(block, sr_slot, ~1);
    jit_or(block, tmp_cpy, sr_slot);
    reg_map[SH4_REG_SR].stat = REG_STATUS_SLOT;

    free_slot(block, tmp_cpy);

    jit_shar(block, slot_no, 1);

    reg_map[reg_no].stat = REG_STATUS_SLOT;

    return true;
}

// SHLR Rn
// 0100nnnn00000001
bool sh4_jit_shlr_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                     struct il_code_block *block, unsigned pc,
                     struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_no = ((inst & 0x0f00) >> 8) + SH4_REG_R0;
    unsigned slot_no = reg_slot(sh4, ctx, block, reg_no, WASHDC_JIT_SLOT_GEN);
    unsigned tmp_cpy = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    unsigned sr_slot = reg_slot(sh4, ctx, block, SH4_REG_SR, WASHDC_JIT_SLOT_GEN);

    // set the T-bit in SR from the shift-out.a
    jit_mov(block, slot_no, tmp_cpy);
    jit_and_const32(block, tmp_cpy, 1);
    jit_and_const32(block, sr_slot, ~1);
    jit_or(block, tmp_cpy, sr_slot);
    reg_map[SH4_REG_SR].stat = REG_STATUS_SLOT;

    free_slot(block, tmp_cpy);

    jit_shlr(block, slot_no, 1);

    reg_map[reg_no].stat = REG_STATUS_SLOT;

    return true;
}

// SHLL Rn
// 0100nnnn00000000
bool sh4_jit_shll_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                     struct il_code_block *block, unsigned pc,
                     struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_no = ((inst & 0x0f00) >> 8) + SH4_REG_R0;
    unsigned slot_no = reg_slot(sh4, ctx, block, reg_no, WASHDC_JIT_SLOT_GEN);
    unsigned tmp_cpy = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    unsigned sr_slot = reg_slot(sh4, ctx, block, SH4_REG_SR, WASHDC_JIT_SLOT_GEN);

    // set the T-bit in SR from the shift-out.
    jit_mov(block, slot_no, tmp_cpy);
    jit_and_const32(block, tmp_cpy, 1<<31);
    jit_shlr(block, tmp_cpy, 31);
    jit_and_const32(block, sr_slot, ~1);
    jit_or(block, tmp_cpy, sr_slot);
    reg_map[SH4_REG_SR].stat = REG_STATUS_SLOT;

    free_slot(block, tmp_cpy);

    jit_shll(block, slot_no, 1);

    reg_map[reg_no].stat = REG_STATUS_SLOT;

    return true;
}

// SHAL Rn
// 0100nnnn00100000
bool sh4_jit_shal_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                     struct il_code_block *block, unsigned pc,
                     struct InstOpcode const *op, cpu_inst_param inst) {
    // As far as I know, SHLL and SHAL do the exact same thing.
    return sh4_jit_shll_rn(sh4, ctx, block, pc, op, inst);
}


// SHAD Rm, Rn
// 0100nnnnmmmm1100
bool sh4_jit_shad_rm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                        struct il_code_block *block, unsigned pc,
                        struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = (inst & 0x00f0) >> 4;
    unsigned reg_dst = (inst & 0x0f00) >> 8;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);

    jit_shad(block, slot_dst, slot_src);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;

    return true;
}

// SHLR2 Rn
// 0100nnnn00001001
bool sh4_jit_shlr2_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                      struct il_code_block *block, unsigned pc,
                      struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_no = ((inst & 0x0f00) >> 8) + SH4_REG_R0;
    unsigned slot_no = reg_slot(sh4, ctx, block, reg_no, WASHDC_JIT_SLOT_GEN);
    jit_shlr(block, slot_no, 2);

    reg_map[reg_no].stat = REG_STATUS_SLOT;

    return true;
}

// SHLR8 Rn
// 0100nnnn00011001
bool sh4_jit_shlr8_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                      struct il_code_block *block, unsigned pc,
                      struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_no = ((inst & 0x0f00) >> 8) + SH4_REG_R0;
    unsigned slot_no = reg_slot(sh4, ctx, block, reg_no, WASHDC_JIT_SLOT_GEN);
    jit_shlr(block, slot_no, 8);

    reg_map[reg_no].stat = REG_STATUS_SLOT;

    return true;
}

// SHLR16 Rn
// 0100nnnn00101001
bool sh4_jit_shlr16_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                       struct il_code_block *block, unsigned pc,
                       struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_no = ((inst & 0x0f00) >> 8) + SH4_REG_R0;
    unsigned slot_no = reg_slot(sh4, ctx, block, reg_no, WASHDC_JIT_SLOT_GEN);
    jit_shlr(block, slot_no, 16);

    reg_map[reg_no].stat = REG_STATUS_SLOT;

    return true;
}

// SWAP.W Rm, Rn
// 0110nnnnmmmm1001
bool sh4_jit_swapw_rm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = ((inst & 0x00f0) >> 4) + SH4_REG_R0;
    unsigned reg_dst = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot_noload(sh4, block, reg_dst);

    unsigned slot_tmp = alloc_slot(block, WASHDC_JIT_SLOT_GEN);

    jit_mov(block, slot_src, slot_tmp);
    jit_shlr(block, slot_tmp, 16);

    jit_mov(block, slot_src, slot_dst);
    jit_and_const32(block, slot_dst, 0xffff);
    jit_shll(block, slot_dst, 16);

    jit_or(block, slot_tmp, slot_dst);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;

    free_slot(block, slot_tmp);

    return true;
}

// CMP/HI Rm, Rn
// 0011nnnnmmmm0110
bool sh4_jit_cmphi_rm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = ((inst & 0x00f0) >> 4) + SH4_REG_R0;
    unsigned reg_dst = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);
    unsigned slot_sr = reg_slot(sh4, ctx, block, SH4_REG_SR, WASHDC_JIT_SLOT_GEN);

    jit_and_const32(block, slot_sr, ~1);
    jit_set_gt_unsigned(block, slot_dst, slot_src, slot_sr);

    reg_map[SH4_REG_SR].stat = REG_STATUS_SLOT;

    return true;
}

// CMP/GT Rm, Rn
// 0011nnnnmmmm0111
bool sh4_jit_cmpgt_rm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = ((inst & 0x00f0) >> 4) + SH4_REG_R0;
    unsigned reg_dst = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);
    unsigned slot_sr = reg_slot(sh4, ctx, block, SH4_REG_SR, WASHDC_JIT_SLOT_GEN);

    jit_and_const32(block, slot_sr, ~1);
    jit_set_gt_signed(block, slot_dst, slot_src, slot_sr);

    reg_map[SH4_REG_SR].stat = REG_STATUS_SLOT;

    return true;
}

// CMP/EQ Rm, Rn
// 0011nnnnmmmm0000
bool sh4_jit_cmpeq_rm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = ((inst & 0x00f0) >> 4) + SH4_REG_R0;
    unsigned reg_dst = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);
    unsigned slot_sr = reg_slot(sh4, ctx, block, SH4_REG_SR, WASHDC_JIT_SLOT_GEN);

    jit_and_const32(block, slot_sr, ~1);
    jit_set_eq(block, slot_dst, slot_src, slot_sr);

    reg_map[SH4_REG_SR].stat = REG_STATUS_SLOT;

    return true;
}

// CMP/HS Rm, Rn
// 0011nnnnmmmm0010
bool sh4_jit_cmphs_rm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = ((inst & 0x00f0) >> 4) + SH4_REG_R0;
    unsigned reg_dst = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);
    unsigned slot_sr = reg_slot(sh4, ctx, block, SH4_REG_SR, WASHDC_JIT_SLOT_GEN);

    jit_and_const32(block, slot_sr, ~1);
    jit_set_ge_unsigned(block, slot_dst, slot_src, slot_sr);

    reg_map[SH4_REG_SR].stat = REG_STATUS_SLOT;

    return true;
}

// MULU.W Rm, Rn
// 0010nnnnmmmm1110
bool sh4_jit_muluw_rm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_lhs = ((inst & 0x00f0) >> 4) + SH4_REG_R0;
    unsigned reg_rhs = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_lhs = reg_slot(sh4, ctx, block, reg_lhs, WASHDC_JIT_SLOT_GEN);
    unsigned slot_rhs = reg_slot(sh4, ctx, block, reg_rhs, WASHDC_JIT_SLOT_GEN);
    unsigned slot_macl = reg_slot(sh4, ctx, block, SH4_REG_MACL, WASHDC_JIT_SLOT_GEN);

    unsigned slot_lhs_16 = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    unsigned slot_rhs_16 = alloc_slot(block, WASHDC_JIT_SLOT_GEN);

    /*
     * TODO: x86 has instructions that can move and zero-extend at the same
     * time, and that would probably be faster than moving plus AND'ing.  I'd
     * have to add new IL op for that, which is why I'm doing it the naive way
     * for now.
     */
    jit_mov(block, slot_lhs, slot_lhs_16);
    jit_mov(block, slot_rhs, slot_rhs_16);
    jit_and_const32(block, slot_lhs_16, 0xffff);
    jit_and_const32(block, slot_rhs_16, 0xffff);

    jit_mul_u32(block, slot_lhs_16, slot_rhs_16, slot_macl);

    reg_map[SH4_REG_MACL].stat = REG_STATUS_SLOT;

    free_slot(block, slot_rhs_16);
    free_slot(block, slot_lhs_16);

    return true;
}

// STS MACL, Rn
// 0000nnnn00011010
bool sh4_jit_sts_macl_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_dst = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);
    unsigned slot_macl = reg_slot(sh4, ctx, block, SH4_REG_MACL,
                                  WASHDC_JIT_SLOT_GEN);

    jit_mov(block, slot_macl, slot_dst);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;

    return true;
}

// CMP/GE Rm, Rn
// 0011nnnnmmmm0011
bool sh4_jit_cmpge_rm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                         struct il_code_block *block, unsigned pc,
                         struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = ((inst & 0x00f0) >> 4) + SH4_REG_R0;
    unsigned reg_dst = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);
    unsigned slot_sr = reg_slot(sh4, ctx, block, SH4_REG_SR, WASHDC_JIT_SLOT_GEN);

    jit_and_const32(block, slot_sr, ~1);
    jit_set_ge_signed(block, slot_dst, slot_src, slot_sr);

    reg_map[SH4_REG_SR].stat = REG_STATUS_SLOT;

    return true;
}

// CMP/PL Rn
// 0100nnnn00010101
bool sh4_jit_cmppl_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                      struct il_code_block *block, unsigned pc,
                      struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_lhs = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_lhs = reg_slot(sh4, ctx, block, reg_lhs, WASHDC_JIT_SLOT_GEN);
    unsigned slot_sr = reg_slot(sh4, ctx, block, SH4_REG_SR, WASHDC_JIT_SLOT_GEN);

    jit_and_const32(block, slot_sr, ~1);
    jit_set_gt_signed_const(block, slot_lhs, 0, slot_sr);

    reg_map[SH4_REG_SR].stat = REG_STATUS_SLOT;

    return true;
}

// CMP/PZ Rn
// 0100nnnn00010001
bool sh4_jit_cmppz_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                      struct il_code_block *block, unsigned pc,
                      struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_lhs = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_lhs = reg_slot(sh4, ctx, block, reg_lhs, WASHDC_JIT_SLOT_GEN);
    unsigned slot_sr = reg_slot(sh4, ctx, block, SH4_REG_SR, WASHDC_JIT_SLOT_GEN);

    jit_and_const32(block, slot_sr, ~1);
    jit_set_ge_signed_const(block, slot_lhs, 0, slot_sr);

    reg_map[SH4_REG_SR].stat = REG_STATUS_SLOT;

    return true;
}

// NOT Rm, Rn
// 0110nnnnmmmm0111
bool sh4_jit_not_rm_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                       struct il_code_block *block, unsigned pc,
                       struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = ((inst & 0x00f0) >> 4) + SH4_REG_R0;
    unsigned reg_dst = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    unsigned slot_dst = reg_slot(sh4, ctx, block, reg_dst, WASHDC_JIT_SLOT_GEN);

    jit_mov(block, slot_src, slot_dst);
    jit_not(block, slot_dst);

    reg_map[reg_dst].stat = REG_STATUS_SLOT;

    return true;
}

// DT Rn
// 0100nnnn00010000
bool sh4_jit_dt_rn(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                   struct il_code_block *block, unsigned pc,
                   struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_no = ((inst & 0x0f00) >> 8) + SH4_REG_R0;
    unsigned slot_no = reg_slot(sh4, ctx, block, reg_no, WASHDC_JIT_SLOT_GEN);
    unsigned sr_slot = reg_slot(sh4, ctx, block, SH4_REG_SR, WASHDC_JIT_SLOT_GEN);
    unsigned tmp_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);

    jit_and_const32(block, sr_slot, ~1);
    jit_add_const32(block, slot_no, ~(uint32_t)0);
    jit_mov(block, slot_no, tmp_slot);
    jit_slot_to_bool_inv(block, tmp_slot);
    jit_or(block, tmp_slot, sr_slot);

    reg_map[reg_no].stat = REG_STATUS_SLOT;
    reg_map[SH4_REG_SR].stat = REG_STATUS_SLOT;

    return true;
}

// CLRT
// 0000000000001000
bool sh4_jit_clrt(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                  struct il_code_block *block, unsigned pc,
                  struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned sr_slot = reg_slot(sh4, ctx, block, SH4_REG_SR, WASHDC_JIT_SLOT_GEN);

    jit_and_const32(block, sr_slot, ~1);

    reg_map[SH4_REG_SR].stat = REG_STATUS_SLOT;

    return true;
}

// SETT
// 0000000000011000
bool sh4_jit_sett(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                  struct il_code_block *block, unsigned pc,
                  struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned sr_slot = reg_slot(sh4, ctx, block, SH4_REG_SR, WASHDC_JIT_SLOT_GEN);

    jit_or_const32(block, sr_slot, 1);

    reg_map[SH4_REG_SR].stat = REG_STATUS_SLOT;

    return true;
}

// MOVT Rn
// 0000nnnn00101001
bool sh4_jit_movt(Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                  struct il_code_block *block, unsigned pc,
                  struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_no = ((inst & 0x0f00) >> 8) + SH4_REG_R0;

    unsigned slot_no = reg_slot(sh4, ctx, block, reg_no, WASHDC_JIT_SLOT_GEN);
    unsigned sr_slot = reg_slot(sh4, ctx, block, SH4_REG_SR, WASHDC_JIT_SLOT_GEN);

    jit_mov(block, sr_slot, slot_no);
    jit_and_const32(block, slot_no, 1);

    reg_map[reg_no].stat = REG_STATUS_SLOT;

    return true;
}

// STS.L PR, @-Rn
// 0100nnnn00100010
bool
sh4_jit_stsl_pr_amrn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                     struct il_code_block *block, unsigned pc,
                     struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned addr_reg = ((inst >> 8) & 0xf) + SH4_REG_R0;
    unsigned addr_slot = reg_slot(sh4, ctx, block, addr_reg, WASHDC_JIT_SLOT_GEN);
    unsigned pr_slot = reg_slot(sh4, ctx, block, SH4_REG_PR, WASHDC_JIT_SLOT_GEN);

    jit_add_const32(block, addr_slot, -4);
    jit_write_32_slot(block, sh4->mem.map, pr_slot, addr_slot);

    reg_map[addr_reg].stat = REG_STATUS_SLOT;
    reg_map[SH4_REG_PR].stat = REG_STATUS_SLOT;

    return true;
}

// EXTU.B Rm, Rn
// 0110nnnnmmmm1100
bool
sh4_jit_extub_rm_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                    struct il_code_block *block, unsigned pc,
                    struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned src_reg = ((inst >> 4) & 0xf) + SH4_REG_R0;
    unsigned dst_reg = ((inst >> 8) & 0xf) + SH4_REG_R0;

    unsigned src_slot = reg_slot(sh4, ctx, block, src_reg, WASHDC_JIT_SLOT_GEN);
    unsigned dst_slot = reg_slot(sh4, ctx, block, dst_reg, WASHDC_JIT_SLOT_GEN);

    jit_mov(block, src_slot, dst_slot);
    jit_and_const32(block, dst_slot, 0xff);

    reg_map[dst_reg].stat = REG_STATUS_SLOT;

    return true;
}

// TRAPA #immed
// 11000011iiiiiiii
bool sh4_jit_trapa_imm(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                       struct il_code_block *block, unsigned pc,
                       struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned tra_slot = reg_slot(sh4, ctx, block,
                                 SH4_REG_TRA, WASHDC_JIT_SLOT_GEN);
    jit_set_slot(block, tra_slot, (inst & BIT_RANGE(0, 7)) << 2);
    reg_map[SH4_REG_TRA].stat = REG_STATUS_SLOT;

    /*
     * set SH4_REG_PC to point to the next instruction after this.
     *
     * sh4_set_exception will use this to initialize SPC so it has to be
     * the correct value.
     */
    unsigned pc_slot = reg_slot(sh4, ctx, block,
                                SH4_REG_PC, WASHDC_JIT_SLOT_GEN);
    jit_set_slot(block, pc_slot, pc + 2);
    reg_map[SH4_REG_PC].stat = REG_STATUS_SLOT;

    /* calling sh4_set_exception will change the sr */
    res_drain_all_regs(sh4, ctx, block);
    res_invalidate_all_regs(block);
    jit_call_func_imm32(block, sh4_set_exception,
                        SH4_EXCP_UNCONDITIONAL_TRAP);

    // jump to new PC which was set by sh4_set_exception
    unsigned hash_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    pc_slot = reg_slot(sh4, ctx, block,
                       SH4_REG_PC, WASHDC_JIT_SLOT_GEN);
    unsigned fpscr_slot = reg_slot(sh4, ctx, block, SH4_REG_FPSCR,
                                   WASHDC_JIT_SLOT_GEN);

    sh4_jit_hash_slot(sh4, block, pc_slot, hash_slot, fpscr_slot);
    free_slot(block, fpscr_slot);

    res_drain_all_regs(sh4, ctx, block);
    jit_jump(block, pc_slot, hash_slot);

    free_slot(block, hash_slot);
    free_slot(block, pc_slot);

    return false;
}

// LDS Rm, FPSCR
// 0100mmmm01101010
bool sh4_jit_lds_rm_fpscr(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned reg_src = ((inst & 0x0f00) >> 8) + SH4_REG_R0;
    unsigned slot_src = reg_slot(sh4, ctx, block, reg_src, WASHDC_JIT_SLOT_GEN);
    res_disassociate_reg(sh4, ctx, block, reg_src);

    /*
     * there are a few different ways editing the FPSCR can cause side-effects (for
     * example by initiating a bank-switch) so we need to make sure everything
     * is committed to the reg array and we also need to make sure we reload any
     * registers referenced after the jit_restore_sr operation.
     */
    res_drain_all_regs(sh4, ctx, block);
    res_invalidate_all_regs(block);

    jit_call_func(block, sh4_set_fpscr, slot_src);

    free_slot(block, slot_src);

    ctx->dirty_fpscr = true;
    if (!ctx->in_delay_slot) {
        /*
         * since the code hash (which includes SZ and PR) just changed, we need to
         * handle this like a jmp instruction.
         */
        unsigned jmp_addr_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
        jit_set_slot(block, jmp_addr_slot, pc + 2);

        unsigned hash_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
        unsigned fpscr_slot = reg_slot(sh4, ctx, block, SH4_REG_FPSCR,
                                       WASHDC_JIT_SLOT_GEN);
        sh4_jit_hash_slot(sh4, block, jmp_addr_slot, hash_slot, fpscr_slot);

        res_drain_all_regs(sh4, ctx, block);
        jit_jump(block, jmp_addr_slot, hash_slot);

        free_slot(block, hash_slot);
        free_slot(block, jmp_addr_slot);

        return false;
    }

    return true;
}

// LDS.L @Rm+, FPSCR
// 0100mmmm01100110
bool sh4_jit_ldsl_armp_fpscr(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                             struct il_code_block *block, unsigned pc,
                             struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned addr_reg = ((inst & 0x0f00) >> 8) + SH4_REG_R0;
    unsigned addr_slot = reg_slot(sh4, ctx, block, addr_reg, WASHDC_JIT_SLOT_GEN);
    unsigned new_val_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);

    jit_read_32_slot(block, sh4->mem.map, addr_slot, new_val_slot);
    jit_add_const32(block, addr_slot, 4);
    reg_map[addr_reg].stat = REG_STATUS_SLOT;

    /*
     * there are a few different ways editing the FPSCR can cause side-effects (for
     * example by initiating a bank-switch) so we need to make sure everything
     * is committed to the reg array and we also need to make sure we reload any
     * registers referenced after the jit_restore_sr operation.
     */
    res_drain_all_regs(sh4, ctx, block);
    res_invalidate_all_regs(block);

    jit_call_func(block, sh4_set_fpscr, new_val_slot);

    free_slot(block, new_val_slot);

    ctx->dirty_fpscr = true;
    if (!ctx->in_delay_slot) {
        /*
         * since the code hash (which includes SZ and PR) just changed, we need to
         * handle this like a jmp instruction.
         */
        unsigned jmp_addr_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
        jit_set_slot(block, jmp_addr_slot, pc + 2);

        unsigned hash_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
        unsigned fpscr_slot = reg_slot(sh4, ctx, block, SH4_REG_FPSCR,
                                       WASHDC_JIT_SLOT_GEN);
        sh4_jit_hash_slot(sh4, block, jmp_addr_slot, hash_slot, fpscr_slot);

        res_drain_all_regs(sh4, ctx, block);
        jit_jump(block, jmp_addr_slot, hash_slot);

        free_slot(block, hash_slot);
        free_slot(block, jmp_addr_slot);

        return false;
    }

    return true;
}

// FSCHG
// 1111001111111101
bool sh4_jit_fschg(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                   struct il_code_block *block, unsigned pc,
                   struct InstOpcode const *op, cpu_inst_param inst) {
    unsigned new_val_slot = reg_slot(sh4, ctx, block, SH4_REG_FPSCR,
                                     WASHDC_JIT_SLOT_GEN);
    res_disassociate_reg(sh4, ctx, block, SH4_REG_FPSCR);

    jit_xor_const32(block, new_val_slot, SH4_FPSCR_SZ_MASK);
    ctx->sz_bit = !ctx->sz_bit;

    /*
     * there are a few different ways editing the FPSCR can cause side-effects (for
     * example by initiating a bank-switch) so we need to make sure everything
     * is committed to the reg array and we also need to make sure we reload any
     * registers referenced after the jit_restore_sr operation.
     */
    res_drain_all_regs(sh4, ctx, block);
    res_invalidate_all_regs(block);

    jit_call_func(block, sh4_set_fpscr, new_val_slot);

    free_slot(block, new_val_slot);

    ctx->dirty_fpscr = true;
    if (!ctx->in_delay_slot) {
        /*
         * since the code hash (which includes SZ and PR) just changed, we need to
         * handle this like a jmp instruction.
         */
        unsigned jmp_addr_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
        jit_set_slot(block, jmp_addr_slot, pc + 2);

        unsigned hash_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
        unsigned fpscr_slot = reg_slot(sh4, ctx, block, SH4_REG_FPSCR,
                                       WASHDC_JIT_SLOT_GEN);
        sh4_jit_hash_slot(sh4, block, jmp_addr_slot, hash_slot, fpscr_slot);
        free_slot(block, fpscr_slot);

        res_drain_all_regs(sh4, ctx, block);
        jit_jump(block, jmp_addr_slot, hash_slot);

        free_slot(block, hash_slot);
        free_slot(block, jmp_addr_slot);

        return false;
    }

    return true;
}

// FMOV.S FRm, @(R0, Rn)
// 1111nnnnmmmm0111
// FMOV DRm, @(R0, Rn)
// 1111nnnnmmm00111
// FMOV XDm, @(R0, Rn)
// 1111nnnnmmm10111
bool sh4_jit_fmov_fpu_a_r0_rn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                              struct il_code_block *block, unsigned pc,
                              struct InstOpcode const *op, cpu_inst_param inst) {
    void (*handler)(void*, cpu_inst_param);

    if (ctx->sz_bit) {
        if (inst & (1 << 4))
            handler = sh4_inst_binary_fmov_xs_binind_r0_gen;
        else
            handler = sh4_inst_binary_fmov_dr_binind_r0_gen;
    } else {
        // single-precistion mode
        handler = sh4_inst_binary_fmovs_fr_binind_r0_gen;
    }

    struct jit_inst il_inst;

    res_drain_all_regs(sh4, ctx, block);
    res_invalidate_all_regs(block);

    il_inst.op = JIT_OP_FALLBACK;
    il_inst.immed.fallback.fallback_fn = handler;
    il_inst.immed.fallback.inst = inst;

    il_code_block_push_inst(block, &il_inst);

    return true;
}

// FMOV.S @Rm+, FRn
// 1111nnnnmmmm1001
// FMOV @Rm+, DRn
// 1111nnn0mmmm1001
// FMOV @Rm+, XDn
// 1111nnn1mmmm1001
bool sh4_jit_fmov_fpu_armp_fpu(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                               struct il_code_block *block, unsigned pc,
                               struct InstOpcode const *op, cpu_inst_param inst) {
   void (*handler)(void*, cpu_inst_param);

   if (ctx->sz_bit) {
       if (inst & (1 << 8))
           handler = sh4_inst_binary_fmov_indgeninc_xd;
       else
           handler = sh4_inst_binary_fmov_indgeninc_dr;
   } else {
       unsigned addr_reg = ((inst & 0x00f0) >> 4) + SH4_REG_R0;
       unsigned dst_reg = ((inst >> 8) & 0xf) + SH4_REG_FR0;

       unsigned slot_addr = reg_slot(sh4, ctx, block, addr_reg, WASHDC_JIT_SLOT_GEN);
       unsigned slot_dst = reg_slot(sh4, ctx, block, dst_reg,
                                    WASHDC_JIT_SLOT_FLOAT);

       jit_read_float_slot(block, sh4->mem.map, slot_addr, slot_dst);
       jit_add_const32(block, slot_addr, 4);

       reg_map[addr_reg].stat = REG_STATUS_SLOT;
       reg_map[dst_reg].stat = REG_STATUS_SLOT;

       return true;
   }

    struct jit_inst il_inst;

    res_drain_all_regs(sh4, ctx, block);
    res_invalidate_all_regs(block);

    il_inst.op = JIT_OP_FALLBACK;
    il_inst.immed.fallback.fallback_fn = handler;
    il_inst.immed.fallback.inst = inst;

    il_code_block_push_inst(block, &il_inst);

    return true;
}

// FMOV.S @Rm, FRn
// 1111nnnnmmmm1000
// FMOV @Rm, DRn
// 1111nnn0mmmm1000
// FMOV @Rm, XDn
// 1111nnn1mmmm1000
bool sh4_jit_fmov_arm_fpu(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst) {
    void (*handler)(void*, cpu_inst_param);

   if (ctx->sz_bit) {
       if (inst & (1 << 8))
           handler = sh4_inst_binary_fmov_indgen_xd;
       else
           handler = sh4_inst_binary_fmov_indgen_dr;
   } else {
       unsigned addr_reg = ((inst >> 4) & 0xf) + SH4_REG_R0;
       unsigned fr_dst_reg = ((inst >> 8) & 0xf) + SH4_REG_FR0;

       unsigned slot_addr = reg_slot(sh4, ctx, block, addr_reg, WASHDC_JIT_SLOT_GEN);
       unsigned slot_dst = reg_slot(sh4, ctx, block, fr_dst_reg,
                                    WASHDC_JIT_SLOT_FLOAT);

       jit_read_float_slot(block, sh4->mem.map, slot_addr, slot_dst);
       reg_map[fr_dst_reg].stat = REG_STATUS_SLOT;

       return true;
   }

    struct jit_inst il_inst;

    res_drain_all_regs(sh4, ctx, block);
    res_invalidate_all_regs(block);

    il_inst.op = JIT_OP_FALLBACK;
    il_inst.immed.fallback.fallback_fn = handler;
    il_inst.immed.fallback.inst = inst;

    il_code_block_push_inst(block, &il_inst);

    return true;
}

// FMOV.S FRm, @-Rn
// 1111nnnnmmmm1011
// FMOV DRm, @-Rn
// 1111nnnnmmm01011
// FMOV XDm, @-Rn
// 1111nnnnmmm11011
bool sh4_jit_fmov_fpu_amrn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                           struct il_code_block *block, unsigned pc,
                           struct InstOpcode const *op, cpu_inst_param inst) {
    void (*handler)(void*, cpu_inst_param);

   if (ctx->sz_bit) {
       if (inst & (1 << 4))
           handler = sh4_inst_binary_fmov_xd_inddecgen;
       else
           handler = sh4_inst_binary_fmov_dr_inddecgen;
   } else {
       unsigned addr_reg = ((inst & 0x0f00) >> 8) + SH4_REG_R0;
       unsigned src_reg = ((inst >> 4) & 0xf) + SH4_REG_FR0;

       unsigned slot_addr = reg_slot(sh4, ctx, block, addr_reg, WASHDC_JIT_SLOT_GEN);
       unsigned slot_src = reg_slot(sh4, ctx, block, src_reg,
                                    WASHDC_JIT_SLOT_FLOAT);

       jit_add_const32(block, slot_addr, -4);
       jit_write_float_slot(block, sh4->mem.map, slot_src, slot_addr);

       reg_map[addr_reg].stat = REG_STATUS_SLOT;
       return true;
   }

    struct jit_inst il_inst;

    res_drain_all_regs(sh4, ctx, block);
    res_invalidate_all_regs(block);

    il_inst.op = JIT_OP_FALLBACK;
    il_inst.immed.fallback.fallback_fn = handler;
    il_inst.immed.fallback.inst = inst;

    il_code_block_push_inst(block, &il_inst);

    return true;
}

// FMUL FRm, FRn
// 1111nnnnmmmm0010
// FMUL DRm, DRn
// 1111nnn0mmm00010
bool sh4_jit_fmul_frm_frn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst) {
    struct jit_inst il_inst;

    if (ctx->pr_bit) {
        res_drain_all_regs(sh4, ctx, block);
        res_invalidate_all_regs(block);

        il_inst.op = JIT_OP_FALLBACK;
        il_inst.immed.fallback.fallback_fn = sh4_inst_binary_fmul_dr_dr;
        il_inst.immed.fallback.inst = inst;

        il_code_block_push_inst(block, &il_inst);
    } else {
        unsigned fr_src_reg = ((inst >> 4) & 0xf) + SH4_REG_FR0;
        unsigned fr_dst_reg = ((inst >> 8) & 0xf) + SH4_REG_FR0;

        unsigned fr_src_slot =
            reg_slot(sh4, ctx, block, fr_src_reg, WASHDC_JIT_SLOT_FLOAT);
        unsigned fr_dst_slot =
            reg_slot(sh4, ctx, block, fr_dst_reg, WASHDC_JIT_SLOT_FLOAT);

        jit_mul_float(block, fr_src_slot, fr_dst_slot);

        reg_map[fr_dst_reg].stat = REG_STATUS_SLOT;
    }

    return true;
}

// FCMP/GT FRm, FRn
// 1111nnnnmmmm0101
// FCMP/GT DRm, DRn
// 1111nnn0mmm00101
bool sh4_jit_fcmpgt_frm_frn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                            struct il_code_block *block, unsigned pc,
                            struct InstOpcode const *op, cpu_inst_param inst) {
    void (*handler)(void*, cpu_inst_param);

    if (ctx->pr_bit)
        handler = sh4_inst_binary_fcmpgt_dr_dr;
    else
        handler = sh4_inst_binary_fcmpgt_fr_fr;

    struct jit_inst il_inst;

    res_drain_all_regs(sh4, ctx, block);
    res_invalidate_all_regs(block);

    il_inst.op = JIT_OP_FALLBACK;
    il_inst.immed.fallback.fallback_fn = handler;
    il_inst.immed.fallback.inst = inst;

    il_code_block_push_inst(block, &il_inst);

    return true;
}

// FSUB FRm, FRn
// 1111nnnnmmmm0001
// FSUB DRm, DRn
// 1111nnn0mmm00001
bool sh4_jit_fsub_frm_frn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst) {
    struct jit_inst il_inst;

    if (ctx->pr_bit) {
        res_drain_all_regs(sh4, ctx, block);
        res_invalidate_all_regs(block);

        il_inst.op = JIT_OP_FALLBACK;
        il_inst.immed.fallback.fallback_fn = sh4_inst_binary_fsub_dr_dr;
        il_inst.immed.fallback.inst = inst;

        il_code_block_push_inst(block, &il_inst);
    } else {
        unsigned fr_src_reg = ((inst >> 4) & 0xf) + SH4_REG_FR0;
        unsigned fr_dst_reg = ((inst >> 8) & 0xf) + SH4_REG_FR0;

        unsigned fr_src_slot =
            reg_slot(sh4, ctx, block, fr_src_reg, WASHDC_JIT_SLOT_FLOAT);
        unsigned fr_dst_slot =
            reg_slot(sh4, ctx, block, fr_dst_reg, WASHDC_JIT_SLOT_FLOAT);

        jit_sub_float(block, fr_src_slot, fr_dst_slot);

        reg_map[fr_dst_reg].stat = REG_STATUS_SLOT;
    }

    return true;
}

// FADD FRm, FRn
// 1111nnnnmmmm0000
// FADD DRm, DRn
// 1111nnn0mmm00000
bool sh4_jit_fadd_frm_frn(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                          struct il_code_block *block, unsigned pc,
                          struct InstOpcode const *op, cpu_inst_param inst) {
    void (*handler)(void*, cpu_inst_param);

    struct jit_inst il_inst;

    if (ctx->pr_bit) {
        handler = sh4_inst_binary_fadd_dr_dr;

        res_drain_all_regs(sh4, ctx, block);
        res_invalidate_all_regs(block);

        il_inst.op = JIT_OP_FALLBACK;
        il_inst.immed.fallback.fallback_fn = handler;
        il_inst.immed.fallback.inst = inst;

        il_code_block_push_inst(block, &il_inst);
    } else {
        unsigned fr_src_reg = ((inst >> 4) & 0xf) + SH4_REG_FR0;
        unsigned fr_dst_reg = ((inst >> 8) & 0xf) + SH4_REG_FR0;

        unsigned fr_src_slot =
            reg_slot(sh4, ctx, block, fr_src_reg, WASHDC_JIT_SLOT_FLOAT);
        unsigned fr_dst_slot =
            reg_slot(sh4, ctx, block, fr_dst_reg, WASHDC_JIT_SLOT_FLOAT);

        jit_add_float(block, fr_src_slot, fr_dst_slot);

        reg_map[fr_dst_reg].stat = REG_STATUS_SLOT;
    }

    return true;
}

// FTRC FRm, FPUL
// 1111mmmm00111101
// FTRC DRm, FPUL
// 1111mmm000111101
bool sh4_jit_ftrc_frm_fpul(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                           struct il_code_block *block, unsigned pc,
                           struct InstOpcode const *op, cpu_inst_param inst) {
    void (*handler)(void*, cpu_inst_param);

    if (ctx->pr_bit)
        handler = sh4_inst_binary_ftrc_dr_fpul;
    else
        handler = sh4_inst_binary_ftrc_fr_fpul;

    struct jit_inst il_inst;

    res_drain_all_regs(sh4, ctx, block);
    res_invalidate_all_regs(block);

    il_inst.op = JIT_OP_FALLBACK;
    il_inst.immed.fallback.fallback_fn = handler;
    il_inst.immed.fallback.inst = inst;

    il_code_block_push_inst(block, &il_inst);

    return true;
}


// FMOV.S @(R0, Rm), FRn
// 1111nnnnmmmm0110
// FMOV @(R0, Rm), DRn
// 1111nnn0mmmm0110
// FMOV @(R0, Rm), XDn
// 1111nnn1mmmm0110
bool sh4_jit_fmovs_a_r0_rm_fpu(struct Sh4 *sh4, struct sh4_jit_compile_ctx* ctx,
                               struct il_code_block *block, unsigned pc,
                               struct InstOpcode const *op, cpu_inst_param inst) {
    void (*handler)(void*, cpu_inst_param);

    if (ctx->sz_bit) {
        if (inst & (1 << 8))
            handler = sh4_inst_binary_fmov_binind_r0_gen_xd;
        else
            handler = sh4_inst_binary_fmov_binind_r0_gen_dr;
    } else {
        unsigned offs_reg = ((inst >> 4) & 0xf) + SH4_REG_R0;
        unsigned fr_dst_reg = ((inst >> 8) & 0xf) + SH4_REG_FR0;

        unsigned offs_slot = reg_slot(sh4, ctx, block, offs_reg,
                                      WASHDC_JIT_SLOT_GEN);
        unsigned r0_slot = reg_slot(sh4, ctx, block, SH4_REG_R0,
                                    WASHDC_JIT_SLOT_GEN);
        unsigned slot_dst = reg_slot(sh4, ctx, block, fr_dst_reg,
                                     WASHDC_JIT_SLOT_FLOAT);
        unsigned addr_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);

        jit_mov(block, offs_slot, addr_slot);
        jit_add(block, r0_slot, addr_slot);
        jit_read_float_slot(block, sh4->mem.map, addr_slot, slot_dst);

        reg_map[fr_dst_reg].stat = REG_STATUS_SLOT;
        free_slot(block, addr_slot);

        return true;
    }

    struct jit_inst il_inst;

    res_drain_all_regs(sh4, ctx, block);
    res_invalidate_all_regs(block);

    il_inst.op = JIT_OP_FALLBACK;
    il_inst.immed.fallback.fallback_fn = handler;
    il_inst.immed.fallback.inst = inst;

    il_code_block_push_inst(block, &il_inst);

    return true;
}

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
                          struct InstOpcode const *op, cpu_inst_param inst) {
    void (*handler)(void*, cpu_inst_param);

    if (ctx->sz_bit) {
        switch (inst & ((1 << 8) | (1 << 4))) {
        case 0:
            handler = sh4_inst_binary_fmov_dr_dr;
            break;
        case (1 << 4):
            handler = sh4_inst_binary_fmov_xd_dr;
            break;
        case (1 << 8):
            handler = sh4_inst_binary_fmov_dr_xd;
            break;
        case (1 << 8) | (1 << 4):
            handler = sh4_inst_binary_fmov_xd_xd;
            break;
        default:
            RAISE_ERROR(ERROR_INTEGRITY); // should never happen
        }
    } else {
        unsigned fr_src_reg = ((inst >> 4) & 0xf) + SH4_REG_FR0;
        unsigned fr_dst_reg = ((inst >> 8) & 0xf) + SH4_REG_FR0;

        unsigned fr_src_slot = reg_slot(sh4, ctx, block,
                                        fr_src_reg, WASHDC_JIT_SLOT_FLOAT);
        unsigned fr_dst_slot = reg_slot(sh4, ctx, block,
                                        fr_dst_reg, WASHDC_JIT_SLOT_FLOAT);

        jit_mov_float(block, fr_src_slot, fr_dst_slot);

        reg_map[fr_dst_reg].stat = REG_STATUS_SLOT;

        return true;
    }

    struct jit_inst il_inst;

    res_drain_all_regs(sh4, ctx, block);
    res_invalidate_all_regs(block);

    il_inst.op = JIT_OP_FALLBACK;
    il_inst.immed.fallback.fallback_fn = handler;
    il_inst.immed.fallback.inst = inst;

    il_code_block_push_inst(block, &il_inst);

    return true;
}

static unsigned reg_slot(Sh4 *sh4, struct sh4_jit_compile_ctx *ctx,
                         struct il_code_block *block, unsigned reg_no,
                         enum washdc_jit_slot_tp tp) {
    struct residency *res = reg_map + reg_no;

    if (res->stat == REG_STATUS_SH4) {
        // need to load it into an unused slot
        unsigned slot_no = alloc_slot(block, tp);
        res_associate_reg(reg_no, slot_no);
        res->stat = REG_STATUS_SLOT_AND_SH4;
        res->slot_no = slot_no;
        // TODO: set res->last_read here
        unsigned regbase_slot = get_regbase_slot(sh4, ctx, block);
        switch (tp) {
        case WASHDC_JIT_SLOT_GEN:
            jit_load_slot_offset(block, regbase_slot, reg_no, slot_no);
            break;
        case WASHDC_JIT_SLOT_FLOAT:
            jit_load_float_slot_offset(block, regbase_slot, reg_no, slot_no);
            break;
        default:
            RAISE_ERROR(ERROR_UNIMPLEMENTED);
        }
    }

    return res->slot_no;
}

static unsigned reg_slot_noload(Sh4 *sh4,
                                struct il_code_block *block, unsigned reg_no) {
    struct residency *res = reg_map + reg_no;
    if (res->stat == REG_STATUS_SH4) {
        unsigned slot_no = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
        res_associate_reg(reg_no, slot_no);
        res->stat = REG_STATUS_SLOT;
        res->slot_no = slot_no;
        // TODO: set res->last_read here
    } else if (res->stat == REG_STATUS_SLOT_AND_SH4) {
        res->stat = REG_STATUS_SLOT;
    }
    return res->slot_no;
}

static void res_associate_reg(unsigned reg_no, unsigned slot_no) {
    struct residency *res = reg_map + reg_no;
    res->slot_no = slot_no;
}

/*
 * drain the given register and then set its status to REG_STATUS_SH4.  The
 * slot the register resided in is still valid and its value is unchanged, but
 * it is no longer associated with the given register.  The caller will need to
 * call res_free_slot on that slot when it is no longer needed.
 */
static void res_disassociate_reg(Sh4 *sh4, struct sh4_jit_compile_ctx *ctx,
                                 struct il_code_block *block, unsigned reg_no) {
    res_drain_reg(sh4, ctx, block, reg_no);
    struct residency *res = reg_map + reg_no;
    res->stat = REG_STATUS_SH4;
}

static void sh4_jit_set_sr(void *ctx, uint32_t new_sr_val) {
    struct Sh4 *sh4 = (struct Sh4*)ctx;
    uint32_t old_sr = sh4->reg[SH4_REG_SR];
    sh4->reg[SH4_REG_SR] = new_sr_val;
    sh4_on_sr_change(sh4, old_sr);
}

static void sh4_jit_hash_slot(struct Sh4 *sh4, struct il_code_block *block,
                              unsigned jmp_addr_slot, unsigned hash_slot,
                              unsigned fpscr_slot) {
    unsigned pr_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);
    unsigned sz_slot = alloc_slot(block, WASHDC_JIT_SLOT_GEN);

    jit_mov(block, jmp_addr_slot, hash_slot);
    jit_and_const32(block, hash_slot, SH4_JIT_HASH_MASK);

    jit_mov(block, fpscr_slot, pr_slot);
    jit_mov(block, fpscr_slot, sz_slot);

    jit_and_const32(block, pr_slot, SH4_FPSCR_PR_MASK);
    jit_and_const32(block, sz_slot, SH4_FPSCR_SZ_MASK);

    jit_shll(block, pr_slot, SH4_JIT_HASH_PR_SHIFT - SH4_FPSCR_PR_SHIFT);
    jit_shll(block, sz_slot, SH4_JIT_HASH_SZ_SHIFT - SH4_FPSCR_SZ_SHIFT);

    jit_or(block, pr_slot, hash_slot);
    jit_or(block, sz_slot, hash_slot);

    free_slot(block, sz_slot);
    free_slot(block, pr_slot);
}

static void
sh4_jit_hash_slot_known_fpscr(struct Sh4 *sh4,
                              struct sh4_jit_compile_ctx const *ctx,
                              struct il_code_block *block,
                              unsigned jmp_addr_slot, unsigned hash_slot) {
    jit_mov(block, jmp_addr_slot, hash_slot);
    jit_and_const32(block, hash_slot, SH4_JIT_HASH_MASK);

    if (ctx->pr_bit)
        jit_or_const32(block, hash_slot, SH4_JIT_HASH_PR_MASK);
    if (ctx->sz_bit)
        jit_or_const32(block, hash_slot, SH4_JIT_HASH_SZ_MASK);
}

static jit_hash sh4_jit_hash_wrapper(void *ctx, uint32_t addr) {
    struct Sh4 *sh4 = (Sh4*)ctx;
    return sh4_jit_hash(sh4, addr, sh4_fpscr_pr(sh4), sh4_fpscr_sz(sh4));
}
