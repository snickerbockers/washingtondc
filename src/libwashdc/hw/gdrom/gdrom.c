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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "cdrom.h"
#include "dreamcast.h"
#include "hw/sh4/sh4.h"
#include "mount.h"
#include "hw/sys/holly_intc.h"
#include "washdc/error.h"
#include "gdrom_response.h"
#include "dc_sched.h"
#include "hw/g1/g1_reg.h"
#include "intmath.h"
#include "compiler_bullshit.h"

#include "gdrom.h"

#define GDROM_TRACE(msg, ...)                                           \
    do {                                                                \
        LOG_DBG("GD-ROM (PC=%08x): ",                                   \
               (unsigned)dreamcast_get_cpu()->reg[SH4_REG_PC]);         \
        LOG_DBG(msg, ##__VA_ARGS__);                                    \
    } while (0)

#define GDROM_INFO(msg, ...)                                            \
    do {                                                                \
        LOG_INFO("GD-ROM (PC=%08x): ",                                  \
                 (unsigned)dreamcast_get_cpu()->reg[SH4_REG_PC]);       \
        LOG_INFO(msg, ##__VA_ARGS__);                                   \
    } while (0)

#define GDROM_WARN(msg, ...)                                            \
    do {                                                                \
        LOG_WARN("GD-ROM (PC=%08x): ",                                  \
               (unsigned)dreamcast_get_cpu()->reg[SH4_REG_PC]);         \
        LOG_WARN(msg, ##__VA_ARGS__);                                   \
    } while (0)

#define GDROM_ERROR(msg, ...)                                           \
    do {                                                                \
        LOG_ERROR("GD-ROM (PC=%08x): ",                                 \
               (unsigned)dreamcast_get_cpu()->reg[SH4_REG_PC]);         \
        LOG_ERROR(msg, ##__VA_ARGS__);                                  \
    } while (0)

static DEF_ERROR_INT_ATTR(gdrom_command)
static DEF_ERROR_INT_ATTR(gdrom_seek_param_tp)
static DEF_ERROR_INT_ATTR(gdrom_seek_seek_pt)

#define GDROM_GDAPRO_DEFAULT 0x00007f00
#define GDROM_G1GDRC_DEFAULT 0x0000ffff
#define GDROM_GDSTAR_DEFAULT 0x00000000 // undefined
#define GDROM_GDLEN_DEFAULT  0x00000000 // undefined
#define GDROM_GDDIR_DEFAULT  0x00000000
#define GDROM_GDEN_DEFAULT   0x00000000
#define GDROM_GDST_DEFAULT   0x00000000
#define GDROM_GDLEND_DEFAULT 0x00000000 // undefined
#define GDROM_DATA_BYTE_COUNT_DEFAULT 0xeb14

#define GDROM_REG_IDX(addr) (((addr) - ADDR_GDROM_FIRST) / 4)

#define ATA_REG_ALT_STATUS     GDROM_REG_IDX(0x5f7018)
#define ATA_REG_RW_DATA        GDROM_REG_IDX(0x5f7080)
#define ATA_REG_W_FEAT         GDROM_REG_IDX(0x5f7084)
#define ATA_REG_R_ERROR        GDROM_REG_IDX(0x5f7084)
#define ATA_REG_R_INT_REASON   GDROM_REG_IDX(0x5f7088)
#define ATA_REG_W_SEC_CNT      GDROM_REG_IDX(0x5f7088)
#define ATA_REG_R_SEC_NUM      GDROM_REG_IDX(0x5f708c)
#define ATA_REG_RW_BYTE_CNT_LO GDROM_REG_IDX(0x5f7090)
#define ATA_REG_RW_BYTE_CNT_HI GDROM_REG_IDX(0x5f7094)
#define ATA_REG_RW_DRIVE_SEL   GDROM_REG_IDX(0x5f7098)
#define ATA_REG_R_STATUS       GDROM_REG_IDX(0x5f709c)
#define ATA_REG_W_CMD          GDROM_REG_IDX(0x5f709c)

static bool bufq_empty(struct gdrom_ctxt *gdrom);

static void gdrom_delayed_processing(struct gdrom_ctxt *gdrom, dc_cycle_stamp_t delay);

static void post_delay_gdrom_delayed_processing(struct SchedEvent *event);

// how long to wait before raising a gdrom interrupt event.
// this value is arbitrary and completely made up.
// TODO: come up with some latency measurements on real hardware.
#define GDROM_INT_DELAY (SCHED_FREQUENCY / 1024)

/* static bool gdrom_int_scheduled; */
/* struct SchedEvent gdrom_int_raise_event = { */
/*     .handler = post_delay_gdrom_delayed_processing */
/* }; */

static inline char const *gdrom_state_name(enum gdrom_state state) {
    switch (state) {
    case GDROM_STATE_NORM:
        return "GDROM_STATE_NORM";
    case GDROM_STATE_INPUT_PKT:
        return "GDROM_STATE_INPUT_PKT";
    case GDROM_STATE_SET_MODE:
        return "GDROM_STATE_SET_MODE";
    case GDROM_STATE_PIO_READ_DELAY:
        return "GDROM_STATE_PIO_READ_DELAY";
    case GDROM_STATE_PIO_READING:
        return "GDROM_STATE_PIO_READING";
    case GDROM_STATE_DMA_READING:
        return "GDROM_STATE_DMA_READING";
    case GDROM_STATE_DMA_WAITING:
        return "GDROM_STATE_DMA_WAITING";
    default:
        return "ERROR/UKNOWN";
    }
}

static void gdrom_state_transition(struct gdrom_ctxt *gdrom,
                                   enum gdrom_state new_state) {
    GDROM_TRACE("DRIVE STATE TRANSITION %s -> %s\n",
                gdrom_state_name(gdrom->state), gdrom_state_name(new_state));
    gdrom->state = new_state;
}

static void gdrom_delayed_processing(struct gdrom_ctxt *gdrom, dc_cycle_stamp_t delay) {
    if (!gdrom->gdrom_int_scheduled) {
        gdrom->gdrom_int_scheduled = true;
        gdrom->gdrom_int_raise_event.when =
            clock_cycle_stamp(gdrom->clk) + delay;
        sched_event(gdrom->clk, &gdrom->gdrom_int_raise_event);
    }
}

static void post_delay_gdrom_delayed_processing(struct SchedEvent *event) {
    struct gdrom_ctxt *gdrom = (struct gdrom_ctxt*)event->arg_ptr;
    gdrom->gdrom_int_scheduled = false;

    switch (gdrom->state) {
    case GDROM_STATE_PIO_READING:
        RAISE_ERROR(ERROR_INTEGRITY);
        break;
    case GDROM_STATE_PIO_READ_DELAY:
        GDROM_TRACE("%s - PIO read complete\n", __func__);
        gdrom->meta.read.bytes_read = 0;

        if (gdrom->meta.read.byte_count == 0) {
            /*
             * This case will only happen if the byte_count parameter in
             * gdrom_state_transfer_pio_read is 0.  Otherwise, gdrom_read_data
             * will transition to GDROM_STATE_NORM when we run out of data.
             */
            gdrom->stat_reg.drq = false;
            gdrom_state_transition(gdrom, GDROM_STATE_NORM);
            gdrom->data_byte_count = 0;
        } else if (gdrom->meta.read.byte_count > 0x8000) {
            gdrom->data_byte_count = 0x8000;
            gdrom->meta.read.byte_count -= 0x8000;
            gdrom->stat_reg.drq = true;
            gdrom_state_transition(gdrom, GDROM_STATE_PIO_READING);
        } else {
            gdrom->data_byte_count = gdrom->meta.read.byte_count;
            gdrom->meta.read.byte_count = 0;
            gdrom->stat_reg.drq = true;
            gdrom_state_transition(gdrom, GDROM_STATE_PIO_READING);
        }

        gdrom->stat_reg.bsy = false;
        if (gdrom->stat_reg.drq) {
            gdrom->int_reason_reg.io = true;
            gdrom->int_reason_reg.cod = false;
        } else {
            gdrom->stat_reg.drdy = true;
            gdrom->int_reason_reg.cod = true;
            gdrom->int_reason_reg.io = true;
        }

        if (!gdrom->dev_ctrl_reg.nien) {
            GDROM_TRACE("%s - raising GDROM EXT IRQ (state="
                        "GDROM_STATE_PIO_READ_DELAY)\n", __func__);
            holly_raise_ext_int(HOLLY_EXT_INT_GDROM);
        }
        break;
    case GDROM_STATE_DMA_READING:
        GDROM_TRACE("%s - DMA read complete\n", __func__);

        if (bufq_empty(gdrom)) {
            gdrom->int_reason_reg.io = true;
            gdrom->int_reason_reg.cod = true;
            gdrom->stat_reg.drdy = true;
            gdrom->stat_reg.drq = false;
            gdrom->stat_reg.bsy = false;
            gdrom_state_transition(gdrom, GDROM_STATE_NORM);
            gdrom->gdlend_reg = gdrom->gdlend_final;

            if (!gdrom->dev_ctrl_reg.nien) {
                GDROM_TRACE("%s - raising GDROM EXT IRQ (state="
                            "GDROM_STATE_DMA_READING)\n", __func__);
                holly_raise_ext_int(HOLLY_EXT_INT_GDROM);
            }
        } else {
            gdrom->int_reason_reg.io = true;
            gdrom->int_reason_reg.cod = false;
            gdrom_state_transition(gdrom, GDROM_STATE_DMA_WAITING);
        }
        GDROM_TRACE("%s - raising GDROM DMA IRQ "
                    "(state=GDROM_STATE_DMA_READING)\n", __func__);
        holly_raise_nrm_int(HOLLY_REG_ISTNRM_GDROM_DMA_COMPLETE);
        gdrom->dma_start_reg = 0;
        break;
    case GDROM_STATE_DMA_WAITING:
        RAISE_ERROR(ERROR_INTEGRITY); // should not happen, i think
        break;
    default:
        GDROM_TRACE("%s - raising GDROM EXT IRQ (state=%s %d)\n",
                    __func__, gdrom_state_name(gdrom->state),
                    (int)gdrom->state);
        if (!gdrom->dev_ctrl_reg.nien)
            holly_raise_ext_int(HOLLY_EXT_INT_GDROM);
    }
}

enum sense_key {
    // no sense key (command execution successful)
    SENSE_KEY_NONE = 0,

    // successful error recovery
    SENSE_KEY_RECOVERED = 1,

    // drive not ready
    SENSE_KEY_NOT_READY = 2,

    // defective disc
    SENSE_KEY_MEDIUM_ERROR = 3,

    // drive failure
    SENSE_KEY_HW_ERROR = 4,

    // invalid parameter/request
    SENSE_KEY_ILLEGAL_REQ = 5,

    // disc removed/drive reset
    SENSE_KEY_UNIT_ATTN = 6,

    // writing to a read-only area
    SENSE_KEY_DATA_PROT = 7,

    // command was aborted
    SENSE_KEY_CMD_ABORT = 11
};

/*
 * 2352 was chosen as the size because that's the most that can be used at a
 * time on a CD (frame size)
 *
 * Most disc accesses will only use 2048 bytes, and some will use far
 * less than that (such as GDROM_PKT_REQ_MODE)
 */
#define GDROM_BUFQ_LEN CDROM_FRAME_SIZE

struct gdrom_bufq_node {
    struct fifo_node fifo_node;

    // idx is the index of the next valid access
    // len is the number of bytes which are valid
    // when idx == len, this buffer is empty and should be removed
    unsigned idx, len;
    uint8_t dat[GDROM_BUFQ_LEN];
};

////////////////////////////////////////////////////////////////////////////////
//
// ATA commands
//
////////////////////////////////////////////////////////////////////////////////

#define GDROM_CMD_RESET    0x08
#define GDROM_CMD_DIAG     0x90
#define GDROM_CMD_NOP      0x00
#define GDROM_CMD_PKT      0xa0
#define GDROM_CMD_IDENTIFY 0xa1
#define GDROM_CMD_ATA_IDENTIFY 0xec
#define GDROM_CMD_SET_FEAT 0xef

////////////////////////////////////////////////////////////////////////////////
//
// Packet Commands
//
////////////////////////////////////////////////////////////////////////////////

#define GDROM_PKT_TEST_UNIT   0x00
#define GDROM_PKT_REQ_STAT    0x10
#define GDROM_PKT_REQ_MODE    0x11
#define GDROM_PKT_SET_MODE    0x12
#define GDROM_PKT_REQ_ERROR   0x13
#define GDROM_PKT_READ_TOC    0x14
#define GDROM_PKT_REQ_SESSION 0x15
#define GDROM_PKT_READ        0x30
#define GDROM_PKT_PLAY        0x20
#define GDROM_PKT_SEEK        0x21
#define GDROM_PKT_SUBCODE     0x40
#define GDROM_PKT_START_DISK  0x70
#define GDROM_PKT_UNKNOWN_71  0x71

// Empty out the bufq and free resources.
static void bufq_clear(struct gdrom_ctxt *ctxt);

/*
 * grab one byte from the queue, pop/clear a node (if necessary) and return 0.
 * this returns non-zero if the queue is empty.
 */
static int bufq_consume_byte(struct gdrom_ctxt *gdrom, unsigned *byte);

static void gdrom_clear_error(struct gdrom_ctxt *gdrom);

static void gdrom_input_read_packet(struct gdrom_ctxt *gdrom);

// called when all 12 bytes of a packet have been written to data
static void gdrom_input_packet(struct gdrom_ctxt *gdrom);

static void gdrom_input_req_mode_packet(struct gdrom_ctxt *gdrom);

static void gdrom_input_set_mode_packet(struct gdrom_ctxt *gdrom);

static void gdrom_input_req_error_packet(struct gdrom_ctxt *gdrom);

static void gdrom_input_test_unit_packet(struct gdrom_ctxt *gdrom);

static void gdrom_input_start_disk_packet(struct gdrom_ctxt *gdrom);

static void gdrom_input_read_toc_packet(struct gdrom_ctxt *gdrom);

static void gdrom_input_packet_71(struct gdrom_ctxt *gdrom);

static void gdrom_input_read_subcode_packet(struct gdrom_ctxt *gdrom);

static void gdrom_input_seek_packet(struct gdrom_ctxt *gdrom);
static void gdrom_input_play_packet(struct gdrom_ctxt *gdrom);

static void gdrom_input_req_session_packet(struct gdrom_ctxt *gdrom);

/* struct gdrom_ctxt gdrom; */

static uint32_t
gdrom_gdapro_mmio_read(struct mmio_region_g1_reg_32 *region,
                       unsigned idx, void *ctxt);
static void
gdrom_gdapro_mmio_write(struct mmio_region_g1_reg_32 *region,
                        unsigned idx, uint32_t val, void *ctxt);
static uint32_t
gdrom_g1gdrc_mmio_read(struct mmio_region_g1_reg_32 *region,
                       unsigned idx, void *ctxt);
static void
gdrom_g1gdrc_mmio_write(struct mmio_region_g1_reg_32 *region,
                        unsigned idx, uint32_t val, void *ctxt);
static uint32_t
gdrom_gdstar_mmio_read(struct mmio_region_g1_reg_32 *region,
                       unsigned idx, void *ctxt);
static void
gdrom_gdstar_mmio_write(struct mmio_region_g1_reg_32 *region,
                        unsigned idx, uint32_t val, void *ctxt);
static uint32_t
gdrom_gdlen_mmio_read(struct mmio_region_g1_reg_32 *region,
                      unsigned idx, void *ctxt);
static void
gdrom_gdlen_mmio_write(struct mmio_region_g1_reg_32 *region,
                       unsigned idx, uint32_t val, void *ctxt);
static uint32_t
gdrom_gddir_mmio_read(struct mmio_region_g1_reg_32 *region,
                      unsigned idx, void *ctxt);
static void
gdrom_gddir_mmio_write(struct mmio_region_g1_reg_32 *region,
                       unsigned idx, uint32_t val, void *ctxt);
static uint32_t
gdrom_gden_mmio_read(struct mmio_region_g1_reg_32 *region,
                     unsigned idx, void *ctxt);
static void
gdrom_gden_mmio_write(struct mmio_region_g1_reg_32 *region,
                      unsigned idx, uint32_t val, void *ctxt);
static uint32_t
gdrom_gdst_reg_read_handler(struct mmio_region_g1_reg_32 *region,
                            unsigned idx, void *ctxt);
static void
gdrom_gdst_reg_write_handler(struct mmio_region_g1_reg_32 *region,
                             unsigned idx, uint32_t val, void *ctxt);
static uint32_t
gdrom_gdlend_mmio_read(struct mmio_region_g1_reg_32 *region,
                       unsigned idx, void *ctxt);
static uint32_t
gdrom_gdstard_reg_read_handler(struct mmio_region_g1_reg_32 *region,
                               unsigned idx, void *ctxt);

static float gdrom_reg_read_float(addr32_t addr, void *ctxt);
static void gdrom_reg_write_float(addr32_t addr, float val, void *ctxt);
static double gdrom_reg_read_double(addr32_t addr, void *ctxt);
static void gdrom_reg_write_double(addr32_t addr, double val, void *ctxt);
static uint8_t gdrom_reg_read_8(addr32_t addr, void *ctxt);
static void gdrom_reg_write_8(addr32_t addr, uint8_t val, void *ctxt);
static uint16_t gdrom_reg_read_16(addr32_t addr, void *ctxt);
static void gdrom_reg_write_16(addr32_t addr, uint16_t val, void *ctxt);
static uint32_t gdrom_reg_read_32(addr32_t addr, void *ctxt);
static void gdrom_reg_write_32(addr32_t addr, uint32_t val, void *ctxt);

/*
 * do a DMA transfer from GD-ROM to host using whatever's in the buffer queue.
 *
 * This function gets all the relevant parameters from the registers,
 * performs the transfer and sets the final value of all relevant registers
 * except the ones that have flags or pertain to interrupts
 */
static void gdrom_complete_dma(struct gdrom_ctxt *gdrom);

static void gdrom_pre_read(struct gdrom_ctxt *gdrom, addr32_t addr,
                           unsigned n_bytes);
static void gdrom_post_write(struct gdrom_ctxt *gdrom, addr32_t addr,
                             unsigned n_bytes);
static void gdrom_reg_init(struct gdrom_ctxt *gdrom);

static void
gdrom_state_transfer_pio_read(struct gdrom_ctxt *gdrom, unsigned byte_count);

struct memory_interface gdrom_reg_intf = {
    .read32 = gdrom_reg_read_32,
    .read16 = gdrom_reg_read_16,
    .read8 = gdrom_reg_read_8,
    .readfloat = gdrom_reg_read_float,
    .readdouble = gdrom_reg_read_double,

    .write32 = gdrom_reg_write_32,
    .write16 = gdrom_reg_write_16,
    .write8 = gdrom_reg_write_8,
    .writefloat = gdrom_reg_write_float,
    .writedouble = gdrom_reg_write_double
};

void gdrom_init(struct gdrom_ctxt *gdrom, struct dc_clock *gdrom_clk) {
    memset(gdrom, 0, sizeof(*gdrom));

    gdrom->gdrom_int_raise_event.handler = post_delay_gdrom_delayed_processing;
    gdrom->gdrom_int_raise_event.arg_ptr = gdrom;

    gdrom->clk = gdrom_clk;
    gdrom->gdapro_reg = GDROM_GDAPRO_DEFAULT;
    gdrom->g1gdrc_reg = GDROM_G1GDRC_DEFAULT;
    gdrom->dma_start_addr_reg = GDROM_GDSTAR_DEFAULT;
    gdrom->dma_len_reg = GDROM_GDLEN_DEFAULT;
    gdrom->dma_dir_reg = GDROM_GDDIR_DEFAULT;
    gdrom->dma_en_reg = GDROM_GDEN_DEFAULT;
    gdrom->dma_start_reg = GDROM_GDST_DEFAULT;
    gdrom->gdlend_reg = GDROM_GDLEND_DEFAULT;

    gdrom->additional_sense = ADDITIONAL_SENSE_NO_ERROR;

    gdrom->error_reg.ili = true;
    gdrom->sect_cnt_reg.trans_mode = TRANS_MODE_PIO_DFLT;
    gdrom->sect_cnt_reg.mode_val = 1;
    gdrom->data_byte_count = GDROM_DATA_BYTE_COUNT_DEFAULT;

    fifo_init(&gdrom->bufq);

    gdrom_reg_init(gdrom);
}

static void gdrom_reg_init(struct gdrom_ctxt *gdrom) {
    /* GD-ROM DMA registers */
    g1_mmio_cell_init_32("SB_GDAPRO", 0x5f74b8,
                         gdrom_gdapro_mmio_read,
                         gdrom_gdapro_mmio_write, gdrom);
    g1_mmio_cell_init_32("SB_G1GDRC", 0x5f74a0,
                         gdrom_g1gdrc_mmio_read,
                         gdrom_g1gdrc_mmio_write, gdrom);
    g1_mmio_cell_init_32("SB_G1GDWC", 0x5f74a4,
                         mmio_region_g1_reg_32_warn_read_handler,
                         mmio_region_g1_reg_32_warn_write_handler,
                         gdrom);
    g1_mmio_cell_init_32("SB_GDSTAR", 0x5f7404,
                         gdrom_gdstar_mmio_read,
                         gdrom_gdstar_mmio_write,
                         gdrom);
    g1_mmio_cell_init_32("SB_GDLEN", 0x5f7408,
                         gdrom_gdlen_mmio_read,
                         gdrom_gdlen_mmio_write,
                         gdrom);
    g1_mmio_cell_init_32("SB_GDDIR", 0x5f740c,
                         gdrom_gddir_mmio_read,
                         gdrom_gddir_mmio_write,
                         gdrom);
    g1_mmio_cell_init_32("SB_GDEN", 0x5f7414,
                         gdrom_gden_mmio_read,
                         gdrom_gden_mmio_write, gdrom);
    g1_mmio_cell_init_32("SB_GDST", 0x5f7418,
                         gdrom_gdst_reg_read_handler,
                         gdrom_gdst_reg_write_handler, gdrom);
    g1_mmio_cell_init_32("SB_GDSTARD", 0x005f74f4,
                         gdrom_gdstard_reg_read_handler,
                         mmio_region_g1_reg_32_readonly_write_error,
                         gdrom);
    g1_mmio_cell_init_32("SB_GDLEND", 0x005f74f8,
                         gdrom_gdlend_mmio_read,
                         mmio_region_g1_reg_32_readonly_write_error,
                         gdrom);
}

void gdrom_reg_cleanup(struct gdrom_ctxt *gdrom) {
}

void gdrom_cleanup(struct gdrom_ctxt *gdrom) {
    gdrom_reg_cleanup(gdrom);
}

static void bufq_clear(struct gdrom_ctxt *gdrom) {
    size_t len = 0;

    while (!fifo_empty(&gdrom->bufq)) {
        struct gdrom_bufq_node *bufq_node =
            &FIFO_DEREF(fifo_pop(&gdrom->bufq), struct gdrom_bufq_node, fifo_node);

        len += bufq_node->len;

        free(bufq_node);
    }

    if (len) {
        GDROM_ERROR("%s just threw out %llu bytes\n",
                    __func__, (long long unsigned)len);
    }
}

static int bufq_consume_byte(struct gdrom_ctxt *gdrom, unsigned *byte) {
    struct fifo_node *node = fifo_peek(&gdrom->bufq);

    if (node) {
        struct gdrom_bufq_node *bufq_node =
            &FIFO_DEREF(node, struct gdrom_bufq_node, fifo_node);

        *byte = (unsigned)bufq_node->dat[bufq_node->idx++];

        if (bufq_node->idx >= bufq_node->len) {
            fifo_pop(&gdrom->bufq);
            free(bufq_node);
        }

        return 0;
    }

    return -1;
}

static bool bufq_empty(struct gdrom_ctxt *gdrom) {
    return fifo_empty(&gdrom->bufq);
}

static void gdrom_clear_error(struct gdrom_ctxt *gdrom) {
    memset(&gdrom->error_reg, 0, sizeof(gdrom->error_reg));
}

static DEF_ERROR_U32_ATTR(gdrom_dma_prot_top)
static DEF_ERROR_U32_ATTR(gdrom_dma_prot_bot)

static void gdrom_complete_dma(struct gdrom_ctxt *gdrom) {
    unsigned bytes_transmitted = 0;
    unsigned bytes_to_transmit = gdrom->dma_len_reg;
    unsigned addr = gdrom->dma_start_addr_reg;

    struct fifo_node *fifo_node = fifo_peek(&gdrom->bufq);

    while (bytes_transmitted < bytes_to_transmit) {
        if (!fifo_node) {
            GDROM_ERROR("%s attempting to transfer more data than there is in"
                        "the bufq available\n", __func__);
            goto done;
        }

        struct gdrom_bufq_node *bufq_node =
            &FIFO_DEREF(fifo_node, struct gdrom_bufq_node, fifo_node);

#ifdef INVARIANTS
        if (bufq_node->idx >= bufq_node->len)
            RAISE_ERROR(ERROR_INTEGRITY);
#endif

        unsigned chunk_sz = bufq_node->len - bufq_node->idx;

        if ((chunk_sz + bytes_transmitted) > bytes_to_transmit)
            chunk_sz = bytes_to_transmit - bytes_transmitted;

        bytes_transmitted += chunk_sz;

        /*
         * enforce the gdapro register
         * bytes_transmitted will still count the full length of chunk_sz
         * because that seems like the logical behavior here.  I have not run
         * any hardware tests to confirm that this is correct.
         *
         * For now we raise unimplemented errors when this happens because I
         * don't have any known testcases.
         *
         * The GDAPRO register only applies to system memory, which is why we
         * don't raise an error for writes that go outside of
         * 0x0c000000-0x0cffffff (thanks to p1pkin for explaining this to me).
         */
        if (addr >= 0x0c000000 && addr <= 0x0cffffff &&
            (addr < gdrom_dma_prot_top(gdrom) ||
             (addr + chunk_sz - 1) > gdrom_dma_prot_bot(gdrom))) {
            // don't do this chunk if the end is below gdrom_dma_prot_top
            error_set_address(addr);
            error_set_length(chunk_sz);
            error_set_gdrom_dma_prot_top(gdrom_dma_prot_top(gdrom));
            error_set_gdrom_dma_prot_bot(gdrom_dma_prot_bot(gdrom));
            error_set_feature("the GD-ROM DMA protection register");
            RAISE_ERROR(ERROR_UNIMPLEMENTED);
        }

        sh4_dmac_transfer_to_mem(dreamcast_get_cpu(), addr, chunk_sz,
                                 1, bufq_node->dat + bufq_node->idx);

        bufq_node->idx += chunk_sz;

        if (bufq_node->idx < bufq_node->len)
            continue;

        fifo_pop(&gdrom->bufq);
        addr += chunk_sz;
        free(bufq_node);
        fifo_node = fifo_peek(&gdrom->bufq);
    }

done:
    if (bytes_transmitted)
        GDROM_TRACE("GD-ROM DMA transfer %u bytes to %08X\n",
                    bytes_transmitted, gdrom->dma_start_addr_reg);


    // set GD_LEND, etc here

    if (bytes_transmitted > BIT_RANGE(5, 24)) {
        /*
         * not sure what happens when it's too big to fit in the GDLEND
         * register.
         */
        RAISE_ERROR(ERROR_UNIMPLEMENTED);
    }

    gdrom->gdlend_final = bytes_transmitted;
    gdrom->dma_start_stamp = clock_cycle_stamp(gdrom->clk);

    /*
     * According to SegaRetro, the Dreamcast's GD-ROM drive can transmit data
     * at approx 1.8 mb/s.
     *
     * The actual delay on real hardware would probably be slower than this due
     * to seek times, as well as any up-front latency just from sending the
     * drive commands.  I am not sure how to model this since an accurate
     * simulation of drive delays is effectively a newtonian mechanics problem.
     *
     * HOWEVER, I currently have the delay coded to 4 mb/s because
     * Street Fighter Alpha 3 won't work with anything slower than that.  This
     * may mean my source for the specs were wrong, or it may mean that the
     * reason why sfa3 wouldn't work is that the GD-ROM's interrupt delay needs
     * to be proportional to some other interrupt delay which it is not
     * currently proportional to.
     */
    gdrom->dma_delay = gdrom->additional_dma_delay;
    gdrom->additional_dma_delay = 0;

    gdrom_state_transition(gdrom, GDROM_STATE_DMA_READING);
    gdrom->stat_reg.check = false;
    gdrom_clear_error(gdrom);

    gdrom_delayed_processing(gdrom, gdrom->dma_delay);
}

static void
gdrom_state_transfer_pio_read(struct gdrom_ctxt *gdrom, unsigned byte_count) {
    gdrom_state_transition(gdrom, GDROM_STATE_PIO_READ_DELAY);
    gdrom->meta.read.byte_count = byte_count;

    gdrom->stat_reg.bsy = true;
    gdrom->stat_reg.drq = false;
    gdrom->stat_reg.check = false;
    gdrom_clear_error(gdrom);

    gdrom_delayed_processing(gdrom, GDROM_INT_DELAY);
}

static void gdrom_input_read_packet(struct gdrom_ctxt *gdrom) {
    GDROM_TRACE("READ_PACKET command received\n");

    unsigned start_addr = (gdrom->pkt_buf[2] << 16) |
        (gdrom->pkt_buf[3] << 8) | gdrom->pkt_buf[4];
    unsigned trans_len = (gdrom->pkt_buf[8] << 16) |
        (gdrom->pkt_buf[9] << 8) | gdrom->pkt_buf[10];
    unsigned data_sel = gdrom->pkt_buf[1] >> 4;
    unsigned param_tp = gdrom->pkt_buf[1] & 1;

    if (data_sel != 0x2) {
        error_set_feature("CD-ROM header/subheader access");
        RAISE_ERROR(ERROR_UNIMPLEMENTED);
    }

    // TODO - check the expected data type (data_tp_expect)

    if (param_tp != 0) {
        /*
         * i think this is a timecode format that maps linearly to FAD/LBA, but
         * for now I'm just not sure.
         */
        error_set_feature("MSF format CD-ROM access");
        RAISE_ERROR(ERROR_UNIMPLEMENTED);
    }

    GDROM_TRACE("request to read %u sectors from FAD %u\n",
                trans_len, start_addr);

    bufq_clear(gdrom);

    unsigned byte_count = CDROM_FRAME_DATA_SIZE * trans_len;
    gdrom->data_byte_count = 0;

    if (!gdrom->feat_reg.dma_enable && gdrom->data_byte_count > UINT16_MAX)
        GDROM_WARN("OVERFLOW: Reading %u bytes from gdrom PIO!\n", gdrom->data_byte_count);

    unsigned fad_offs = 0;
    while (trans_len--) {
        struct gdrom_bufq_node *node =
            (struct gdrom_bufq_node*)calloc(1, sizeof(struct gdrom_bufq_node));

        if (!node)
            RAISE_ERROR(ERROR_FAILED_ALLOC);

        if (mount_read_sectors(node->dat, start_addr + fad_offs++, 1) < 0)
            GDROM_ERROR("GD-ROM failed to read fad %u\n", fad_offs);

        node->idx = 0;
        node->len = CDROM_FRAME_DATA_SIZE;

        fifo_push(&gdrom->bufq, &node->fifo_node);
    }

    if (gdrom->feat_reg.dma_enable) {
        // wait for them to write 1 to GDST before doing something
        GDROM_TRACE("DMA READ ACCESS\n");
        gdrom->additional_dma_delay = GDROM_INT_DELAY;
        gdrom_state_transition(gdrom, GDROM_STATE_DMA_WAITING);
    } else {
        /*
         * TODO: limit based on read bandwidth.  Currently this is implemented
         * for DMA (see gdrom_complete_dma) but not for PIO.  Most large
         * transfers are probably done through DMA anyways so I don't think
         * this matters too much, but it should still be done for PIO.
         */
        gdrom_state_transfer_pio_read(gdrom, byte_count);
    }
}

/*
 * this function is called after 12 bytes have been written to the data
 * register after the drive has received GDROM_CMD_PKT (which puts it in
 * GDROM_STATE_INPUT_PKT
 */
static void gdrom_input_packet(struct gdrom_ctxt *gdrom) {
    gdrom->stat_reg.drq = false;
    gdrom->stat_reg.bsy = false;

    switch (gdrom->pkt_buf[0]) {
    case GDROM_PKT_TEST_UNIT:
        gdrom_input_test_unit_packet(gdrom);
        break;
    case GDROM_PKT_REQ_STAT:
        // TODO: implement this
        GDROM_ERROR("UNIMPLEMENTED REQ_STAT COMMAND RECEIVED!\n");
        gdrom_state_transition(gdrom, GDROM_STATE_NORM);
        gdrom_delayed_processing(gdrom, GDROM_INT_DELAY);
        break;
    case GDROM_PKT_REQ_MODE:
        gdrom_input_req_mode_packet(gdrom);
        break;
    case GDROM_PKT_SET_MODE:
        gdrom_input_set_mode_packet(gdrom);
        break;
    case GDROM_PKT_REQ_ERROR:
        gdrom_input_req_error_packet(gdrom);
        break;
    case GDROM_PKT_START_DISK:
        gdrom_input_start_disk_packet(gdrom);
        break;
    case GDROM_PKT_READ_TOC:
        gdrom_input_read_toc_packet(gdrom);
        break;
    case GDROM_PKT_SUBCODE:
        gdrom_input_read_subcode_packet(gdrom);
        break;
    case GDROM_PKT_READ:
        gdrom_input_read_packet(gdrom);
        break;
    case GDROM_PKT_UNKNOWN_71:
        gdrom_input_packet_71(gdrom);
        break;
    case GDROM_PKT_SEEK:
        gdrom_input_seek_packet(gdrom);
        break;
    case GDROM_PKT_PLAY:
        gdrom_input_play_packet(gdrom);
        break;
    case GDROM_PKT_REQ_SESSION:
        gdrom_input_req_session_packet(gdrom);
        break;
    default:
        error_set_feature("unknown GD-ROM packet command");
        error_set_gdrom_command((unsigned)gdrom->pkt_buf[0]);
        RAISE_ERROR(ERROR_UNIMPLEMENTED);
        /* gdrom_state_transition(gdrom, GDROM_STATE_NORM); */
    }
}

void gdrom_cmd_set_features(struct gdrom_ctxt *gdrom) {
    /* bool set; */

    GDROM_TRACE("SET_FEATURES command received\n");

    if (gdrom->feat_reg.set_feat_enable) {
        /* set = (bool)(feat_reg >> 7); */
    } else {
        GDROM_TRACE("software executed \"Set Features\" command without "
                    "writing 3 to the features register\n");
        return;
    }

    switch (gdrom->sect_cnt_reg.trans_mode) {
    case TRANS_MODE_PIO_DFLT:
        gdrom->trans_mode_vals[TRANS_MODE_PIO_DFLT] =
            gdrom->sect_cnt_reg.mode_val;
        GDROM_TRACE("default PIO transfer mode set to 0x%02x\n",
                    (unsigned)gdrom->trans_mode_vals[TRANS_MODE_PIO_DFLT]);
        break;
    case TRANS_MODE_PIO_FLOW_CTRL:
        gdrom->trans_mode_vals[TRANS_MODE_PIO_FLOW_CTRL] =
            gdrom->sect_cnt_reg.mode_val;
        GDROM_TRACE("flow-control PIO transfer mode set to 0x%02x\n",
                    (unsigned)gdrom->trans_mode_vals[TRANS_MODE_PIO_FLOW_CTRL]);
        break;
    case TRANS_MODE_SINGLE_WORD_DMA:
        gdrom->trans_mode_vals[TRANS_MODE_SINGLE_WORD_DMA] =
            gdrom->sect_cnt_reg.mode_val;
        GDROM_TRACE("single-word DMA transfer mode set to 0x%02x\n",
                    (unsigned)gdrom->trans_mode_vals[TRANS_MODE_SINGLE_WORD_DMA]);
        break;
    case TRANS_MODE_MULTI_WORD_DMA:
        gdrom->trans_mode_vals[TRANS_MODE_MULTI_WORD_DMA] =
            gdrom->sect_cnt_reg.mode_val;
        GDROM_TRACE("multi-word DMA transfer mode set to 0x%02x\n",
                    (unsigned)gdrom->trans_mode_vals[TRANS_MODE_MULTI_WORD_DMA]);
        break;
    case TRANS_MODE_PSEUDO_DMA:
        gdrom->trans_mode_vals[TRANS_MODE_PSEUDO_DMA] =
            gdrom->sect_cnt_reg.mode_val;
        GDROM_TRACE("pseudo-DMA transfer mode set to 0x%02x\n",
                    (unsigned)gdrom->trans_mode_vals[TRANS_MODE_PSEUDO_DMA]);
        break;
    default:
        /*
         * I'm pretty sure this can never happen due to the
         * 'unrecognized transfer mode' ERROR_UNIMPLEMENTED in
         * gdrom_set_sect_cnt_reg.  If that ever gets changed from an error to
         * a warning, then we're going to have to set the trans_mode to some
         * special constant value to show that it's invalid.
         *
         * One other problem is that of the default value; currently it
         * defaults to TRANS_MODE_PIO_DFLT (because that's zero), but I'm not
         * sure if this is the correct default value for the sector count
         * register.
         */
        RAISE_ERROR(ERROR_INTEGRITY);
    }

    gdrom->stat_reg.check = false;
    gdrom_clear_error(gdrom);
    gdrom->int_reason_reg.cod = true; // is this correct ?

    gdrom_delayed_processing(gdrom, GDROM_INT_DELAY);
}

/*
 * XXX This command seemingly has an inaccuracy, in that in real hardware the final
 * status is 0xd0, which corresponds to BSY, DRDY and DSC set after DRQ clears?.
 * This might mean there's more data left to transfer after it does more
 * processing, but IDK if that even makes sense because at this point all 80
 * bytes have been transmitted.
 *
 * It could also just be a timing thing because eventually it settles to 0x50,
 * which is just DSC and DRDY (DRQ never gets raised).  In WashingtonDC's case,
 * it's just missing DSC...
 */
void gdrom_cmd_identify(struct gdrom_ctxt *gdrom) {
    GDROM_TRACE("IDENTIFY command received\n");

    bufq_clear(gdrom);

    struct gdrom_bufq_node *node =
        (struct gdrom_bufq_node*)malloc(sizeof(struct gdrom_bufq_node));

    if (!node)
        RAISE_ERROR(ERROR_FAILED_ALLOC);

    node->idx = 0;
    node->len = GDROM_IDENT_RESP_LEN;
    memcpy(node->dat, gdrom_ident_resp, sizeof(gdrom_ident_resp));

    fifo_push(&gdrom->bufq, &node->fifo_node);

    gdrom_state_transfer_pio_read(gdrom, GDROM_IDENT_RESP_LEN);
}

void gdrom_cmd_begin_packet(struct gdrom_ctxt *gdrom) {
    GDROM_TRACE("PACKET command received\n");

    // clear errors
    // TODO: I'm not sure if this should be done for all commands, or just packet commands
    gdrom->stat_reg.check = false;
    /* memset(&error_reg, 0, sizeof(error_reg)); */

    gdrom->int_reason_reg.io = false;
    gdrom->int_reason_reg.cod = true;
    gdrom->stat_reg.drq = true;
    gdrom->n_bytes_received = 0;
    gdrom_state_transition(gdrom, GDROM_STATE_INPUT_PKT);
}

static void gdrom_input_test_unit_packet(struct gdrom_ctxt *gdrom) {
    GDROM_TRACE("TEST_UNIT packet received\n");

    // is this correct?
    gdrom->int_reason_reg.cod = true;
    gdrom->int_reason_reg.io = true;
    gdrom->stat_reg.drdy = true;
    gdrom->stat_reg.bsy = false;
    gdrom->stat_reg.drq = false;

    // raise interrupt if it is enabled
    gdrom_delayed_processing(gdrom, GDROM_INT_DELAY);

    gdrom_state_transition(gdrom, GDROM_STATE_NORM);

    gdrom_clear_error(gdrom);
    if (mount_check()) {
        gdrom->stat_reg.check = false;
    } else {
        gdrom->stat_reg.check = true;
        gdrom->error_reg.sense_key = SENSE_KEY_NOT_READY;
        gdrom->additional_sense = ADDITIONAL_SENSE_NO_DISC;
    }
}

static void gdrom_input_req_error_packet(struct gdrom_ctxt *gdrom) {
    GDROM_TRACE("REQ_ERROR packet received\n");

    uint8_t len = gdrom->pkt_buf[4];

    uint8_t dat_out[10] = {
        0xf0,
        0,
        gdrom->error_reg.sense_key & 0xf,
        0,
        0,
        0,
        0,
        0,
        (uint8_t)(gdrom->additional_sense),
        0
    };

    if (len > 10)
        len = 10;

    bufq_clear(gdrom);

    unsigned byte_count;
    if (len != 0) {
        struct gdrom_bufq_node *node =
            (struct gdrom_bufq_node*)malloc(sizeof(struct gdrom_bufq_node));
        node->idx = 0;
        node->len = len;
        memcpy(&node->dat, dat_out, len);
        fifo_push(&gdrom->bufq, &node->fifo_node);
        byte_count = node->len;
    } else {
        byte_count = 0;
    }

    gdrom_state_transfer_pio_read(gdrom, byte_count);
}

static DEF_ERROR_INT_ATTR(session_number)

static void gdrom_input_req_session_packet(struct gdrom_ctxt *gdrom) {
    unsigned session_no = gdrom->pkt_buf[2];
    unsigned alloc_len = gdrom->pkt_buf[4];

    bufq_clear(gdrom);

    unsigned tno, fad;

    unsigned sess_count = mount_session_count();

    if (session_no == 0) {
        fad = cdrom_lba_to_fad(mount_get_leadout());
        tno = sess_count;
    } else {
        if (session_no > sess_count) {
            /*
             * I think the correct behavior in this situation is to never raise
             * the DRQ flag.  I'm not sure what exactly happens, I just know
             * that it never raises the DRQ flag.
             *
             * Whatever the case, it obviously doesn't work on real hardware so
             * I can't imagine that there are any games that try to do this.
             */
            error_set_feature("REQ_SESSION packet for non-existant sessions");
            error_set_session_number(session_no);
            RAISE_ERROR(ERROR_UNIMPLEMENTED);
        }

        session_no -= 1;
        mount_get_session_start(session_no, &tno, &fad);
        tno++; // CD standard has tracks start at 1 instead of 0
    }

    uint8_t reply[6] = {
        gdrom_get_drive_state(),
        0,
        tno,
        (fad >> 16) & 0xff,
        (fad >> 8) & 0xff,
        fad & 0xff
    };

    struct gdrom_bufq_node *node =
        (struct gdrom_bufq_node*)malloc(sizeof(struct gdrom_bufq_node));
    if (!node)
        RAISE_ERROR(ERROR_FAILED_ALLOC);

    memcpy(node->dat, reply, sizeof(reply));
    node->idx = 0;
    node->len = alloc_len < 6 ? alloc_len : 6;
    fifo_push(&gdrom->bufq, &node->fifo_node);

    gdrom_state_transfer_pio_read(gdrom, node->len);
}

/*
 * Exactly what this command does is a mystery to me.  It doesn't appear to
 * convey any data because the bios does not check for any.  What little
 * information I can find would seem to convey that this is some sort of a
 * disk initialization function?
 */
static void gdrom_input_start_disk_packet(struct gdrom_ctxt *gdrom) {
    GDROM_TRACE("START_DISK(=0x70) packet received\n");

    // is this correct?
    gdrom->int_reason_reg.cod = true;
    gdrom->int_reason_reg.io = true;
    gdrom->stat_reg.drdy = true;
    gdrom->stat_reg.bsy = false;
    gdrom->stat_reg.drq = false;

    gdrom_state_transition(gdrom, GDROM_STATE_NORM);

    gdrom->stat_reg.check = false;
    gdrom_clear_error(gdrom);
    gdrom_delayed_processing(gdrom, GDROM_INT_DELAY);
}

/*
 * Packet 0x71 is not available in any of the documentation I have on hand,
 * and its purpose is not apparent.  What it does is return a huge chunk of
 * data.  The data returned is never the same (even on the same Dreamcast
 * with the same disc inserted!), and it's not even the same length.
 *
 * TODO: This is some sort of security check.  See p1pkin's comments on
 * https://github.com/washingtondc-emu/washingtondc/commit/3d282f10a22a9e15de6fa5731834ca0a8ed4437a
 * for an explanation.
 *
 * For now, the below implementation returns a string that I captured on a live
 * Dreamcast.  Even though it's always the same string, this seems to work well
 * enough.
 */
static_assert(GDROM_PKT_71_RESP_LEN < GDROM_BUFQ_LEN,
              "GDROM_BUFQ_LEN is too small for the response to packet 0x71");
static void gdrom_input_packet_71(struct gdrom_ctxt *gdrom) {
    GDROM_TRACE("GDROM_PKT_UNKNOWN_71 packet received; sending pre-recorded "
                "response\n");

    bufq_clear(gdrom);

    struct gdrom_bufq_node *node =
        (struct gdrom_bufq_node*)malloc(sizeof(struct gdrom_bufq_node));
    node->idx = 0;
    node->len = GDROM_PKT_71_RESP_LEN;

    /*
     * XXX this works because GDROM_PKT_71_RESP_LEN is less than GDROM_BUFQ_LEN.
     * if that ever changes, so must this code
     */
    memcpy(node->dat, pkt71_resp, GDROM_PKT_71_RESP_LEN);

    fifo_push(&gdrom->bufq, &node->fifo_node);

    gdrom_state_transfer_pio_read(gdrom, GDROM_PKT_71_RESP_LEN);
}

static void gdrom_input_set_mode_packet(struct gdrom_ctxt *gdrom) {
    // TODO: actually implement this for real instead of ignoring the data

    WASHDC_UNUSED unsigned starting_addr = gdrom->pkt_buf[2];
    WASHDC_UNUSED unsigned len = gdrom->pkt_buf[4];

    GDROM_TRACE("SET_MODE command received\n");
    GDROM_TRACE("read %u bytes starting at %u\n", len, starting_addr);

    // read features, byte count here
    gdrom->set_mode_bytes_remaining = gdrom->data_byte_count;
    GDROM_TRACE("data_byte_count is %u\n", (unsigned)gdrom->data_byte_count);

    if (gdrom->feat_reg.dma_enable) {
        error_set_feature("GD-ROM SET_MODE command DMA support");
        RAISE_ERROR(ERROR_UNIMPLEMENTED);
    }

    gdrom->int_reason_reg.io = true;
    gdrom->int_reason_reg.cod = false;
    gdrom->stat_reg.drq = true;

    gdrom_state_transition(gdrom, GDROM_STATE_SET_MODE);

    gdrom_delayed_processing(gdrom, GDROM_INT_DELAY);
}

static void gdrom_input_req_mode_packet(struct gdrom_ctxt *gdrom) {
    unsigned starting_addr = gdrom->pkt_buf[2];
    unsigned len = gdrom->pkt_buf[4];

    GDROM_TRACE("REQ_MODE command received\n");
    GDROM_TRACE("read %u bytes starting at %u\n", len, starting_addr);

    bufq_clear(gdrom);

    unsigned byte_count;
    if (len != 0) {
        unsigned first_idx = starting_addr;
        unsigned last_idx = starting_addr + (len - 1);

        if (first_idx > (GDROM_REQ_MODE_RESP_LEN - 1))
            first_idx = GDROM_REQ_MODE_RESP_LEN - 1;
        if (last_idx > (GDROM_REQ_MODE_RESP_LEN - 1))
            last_idx = GDROM_REQ_MODE_RESP_LEN - 1;

        struct gdrom_bufq_node *node =
            (struct gdrom_bufq_node*)malloc(sizeof(struct gdrom_bufq_node));

        node->idx = 0;
        node->len = last_idx - first_idx + 1;
        memcpy(&node->dat, gdrom_req_mode_resp + first_idx,
               node->len * sizeof(uint8_t));

        bufq_clear(gdrom);
        fifo_push(&gdrom->bufq, &node->fifo_node);
        byte_count = node->len;
    } else {
        byte_count = 0;
    }

    gdrom_state_transfer_pio_read(gdrom, byte_count);
}

static void gdrom_input_read_toc_packet(struct gdrom_ctxt *gdrom) {
    unsigned region = gdrom->pkt_buf[1] & 1;
    unsigned len = (((unsigned)gdrom->pkt_buf[3]) << 8) | gdrom->pkt_buf[4];

    GDROM_TRACE("GET_TOC command received\n");
    GDROM_TRACE("request to read %u bytes from the Table of Contents for "
                "region %s\n", len, region ? "HIGH_DENSITY" : "LOW_DENSITY");

    struct mount_toc toc;
    memset(&toc, 0, sizeof(toc));

    // TODO: call mount_check and signal an error if nothing is mounted
    mount_read_toc(&toc, region);

    bufq_clear(gdrom);
    struct gdrom_bufq_node *node =
        (struct gdrom_bufq_node*)malloc(sizeof(struct gdrom_bufq_node));

    uint8_t const *ptr = mount_encode_toc(&toc);

    if (len > CDROM_TOC_SIZE)
        len = CDROM_TOC_SIZE;

    node->idx = 0;
    node->len = len;
    memcpy(node->dat, ptr, len);

    fifo_push(&gdrom->bufq, &node->fifo_node);

    gdrom_state_transfer_pio_read(gdrom, len);
}

static void gdrom_input_read_subcode_packet(struct gdrom_ctxt *gdrom) {
    unsigned idx;
    unsigned len = (((unsigned)gdrom->pkt_buf[3]) << 8) | gdrom->pkt_buf[4];
    GDROM_TRACE("WARNING: semi-unimplemented CD_SCD packet received:\n");
    for (idx = 0; idx < PKT_LEN; idx++)
        GDROM_TRACE("\t%02x\n", (unsigned)gdrom->pkt_buf[idx]);


    bufq_clear(gdrom);
    struct gdrom_bufq_node *node =
        (struct gdrom_bufq_node*)malloc(sizeof(struct gdrom_bufq_node));

    node->idx = 0;
    node->len = len;

    // TODO: fill in with real data instead of all zeroes
    memset(node->dat, 0, len);

    fifo_push(&gdrom->bufq, &node->fifo_node);

    gdrom_state_transfer_pio_read(gdrom, len);
}

static void gdrom_input_seek_packet(struct gdrom_ctxt *gdrom) {
    unsigned param_tp = gdrom->pkt_buf[1] & 0xf;
    unsigned seek_pt = (((unsigned)gdrom->pkt_buf[2]) << 16) |
        (((unsigned)gdrom->pkt_buf[3]) << 8) |
        (((unsigned)gdrom->pkt_buf[4]) << 24);

    char const *param_tp_str;
    switch (param_tp) {
    case 1:
        param_tp_str = "FAD";
        break;
    case 2:
        param_tp_str = "MSF";
        break;
    case 3:
        param_tp_str = "STOP";
        break;
    case 4:
        param_tp_str = "PAUSE";
        break;
    default:
        param_tp_str = "UNKNOWN/CORRUPT";
    }

    // CDDA playback isn't implemented yet, so we can't do anything here.
    GDROM_INFO("%s - CDDA SEEK command received.\n", __func__);
    GDROM_INFO("\tparam_tp = %s (%u)\n", param_tp_str, param_tp);
    GDROM_INFO("\tseek_pt = %06X\n", seek_pt);

    gdrom_delayed_processing(gdrom, GDROM_INT_DELAY);
}

static void gdrom_input_play_packet(struct gdrom_ctxt *gdrom) {
    unsigned param_tp = gdrom->pkt_buf[1] & 0x7;
    unsigned start = (((unsigned)gdrom->pkt_buf[2]) << 16) |
        (((unsigned)gdrom->pkt_buf[3]) << 8) |
        (((unsigned)gdrom->pkt_buf[4]) << 24);
    unsigned n_repeat = gdrom->pkt_buf[6] & 0xf;
    unsigned end = (((unsigned)gdrom->pkt_buf[8]) << 16) |
        (((unsigned)gdrom->pkt_buf[9]) << 8) |
        (((unsigned)gdrom->pkt_buf[10]) << 24);

    gdrom_delayed_processing(gdrom, GDROM_INT_DELAY);

    GDROM_INFO("%s - CDDA PLAY command received.\n", __func__);
    GDROM_INFO("\tparam_tp = 0x%02x\n", param_tp);
    GDROM_INFO("\tstart = 0x%04x\n", start);
    GDROM_INFO("\tend = 0x%04x\n", end);
    GDROM_INFO("\tn_repeat = %u\n", n_repeat);
}

unsigned gdrom_dma_prot_top(struct gdrom_ctxt *gdrom) {
    return (((gdrom->gdapro_reg & 0x7f00) >> 8) << 20) | 0x08000000;
}

unsigned gdrom_dma_prot_bot(struct gdrom_ctxt *gdrom) {
    return ((gdrom->gdapro_reg & 0x7f) << 20) | 0x080fffff;
}

void gdrom_read_data(struct gdrom_ctxt *gdrom, uint8_t *buf, unsigned n_bytes) {
    uint8_t *ptr = buf;

    if (gdrom->state != GDROM_STATE_PIO_READING) {
        GDROM_WARN("Game tried to read from GD-ROM data register before data "
                   "was ready\n");
        memset(buf, 0, n_bytes);
        return;
    }

    while (n_bytes--) {
        unsigned dat;
        if (bufq_consume_byte(gdrom, &dat) == 0 &&
            gdrom->meta.read.bytes_read < gdrom->data_byte_count) {
            *ptr++ = dat;
        } else {
            GDROM_ERROR("%s bufq is out of data!  returning 0\n", __func__);
            *ptr++ = 0;
        }
        gdrom->meta.read.bytes_read++;
    }

    if (gdrom->meta.read.bytes_read == gdrom->data_byte_count) {
        if (!gdrom->meta.read.byte_count) {
            // done transmitting data from gdrom to host - notify host
            GDROM_TRACE("DATA TRANSMIT COMPLETE.\n");
            gdrom->stat_reg.drq = false;
            gdrom->stat_reg.bsy = false;
            gdrom->stat_reg.drdy = true;
            gdrom->int_reason_reg.cod = true;
            gdrom->int_reason_reg.io = true;
            gdrom_state_transition(gdrom, GDROM_STATE_NORM);
            gdrom_delayed_processing(gdrom, GDROM_INT_DELAY);
        } else {
            GDROM_TRACE("MORE DATA TO FOLLOW\n");
            gdrom->stat_reg.drq = false;
            gdrom->stat_reg.bsy = true;
            gdrom_state_transition(gdrom, GDROM_STATE_PIO_READ_DELAY);
            gdrom_delayed_processing(gdrom, GDROM_INT_DELAY);
        }
    } else if (gdrom->meta.read.bytes_read > gdrom->data_byte_count) {
        error_set_feature("reading more data from the GD-ROM than is "
                          "available.\n");
        RAISE_ERROR(ERROR_UNIMPLEMENTED);
    }
}

void
gdrom_write_data(struct gdrom_ctxt *gdrom, uint8_t const *buf,
                 unsigned n_bytes) {
    uint32_t dat = 0;
    n_bytes = n_bytes < sizeof(dat) ? n_bytes : sizeof(dat);

    memcpy(&dat, buf, n_bytes);

    GDROM_TRACE("write 0x%04x to data register (%u bytes)\n",
                (unsigned)dat, (unsigned)n_bytes);

    if (gdrom->state == GDROM_STATE_INPUT_PKT) {
        gdrom->pkt_buf[gdrom->n_bytes_received] = dat & 0xff;
        gdrom->pkt_buf[gdrom->n_bytes_received + 1] = (dat >> 8) & 0xff;
        gdrom->n_bytes_received += 2;

        if (gdrom->n_bytes_received >= 12) {
            gdrom->n_bytes_received = 0;
            gdrom_input_packet(gdrom);
        }
    } else if (gdrom->state == GDROM_STATE_SET_MODE) {
        gdrom->set_mode_bytes_remaining -= n_bytes;
        GDROM_TRACE("received data for SET_MODE, %u bytes remaining\n",
                    gdrom->set_mode_bytes_remaining);

        if (gdrom->set_mode_bytes_remaining <= 0) {
            gdrom->stat_reg.drq = false;
            gdrom_state_transition(gdrom, GDROM_STATE_NORM);

            gdrom_delayed_processing(gdrom, GDROM_INT_DELAY);
        }
    }
}

/*
 * should return the type of disc in the drive (which will usually be
 * DISC_TYPE_GDROM)
 */
enum mount_disc_type gdrom_get_disc_type(void) {
    if (mount_check())
        return mount_get_disc_type();

    /*
     * this technically evaluates to DISC_TYPE_CDDA, but it doesn't matter
     * because anything that calls this function will be smart enough to check
     * the drive state and realize that there's nothing inserted.
     */
    return (enum mount_disc_type)0;
}

/*
 * return the state the physical drive is in (GDROM_STATE_NODISC,
 * GDROM_STATE_PAUSE, etc).
 */
enum gdrom_disc_state gdrom_get_drive_state(void) {
    if (mount_check()) {
        return GDROM_STATE_PAUSE;
    }
    return GDROM_STATE_NODISC;
}

void gdrom_start_dma(struct gdrom_ctxt *gdrom) {
    if (gdrom->dma_start_reg) {
        if (gdrom->state != GDROM_STATE_DMA_WAITING) {
            GDROM_ERROR("current GD-ROM state is %d\n", gdrom->state);
            RAISE_ERROR(ERROR_UNIMPLEMENTED);
        }

        gdrom->stat_reg.drq = false;
        gdrom->stat_reg.bsy = true;
        gdrom_complete_dma(gdrom);
    }
}

void gdrom_input_cmd(struct gdrom_ctxt *gdrom, unsigned cmd) {
    switch (cmd) {
    case GDROM_CMD_PKT:
        gdrom_cmd_begin_packet(gdrom);
        break;
    case GDROM_CMD_SET_FEAT:
        gdrom_cmd_set_features(gdrom);
        break;
    case GDROM_CMD_IDENTIFY:
        gdrom_cmd_identify(gdrom);
        break;
    case GDROM_CMD_ATA_IDENTIFY:
        /*
         * DreamShell uses this to probe if there's an HDD modded into this
         * Dreamcast.  AFAIK, ATAPI CD-ROM devs are supposed to set the check
         * bit in the status register so that's what we do here.  This behavior
         * has *not* been verified on real hardware.
         */
        LOG_ERROR("GD-ROM DRIVE RECEIVED ATA IDENTIFY COMMAND.  SETTING "
                  "CHECK BIT.\n");
        // intentional fall-through
    case GDROM_CMD_NOP:
        if (gdrom->gdrom_int_scheduled) {
            error_set_feature("using GDROM_CMD_NOP to abort during an interrupt");
            RAISE_ERROR(ERROR_UNIMPLEMENTED);
        } else {
            GDROM_INFO("ATA NOP command received\n");
            gdrom->stat_reg.bsy = false;
            gdrom->stat_reg.check = true;
            gdrom_state_transition(gdrom, GDROM_STATE_NORM);
            gdrom->error_reg.abrt = true;
            if (!gdrom->dev_ctrl_reg.nien)
                holly_raise_ext_int(HOLLY_EXT_INT_GDROM);
        }
        break;
    default:
        error_set_feature("unknown GD-ROM command");
        error_set_gdrom_command(cmd);
        RAISE_ERROR(ERROR_UNIMPLEMENTED);
    }
}

#define GDROM_ERROR_SENSE_KEY_SHIFT 4
#define GDROM_ERROR_SENSE_KEY_MASK (0xf << GDROM_ERROR_SENSE_KEY_SHIFT)

#define GDROM_ERROR_MCR_SHIFT 3
#define GDROM_ERROR_MCR_MASK (1 << GDROM_ERROR_MCR_SHIFT)

#define GDROM_ERROR_ABRT_SHIFT 2
#define GDROM_ERROR_ABRT_MASK (1 << GDROM_ERROR_ABRT_SHIFT)

#define GDROM_ERROR_EOMF_SHIFT 1
#define GDROM_ERROR_EOMF_MASK (1 << GDROM_ERROR_EOMF_SHIFT)

#define GDROM_ERROR_ILI_SHIFT 0
#define GDROM_ERROR_ILI_MASK (1 << GDROM_ERROR_ILI_SHIFT)

static reg32_t gdrom_get_error_reg(struct gdrom_error const *error_in) {
    reg32_t error_reg =
        (((reg32_t)error_in->sense_key) << GDROM_ERROR_SENSE_KEY_SHIFT) &
        GDROM_ERROR_SENSE_KEY_MASK;

    if (error_in->ili)
        error_reg |= GDROM_ERROR_ILI_MASK;
    if (error_in->eomf)
        error_reg |= GDROM_ERROR_EOMF_MASK;
    if (error_in->abrt)
        error_reg |= GDROM_ERROR_ABRT_MASK;
    if (error_in->mcr)
        error_reg |= GDROM_ERROR_MCR_MASK;

    return error_reg;
}


// ready to receive command
#define INT_REASON_COD_SHIFT 0
#define INT_REASON_COD_MASK (1 << INT_REASON_COD_SHIFT)

/*
 * ready to receive data from software to drive if set
 * ready to send data from drive to software if not set
 */
#define INT_REASON_IO_SHIFT 1
#define INT_REASON_IO_MASK (1 << INT_REASON_IO_SHIFT)

static reg32_t gdrom_get_int_reason_reg(struct gdrom_int_reason *int_reason_in) {
    reg32_t reg_out = 0;

    if (int_reason_in->cod)
        reg_out |= INT_REASON_COD_MASK;
    if (int_reason_in->io)
        reg_out |= INT_REASON_IO_MASK;

    return reg_out;
}

#define SEC_NUM_STATUS_SHIFT 0
#define SEC_NUM_STATUS_MASK (0xf << SEC_NUM_STATUS_SHIFT)

#define SEC_NUM_DISC_TYPE_SHIFT 4
#define SEC_NUM_DISC_TYPE_MASK (0xf << SEC_NUM_DISC_TYPE_SHIFT)

#define SEC_NUM_FMT_SHIFT 4
#define SEC_NUM_FMT_MASK (0xf << SEC_NUM_FMT_SHIFT)

////////////////////////////////////////////////////////////////////////////////
//
// Status register flags
//
////////////////////////////////////////////////////////////////////////////////

// the drive is processing a command
#define GDROM_STAT_BSY_SHIFT 7
#define GDROM_STAT_BSY_MASK (1 << GDROM_STAT_BSY_SHIFT)

// response to ATA command is possible
#define GDROM_STAT_DRDY_SHIFT 6
#define GDROM_STAT_DRDY_MASK (1 << GDROM_STAT_DRDY_SHIFT)

// drive fault
#define GDROM_STAT_DF_SHIFT 5
#define GDROM_STAT_DF_MASK (1 << GDROM_STAT_DF_SHIFT)

// seek processing is complete
#define GDROM_STAT_DSC_SHIFT 4
#define GDROM_STAT_DSC_MASK (1 << GDROM_STAT_DSC_SHIFT)

// data transfer possible
#define GDROM_STAT_DRQ_SHIFT 3
#define GDROM_STAT_DRQ_MASK (1 << GDROM_STAT_DRQ_SHIFT)

// correctable error flag
#define GDROM_STAT_CORR_SHIFT 2
#define GDROM_STAT_CORR_MASK (1 << GDROM_STAT_CORR_SHIFT)

// error flag
#define GDROM_STAT_CHECK_SHIFT 0
#define GDROM_STAT_CHECK_MASK (1 << GDROM_STAT_CHECK_SHIFT)

static reg32_t gdrom_get_status_reg(struct gdrom_status const *stat_in) {
    reg32_t stat_reg = 0;

    if (stat_in->bsy)
        stat_reg |= GDROM_STAT_BSY_MASK;
    if (stat_in->drdy)
        stat_reg |= GDROM_STAT_DRDY_MASK;
    if (stat_in->df)
        stat_reg |= GDROM_STAT_DF_MASK;
    if (stat_in->dsc)
        stat_reg |= GDROM_STAT_DSC_MASK;
    if (stat_in->drq)
        stat_reg |= GDROM_STAT_DRQ_MASK;
    if (stat_in->corr)
        stat_reg |= GDROM_STAT_CORR_MASK;
    if (stat_in->check)
        stat_reg |= GDROM_STAT_CHECK_MASK;

    return stat_reg;
}

////////////////////////////////////////////////////////////////////////////////
//
// feature register flags
//
////////////////////////////////////////////////////////////////////////////////

#define FEAT_REG_DMA_SHIFT 0
#define FEAT_REG_DMA_MASK (1 << FEAT_REG_DMA_SHIFT)

static void
gdrom_set_features_reg(struct gdrom_features *features_out, reg32_t feat_reg) {
    if (feat_reg & FEAT_REG_DMA_MASK)
        features_out->dma_enable = true;
    else
        features_out->dma_enable = false;

    if ((feat_reg & 0x7f) == 3)
        features_out->set_feat_enable = true;
    else
        features_out->set_feat_enable = false;
}

////////////////////////////////////////////////////////////////////////////////
//
// Transfer Modes (for the sector count register in GDROM_CMD_SEAT_FEAT)
//
////////////////////////////////////////////////////////////////////////////////

#define TRANS_MODE_PIO_DFLT_MASK        0xfe
#define TRANS_MODE_PIO_DFLT_VAL         0x00

#define TRANS_MODE_PIO_FLOW_CTRL_MASK   0xf8
#define TRANS_MODE_PIO_FLOW_CTRL_VAL    0x08

#define TRANS_MODE_SINGLE_WORD_DMA_MASK 0xf8
#define TRANS_MODE_SINGLE_WORD_DMA_VAL  0x10

#define TRANS_MODE_MULTI_WORD_DMA_MASK  0xf8
#define TRANS_MODE_MULTI_WORD_DMA_VAL   0x20

#define TRANS_MODE_PSEUDO_DMA_MASK      0xf8
#define TRANS_MODE_PSEUDO_DMA_VAL       0x18

#define SECT_CNT_MODE_VAL_SHIFT 0
#define SECT_CNT_MODE_VAL_MASK (0xf << SECT_CNT_MODE_VAL_SHIFT)

////////////////////////////////////////////////////////////////////////////////
//
// Device control register flags
//
////////////////////////////////////////////////////////////////////////////////

#define DEV_CTRL_NIEN_SHIFT 1
#define DEV_CTRL_NIEN_MASK (1 << DEV_CTRL_NIEN_SHIFT)

#define DEV_CTRL_SRST_SHIFT 2
#define DEV_CTRL_SRST_MASK (1 << DEV_CTRL_SRST_SHIFT)

static void
gdrom_set_dev_ctrl_reg(struct gdrom_dev_ctrl *dev_ctrl_out,
                       reg32_t dev_ctrl_reg) {
    dev_ctrl_out->nien = (bool)(dev_ctrl_reg & DEV_CTRL_NIEN_MASK);
    dev_ctrl_out->srst = (bool)(dev_ctrl_reg & DEV_CTRL_SRST_MASK);
}

static void
gdrom_set_sect_cnt_reg(struct gdrom_sector_count *sect_cnt_out,
                       reg32_t sect_cnt_reg) {
    unsigned mode_val =
        (sect_cnt_reg & SECT_CNT_MODE_VAL_MASK) >> SECT_CNT_MODE_VAL_SHIFT;
    if ((sect_cnt_reg & TRANS_MODE_PIO_DFLT_MASK) ==
        TRANS_MODE_PIO_DFLT_VAL) {
        sect_cnt_out->trans_mode = TRANS_MODE_PIO_DFLT;
    } else if ((sect_cnt_reg & TRANS_MODE_PIO_FLOW_CTRL_MASK) ==
               TRANS_MODE_PIO_FLOW_CTRL_VAL) {
        sect_cnt_out->trans_mode = TRANS_MODE_PIO_FLOW_CTRL;
    } else if ((sect_cnt_reg & TRANS_MODE_SINGLE_WORD_DMA_MASK) ==
               TRANS_MODE_SINGLE_WORD_DMA_VAL) {
        sect_cnt_out->trans_mode = TRANS_MODE_SINGLE_WORD_DMA;
    } else if ((sect_cnt_reg & TRANS_MODE_MULTI_WORD_DMA_MASK) ==
               TRANS_MODE_MULTI_WORD_DMA_VAL) {
        sect_cnt_out->trans_mode = TRANS_MODE_MULTI_WORD_DMA;
    } else if ((sect_cnt_reg & TRANS_MODE_PSEUDO_DMA_MASK) ==
               TRANS_MODE_PSEUDO_DMA_VAL) {
        sect_cnt_out->trans_mode = TRANS_MODE_PSEUDO_DMA;
    } else {
        // TODO: maybe this should be a soft warning instead of an error
        GDROM_TRACE("unrecognized transfer mode (sec_cnt_reg is 0x%08x)\n",
                    sect_cnt_reg);
        error_set_feature("unrecognized transfer mode\n");
        RAISE_ERROR(ERROR_UNIMPLEMENTED);
    }

    sect_cnt_out->mode_val = mode_val;
}

static void
gdrom_pre_read(struct gdrom_ctxt *gdrom, addr32_t addr, unsigned n_bytes) {
#ifdef INVARIANTS
    /*
     * non-aligned access should not even be possible due to the way SH-4
     * encodes offsets.
     */
    if ((addr - 0x5f7000) % 4)
        RAISE_ERROR(ERROR_UNIMPLEMENTED);
#endif

    unsigned idx = (addr - ADDR_GDROM_FIRST) / 4;
    switch(idx) {
    case ATA_REG_RW_DATA:
        gdrom->regs[ATA_REG_RW_DATA] = 0;
        gdrom_read_data(gdrom, (uint8_t*)(gdrom->regs + ATA_REG_RW_DATA), n_bytes);
        GDROM_TRACE("read 0x%08x (%u bytes) from data register\n",
                    (unsigned)gdrom->regs[ATA_REG_RW_DATA], n_bytes);
        break;
    case ATA_REG_R_ERROR:
        gdrom->regs[ATA_REG_R_ERROR] = gdrom_get_error_reg(&gdrom->error_reg);
        GDROM_TRACE("read 0x%02x from error register\n",
                    (unsigned)gdrom->regs[ATA_REG_R_ERROR]);
        break;
    case ATA_REG_R_INT_REASON:
        gdrom->regs[ATA_REG_R_INT_REASON] =
            gdrom_get_int_reason_reg(&gdrom->int_reason_reg);
        GDROM_TRACE("int_reason is 0x%08x\n",
                    (unsigned)gdrom->regs[ATA_REG_R_INT_REASON]);
        break;
    case ATA_REG_R_SEC_NUM:
        gdrom->regs[ATA_REG_R_SEC_NUM] =
            ((uint8_t)gdrom_get_drive_state() << SEC_NUM_STATUS_SHIFT) |
            ((uint8_t)gdrom_get_disc_type() << SEC_NUM_DISC_TYPE_SHIFT);
        break;
    case ATA_REG_RW_BYTE_CNT_LO:
        gdrom->regs[ATA_REG_RW_BYTE_CNT_LO] =
            gdrom->data_byte_count & 0xff;
        GDROM_TRACE("read 0x%02x from byte_count_low\n",
                    (unsigned)gdrom->regs[ATA_REG_RW_BYTE_CNT_LO]);
        if (gdrom->data_byte_count > UINT16_MAX) {
            error_set_feature("reading more than 64 kilobytes from GD-ROM");
            RAISE_ERROR(ERROR_UNIMPLEMENTED);
        }
        break;
    case ATA_REG_RW_BYTE_CNT_HI:
        gdrom->regs[ATA_REG_RW_BYTE_CNT_HI] =
            (gdrom->data_byte_count & 0xff00) >> 8;
        GDROM_TRACE("read 0x%02x from byte_count_high\n",
                    (unsigned)gdrom->regs[ATA_REG_RW_BYTE_CNT_HI]);
        if (gdrom->data_byte_count > UINT16_MAX) {
            error_set_feature("reading more than 64 kilobytes from GD-ROM");
            RAISE_ERROR(ERROR_UNIMPLEMENTED);
        }
        break;
    case ATA_REG_RW_DRIVE_SEL:
        gdrom->regs[ATA_REG_RW_DRIVE_SEL] = gdrom->drive_sel_reg;
        break;
    case ATA_REG_R_STATUS:
        holly_clear_ext_int(HOLLY_EXT_INT_GDROM);

        gdrom->regs[ATA_REG_R_STATUS] =
            gdrom_get_status_reg(&gdrom->stat_reg);
        GDROM_TRACE("read 0x%02x from status register\n",
                    (unsigned)gdrom->regs[ATA_REG_R_STATUS]);
        break;
    case ATA_REG_ALT_STATUS:
        gdrom->regs[ATA_REG_ALT_STATUS] =
            gdrom_get_status_reg(&gdrom->stat_reg);
        GDROM_TRACE("read 0x%02x from alternate status register\n",
                    (unsigned)gdrom->regs[ATA_REG_ALT_STATUS]);
        break;
    default:
        error_set_address(addr);
        error_set_length(n_bytes);
        RAISE_ERROR(ERROR_UNIMPLEMENTED);
    }
}

static void
gdrom_post_write(struct gdrom_ctxt *gdrom, addr32_t addr, unsigned n_bytes) {
#ifdef INVARIANTS
    /*
     * non-aligned access should not even be possible due to the way SH-4
     * encodes offsets.
     */
    if ((addr - 0x5f7000) % 4)
        RAISE_ERROR(ERROR_UNIMPLEMENTED);
#endif

    unsigned idx = (addr - ADDR_GDROM_FIRST) / 4;
    switch(idx) {
    case ATA_REG_RW_DATA:
        gdrom_write_data(gdrom, (uint8_t*)(gdrom->regs + ATA_REG_RW_DATA),
                         n_bytes);
        break;
    case ATA_REG_W_FEAT:
        GDROM_TRACE("write 0x%08x to the features register\n",
                    (unsigned)gdrom->regs[ATA_REG_W_FEAT]);
        gdrom_set_features_reg(&gdrom->feat_reg, gdrom->regs[ATA_REG_W_FEAT]);
        break;
    case ATA_REG_W_SEC_CNT:
        GDROM_TRACE("Write %08x to sec_cnt_reg\n",
                    (unsigned)gdrom->regs[ATA_REG_W_SEC_CNT]);
        gdrom_set_sect_cnt_reg(&gdrom->sect_cnt_reg,
                               gdrom->regs[ATA_REG_W_SEC_CNT]);
        break;

    case ATA_REG_RW_BYTE_CNT_LO:
        GDROM_TRACE("write 0x%02x to byte_count_low\n",
                    (unsigned)(gdrom->regs[ATA_REG_RW_BYTE_CNT_LO] & 0xff));
        gdrom->data_byte_count = (gdrom->data_byte_count & ~0xff) |
            (gdrom->regs[ATA_REG_RW_BYTE_CNT_LO] & 0xff);
        break;
    case ATA_REG_RW_BYTE_CNT_HI:
        GDROM_TRACE("write 0x%02x to byte_count_high\n",
                    (unsigned)((gdrom->regs[ATA_REG_RW_BYTE_CNT_HI] & 0xff) <<
                               8));
        gdrom->data_byte_count =
            (gdrom->data_byte_count & ~0xff00) |
            ((gdrom->regs[ATA_REG_RW_BYTE_CNT_HI] & 0xff) << 8);
        break;
    case ATA_REG_RW_DRIVE_SEL:
        gdrom->drive_sel_reg = gdrom->regs[ATA_REG_RW_DRIVE_SEL];
        break;
    case ATA_REG_W_CMD:
        GDROM_TRACE("write 0x%x to command register (4 bytes)\n",
                    (unsigned)gdrom->regs[ATA_REG_W_CMD]);
        gdrom_input_cmd(gdrom, gdrom->regs[ATA_REG_W_CMD]);
        break;
    case ATA_REG_ALT_STATUS:
        gdrom_set_dev_ctrl_reg(&gdrom->dev_ctrl_reg,
                               gdrom->regs[ATA_REG_ALT_STATUS]);
        GDROM_TRACE("Write %08x to dev_ctrl_reg\n",
                    (unsigned)gdrom->regs[ATA_REG_ALT_STATUS]);
        break;
    }
}

static void gdrom_check_addr(addr32_t addr, size_t n_bytes) {
    addr32_t first = addr;
    addr32_t last = addr + (n_bytes - 1);

    if (!(first >= 0x5f7000 && first <= 0x5f70ff) ||
        !(last >= 0x5f7000 && last <= 0x5f70ff)) {
        error_set_address(addr);
        error_set_length(n_bytes);
        RAISE_ERROR(ERROR_MEM_OUT_OF_BOUNDS);
    }
}

static float gdrom_reg_read_float(addr32_t addr, void *ctxt) {
    float out;
    struct gdrom_ctxt *gdrom = (struct gdrom_ctxt*)ctxt;
    gdrom_check_addr(addr, sizeof(out));
    gdrom_pre_read(gdrom, addr, sizeof(out));
    memcpy(&out, ((uint8_t*)gdrom->regs) + (addr - ADDR_GDROM_FIRST),
           sizeof(out));
    return out;
}

static void gdrom_reg_write_float(addr32_t addr, float val, void *ctxt) {
    struct gdrom_ctxt *gdrom = (struct gdrom_ctxt*)ctxt;
    gdrom_check_addr(addr, sizeof(val));
    memcpy(((uint8_t*)gdrom->regs) + (addr - ADDR_GDROM_FIRST), &val, sizeof(val));
    gdrom_post_write(gdrom, addr, sizeof(val));
}

static double gdrom_reg_read_double(addr32_t addr, void *ctxt) {
    double out;
    struct gdrom_ctxt *gdrom = (struct gdrom_ctxt*)ctxt;
    gdrom_check_addr(addr, sizeof(out));
    gdrom_pre_read(gdrom, addr, sizeof(out));
    memcpy(&out, ((uint8_t*)gdrom->regs) + (addr - ADDR_GDROM_FIRST),
           sizeof(out));
    return out;
}

static void gdrom_reg_write_double(addr32_t addr, double val, void *ctxt) {
    struct gdrom_ctxt *gdrom = (struct gdrom_ctxt*)ctxt;
    gdrom_check_addr(addr, sizeof(val));
    memcpy(((uint8_t*)gdrom->regs) + (addr - ADDR_GDROM_FIRST), &val, sizeof(val));
    gdrom_post_write(gdrom, addr, sizeof(val));
}

static uint8_t gdrom_reg_read_8(addr32_t addr, void *ctxt) {
    uint8_t out;
    struct gdrom_ctxt *gdrom = (struct gdrom_ctxt*)ctxt;
    gdrom_check_addr(addr, sizeof(out));
    gdrom_pre_read(gdrom, addr, sizeof(out));
    memcpy(&out, ((uint8_t*)gdrom->regs) + (addr - ADDR_GDROM_FIRST),
           sizeof(out));
    return out;
}

static void gdrom_reg_write_8(addr32_t addr, uint8_t val, void *ctxt) {
    struct gdrom_ctxt *gdrom = (struct gdrom_ctxt*)ctxt;
    gdrom_check_addr(addr, sizeof(val));
    memcpy(((uint8_t*)gdrom->regs) + (addr - ADDR_GDROM_FIRST), &val, sizeof(val));
    gdrom_post_write(gdrom, addr, sizeof(val));
}

static uint16_t gdrom_reg_read_16(addr32_t addr, void *ctxt) {
    uint16_t out;
    struct gdrom_ctxt *gdrom = (struct gdrom_ctxt*)ctxt;
    gdrom_check_addr(addr, sizeof(out));
    gdrom_pre_read(gdrom, addr, sizeof(out));
    memcpy(&out, ((uint8_t*)gdrom->regs) + (addr - ADDR_GDROM_FIRST),
           sizeof(out));
    return out;
}

static void gdrom_reg_write_16(addr32_t addr, uint16_t val, void *ctxt) {
    struct gdrom_ctxt *gdrom = (struct gdrom_ctxt*)ctxt;
    gdrom_check_addr(addr, sizeof(val));
    memcpy(((uint8_t*)gdrom->regs) + (addr - ADDR_GDROM_FIRST), &val, sizeof(val));
    gdrom_post_write(gdrom, addr, sizeof(val));
}

static uint32_t gdrom_reg_read_32(addr32_t addr, void *ctxt) {
    uint32_t out;
    struct gdrom_ctxt *gdrom = (struct gdrom_ctxt*)ctxt;
    gdrom_check_addr(addr, sizeof(out));
    gdrom_pre_read(gdrom, addr, sizeof(out));
    memcpy(&out, ((uint8_t*)gdrom->regs) + (addr - ADDR_GDROM_FIRST),
           sizeof(out));
    return out;
}

static void gdrom_reg_write_32(addr32_t addr, uint32_t val, void *ctxt) {
    struct gdrom_ctxt *gdrom = (struct gdrom_ctxt*)ctxt;
    gdrom_check_addr(addr, sizeof(val));
    memcpy(((uint8_t*)gdrom->regs) + (addr - ADDR_GDROM_FIRST), &val, sizeof(val));
    gdrom_post_write(gdrom, addr, sizeof(val));
}

static uint32_t
gdrom_gdapro_mmio_read(struct mmio_region_g1_reg_32 *region,
                       unsigned idx, void *ctxt) {
    struct gdrom_ctxt *gdrom_ctxt = (struct gdrom_ctxt*)ctxt;
    GDROM_TRACE("read %08x from GDAPRO\n", gdrom_ctxt->gdapro_reg);
    return gdrom_ctxt->gdapro_reg;
}

static void
gdrom_gdapro_mmio_write(struct mmio_region_g1_reg_32 *region,
                        unsigned idx, uint32_t val, void *ctxt) {
    // check security code
    if ((val & 0xffff0000) != 0x88430000)
        return;

    struct gdrom_ctxt *gdrom_ctxt = (struct gdrom_ctxt*)ctxt;
    gdrom_ctxt->gdapro_reg = val;

    GDROM_TRACE("GDAPRO (0x%08x) - allowing writes from 0x%08x through "
                "0x%08x\n",
                gdrom_ctxt->gdapro_reg,
                gdrom_dma_prot_top(gdrom_ctxt), gdrom_dma_prot_bot(gdrom_ctxt));
}

static uint32_t
gdrom_g1gdrc_mmio_read(struct mmio_region_g1_reg_32 *region,
                       unsigned idx, void *ctxt) {
    struct gdrom_ctxt *gdrom_ctxt = (struct gdrom_ctxt*)ctxt;
    GDROM_TRACE("read %08x from G1GDRC\n", gdrom_ctxt->g1gdrc_reg);
    return gdrom_ctxt->g1gdrc_reg;
}

static void
gdrom_g1gdrc_mmio_write(struct mmio_region_g1_reg_32 *region,
                        unsigned idx, uint32_t val, void *ctxt) {
    struct gdrom_ctxt *gdrom_ctxt = (struct gdrom_ctxt*)ctxt;
    GDROM_TRACE("write %08x to G1GDRC\n", gdrom_ctxt->g1gdrc_reg);
    gdrom_ctxt->g1gdrc_reg = val;
}

static uint32_t
gdrom_gdstar_mmio_read(struct mmio_region_g1_reg_32 *region,
                       unsigned idx, void *ctxt) {
    struct gdrom_ctxt *gdrom_ctxt = (struct gdrom_ctxt*)ctxt;
    GDROM_TRACE("read %08x from GDSTAR\n", gdrom_ctxt->dma_start_addr_reg);
    return gdrom_ctxt->dma_start_addr_reg;
}

static void
gdrom_gdstar_mmio_write(struct mmio_region_g1_reg_32 *region,
                        unsigned idx, uint32_t val, void *ctxt) {
    struct gdrom_ctxt *gdrom_ctxt = (struct gdrom_ctxt*)ctxt;
    gdrom_ctxt->dma_start_addr_reg = val;
    gdrom_ctxt->dma_start_addr_reg &= ~0xe0000000;
    GDROM_TRACE("write %08x to GDSTAR\n", gdrom_ctxt->dma_start_addr_reg);
}

static uint32_t
gdrom_gdlen_mmio_read(struct mmio_region_g1_reg_32 *region,
                      unsigned idx, void *ctxt) {
    struct gdrom_ctxt *gdrom_ctxt = (struct gdrom_ctxt*)ctxt;
    GDROM_TRACE("read %08x from GDLEN\n", gdrom_ctxt->dma_len_reg);
    return gdrom_ctxt->dma_len_reg;
}

static void
gdrom_gdlen_mmio_write(struct mmio_region_g1_reg_32 *region,
                       unsigned idx, uint32_t val, void *ctxt) {
    struct gdrom_ctxt *gdrom_ctxt = (struct gdrom_ctxt*)ctxt;
    gdrom_ctxt->dma_len_reg = val;
    GDROM_TRACE("write %08x to GDLEN\n", gdrom_ctxt->dma_len_reg);
}

static uint32_t
gdrom_gddir_mmio_read(struct mmio_region_g1_reg_32 *region,
                      unsigned idx, void *ctxt) {
    struct gdrom_ctxt *gdrom_ctxt = (struct gdrom_ctxt*)ctxt;
    GDROM_TRACE("read %08x from GDDIR\n", gdrom_ctxt->dma_dir_reg);
    return gdrom_ctxt->dma_dir_reg;
}

static void
gdrom_gddir_mmio_write(struct mmio_region_g1_reg_32 *region,
                       unsigned idx, uint32_t val, void *ctxt) {
    struct gdrom_ctxt *gdrom_ctxt = (struct gdrom_ctxt*)ctxt;
    gdrom_ctxt->dma_dir_reg = val;
    GDROM_TRACE("write %08x to GDDIR\n", gdrom_ctxt->dma_dir_reg);
}

static uint32_t
gdrom_gden_mmio_read(struct mmio_region_g1_reg_32 *region,
                     unsigned idx, void *ctxt) {
    struct gdrom_ctxt *gdrom_ctxt = (struct gdrom_ctxt*)ctxt;
    GDROM_TRACE("read %08x from GDEN\n", gdrom_ctxt->dma_en_reg);
    return gdrom_ctxt->dma_en_reg;
}

static void
gdrom_gden_mmio_write(struct mmio_region_g1_reg_32 *region,
                      unsigned idx, uint32_t val, void *ctxt) {
    struct gdrom_ctxt *gdrom_ctxt = (struct gdrom_ctxt*)ctxt;
    gdrom_ctxt->dma_en_reg = val;
    GDROM_TRACE("write %08x to GDEN\n", gdrom_ctxt->dma_en_reg);
}

static uint32_t
gdrom_gdst_reg_read_handler(struct mmio_region_g1_reg_32 *region,
                            unsigned idx, void *ctxt) {
    struct gdrom_ctxt *gdrom_ctxt = (struct gdrom_ctxt*)ctxt;
    GDROM_TRACE("read %08x from GDST\n", gdrom_ctxt->dma_start_reg);
    return gdrom_ctxt->dma_start_reg;
}

static void
gdrom_gdst_reg_write_handler(struct mmio_region_g1_reg_32 *region,
                             unsigned idx, uint32_t val, void *ctxt) {
    struct gdrom_ctxt *gdrom_ctxt = (struct gdrom_ctxt*)ctxt;
    gdrom_ctxt->dma_start_reg = val;
    GDROM_TRACE("write %08x to GDST\n", gdrom_ctxt->dma_start_reg);
    gdrom_start_dma(gdrom_ctxt);
}

static void gdrom_dma_progress_update(struct gdrom_ctxt *gdrom) {
    if (gdrom->state == GDROM_STATE_DMA_READING) {
        dc_cycle_stamp_t stamp = clock_cycle_stamp(gdrom->clk);
        dc_cycle_stamp_t delta = stamp - gdrom->dma_start_stamp;

        if (delta < gdrom->dma_delay) {
            gdrom->gdlend_reg =
                ((double)delta / (double)gdrom->dma_delay) * gdrom->gdlend_final;
        } else {
            gdrom->gdlend_reg = gdrom->gdlend_final;
        }
        if (gdrom->gdlend_reg >= gdrom->gdlend_final)
            gdrom->gdlend_reg = gdrom->gdlend_final;
    }
}

static uint32_t
gdrom_gdlend_mmio_read(struct mmio_region_g1_reg_32 *region,
                       unsigned idx, void *ctxt) {
    struct gdrom_ctxt *gdrom = (struct gdrom_ctxt*)ctxt;
    gdrom_dma_progress_update(gdrom);

    GDROM_TRACE("read %08x from GDLEND\n", gdrom->gdlend_reg);
    return gdrom->gdlend_reg;
}

static uint32_t
gdrom_gdstard_reg_read_handler(struct mmio_region_g1_reg_32 *region,
                               unsigned idx, void *ctxt) {
    struct gdrom_ctxt *gdrom = (struct gdrom_ctxt*)ctxt;
    gdrom_dma_progress_update(gdrom);

    uint32_t val = gdrom->gdlend_reg + gdrom->dma_start_addr_reg;
    GDROM_TRACE("read %08x from GDSTARD\n", val);
    return val;
}
