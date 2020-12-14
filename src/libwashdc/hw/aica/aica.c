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

/*
 * Yamaha AICA Super-Intelligent Sound Processor.
 *
 * This implementation is based on Neill Corlett's AICA notes and a little bit
 * of my own experimentation.
 */

#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>

#include "sound.h"
#include "log.h"
#include "dc_sched.h"
#include "washdc/error.h"
#include "intmath.h"
#include "hw/arm7/arm7.h"
#include "hw/sys/holly_intc.h"
#include "adpcm.h"
#include "intmath.h"
#include "compiler_bullshit.h"

#include "aica.h"

// fixed-point format used for attenuation scaling
typedef uint32_t aica_atten;
#define AICA_ATTEN_SHIFT 16
#define AICA_ATTEN_UNIT (1 << AICA_ATTEN_SHIFT)

/*
 * TODO: I'm really only assuming this is 44.1KHz because that's the standard,
 * I don't actually know if this is correct.
 */
#define AICA_SAMPLE_FREQ 44100
#define AICA_EXTERNAL_FREQ (1 * AICA_SAMPLE_FREQ)
#define AICA_FREQ_RATIO (AICA_EXTERNAL_FREQ / AICA_SAMPLE_FREQ)

/*
 * TODO: SCHED_FREQUENCY is not an integer multiple of AICA_SAMPLE_FREQ, so
 * there will be some inaccuracies here.
 */
#define TICKS_PER_SAMPLE (SCHED_FREQUENCY / AICA_SAMPLE_FREQ)

#define AICA_CHAN_PLAY_CTRL 0x0000
#define AICA_CHAN_SAMPLE_ADDR_LOW 0x0004
#define AICA_CHAN_LOOP_START 0x0008
#define AICA_CHAN_LOOP_END 0x000c
#define AICA_CHAN_AMP_ENV1 0x0010
#define AICA_CHAN_AMP_ENV2 0x0014
#define AICA_CHAN_SAMPLE_RATE_PITCH 0x0018
#define AICA_CHAN_LFO_CTRL 0x001c
#define AICA_CHAN_DSP_SEND 0x0020
#define AICA_CHAN_DIR_PAN_VOL_SEND 0x0024
#define AICA_CHAN_LPF1_VOL 0x0028
#define AICA_CHAN_LPF2 0x002c
#define AICA_CHAN_LPF3 0x0030
#define AICA_CHAN_LPF4 0x0034
#define AICA_CHAN_LPF5 0x0038
#define AICA_CHAN_LPF6 0x003c
#define AICA_CHAN_LPF7 0x0040
#define AICA_CHAN_LPF8 0x0044

#define AICA_MASTER_VOLUME 0x2800

#define AICA_ARM7_RST 0x2c00

#define AICA_RINGBUFFER_ADDRESS 0x2804

#define AICA_PLAYSTATUS 0x2810

#define AICA_PLAYPOS 0x2814

#define AICA_UNKNOWN_2880 0x2880

#define AICA_TIMERA_CTRL 0x2890
#define AICA_TIMERB_CTRL 0x2894
#define AICA_TIMERC_CTRL 0x2898

#define AICA_SCILV0 0x28a8
#define AICA_SCILV1 0x28ac
#define AICA_SCILV2 0x28b0

// interrupt enable
#define AICA_SCIEB 0x289c

// interrupt pending
#define AICA_SCIPD 0x28a0

// interrupt reset
#define AICA_SCIRE 0x28a4

// SH4 interrupt enable
#define AICA_MCIEB 0x28b4

// SH4 interrupt pending
#define AICA_MCIPD 0x28b8

// SH4 interrupt reset
#define AICA_MCIRE 0x28bc

#define AICA_MIDI_INPUT 0x2808

#define AICA_INTREQ 0x2d00

#define AICA_INTCLEAR 0x2d04

#define AICA_CHANINFOREQ 0x280c

#define AICA_INT_EXTERNAL_SHIFT 0
#define AICA_INT_EXTERNAL_MASK (1 << AICA_INT_EXTERNAL_SHIFT)

#define AICA_INT_MIDI_IN_SHIFT 3
#define AICA_INT_MIDI_IN_MASK (1 << AICA_INT_MIDI_IN_SHIFT)

#define AICA_INT_DMA_SHIFT 4
#define AICA_INT_DMA_MASK (1 << AICA_INT_DMA_SHIFT)

#define AICA_INT_CPU_SHIFT 5
#define AICA_INT_CPU_MASK (1 << AICA_INT_CPU_SHIFT)

#define AICA_INT_TIMA_SHIFT 6
#define AICA_INT_TIMA_MASK (1 << AICA_INT_TIMA_SHIFT)

#define AICA_INT_TIMB_SHIFT 7
#define AICA_INT_TIMB_MASK (1 << AICA_INT_TIMB_SHIFT)

#define AICA_INT_TIMC_SHIFT 8
#define AICA_INT_TIMC_MASK (1 << AICA_INT_TIMC_SHIFT)

#define AICA_INT_MIDI_OUT_SHIFT 9
#define AICA_INT_MIDI_OUT_MASK (1 << AICA_INT_MIDI_OUT_SHIFT)

#define AICA_INT_SAMPLE_INTERVAL_SHIFT 10
#define AICA_INT_SAMPLE_INTERVAL_MASK (1 << AICA_INT_SAMPLE_INTERVAL_SHIFT)

// Mask of all the interrupt bits that we care about
#define AICA_ALL_INT_MASK (AICA_INT_SAMPLE_INTERVAL_MASK |      \
                           AICA_INT_MIDI_OUT_MASK |             \
                           AICA_INT_TIMC_MASK |                 \
                           AICA_INT_TIMB_MASK |                 \
                           AICA_INT_TIMA_MASK |                 \
                           AICA_INT_CPU_MASK |                  \
                           AICA_INT_DMA_MASK |                  \
                           AICA_INT_MIDI_IN_MASK |              \
                           AICA_INT_EXTERNAL_MASK)

/*
 * 0 is probably the correct value for this since this interrupt is actually
 * triggered by software on a different CPU
 */
#define AICA_SH4_INT_DELAY 0

static DEF_ERROR_INT_ATTR(channel)

static void raise_aica_sh4_int(struct aica *aica);
static void post_delay_raise_aica_sh4_int(struct SchedEvent *event);

// If this is defined, WashingtonDC will panic on unrecognized AICA addresses.
#define AICA_PEDANTIC

#ifdef ENABLE_LOG_DEBUG
static char const *aica_chan_reg_name(int idx);
#endif

static void aica_update_interrupts(struct aica *aica);

static void
aica_sys_reg_read(struct aica *aica, addr32_t addr,
                  void *out, unsigned n_bytes, bool from_sh4);
static void aica_sys_reg_write(struct aica *aica, addr32_t addr,
                               void const *val_in, unsigned n_bytes,
                               bool from_sh4);

static float aica_sys_read_float(addr32_t addr, void *ctxt);
static void aica_sys_write_float(addr32_t addr, float val, void *ctxt);
static double aica_sys_read_double(addr32_t addr, void *ctxt);
static void aica_sys_write_double(addr32_t addr, double val, void *ctxt);
static uint32_t aica_sys_read_32(addr32_t addr, void *ctxt);
static void aica_sys_write_32(addr32_t addr, uint32_t val, void *ctxt);
static uint16_t aica_sys_read_16(addr32_t addr, void *ctxt);
static void aica_sys_write_16(addr32_t addr, uint16_t val, void *ctxt);
static uint8_t aica_sys_read_8(addr32_t addr, void *ctxt);
static void aica_sys_write_8(addr32_t addr, uint8_t val, void *ctxt);

static void aica_sys_channel_read(struct aica *aica, void *dst,
                                  uint32_t addr, unsigned len);
static void aica_dsp_mixer_read(struct aica *aica, void *dst,
                                uint32_t addr, unsigned len);
static void aica_dsp_reg_read(struct aica *aica, void *dst,
                              uint32_t addr, unsigned len);
static void aica_sys_channel_write(struct aica *aica, void const *src,
                                   uint32_t addr, unsigned len);
static void aica_dsp_mixer_write(struct aica *aica, void const *src,
                                 uint32_t addr, unsigned len);
static void aica_dsp_reg_write(struct aica *aica, void const *src,
                               uint32_t addr, unsigned len);

WASHDC_UNUSED
static void aica_unsched_all_timers(struct aica *aica);

WASHDC_UNUSED
static void aica_sched_all_timers(struct aica *aica);

static void aica_unsched_timer(struct aica *aica, unsigned tim_idx);
static void aica_sched_timer(struct aica *aica, unsigned tim_idx);

static void aica_sync_timer(struct aica *aica, unsigned tim_idx);
static dc_cycle_stamp_t aica_get_sample_count(struct aica *aica);

static void
on_timer_ctrl_write(struct aica *aica, unsigned tim_idx, uint32_t val);

static void
aica_timer_a_handler(struct SchedEvent *evt);
static void
aica_timer_b_handler(struct SchedEvent *evt);
static void
aica_timer_c_handler(struct SchedEvent *evt);

static void aica_timer_handler(struct aica *aica, unsigned tim_idx);

static unsigned aica_read_sci(struct aica *aica, unsigned bit);

static char const *fmt_name(enum aica_fmt fmt);

static void aica_sync(struct aica *aica);

static unsigned aica_chan_effective_rate(struct aica const *aica, unsigned chan_no);

static unsigned aica_samples_per_step(unsigned effective_rate, unsigned step_no);

static void aica_process_sample(struct aica *aica);

static int get_octave_signed(struct aica_chan const *chan);
static aica_sample_pos get_sample_rate_multiplier(struct aica_chan const *chan);

static void aica_chan_reset_adpcm(struct aica_chan *chan) {
    chan->step = 0;
    chan->predictor = 0;
    chan->adpcm_next_step = true;
}

struct memory_interface aica_sys_intf = {
    .read32 = aica_sys_read_32,
    .read16 = aica_sys_read_16,
    .read8 = aica_sys_read_8,
    .readfloat = aica_sys_read_float,
    .readdouble = aica_sys_read_double,

    .write32 = aica_sys_write_32,
    .write16 = aica_sys_write_16,
    .write8 = aica_sys_write_8,
    .writefloat = aica_sys_write_float,
    .writedouble = aica_sys_write_double
};

void aica_init(struct aica *aica, struct arm7 *arm7,
               struct dc_clock *clk, struct dc_clock *sh4_clk) {
    memset(aica, 0, sizeof(*aica));

    aica->clk = clk;
    aica->sh4_clk = sh4_clk;
    aica->arm7 = arm7;

    aica->aica_sh4_raise_event.handler = post_delay_raise_aica_sh4_int;
    aica->aica_sh4_raise_event.arg_ptr = aica;

    // HACK
    aica->int_enable = AICA_INT_TIMA_MASK;

    /*
     * The corlett docs say these are default values
     */
    aica->sys_reg[AICA_SCILV0 / 4] = 0x18;
    aica->sys_reg[AICA_SCILV1 / 4] = 0x50;
    aica->sys_reg[AICA_SCILV2 / 4] = 0x08;

    aica->timers[0].evt.handler = aica_timer_a_handler;
    aica->timers[1].evt.handler = aica_timer_b_handler;
    aica->timers[2].evt.handler = aica_timer_c_handler;
    aica->timers[0].evt.arg_ptr = aica;
    aica->timers[1].evt.arg_ptr = aica;
    aica->timers[2].evt.arg_ptr = aica;

    aica_sched_all_timers(aica);

    aica_wave_mem_init(&aica->mem);
}

void aica_cleanup(struct aica *aica) {
    aica_wave_mem_cleanup(&aica->mem);
}

static float aica_sys_read_float(addr32_t addr, void *ctxt) {
    addr &= AICA_SYS_MASK;

    error_set_address(addr);
    error_set_length(4);
    RAISE_ERROR(ERROR_UNIMPLEMENTED);
}

static void aica_sys_write_float(addr32_t addr, float val, void *ctxt) {
    addr &= AICA_SYS_MASK;

    error_set_address(addr);
    error_set_length(4);
    RAISE_ERROR(ERROR_UNIMPLEMENTED);
}

static double aica_sys_read_double(addr32_t addr, void *ctxt) {
    addr &= AICA_SYS_MASK;

    error_set_address(addr);
    error_set_length(8);
    RAISE_ERROR(ERROR_UNIMPLEMENTED);
}

static void aica_sys_write_double(addr32_t addr, double val, void *ctxt) {
    addr &= AICA_SYS_MASK;

    error_set_address(addr);
    error_set_length(8);
    RAISE_ERROR(ERROR_UNIMPLEMENTED);
}

static void
aica_sys_reg_pre_read(struct aica *aica, unsigned idx, bool from_sh4) {

    /*
     * TODO: this only really needs to be called for registers which may have
     * changed due to time, such as PLAYPOS and the timer registers.
     */
    aica_sync(aica);

    uint32_t val;
    struct aica_chan *chan;
    switch (4 * idx) {
    case AICA_MASTER_VOLUME:
        // Neill Corlett's AICA notes say this is always 16 when you read from it
        val = 16;
        memcpy(aica->sys_reg + idx, &val, sizeof(uint32_t));
        break;
    case AICA_ARM7_RST:
        if (!from_sh4) {
            LOG_ERROR("ARM7 suicide unimplemented\n");
            RAISE_ERROR(ERROR_UNIMPLEMENTED);
        }
        break;
    case AICA_SCIRE:
        /*
         * Writing to this register clears interrupts, it's not clear what
         * would happen if it is read from.
         */
        RAISE_ERROR(ERROR_UNIMPLEMENTED);
        break;
    case AICA_SCIPD:
        memcpy(aica->sys_reg + idx, &aica->int_pending, sizeof(uint32_t));
        break;
    case AICA_SCIEB:
        memcpy(aica->sys_reg + idx, &aica->int_enable, sizeof(uint32_t));
        break;
    case AICA_MCIEB:
        memcpy(aica->sys_reg + idx, &aica->int_enable_sh4, sizeof(uint32_t));
        break;
    case AICA_MCIPD:
        memcpy(aica->sys_reg + idx, &aica->int_pending_sh4, sizeof(uint32_t));
        break;
    case AICA_MIDI_INPUT:
        /*
         * The MIDI interface, as far as I know, only exists on development
         * systems and not on retail Dreamcasts.  The value hardcoded below will
         * hopefully convince programs that the MIDI is empty (see the Corlett
         * doc).
         */
        val = (1 << 11) | (1 << 8);
        memcpy(aica->sys_reg + idx, &val, sizeof(uint32_t));
        break;
    case AICA_PLAYPOS:
        chan = aica->channels + aica->chan_sel;
        aica->sys_reg[idx] = chan->sample_pos & 0xffff;
        LOG_DBG("Reading 0x%08x from AICA_PLAYPOS\n",
                (unsigned)aica->sys_reg[idx]);
        break;
    case AICA_PLAYSTATUS:
        chan = aica->channels + aica->chan_sel;

        if (aica->afsel != AICA_AFSEL_ATTEN)
            RAISE_ERROR(ERROR_UNIMPLEMENTED);
        if (chan->atten > 0x3bf && chan->atten != 0x1fff)
            RAISE_ERROR(ERROR_INTEGRITY); // should have already been clamped

        val = chan->atten | (((unsigned)chan->atten_env_state) << 13) |
            (chan->loop_end_playstatus_flag ? (1 << 15) : 0);
        chan->loop_end_playstatus_flag = false;

        memcpy(aica->sys_reg + idx, &val, sizeof(uint32_t));
        LOG_DBG("Reading 0x%08x from AICA_PLAYSTATUS\n",
                (unsigned)aica->sys_reg[idx]);
        break;

    case AICA_TIMERA_CTRL:
        LOG_DBG("read AICA_TIMERA_CTRL\n");
        aica->sys_reg[AICA_TIMERA_CTRL / 4] =
            ((aica->timers[0].prescale_log & 0x7) << 8) |
            (aica->timers[0].counter & 0xf);
        break;
    case AICA_TIMERB_CTRL:
        LOG_DBG("read AICA_TIMERB_CTRL\n");
        aica->sys_reg[AICA_TIMERB_CTRL / 4] =
            ((aica->timers[1].prescale_log & 0x7) << 8) |
            (aica->timers[1].counter & 0xf);
        break;
    case AICA_TIMERC_CTRL:
        LOG_DBG("read AICA_TIMERC_CTRL\n");
        aica->sys_reg[AICA_TIMERC_CTRL / 4] =
            ((aica->timers[2].prescale_log & 0x7) << 8) |
            (aica->timers[2].counter & 0xf);
        break;
    case AICA_INTREQ:
        break;
    case AICA_CHANINFOREQ:
        LOG_DBG("read AICA_CHANINFOREQ\n");
        break;
    default:
#ifdef AICA_PEDANTIC
        error_set_value(aica->sys_reg[idx]);
        error_set_address(4 * idx);
        RAISE_ERROR(ERROR_UNIMPLEMENTED);
#endif
        break;
    }
}

static void
aica_sys_reg_read(struct aica *aica, addr32_t addr,
                  void *out, unsigned n_bytes, bool from_sh4) {
#ifdef INVARANTS
    if (addr <= 0x7fff) {
        error_set_address(addr);
        RAISE_ERROR(ERROR_INTEGRITY);
    }
#endif

    aica_sys_reg_pre_read(aica, addr / 4, from_sh4);

    memcpy(out, ((uint8_t*)aica->sys_reg) + addr, n_bytes);
}

static void
aica_sys_reg_post_write(struct aica *aica, unsigned idx, bool from_sh4) {
    uint32_t val;
    uint32_t mcire;

    switch (idx * 4) {
    case AICA_MASTER_VOLUME:
        LOG_DBG("Writing to AICA_MASTER_VOLUME\n");
        break;
    case AICA_ARM7_RST:
        memcpy(&val, aica->sys_reg + (AICA_ARM7_RST/4), sizeof(val));
        if (from_sh4) {
            arm7_reset(aica->arm7, !(val & 1));
        } else {
            LOG_ERROR("ARM7 suicide unimplemented\n");
            RAISE_ERROR(ERROR_UNIMPLEMENTED);
        }
        break;
    case AICA_SCIRE:
        memcpy(&val, aica->sys_reg + (AICA_SCIRE/4), sizeof(val));
        if ((aica->int_pending & AICA_INT_TIMA_MASK) & (val & AICA_INT_TIMA_MASK)) {
            LOG_DBG("AICA: clearing timerA interrupt\n");
        }
        aica->int_pending &= ~val;
        aica_update_interrupts(aica);
        break;
    case AICA_MCIRE:
        memcpy(&val, aica->sys_reg + (AICA_MCIRE/4), sizeof(val));
        aica->int_pending_sh4 &= ~val;
        aica_update_interrupts(aica);
        if (val & (1<<5))
            holly_clear_ext_int(HOLLY_EXT_INT_AICA);
        break;
    case AICA_SCIPD:
        /*
         * TODO: Neill Corlett's doc says that interrupt 5 (CPU interrupt) can
         * be manually triggered by writing to bit 5 of this register.
         */
        RAISE_ERROR(ERROR_UNIMPLEMENTED);
        break;
    case AICA_MCIPD:
        /*
         * TODO: You can write to bit 5 (CPU interrupt) to send an interrupt to
         * the SH4.
         */


        memcpy(&val, aica->sys_reg + (AICA_MCIPD/4), sizeof(val));
        memcpy(&mcire, aica->sys_reg + (AICA_MCIRE/4), sizeof(mcire));

        if (!(val & (1<<5))) // TODO: what if guest writes 0?
            RAISE_ERROR(ERROR_UNIMPLEMENTED);

        if ((val & (1<<5)) && !(mcire & (1<<5)))
            RAISE_ERROR(ERROR_UNIMPLEMENTED);
        if (val & (1<<5)) {
            raise_aica_sh4_int(aica);
        }
        break;
    case AICA_SCIEB:
        memcpy(&val, aica->sys_reg + (AICA_SCIEB/4), sizeof(val));
        aica->int_enable = val;
        aica_update_interrupts(aica);
        break;
    case AICA_MCIEB:
        memcpy(&val, aica->sys_reg + (AICA_MCIEB/4), sizeof(val));
        if (val & ~AICA_INT_CPU_MASK)
            RAISE_ERROR(ERROR_UNIMPLEMENTED);
        aica->int_enable_sh4 = val;
        break;
    case AICA_RINGBUFFER_ADDRESS:
        memcpy(&val, aica->sys_reg + (AICA_RINGBUFFER_ADDRESS/4), sizeof(val));
        aica->ringbuffer_addr = (val & BIT_RANGE(0, 11)) << 11;
        aica->ringbuffer_size = (val & BIT_RANGE(13, 14)) >> 13;
        aica->ringbuffer_bit15 = (bool)(val & (1 << 15));
        LOG_DBG("Writing 0x%08x to AICA_RINGBUFFER_ADDRESS\n", (unsigned)val);
        break;
    case AICA_UNKNOWN_2880:
        memcpy(&val, aica->sys_reg + (AICA_UNKNOWN_2880/4), sizeof(val));
        LOG_DBG("Writing 0x%08x to AICA_UNKNOWN_2880\n", (unsigned)val);
        break;

        /*
         * TODO: there are three timers in the AICA system.
         *
         * The lower byte of the timer register is a counter which increments
         * periodically and raises an interrupt when it overflows.
         *
         * bits 10-8 are the base-2 logarithm of how many samples occur per
         * timer increment.  Ostensibly "samples" refers to audio samples, but
         * I still don't understand AICA well enough to implement that so I
         * didn't implement that.
         */
    case AICA_TIMERA_CTRL:
        LOG_DBG("AICA: write to TIMERA_CTRL\n");
        memcpy(&val, aica->sys_reg + (AICA_TIMERA_CTRL/4), sizeof(val));
        on_timer_ctrl_write(aica, 0, val);
        break;
    case AICA_TIMERB_CTRL:
        LOG_DBG("AICA: write to TIMERA_CTRL\n");
        memcpy(&val, aica->sys_reg + (AICA_TIMERB_CTRL/4), sizeof(val));
        on_timer_ctrl_write(aica, 1, val);
        break;
    case AICA_TIMERC_CTRL:
        LOG_DBG("AICA: write to TIMERA_CTRL\n");
        memcpy(&val, aica->sys_reg + (AICA_TIMERC_CTRL/4), sizeof(val));
        on_timer_ctrl_write(aica, 2, val);
        break;

    case AICA_SCILV0:
        memcpy(&val, aica->sys_reg + (AICA_SCILV0/4), sizeof(val));
        LOG_DBG("Writing 0x%08x to AICA_SCILV0\n", (unsigned)val);
        break;
    case AICA_SCILV1:
        memcpy(&val, aica->sys_reg + (AICA_SCILV1/4), sizeof(val));
        LOG_DBG("Writing 0x%08x to AICA_SCILV1\n", (unsigned)val);
        break;
    case AICA_SCILV2:
        memcpy(&val, aica->sys_reg + (AICA_SCILV2/4), sizeof(val));
        LOG_DBG("Writing 0x%08x to AICA_SCILV2\n", (unsigned)val);
        break;

    case AICA_INTCLEAR:
        memcpy(&val, aica->sys_reg + (AICA_INTCLEAR/4), sizeof(val));
        LOG_DBG("Writing 0x%08x to AICA_INTCLEAR\n", (unsigned)val);
        if ((val & 0xff) == 1)
            arm7_clear_fiq(aica->arm7);
        break;

    case AICA_CHANINFOREQ:
        memcpy(&val, aica->sys_reg + (AICA_CHANINFOREQ/4), sizeof(val));
        LOG_DBG("Writing 0x%08x to AICA_CHANINFOREQ\n", (unsigned)val);

        aica->chan_sel = (val >> 8) & (0x40 - 1);
        aica->afsel = (enum aica_afsel)(val >> 14);

        break;
    case 0x2884:
    case 0x2888:
    case 0x288c:
        /*
         * Twinkle Star Sprites writes 0 to these three registers once during
         * boot.  They don't appear to do anything impoprtant.  Let 0 through
         * and panic if we ever see anything else get written.
         */
        if (!aica->sys_reg[idx]) {
            LOG_DBG("AICA: Writing 0x%08x to register index %u\n",
                    (unsigned)aica->sys_reg[idx], idx);
        } else {
            error_set_value(aica->sys_reg[idx]);
            error_set_address(4 * idx);
            RAISE_ERROR(ERROR_UNIMPLEMENTED);
        }
        break;
    case AICA_MIDI_INPUT:
        /*
         * I *think* this isn't even on retail systems?
         * some of the 2k sports games want to write 0 to this for some dumb
         * reason.
         */
        memcpy(&val, aica->sys_reg + (AICA_MIDI_INPUT/4), sizeof(val));
        LOG_DBG("Writing %08X to AICA_MIDI_INPUT\n", (unsigned)val);
        break;
    default:
#ifdef AICA_PEDANTIC
        error_set_value(aica->sys_reg[idx]);
        error_set_address(4 * idx);
        RAISE_ERROR(ERROR_UNIMPLEMENTED);
#endif
        break;
    }
}

static void aica_sys_reg_write(struct aica *aica, addr32_t addr,
                               void const *val_in, unsigned n_bytes,
                               bool from_sh4) {
#ifdef INVARANTS
    if (addr <= 0x7fff) {
        error_set_address(addr);
        RAISE_ERROR(ERROR_INTEGRITY);
    }
#endif

    memcpy(((uint8_t*)aica->sys_reg) + addr, val_in, n_bytes);
    aica_sys_reg_post_write(aica, addr / 4, from_sh4);
}

static void aica_sys_channel_read(struct aica *aica, void *dst,
                                  uint32_t addr, unsigned len) {
    uint32_t addr_first = addr;
    uint32_t addr_last = addr + (len - 1);
    if (addr_first > 0x1fff || addr_last > 0x1fff) {
        error_set_length(len);
        error_set_address(addr);
        RAISE_ERROR(ERROR_MEM_OUT_OF_BOUNDS);
    }
    if (aica_log_verbose_val) {
        LOG_DBG("AICA CHANNEL DATA: Reading %u bytes from 0x%08x\n",
                len, (unsigned)addr);
    }

    unsigned chan_no = addr / AICA_CHAN_LEN;
    unsigned chan_reg = addr % AICA_CHAN_LEN;

    struct aica_chan *chan = aica->channels + chan_no;
    uint32_t tmp;
    unsigned idx = chan_reg / 4;
    unsigned reg_no = 4 * idx;

    LOG_DBG("Reading from AICA channel %u register \"%s\"\n",
            chan_no, aica_chan_reg_name(reg_no));
    switch (reg_no) {
    case AICA_CHAN_PLAY_CTRL:
        memcpy(&tmp, chan->raw + AICA_CHAN_PLAY_CTRL, sizeof(tmp));
        tmp &= ~(1 << 15);
        memcpy(chan->raw + AICA_CHAN_PLAY_CTRL, &tmp, sizeof(tmp));
        break;
    case AICA_CHAN_SAMPLE_ADDR_LOW:
        tmp = chan->addr_start & 0xffff;
        memcpy(chan->raw + AICA_CHAN_SAMPLE_ADDR_LOW, &tmp, sizeof(tmp));
        break;
    case AICA_CHAN_LOOP_START:
    case AICA_CHAN_LOOP_END:
    case AICA_CHAN_SAMPLE_RATE_PITCH:
    case AICA_CHAN_DSP_SEND:
    case AICA_CHAN_LFO_CTRL:
    case AICA_CHAN_DIR_PAN_VOL_SEND:
    case AICA_CHAN_LPF1_VOL:
    case AICA_CHAN_LPF2:
    case AICA_CHAN_LPF3:
    case AICA_CHAN_LPF4:
    case AICA_CHAN_LPF5:
    case AICA_CHAN_LPF6:
    case AICA_CHAN_LPF7:
    case AICA_CHAN_LPF8:
    case AICA_CHAN_AMP_ENV1:
    case AICA_CHAN_AMP_ENV2:
        break;
#ifdef AICA_PEDANTIC
    default:
        error_set_channel(chan_no);
        error_set_address(reg_no);
        RAISE_ERROR(ERROR_UNIMPLEMENTED);
#endif
    }

    memcpy(dst, chan->raw + chan_reg, len);
}

static void aica_dsp_mixer_read(struct aica *aica, void *dst,
                                uint32_t addr, unsigned len) {
    uint32_t addr_first = addr;
    uint32_t addr_last = addr + (len - 1);
    if (addr_first >= 0x2048 || addr_last >= 0x2048 ||
        addr_first <= 0x1fff || addr_last <= 0x1fff) {
        error_set_length(len);
        error_set_address(addr);
        RAISE_ERROR(ERROR_MEM_OUT_OF_BOUNDS);
    }
    if (aica_log_verbose_val) {
        LOG_DBG("AICA DSP MIXER: Reading %u bytes from 0x%08x\n",
                len, (unsigned)addr);
    }
    memcpy(dst, ((uint8_t*)aica->sys_reg) + addr, len);
}

static void aica_dsp_reg_read(struct aica *aica, void *dst,
                              uint32_t addr, unsigned len) {
    uint32_t addr_first = addr;
    uint32_t addr_last = addr + (len - 1);
    if (addr_first >= 0x8000 || addr_last >= 0x8000 ||
        addr_first < 0x3000 || addr_last < 0x3000) {
        error_set_length(len);
        error_set_address(addr);
        RAISE_ERROR(ERROR_MEM_OUT_OF_BOUNDS);
    }
    if (aica_log_verbose_val) {
        LOG_DBG("AICA DSP REG: Reading %u bytes from 0x%08x\n",
                len, (unsigned)addr);
    }
    memcpy(dst, ((uint8_t*)aica->sys_reg) + addr, len);
}

static void aica_do_keyon(struct aica *aica) {
    unsigned chan_no;
    for (chan_no = 0; chan_no < AICA_CHAN_COUNT; chan_no++) {
        struct aica_chan *chan = aica->channels + chan_no;
        if (chan->ready_keyon &&
            (!chan->playing || chan->atten_env_state == AICA_ENV_RELEASE)) {
            chan->playing = true;
            chan->step_no = 0;
            chan->sample_no = 0;
            chan->sample_pos = 0/* chan->addr_start */;
            chan->sample_partial = 0;
            chan->addr_cur = chan->addr_start;
            chan->atten_env_state = AICA_ENV_ATTACK;
            chan->atten = 0x280;
            chan->loop_end_playstatus_flag = false;
            chan->loop_end_signaled = false;

            aica_chan_reset_adpcm(chan);

            LOG_INFO("AICA channel %u key-on fmt %s ptr 0x%08x\n",
                   chan_no, fmt_name(chan->fmt),
                   (unsigned)chan->addr_start);
        } else if (!chan->ready_keyon && chan->playing && chan->atten_env_state != AICA_ENV_RELEASE) {
            chan->atten_env_state = AICA_ENV_RELEASE;
            LOG_INFO("AICA channel %u key-off\n", chan_no);
        }
    }
}

static void aica_chan_playctrl_write(struct aica *aica, unsigned chan_no) {
    uint32_t val;
    struct aica_chan *chan = aica->channels + chan_no;
    memcpy(&val, chan->raw + AICA_CHAN_PLAY_CTRL, sizeof(val));

    chan->fmt = (val >> 7) & 3;
    chan->addr_start &= ~(0xffff << 16);
    chan->addr_start |= (val & 0x7f) << 16;
    chan->addr_cur = chan->addr_start;
    chan->loop_en = (bool)(val & (1 << 9));
    LOG_DBG("AICA: addr_start is now 0x%08x\n", (unsigned)chan->addr_start);

    chan->ready_keyon = (bool)(val & (1 << 14));
    if (val & (1 << 15))
        aica_do_keyon(aica);
    chan->keyon = (bool)(val & (1 << 15));
}

static void aica_sys_channel_write(struct aica *aica, void const *src,
                                   uint32_t addr, unsigned len) {
    uint32_t tmp;
    uint32_t addr_first = addr;
    uint32_t addr_last = addr + (len - 1);
    if (addr_first > 0x1fff || addr_last > 0x1fff) {
        error_set_length(len);
        error_set_address(addr);
        RAISE_ERROR(ERROR_MEM_OUT_OF_BOUNDS);
    }

    unsigned chan_no = addr / AICA_CHAN_LEN;
    unsigned chan_reg = addr % AICA_CHAN_LEN;

    struct aica_chan *chan = aica->channels + chan_no;

    if (aica_log_verbose_val) {
        LOG_DBG("AICA CHANNEL DATA: Writing %u bytes from 0x%08x\n",
                len, (unsigned)addr);
    }
    memcpy(chan->raw + chan_reg, src, len);
    unsigned reg_no = 4 * (chan_reg / 4);

    memcpy(&tmp, src, sizeof(tmp));
    LOG_DBG("AICA: write 0x%08x to channel %u register \"%s\"\n",
             (int)tmp, chan_no, aica_chan_reg_name(reg_no));

    switch (reg_no) {
    case AICA_CHAN_PLAY_CTRL:
        aica_chan_playctrl_write(aica, chan_no);
        break;
    case AICA_CHAN_SAMPLE_ADDR_LOW:
        memcpy(&tmp, chan->raw + AICA_CHAN_SAMPLE_ADDR_LOW, sizeof(tmp));
        chan->addr_start &= ~0xffff;
        chan->addr_start |= tmp & 0xffff;
        chan->addr_cur = chan->addr_start;
        LOG_DBG("AICA: chan %u addr_start is now 0x%08x\n",
               chan_no, (unsigned)chan->addr_start);
        break;
    case AICA_CHAN_LOOP_START:
        memcpy(&chan->loop_start, chan->raw + AICA_CHAN_LOOP_START,
               sizeof(chan->loop_start));
        /* chan->loop_start &= ~0xffff; */
        LOG_DBG("AICA: chan %u loop_start is now 0x%08x\n",
               chan_no, (unsigned)chan->loop_start);
        break;
    case AICA_CHAN_LOOP_END:
        memcpy(&chan->loop_end, chan->raw + AICA_CHAN_LOOP_END,
               sizeof(chan->loop_end));
        /* chan->loop_end &= ~0xffff; */
        LOG_DBG("AICA: chan %u loop_end is now 0x%08x\n",
               chan_no, (unsigned)chan->loop_end);
        break;
    case AICA_CHAN_AMP_ENV1:
        memcpy(&tmp, chan->raw + AICA_CHAN_AMP_ENV2, sizeof(tmp));
        chan->attack_rate = tmp & BIT_RANGE(0, 4);
        chan->decay_rate = (tmp & BIT_RANGE(6, 10)) >> 6;
        chan->sustain_rate = (tmp & BIT_RANGE(11, 15)) >> 11;
        break;
    case AICA_CHAN_AMP_ENV2:
        memcpy(&tmp, chan->raw + AICA_CHAN_AMP_ENV2, sizeof(tmp));
        chan->krs = (tmp >> 10) & 0xf;
        chan->decay_level = tmp & BIT_RANGE(5, 9);
        chan->release_rate = tmp & BIT_RANGE(0, 4);
        break;
    case AICA_CHAN_SAMPLE_RATE_PITCH:
        memcpy(&tmp, chan->raw + AICA_CHAN_SAMPLE_RATE_PITCH, sizeof(tmp));
        chan->fns = tmp & BIT_RANGE(0, 10);

        // octage is a 4-bit two's complement value that ranges from -8 to +7
        uint32_t oct32 = (tmp & BIT_RANGE(11, 14)) >> 11;
        chan->octave = oct32;

#ifdef ENABLE_LOG_DEBUG
        double sample_rate = (double)get_sample_rate_multiplier(chan) /
            (double)(1 << AICA_SAMPLE_POS_SHIFT);

        LOG_DBG("AICA channel %u sample_rate is %f oct %d fns 0x%04x\n",
                 chan_no, sample_rate, get_octave_signed(chan), chan->fns);
#endif
        break;
    case AICA_CHAN_LFO_CTRL:
        memcpy(&tmp, src, sizeof(tmp));
        if (tmp & (1 << 15))
            LOG_WARN("AICA: low-frequency oscillator is not implemented!\n");
        break;
    case AICA_CHAN_DIR_PAN_VOL_SEND:
        memcpy(&tmp, chan->raw + AICA_CHAN_DIR_PAN_VOL_SEND, sizeof(tmp));
        chan->volume = (tmp >> 8) & 0xf;
        chan->pan = tmp & 0x1f;
        break;
    case AICA_CHAN_DSP_SEND:
    case AICA_CHAN_LPF1_VOL:
    case AICA_CHAN_LPF2:
    case AICA_CHAN_LPF3:
    case AICA_CHAN_LPF4:
    case AICA_CHAN_LPF5:
    case AICA_CHAN_LPF6:
    case AICA_CHAN_LPF7:
    case AICA_CHAN_LPF8:
        break;
    default:
        memcpy(&tmp, src, sizeof(tmp));
        LOG_DBG("AICA: write to addr 0x%08x chan %u offset %u val 0x%08x\n",
                (unsigned)addr, chan_no, chan_reg, (unsigned)tmp);
#ifdef AICA_PEDANTIC
        if (tmp) {
            error_set_channel(chan_no);
            error_set_address(reg_no);
            error_set_value(tmp);
            RAISE_ERROR(ERROR_UNIMPLEMENTED);
        }
#endif
    }
}

static void aica_dsp_mixer_write(struct aica *aica, void const *src,
                                uint32_t addr, unsigned len) {
    uint32_t addr_first = addr;
    uint32_t addr_last = addr + (len - 1);
    if (addr_first >= 0x2048 || addr_last >= 0x2048 ||
        addr_first <= 0x1fff || addr_last <= 0x1fff) {
        error_set_length(len);
        error_set_address(addr);
        RAISE_ERROR(ERROR_MEM_OUT_OF_BOUNDS);
    }
    if (aica_log_verbose_val) {
        LOG_DBG("AICA DSP MIXER: Writing %u bytes from 0x%08x\n",
                len, (unsigned)addr);
    }
    memcpy(((uint8_t*)aica->sys_reg) + addr, src, len);
}

static void aica_dsp_reg_write(struct aica *aica, void const *src,
                               uint32_t addr, unsigned len) {
    uint32_t addr_first = addr;
    uint32_t addr_last = addr + (len - 1);
    if (addr_first >= 0x8000 || addr_last >= 0x8000 ||
        addr_first < 0x3000 || addr_last < 0x3000) {
        error_set_length(len);
        error_set_address(addr);
        RAISE_ERROR(ERROR_MEM_OUT_OF_BOUNDS);
    }
    if (aica_log_verbose_val) {
        LOG_DBG("AICA DSP REG: Writing %u bytes from 0x%08x\n",
                len, (unsigned)addr);
    }
    memcpy(((uint8_t*)aica->sys_reg) + addr, src, len);
}

static uint32_t aica_sys_read_32(addr32_t addr, void *ctxt) {
    struct aica *aica = (struct aica*)ctxt;
    bool from_sh4 = (addr & 0x00f00000) == 0x00700000;

    addr &= AICA_SYS_MASK;

    if (addr < 0x1fff) {
        // Channel registers
        uint32_t ret;
        aica_sys_channel_read(aica, &ret, addr, sizeof(ret));
        return ret;
    }

    if (addr <= 0x2044) {
        // DSP mixer
        uint32_t ret;
        aica_dsp_mixer_read(aica, &ret, addr, sizeof(ret));
        return ret;
    }

    if (addr >= 0x3000 && addr <= 0x7fff) {
        // DSP registers
        uint32_t ret;
        aica_dsp_reg_read(aica, &ret, addr, sizeof(ret));
        return ret;
    }

    if (addr >= 0x2800 && addr <= 0x2fff) {
        uint32_t ret;
        aica_sys_reg_read(aica, addr, &ret, sizeof(ret), from_sh4);
        return ret;
    }

    error_set_address(addr);
    error_set_length(4);
    RAISE_ERROR(ERROR_UNIMPLEMENTED);
}

static void aica_sys_write_32(addr32_t addr, uint32_t val, void *ctxt) {
    struct aica *aica = (struct aica*)ctxt;
    bool from_sh4 = (addr & 0x00f00000) == 0x00700000;

    addr &= AICA_SYS_MASK;

    if (addr <= 0x1fff) {
        // channel data
        aica_sys_channel_write(aica, &val, addr, sizeof(val));
        return;
    }

    if (addr <= 0x2044) {
        // DSP mixer
        aica_dsp_mixer_write(aica, &val, addr, sizeof(val));
        return;
    }

    if (addr >= 0x3000 && addr <= 0x7fff) {
        // DSP registers
        aica_dsp_reg_write(aica, &val, addr, sizeof(val));
        return;
    }

    if (addr >= 0x2800 && addr <= 0x2fff) {
        aica_sys_reg_write(aica, addr, &val, sizeof(val), from_sh4);
    } else {
        error_set_address(addr);
        error_set_length(4);
        RAISE_ERROR(ERROR_UNIMPLEMENTED);
    }
}

static uint16_t aica_sys_read_16(addr32_t addr, void *ctxt) {
    struct aica *aica = (struct aica*)ctxt;
    bool from_sh4 = (addr & 0x00f00000) == 0x00700000;

    addr &= AICA_SYS_MASK;

    if (addr < 0x1fff) {
        // Channel registers
        uint16_t ret;
        aica_sys_channel_read(aica, &ret, addr, sizeof(ret));
        return ret;
    }

    if (addr <= 0x2044) {
        // DSP mixer
        uint16_t ret;
        aica_dsp_mixer_read(aica, &ret, addr, sizeof(ret));
        return ret;
    }

    if (addr >= 0x3000 && addr <= 0x7fff) {
        // DSP registers
        uint16_t ret;
        aica_dsp_reg_read(aica, &ret, addr, sizeof(ret));
        return ret;
    }

    if (addr >= 0x2800 && addr <= 0x2fff) {
        uint16_t ret;
        aica_sys_reg_read(aica, addr, &ret, sizeof(ret), from_sh4);
        return ret;
    }

    error_set_address(addr);
    error_set_length(2);
    RAISE_ERROR(ERROR_UNIMPLEMENTED);
}

static void aica_sys_write_16(addr32_t addr, uint16_t val, void *ctxt) {
    struct aica *aica = (struct aica*)ctxt;
    bool from_sh4 = (addr & 0x00f00000) == 0x00700000;

    addr &= AICA_SYS_MASK;

    if (addr <= 0x1fff) {
        // channel data
        aica_sys_channel_write(aica, &val, addr, sizeof(val));
        return;
    }

    if (addr <= 0x2044) {
        // DSP mixer
        aica_dsp_mixer_write(aica, &val, addr, sizeof(val));
        return;
    }

    if (addr >= 0x3000 && addr <= 0x7fff) {
        // DSP registers
        aica_dsp_reg_write(aica, &val, addr, sizeof(val));
        return;
    }

    if (addr >= 0x2800 && addr <= 0x2fff) {
        aica_sys_reg_write(aica, addr, &val, sizeof(val), from_sh4);
    } else {
        error_set_address(addr);
        error_set_length(2);
        RAISE_ERROR(ERROR_UNIMPLEMENTED);
    }
}

static uint8_t aica_sys_read_8(addr32_t addr, void *ctxt) {
    struct aica *aica = (struct aica*)ctxt;
    bool from_sh4 = (addr & 0x00f00000) == 0x00700000;

    addr &= AICA_SYS_MASK;

    if (addr < 0x1fff) {
        // Channel registers
        uint8_t ret;
        aica_sys_channel_read(aica, &ret, addr, sizeof(ret));
        return ret;
    }

    if (addr <= 0x2044) {
        // DSP mixer
        uint8_t ret;
        aica_dsp_mixer_read(aica, &ret, addr, sizeof(ret));
        return ret;
    }

    if (addr >= 0x3000 && addr <= 0x7fff) {
        // DSP registers
        uint8_t ret;
        aica_dsp_reg_read(aica, &ret, addr, sizeof(ret));
        return ret;
    }

    if (addr >= 0x2800 && addr <= 0x2fff) {
        uint8_t ret;
        aica_sys_reg_read(aica, addr, &ret, sizeof(ret), from_sh4);
        return ret;
    }

    error_set_address(addr);
    error_set_length(1);
    RAISE_ERROR(ERROR_UNIMPLEMENTED);
}

static void aica_sys_write_8(addr32_t addr, uint8_t val, void *ctxt) {
    struct aica *aica = (struct aica*)ctxt;
    bool from_sh4 = (addr & 0x00f00000) == 0x00700000;

    addr &= AICA_SYS_MASK;

    if (addr <= 0x1fff) {
        // channel data
        aica_sys_channel_write(aica, &val, addr, sizeof(val));
        return;
    }

    if (addr <= 0x2047) {
        // DSP mixer
        aica_dsp_mixer_write(aica, &val, addr, sizeof(val));
        return;
    }

    if (addr >= 0x3000 && addr <= 0x7fff) {
        // DSP registers
        aica_dsp_reg_write(aica, &val, addr, sizeof(val));
        return;
    }

    if (addr >= 0x2800 && addr <= 0x2fff) {
        aica_sys_reg_write(aica, addr, &val, sizeof(val), from_sh4);
    } else {
        error_set_address(addr);
        error_set_length(1);
        RAISE_ERROR(ERROR_UNIMPLEMENTED);
    }
}

static void aica_update_interrupts(struct aica *aica) {
    /*
     * this is really just a placeholder in case I ever want to put some logging
     * in or something, this function doesn't actually need to be here.
     */
    LOG_DBG("FIQ: aica->int_enable is now 0x%08x\n",
            (unsigned)aica->int_enable);
}

static void aica_unsched_all_timers(struct aica *aica) {
    unsigned idx;
    for (idx = 0; idx < 3; idx++)
        aica_unsched_timer(aica, idx);
}

static void aica_sched_all_timers(struct aica *aica) {
    unsigned idx;
    for (idx = 0; idx < 3; idx++)
        aica_sched_timer(aica, idx);
}

static void aica_unsched_timer(struct aica *aica, unsigned tim_idx) {
    struct aica_timer *timer = aica->timers + tim_idx;
    if (timer->scheduled) {
        timer->scheduled = false;
        cancel_event(aica->clk, &timer->evt);
    }
}

static void aica_sched_timer(struct aica *aica, unsigned tim_idx) {
    struct aica_timer *timer = aica->timers + tim_idx;

    if (timer->scheduled)
        return;

    struct SchedEvent *evt = &timer->evt;
    struct dc_clock *clk = aica->clk;

    unsigned prescale = 1 << timer->prescale_log;
    unsigned ticks_to_go = 256 - timer->counter;
    dc_cycle_stamp_t clk_ticks = TICKS_PER_SAMPLE * ticks_to_go * prescale;

    evt->when = clock_cycle_stamp(clk) + clk_ticks;

    sched_event(aica->clk, evt);

    timer->scheduled = true;
}

static void aica_sync_timer(struct aica *aica, unsigned tim_idx) {
    struct aica_timer *timer = aica->timers + tim_idx;
    unsigned prescale = 1 << timer->prescale_log;
    dc_cycle_stamp_t sample_delta =
        aica_get_sample_count(aica) - timer->last_sample_sync;

    if (sample_delta) {
        unsigned clock_tick_delta = sample_delta / prescale;

        if (clock_tick_delta) {
            timer->counter += clock_tick_delta;
            timer->counter %= 256;
            timer->last_sample_sync = aica_get_sample_count(aica);
        }
    }
}

static void
on_timer_ctrl_write(struct aica *aica, unsigned tim_idx, uint32_t val) {
    struct aica_timer *timer = aica->timers + tim_idx;

    aica_sync(aica);
    aica_unsched_timer(aica, tim_idx);

    timer->counter = val & 0xff;
    timer->prescale_log = (val >> 8) & 0x7;

    aica_sched_timer(aica, tim_idx);

    LOG_DBG("Writing 0x%08x to AICA_TIMER%c_CTRL\n",
            (unsigned)val,
            tim_idx == 0 ? 'A' : (tim_idx == 1 ? 'B' : 'C'));
}

static void
aica_timer_a_handler(struct SchedEvent *evt) {
    struct aica *aica = (struct aica*)evt->arg_ptr;
    aica_timer_handler(aica, 0);
}

static void
aica_timer_b_handler(struct SchedEvent *evt) {
    struct aica *aica = (struct aica*)evt->arg_ptr;
    aica_timer_handler(aica, 1);
}

static void
aica_timer_c_handler(struct SchedEvent *evt) {
    struct aica *aica = (struct aica*)evt->arg_ptr;
    aica_timer_handler(aica, 2);
}

static void aica_timer_handler(struct aica *aica, unsigned tim_idx) {
    struct aica_timer *timer = aica->timers + tim_idx;

    timer->scheduled = false;
    aica_sync(aica);

    if (timer->counter) {
        LOG_ERROR("timer->counter is %u\n", timer->counter);
        RAISE_ERROR(ERROR_INTEGRITY);
    }

    /*
     * it is not a mistake that timer B and timer C both share pin 7 scilv.
     * the corlett doc says that bit 7 of scilv referes to bits 7, 8, 9 and 10
     * of SCIPD all at the same time.
     */
    switch (tim_idx) {
    case 0:
        aica->int_pending |= AICA_INT_TIMA_MASK;
        if (aica->int_enable & AICA_INT_TIMA_MASK) {
            aica->sys_reg[AICA_INTREQ/4] = aica_read_sci(aica, 6);
            arm7_set_fiq(aica->arm7);
        }
        break;
    case 1:
        aica->int_pending |= AICA_INT_TIMB_MASK;
        if (aica->int_enable & AICA_INT_TIMB_MASK) {
            aica->sys_reg[AICA_INTREQ/4] = aica_read_sci(aica, 7);
            arm7_set_fiq(aica->arm7);
        }
        break;
    case 2:
        aica->int_pending |= AICA_INT_TIMC_MASK;
        if (aica->int_enable & AICA_INT_TIMC_MASK) {
            aica->sys_reg[AICA_INTREQ/4] = aica_read_sci(aica, 7);
            arm7_set_fiq(aica->arm7);
        }
        break;
    }

    aica_sched_timer(aica, tim_idx);
}

static unsigned aica_read_sci(struct aica *aica, unsigned bit) {
    if (bit >= 8)
        RAISE_ERROR(ERROR_INTEGRITY);

    unsigned bits[3] = {
        (aica->sys_reg[AICA_SCILV0 / 4] >> bit) & 1,
        (aica->sys_reg[AICA_SCILV1 / 4] >> bit) & 1,
        (aica->sys_reg[AICA_SCILV2 / 4] >> bit) & 1
    };

    return (bits[2] << 2) | (bits[1] << 1) | bits[0];
}

static char const *fmt_name(enum aica_fmt fmt) {
    switch (fmt) {
    case AICA_FMT_16_BIT_SIGNED:
        return "16-bit signed";
    case AICA_FMT_8_BIT_SIGNED:
        return "8-bit signed";
    default:
    case AICA_FMT_4_BIT_ADPCM:
        return "4-bit Yamaha ADPCM";
    }
}

static dc_cycle_stamp_t aica_get_sample_count(struct aica *aica) {
    return clock_cycle_stamp(aica->clk) / TICKS_PER_SAMPLE;
}

static void aica_sync(struct aica *aica) {
    aica_sync_timer(aica, 0);
    aica_sync_timer(aica, 1);
    aica_sync_timer(aica, 2);

    if (aica->last_sample_sync != aica_get_sample_count(aica)) {
        /*
         * process all samples between aica->last_sample_sync and aica_get
         * sample_count(aica)
         */
        dc_cycle_stamp_t n_samples = AICA_FREQ_RATIO *
            (aica_get_sample_count(aica) - aica->last_sample_sync);

        while (n_samples--)
            aica_process_sample(aica);

        aica->last_sample_sync = aica_get_sample_count(aica);
    }
}

static unsigned aica_chan_effective_rate(struct aica const *aica, unsigned chan_no) {
    struct aica_chan const *chan = aica->channels + chan_no;
    unsigned rate;
    switch (chan->atten_env_state) {
    case AICA_ENV_ATTACK:
        rate = chan->attack_rate;
        break;
    case AICA_ENV_DECAY:
        rate = chan->decay_rate;
        break;
    case AICA_ENV_SUSTAIN:
        rate = chan->sustain_rate;
        break;
    case AICA_ENV_RELEASE:
        rate = chan->release_rate;
        break;
    default:
        RAISE_ERROR(ERROR_INTEGRITY);
    }

    if (chan->krs == 15) {
        return rate * 2;
    } else {
        /*
         * Effective rate determines how quickly the state Amplitude Envelope
         * Generator transitions between states, so if the effective-rate is
         * not being calculated correctly, then possible bugs include channels
         * repeating after they should have stopped, and channels not playing
         * at all.
         *
         * XXX The corlett docs say this should be (KRS + rate + octave * 2) +
         *     bit-9 of FNS.  i've found this to be problematic, since octave
         *     can be a negative value but the rate cannot be negative so I've
         *     removed it.  I do not have any information on how this case is
         *     handled.  Removing octave also fixed many channels that were
         *     looping after they should have terminated in Crazy Taxi.
         */
        return (chan->krs + rate) * 2 + ((chan->fns >> 9) & 1);
    }
}

static unsigned aica_samples_per_step(unsigned effective_rate, unsigned step_no) {
    switch (effective_rate) {
    case 0:
    case 1:
        return 0;
    case 2: {
        unsigned const static tbl[3] = { 8192, 4096, 4096 };
        return tbl[step_no % 3];
    }
    case 3: {
        unsigned const static tbl[7] = { 8192, 4096, 4096,
                                         4096, 4096, 4096, 4096 };
        return tbl[step_no % 7];
    }
    case 4:
        return 4096;
    case 5: {
        unsigned const static tbl[5] = { 4096, 4096, 4096, 2048, 2048 };
        return tbl[step_no % 5];
    }
    case 6: {
        unsigned const static tbl[3] = { 4096, 2048, 2048 };
        return tbl[step_no % 3];
    }
    case 7: {
        unsigned const static tbl[7] = { 4096, 2048, 2048,
                                         2048, 2048, 2048, 2048 };
        return tbl[step_no % 7];
    }
    case 8:
        return 2048;
    case 9: {
        unsigned const static tbl[5] = { 2048, 2048, 2048, 1024, 1024 };
        return tbl[step_no % 5];
    }
    case 10: {
        unsigned const static tbl[3] = { 2048, 1024, 1024 };
        return tbl[step_no % 3];
    }
    case 11: {
        unsigned const static tbl[7] = { 2048, 1024, 1024,
                                         1024, 1024, 1024, 1024 };
        return tbl[step_no % 7];
    }
    case 12:
        return 1024;
    case 13: {
        unsigned const static tbl[5] = { 1024, 1024, 1024, 512, 512 };
        return tbl[step_no % 5];
    }
    case 14: {
        unsigned const static tbl[3] = { 1024, 512, 512 };
        return tbl[step_no % 3];
    }
    case 15: {
        unsigned const static tbl[7] = { 1024, 512, 512, 512, 512, 512, 512 };
        return tbl[step_no % 7];
    }
    case 16:
        return 512;
    case 17: {
        unsigned const static tbl[5] = { 512, 512, 512, 256, 256 };
        return tbl[step_no % 5];
    }
    case 18: {
        unsigned const static tbl[3] = { 512, 256, 256 };
        return tbl[step_no % 3];
    }
    case 19: {
        unsigned const static tbl[7] = { 512, 256, 256, 256, 256, 256, 256 };
        return tbl[step_no % 7];
    }
    case 20:
        return 256;
    case 21: {
        unsigned const static tbl[5] = { 256, 256, 256, 128, 128 };
        return tbl[step_no % 5];
    }
    case 22: {
        unsigned const static tbl[3] = { 256, 128, 128 };
        return tbl[step_no % 3];
    }
    case 23: {
        unsigned const static tbl[7] = { 256, 128, 128, 128, 128, 128, 128 };
        return tbl[step_no % 7];
    }
    case 24:
        return 128;
    case 25: {
        unsigned const static tbl[5] = { 128, 128, 128, 64, 64 };
        return tbl[step_no % 5];
    }
    case 26: {
        unsigned const static tbl[3] = { 128, 64, 64 };
        return tbl[step_no % 3];
    }
    case 27: {
        unsigned const static tbl[7] = { 128, 64, 64, 64, 64, 64, 64 };
        return tbl[step_no % 7];
    }
    case 28:
        return 64;
    case 29: {
        unsigned const static tbl[5] = { 64, 64, 64, 32, 32 };
        return tbl[step_no % 5];
    }
    case 30: {
        unsigned const static tbl[3] = { 64, 32, 32 };
        return tbl[step_no % 3];
    }
    case 31: {
        unsigned const static tbl[7] = { 64, 32, 32, 32, 32, 32, 32 };
        return tbl[step_no % 7];
    }
    case 32:
        return 32;
    case 33: {
        unsigned const static tbl[5] = { 32, 32, 32, 16, 16 };
        return tbl[step_no % 5];
    }
    case 34: {
        unsigned const static tbl[3] = { 32, 16, 16 };
        return tbl[step_no % 3];
    }
    case 35: {
        unsigned const static tbl[7] = { 32, 16, 16, 16, 16, 16, 16 };
        return tbl[step_no % 7];
    }
    case 36:
        return 16;
    case 37: {
        unsigned const static tbl[5] = { 16, 16, 16, 8, 8 };
        return tbl[step_no % 5];
    }
    case 38: {
        unsigned const static tbl[3] = { 16, 8, 8 };
        return tbl[step_no % 3];
    }
    case 39: {
        unsigned const static tbl[7] = { 16, 8, 8, 8, 8, 8, 8 };
        return tbl[step_no % 7];
    }
    case 40:
        return 8;
    case 41: {
        unsigned const static tbl[5] = { 8, 8, 8, 4, 4 };
        return tbl[step_no % 5];
    }
    case 42: {
        unsigned const static tbl[3] = { 8, 4, 4 };
        return tbl[step_no % 3];
    }
    case 43: {
        unsigned const static tbl[7] = { 8, 4, 4, 4, 4, 4, 4 };
        return tbl[step_no % 7];
    }
    case 44:
        return 4;
    case 45: {
        unsigned const static tbl[5] = { 4, 4, 4, 2, 2 };
        return tbl[step_no % 5];
    }
    case 46: {
        unsigned const static tbl[3] = { 4, 2, 2 };
        return tbl[step_no % 3];
    }
    case 47: {
        unsigned const static tbl[7] = { 4, 2, 2, 2, 2, 2, 2 };
        return tbl[step_no % 7];
    }
    default:
        return 2;
    }
}

static unsigned const attack_step_delta[][4] = {
    { 4, 4, 4, 4 }, // 0x30
    { 3, 4, 4, 4 }, // 0x31
    { 3, 4, 3, 4 }, // 0x32
    { 3, 3, 3, 4 }, // 0x33
    { 3, 3, 3, 3 }, // 0x34
    { 2, 3, 3, 3 }, // 0x35
    { 2, 3, 2, 3 }, // 0x36
    { 2, 2, 2, 3 }, // 0x37
    { 2, 2, 2, 2 }, // 0x38
    { 1, 2, 2, 2 }, // 0x39
    { 1, 2, 1, 2 }, // 0x3a
    { 1, 1, 1, 1 }, // 0x3b
    { 1, 1, 1, 1 }  // 0x3c
};

static unsigned const decay_step_delta[][4] = {
    { 1, 1, 1, 1 }, // 0x30
    { 2, 1, 1, 1 }, // 0x31
    { 2, 1, 2, 1 }, // 0x32
    { 2, 2, 2, 1 }, // 0x33
    { 2, 2, 2, 2 }, // 0x34
    { 4, 2, 2, 2 }, // 0x35
    { 4, 2, 4, 2 }, // 0x36
    { 4, 4, 4, 2 }, // 0x37
    { 4, 4, 4, 4 }, // 0x38
    { 8, 4, 4, 4 }, // 0x39
    { 8, 4, 8, 4 }, // 0x3a
    { 8, 8, 8, 4 }, // 0x3b
    { 8, 8, 8, 8 }  // 0x3c
};

static inline int32_t add_sample32(int32_t s1, int32_t s2) {
    if ((s2 > 0) && (s1 > INT32_MAX - s2))
        return INT32_MAX;
    if ((s2 < 0) && (s1 < INT32_MIN - s2))
        return INT32_MIN;
    return s1 + s2;
}

static aica_sample_pos get_sample_rate_multiplier(struct aica_chan const *chan) {
    // add 1.0 to the mantissa
    aica_sample_pos mantissa = (chan->fns ^ 0x400);

    /*
     * subtract 10 because thats how many bits of precision the mantissa has in
     * AICA's format
     */
    int log = get_octave_signed(chan) + (AICA_SAMPLE_POS_SHIFT - 10);

    if (log > 0)
        return mantissa << log;
    else
        return mantissa >> (-log);
}

static int get_octave_signed(struct aica_chan const *chan) {
    int oct_signed = chan->octave;
    return (oct_signed ^ 8) - 8;
}

static aica_atten atten_scale(unsigned atten) {
    if (atten > 0x3bf) atten = 0x3bf;
    unsigned mantissa = atten & 0x3f;
    unsigned log = atten >> 6;
    mantissa = ((~mantissa) + 0x40);
    mantissa <<= (AICA_ATTEN_SHIFT - 6);
    unsigned scale = mantissa >> log;
    if (scale > AICA_ATTEN_UNIT) {
        LOG_ERROR("scale is %u!\n", scale);
        LOG_ERROR("atten was 0x%03x\n", atten);
        LOG_ERROR("log was %u\n", log);
        LOG_ERROR("mantissa was 0x%06x\n", mantissa);
        RAISE_ERROR(ERROR_INTEGRITY);
    }
    return scale;
}

static void aica_process_sample(struct aica *aica) {
    unsigned chan_no;

    int32_t sample_total = 0;

    for (chan_no = 0; chan_no < AICA_CHAN_COUNT; chan_no++) {
        struct aica_chan *chan = aica->channels + chan_no;

        if (!chan->playing)
            continue;

        aica_sample_pos sample_rate =
            get_sample_rate_multiplier(chan) / AICA_FREQ_RATIO;
        unsigned effective_rate = aica_chan_effective_rate(aica, chan_no);
        unsigned samples_per_step = aica_samples_per_step(effective_rate,
                                                          chan->step_no);

        bool did_increment = false;
        if (chan->fmt == AICA_FMT_16_BIT_SIGNED) {
            int32_t sample =
                (int32_t)(int16_t)aica_wave_mem_read_16(chan->addr_cur,
                                                        &aica->mem);
            // TODO: linear interpolation
            if (!chan->is_muted)
                sample_total = add_sample32(sample_total, sample);

            chan->sample_partial += sample_rate;
            while (chan->sample_partial >= AICA_SAMPLE_POS_UNIT) {
                chan->sample_partial -= AICA_SAMPLE_POS_UNIT;
                chan->addr_cur += 2;
                chan->sample_pos++;
                did_increment = true;
            }
        } else if (chan->fmt == AICA_FMT_8_BIT_SIGNED) {
            int32_t sample =
                (int32_t)(int8_t)aica_wave_mem_read_8(chan->addr_cur,
                                                      &aica->mem);
            sample = sat_shift(sample, 8);

            // TODO: linear interpolation
            if (!chan->is_muted)
                sample_total = add_sample32(sample_total, sample);

            chan->sample_partial += sample_rate;
            while (chan->sample_partial >= AICA_SAMPLE_POS_UNIT) {
                chan->sample_partial -= AICA_SAMPLE_POS_UNIT;
                chan->addr_cur++;
                chan->sample_pos++;
                did_increment = true;
            }
        } else {
            // 4-bit ADPCM
            if (chan->adpcm_next_step) {
                uint8_t sample = aica_wave_mem_read_8(chan->addr_cur, &aica->mem);
                if (chan->sample_pos & 1)
                    sample = (sample >> 4) & 0xf;
                else
                    sample &= 0xf;

                chan->adpcm_sample = adpcm_yamaha_expand_nibble(chan, sample);
                chan->adpcm_next_step = false;
            }

            int32_t sample = chan->adpcm_sample;

            if (!chan->is_muted)
                sample_total = add_sample32(sample_total, sample);

            chan->sample_partial += sample_rate;
            if (chan->sample_partial >= AICA_SAMPLE_POS_UNIT) {
                chan->sample_partial -= AICA_SAMPLE_POS_UNIT;
                if (chan->sample_pos & 1) {
                    chan->addr_cur++;
                }
                chan->sample_pos++;
                did_increment = true;
                chan->adpcm_next_step = true;
            }
        }

        if (chan->sample_pos > chan->loop_end) {
            aica_chan_reset_adpcm(chan);

            if (!chan->loop_end_signaled)
                chan->loop_end_playstatus_flag = true;

            if (chan->loop_en) {
                chan->sample_pos = chan->loop_start;
                switch (chan->fmt) {
                case AICA_FMT_16_BIT_SIGNED:
                    chan->addr_cur = chan->addr_start + chan->loop_start * 2;
                    break;
                case AICA_FMT_8_BIT_SIGNED:
                    chan->addr_cur = chan->addr_start + chan->loop_start;
                    break;
                default:
                    // 4-bit ADPCM
                    chan->addr_cur = chan->addr_start + chan->loop_start / 2;
                }
            } else {
                chan->sample_pos = chan->loop_end;
                chan->addr_cur = chan->loop_end;
                chan->loop_end_signaled = true;
            }
        }

        if (did_increment) {
            chan->sample_no++;
            if (samples_per_step && chan->sample_no >= samples_per_step) {
                unsigned step_mod = chan->step_no % 4;
                unsigned rate_idx;
                if (effective_rate >= 0x30 && effective_rate <= 0x3c)
                    rate_idx = effective_rate - 0x30;
                else if (effective_rate < 0x30)
                    rate_idx = 0;
                else
                    rate_idx = 0x3c - 0x30;

                if (chan->atten_env_state == AICA_ENV_ATTACK) {
                    chan->atten -=
                        (chan->atten >> attack_step_delta[rate_idx][step_mod]) + 1;
                    if (!chan->atten) {
                        chan->atten_env_state = AICA_ENV_DECAY;
                    }
                } else {
                    chan->atten += decay_step_delta[rate_idx][step_mod];

                    if (chan->atten >= 0x3bf)
                        chan->atten = 0x1fff;

                    if (chan->atten_env_state == AICA_ENV_DECAY) {
                        if (chan->atten >= chan->decay_level)
                            chan->atten_env_state = AICA_ENV_SUSTAIN;
                    } else {
                        // sustain or release
                        if (chan->atten >= 0x3bf)
                            chan->playing = false;
                    }
                }

                chan->sample_no = 0;
                chan->step_no++;
            }
        }
    }

    dc_submit_sound_samples(&sample_total, 1);
}

static void raise_aica_sh4_int(struct aica *aica) {
    holly_raise_ext_int(HOLLY_EXT_INT_AICA);
    aica->int_pending_sh4 |= (1<<5);
    aica->aica_sh4_int_scheduled = false;
}

static void post_delay_raise_aica_sh4_int(struct SchedEvent *event) {
    struct aica *aica = (struct aica*)event->arg_ptr;
    if (!aica->aica_sh4_int_scheduled) {
        aica->aica_sh4_int_scheduled = true;
        aica->aica_sh4_raise_event.when =
            clock_cycle_stamp(aica->clk) + AICA_SH4_INT_DELAY;
        sched_event(aica->sh4_clk, &aica->aica_sh4_raise_event);
    }
}

#ifdef ENABLE_LOG_DEBUG
#define AICA_REG_NAME_CASE(reg) case (reg): return #reg
static char const *aica_chan_reg_name(int idx) {
    static char tmp[32];
    switch (idx) {
        AICA_REG_NAME_CASE(AICA_CHAN_PLAY_CTRL);
        AICA_REG_NAME_CASE(AICA_CHAN_SAMPLE_ADDR_LOW);
        AICA_REG_NAME_CASE(AICA_CHAN_LOOP_START);
        AICA_REG_NAME_CASE(AICA_CHAN_LOOP_END);
        AICA_REG_NAME_CASE(AICA_CHAN_AMP_ENV1);
        AICA_REG_NAME_CASE(AICA_CHAN_AMP_ENV2);
        AICA_REG_NAME_CASE(AICA_CHAN_SAMPLE_RATE_PITCH);
        AICA_REG_NAME_CASE(AICA_CHAN_LFO_CTRL);
        AICA_REG_NAME_CASE(AICA_CHAN_DSP_SEND);
        AICA_REG_NAME_CASE(AICA_CHAN_DIR_PAN_VOL_SEND);
        AICA_REG_NAME_CASE(AICA_CHAN_LPF1_VOL);
        AICA_REG_NAME_CASE(AICA_CHAN_LPF2);
        AICA_REG_NAME_CASE(AICA_CHAN_LPF3);
        AICA_REG_NAME_CASE(AICA_CHAN_LPF4);
        AICA_REG_NAME_CASE(AICA_CHAN_LPF5);
        AICA_REG_NAME_CASE(AICA_CHAN_LPF6);
        AICA_REG_NAME_CASE(AICA_CHAN_LPF7);
        AICA_REG_NAME_CASE(AICA_CHAN_LPF8);
    default:
        snprintf(tmp, sizeof(tmp), "unknown channel register 0x%04x", idx);
        tmp[31] = '\0';
        return tmp;
    }
}
#endif

void aica_get_sndchan_stat(struct aica const *aica,
                           unsigned ch_no,
                           struct washdc_sndchan_stat *stat) {
    if (ch_no < AICA_CHAN_COUNT) {
        stat->playing = aica->channels[ch_no].playing;
        stat->n_vars = 19;
        stat->ch_idx = ch_no;
    } else {
        LOG_ERROR("%s - AICA INVALID CHANNEL INDEX %u\n", __func__, ch_no);
        stat->playing = false;
    }
}

void aica_get_sndchan_var(struct aica const *aica,
                          struct washdc_sndchan_stat const *stat,
                          unsigned var_no,
                          struct washdc_var *var) {
    double sample_rate;
    if (stat->ch_idx >= AICA_CHAN_COUNT)
        goto inval;
    struct aica_chan const *chan = aica->channels + stat->ch_idx;
    switch (var_no) {
    case 0:
        strncpy(var->name, "ready_keyon", WASHDC_VAR_NAME_LEN);
        var->name[WASHDC_VAR_NAME_LEN - 1] = '\0';
        var->tp = WASHDC_VAR_BOOL;
        var->val.as_bool = chan->ready_keyon;
        return;
    case 1:
        strncpy(var->name, "attenuation", WASHDC_VAR_NAME_LEN);
        var->name[WASHDC_VAR_NAME_LEN - 1] = '\0';
        var->tp = WASHDC_VAR_HEX;
        var->val.as_int = chan->atten;
        return;
    case 2:
        strncpy(var->name, "atten-scale", WASHDC_VAR_NAME_LEN);
        var->name[WASHDC_VAR_NAME_LEN - 1] = '\0';
        var->tp = WASHDC_VAR_DOUBLE;
        var->val.as_double =
            (double)atten_scale(chan->atten) / (double)AICA_ATTEN_UNIT;
        return;
    case 3:
        strncpy(var->name, "octave", WASHDC_VAR_NAME_LEN);
        var->name[WASHDC_VAR_NAME_LEN - 1] = '\0';
        var->tp = WASHDC_VAR_HEX;
        var->val.as_int = chan->octave;
        return;
    case 4:
        strncpy(var->name, "FNS", WASHDC_VAR_NAME_LEN);
        var->name[WASHDC_VAR_NAME_LEN - 1] = '\0';
        var->tp = WASHDC_VAR_HEX;
        var->val.as_int = chan->fns;
        return;
    case 5:
        sample_rate =
            ((uint64_t)get_sample_rate_multiplier(chan) * (uint64_t)44100) >> AICA_SAMPLE_POS_SHIFT;
        strncpy(var->name, "Sample Rate", WASHDC_VAR_NAME_LEN);
        var->name[WASHDC_VAR_NAME_LEN - 1] = '\0';
        var->tp = WASHDC_VAR_INT;
        var->val.as_int = (int)sample_rate;
        return;
    case 6:
        strncpy(var->name, "Effective Rate", WASHDC_VAR_NAME_LEN);
        var->name[WASHDC_VAR_NAME_LEN - 1] = '\0';
        var->tp = WASHDC_VAR_INT;
        var->val.as_int = aica_chan_effective_rate(aica, stat->ch_idx);
        return;
    case 7:
        strncpy(var->name, "Envelope State", WASHDC_VAR_NAME_LEN);
        var->name[WASHDC_VAR_NAME_LEN - 1] = '\0';
        var->tp = WASHDC_VAR_STR;
        switch(chan->atten_env_state) {
        case AICA_ENV_ATTACK:
            strncpy(var->val.as_str, "attack", WASHDC_VAR_STR_LEN);
            break;
        case AICA_ENV_DECAY:
            strncpy(var->val.as_str, "decay", WASHDC_VAR_STR_LEN);
            break;
        case AICA_ENV_SUSTAIN:
            strncpy(var->val.as_str, "sustain", WASHDC_VAR_STR_LEN);
            break;
        case AICA_ENV_RELEASE:
            strncpy(var->val.as_str, "release", WASHDC_VAR_STR_LEN);
            break;
        default:
            strncpy(var->val.as_str, "unknown (ERROR!)", WASHDC_VAR_STR_LEN);
            break;
        }
        var->val.as_str[WASHDC_VAR_STR_LEN - 1] = '\0';
        return;
    case 8:
        strncpy(var->name, "Format", WASHDC_VAR_NAME_LEN);
        var->name[WASHDC_VAR_NAME_LEN - 1] = '\0';
        var->tp = WASHDC_VAR_STR;
        strncpy(var->val.as_str, fmt_name(chan->fmt), WASHDC_VAR_STR_LEN);
        var->val.as_str[WASHDC_VAR_STR_LEN - 1] = '\0';
        return;
    case 9:
        //addr_start
        strncpy(var->name, "Start Address", WASHDC_VAR_NAME_LEN);
        var->name[WASHDC_VAR_NAME_LEN - 1] = '\0';
        var->tp = WASHDC_VAR_HEX;
        var->val.as_int = chan->addr_start;
        return;
    case 10:
        // loop_start
        strncpy(var->name, "Loop Start", WASHDC_VAR_NAME_LEN);
        var->name[WASHDC_VAR_NAME_LEN - 1] = '\0';
        var->tp = WASHDC_VAR_HEX;
        var->val.as_int = chan->loop_start;
        return;
    case 11:
        //loop_end
        strncpy(var->name, "Loop End", WASHDC_VAR_NAME_LEN);
        var->name[WASHDC_VAR_NAME_LEN - 1] = '\0';
        var->tp = WASHDC_VAR_HEX;
        var->val.as_int = chan->loop_end;
        return;
    case 12:
        // loop_enable
        strncpy(var->name, "Loop Enable", WASHDC_VAR_NAME_LEN);
        var->name[WASHDC_VAR_NAME_LEN - 1] = '\0';
        var->tp = WASHDC_VAR_BOOL;
        var->val.as_bool = chan->loop_en;
        return;
    case 13:
        // direct send volume (DISDL)
        strncpy(var->name, "volume", WASHDC_VAR_NAME_LEN);
        var->name[WASHDC_VAR_NAME_LEN - 1] = '\0';
        var->tp = WASHDC_VAR_HEX;
        var->val.as_int = chan->volume;
        return;
    case 14:
        // direct send pan (DIPAN)
        strncpy(var->name, "pan", WASHDC_VAR_NAME_LEN);
        var->name[WASHDC_VAR_NAME_LEN - 1] = '\0';
        var->tp = WASHDC_VAR_HEX;
        var->val.as_int = chan->pan;
        return;
    case 15:
        // attack-rate
        strncpy(var->name, "attack-rate", WASHDC_VAR_NAME_LEN);
        var->name[WASHDC_VAR_NAME_LEN - 1] = '\0';
        var->tp = WASHDC_VAR_HEX;
        var->val.as_int = chan->attack_rate;
        return;
    case 16:
        // decay-rate
        strncpy(var->name, "decay-rate", WASHDC_VAR_NAME_LEN);
        var->name[WASHDC_VAR_NAME_LEN - 1] = '\0';
        var->tp = WASHDC_VAR_HEX;
        var->val.as_int = chan->decay_rate;
        return;
    case 17:
        // sustain-rate
        strncpy(var->name, "sustain-rate", WASHDC_VAR_NAME_LEN);
        var->name[WASHDC_VAR_NAME_LEN - 1] = '\0';
        var->tp = WASHDC_VAR_HEX;
        var->val.as_int = chan->sustain_rate;
        return;
    case 18:
        // release-rate
        strncpy(var->name, "release-rate", WASHDC_VAR_NAME_LEN);
        var->name[WASHDC_VAR_NAME_LEN - 1] = '\0';
        var->tp = WASHDC_VAR_HEX;
        var->val.as_int = chan->release_rate;
        return;
    default:
        goto inval;
    }
 inval:
    memset(var, 0, sizeof(*var));
    var->tp = WASHDC_VAR_INVALID;
}

/*
 * This is ultimately called from the UI code when the user wants to forcibly
 * mute a channel.
 */
void aica_mute_chan(struct aica *aica, unsigned chan_no, bool is_muted) {
    aica->channels[chan_no].is_muted = is_muted;
}
