/**
 * Copyright (c) 2015 Google Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Kim Mui
 * @brief GDMAC driver
 */

#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/device.h>
#include <nuttx/device_dma.h>

#include "debug.h"
#include "up_arch.h"

#include "tsb_scm.h"

#include "chip.h"
#include "tsb_dma_gdmac_asm.h"
#include "tsb_dma.h"

#define TSB_DMA_GDMAC_CHANNELS        8
#define TSB_DMA_GDMAC_EVENTS          32

#define TSB_DMA_GDMAC_INVALID_EVENT   0xFFFFFFFF

/* GDMAC register addresses */
#define GDMAC_CONTROL_REGS_OFFSET   0x0000
#define GDMAC_CHANNEL_REGS_OFFSET   0x0100
#define GDMAC_DBG_REGS_OFFSET       0x0D00
#define GDMAC_CONFIG_REGS_OFFSET    0x0E00
#define GDMAC_ID_REGS_OFFSET        0x0FE0

#define GDMAC_CONTROL_REGS_ADDRESS  (GDMAC_BASE + GDMAC_CONTROL_REGS_OFFSET)
#define GDMAC_CHANNEL_REGS_ADDRESS  (GDMAC_BASE + GDMAC_CHANNEL_REGS_OFFSET)
#define GDMAC_DBG_REGS_ADDRESS      (GDMAC_BASE + GDMAC_DBG_REGS_OFFSET)
#define GDMAC_CONFIG_REGS_ADDRESS   (GDMAC_BASE + GDMAC_CONFIG_REGS_OFFSET)
#define GDMAC_ID_REGS_ADDRESS       (GDMAC_BASE + GDMAC_ID_REGS_OFFSET)

/* Define PL330 events associated with each channel. */
#define GDMAC_END_OF_TX_EVENT(channel_info)  (channel_info->end_of_tx_event)
#define GDMAC_NEXT_TX_EVENT(channel_info)    (channel_info->next_tx_event)
#define GDMAC_EVENT_MASK(event)              (1 << event)
#define DMA_CHANNEL_TO_TSB_IRQ(event_id)     (event_id + TSB_IRQ_GDMAC00)
#define IRQN_TO_DMA_CHANNEL(irqn)            \
    ((irqn - TSB_IRQ_GDMAC00 >= TSB_DMA_GDMAC_CHANNELS) ?\
     DEVICE_DMA_INVALID_CHANNEL :            \
     gdmac_event_to_channel_map[irqn - TSB_IRQ_GDMAC00])

#define GDMAC_DO_LOOPBACK                   (1)
#define GDMAC_DONOT_LOOPBACK                (0)

#define UNIPRO_LOOPBACK_COUNT(src, dst)     \
    (offsetof(tsb_dma_gdmac_unipro_tx_channel, src) - \
     offsetof(tsb_dma_gdmac_unipro_tx_channel, dst))

#define GDMAC_PROGRAM_SET_EVENT(event_field, event) \
    putreg8(getreg8(&event_field) | event << 3, &event_field)

/* Structures for PL330 GDMAC registers */
typedef struct {
    uint32_t dsr;
    uint32_t dpc;
    uint32_t pad0[6];
    uint32_t inten;
    uint32_t int_event_ris;
    uint32_t intmis;
    uint32_t intclr;
    uint32_t fsrd;
    uint32_t fsrc;
    uint32_t ftrd;
    uint32_t pad1;
    uint32_t ftr[TSB_DMA_GDMAC_CHANNELS];
} tsb_dma_gdmac_control_regs;

typedef struct {
    uint32_t csr;
    uint32_t cpc;
} tsb_dma_gdmac_channel_status_regs;

typedef struct {
    uint32_t sar;
    uint32_t dar;
    uint32_t ccr;
    uint32_t lc0;
    uint32_t lc1;
} tsb_dma_gdmac_channel_control_regs;

typedef struct {
    tsb_dma_gdmac_channel_status_regs
        channel_status_regs[TSB_DMA_GDMAC_CHANNELS];
    tsb_dma_gdmac_channel_control_regs
        channel_control_regs[TSB_DMA_GDMAC_CHANNELS];
} tsb_dma_gdmac_channel_regs;

typedef struct {
    uint32_t dbg_status;
    uint32_t dbg_cmd;
    uint32_t dbg_inst_0;
    uint32_t dbg_inst_1;
} tsb_dma_gdmac_dbg_regs;

typedef struct {
    uint32_t cr0;
    uint32_t cr1;
    uint32_t cr2;
    uint32_t cr3;
    uint32_t cr4;
    uint32_t crd;
} tsb_dma_gdmac_config_regs;

typedef struct {
    uint32_t periph_id_0;
    uint32_t periph_id_1;
    uint32_t periph_id_2;
    uint32_t periph_id_3;
    uint32_t pcell_id_0;
    uint32_t pcell_id_1;
    uint32_t pcell_id_2;
    uint32_t pcell_id_3;
} tsb_dma_gdmac_id_regs;

#define GDMAC_INSTR_DMAMOV(value_name)      \
    struct __attribute__((__packed__))      \
    { uint8_t instr[2]; uint32_t value; } value_name
#define GDMAC_INSTR_DMASEV(value_name)      \
    struct __attribute__((__packed__))      \
    { uint8_t instr[1]; uint8_t value; } value_name
#define GDMAC_INSTR_DMALP(value_name)       \
    struct __attribute__((__packed__))      \
    { uint8_t instr[1]; uint8_t value; } value_name
#define GDMAC_INSTR_DMAWFE(value_name)      \
    struct __attribute__((__packed__))      \
    { uint8_t instr[1]; uint8_t value; } value_name
#define GDMAC_INSTR_DMALPEND(value_name)    \
    struct __attribute__((__packed__))      \
    { uint8_t instr[1]; uint8_t value; } value_name

/* structure for UniPro TX channel information. */
typedef struct {
    unsigned int end_of_tx_event;
    unsigned int next_tx_event;

    /* Channel program. */
    struct __attribute__((__packed__)) {
        uint8_t gdmac_program[0];
        uint8_t unipro_tx_prg_end[DMA_END_SIZE];
        uint8_t unipro_eom[0];
        GDMAC_INSTR_DMAMOV(unipro_eom_value_addr);
        GDMAC_INSTR_DMAMOV(unipro_eom_reg_addr);
        uint8_t pad0[DMA_LD_SIZE + DMA_ST_SIZE + DMA_WMB_SIZE];
        GDMAC_INSTR_DMASEV(unipro_eom_tx_event);
        uint8_t pad10[DMA_END_SIZE];
        uint8_t unipro_starting_tx[0];
        GDMAC_INSTR_DMAMOV(unipro_src_addr);
        GDMAC_INSTR_DMAMOV(unipro_dst_addr);
        uint8_t unipro_continue_tx[0];
        GDMAC_INSTR_DMALP(unpro_len_loopcount);
        uint8_t pad20[DMA_LD_SIZE + DMA_ST_SIZE + DMA_LPEND_SIZE];
        GDMAC_INSTR_DMALP(unpro_eom_loopcount);
        uint8_t unipro_eom_loopback[DMA_LPEND_SIZE + DMA_WMB_SIZE];
        GDMAC_INSTR_DMASEV(unipro_end_of_tx_event);
        GDMAC_INSTR_DMAWFE(unipro_next_tx_event);
        GDMAC_INSTR_DMALP(unipro_next_loopcount);
        GDMAC_INSTR_DMALPEND(unipro_next_loopback);
        uint8_t pad30[DMA_END_SIZE];
        /* PL330 program starting point */
        uint8_t unipro_program_entry_point[DMA_MOV_SIZE];
        GDMAC_INSTR_DMALP(unipro_start_loopcount);
        GDMAC_INSTR_DMALPEND(unipro_start_tx_loopback);
        uint8_t pad40[DMA_LP_SIZE];
        GDMAC_INSTR_DMALPEND(unipro_start_eom_loopback);
        uint8_t pad50[DMA_END_SIZE];
    };
} tsb_dma_gdmac_unipro_tx_channel;

typedef enum {
    GDMAC_CHANNEL_STOPPED,
    GDMAC_CHANNEL_RUNNING,
    GDMAC_CHANNEL_BLOCKED,
    GDMAC_CHANNEL_KILLING,
    GDMAC_CHANNEL_UNDEFINED
} gdmac_channel_state;

typedef gdmac_channel_state (*gdmac_process_callback_cmd)(struct device *,
        struct tsb_dma_channel *, enum device_dma_cmd);

struct tsb_dma_channel_info {
    gdmac_channel_state state;
    device_dma_transfer_arg *current_transfer_arg;
    gdmac_process_callback_cmd process_callback_cmd;
    union {
        tsb_dma_gdmac_unipro_tx_channel unipro_tx_channel;
    /* other GDMAC channels will be added as more GDMAC channels are supported
     */
    };
};

/* template for UniPro Tx GDMAC binary code. */
static const uint8_t tsb_dma_gdma_unipro_tx_prg[] = {
        /* unipro_tx_prg_end */
        DMAEND,
        /* unipro_eom */
        DMAMOV(sar, 0),
        DMAMOV(dar, 0),
        DMALD,
        DMAST,
        DMAWMB,
        DMASEV(0),
        DMAEND,
        /* unipro_starting_tx */
        DMAMOV(sar, 0),
        DMAMOV(dar, 0),
        /* unipro_continue_tx */
        DMALP(lc0, 0),
        DMALD,
        DMAST,
        DMALPEND(lc0, DMA_LD_SIZE + DMA_ST_SIZE),
        DMALP(lc0, GDMAC_DONOT_LOOPBACK),
        DMALPEND(lc0,
                UNIPRO_LOOPBACK_COUNT(unipro_eom_loopback, unipro_eom)),
        DMAWMB,
        DMASEV(0),
        DMAWFE(0, invalid),
        DMALP(lc0, GDMAC_DONOT_LOOPBACK),
        DMALPEND(lc0, 0),
        /* DMA execution starts here. Not the first instruction. */
        DMAEND,
        DMAMOV(ccr, 0x01C04701),
        DMALP(lc0, GDMAC_DO_LOOPBACK),
        DMALPEND(lc0,
                UNIPRO_LOOPBACK_COUNT(unipro_start_tx_loopback, unipro_starting_tx)),
        DMALP(lc0, GDMAC_DO_LOOPBACK),
        DMALPEND(lc0,
                UNIPRO_LOOPBACK_COUNT(unipro_start_eom_loopback, unipro_eom)),
        DMAEND
};

/* A fixed value write to UniPro EOM register after EOM packet was sent. */
static const uint8_t eom_value = 0x01;

/* GDMAC device */
struct device *tsb_dma_gdmac_dev = NULL;

static unsigned int gdmac_event_to_channel_map[TSB_DMA_GDMAC_EVENTS];

static inline gdmac_channel_state gdmac_get_channel_state(
        unsigned int channel_id)
{
    tsb_dma_gdmac_channel_regs *gdmac_channel_regs =
            (tsb_dma_gdmac_channel_regs *) GDMAC_CHANNEL_REGS_ADDRESS;
    gdmac_channel_state channel_state = GDMAC_CHANNEL_UNDEFINED;
    uint32_t csr_reg =
            getreg32(&gdmac_channel_regs->channel_status_regs[channel_id].csr);

    switch (csr_reg & 0x0F) {
    case 0x0:
        channel_state = GDMAC_CHANNEL_STOPPED;
        break;
    case 0x1:
    case 0x2:
    case 0x3:
    case 0x5:
    case 0x7:
    case 0x9:
        channel_state = GDMAC_CHANNEL_RUNNING;
        break;
    case 0x4:
        channel_state = GDMAC_CHANNEL_BLOCKED;
        break;
    case 0x8:
        channel_state = GDMAC_CHANNEL_KILLING;
        break;
    default:
        channel_state = GDMAC_CHANNEL_UNDEFINED;
        break;
    }

    return channel_state;
}

static inline enum device_dma_event gdmac_get_device_dma_event(
        unsigned int channel_id)
{
    switch (gdmac_get_channel_state(channel_id)) {
    case GDMAC_CHANNEL_STOPPED:
        return DEVICE_DMA_EVENT_COMPLETED;
        break;
    case GDMAC_CHANNEL_BLOCKED:
        return DEVICE_DMA_EVENT_BLOCKED;
        break;
    default:
        break;
    }

    return DEVICE_DMA_EVENT_INVALID;
}

static inline unsigned int gdmac_allocate_event(unsigned int channel_id)
{
    unsigned int index;

    for (index = 0; index < TSB_DMA_GDMAC_EVENTS; index++) {
        if (gdmac_event_to_channel_map[index]
                == DEVICE_DMA_INVALID_CHANNEL) {
            gdmac_event_to_channel_map[index] = channel_id;
            return index;
        }
    }

    return TSB_DMA_GDMAC_INVALID_EVENT;
}

static inline void gdmac_release_event(unsigned int event)
{
    if (event < TSB_DMA_GDMAC_EVENTS) {
        gdmac_event_to_channel_map[event] = DEVICE_DMA_INVALID_CHANNEL;
    }
}

static inline void dma_execute_instruction(uint8_t* insn)
{
    tsb_dma_gdmac_dbg_regs *dbg_regs =
            (tsb_dma_gdmac_dbg_regs*) GDMAC_DBG_REGS_ADDRESS;
    uint32_t value;

    value = (insn[0] << 16) | (insn[1] << 24);
    putreg32(value, &dbg_regs->dbg_inst_0);

    value = *(uint32_t *) &insn[2];
    putreg32(value, &dbg_regs->dbg_inst_1);

    /* Get going */
    putreg32(0, &dbg_regs->dbg_cmd);
}

static bool inline dma_start_thread(uint8_t thread_id,
        uint8_t* program_addr)
{
    uint8_t go_instr[] = { DMAGO(thread_id, ns, program_addr) };

    dma_execute_instruction(&go_instr[0]);

    return true;
}

static bool inline dma_signal_event(uint8_t event_id)
{
    uint8_t go_instr[] = { DMASEV(event_id), 0, 0, 0, 0, 0, 0, 0 };

    dma_execute_instruction(&go_instr[0]);

    return true;
}

int tsb_dma_irq_handler(int irq, void *context)
{
    tsb_dma_gdmac_control_regs *control_regs =
            (tsb_dma_gdmac_control_regs*) GDMAC_CONTROL_REGS_ADDRESS;
    unsigned int channel = IRQN_TO_DMA_CHANNEL(irq);
    (void) context;

    if (channel != DEVICE_DMA_INVALID_CHANNEL) {
        putreg32(GDMAC_EVENT_MASK(channel), &control_regs->intclr);

        if (tsb_dma_gdmac_dev == NULL) {
            lldbg("Invalid device handle.\n");
        } else {
            struct tsb_dma_info *info = device_get_private(
                    tsb_dma_gdmac_dev);
            struct tsb_dma_channel_info *channel_info =
                    info->dma_channel[channel].channel_info;
            enum device_dma_cmd callback_cmd;

            callback_cmd = tsb_dma_transfer_done_callback(tsb_dma_gdmac_dev,
                    channel, gdmac_get_device_dma_event(channel),
                    channel_info->current_transfer_arg);

            if ((callback_cmd != DEVICE_DMA_CMD_NONE)
                    && (channel_info->process_callback_cmd != NULL)) {
                channel_info->state = channel_info->process_callback_cmd(
                        tsb_dma_gdmac_dev, &info->dma_channel[channel],
                        callback_cmd);
            } else {

            }
        }
    } else {
        lldbg("received invalid DMA interrupt(%d)\n", irq);
    }

    return OK;
}

void gdmac_unipro_tx_release_channel(struct tsb_dma_channel *channel)
{
    if (channel->channel_info != NULL) {
        struct tsb_dma_channel_info *channel_info = channel->channel_info;

        gdmac_release_event(
                channel_info->unipro_tx_channel.end_of_tx_event);
        gdmac_release_event(
                channel_info->unipro_tx_channel.next_tx_event);

        free(channel_info);
        channel->channel_info = NULL;
        channel->do_dma_transfer = NULL;
        channel->release_channel = NULL;
    }
}

static gdmac_channel_state gdmac_unipro_stop_tx(
        struct tsb_dma_channel *channel)
{
    tsb_dma_gdmac_unipro_tx_channel *unipro_tx_channel =
            &channel->channel_info->unipro_tx_channel;

    putreg32(GDMAC_DONOT_LOOPBACK,
             &unipro_tx_channel->unipro_next_loopcount.value);

    dma_signal_event(unipro_tx_channel->next_tx_event);

    return GDMAC_CHANNEL_STOPPED;
}

static gdmac_channel_state gdmac_unipro_continue_tx(
        struct tsb_dma_channel *channel)
{
    gdmac_channel_state ret = GDMAC_CHANNEL_RUNNING;

    tsb_dma_gdmac_unipro_tx_channel *unipro_tx_channel =
            &channel->channel_info->unipro_tx_channel;
    device_dma_unipro_tx_arg *unipro_tx_arg =
            &channel->channel_info->current_transfer_arg->unipro_tx_arg;

    if (unipro_tx_arg->next_transfer_len > 0) {
        if (unipro_tx_arg->eom_addr != NULL) {
            /* If this is the last fragmented message, set loop count to 2 to
             * cause the GDMAC to run the little piece of code at the very top
             *  to hit the EOM register.
             */
            unipro_tx_channel->unpro_eom_loopcount.value = GDMAC_DO_LOOPBACK;
            unipro_tx_channel->unipro_eom_reg_addr.value =
                    (uint32_t) unipro_tx_arg->eom_addr;
        } else {
            /* If this is NOT the last fragmented message, set loop count to 1 to
             * cause the GDMAC to raise the AP interrupt event and ends the
             * execution.
             */
            unipro_tx_channel->unpro_eom_loopcount.value = GDMAC_DONOT_LOOPBACK;
        }

        /* setup the lenght of next transfer and set case to 3 to perform next
         * transfer
         */
        unipro_tx_channel->unpro_len_loopcount.value =
                (uint8_t) unipro_tx_arg->next_transfer_len;
        unipro_tx_channel->unipro_next_loopcount.value = GDMAC_DO_LOOPBACK;
        unipro_tx_channel->unipro_next_loopback.value =
                UNIPRO_LOOPBACK_COUNT(unipro_next_loopback, unipro_continue_tx);
    } else {
        if (unipro_tx_arg->eom_addr != NULL) {
            unipro_tx_channel->unipro_next_loopcount.value = GDMAC_DO_LOOPBACK;
            unipro_tx_channel->unipro_eom_reg_addr.value =
                    (uint32_t) unipro_tx_arg->eom_addr;
            unipro_tx_channel->unipro_next_loopback.value =
                    UNIPRO_LOOPBACK_COUNT(unipro_next_loopback, unipro_eom);
        } else {
            lldbg("ERROR: cmd is continue and len is 0 and eom if false. Stopping GDMAC.\n");
            unipro_tx_channel->unipro_next_loopcount.value =
                    GDMAC_DONOT_LOOPBACK;
            ret = GDMAC_CHANNEL_STOPPED;
        }
    }

    dma_signal_event(unipro_tx_channel->next_tx_event);

    return ret;
}

static gdmac_channel_state gdmac_unipro_process_callback_cmd(
        struct device *dev, struct tsb_dma_channel *dma_channel,
        enum device_dma_cmd cmd)
{
    gdmac_channel_state channel_state = gdmac_get_channel_state(
            dma_channel->channel_id);

    switch (channel_state) {
    case GDMAC_CHANNEL_STOPPED:
        if (cmd != DEVICE_DMA_CMD_STOP) {
            lldbg("GDMAC Stopped while callback command = %d\n", cmd);
        }
        break;
    case GDMAC_CHANNEL_RUNNING:
        lldbg("GDMAC is running while callback command = %d\n",
                cmd);
        break;
    case GDMAC_CHANNEL_BLOCKED:
        if (dma_channel->channel_info->state == GDMAC_CHANNEL_RUNNING) {
            if (cmd == DEVICE_DMA_CMD_CONTINUE) {
                gdmac_unipro_continue_tx(dma_channel);
            } else {
                if (cmd == DEVICE_DMA_CMD_STOP) {
                    gdmac_unipro_stop_tx(dma_channel);
                } else {
                    lldbg("GDMAC is blocked while callback cmd is %d\n",
                          cmd);
                }
            }
        } else {
            lldbg("GDMAC is blocked while channel is not in running state\n");
        }
        break;
    default:
        break;
    }

    return channel_state;
}

static int gdmac_unipro_tx_transfer(struct tsb_dma_channel *dma_channel,
        void* buf, void *tx_buf, size_t len, device_dma_transfer_arg *arg)
{
    device_dma_unipro_tx_arg *unipro_tx_arg =
            (device_dma_unipro_tx_arg*) arg;
    tsb_dma_gdmac_unipro_tx_channel *unipro_tx_channel =
            &dma_channel->channel_info->unipro_tx_channel;
    gdmac_channel_state channel_state = gdmac_get_channel_state(
            dma_channel->channel_id);
    int status;

    if ((dma_channel->channel_info->state != GDMAC_CHANNEL_STOPPED)
            && (channel_state != GDMAC_CHANNEL_STOPPED)) {
        lldbg("GDMAC is in running state.\n");
    }

    /* data len is 0 and EOM register address is not set. */
    if ((len == 0) && (unipro_tx_arg->eom_addr == NULL)) {
        return -EINVAL;
    }

    /* Set the source and destination addresses, as well as the data count. */
    unipro_tx_channel->unipro_src_addr.value = (uint32_t) buf;
    unipro_tx_channel->unipro_dst_addr.value = (uint32_t) tx_buf;
    unipro_tx_channel->unpro_len_loopcount.value = (uint8_t) len;
    if (unipro_tx_arg->eom_addr != NULL) {
        /* If this is the last fragmented message, set loop count to 2 to
         * cause the GDMAC to run the little piece of code at the very top to
         * hit the EOM register.
         */
        unipro_tx_channel->unpro_eom_loopcount.value = GDMAC_DO_LOOPBACK;
        unipro_tx_channel->unipro_eom_reg_addr.value = (uint32_t) unipro_tx_arg->eom_addr;
        if (len == 0) {
            unipro_tx_channel->unipro_start_loopcount.value =
                    GDMAC_DONOT_LOOPBACK;
            unipro_tx_channel->unipro_eom_reg_addr.value =
                    (uint32_t) unipro_tx_arg->eom_addr;
        } else {
            unipro_tx_channel->unipro_start_loopcount.value = GDMAC_DO_LOOPBACK;
        }
    } else {
        /* If this is NOT the last fragmented message, set loop count to 1 to
         * cause the GDMAC to raise the AP interrupt event and ends the
         * execution.
         */
        unipro_tx_channel->unpro_eom_loopcount.value = GDMAC_DONOT_LOOPBACK;
    }

    dma_channel->channel_info->current_transfer_arg = arg;
    dma_channel->channel_info->process_callback_cmd =
            gdmac_unipro_process_callback_cmd;

    /* Start the transfer and wait for end of transfer interrupt. */
    dma_channel->channel_info->state = GDMAC_CHANNEL_RUNNING;
    status = dma_start_thread(dma_channel->channel_id,
            &unipro_tx_channel->unipro_program_entry_point[0]);

    return status;
}

/* Allocate an UniPro TX channel. */
int tsb_dma_allocal_unipro_tx_channel(struct tsb_dma_channel *channel)
{
    int ret;
    tsb_dma_gdmac_unipro_tx_channel *unipro_tx_channel;
    struct tsb_dma_channel_info *channel_info = zalloc(
            sizeof(struct tsb_dma_channel_info));
    tsb_dma_gdmac_control_regs *control_regs =
            (tsb_dma_gdmac_control_regs*) GDMAC_CONTROL_REGS_ADDRESS;

    if (channel_info == NULL) {
        return -ENOMEM;
    }

    unipro_tx_channel = &channel_info->unipro_tx_channel;

    /* allocate an end of transfer event; */
    unipro_tx_channel->end_of_tx_event = gdmac_allocate_event(
            channel->channel_id);
    unipro_tx_channel->next_tx_event = gdmac_allocate_event(
            channel->channel_id);

    if ((unipro_tx_channel->end_of_tx_event == TSB_DMA_GDMAC_INVALID_EVENT)
            || (unipro_tx_channel->next_tx_event
                    == TSB_DMA_GDMAC_INVALID_EVENT)) {
        if (unipro_tx_channel->end_of_tx_event
                != TSB_DMA_GDMAC_INVALID_EVENT) {
            gdmac_release_event(unipro_tx_channel->end_of_tx_event);
        }

        if (unipro_tx_channel->next_tx_event != TSB_DMA_GDMAC_INVALID_EVENT) {
            gdmac_release_event(unipro_tx_channel->next_tx_event);
        }

        free(unipro_tx_channel);

        lldbg("Unable to allocate GDMAC event(s).\n");
        return -ENOMEM;
    }

    /* make a copy of the UniPro TX binary code. */
    memcpy(&unipro_tx_channel->gdmac_program[0],
            &tsb_dma_gdma_unipro_tx_prg[0],
            sizeof(tsb_dma_gdma_unipro_tx_prg));

    /* Set the source address of EOM flag. */
    unipro_tx_channel->unipro_eom_value_addr.value = (uint32_t)&eom_value;

    /* Set a pointer to the PL330 event used for this UniPro TX channel. */
    GDMAC_PROGRAM_SET_EVENT(unipro_tx_channel->unipro_eom_tx_event.value,
            GDMAC_END_OF_TX_EVENT(unipro_tx_channel));

    /* Set EOP (end-of-packet) transfer event. */
    GDMAC_PROGRAM_SET_EVENT(unipro_tx_channel->unipro_end_of_tx_event.value,
            GDMAC_END_OF_TX_EVENT(unipro_tx_channel));

    /* Set next transfer event. */
    GDMAC_PROGRAM_SET_EVENT(unipro_tx_channel->unipro_next_tx_event.value,
            GDMAC_NEXT_TX_EVENT(unipro_tx_channel));

    /* Set interrupt handler for this GDMAC UniPro TX channel. */
    ret = irq_attach(
            DMA_CHANNEL_TO_TSB_IRQ(
                    GDMAC_END_OF_TX_EVENT(unipro_tx_channel)),
            tsb_dma_irq_handler);
    if (ret != OK) {
        lldbg("Failed to attach interrupt %d.\n",
                DMA_CHANNEL_TO_TSB_IRQ(
                        GDMAC_END_OF_TX_EVENT(unipro_tx_channel)));
        return -EIO;
    } else {
        putreg32(GDMAC_EVENT_MASK(
                 GDMAC_END_OF_TX_EVENT(unipro_tx_channel)),
                 &control_regs->inten);
        up_enable_irq(
                DMA_CHANNEL_TO_TSB_IRQ(
                        GDMAC_END_OF_TX_EVENT(unipro_tx_channel)));
    }

    /* Set the transfer and transfer done handlers */
    channel->do_dma_transfer = gdmac_unipro_tx_transfer;
    channel->release_channel = gdmac_unipro_tx_release_channel;
    channel->channel_info = channel_info;

    channel_info->state = GDMAC_CHANNEL_STOPPED;

    return 0;
}

void tsb_dma_init_controller(struct device *dev)
{
    /* initialize all events are available. */
    memset(&gdmac_event_to_channel_map[0], 0xFF,
            sizeof(gdmac_event_to_channel_map));

    /* enable clock to GDMAC and reset GDMAC. */
    tsb_clk_enable(TSB_CLK_GDMA);
    tsb_reset(TSB_RST_GDMA);

    tsb_dma_gdmac_dev = dev;
}

void tsb_dma_deinit_controller(struct device *dev)
{
    tsb_dma_gdmac_dev = NULL;
}

int tsb_dma_max_number_of_channels(void)
{
    return TSB_DMA_GDMAC_CHANNELS;
}
