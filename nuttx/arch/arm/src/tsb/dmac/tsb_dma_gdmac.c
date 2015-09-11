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

#define DMA_SET_EOM_LOOPBACK_COUNT  \
    (DMA_MOV_SIZE * 5 + DMA_LD_SIZE * 2 + DMA_ST_SIZE * 2 + DMA_WMB_SIZE + \
     DMA_SEV_SIZE + DMA_END_SIZE + DMA_LP_SIZE * 2 + DMA_LPEND_SIZE)

/* Define PL330 events associated with each channel. */
#define GDMAC_END_OF_TX_EVENT(channel_info)  (channel_info->end_of_tx_event)
#define GDMAC_EVENT_MASK(channel_id)        (1 << channel_id)
#define DMA_CHANNEL_TO_TSB_IRQ(channel_id)  (channel_id + TSB_IRQ_GDMAC00)
#define IRQN_TO_DMA_CHANNEL(irqn)            \
    ((irqn - TSB_IRQ_GDMAC00 >= TSB_DMA_GDMAC_CHANNELS) ?\
     DEVICE_DMA_INVALID_CHANNEL :            \
     gdmac_event_to_channel_map[irqn - TSB_IRQ_GDMAC00])

#define GDMAC_DO_LOOPBACK                   (1)
#define GDMAC_DONOT_LOOPBACK                (0)

#define DMA_CHANNEL_STOPPED(thread_id)      \
    ((((dma_channel_regs *)                 \
     GDMAC_CHANNEL_REGS_ADDRESS)->channel_status_regs[thread_id].csr & 0x0F)\
            == 0)

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
    sem_t tx_sem;

    /* Channel program. */
    struct __attribute__((__packed__)) {
        uint8_t gdmac_program[0];
        uint8_t unipro_eom[0];
        GDMAC_INSTR_DMAMOV(unipro_eom_value_addr);
        GDMAC_INSTR_DMAMOV(unipro_eom_reg_addr);
        uint8_t pad0[DMA_LD_SIZE + DMA_ST_SIZE + DMA_WMB_SIZE];
        GDMAC_INSTR_DMASEV(unipro_eom_tx_event);
        uint8_t pad10[DMA_END_SIZE];
        uint8_t unipro_program_entry_point[DMA_MOV_SIZE];
        GDMAC_INSTR_DMAMOV(unipro_src_addr);
        GDMAC_INSTR_DMAMOV(unipro_dst_addr);
        uint8_t unipro_continue_tx[0];
        GDMAC_INSTR_DMALP(unpro_len_loopcount);
        uint8_t pad20[DMA_LD_SIZE + DMA_ST_SIZE + DMA_LPEND_SIZE];
        GDMAC_INSTR_DMALP(unpro_eom_loopcount);
        uint8_t unipro_eom_loopback[DMA_LPEND_SIZE + DMA_WMB_SIZE];
        GDMAC_INSTR_DMASEV(unipro_end_of_tx_event);
        uint8_t pad100[DMA_END_SIZE];
    };
} tsb_dma_gdmac_unipro_tx_channel;

/* a fixed value write to UniPro EOM register after EOM packet was sent. */
static const uint8_t eom_value = 0x01;

static unsigned int gdmac_event_to_channel_map[TSB_DMA_GDMAC_EVENTS];

/* GDMAC device */
struct device *tsb_dma_gdmac_dev = NULL;

/* template for UniPro Tx GDMAC binary code. */
static const uint8_t tsb_dma_gdma_unipro_tx_prg[] = {
        DMAMOV(sar, 0),
        DMAMOV(dar, 0),
        DMALD,
        DMAST,
        DMAWMB,
        DMASEV(0),
        DMAEND,
        /* DMA execution starts here. Not the first instruction. */
        DMAMOV(ccr, 0x01C04701),
        DMAMOV(sar, 0),
        DMAMOV(dar, 0),
        DMALP(lc0, 0),
        DMALD,
        DMAST,
        DMALPEND(lc0, DMA_LD_SIZE + DMA_ST_SIZE),
        DMALP(lc0, GDMAC_DO_LOOPBACK),
        DMALPEND(lc0, DMA_SET_EOM_LOOPBACK_COUNT),
        DMAWMB,
        DMASEV(0),
        DMAEND
};

struct tsb_dma_channel_info {
    union {
        tsb_dma_gdmac_unipro_tx_channel unipro_tx_channel;
        /* other GDMAC channels will be added as more GDMAC channels are
         * supported.
         */
    };
};

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

static bool inline dma_start_thread(uint8_t thread_id, uint8_t* program_addr)
{
    uint8_t go_instr[] = { DMAGO(thread_id, ns, program_addr) };

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
            struct tsb_dma_info *info = device_get_private(tsb_dma_gdmac_dev);

            sem_post(
                    &info->dma_channel[channel].channel_info->unipro_tx_channel.tx_sem);
        }
    } else {
        lldbg("received invalid DMA interrupt(%d)\n", irq);
    }

    return OK;
}

void gdmac_unipro_tx_release_channel(struct tsb_dma_channel *channel)
{
    /* For Now that is all we need */
    if (channel->channel_info != NULL) {
        free(channel->channel_info);
        channel->channel_info = NULL;
        channel->do_dma_transfer = NULL;
        channel->release_channel = NULL;
    }
}

static int gdmac_unipro_tx_transfer(struct tsb_dma_channel *channel, void* buf,
        void *tx_buf, size_t len, device_dma_transfer_arg *arg)
{
    device_dma_unipro_tx_arg *unipro_tx_arg = (device_dma_unipro_tx_arg*) arg;
    tsb_dma_gdmac_unipro_tx_channel *unipro_tx_channel =
            &channel->channel_info->unipro_tx_channel;
    int status;

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
    } else {
        /* If this is NOT the last fragmented message, set loop count to 1 to
         * cause the GDMAC to raise the AP interrupt event and ends the
         * execution.
         */
        unipro_tx_channel->unpro_eom_loopcount.value = GDMAC_DONOT_LOOPBACK;
    }

    /* Start the transfer and wait for end of transfer interrupt. */
    status = dma_start_thread(channel->channel_id,
            &unipro_tx_channel->unipro_program_entry_point[0]);

    sem_wait(&unipro_tx_channel->tx_sem);

    return status;
}

/* Allocate an UniPro TX channel. */
int tsb_dma_allocal_unipro_tx_channel(struct tsb_dma_channel *channel)
{
    int ret;
    tsb_dma_gdmac_unipro_tx_channel *unipro_tx_channel;
    struct tsb_dma_channel_info *channel_info = zalloc(
            sizeof(struct tsb_dma_channel_info));
    volatile tsb_dma_gdmac_control_regs *control_regs =
            (tsb_dma_gdmac_control_regs*) GDMAC_CONTROL_REGS_ADDRESS;

    if (channel_info == NULL) {
        return -ENOMEM;
    }

    unipro_tx_channel = &channel_info->unipro_tx_channel;

    /* allocate an end of transfer event; */
    unipro_tx_channel->end_of_tx_event = gdmac_allocate_event(
            channel->channel_id);
    if (unipro_tx_channel->end_of_tx_event == TSB_DMA_GDMAC_INVALID_EVENT) {
        free(unipro_tx_channel);

        lldbg("Unable to allocate GDMAC event.\n");
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

    /* Set interrupt handler for this GDMAC UniPro TX channel */
    ret = irq_attach(DMA_CHANNEL_TO_TSB_IRQ(channel->channel_id),
            tsb_dma_irq_handler);
    if (ret != OK) {
        lldbg("Failed to attach interrupt %d.\n",
                DMA_CHANNEL_TO_TSB_IRQ(channel->channel_id));
        return -EIO;
    } else {
        sem_init(&unipro_tx_channel->tx_sem, 0, 0);
        putreg32(GDMAC_EVENT_MASK(channel->channel_id), &control_regs->inten);
        up_enable_irq(DMA_CHANNEL_TO_TSB_IRQ(channel->channel_id));
    }

    /* Set the transfer and transfer done handlers */
    channel->do_dma_transfer = gdmac_unipro_tx_transfer;
    channel->release_channel = gdmac_unipro_tx_release_channel;
    channel->channel_info = (struct tsb_dma_channel_info*) unipro_tx_channel;

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
