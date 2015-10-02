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

#define GDMAC_NUMBER_OF_CHANNELS    8
#define GDMAC_NUMBER_OF_EVENTS      32

#define GDMAC_INVALID_EVENT         0xFFFFFFFF
#define GDMAC_INVALID_CHANNEL       NULL

#define GDMAC_DO_LOOPBACK           (1)
#define GDMAC_DONOT_LOOPBACK        (0)

#define GDMAC_INSTRUCTION(offset, type, name)	\
	    struct __attribute__((__packed__))      \
	    { uint8_t instr[offset]; type value; } name

#define GDMAC_INSTR_DMAMOV(value_name)      \
    GDMAC_INSTRUCTION(2, uint32_t, value_name)
#define GDMAC_INSTR_DMASEV(value_name)      \
    GDMAC_INSTRUCTION(1, uint8_t, value_name)
#define GDMAC_INSTR_DMALP(value_name)       \
    GDMAC_INSTRUCTION(1, uint8_t, value_name)
#define GDMAC_INSTR_DMAWFE(value_name)      \
    GDMAC_INSTRUCTION(1, uint8_t, value_name)
#define GDMAC_INSTR_DMALPEND(value_name)    \
    GDMAC_INSTRUCTION(1, uint8_t, value_name)

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
#define GDMAC_EVENT_MASK(event)              (1 << event)
#define IRQN_TO_DMA_EVENT(irqn)              \
	((irqn - TSB_IRQ_GDMAC00 >= GDMAC_NUMBER_OF_EVENTS) ?\
     GDMAC_INVALID_EVENT : (irqn - TSB_IRQ_GDMAC00))
#define GDMAC_EVENT_TO_DMA_CHANNEL(event_id) \
    &gdmac_event_to_chan_map[event_id]
#define GDMAC_EVENT_TO_IRQN(event_id)        \
    (event_id + TSB_IRQ_GDMAC00)

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
    uint32_t ftr[GDMAC_NUMBER_OF_CHANNELS];
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
        channel_status_regs[GDMAC_NUMBER_OF_CHANNELS];
    tsb_dma_gdmac_channel_control_regs
        channel_control_regs[GDMAC_NUMBER_OF_CHANNELS];
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

/* structure for memory to memory transfer channel information. */
typedef struct {
    /* Channel program. */
    struct __attribute__((__packed__)) {
        uint8_t gdmac_program[0];
        GDMAC_INSTR_DMAMOV(chan_ctrl_reg);
        GDMAC_INSTR_DMAMOV(source_addr);
        GDMAC_INSTR_DMAMOV(dest_addr);
        GDMAC_INSTR_DMALP(block_count);
        GDMAC_INSTR_DMALP(block_length);
        uint16_t block_load_and_store;
        uint8_t pad0[DMA_LPEND_SIZE + DMA_LPEND_SIZE];
        GDMAC_INSTR_DMALP(data_length);
        uint8_t pad1[DMA_LD_SIZE + DMA_ST_SIZE + DMA_LPEND_SIZE +
                     DMA_WMB_SIZE];
        GDMAC_INSTR_DMASEV(end_of_tx_event);
        uint8_t pad2[DMA_END_SIZE];
    };
} mem_to_mem_chan;

typedef int (*gdmac_transfer)(struct device *dev, struct tsb_dma_chan *chan,
        struct device_dma_op *op, enum device_dma_error *error);

typedef void (*gdmac_release_channel)(struct tsb_dma_chan *chan);

struct gdmac_chan {
    struct tsb_dma_chan tsb_chan;
    struct device *gdmac_dev;
    unsigned int end_of_tx_event;
    gdmac_transfer do_dma_transfer;
    gdmac_release_channel release_channel;

    union {
        mem_to_mem_chan mem2mem_chan;
    /* other GDMAC channels will be added as more GDMAC channels are supported
     */
    };
};

/* template for memory to memory GDMAC binary code. */
static const uint8_t gdmac_mem2mem_program[] = {
        DMAMOV(ccr, 0x01C04701),
        DMAMOV(sar, 0),
        DMAMOV(dar, 0),
        DMALP(lc0, 0),
        DMALP(lc1, 0),
        DMALD,
        DMAST,
        DMALPEND(lc1, DMA_LD_SIZE + DMA_ST_SIZE),
        DMALPEND(lc0,
                DMA_LP_SIZE + DMA_LD_SIZE + DMA_ST_SIZE + DMA_LPEND_SIZE),
        DMALP(lc0, 0),
        DMALD,
        DMAST,
        DMALPEND(lc0, DMA_LD_SIZE + DMA_ST_SIZE),
        DMAWMB,
        DMASEV(0),
    };

struct gdmac_event {
    struct gdmac_chan *dma_chan;
    int event;
};

static struct gdmac_event gdmac_event_to_chan_map[GDMAC_NUMBER_OF_EVENTS];

static const uint16_t skip_mem_load_and_store = (DMANOP | (DMANOP << 8));
static const uint16_t do_mem_load_and_store = (DMALD | (DMAST << 8));

static inline int gdmac_allocate_event(struct gdmac_chan *chan, int event,
        unsigned int *event_id)
{
    int index;

    for (index = 0; index < GDMAC_NUMBER_OF_EVENTS; index++) {
        if (gdmac_event_to_chan_map[index].dma_chan == NULL) {
            gdmac_event_to_chan_map[index].dma_chan = chan;
            gdmac_event_to_chan_map[index].event = event;

            *event_id = index;
            return OK;
        }
    }

    return -ENOMEM;
}

static inline void gdmac_release_event(unsigned int event_id)
{
    if (event_id < GDMAC_NUMBER_OF_EVENTS) {
        gdmac_event_to_chan_map[event_id].dma_chan = NULL;
        gdmac_event_to_chan_map[event_id].event = 0;
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

int gdmac_irq_handler(int irq, void *context)
{
    tsb_dma_gdmac_control_regs *control_regs =
            (tsb_dma_gdmac_control_regs*) GDMAC_CONTROL_REGS_ADDRESS;
    unsigned int event_id = IRQN_TO_DMA_EVENT(irq);
    struct gdmac_event *event_entry;
    (void) context;

    if (event_id == GDMAC_INVALID_EVENT) {
        lldbg("Error: invalid irq %d\n", irq);
        return OK;
    }

    putreg32(GDMAC_EVENT_MASK(event_id), &control_regs->intclr);

    event_entry = GDMAC_EVENT_TO_DMA_CHANNEL(event_id);
    if (event_entry != NULL) {
        struct gdmac_chan *gdmac_chan = event_entry->dma_chan;

        tsb_dma_callback(gdmac_chan->gdmac_dev, &gdmac_chan->tsb_chan,
                event_entry->event);
    } else {
        lldbg("received invalid DMA interrupt(%d)\n", irq);
    }

    return OK;
}

void gdmac_mem2mem_release_channel(struct tsb_dma_chan *tsb_chan)
{
    struct gdmac_chan *gdmac_chan =
            containerof(tsb_chan, struct gdmac_chan, tsb_chan);

    gdmac_release_event(gdmac_chan->end_of_tx_event);

    free(gdmac_chan);
}

static int gdmac_mem2mem_transfer(struct device *dev,
        struct tsb_dma_chan *tsb_chan, struct device_dma_op *op,
        enum device_dma_error *error)
{
    struct gdmac_chan
    *gdmac_chan = containerof(tsb_chan, struct gdmac_chan, tsb_chan);

    mem_to_mem_chan *mem2mem_chan = &gdmac_chan->mem2mem_chan;
    struct device_dma_sg *sg = &op->sg[0];
    int retval = OK;
    size_t data_len = sg->len;
    uint32_t numb_of_blocks = (data_len + 255) / 256;

    if (op->sg_count != 1) {
        lldbg("Error: sg_count != 1.");
        return -EINVAL;
    }

    /* Set the source and destination addresses, as well as the data count. */
    mem2mem_chan->source_addr.value = (uint32_t) sg->src_addr;
    mem2mem_chan->dest_addr.value = (uint32_t) sg->dst_addr;
    if (numb_of_blocks-- > 1) {
        if (numb_of_blocks > 256) {
            lldbg("Error: Data len too large!");
            return -EINVAL;
        }

        mem2mem_chan->block_count.value = (uint8_t) (numb_of_blocks - 1);
        mem2mem_chan->block_length.value = 255;
        mem2mem_chan->block_load_and_store = do_mem_load_and_store;
        data_len -= numb_of_blocks * 256;
    } else {
        mem2mem_chan->block_count.value = 0;
        mem2mem_chan->block_length.value = 0;
        mem2mem_chan->block_load_and_store = skip_mem_load_and_store;
    }

    if (data_len <= 256) {
        mem2mem_chan->data_length.value = (uint8_t) data_len - 1;
    }

    /* Start the transfer and wait for end of transfer interrupt. */
    if (dma_start_thread(tsb_chan->chan_id, &mem2mem_chan->gdmac_program[0])
            == false) {
        retval = -EIO;
    }

    return retval;
}

/* Allocate an UniPro TX channel. */
int tsb_gdmac_allocal_mem2Mem_chan(struct device *dev,
        struct device_dma_params *params, struct tsb_dma_chan **tsb_chan)
{
    int retval = OK;
    mem_to_mem_chan *mem2mem_chan;
    struct gdmac_chan *gdmac_chan;
    tsb_dma_gdmac_control_regs *control_regs =
            (tsb_dma_gdmac_control_regs*) GDMAC_CONTROL_REGS_ADDRESS;

    gdmac_chan = zalloc(sizeof(struct gdmac_chan));
    if (gdmac_chan == NULL) {
        return -ENOMEM;
    }

    /* allocate an end of transfer event; */
    retval = gdmac_allocate_event(gdmac_chan,
            DEVICE_DMA_CALLBACK_EVENT_COMPLETE, &gdmac_chan->end_of_tx_event);
    if (retval != OK) {
        free(gdmac_chan);
        lldbg("Unable to allocate GDMAC event(s).\n");
        return retval;
    }

    mem2mem_chan = &gdmac_chan->mem2mem_chan;

    /* make a copy of the UniPro TX binary code. */
    memcpy(&mem2mem_chan->gdmac_program[0], &gdmac_mem2mem_program[0],
            sizeof(gdmac_mem2mem_program));

    /* Set end of transfer event. */
    mem2mem_chan->end_of_tx_event.value |= (gdmac_chan->end_of_tx_event << 3);

    /* Set interrupt handler for this GDMAC UniPro TX channel. */
    retval = irq_attach(GDMAC_EVENT_TO_IRQN(gdmac_chan->end_of_tx_event),
            gdmac_irq_handler);
    if (retval != OK) {
        lldbg("Failed to attach interrupt %d.\n",
                GDMAC_EVENT_TO_IRQN(gdmac_chan->end_of_tx_event));
        return -EIO;
    } else {
        uint32_t enable_flags = getreg32(&control_regs->inten);

        putreg32(GDMAC_EVENT_MASK(gdmac_chan->end_of_tx_event) | enable_flags,
                &control_regs->inten);
        up_enable_irq(GDMAC_EVENT_TO_IRQN(gdmac_chan->end_of_tx_event));
    }

    /* Set the transfer and transfer done handlers */
    gdmac_chan->do_dma_transfer = gdmac_mem2mem_transfer;
    gdmac_chan->release_channel = gdmac_mem2mem_release_channel;
    gdmac_chan->gdmac_dev = dev;

    *tsb_chan = &gdmac_chan->tsb_chan;

    return retval;
}

int gdmac_get_caps(struct device *dev, struct device_dma_caps *caps)
{
    caps->addr_alignment = DEVICE_DMA_ALIGNMENT_8 | DEVICE_DMA_ALIGNMENT_16
            | DEVICE_DMA_ALIGNMENT_32 | DEVICE_DMA_ALIGNMENT_64;
    caps->transfer_sizes = DEVICE_DMA_TRANSFER_SIZE_8
            | DEVICE_DMA_TRANSFER_SIZE_16 | DEVICE_DMA_TRANSFER_SIZE_32
            | DEVICE_DMA_TRANSFER_SIZE_64;
    caps->burst_len = 0xFFFF;
    caps->swap_options = DEVICE_DMA_SWAP_SIZE_NONE | DEVICE_DMA_SWAP_SIZE_16
            | DEVICE_DMA_SWAP_SIZE_32 | DEVICE_DMA_SWAP_SIZE_64
            | DEVICE_DMA_SWAP_SIZE_128;
    caps->inc_options = DEVICE_DMA_INC_AUTO | DEVICE_DMA_INC_NOAUTO;
    caps->sg_max = TSB_DMA_SG_MAX;

    return 0;
}

int gdmac_chan_alloc(struct device *dev, struct device_dma_params *params,
        struct tsb_dma_chan **tsb_chan)
{
    int retval = OK;

    if ((params->src_dev == DEVICE_DMA_DEV_MEM)
            && (params->dst_dev == DEVICE_DMA_DEV_MEM)) {
        if ((params->src_devid != 0) || (params->dst_devid != 0)) {
            return -EINVAL;
        }

        retval = tsb_gdmac_allocal_mem2Mem_chan(dev, params, tsb_chan);
    } else {
        lldbg("user requested not supported DMA channel.\n");
    }

    return retval;
}

int gdmac_start_op(struct device *dev, struct tsb_dma_chan *tsb_chan,
        struct device_dma_op *op, enum device_dma_error *error)
{
    int retval = OK;
    struct gdmac_chan *gdmac_chan =
            containerof(tsb_chan, struct gdmac_chan, tsb_chan);

    if (gdmac_chan->do_dma_transfer != NULL) {
        retval = gdmac_chan->do_dma_transfer(dev, tsb_chan, op, error);
    } else {
        retval = -EIO;
    }

    return retval;
}

void gdmac_init_controller(struct device *dev)
{
    /* initialize all events are available. */
    memset(&gdmac_event_to_chan_map[0], 0, sizeof(gdmac_event_to_chan_map));

    /* enable clock to GDMAC and reset GDMAC. */
    tsb_clk_enable(TSB_CLK_GDMA);
    tsb_reset(TSB_RST_GDMA);
}

void gdmac_deinit_controller(struct device *dev)
{
    tsb_clk_disable(TSB_CLK_GDMA);

    return;
}

int gdmac_max_number_of_channels(void)
{
    int numb_of_chan = 0;
    tsb_dma_gdmac_config_regs *config_regs =
            (tsb_dma_gdmac_config_regs*) GDMAC_CONFIG_REGS_ADDRESS;

    numb_of_chan = (getreg32(&config_regs->cr0) & 0x70) >> 4;
    numb_of_chan++;

    return numb_of_chan;
}

int gdmac_chan_check_op_params(struct device *dev,
        struct tsb_dma_chan *tsb_chan, struct device_dma_op *op)
{
    struct device_dma_params *chan_params = &tsb_chan->chan_params;
    int retval = DEVICE_DMA_ERROR_NONE;
    uint32_t index;

    if (chan_params->swap != DEVICE_DMA_SWAP_SIZE_NONE) {
        uint32_t aligment_mask = chan_params->swap - 1;

        for (index = 0; index < op->sg_count; index++) {
            struct device_dma_sg *desc = &op->sg[index];

            if (((desc->src_addr & aligment_mask) != 0) ||
                ((desc->dst_addr & aligment_mask) != 0)) {
                return DEVICE_DMA_ERROR_BAD_ADDR;
            }

            if (desc->len & aligment_mask) {
                return DEVICE_DMA_ERROR_BAD_LEN;
            }

            if (chan_params->src_inc_options == DEVICE_DMA_INC_NOAUTO) {
                if (chan_params->transfer_size < chan_params->swap) {
                    return DEVICE_DMA_ERROR_BAD_TX_SIZE;
                }
            }

            if (chan_params->dst_inc_options == DEVICE_DMA_INC_NOAUTO) {
                if (chan_params->transfer_size < chan_params->swap) {
                    return DEVICE_DMA_ERROR_BAD_TX_SIZE;
                }
            } else {
                uint32_t total_burst_len =
                        chan_params->transfer_size * chan_params->burst_len;

                if ((total_burst_len & aligment_mask) != 0) {
                    return DEVICE_DMA_ERROR_BAD_BURST_LEN;
                }
            }
        }
    }

    return retval;
}

int gdmac_chan_free(struct device *dev, struct tsb_dma_chan *tsb_chan)
{
    tsb_dma_gdmac_control_regs *control_regs =
            (tsb_dma_gdmac_control_regs*) GDMAC_CONTROL_REGS_ADDRESS;
    struct gdmac_chan *gdmac_chan =
            containerof(tsb_chan, struct gdmac_chan, tsb_chan);
    uint32_t enable_flags = getreg32(&control_regs->inten);

    putreg32(~GDMAC_EVENT_MASK(gdmac_chan->end_of_tx_event) & enable_flags,
            &control_regs->inten);
    up_disable_irq(GDMAC_EVENT_TO_IRQN(gdmac_chan->end_of_tx_event));

    free(tsb_chan);

    return OK;
}
