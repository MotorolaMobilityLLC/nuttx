/**
 * Copyright (c) 2016 Motorola Mobility, LLC
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
#include <arch/bitops.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/device.h>
#include <nuttx/device_dma.h>
#include <nuttx/bufram.h>

#include "debug.h"
#include "up_arch.h"

#include "tsb_scm.h"

#include "chip.h"
#include "tsb_dma_gdmac_asm.h"
#include "tsb_dma.h"

#define GDMAC_MAX_DESC              2

#define GDMAC_NUMBER_OF_CHANNELS    8
#define GDMAC_NUMBER_OF_EVENTS      32

#define GDMAC_CHANNEL_REGISTER_GAP  0x2C0

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
#define GDMAC_INSTR_DMAWFP(value_name)      \
    GDMAC_INSTRUCTION(1, uint8_t, value_name)
#define GDMAC_INSTR_DMASTP(value_name)      \
    GDMAC_INSTRUCTION(1, uint8_t, value_name)
#define GDMAC_INSTR_DMALDP(value_name)      \
    GDMAC_INSTRUCTION(1, uint8_t, value_name)

/* GDMAC register addresses */
#define GDMAC_CONTROL_REGS_OFFSET   0x0000
#define GDMAC_CHANNEL_REGS_OFFSET   0x0100
#define GDMAC_DBG_REGS_OFFSET       0x0D00
#define GDMAC_CONFIG_REGS_OFFSET    0x0E00
#define GDMAC_ID_REGS_OFFSET        0x0FE0

#define GDMAC_CONTROL_REGS_ADDRESS  \
    (gdmac_resource_info.reg_base + GDMAC_CONTROL_REGS_OFFSET)
#define GDMAC_CHANNEL_REGS_ADDRESS  \
    (gdmac_resource_info.reg_base + GDMAC_CHANNEL_REGS_OFFSET)
#define GDMAC_DBG_REGS_ADDRESS      \
    (gdmac_resource_info.reg_base + GDMAC_DBG_REGS_OFFSET)
#define GDMAC_CONFIG_REGS_ADDRESS   \
    (gdmac_resource_info.reg_base + GDMAC_CONFIG_REGS_OFFSET)
#define GDMAC_ID_REGS_ADDRESS       \
    (gdmac_resource_info.reg_base + GDMAC_ID_REGS_OFFSET)

/* Define PL330 events associated with each channel. */
#define GDMAC_EVENT_MASK(event)              (1 << event)
#define IRQN_TO_DMA_EVENT(irqn)              \
	((irqn - gdmac_resource_info.irq_base >= GDMAC_NUMBER_OF_EVENTS) ?\
     GDMAC_INVALID_EVENT : (irqn - gdmac_resource_info.irq_base))
#define GDMAC_EVENT_TO_DMA_CHANNEL(event_id) \
    &gdmac_event_to_chan_map[event_id]
#define GDMAC_EVENT_TO_IRQN(event_id)        \
    (event_id + gdmac_resource_info.irq_base)

#define CCR_FIXED_VALUE                 0x01c00700
#define DMA_LOADANDSTORE                (DMALD | (DMAST << 8))
#define DMA_NO_LOADANDSTORE             (DMANOP | (DMANOP << 8))
#define DMA_NO_PERIPHERALSTORE          (DMANOP | (DMANOP << 8))
#define DMA_NO_PERIPHERALLOAD           (DMANOP | (DMANOP << 8))
#define DMA_NO_WAITFORPERIPGERAL        (DMANOP | (DMANOP << 8))

#define CCR_INC_VALE(inc)               ((inc == DEVICE_DMA_INC_AUTO) ? 1 : 0)

#define CCR_ENDIAN_SWAP(params)         \
    (gsmac_bit_to_pos(params->swap) << 28)

#define CCR_SRC_CONFIG(burst_len, burst_size)   \
    ((burst_len - 1) << 4 | (burst_size - 1) << 1)

#define CCR_DST_CONFIG(burst_len, burst_size)   \
    ((burst_len - 1) << 18 | (burst_size - 1) << 15)


/* Structures for PL330 GDMAC registers */
struct tsb_dma_gdmac_control_regs {
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
};

struct tsb_dma_gdmac_channel_status_regs {
    uint32_t csr;
    uint32_t cpc;
};

struct tsb_dma_gdmac_channel_control_regs {
    uint32_t sar;
    uint32_t dar;
    uint32_t ccr;
    uint32_t lc0;
    uint32_t lc1;
};

struct tsb_dma_gdmac_channel_regs {
    struct tsb_dma_gdmac_channel_status_regs
        channel_status_regs[GDMAC_NUMBER_OF_CHANNELS];
    uint8_t pad[GDMAC_CHANNEL_REGISTER_GAP];
    struct tsb_dma_gdmac_channel_control_regs
        channel_control_regs[GDMAC_NUMBER_OF_CHANNELS];
};

struct tsb_dma_gdmac_dbg_regs {
    uint32_t dbg_status;
    uint32_t dbg_cmd;
    uint32_t dbg_inst_0;
    uint32_t dbg_inst_1;
};

struct tsb_dma_gdmac_config_regs {
    uint32_t cr0;
    uint32_t cr1;
    uint32_t cr2;
    uint32_t cr3;
    uint32_t cr4;
    uint32_t crd;
};

struct tsb_dma_gdmac_id_regs {
    uint32_t periph_id_0;
    uint32_t periph_id_1;
    uint32_t periph_id_2;
    uint32_t periph_id_3;
    uint32_t pcell_id_0;
    uint32_t pcell_id_1;
    uint32_t pcell_id_2;
    uint32_t pcell_id_3;
};

union peripheral_instr {
    struct __attribute__((__packed__)) {
        uint8_t op_code;
        uint8_t peripheral_id;
    } instr;
    uint16_t value;
};

/* structure for memory to memory transfer channel information. */
struct __attribute__ ((__packed__)) mem2mem_pl330_code {
    GDMAC_INSTR_DMAMOV(source_addr);
    GDMAC_INSTR_DMAMOV(dest_addr);
    GDMAC_INSTR_DMAMOV(chan_ctrl_reg);
    GDMAC_INSTR_DMALP(init_dst_align_count);
    uint16_t init_dst_align_load_and_store;
    uint8_t pad0[DMA_LPEND_SIZE];
    GDMAC_INSTR_DMAMOV(preburst_load_chan_ctrl_reg);
    uint8_t preburst_load;
    GDMAC_INSTR_DMAMOV(burst_block_chan_ctrl_reg);
    GDMAC_INSTR_DMALP(burst_block_count);
    uint16_t burst_block_load_and_store;
    uint8_t pad1[DMA_LPEND_SIZE];
    GDMAC_INSTR_DMAMOV(postburst_block_chan_ctrl_reg);
    uint16_t postburst_block_load_and_store;
    GDMAC_INSTR_DMAMOV(postburst_chan_ctrl_reg);
    GDMAC_INSTR_DMALP(postburst_load_count);
    uint8_t postburst_load;
    uint8_t pad2[DMA_LPEND_SIZE];
    uint8_t postburst_store;
    GDMAC_INSTR_DMAMOV(final_store_chan_ctrl_reg);
    GDMAC_INSTR_DMALP(final_store_count);
    uint8_t final_store;
    uint8_t pad3[DMA_LPEND_SIZE];
};

/* structure for memory to IO transfer channel information. */
struct __attribute__((__packed__)) mem2io_pl330_code {
        GDMAC_INSTR_DMAMOV(source_addr);
        GDMAC_INSTR_DMAMOV(dest_addr);
        GDMAC_INSTR_DMAMOV(preburst_load_chan_ctrl_reg);
        uint8_t preburst_load;
        GDMAC_INSTR_DMAMOV(burst_block_chan_ctrl_reg);
        GDMAC_INSTR_DMALP(burst_block_count);
        GDMAC_INSTR_DMALP(burst_block_size);
        uint16_t burst_block_wait;
        uint8_t burst_block_load;
        uint16_t burst_block_store;
        uint8_t pad2[DMA_LPEND_SIZE + DMA_LPEND_SIZE];
        GDMAC_INSTR_DMALP(postburst_block_count);
        uint16_t postburst_block_wait;
        uint8_t postburst_block_load;
        uint16_t postburst_block_store;
        uint8_t pad11[DMA_LPEND_SIZE];
        GDMAC_INSTR_DMAMOV(burst_chan_ctrl_reg);
        uint16_t burst_wait;
        uint8_t burst_load;
        uint16_t burst_store;
        GDMAC_INSTR_DMAMOV(postburst_chan_ctrl_reg);
        GDMAC_INSTR_DMALP(postburst_load_count);
        uint16_t postburst_wait;
        uint8_t postburst_load;
        uint8_t pad21[DMA_LPEND_SIZE];
        uint16_t postburst_store;
        GDMAC_INSTR_DMAMOV(final_store_chan_ctrl_reg);
        GDMAC_INSTR_DMALP(final_store_count);
        uint16_t final_store_wait;
        uint16_t final_store;
        uint8_t pad30[DMA_LPEND_SIZE];
};

/* structure for IO to memory transfer channel information. */
struct __attribute__((__packed__)) io2mem_pl330_code {
        GDMAC_INSTR_DMAMOV(source_addr);
        GDMAC_INSTR_DMAMOV(dest_addr);
        GDMAC_INSTR_DMAMOV(burst_block_chan_ctrl_reg);
        GDMAC_INSTR_DMALP(burst_block_count);
        GDMAC_INSTR_DMALP(burst_block_size);
        uint16_t burst_block_wait;
        uint16_t burst_block_load;
        uint8_t burst_block_store;
        uint8_t pad2[DMA_LPEND_SIZE + DMA_LPEND_SIZE];
        GDMAC_INSTR_DMALP(postburst_block_count);
        uint16_t postburst_block_wait;
        uint16_t postburst_block_load;
        uint8_t postburst_block_store;
        uint8_t pad11[DMA_LPEND_SIZE];
        GDMAC_INSTR_DMAMOV(burst_chan_ctrl_reg);
        uint16_t burst_wait;
        uint16_t burst_load;
        uint8_t burst_store;
        GDMAC_INSTR_DMAMOV(postburst_chan_ctrl_reg);
        GDMAC_INSTR_DMALP(postburst_load_count);
        uint16_t postburst_wait;
        uint16_t postburst_load;
        uint8_t pad21[DMA_LPEND_SIZE];
        uint8_t postburst_store;
        GDMAC_INSTR_DMAMOV(final_store_chan_ctrl_reg);
        GDMAC_INSTR_DMALP(final_store_count);
        uint8_t final_store;
        uint8_t pad30[DMA_LPEND_SIZE];
};

struct __attribute__((__packed__)) pl330_end_code {
    uint8_t pad0[DMA_WMB_SIZE];
    GDMAC_INSTR_DMASEV(end_of_tx_event);
    uint8_t pad1[DMA_END_SIZE];
};

struct mem_to_mem_chan {
    uint32_t burst_size;
    uint32_t burst_len;
    uint32_t ccr_transfer_size;
    uint32_t burst_block_size;
    uint32_t align_mask;
    uint32_t ccr_base_value;
    /* Channel program. */
    struct mem2mem_pl330_code pl330_code[0];
};

struct mem_to_io_chan {
    uint32_t burst_size;
    uint32_t burst_len;
    uint32_t ccr_transfer_size;
    uint32_t burst_block_size;
    uint32_t align_mask;
    uint32_t ccr_base_value;
    uint32_t perihperal_id;
    /* Channel program. */
    struct mem2io_pl330_code pl330_code[0];
};

struct io_to_mem_chan {
    uint32_t burst_size;
    uint32_t burst_len;
    uint32_t ccr_transfer_size;
    uint32_t burst_block_size;
    uint32_t align_mask;
    uint32_t ccr_base_value;
    uint32_t perihperal_id;
    /* Channel program. */
    struct io2mem_pl330_code pl330_code[0];
};

typedef int (*gdmac_transfer)(struct device *dev, struct tsb_dma_chan *chan,
        struct device_dma_op *op, enum device_dma_error *error);

typedef void (*gdmac_release_channel)(struct tsb_dma_chan *chan);

struct gdmac_chan {
    struct tsb_dma_chan tsb_chan;
    struct device *gdmac_dev;
    unsigned int end_of_tx_event;
    gdmac_transfer do_dma_transfer;
    gdmac_release_channel release_channel;

    uint32_t burst_size;
    uint32_t burst_len;
    uint32_t ccr_transfer_size;
    uint32_t burst_block_size;
    uint32_t align_mask;
    uint32_t ccr_base_value;

    union {
        struct mem_to_mem_chan mem2mem_chan;
        struct mem_to_io_chan mem2io_chan;
        struct io_to_mem_chan io2mem_chan;
    };
};

/* template for memory to memory GDMAC binary code. */
static const uint8_t gdmac_mem2mem_program[] = {
        DMAMOV(sar, 0),
        DMAMOV(dar, 0),
        DMAMOV(ccr, 0x01C04701),
        DMALP(lc0, 0),
        DMALD,
        DMAST,
        DMALPEND(lc0, DMA_LD_SIZE + DMA_ST_SIZE),
        DMAMOV(ccr, 0),
        DMALD,
        DMAMOV(ccr, 0),
        DMALP(lc0, 0),
        DMALD,
        DMAST,
        DMALPEND(lc0, DMA_LD_SIZE + DMA_ST_SIZE),
        DMAMOV(ccr, 0),
        DMALD,
        DMAST,
        DMAMOV(ccr, 0),
        DMALP(lc0, 0),
        DMALD,
        DMALPEND(lc0, DMA_LD_SIZE),
        DMAST,
        DMAMOV(ccr, 0),
        DMALP(lc0, 0),
        DMAST,
        DMALPEND(lc0, DMA_LD_SIZE),
    };

/* template for memory to io GDMAC binary code. */
static const uint8_t gdmac_mem2io_program[] = {
        DMAMOV(sar, 0),
        DMAMOV(dar, 0),
        DMAMOV(ccr, 0),
        DMALD,
        DMAMOV(ccr, 0),
        DMALP(lc0, 0),
        DMALP(lc1, 255),
        DMAWFP(0, P),
        DMALD,
        DMASTPB(0),
        DMALPEND(lc1, DMA_WFP_SIZE + DMA_LD_SIZE + DMA_STP_SIZE),
        DMALPEND(lc0, DMA_LP_SIZE + DMA_WFP_SIZE + DMA_LD_SIZE +
                      DMA_STP_SIZE + DMA_LPEND_SIZE),
        DMALP(lc1, 0),
        DMAWFP(0, P),
        DMALD,
        DMASTPB(0),
        DMALPEND(lc1, DMA_WFP_SIZE + DMA_LD_SIZE + DMA_STP_SIZE),
        DMAMOV(ccr, 0),
        DMAWFP(0, P),
        DMALD,
        DMASTPB(0),
        DMAMOV(ccr, 0),
        DMALP(lc0, 0),
        DMAWFP(0, P),
        DMALD,
        DMALPEND(lc0, DMA_LD_SIZE),
        DMASTPB(0),
        DMAMOV(ccr, 0),
        DMALP(lc0, 0),
        DMAWFP(0, P),
        DMASTPB(0),
        DMALPEND(lc0, DMA_LD_SIZE),
    };

/* template for io to memory GDMAC binary code. */
static const uint8_t gdmac_io2mem_program[] = {
        DMAMOV(sar, 0),
        DMAMOV(dar, 0),
        DMAMOV(ccr, 0),
        DMALP(lc0, 0),
        DMALP(lc1, 255),
        DMAWFP(0, P),
        DMALDPB(0),
        DMAST,
        DMALPEND(lc1, DMA_WFP_SIZE + DMA_LD_SIZE + DMA_STP_SIZE),
        DMALPEND(lc0, DMA_LP_SIZE + DMA_WFP_SIZE + DMA_LD_SIZE +
                      DMA_STP_SIZE + DMA_LPEND_SIZE),
        DMALP(lc1, 0),
        DMAWFP(0, P),
        DMALDPB(0),
        DMAST,
        DMALPEND(lc1, DMA_WFP_SIZE + DMA_LD_SIZE + DMA_STP_SIZE),
        DMAMOV(ccr, 0),
        DMAWFP(0, P),
        DMALDPB(0),
        DMAST,
        DMAMOV(ccr, 0),
        DMALP(lc0, 0),
        DMAWFP(0, P),
        DMALDPB(0),
        DMALPEND(lc0, DMA_LD_SIZE),
        DMAST,
        DMAMOV(ccr, 0),
        DMALP(lc0, 0),
        DMAST,
        DMALPEND(lc0, DMA_LD_SIZE),
    };

static const uint8_t gdmac_pl330_end_code[] = {
        DMAWMB,
        DMASEV(0),
        DMAEND
    };

struct gdmac_event {
    struct gdmac_chan *dma_chan;
    int event;
};

static struct tsb_gdmac_drv_info {
    uint32_t    reg_base;
    uint32_t    irq_base;
} gdmac_resource_info;

static struct gdmac_event gdmac_event_to_chan_map[GDMAC_NUMBER_OF_EVENTS];

static inline uint32_t gsmac_bit_to_pos(uint32_t value)
{
    if (value == 0) {
        lldbg("Error: value shouldn't be %x\n", value);
        return 0;
    }

    return ffs(value)-1;
}

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
    struct tsb_dma_gdmac_dbg_regs *dbg_regs =
            (struct tsb_dma_gdmac_dbg_regs*) GDMAC_DBG_REGS_ADDRESS;
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
    struct tsb_dma_gdmac_control_regs *control_regs =
            (struct tsb_dma_gdmac_control_regs*) GDMAC_CONTROL_REGS_ADDRESS;
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

    struct mem_to_mem_chan *mem2mem_chan = &gdmac_chan->mem2mem_chan;
    uint32_t desc_index;
    struct mem2mem_pl330_code *pl330_code;

    int retval = OK;

    if (op->sg_count > GDMAC_MAX_DESC) {
        lldbg("Error: too many descriptors.");
        return -EINVAL;
    }

    pl330_code = &mem2mem_chan->pl330_code[GDMAC_MAX_DESC - op->sg_count];
    for (desc_index = 0; desc_index < op->sg_count; desc_index++) {
        uint32_t load_len;
        uint32_t store_len;
        uint32_t align_mask = mem2mem_chan->align_mask;
        uint32_t total_load_len;
        uint32_t total_store_len;
        uint32_t count;
        struct device_dma_sg *sg = &op->sg[desc_index];
        uint32_t src_addr = (uint32_t) sg->src_addr;
        uint32_t dst_addr = (uint32_t) sg->dst_addr;
        size_t data_len = sg->len;

        /* Set source and destination addresses, as well as the data count. */
        pl330_code->source_addr.value = (uint32_t) sg->src_addr;
        pl330_code->dest_addr.value = (uint32_t) sg->dst_addr;

        /* We need to copy the first few bytes to make ensure the destination
         * address is aligned before we do multi-bytes store, otherwise the
         * memory location before destination address might end up with some
         * undefine values. It appears unaligned store only works on the bus
         * width of the device.
         */
        store_len = (uint32_t)sg->dst_addr & align_mask;
        if (store_len != 0) {
            store_len = align_mask - store_len + 1;
            store_len = MIN(store_len, data_len);

            pl330_code->init_dst_align_count.value = (uint8_t)(store_len - 1);
            pl330_code->init_dst_align_load_and_store = DMA_LOADANDSTORE;

            src_addr += store_len;
            dst_addr += store_len;
            data_len -= store_len;

            store_len = 0;
        } else {
            pl330_code->init_dst_align_count.value = 0;
            pl330_code->init_dst_align_load_and_store = DMA_NO_LOADANDSTORE;
        }

        load_len = (uint32_t)src_addr & align_mask;

        /* Now, we are ready to use multi-bytes and burst load and store. We
         * need to check if we need to do a preload before we get into
         * bursting blocks of memory.
         */
        total_load_len = data_len;
        total_store_len = data_len;

        /* Adjust the length of source by taking the unalignement data count
         * so we don't need to have messy calculation later on.
         */
        if (((src_addr + data_len) & align_mask) < data_len) {
            total_load_len += (src_addr & align_mask);
            if (load_len > store_len) {
                pl330_code->preburst_load = DMALD;
                total_load_len -= mem2mem_chan->burst_size;
            } else {
                pl330_code->preburst_load = DMANOP;
            }
        }

        /* Adjust the length of destination by taking the unalignement data
         * data so we don't need to have messy calculation later on.
         */
        if (((dst_addr + data_len) & align_mask) < data_len) {
            total_store_len += (dst_addr & align_mask);
        }

        /* Do blocks of complete burst of data load and store */
        count = total_load_len / mem2mem_chan->burst_block_size;
        if (count > 0) {
            pl330_code->burst_block_count.value = count - 1;
            pl330_code->burst_block_load_and_store = DMA_LOADANDSTORE;
            load_len = mem2mem_chan->burst_block_size * count;
            total_load_len -= load_len;
            total_store_len -= load_len;
        } else {
            pl330_code->burst_block_count.value = 0;
            pl330_code->burst_block_load_and_store = DMA_NO_LOADANDSTORE;
        }

        /* Do the last burst of data load and store with the burst length less
         * than the channel's burst length.
         */
        count = total_load_len / mem2mem_chan->burst_size;
        if (count > 0) {
            pl330_code->postburst_block_chan_ctrl_reg.value =
                    mem2mem_chan->ccr_base_value |
                    CCR_DST_CONFIG(count, mem2mem_chan->ccr_transfer_size) |
                    CCR_SRC_CONFIG(count, mem2mem_chan->ccr_transfer_size);
            pl330_code->postburst_block_load_and_store = DMA_LOADANDSTORE;
            load_len = mem2mem_chan->burst_size * count;
            load_len = count << 3;
            total_load_len -= load_len;
            total_store_len -= load_len;
        } else {
            pl330_code->postburst_block_chan_ctrl_reg.value =
                    mem2mem_chan->ccr_base_value |
                    CCR_DST_CONFIG(1, mem2mem_chan->ccr_transfer_size) |
                    CCR_SRC_CONFIG(1, mem2mem_chan->ccr_transfer_size);
            pl330_code->postburst_block_load_and_store = DMA_NO_LOADANDSTORE;
        }

        /* If we still have some data left unread, finish loading them */
        if (total_load_len > 0) {
            pl330_code->postburst_load_count.value =
                    (uint8_t)(total_load_len - 1);
            pl330_code->postburst_load = DMALD;
            total_load_len = 0;
        } else {
            pl330_code->postburst_load_count.value = 0;
            pl330_code->postburst_load = DMANOP;
        }

        /* If the there is enouph data to do a multi-bytes transfer, perform a
         * multi-byte transfer.
         */
        if (total_store_len >= mem2mem_chan->burst_size) {
            pl330_code->postburst_store = DMAST;
            total_store_len -= mem2mem_chan->burst_size;
        } else {
            pl330_code->postburst_store = DMANOP;
        }

        /* If we still have some data left in the MFIFO, flush them out */
        if (total_store_len > 0) {
            pl330_code->final_store_count.value =
                    (uint8_t)(total_store_len - 1);
            pl330_code->final_store = DMAST;
            total_store_len -= total_store_len;
        } else {
            pl330_code->final_store_count.value = 0;
            pl330_code->final_store = DMANOP;
        }

        pl330_code++;
    }

    pl330_code = &mem2mem_chan->pl330_code[GDMAC_MAX_DESC - op->sg_count];
    /* Start the transfer and wait for end of transfer interrupt. */
    if (dma_start_thread(tsb_chan->chan_id, (uint8_t *)pl330_code)
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
    struct mem_to_mem_chan *mem2mem_chan;
    struct gdmac_chan *gdmac_chan;
    struct tsb_dma_gdmac_control_regs *control_regs =
            (struct tsb_dma_gdmac_control_regs*) GDMAC_CONTROL_REGS_ADDRESS;
    uint32_t desc_index;
    struct mem2mem_pl330_code *pl330_code;
    uint32_t burst_size;
    uint32_t burst_len;
    uint32_t ccr_transfer_size;

    gdmac_chan = bufram_alloc(sizeof(struct gdmac_chan) +
            GDMAC_MAX_DESC * sizeof(struct mem2mem_pl330_code) +
            sizeof(struct pl330_end_code));
    if (gdmac_chan == NULL) {
        return -ENOMEM;
    }

    /* allocate an end of transfer event; */
    retval = gdmac_allocate_event(gdmac_chan,
            DEVICE_DMA_CALLBACK_EVENT_COMPLETE, &gdmac_chan->end_of_tx_event);
    if (retval != OK) {
        bufram_free(gdmac_chan);
        lldbg("Unable to allocate GDMAC event(s).\n");
        return retval;
    }

    mem2mem_chan = &gdmac_chan->mem2mem_chan;
    mem2mem_chan->burst_size = 0;
    mem2mem_chan->burst_len = 0;
    mem2mem_chan->ccr_transfer_size = 0;
    mem2mem_chan->burst_block_size = 0;
    mem2mem_chan->align_mask = 0;
    mem2mem_chan->ccr_base_value = 0;

    pl330_code = &mem2mem_chan->pl330_code[0];
    /* make a copy of the UniPro TX binary code. */
    for (desc_index = 0; desc_index < GDMAC_MAX_DESC; desc_index++) {
        memcpy(&pl330_code[desc_index], &gdmac_mem2mem_program[0],
                sizeof(gdmac_mem2mem_program));
    }
    memcpy(&pl330_code[desc_index],
           &gdmac_pl330_end_code[0], sizeof(gdmac_pl330_end_code));

    burst_size = mem2mem_chan->burst_size = params->transfer_size;
    burst_len = mem2mem_chan->burst_len =
            gsmac_bit_to_pos(params->burst_len) + 1;
    ccr_transfer_size =
            mem2mem_chan->ccr_transfer_size = gsmac_bit_to_pos(burst_len);
    mem2mem_chan->burst_block_size =
            mem2mem_chan->burst_size * mem2mem_chan->burst_len;
    mem2mem_chan->align_mask = burst_size - 1;
    mem2mem_chan->ccr_base_value =
            CCR_FIXED_VALUE | CCR_ENDIAN_SWAP(params) |
            CCR_INC_VALE(params->dst_inc_options) << 14 |
            CCR_INC_VALE(params->src_inc_options);

    for (desc_index = 0; desc_index < GDMAC_MAX_DESC; desc_index++) {
        /* set Channel Control registers */
        pl330_code->preburst_load_chan_ctrl_reg.value =
                mem2mem_chan->ccr_base_value |
                CCR_DST_CONFIG(1, ccr_transfer_size) |
                CCR_SRC_CONFIG(1, ccr_transfer_size);

        pl330_code->burst_block_chan_ctrl_reg.value =
                mem2mem_chan->ccr_base_value |
                CCR_DST_CONFIG(burst_len, ccr_transfer_size) |
                CCR_SRC_CONFIG(burst_len, ccr_transfer_size);

        pl330_code->postburst_chan_ctrl_reg.value =
                mem2mem_chan->ccr_base_value |
               CCR_DST_CONFIG(1, ccr_transfer_size) | CCR_SRC_CONFIG(1, 1);

        pl330_code->final_store_chan_ctrl_reg.value =
               mem2mem_chan->ccr_base_value |
               CCR_DST_CONFIG(1, 1) | CCR_SRC_CONFIG(1, 1);

        pl330_code++;
    }
    /* Set end of transfer event. */
    ((struct pl330_end_code *)pl330_code)->end_of_tx_event.value |=
        (gdmac_chan->end_of_tx_event << 3);

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

void gdmac_mem2io_release_channel(struct tsb_dma_chan *tsb_chan)
{
    struct gdmac_chan *gdmac_chan =
            containerof(tsb_chan, struct gdmac_chan, tsb_chan);

    gdmac_release_event(gdmac_chan->end_of_tx_event);

    free(gdmac_chan);
}

static int gdmac_mem2io_transfer(struct device *dev,
        struct tsb_dma_chan *tsb_chan, struct device_dma_op *op,
        enum device_dma_error *error)
{
    struct gdmac_chan
     *gdmac_chan = containerof(tsb_chan, struct gdmac_chan, tsb_chan);

    struct mem_to_io_chan *mem2io_chan = &gdmac_chan->mem2io_chan;
     int retval = OK;
     uint32_t desc_index;
     struct mem2io_pl330_code *pl330_code;
     uint32_t interface_id = mem2io_chan->perihperal_id;
     union peripheral_instr store_instr = { .instr = {DMASTPB(interface_id)} };
     union peripheral_instr wait_for_peripheral_instr =
                          { .instr = {DMAWFP(interface_id, B)} };

     if (op->sg_count != 1) {
         lldbg("Error: sg_count != 1.");
         return -EINVAL;
     }

     pl330_code = &mem2io_chan->pl330_code[GDMAC_MAX_DESC - op->sg_count];
     for (desc_index = 0; desc_index < op->sg_count; desc_index++) {
         uint32_t load_len;
         uint32_t store_len;
         uint32_t align_mask = mem2io_chan->align_mask;
         uint32_t total_load_len;
         uint32_t total_store_len;
         uint32_t count;
         struct device_dma_sg *sg = &op->sg[desc_index];
         uint32_t src_addr = (uint32_t) sg->src_addr;
         uint32_t dst_addr = (uint32_t) sg->dst_addr;
         size_t data_len = sg->len;

         /* Set source and destination addresses, as well as the data count. */
         pl330_code->source_addr.value = (uint32_t) sg->src_addr;
         pl330_code->dest_addr.value = (uint32_t) sg->dst_addr;

         /* We need to copy the first few bytes to make ensure the destination
          * address is aligned before we do multi-bytes store, otherwise the
          * memory location before destination address might end up with some
          * undefine values. It appears unaligned store only works on the bus
          * width of the device.
          */
         store_len = (uint32_t)sg->dst_addr & align_mask;
         if (store_len != 0) {
             lldbg("Error: Destination address must be aligned.\n");
             return -EINVAL;
         }

         load_len = (uint32_t)src_addr & align_mask;

         /* Now, we are ready to use multi-bytes and burst load and store. We
          * need to check if we need to do a preload before we get into
          * bursting blocks of memory.
          */
         total_load_len = data_len;
         total_store_len = data_len;

         /* Adjust the length of source by taking the unalignement data count
          * so we don't need to have messy calculation later on.
          */
         if (((src_addr + data_len) & align_mask) < data_len) {
             total_load_len += (src_addr & align_mask);
             if (load_len > store_len) {
                 pl330_code->preburst_load = DMALD;
                 total_load_len -= mem2io_chan->burst_size;
             } else {
                 pl330_code->preburst_load = DMANOP;
             }
         }

         /* Adjust the length of destination by taking the unalignement data
          * data so we don't need to have messy calculation later on.
          */
         if (((dst_addr + data_len) & align_mask) < data_len) {
             total_store_len += (dst_addr & align_mask);
         }

         /* Do blocks of complete burst of data load and store */
         count = total_load_len / mem2io_chan->burst_block_size;
         if (count > 0) {
             if (count > 256) {
                 pl330_code->burst_block_count.value = (count / 256)- 1;
                 pl330_code->burst_block_size.value = 255;
                 pl330_code->burst_block_wait =
                         wait_for_peripheral_instr.value;
                 pl330_code->burst_block_load = DMALD;
                 pl330_code->burst_block_store = store_instr.value;

                 /* Each loop through this part of the dma code will do 256 iterations
                  * of burst_block_size.  This equates to
                  * (burst_block_size.value+1)*(burst_block_count.value+1) or to
                  * keep the math constant 256*(burst_block_count.value+1).
                  */
                 load_len = mem2io_chan->burst_block_size * 256 * (pl330_code->burst_block_count.value+1);
                 total_load_len -= load_len;
                 total_store_len -= load_len;
             } else {
                 pl330_code->burst_block_count.value = 0;
                 pl330_code->burst_block_size.value = 0;
                 pl330_code->burst_block_wait = DMA_NO_WAITFORPERIPGERAL;
                 pl330_code->burst_block_load = DMANOP;
                 pl330_code->burst_block_store = DMA_NO_PERIPHERALSTORE;
             }
             /* Copy the bursts which could not be done as a multipe of 256. */
             count = total_load_len / mem2io_chan->burst_block_size;
             if (count > 0 ) {
                 pl330_code->postburst_block_count.value = count - 1;
                 pl330_code->postburst_block_wait =
                         wait_for_peripheral_instr.value;
                 pl330_code->postburst_block_load = DMALD;
                 pl330_code->postburst_block_store = store_instr.value;

                 load_len = mem2io_chan->burst_block_size * count;
                 total_load_len -= load_len;
                 total_store_len -= load_len;

             } else {
                 pl330_code->postburst_block_count.value = 0;
                 pl330_code->postburst_block_wait = DMA_NO_WAITFORPERIPGERAL;
                 pl330_code->postburst_block_load = DMANOP;
                 pl330_code->postburst_block_store = DMA_NO_PERIPHERALSTORE;
             }
         } else {
             pl330_code->burst_block_count.value = 0;
             pl330_code->burst_block_size.value = 0;
             pl330_code->burst_block_wait = DMA_NO_WAITFORPERIPGERAL;
             pl330_code->burst_block_load = DMANOP;
             pl330_code->burst_block_store = DMA_NO_PERIPHERALSTORE;

             pl330_code->postburst_block_count.value = 0;
             pl330_code->postburst_block_wait = DMA_NO_WAITFORPERIPGERAL;
             pl330_code->postburst_block_load = DMANOP;
             pl330_code->postburst_block_store = DMA_NO_PERIPHERALSTORE;
         }

         /* Do the last chunk of data load and store with the burst length less
          * than the channel's burst length.
          */
         count = total_load_len / mem2io_chan->burst_size;
         if (count > 0) {
             pl330_code->burst_chan_ctrl_reg.value =
                     mem2io_chan->ccr_base_value |
                     CCR_DST_CONFIG(count, mem2io_chan->ccr_transfer_size) |
                     CCR_SRC_CONFIG(count, mem2io_chan->ccr_transfer_size);
             pl330_code->burst_wait = wait_for_peripheral_instr.value;
             pl330_code->burst_load = DMALD;
             pl330_code->burst_store = store_instr.value;
             load_len = mem2io_chan->burst_size * count;
             load_len = count << 3;
             total_load_len -= load_len;
             total_store_len -= load_len;
         } else {
             pl330_code->burst_chan_ctrl_reg.value =
                     mem2io_chan->ccr_base_value |
                     CCR_DST_CONFIG(1, mem2io_chan->ccr_transfer_size) |
                     CCR_SRC_CONFIG(1, mem2io_chan->ccr_transfer_size);
             pl330_code->burst_wait = DMA_NO_WAITFORPERIPGERAL;
             pl330_code->burst_load = DMANOP;
             pl330_code->burst_store = DMA_NO_PERIPHERALSTORE;
         }

         /* If we still have some data left unread, finish loading them */
         if (total_load_len > 0) {
             pl330_code->postburst_load_count.value =
                     (uint8_t)(total_load_len - 1);
             pl330_code->postburst_load = DMALD;
             total_load_len = 0;
         } else {
             pl330_code->postburst_load_count.value = 0;
             pl330_code->postburst_load = DMANOP;
         }

         /* If the there is enouph data to do a multi-bytes transfer, perform a
          * multi-byte transfer.
          */
         if (total_store_len >= mem2io_chan->burst_size) {
             pl330_code->postburst_wait = wait_for_peripheral_instr.value;
             pl330_code->postburst_store = store_instr.value;
             total_store_len -= mem2io_chan->burst_size;
         } else {
             pl330_code->postburst_wait = DMA_NO_WAITFORPERIPGERAL;
             pl330_code->postburst_store = DMA_NO_PERIPHERALSTORE;
         }

         /* If we still have some data left in the MFIFO, flush them out */
         if (total_store_len > 0) {
             pl330_code->final_store_count.value =
                     (uint8_t)(total_store_len - 1);
             pl330_code->final_store_wait = wait_for_peripheral_instr.value;
             pl330_code->final_store = store_instr.value;
             total_store_len -= total_store_len;
         } else {
             pl330_code->final_store_count.value = 0;
             pl330_code->final_store_wait = DMA_NO_WAITFORPERIPGERAL;
             pl330_code->final_store = DMA_NO_PERIPHERALSTORE;
         }

         pl330_code++;
     }

     pl330_code = &mem2io_chan->pl330_code[GDMAC_MAX_DESC - op->sg_count];
     /* Start the transfer and wait for end of transfer interrupt. */
     if (dma_start_thread(tsb_chan->chan_id, (uint8_t *)pl330_code)
             == false) {
         retval = -EIO;
     }

     return retval;
}

/* Allocate a memory to IO channel. */
int tsb_gdmac_allocal_mem2io_chan(struct device *dev,
        struct device_dma_params *params, struct tsb_dma_chan **tsb_chan)
{
    int retval = OK;
    struct mem_to_io_chan *mem2io_chan;
    struct gdmac_chan *gdmac_chan;
    struct tsb_dma_gdmac_control_regs *control_regs =
            (struct tsb_dma_gdmac_control_regs*) GDMAC_CONTROL_REGS_ADDRESS;
    uint32_t desc_index;
    struct mem2io_pl330_code *pl330_code;
    uint32_t burst_size;
    uint32_t burst_len;
    uint32_t ccr_transfer_size;

    gdmac_chan = bufram_alloc(sizeof(struct gdmac_chan) +
            GDMAC_MAX_DESC * sizeof(struct mem2io_pl330_code) +
            sizeof(struct pl330_end_code));
    if (gdmac_chan == NULL) {
        return -ENOMEM;
    }

    /* allocate an end of transfer event; */
    retval = gdmac_allocate_event(gdmac_chan,
            DEVICE_DMA_CALLBACK_EVENT_COMPLETE, &gdmac_chan->end_of_tx_event);
    if (retval != OK) {
        bufram_free(gdmac_chan);
        lldbg("Unable to allocate GDMAC event(s).\n");
        return retval;
    }

    mem2io_chan = &gdmac_chan->mem2io_chan;
    mem2io_chan->burst_size = 0;
    mem2io_chan->burst_len = 0;
    mem2io_chan->ccr_transfer_size = 0;
    mem2io_chan->burst_block_size = 0;
    mem2io_chan->align_mask = 0;
    mem2io_chan->ccr_base_value = 0;

    pl330_code = &mem2io_chan->pl330_code[0];

    /* make a copy of the UniPro TX binary code. */
    for (desc_index = 0; desc_index < GDMAC_MAX_DESC; desc_index++) {
        memcpy(&pl330_code[desc_index], &gdmac_mem2io_program[0],
                sizeof(gdmac_mem2io_program));
    }
    memcpy(&pl330_code[desc_index],
           &gdmac_pl330_end_code[0], sizeof(gdmac_pl330_end_code));

    /* Set end of transfer event. */
    burst_size = mem2io_chan->burst_size = params->transfer_size;
    burst_len = mem2io_chan->burst_len =
            gsmac_bit_to_pos(params->burst_len) + 1;
    ccr_transfer_size = mem2io_chan->ccr_transfer_size =
            gsmac_bit_to_pos(burst_size) + 1;
    mem2io_chan->burst_block_size =
            mem2io_chan->burst_size * mem2io_chan->burst_len;
    mem2io_chan->align_mask = burst_size - 1;
    mem2io_chan->ccr_base_value =
            CCR_FIXED_VALUE | CCR_ENDIAN_SWAP(params) |
            CCR_INC_VALE(params->dst_inc_options) << 14 |
            CCR_INC_VALE(params->src_inc_options);

    mem2io_chan->perihperal_id = params->dst_devid;

    for (desc_index = 0; desc_index < GDMAC_MAX_DESC; desc_index++) {
        /* set Channel Control registers */
        pl330_code->preburst_load_chan_ctrl_reg.value =
                mem2io_chan->ccr_base_value |
                CCR_DST_CONFIG(1, ccr_transfer_size) |
                CCR_SRC_CONFIG(1, ccr_transfer_size);

        pl330_code->burst_block_chan_ctrl_reg.value =
                mem2io_chan->ccr_base_value |
                CCR_DST_CONFIG(burst_len, ccr_transfer_size) |
                CCR_SRC_CONFIG(burst_len, ccr_transfer_size);

        pl330_code->postburst_chan_ctrl_reg.value =
                mem2io_chan->ccr_base_value |
               CCR_DST_CONFIG(1, ccr_transfer_size) | CCR_SRC_CONFIG(1, 1);

        pl330_code->final_store_chan_ctrl_reg.value =
               mem2io_chan->ccr_base_value |
               CCR_DST_CONFIG(1, 1) | CCR_SRC_CONFIG(1, 1);

        pl330_code++;
    }
    /* Set end of transfer event. */
    ((struct pl330_end_code *)pl330_code)->end_of_tx_event.value |=
        (gdmac_chan->end_of_tx_event << 3);

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
    gdmac_chan->do_dma_transfer = gdmac_mem2io_transfer;
    gdmac_chan->release_channel = gdmac_mem2io_release_channel;
    gdmac_chan->gdmac_dev = dev;

    *tsb_chan = &gdmac_chan->tsb_chan;

    return retval;
}

void gdmac_io2mem_release_channel(struct tsb_dma_chan *tsb_chan)
{
    struct gdmac_chan *gdmac_chan =
            containerof(tsb_chan, struct gdmac_chan, tsb_chan);

    gdmac_release_event(gdmac_chan->end_of_tx_event);

    free(gdmac_chan);
}

static int gdmac_io2mem_transfer(struct device *dev,
        struct tsb_dma_chan *tsb_chan, struct device_dma_op *op,
        enum device_dma_error *error)
{
    struct gdmac_chan
     *gdmac_chan = containerof(tsb_chan, struct gdmac_chan, tsb_chan);

     struct io_to_mem_chan *io2mem_chan = &gdmac_chan->io2mem_chan;
     int retval = OK;
     struct io2mem_pl330_code *pl330_code;
     uint32_t desc_index;
     uint32_t interface_id = io2mem_chan->perihperal_id;
     union peripheral_instr load_instr = { .instr = {DMALDPB(interface_id)} };
     union peripheral_instr wait_for_peripheral_instr =
                          { .instr = {DMAWFP(interface_id, P)} };

     /* Set the source and destination addresses, as well as the data count. */
     pl330_code = &io2mem_chan->pl330_code[GDMAC_MAX_DESC - op->sg_count];
     for (desc_index = 0; desc_index < op->sg_count; desc_index++) {
         uint32_t load_len;
         uint32_t align_mask = io2mem_chan->align_mask;
         uint32_t total_load_len;
         uint32_t total_store_len;
         uint32_t count;
         struct device_dma_sg *sg = &op->sg[desc_index];
         uint32_t src_addr = (uint32_t) sg->src_addr;
         uint32_t dst_addr = (uint32_t) sg->dst_addr;
         size_t data_len = sg->len;

         /* Set source and destination addresses, as well as the data count. */
         pl330_code->source_addr.value = src_addr;
         pl330_code->dest_addr.value = dst_addr;

         /* We need to copy the first few bytes to make ensure the destination
          * address is aligned before we do multi-bytes store, otherwise the
          * memory location before destination address might end up with some
          * undefine values. It appears unaligned store only works on the bus
          * width of the device.
          */
         load_len = (uint32_t)sg->src_addr & align_mask;
         if (load_len != 0) {
             lldbg("Error: Destination address must be aligned.\n");
             return -EINVAL;
         }

         /* Now, we are ready to use multi-bytes and burst load and store. We
          * need to check if we need to do a preload before we get into
          * bursting blocks of memory.
          */
         total_load_len = data_len;
         total_store_len = data_len;

         /* Adjust the length of destination by taking the unalignement data
          * data so we don't need to have messy calculation later on.
          */
         if (((dst_addr + data_len) & align_mask) < data_len) {
             total_store_len += (dst_addr & align_mask);
         }

         /* Do blocks of complete burst of data load and store */
         count = total_load_len / io2mem_chan->burst_block_size;
         if (count > 0) {
             if (count > 256) {
                 pl330_code->burst_block_count.value = (count / 256)- 1;
                 pl330_code->burst_block_size.value = 255;
                 pl330_code->burst_block_wait =
                         wait_for_peripheral_instr.value;
                 pl330_code->burst_block_load = load_instr.value;
                 pl330_code->burst_block_store = DMAST;

                 /* Each loop through this part of the dma code will do 256 iterations
                  * of burst_block_size.  This equates to
                  * (burst_block_size.value+1)*(burst_block_count.value+1) or to
                  * keep the math constant 256*(burst_block_count.value+1).
                  */
                 load_len = io2mem_chan->burst_block_size * 256 * (pl330_code->burst_block_count.value + 1);
                 total_load_len -= load_len;
                 total_store_len -= load_len;
             } else {
                 pl330_code->burst_block_count.value = 0;
                 pl330_code->burst_block_size.value = 0;
                 pl330_code->burst_block_wait = DMA_NO_WAITFORPERIPGERAL;
                 pl330_code->burst_block_load = DMA_NO_PERIPHERALLOAD;
                 pl330_code->burst_block_store = DMANOP;
             }
             /* Copy the bursts which could not be done as a multipe of 256. */
             count = total_load_len / io2mem_chan->burst_block_size;
             if (count > 0 ) {
                 pl330_code->postburst_block_count.value = count - 1;
                 pl330_code->postburst_block_wait =
                         wait_for_peripheral_instr.value;
                 pl330_code->postburst_block_load = load_instr.value;
                 pl330_code->postburst_block_store = DMAST;

                 load_len = io2mem_chan->burst_block_size * count;
                 total_load_len -= load_len;
                 total_store_len -= load_len;

             } else {
                 pl330_code->postburst_block_count.value = 0;
                 pl330_code->postburst_block_wait = DMA_NO_WAITFORPERIPGERAL;
                 pl330_code->postburst_block_load = DMA_NO_PERIPHERALLOAD;
                 pl330_code->postburst_block_store = DMANOP;
             }
         } else {
             pl330_code->burst_block_count.value = 0;
             pl330_code->burst_block_size.value = 0;
             pl330_code->burst_block_wait = DMA_NO_WAITFORPERIPGERAL;
             pl330_code->burst_block_load = DMA_NO_PERIPHERALLOAD;
             pl330_code->burst_block_store = DMANOP;

             pl330_code->postburst_block_count.value = 0;
             pl330_code->postburst_block_wait = DMA_NO_WAITFORPERIPGERAL;
             pl330_code->postburst_block_load = DMA_NO_PERIPHERALLOAD;
             pl330_code->postburst_block_store = DMANOP;
         }

         /* Do the last chunk of data load and store with the burst length less
          * than the channel's burst length.
          */
         count = total_load_len / io2mem_chan->burst_size;
         if (count > 0) {
             pl330_code->burst_chan_ctrl_reg.value =
                     io2mem_chan->ccr_base_value |
                     CCR_DST_CONFIG(count, io2mem_chan->ccr_transfer_size) |
                     CCR_SRC_CONFIG(count, io2mem_chan->ccr_transfer_size);
             pl330_code->burst_wait = wait_for_peripheral_instr.value;
             pl330_code->burst_load = load_instr.value;
             pl330_code->burst_store = DMAST;
             load_len = io2mem_chan->burst_size * count;
             load_len = count << 3;
             total_load_len -= load_len;
             total_store_len -= load_len;
         } else {
             pl330_code->burst_chan_ctrl_reg.value =
                     io2mem_chan->ccr_base_value |
                     CCR_DST_CONFIG(1, io2mem_chan->ccr_transfer_size) |
                     CCR_SRC_CONFIG(1, io2mem_chan->ccr_transfer_size);
             pl330_code->burst_wait = DMA_NO_WAITFORPERIPGERAL;
             pl330_code->burst_load = DMA_NO_PERIPHERALLOAD;
             pl330_code->burst_store = DMANOP;
         }

         /* If we still have some data left unread, finish loading them */
         if (total_load_len > 0) {
             pl330_code->postburst_load_count.value =
                     (uint8_t)(total_load_len - 1);
             pl330_code->postburst_load = load_instr.value;
             total_load_len = 0;
         } else {
             pl330_code->postburst_load_count.value = 0;
             pl330_code->postburst_load = DMA_NO_PERIPHERALLOAD;
         }

         /* If the there is enouph data to do a multi-bytes transfer, perform a
          * multi-byte transfer.
          */
         if (total_store_len >= io2mem_chan->burst_size) {
             pl330_code->postburst_wait = wait_for_peripheral_instr.value;
             pl330_code->postburst_store = DMAST;
             total_store_len -= io2mem_chan->burst_size;
         } else {
             pl330_code->postburst_wait = DMA_NO_WAITFORPERIPGERAL;
             pl330_code->postburst_store = DMANOP;
         }

         /* If we still have some data left in the MFIFO, flush them out */
         if (total_store_len > 0) {
             pl330_code->final_store_count.value =
                     (uint8_t)(total_store_len - 1);
             pl330_code->final_store = DMAST;
             total_store_len -= total_store_len;
         } else {
             pl330_code->final_store_count.value = 0;
             pl330_code->final_store = DMANOP;
         }

         pl330_code++;
     }

     pl330_code = &io2mem_chan->pl330_code[GDMAC_MAX_DESC - op->sg_count];

     /* Start the transfer and wait for end of transfer interrupt. */
     if (dma_start_thread(tsb_chan->chan_id, (uint8_t *)pl330_code)
             == false) {
         retval = -EIO;
     }

     return retval;
}

/* Allocate a memory to IO channel. */
int tsb_gdmac_allocal_io2mem_chan(struct device *dev,
        struct device_dma_params *params, struct tsb_dma_chan **tsb_chan)
{
    int retval = OK;
    struct io_to_mem_chan *io2mem_chan;
    struct gdmac_chan *gdmac_chan;
    struct tsb_dma_gdmac_control_regs *control_regs =
            (struct tsb_dma_gdmac_control_regs*) GDMAC_CONTROL_REGS_ADDRESS;
    struct io2mem_pl330_code *pl330_code;
    uint32_t desc_index;
    uint32_t burst_size;
    uint32_t burst_len;
    uint32_t ccr_transfer_size;

    gdmac_chan = bufram_alloc(sizeof(struct gdmac_chan) +
            GDMAC_MAX_DESC * sizeof(struct io2mem_pl330_code) +
            sizeof(struct pl330_end_code));
    if (gdmac_chan == NULL) {
        return -ENOMEM;
    }

    /* allocate an end of transfer event; */
    retval = gdmac_allocate_event(gdmac_chan,
            DEVICE_DMA_CALLBACK_EVENT_COMPLETE, &gdmac_chan->end_of_tx_event);
    if (retval != OK) {
        bufram_free(gdmac_chan);
        lldbg("Unable to allocate GDMAC event(s).\n");
        return retval;
    }

    io2mem_chan = &gdmac_chan->io2mem_chan;
    io2mem_chan->burst_size = 0;
    io2mem_chan->burst_len = 0;
    io2mem_chan->ccr_transfer_size = 0;
    io2mem_chan->burst_block_size = 0;
    io2mem_chan->align_mask = 0;
    io2mem_chan->ccr_base_value = 0;

    pl330_code = &io2mem_chan->pl330_code[0];

    /* make a copy of the UniPro TX binary code. */
    for (desc_index = 0; desc_index < GDMAC_MAX_DESC; desc_index++) {
        memcpy(&pl330_code[desc_index], &gdmac_io2mem_program[0],
                sizeof(gdmac_io2mem_program));
    }
    memcpy(&pl330_code[desc_index],
           &gdmac_pl330_end_code[0], sizeof(gdmac_pl330_end_code));

    /* Set end of transfer event. */
    burst_size = io2mem_chan->burst_size = params->transfer_size;
    burst_len = io2mem_chan->burst_len =
            gsmac_bit_to_pos(params->burst_len) + 1;
    ccr_transfer_size = io2mem_chan->ccr_transfer_size =
            gsmac_bit_to_pos(burst_size) + 1;
    io2mem_chan->burst_block_size =
            io2mem_chan->burst_size * io2mem_chan->burst_len;
    io2mem_chan->align_mask = burst_size - 1;
    io2mem_chan->ccr_base_value =
            CCR_FIXED_VALUE | CCR_ENDIAN_SWAP(params) |
            CCR_INC_VALE(params->dst_inc_options) << 14 |
            CCR_INC_VALE(params->src_inc_options);

    io2mem_chan->perihperal_id = params->src_devid;

    for (desc_index = 0; desc_index < GDMAC_MAX_DESC; desc_index++) {
        /* set Channel Control registers */
        pl330_code->burst_block_chan_ctrl_reg.value =
                io2mem_chan->ccr_base_value |
                CCR_DST_CONFIG(burst_len, ccr_transfer_size) |
                CCR_SRC_CONFIG(burst_len, ccr_transfer_size);

        pl330_code->postburst_chan_ctrl_reg.value =
                io2mem_chan->ccr_base_value |
               CCR_DST_CONFIG(1, ccr_transfer_size) | CCR_SRC_CONFIG(1, 1);

        pl330_code->final_store_chan_ctrl_reg.value =
                io2mem_chan->ccr_base_value |
               CCR_DST_CONFIG(1, 1) | CCR_SRC_CONFIG(1, 1);
        pl330_code++;
    }
    /* Set end of transfer event. */
    ((struct pl330_end_code *)pl330_code)->end_of_tx_event.value |=
        (gdmac_chan->end_of_tx_event << 3);

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
    gdmac_chan->do_dma_transfer = gdmac_io2mem_transfer;
    gdmac_chan->release_channel = gdmac_io2mem_release_channel;
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

        return tsb_gdmac_allocal_mem2Mem_chan(dev, params, tsb_chan);
    }

    if ((params->src_dev == DEVICE_DMA_DEV_MEM)
            && (params->dst_dev == DEVICE_DMA_DEV_IO)) {
        if ((params->src_devid != 0) || (params->dst_devid >= 32)) {
            return -EINVAL;
        }
        return tsb_gdmac_allocal_mem2io_chan(dev, params, tsb_chan);
    }

    if ((params->src_dev == DEVICE_DMA_DEV_IO)
            && (params->dst_dev == DEVICE_DMA_DEV_MEM)) {
        if ((params->dst_devid != 0) || (params->src_devid >= 32)) {
            return -EINVAL;
        }

        return tsb_gdmac_allocal_io2mem_chan(dev, params, tsb_chan);
    }

    if ((params->src_dev == DEVICE_DMA_DEV_MEM)
            && (params->dst_dev == DEVICE_DMA_DEV_IO)) {
        if ((params->src_devid != 0) || (params->dst_devid >= 32)) {
            return -EINVAL;
        }

        return tsb_gdmac_allocal_mem2io_chan(dev, params, tsb_chan);
    }

    lldbg("user requested not supported DMA channel.\n");

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

static void gdmac_kill_chan(uint32_t chan)
{
    struct tsb_dma_gdmac_dbg_regs *dbg_regs =
            (struct tsb_dma_gdmac_dbg_regs*) GDMAC_DBG_REGS_ADDRESS;
    uint32_t value;

    value = (DMAKILL << 16) | chan << 8 | 0x01;
    putreg32(value, &dbg_regs->dbg_inst_0);
    putreg32(0, &dbg_regs->dbg_inst_1);

    /* Get going */
    putreg32(0, &dbg_regs->dbg_cmd);
}

void gdmac_abort_chan(struct device *dev, struct tsb_dma_chan *tsb_chan)
{
    gdmac_kill_chan(tsb_chan->chan_id);
    tsb_dma_callback(dev, tsb_chan, DEVICE_DMA_CALLBACK_EVENT_COMPLETE);
}

int gdmac_irq_abort_handler(int irq, void *context)
{
    struct tsb_dma_gdmac_control_regs *control_regs =
            (struct tsb_dma_gdmac_control_regs*) GDMAC_CONTROL_REGS_ADDRESS;
    uint32_t fsrc = getreg32(&control_regs->fsrc);
    uint32_t chan, index;

    if (fsrc == 0) {
        lldbg("Unexpected FSRC value %x\n", fsrc);
        return OK;
    }

    chan = gsmac_bit_to_pos(fsrc) & 0x07;
    gdmac_kill_chan(chan);

    for (index = 0; index < GDMAC_NUMBER_OF_EVENTS; index++) {
        struct gdmac_chan *gdmac_chan;

        gdmac_chan = gdmac_event_to_chan_map[index].dma_chan;
        if ((gdmac_chan != NULL) &&
            (gdmac_chan->tsb_chan.chan_id == chan)) {
            tsb_dma_callback(gdmac_chan->gdmac_dev, &gdmac_chan->tsb_chan,
                    DEVICE_DMA_ERROR_DMA_FAILED);

            break;
        }
    }

    return OK;
}

static int tsb_gdmac_extract_resources(struct device *dev,
                                       struct tsb_gdmac_drv_info * info)
{
    struct device_resource *r;

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_REGS, "reg_base");
    if (!r) {
        return -EINVAL;
    }
    info->reg_base = r->start;

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_IRQ, "irq_gdmac");
    if (!r) {
        return -EINVAL;
    }
    info->irq_base = r->start;

    return OK;
}

int gdmac_init_controller(struct device *dev)
{
    int retval = OK;

    /* initialize all events are available. */
    memset(&gdmac_event_to_chan_map[0], 0, sizeof(gdmac_event_to_chan_map));

    retval = tsb_gdmac_extract_resources(dev, &gdmac_resource_info);
    if (retval != OK) {
        lldbg("Failed to retrieve GDMAC resource.\n");
        return retval;
    }

    /* enable clock to GDMAC and reset GDMAC. */
    tsb_clk_enable(TSB_CLK_GDMA);
    tsb_reset(TSB_RST_GDMA);

    retval = irq_attach(TSB_IRQ_GDMACABORT, gdmac_irq_abort_handler);
    if (retval != OK) {
        lldbg("Failed to attach GDMAC Abort interrupt.\n");
        return retval;
    } else {
        up_enable_irq(TSB_IRQ_GDMACABORT);
    }

    return OK;
}

void gdmac_deinit_controller(struct device *dev)
{
    int retval = OK;

    tsb_clk_disable(TSB_CLK_GDMA);

    up_disable_irq(TSB_IRQ_GDMACABORT);

    retval = irq_attach(TSB_IRQ_GDMACABORT, NULL);
    if (retval != OK) {
        lldbg("Failed to detach GDMAC Abort interrupt.\n");
    }

    return;
}

int gdmac_max_number_of_channels(void)
{
    int numb_of_chan = 0;
    struct tsb_dma_gdmac_config_regs *config_regs =
            (struct tsb_dma_gdmac_config_regs*) GDMAC_CONFIG_REGS_ADDRESS;

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
    struct tsb_dma_gdmac_control_regs *control_regs =
            (struct tsb_dma_gdmac_control_regs*) GDMAC_CONTROL_REGS_ADDRESS;
    struct gdmac_chan *gdmac_chan =
            containerof(tsb_chan, struct gdmac_chan, tsb_chan);
    uint32_t enable_flags = getreg32(&control_regs->inten);

    putreg32(~GDMAC_EVENT_MASK(gdmac_chan->end_of_tx_event) & enable_flags,
            &control_regs->inten);
    up_disable_irq(GDMAC_EVENT_TO_IRQN(gdmac_chan->end_of_tx_event));

    bufram_free(tsb_chan);

    return OK;
}
