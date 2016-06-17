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
 * @brief MIPI UniPro stack for Bridges
 */

#include <string.h>

#include <nuttx/util.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/list.h>

#include <nuttx/unipro/unipro.h>
#include <nuttx/greybus/tsb_unipro.h>

#include <arch/tsb/irq.h>
#include <errno.h>

#include "debug.h"
#include "up_arch.h"
#include "tsb_scm.h"
#include "tsb_unipro.h"
#include "tsb_unipro_es2.h"
#include "tsb_es2_mphy_fixups.h"

#define UNIPRO_LUP_DONE     BIT(0)

#if defined(CONFIG_UNIPRO_P2P)
#include <arch/chip/unipro_p2p.h>
#endif

#define TRANSFER_MODE_2_CTRL_0 (0xAAAAAAAA) // Transfer mode 2 for CPorts 0-15
/*
 * CPorts 16-43 are present on the AP Bridge only.  CPorts 16 and 17 are
 * reserved for camera and display use, and we leave their transfer mode
 * at the power-on default of Mode 1.
 */
#define TRANSFER_MODE_2_CTRL_1 (0xAAAAAAA5) // Transfer mode 2 for CPorts 18-31
#define TRANSFER_MODE_2_CTRL_2 (0x00AAAAAA) // Transfer mode 2 for CPorts 32-43

#define CPORT_SW_RESET_BITS 3

#define CPORT_TX_BUF_BASE         (0x50000000U)
#define CPORT_TX_BUF(cport)       (uint8_t*)(CPORT_TX_BUF_BASE + \
                                      (CPORT_TX_BUF_SIZE * cport))

#define CPORTID_CDSI0    (16)
#define CPORTID_CDSI1    (17)

#define CPB_TX_BUFFER_SPACE_MASK        ((uint32_t) 0x0000007F)
#define CPB_TX_BUFFER_SPACE_OFFSET_MASK CPB_TX_BUFFER_SPACE_MASK

static struct cport *cporttable;
static unipro_event_handler_t evt_handler;

#if defined(CONFIG_TSB_CHIP_REV_ES2)
#define APBRIDGE_CPORT_MAX 44 // number of CPorts available on the APBridges
#define GPBRIDGE_CPORT_MAX 16 // number of CPorts available on the GPBridges
#else
#define APBRIDGE_CPORT_MAX 40 // number of CPorts available on the APBridges
#define GPBRIDGE_CPORT_MAX 16 // number of CPorts available on the GPBridges
#endif

/*
 * During unipro_unit(), we'll compute and cache the number of CPorts that this
 * bridge has, for use by the functions in this source file.
 */
static unsigned int cport_count = 0;

unsigned int unipro_cport_count(void) {
#ifdef CONFIG_MHB
    return CPORTID_CDSI1 + 1;
#else
    /*
     * Reduce the run-time CPort count to what's available on the
     * GPBridges, unless we can determine that we're running on an
     * APBridge.
     */
    return ((tsb_get_product_id() == tsb_pid_apbridge) ?
            APBRIDGE_CPORT_MAX : GPBRIDGE_CPORT_MAX);
#endif
}

struct cport *cport_handle(unsigned int cportid) {
    if (cportid >= cport_count || cportid == CPORTID_CDSI0 ||
        cportid == CPORTID_CDSI1) {
        return NULL;
    } else {
        return &cporttable[cportid];
    }
}

#define irqn_to_cport(irqn)          cport_handle((irqn - TSB_IRQ_UNIPRO_RX_EOM00))
#define cportid_to_irqn(cportid)     (TSB_IRQ_UNIPRO_RX_EOM00 + cportid)

/* Helpers */
static uint32_t cport_get_status(struct cport*);
static inline void clear_rx_interrupt(struct cport*);
static inline void enable_rx_interrupt(struct cport*);
static void configure_transfer_mode(int);
#ifdef UNIPRO_DEBUG
static void dump_regs(void);
#endif

/* irq handlers */
static int irq_rx_eom(int, void*);
static int irq_unipro(int, void*);

static uint32_t unipro_read(uint32_t offset) {
    return getreg32((volatile unsigned int*)(AIO_UNIPRO_BASE + offset));
}

static void unipro_write(uint32_t offset, uint32_t v) {
    putreg32(v, (volatile unsigned int*)(AIO_UNIPRO_BASE + offset));
}



/**
 * @brief Read CPort status from unipro controller
 * @param cport cport to retrieve status for
 * @return 0: CPort is configured and usable
 *         1: CPort is not connected
 *         2: reserved
 *         3: CPort is connected to a TC which is not present in the peer
 *          Unipro node
 */
static uint32_t cport_get_status(struct cport *cport) {
    uint32_t val;
    unsigned int cportid = cport->cportid;
    uint32_t reg;

    reg = CPORT_STATUS_0 + ((cportid/16) << 2);
    val = unipro_read(reg);
    val >>= ((cportid % 16) << 1);
    return (val & (0x3));
}

/**
 * @brief Clear and disable UniPro interrupt
 */
static void clear_int(unsigned int cportid) {
    unsigned int i;
    uint32_t int_en;

    i = cportid * 2;
    if (cportid < 16) {
        unipro_write(AHM_RX_EOM_INT_BEF_0, 0x3 << i);
        int_en = unipro_read(AHM_RX_EOM_INT_EN_0);
        int_en &= ~(0x3 << i);
        unipro_write(AHM_RX_EOM_INT_EN_0, int_en);
    } else if (cportid < 32) {
        unipro_write(AHM_RX_EOM_INT_BEF_1, 0x3 << i);
        int_en = unipro_read(AHM_RX_EOM_INT_EN_1);
        int_en &= ~(0x3 << i);
        unipro_write(AHM_RX_EOM_INT_EN_1, int_en);
    } else {
        unipro_write(AHM_RX_EOM_INT_BEF_2, 0x3 << i);
        int_en = unipro_read(AHM_RX_EOM_INT_EN_2);
        int_en &= ~(0x3 << i);
        unipro_write(AHM_RX_EOM_INT_EN_2, int_en);
    }
    tsb_irq_clear_pending(cportid_to_irqn(cportid));
}

/**
 * @brief Enable EOM interrupt on cport
 */
static void enable_int(unsigned int cportid) {
    struct cport *cport;
    unsigned int irqn;

    cport = cport_handle(cportid);
    if (!cport) {
        return;
    }

    irqn = cportid_to_irqn(cportid);
    enable_rx_interrupt(cport);
    irq_attach(irqn, irq_rx_eom);
    up_enable_irq(irqn);
}

/**
 * @brief Enable a CPort that has a connected connection.
 */
static int configure_connected_cport(unsigned int cportid) {
    int ret = 0;
    struct cport *cport;
    unsigned int rc;
    irqstate_t flags;

    cport = cport_handle(cportid);
    if (!cport) {
        return -EINVAL;
    }
    rc = cport_get_status(cport);
    switch (rc) {
    case CPORT_STATUS_CONNECTED:
        cport->connected = 1;

        /*
         * Clear any pending EOM interrupts, then enable them.
         */
        flags = irqsave();
        clear_int(cportid);
        enable_int(cportid);
        irqrestore(flags);

        /* Start the flow of received data */
        unipro_unpause_rx(cportid);
        break;
    case CPORT_STATUS_UNCONNECTED:
        ret = -ENOTCONN;
        break;
    default:
        lldbg("Unexpected status: CP%u: status: 0x%u\n", cportid, rc);
        ret = -EIO;
    }
    return ret;
}

/**
 * @brief Enable E2EFC on cport
 */
static void enable_e2efc(unsigned int cportid) {
    uint32_t e2efc;

    DBG_UNIPRO("Enabling E2EFC on cport %u\n", cportid);
    if (cportid < 32) {
        e2efc = unipro_read(CPB_RX_E2EFC_EN_0);
        e2efc |= (1 << cportid);
        unipro_write(CPB_RX_E2EFC_EN_0, e2efc);
    } else if (cportid < APBRIDGE_CPORT_MAX) {
        e2efc = unipro_read(CPB_RX_E2EFC_EN_1);
        e2efc |= (1 << (cportid - 32));
        unipro_write(CPB_RX_E2EFC_EN_1, e2efc);
    }
}

/**
 * @brief perform a DME access
 * @param attr attribute to access
 * @param val pointer to value to either read or write
 * @param peer 0 for local access, 1 for peer
 * @param write 0 for read, 1 for write
 * @param result_code unipro return code, optional
 */
int unipro_attr_access(uint16_t attr,
                       uint32_t *val,
                       uint16_t selector,
                       int peer,
                       int write) {
    int ret;
    irqstate_t flags;

    flags = irqsave();
    uint32_t ctrl = (REG_ATTRACS_CTRL_PEERENA(peer) |
                     REG_ATTRACS_CTRL_SELECT(selector) |
                     REG_ATTRACS_CTRL_WRITE(write) |
                     attr);

    unipro_write(A2D_ATTRACS_CTRL_00, ctrl);
    if (write) {
        unipro_write(A2D_ATTRACS_DATA_CTRL_00, *val);
    }

    /* Start the access */
    unipro_write(A2D_ATTRACS_MSTR_CTRL,
                 REG_ATTRACS_CNT(1) | REG_ATTRACS_UPD);

    while (!unipro_read(A2D_ATTRACS_INT_BEF))
        ;

    /* Clear status bit */
    unipro_write(A2D_ATTRACS_INT_BEF, 0x1);

    if (!write) {
        *val = unipro_read(A2D_ATTRACS_DATA_STS_00);
    }

    ret = unipro_read(A2D_ATTRACS_STS_00);
    irqrestore(flags);

    return ret;
}

/**
 * @brief Clear UniPro interrupts on a given cport
 * @param cport cport
 */
static inline void clear_rx_interrupt(struct cport *cport) {
    unsigned int cportid = cport->cportid;
    unipro_write(AHM_RX_EOM_INT_BEF_REG(cportid),
                 AHM_RX_EOM_INT_BEF(cportid));
}

/**
 * @brief Enable UniPro RX interrupts on a given cport
 * @param cport cport
 */
static inline void enable_rx_interrupt(struct cport *cport) {
    unsigned int cportid = cport->cportid;
    uint32_t reg = AHM_RX_EOM_INT_EN_REG(cportid);
    uint32_t bit = AHM_RX_EOM_INT_EN(cportid);

    unipro_write(reg, unipro_read(reg) | bit);
}

/**
 * @brief RX EOM interrupt handler
 * @param irq irq number
 * @param context register context (unused)
 */
static int irq_rx_eom(int irq, void *context) {
    struct cport *cport = irqn_to_cport(irq);
    void *data = cport->rx_buf;
    uint32_t transferred_size;
    (void)context;
    void *newbuf;

    clear_rx_interrupt(cport);

    if (!cport->driver) {
        lldbg("dropping message on cport %hu where no driver is registered\n",
              cport->cportid);
        return -ENODEV;
    }

    transferred_size = unipro_read(CPB_RX_TRANSFERRED_DATA_SIZE_00 +
                                   (cport->cportid * sizeof(uint32_t)));
    DBG_UNIPRO("cport: %u driver: %s size=%u payload=0x%x\n",
                cport->cportid,
                cport->driver->name, transferred_size,
                data);

    if (cport->driver->rx_handler) {
        newbuf = unipro_rxbuf_alloc(cport->cportid);
        if (newbuf) {
            unipro_switch_rxbuf(cport->cportid, newbuf);
            unipro_unpause_rx(cport->cportid);
        } else {
            cport->switch_buf_on_free = true;
        }

        cport->driver->rx_handler(cport->cportid, data,
                                  (size_t)transferred_size);
    }

    return 0;
}

#if !CONFIG_UNIPRO_P2P_APBA
static int tsb_unipro_mbox_ack(uint16_t val);

/**
 * @brief See ENG-376.
 *
 * We use a mailbox notification from the SVC to solve a race condition
 * involving FCT transmission. When the SVC makes a connection, it sets all the
 * relevant DME parameters as defined in MIPI UniPro 1.6, then pokes the
 * bridge mailbox, telling it that it is safe to send FCTs on a given CPort.
 */
static int mailbox_evt(void)
{
    uint32_t cportid;
    int rc;
    uint32_t e2efc;
    uint32_t val;

    DBG_UNIPRO("mailbox interrupt received\n");

    /*
     * Figure out which CPort to turn on FCT. The desired CPort is always
     * the mailbox value - 1.
     */
    rc = unipro_attr_local_read(TSB_MAILBOX, &cportid, 0);
    if (rc) {
        return rc;
    }
    cportid--;
    if (cportid >= cport_count) {
        DBG_UNIPRO("cportid %d in mailbox exceeds count of cports %d\n",
                   cportid, cport_count);
        return -EINVAL;
    }

    rc = unipro_attr_local_read(T_CPORTFLAGS, &val, cportid);
    if (rc) {
        return rc;
    }
    if (val & CPORT_FLAGS_E2EFC) {
        DBG_UNIPRO("Enabling E2EFC on cport %u\n", cportid);
        if (cportid < 32) {
            e2efc = unipro_read(CPB_RX_E2EFC_EN_0);
            e2efc |= (1 << cportid);
            unipro_write(CPB_RX_E2EFC_EN_0, e2efc);
        } else if (cportid < APBRIDGE_CPORT_MAX) {
            e2efc = unipro_read(CPB_RX_E2EFC_EN_1);
            e2efc |= (1 << (cportid - 32));
            unipro_write(CPB_RX_E2EFC_EN_1, e2efc);
        }
    }

    configure_connected_cport(cportid);

    /* Acknowledge the mailbox write */
    rc = tsb_unipro_mbox_ack(cportid + 1);
    if (rc) {
        return rc;
    }

    return 0;
}
#endif

static void unipro_evt_handler(enum unipro_event evt)
{
    DBG_UNIPRO("UniPro: event %d.\n", evt);

    switch (evt) {
    case UNIPRO_EVT_MAILBOX:
#if !CONFIG_UNIPRO_P2P_APBA
        mailbox_evt();
#endif
        break;

    case UNIPRO_EVT_LUP_DONE:
#if defined(CONFIG_TSB_CHIP_REV_ES2) && !defined(CONFIG_UNIPRO_P2P)
        es2_apply_mphy_fixup();
#endif
        break;
    default:
        break;
    }

    if (evt_handler) {
        evt_handler(evt);
    }
}

static int irq_unipro(int irq, void *context) {
    int rc;
    uint32_t val;

    tsb_irq_clear_pending(TSB_IRQ_UNIPRO);

    if (unipro_read(LUP_INT_BEF) & UNIPRO_LUP_DONE) {
        unipro_write(LUP_INT_BEF, UNIPRO_LUP_DONE);
        unipro_evt_handler(UNIPRO_EVT_LUP_DONE);
     }

    /*
     * Clear the initial interrupt
     */
    rc = unipro_attr_local_read(TSB_INTERRUPTSTATUS, &val, 0);
    if (rc) {
        goto done;
    }

    if (val & TSB_INTERRUPTSTATUS_LINKLOSTIND) {
        unipro_evt_handler(UNIPRO_EVT_LINK_LOST);
    }

    if (val & TSB_INTERRUPTSTATUS_POWERMODIND) {
        unipro_evt_handler(UNIPRO_EVT_PWRMODE);
    }

    if (val & TSB_INTERRUPTSTATUS_ERRORPHYIND) {
        unipro_evt_handler(UNIPRO_EVT_PHY_ERROR);
    }

    if (val & TSB_INTERRUPTSTATUS_ERRORPAIND) {
        unipro_evt_handler(UNIPRO_EVT_PA_ERROR);
    }

    if (val & TSB_INTERRUPTSTATUS_ERRORDIND) {
        unipro_evt_handler(UNIPRO_EVT_D_ERROR);
    }

    if (val & TSB_INTERRUPTSTATUS_ERRORNIND) {
        unipro_evt_handler(UNIPRO_EVT_N_ERROR);
    }

    if (val & TSB_INTERRUPTSTATUS_ERRORTIND) {
        unipro_evt_handler(UNIPRO_EVT_T_ERROR);
    }

    if (val & TSB_INTERRUPTSTATUS_PAINITERR) {
        unipro_evt_handler(UNIPRO_EVT_PAINIT_ERROR);
    }

    if (val & TSB_INTERRUPTSTATUS_MAILBOX) {
        unipro_evt_handler(UNIPRO_EVT_MAILBOX);
    }

done:
    return 0;
}

int unipro_unpause_rx(unsigned int cportid)
{
    struct cport *cport;

    cport = cport_handle(cportid);
    if (!cport || !cport->connected) {
        return -EINVAL;
    }

    /* Restart the flow of received data */
    unipro_write(REG_RX_PAUSE_SIZE_00 + (cport->cportid * sizeof(uint32_t)),
                 (1 << 31) | CPORT_BUF_SIZE);

    return 0;
}

static void configure_transfer_mode(int mode) {
    /*
     * Set transfer mode 2
     */
    switch (mode) {
    case 2:
        unipro_write(AHM_MODE_CTRL_0, TRANSFER_MODE_2_CTRL_0);
        if (tsb_get_product_id() == tsb_pid_apbridge) {
            unipro_write(AHM_MODE_CTRL_1, TRANSFER_MODE_2_CTRL_1);
            unipro_write(AHM_MODE_CTRL_2, TRANSFER_MODE_2_CTRL_2);
        }
        break;
    default:
        lldbg("Unsupported transfer mode: %u\n", mode);
        break;
    }
}

/**
 * @brief           Return the free space in Unipro TX FIFO (in bytes)
 * @return          free space in Unipro TX FIFO (in bytes)
 * @param[in]       cport: CPort handle
 */
uint16_t unipro_get_tx_free_buffer_space(struct cport *cport)
{
    unsigned int cportid = cport->cportid;
    uint32_t tx_space;

    tx_space = 8 * (unipro_read(CPB_TX_BUFFER_SPACE_REG(cportid)) &
                    CPB_TX_BUFFER_SPACE_MASK);
    tx_space -= 8 * (unipro_read(REG_TX_BUFFER_SPACE_OFFSET_REG(cportid)) &
                     CPB_TX_BUFFER_SPACE_OFFSET_MASK);

    return tx_space;
}

#ifdef UNIPRO_DEBUG
/**
 * @brief UniPro debug dump
 */
static void dump_regs(void) {
    uint32_t val;
    unsigned int i;

#define DBG_ATTR(attr) do {                  \
    unipro_attr_local_read(attr, &val, 0); \
    lldbg("    [%s]: 0x%x\n", #attr, val);   \
} while (0);

#define DBG_CPORT_ATTR(attr, cportid) do {         \
    unipro_attr_local_read(attr, &val, cportid); \
    lldbg("    [%s]: 0x%x\n", #attr, val);         \
} while (0);

#define REG_DBG(reg) do {                 \
    val = unipro_read(reg);               \
    lldbg("    [%s]: 0x%x\n", #reg, val); \
} while (0)

    lldbg("DME Attributes\n");
    lldbg("========================================\n");
    DBG_ATTR(PA_ACTIVETXDATALANES);
    DBG_ATTR(PA_ACTIVERXDATALANES);
    DBG_ATTR(PA_TXGEAR);
    DBG_ATTR(PA_TXTERMINATION);
    DBG_ATTR(PA_HSSERIES);
    DBG_ATTR(PA_PWRMODE);
    DBG_ATTR(PA_ACTIVERXDATALANES);
    DBG_ATTR(PA_RXGEAR);
    DBG_ATTR(PA_RXTERMINATION);
    DBG_ATTR(PA_PWRMODEUSERDATA0);
    DBG_ATTR(N_DEVICEID);
    DBG_ATTR(N_DEVICEID_VALID);
    DBG_ATTR(DME_DDBL1_REVISION);
    DBG_ATTR(DME_DDBL1_LEVEL);
    DBG_ATTR(DME_DDBL1_DEVICECLASS);
    DBG_ATTR(DME_DDBL1_MANUFACTURERID);
    DBG_ATTR(DME_DDBL1_PRODUCTID);
    DBG_ATTR(DME_DDBL1_LENGTH);
    DBG_ATTR(TSB_DME_DDBL2_A);
    DBG_ATTR(TSB_DME_DDBL2_B);
    DBG_ATTR(TSB_MAILBOX);
    DBG_ATTR(TSB_MAXSEGMENTCONFIG);
    DBG_ATTR(TSB_DME_POWERMODEIND);

    lldbg("Unipro Interrupt Info:\n");
    lldbg("========================================\n");
    REG_DBG(UNIPRO_INT_EN);
    REG_DBG(AHM_RX_EOM_INT_EN_0);
    REG_DBG(AHM_RX_EOM_INT_EN_1);

    REG_DBG(UNIPRO_INT_BEF);
    REG_DBG(AHS_TIMEOUT_INT_BEF_0);
    REG_DBG(AHS_TIMEOUT_INT_BEF_1);
    REG_DBG(AHM_HRESP_ERR_INT_BEF_0);
    REG_DBG(AHM_HRESP_ERR_INT_BEF_1);
    REG_DBG(CPB_RX_E2EFC_RSLT_ERR_INT_BEF_0);
    REG_DBG(CPB_RX_E2EFC_RSLT_ERR_INT_BEF_1);
    REG_DBG(CPB_TX_RSLTCODE_ERR_INT_BEF_0);
    REG_DBG(CPB_TX_RSLTCODE_ERR_INT_BEF_1);
    REG_DBG(CPB_RX_MSGST_ERR_INT_BEF_0);
    REG_DBG(CPB_RX_MSGST_ERR_INT_BEF_1);
    REG_DBG(LUP_INT_BEF);
    REG_DBG(A2D_ATTRACS_INT_BEF);
    REG_DBG(AHM_RX_EOM_INT_BEF_0);
    REG_DBG(AHM_RX_EOM_INT_BEF_1);
    REG_DBG(AHM_RX_EOM_INT_BEF_2);
    REG_DBG(AHM_RX_EOT_INT_BEF_0);
    REG_DBG(AHM_RX_EOT_INT_BEF_1);

    lldbg("Unipro Registers:\n");
    lldbg("========================================\n");
    REG_DBG(AHM_MODE_CTRL_0);
    if (tsb_get_product_id() == tsb_pid_apbridge) {
        REG_DBG(AHM_MODE_CTRL_1);
        REG_DBG(AHM_MODE_CTRL_2);
    }
    REG_DBG(AHM_ADDRESS_00);
    REG_DBG(REG_RX_PAUSE_SIZE_00);
    REG_DBG(CPB_RX_TRANSFERRED_DATA_SIZE_00);
    REG_DBG(CPB_TX_BUFFER_SPACE_00);
    REG_DBG(CPB_TX_RESULTCODE_0);
    REG_DBG(AHS_HRESP_MODE_0);
    REG_DBG(AHS_TIMEOUT_00);
    REG_DBG(CPB_TX_E2EFC_EN_0);
    REG_DBG(CPB_TX_E2EFC_EN_1);
    REG_DBG(CPB_RX_E2EFC_EN_0);
    REG_DBG(CPB_RX_E2EFC_EN_1);
    REG_DBG(CPORT_STATUS_0);
    REG_DBG(CPORT_STATUS_1);
    REG_DBG(CPORT_STATUS_2);

    lldbg("Connected CPorts:\n");
    lldbg("========================================\n");
    for (i = 0; i < cport_count; i++) {
        struct cport *cport = cport_handle(i);
        if (!cport) {
            continue;
        }

        val = cport_get_status(cport);

        if (val == CPORT_STATUS_CONNECTED) {
            lldbg("CPORT %u:\n", i);
            DBG_CPORT_ATTR(T_PEERDEVICEID, i);
            DBG_CPORT_ATTR(T_PEERCPORTID, i);
            DBG_CPORT_ATTR(T_TRAFFICCLASS, i);
            DBG_CPORT_ATTR(T_CPORTFLAGS, i);
            DBG_CPORT_ATTR(T_LOCALBUFFERSPACE, i);
            DBG_CPORT_ATTR(T_PEERBUFFERSPACE, i);
            DBG_CPORT_ATTR(T_CREDITSTOSEND, i);
            DBG_CPORT_ATTR(T_RXTOKENVALUE, i);
            DBG_CPORT_ATTR(T_TXTOKENVALUE, i);
            DBG_CPORT_ATTR(T_CONNECTIONSTATE, i);
        }
    }

    lldbg("NVIC:\n");
    lldbg("========================================\n");
    tsb_dumpnvic();
}

/*
 * public interfaces
 */

/**
 * @brief Print out a bunch of debug information on the console
 */
void unipro_info(void)
{
    dump_regs();
}
#endif

void unipro_switch_rxbuf(unsigned int cportid, void *buffer)
{
    uint32_t ahm_address = AHM_ADDRESS_00;
    struct cport *cport = cport_handle(cportid);

    if (!cport)
        return;

    if (cportid < CPORTID_CDSI0)
        ahm_address += (cportid * sizeof(uint32_t));
    else
        ahm_address += ((cportid - 2) * sizeof(uint32_t));

    cport->rx_buf = buffer;
    unipro_write(ahm_address, (uint32_t) buffer);
}

/**
 * @brief Initialize one UniPro cport
 */
static int unipro_init_cport(unsigned int cportid)
{
    struct cport *cport = cport_handle(cportid);

    if (!cport) {
        return -EINVAL;
    }

    if (cport->connected)
        return 0;

    _unipro_reset_cport(cportid);

    if (cport->rx_buf)
        unipro_rxbuf_free(cportid, cport->rx_buf);

    atomic_init(&cport->inflight_buf_count, 0);
#if defined(CONFIG_APBRIDGEA)
    cport->max_inflight_buf_count = 1;
#else
    cport->max_inflight_buf_count = CONFIG_TSB_UNIPRO_MAX_INFLIGHT_BUFCOUNT;
#endif
    cport->switch_buf_on_free = false;

    cport->rx_buf = unipro_rxbuf_alloc(cportid);
    if (!cport->rx_buf) {
        lowsyslog("unipro: couldn't allocate initial buffer for CP%u\n",
                  cportid);
        return -ENOMEM;
    }

    unipro_switch_rxbuf(cportid, cport->rx_buf);

#ifdef UNIPRO_DEBUG
    unipro_info();
#endif

    return 0;
}

void unipro_set_event_handler(unipro_event_handler_t handler)
{
    evt_handler = handler;
}

void unipro_init_with_event_handler(unipro_event_handler_t handler)
{
    unipro_set_event_handler(handler);
    unipro_init();
}

/**
 * @brief Initialize the UniPro core
 */
void unipro_init(void)
{
    unsigned int i;
    int retval;
    struct cport *cport;

    if (!cporttable) {
        cport_count = unipro_cport_count();
        cporttable = zalloc(sizeof(struct cport) * cport_count);
        if (!cporttable) {
            return;
        }

        retval = unipro_tx_init();
        if (retval) {
            free(cporttable);
            cporttable = NULL;
            return;
        }

        for (i = 0; i < cport_count; i++) {
            cport = &cporttable[i];
            cport->tx_buf = CPORT_TX_BUF(i);
            cport->cportid = i;
            cport->connected = 0;
            list_init(&cport->tx_fifo);
        }
    }

    unipro_write(LUP_INT_EN, 0x1);

#if defined(CONFIG_UNIPRO_P2P)
    unipro_p2p_setup();
#endif

    /*
     * Set transfer mode 2 on all cports
     * Receiver choses address for received message
     * Header is delivered transparently to receiver (and used to carry the first eight
     * L4 payload bytes)
     */
    DEBUGASSERT(TRANSFER_MODE == 2);
    configure_transfer_mode(TRANSFER_MODE);

    /*
     * Initialize cports.
     */
    unipro_write(UNIPRO_INT_EN, 0x0);
    for (i = 0; i < cport_count; i++) {
        unipro_init_cport(i);
    }
    unipro_write(UNIPRO_INT_EN, 0x1);

    /*
     * Disable FCT transmission. See ENG-376.
     */
    unipro_write(CPB_RX_E2EFC_EN_0, 0x0);
    if (tsb_get_product_id() == tsb_pid_apbridge) {
        unipro_write(CPB_RX_E2EFC_EN_1, 0x0);
    }

    /*
     * Enable the mailbox interrupt
     */
    retval = unipro_attr_local_write(TSB_INTERRUPTENABLE, 0x1 << 15, 0);
    if (retval) {
        lldbg("Failed to enable mailbox interrupt\n");
    }
    irq_attach(TSB_IRQ_UNIPRO, irq_unipro);
    up_enable_irq(TSB_IRQ_UNIPRO);

#ifdef UNIPRO_DEBUG
    unipro_info();
#endif
    lldbg("UniPro enabled\n");
}

/**
 * @brief Perform a DME get request
 * @param attr DME attribute address
 * @param val destination to read into
 * @param selector attribute selector index, or NCP_SELINDEXNULL if none
 * @param peer 1 if peer access, 0 if local
 * @param result_code destination for access result
 * @return 0
 */
int unipro_attr_read(uint16_t attr,
                     uint32_t *val,
                     uint16_t selector,
                     int peer)
{
    return unipro_attr_access(attr, val, selector, peer, 0);
}

/**
 * @brief Perform a DME set request
 * @param attr DME attribute address
 * @param val value to write
 * @param selector attribute selector index, or NCP_SELINDEXNULL if none
 * @param peer 1 if peer access, 0 if local
 * @param result_code destination for access result
 * @return 0
 */
int unipro_attr_write(uint16_t attr,
                      uint32_t val,
                      uint16_t selector,
                      int peer)
{
    return unipro_attr_access(attr, &val, selector, peer, 1);
}

int _unipro_reset_cport(unsigned int cportid)
{
    int rc;
    int retval = 0;
    uint32_t tx_reset_offset;
    uint32_t rx_reset_offset;
    uint32_t tx_queue_empty_offset;
    unsigned tx_queue_empty_bit;

    if (cportid >= cport_count) {
        return -EINVAL;
    }

    tx_queue_empty_offset = CPB_TXQUEUEEMPTY_0 + ((cportid / 32) << 2);
    tx_queue_empty_bit = (1 << (cportid % 32));

    if (cportid != CPORTID_CDSI0 && cportid != CPORTID_CDSI1) {
        while (!(getreg32(AIO_UNIPRO_BASE + tx_queue_empty_offset) &
                tx_queue_empty_bit)) {
        }
    }

    tx_reset_offset = TX_SW_RESET_00 + (cportid << 2);
    rx_reset_offset = RX_SW_RESET_00 + (cportid << 2);

    putreg32(CPORT_SW_RESET_BITS, AIO_UNIPRO_BASE + tx_reset_offset);

    rc = unipro_attr_local_write(T_CONNECTIONSTATE, 0, cportid);
    if (rc) {
        lowsyslog("error resetting T_CONNECTIONSTATE (%d)\n", rc);
        retval = -EIO;
    }

    rc = unipro_attr_local_write(T_LOCALBUFFERSPACE, 0, cportid);
    if (rc) {
        lowsyslog("error resetting T_LOCALBUFFERSPACE (%d)\n", rc);
        retval = -EIO;
    }

    rc = unipro_attr_local_write(T_PEERBUFFERSPACE, 0, cportid);
    if (rc) {
        lowsyslog("error resetting T_PEERBUFFERSPACE (%d)\n", rc);
        retval = -EIO;
    }

    rc = unipro_attr_local_write(T_CREDITSTOSEND, 0, cportid);
    if (rc) {
        lowsyslog("error resetting T_CREDITSTOSEND (%d)\n", rc);
        retval = -EIO;
    }

    putreg32(CPORT_SW_RESET_BITS, AIO_UNIPRO_BASE + rx_reset_offset);
    putreg32(0, AIO_UNIPRO_BASE + tx_reset_offset);
    putreg32(0, AIO_UNIPRO_BASE + rx_reset_offset);

    return retval;
}

int unipro_reset_cport(unsigned int cportid, cport_reset_completion_cb_t cb,
                       void *priv)
{
    struct cport *cport;

    cport = cport_handle(cportid);
    if (!cport)
        return -EINVAL;

    if (cport->pending_reset || cport->reset_completion_cb ||
        cport->reset_completion_cb_priv) {
        return -EINPROGRESS;
    }

    cport->reset_completion_cb_priv = priv;
    cport->reset_completion_cb = cb;
    cport->pending_reset = true;

    unipro_reset_notify(cportid);

    return 0;
}

/**
 * @brief Enable the cport registers
 * @param cportid cport number to associate this driver to
 * @return 0 on success, <0 on error
 */
int unipro_enable_cport(unsigned int cportid) {
    int rc;
    uint32_t val = 0;

    rc = unipro_attr_local_read(T_CPORTFLAGS, &val, cportid);
    if (rc) {
        goto done;
    }

    if (val & CPORT_FLAGS_E2EFC) {
        enable_e2efc(cportid);
    }

done:
    rc = configure_connected_cport(cportid);
    return rc;
}

/**
 * @brief Register a driver with the unipro core
 * @param drv unipro driver to register
 * @param cportid cport number to associate this driver to
 * @return 0 on success, <0 on error
 */
int unipro_driver_register(struct unipro_driver *driver, unsigned int cportid)
{
    struct cport *cport = cport_handle(cportid);
    if (!cport) {
        return -ENODEV;
    }

    if (cport->driver) {
        lldbg("ERROR: Already registered by: %s\n",
              cport->driver->name);
        return -EEXIST;
    }

    cport->driver = driver;

    lldbg("Registered driver %s on %sconnected CP%u\n",
          cport->driver->name, cport->connected ? "" : "un",
          cport->cportid);
    return 0;
}

int unipro_driver_unregister(unsigned int cportid)
{
    struct cport *cport = cport_handle(cportid);
    if (!cport) {
        return -ENODEV;
    }

    lldbg("Unregistered driver %s on %sconnected CP%u\n",
          cport->driver->name, cport->connected ? "" : "un",
          cport->cportid);

    cport->driver = NULL;

    return 0;
}

/**
 * @brief Set the mailbox value and wait for it to be cleared.
 */
int tsb_unipro_mbox_send(uint32_t val) {
    int rc;
    uint32_t irq_status, retries = 2048;

    rc = unipro_attr_peer_write(TSB_MAILBOX, val, 0);
    if (rc) {
        lldbg("%s: TSB_MAILBOX write failed: %d\n", __func__, rc);
        return rc;
    }

    do {
        rc = unipro_attr_peer_read(TSB_INTERRUPTSTATUS, &irq_status, 0);
        if (rc) {
            lldbg("%s: TSB_INTERRUPTSTATUS poll failed: %d\n", __func__, rc);
            return rc;
        }
    } while ((irq_status & TSB_INTERRUPTSTATUS_MAILBOX) && --retries > 0);

    if (!retries) {
        lldbg("%s: poll of mbox ISR timed out\n", __func__);
        return -ETIMEDOUT;
    }

    return rc;
}

#if !CONFIG_UNIPRO_P2P_APBA
/**
 * Since the switch has no 32-bit MBOX_ACK_ATTR attribute, we need to repurpose
 * a 16-bit attribute, which means that received mbox values must fit inside a
 * uint16_t.
 */
static int tsb_unipro_mbox_ack(uint16_t val) {
    int rc;

    rc = unipro_attr_local_write(MBOX_ACK_ATTR, val, 0);
    if (rc) {
        lldbg("MBOX_ACK_ATTR complement write of 0x%x failed: %d\n", val, rc);
        return rc;
    }

    return 0;
}
#endif

