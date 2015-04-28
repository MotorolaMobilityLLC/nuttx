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
 * @brief MIPI UniPro stack for ES2 Bridges
 */

#include <nuttx/greybus/unipro.h>
#include <nuttx/greybus/tsb_unipro.h>

#include <arch/tsb/unipro.h>
#include <arch/tsb/irq.h>
#include <errno.h>

#include "debug.h"
#include "up_arch.h"
#include "tsb_scm.h"
#include "tsb_unipro_es2.h"
#include "tsb_es2_mphy_fixups.h"

/*
 * "Map" constants for M-PHY fixups.
 */
#define TSB_MPHY_MAP (0x7F)
    #define TSB_MPHY_MAP_TSB_REGISTER_1 (0x01)
    #define TSB_MPHY_MAP_NORMAL         (0x00)
    #define TSB_MPHY_MAP_TSB_REGISTER_2 (0x81)

static uint32_t unipro_read(uint32_t offset) {
    return getreg32((volatile unsigned int*)(AIO_UNIPRO_BASE + offset));
}

static void unipro_write(uint32_t offset, uint32_t v) {
    putreg32(v, (volatile unsigned int*)(AIO_UNIPRO_BASE + offset));
}

static int es2_fixup_mphy(void)
{
    uint32_t debug_0720 = tsb_get_debug_reg(0x0720);
    uint32_t urc;
    const struct tsb_mphy_fixup *fu;

    /*
     * Apply the "register 2" map fixups.
     */
    unipro_attr_local_write(TSB_MPHY_MAP, TSB_MPHY_MAP_TSB_REGISTER_2, 0,
                            &urc);
    if (urc) {
        lldbg("%s: failed to switch to register 2 map: %u\n",
              __func__, urc);
        return urc;
    }
    fu = tsb_register_2_map_mphy_fixups;
    do {
        unipro_attr_local_write(fu->attrid, fu->value, fu->select_index,
                                &urc);
        if (urc) {
            lldbg("%s: failed to apply register 1 map fixup: %u\n",
                  __func__, urc);
            return urc;
        }
    } while (!tsb_mphy_fixup_is_last(fu++));

    /*
     * Switch to "normal" map.
     */
    unipro_attr_local_write(TSB_MPHY_MAP, TSB_MPHY_MAP_NORMAL, 0,
                            &urc);
    if (urc) {
        lldbg("%s: failed to switch to normal map: %u\n",
              __func__, urc);
        return urc;
    }

    /*
     * Apply the "register 1" map fixups.
     */
    unipro_attr_local_write(TSB_MPHY_MAP, TSB_MPHY_MAP_TSB_REGISTER_1, 0,
                            &urc);
    if (urc) {
        lldbg("%s: failed to switch to register 1 map: %u\n",
              __func__, urc);
        return urc;
    }
    fu = tsb_register_1_map_mphy_fixups;
    do {
        if (tsb_mphy_r1_fixup_is_magic(fu)) {
            /* The magic R1 fixups come from the mysterious and solemn
             * debug register 0x0720. */
            unipro_attr_local_write(0x8002, (debug_0720 >> 1) & 0x1f, 0, &urc);
        } else {
            unipro_attr_local_write(fu->attrid, fu->value, fu->select_index,
                                    &urc);
        }
        if (urc) {
            lldbg("%s: failed to apply register 1 map fixup: %u\n",
                  __func__, urc);
            return urc;
        }
    } while (!tsb_mphy_fixup_is_last(fu++));

    /*
     * Switch to "normal" map.
     */
    unipro_attr_local_write(TSB_MPHY_MAP, TSB_MPHY_MAP_NORMAL, 0,
                            &urc);
    if (urc) {
        lldbg("%s: failed to switch to normal map: %u\n",
              __func__, urc);
        return urc;
    }

    return 0;
}

static uint32_t cport_get_status(unsigned int cport) {
    uint32_t val;
    uint32_t reg;

    reg = CPORT_STATUS_0 + ((cport/16) << 2);
    val = unipro_read(reg);
    val >>= ((cport % 16) << 1);
    return (val & (0x3));
}

/**
 * @brief perform a DME access
 * @param attr attribute to access
 * @param val pointer to value to either read or write
 * @param peer 0 for local access, 1 for peer
 * @param write 0 for read, 1 for write
 * @param result_code unipro return code, optional
 */
static int unipro_attr_access(uint16_t attr,
                              uint32_t *val,
                              uint16_t selector,
                              int peer,
                              int write,
                              uint32_t *result_code) {

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

    if (result_code) {
        *result_code = unipro_read(A2D_ATTRACS_STS_00);
    }

    if (!write) {
        *val = unipro_read(A2D_ATTRACS_DATA_STS_00);
    }

    return 0;
}

/**
 * @brief UniPro debug dump
 */
static void dump_regs(void) {
    uint32_t val;
    unsigned int rc;
    unsigned int i;

#define DBG_ATTR(attr) do {                  \
    unipro_attr_local_read(attr, &val, 0, &rc); \
    lldbg("    [%s]: 0x%x\n", #attr, val);   \
} while (0);

#define DBG_CPORT_ATTR(attr, cportid) do {         \
    unipro_attr_local_read(attr, &val, cportid, &rc); \
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
    for (i = 0; i < CPORT_MAX; i++) {
        val = cport_get_status(i);

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

/**
 * @brief Initialize one UniPro cport
 */
int unipro_init_cport(unsigned int cportid)
{
    return -ENOSYS;
}

/**
 * @brief Initialize the UniPro core
 */
void unipro_init(void)
{
    if (es2_fixup_mphy()) {
        lldbg("Failed to apply M-PHY fixups (results in link instability at HS-G1).\n");
    }
}

/**
 * @brief send data down a CPort
 * @param cportid cport to send down
 * @param buf data buffer
 * @param len size of data to send
 * @param 0 on success, <0 on error
 */
int unipro_send(unsigned int cportid, const void *buf, size_t len)
{
    return -ENOSYS;
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
                     int peer,
                     uint32_t *result_code)
{
    return unipro_attr_access(attr, val, selector, peer, 0, result_code);
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
                      int peer,
                      uint32_t *result_code)
{
    return unipro_attr_access(attr, &val, selector, peer, 1, result_code);
}

/**
 * @brief Register a driver with the unipro core
 * @param drv unipro driver to register
 * @param cportid cport number to associate this driver to
 * @return 0 on success, <0 on error
 */
int unipro_driver_register(struct unipro_driver *driver, unsigned int cportid)
{
    return -ENOSYS;
}
