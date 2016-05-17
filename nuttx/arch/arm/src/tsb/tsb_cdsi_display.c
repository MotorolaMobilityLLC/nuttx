/*
 * Copyright (c) 2015 Motorola Mobility, LLC.
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
 */

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/types.h>

#include <arch/byteorder.h>

#include <arch/chip/cdsi.h>
#include <arch/chip/cdsi_config.h>
#include <arch/chip/cdsi_display.h>
#include <arch/chip/cdsi_reg_def.h>

#include <nuttx/util.h>

#define CDSI_NUM_LOOPS(n) (n / sizeof(uint32_t)) + (n % sizeof(uint32_t) ? 1 : 0)

int dsi_write_cmd(struct cdsi_dev *dev, const struct dsi_cmd *cmd) {
    int ret;

    vdbg("\n");

    /* Wait until APF is not busy. */
    ret = CDSI_READ_UNTIL_CLR_RETRIES(dev, CDSI_CDSITX_SIDEBAND_STATUS_05,
                APF_CMD_BUSY, CDSI_DEFAULT_RETRIES);
    if (ret < 0) {
        dbg("ERROR: APF command is busy\n");
    }
    ret = CDSI_READ_UNTIL_CLR_RETRIES(dev, CDSI_CDSITX_INTERRUPT_STATUS_06,
                INT_APF_CMD_DONE, CDSI_DEFAULT_RETRIES);
    if (ret < 0) {
        dbg("ERROR: APF command not done\n");
    }

    /* Write APF command type */
    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_CMDIF_01_OFFS, cmd->ctype);

    /* Write the DI and either the word-count (for long packets) or the data
       in the word-count field (for short packets). */
    if (cmd->ctype & CTYPE_LONG_FLAG) {
        CDSI_WRITE(dev, CDSI_CDSITX_SIDEBAND_CMDIF_02, APF_CMD_PKT_DI, cmd->dt);
        CDSI_MODIFY(dev, CDSI_CDSITX_SIDEBAND_CMDIF_02, APF_CMD_PKT_WC, cmd->u.lp.length);
    } else {
        CDSI_WRITE(dev, CDSI_CDSITX_SIDEBAND_CMDIF_02, APF_CMD_PKT_DI, cmd->dt);
        /* The least significant 16-bits of parameter make up the the 16-bits of data. */
        CDSI_MODIFY(dev, CDSI_CDSITX_SIDEBAND_CMDIF_02, APF_CMD_PKT_WC, cmd->u.sp.data);
    }

    /* Set command request. */
    CDSI_WRITE(dev, CDSI_CDSITX_SIDEBAND_CMDIF_00, APF_CMD_REQ, 1);

    /* If a long packet, write the data. */
    if (cmd->ctype & CTYPE_LONG_FLAG) {
        size_t loops = CDSI_NUM_LOOPS(cmd->u.lp.length);
        size_t i;
        for (i = 0; i < loops; i++) {
            cdsi_write(dev, CDSI_CDSITX_SIDEBAND_CMDIF_03_OFFS, cmd->u.lp.data[i]);
        }
    }

    /* Wait until APF command is done. */
    ret = CDSI_READ_UNTIL_SET_RETRIES(dev, CDSI_CDSITX_INTERRUPT_STATUS_06,
                INT_APF_CMD_DONE, CDSI_DEFAULT_RETRIES);
    if (ret < 0) {
        dbg("ERROR: APF command not complete\n");
    }
    /* Clear APF command done. */
    CDSI_WRITE(dev, CDSI_CDSITX_INTERRUPT_STATUS_06, INT_APF_CMD_DONE, 1);

    return 0;
}

static size_t _dsi_read_cmd(struct cdsi_dev *dev, uint8_t *buffer, size_t buffer_length) {
    vdbg("\n");
    size_t count = 0;
    int ret;

    /* First send a command that starts a BTA and read operation. */

    /* Wait for RX BTA to complete. */
    ret = CDSI_READ_UNTIL_SET_RETRIES(dev, CDSI_CDSITX_INTERRUPT_STATUS_05,
                INT_DPHY_DIRECTION_RX, CDSI_DEFAULT_RETRIES);
    if (ret < 0) {
        dbg("ERROR: RX BTA is not complete\n");
    }

    /* Clear the RX BTA interrupt status. */
    CDSI_WRITE(dev, CDSI_CDSITX_INTERRUPT_STATUS_05, INT_DPHY_DIRECTION_RX, 1);

    /* Wait for packet reception. */
    ret = CDSI_READ_UNTIL_SET_RETRIES(dev, CDSI_CDSITX_INTERRUPT_STATUS_05,
                INT_APF_LPRX_PKTSTART, CDSI_DEFAULT_RETRIES);
    if (ret < 0) {
        dbg("ERROR: Packet not received\n");
    }

    uint8_t dt = CDSI_READ(dev, CDSI_CDSITX_SIDEBAND_LPRXIF_01, SBO_APF_LPRX_PKTID);
    vdbg("dt=%x\n", dt);
    uint16_t word_count = CDSI_READ(dev, CDSI_CDSITX_SIDEBAND_LPRXIF_01, SBO_APF_LPRX_PKTWC);
    vdbg("wc=%x\n", word_count);

    if (dt == DT_RSP_DCS_SHORT_READ1) {
        vdbg("short read 1\n");
        CDSI_WRITE(dev, CDSI_CDSITX_INTERRUPT_STATUS_05, INT_APF_LPRX_PKTSTART, 1);
        CDSI_WRITE(dev, CDSI_CDSITX_INTERRUPT_STATUS_05, INT_APF_LPRX_PKTEND, 1);

        size_t n = MIN(buffer_length, sizeof(uint8_t));
        memcpy(buffer, &word_count, n);
        count = n;
    } else if (dt == DT_RSP_DCS_SHORT_READ2) {
        vdbg("short read 2\n");
        CDSI_WRITE(dev, CDSI_CDSITX_INTERRUPT_STATUS_05, INT_APF_LPRX_PKTSTART, 1);
        CDSI_WRITE(dev, CDSI_CDSITX_INTERRUPT_STATUS_05, INT_APF_LPRX_PKTEND, 1);

        size_t n = MIN(buffer_length, sizeof(uint16_t));
        word_count = be16_to_cpu(word_count);
        memcpy(buffer, &word_count, n);
        count = n;
    } else if (dt == DT_RSP_DCS_LONG_READ) {
        vdbg("long read\n");
        ret = CDSI_READ_UNTIL_SET_RETRIES(dev, CDSI_CDSITX_INTERRUPT_STATUS_05,
                INT_APF_LPRX_PKTEND, CDSI_DEFAULT_RETRIES);
        if (ret < 0) {
            dbg("ERROR: RX packet end timed out\n");
        }
        CDSI_WRITE(dev, CDSI_CDSITX_INTERRUPT_STATUS_05, INT_APF_LPRX_PKTEND, 1);

        // TODO: Are word_count and fifo_level set to the same value?
        count = CDSI_READ(dev, CDSI_CDSITX_SIDEBAND_LPRXIF_03, SBO_APF_LPRX_FIFO_LVL);
        count = MIN(count, buffer_length);
        size_t remaining = count;

        size_t loops = CDSI_NUM_LOOPS(count);
        size_t i;
        for (i = 0; i < loops; i++) {
            uint32_t value = CDSI_READ(dev, CDSI_CDSITX_SIDEBAND_LPRXIF_02, SBO_APF_LPRX_FIFO_DATA);

            size_t n = MIN(remaining, sizeof(value));
            memcpy(buffer, &value, n);

            buffer += n;
            remaining -= n;
        }
    } else if (dt == DT_RSP_ACK_AND_ERR) {
        CDSI_WRITE(dev, CDSI_CDSITX_INTERRUPT_STATUS_05, INT_APF_LPRX_ACKERR, 1);
#if CONFIG_DEBUG
        uint16_t ackerr = CDSI_READ(dev, CDSI_CDSITX_SIDEBAND_LPRXIF_04, SBO_APF_LPRX_ACKERR_PKT);
        dbg("ack or error report: 0x%04x\n", ackerr);
#endif
    } else {
        dbg("ERROR: unexpected DT: 0x%02x\n", dt);
        return -EINVAL;
    }

    /* Wait for TX BTA to complete. */
    ret = CDSI_READ_UNTIL_SET_RETRIES(dev, CDSI_CDSITX_INTERRUPT_STATUS_05,
                    INT_DPHY_DIRECTION_TX, CDSI_DEFAULT_RETRIES);
    if (ret < 0) {
        dbg("ERROR: TX BTA not complete\n");
    }
    ret = CDSI_READ_UNTIL_CLR_RETRIES(dev, CDSI_CDSITX_INTERRUPT_STATUS_01,
                INT_DPHY_RXTRIGGERESC0, CDSI_DEFAULT_RETRIES);
    if (ret < 0) {
        dbg("ERROR: RX trigger ESC 0 timed out\n");
    }
    ret = CDSI_READ_UNTIL_CLR_RETRIES(dev, CDSI_CDSITX_INTERRUPT_STATUS_01,
                INT_DPHY_RXTRIGGERESC1, CDSI_DEFAULT_RETRIES);
    if (ret < 0) {
        dbg("ERROR: RX trigger ESC 1 timed out\n");
    }
    ret = CDSI_READ_UNTIL_CLR_RETRIES(dev, CDSI_CDSITX_INTERRUPT_STATUS_01,
                INT_DPHY_RXTRIGGERESC2, CDSI_DEFAULT_RETRIES);
    if (ret < 0) {
        dbg("ERROR: RX trigger ESC 2 timed out\n");
    }
    ret = CDSI_READ_UNTIL_CLR_RETRIES(dev, CDSI_CDSITX_INTERRUPT_STATUS_01,
                INT_DPHY_RXTRIGGERESC3, CDSI_DEFAULT_RETRIES);
    if (ret < 0) {
        dbg("ERROR: RX trigger ESC 3 timed out\n");
    }

    /* Clear the TX BTA interrupt status. */
    CDSI_WRITE(dev, CDSI_CDSITX_INTERRUPT_STATUS_05, INT_DPHY_DIRECTION_TX, 1);

    vdbg("count=%d\n", count);
    return count;
}

size_t dsi_read_cmd(struct cdsi_dev *dev, const struct dsi_cmd *cmd, uint8_t *buffer, size_t buffer_length) {
    vdbg("\n");
    const struct dsi_cmd max_ret_pkt = {CTYPE_LP_SHORT, DT_MAX_RET_PKT, .u = { .sp = { buffer_length } }};
    dsi_write_cmd(dev, &max_ret_pkt);
    dsi_write_cmd(dev, cmd);
    return _dsi_read_cmd(dev, buffer, buffer_length);
}

const static struct dsi_cmd GET_SUPPLIER_ID = { CTYPE_LP_SHORT, DT_DCS_READ0, .u = { .sp = { 0x00a1 } } };
const static struct dsi_cmd GET_ID0 = { CTYPE_LP_SHORT, DT_DCS_READ0, .u = { .sp = { 0x00da } } };
const static struct dsi_cmd GET_ID1 = { CTYPE_LP_SHORT, DT_DCS_READ0, .u = { .sp = { 0x00db } } };
const static struct dsi_cmd GET_ID2 = { CTYPE_LP_SHORT, DT_DCS_READ0, .u = { .sp = { 0x00dc } } };

int dsi_read_panel_info(struct cdsi_dev *dev, uint16_t *supplier_id, uint8_t *id0, uint8_t *id1, uint8_t *id2) {
    vdbg("\n");
    if (supplier_id) {
        dsi_read_cmd(dev, &GET_SUPPLIER_ID, (uint8_t *)supplier_id, sizeof(*supplier_id));
        dbg("supplier_id=0x%02x\n", *supplier_id);
    }

    if (id0) {
        dsi_read_cmd(dev, &GET_ID0, id0, sizeof(*id0));
        dbg("id0=0x%02x\n", *id0);
    }

    if (id1) {
        dsi_read_cmd(dev, &GET_ID1, id1, sizeof(*id1));
        dbg("id1=0x%02x\n", *id1);
    }

    if (id2) {
        dsi_read_cmd(dev, &GET_ID2, id2, sizeof(*id2));
        dbg("id2=0x%02x\n", *id2);
    }

    return 0;
}

const static struct dsi_cmd GET_POWER_MODE = { CTYPE_LP_SHORT, DT_DCS_READ0, .u = { .sp = { 0x000a } } };

int dsi_read_power_mode(struct cdsi_dev *dev, uint8_t *mode) {
    vdbg("\n");
    if (mode) {
        dsi_read_cmd(dev, &GET_POWER_MODE, mode, sizeof(*mode));
        dbg("power_mode=0x%02x\n", *mode);
    }

    return 0;
}
