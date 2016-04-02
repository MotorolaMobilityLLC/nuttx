/*
 * Copyright (c) 2016 Motorola Mobility, LLC.
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

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>

#include <arch/byteorder.h>

#include <arch/chip/cdsi.h>
#include <arch/chip/cdsi_config.h>
#include <arch/chip/cdsi_dbg.h>
#include <arch/chip/cdsi_display.h>
#include <arch/chip/cdsi_reg_def.h>
#include <arch/chip/unipro_p2p.h>

#include <nuttx/device.h>
#include <nuttx/util.h>

#include <nuttx/mhb/device_mhb.h>
#include <nuttx/mhb/mhb_protocol.h>

#ifdef CONFIG_RAMLOG_SYSLOG
    #include <nuttx/syslog/ramlog.h>
#endif

#include <nuttx/unipro/unipro.h>

#include <apps/ice/ipc.h>

#include "sm.h"

#define CDSI0_CPORT (16)
#define CDSI1_CPORT (17)

#define CDSI_MAX_INST (2)

#define CDSI_INST_TO_CPORT(x) (x + CDSI0_CPORT)
#define CDSI_CPORT_TO_INST(x) (x - CDSI0_CPORT)

#define I2S_CPORT (18)

#define I2S_INST_TO_CPORT(x) (I2S_CPORT)
#define I2S_CPORT_TO_INST(x) (I2S_CPORT)

#define MHB_IPC_TIMEOUT (500)

static struct device *g_uart_dev;

struct mhb_msg {
    struct mhb_hdr *hdr;
    uint8_t *payload;
    size_t payload_length;
    size_t payload_max;
};

struct mhb_transaction {
    struct device *dev;
    struct mhb_msg in_msg;
    struct mhb_msg out_msg;
    bool send_rsp;
};

struct cdsi_block {
    struct cdsi_dev *dev;
    struct cdsi_config config;
};

static struct cdsi_block g_cdsi_blocks[CDSI_MAX_INST];

#if CONFIG_ICE_IPC_CLIENT
static int mhb_unipro_send(struct mhb_transaction *transaction);
#endif

/* PM */
static int _mhb_send_pm_status_not(struct device *dev, uint8_t status)
{
    struct mhb_pm_status_not not;
    not.status = status;

    struct mhb_hdr hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_PM;
    hdr.type = MHB_TYPE_PM_STATUS_NOT;

    return device_mhb_send(dev, &hdr, (uint8_t *)&not, sizeof(not), 0);
}

int mhb_send_pm_status_not(uint8_t status)
{
    return _mhb_send_pm_status_not(g_uart_dev, status);
}

/* UniPro */
static int mhb_handle_unipro_config_req(struct mhb_transaction *transaction)
{
    if (transaction->in_msg.payload_length != 0) {
        return -EINVAL;
    }

    transaction->out_msg.hdr->addr = MHB_ADDR_UNIPRO;
    transaction->out_msg.hdr->type = MHB_TYPE_UNIPRO_CONFIG_RSP;
    transaction->out_msg.hdr->result = MHB_RESULT_SUCCESS;
    transaction->send_rsp = true;
    return 0;
}

static int mhb_handle_unipro_control_req(struct mhb_transaction *transaction)
{
    struct mhb_unipro_control_req *req =
        (struct mhb_unipro_control_req *)transaction->in_msg.payload;

    if (transaction->in_msg.payload_length != sizeof(*req)) {
        return -EINVAL;
    }

    // TODO: Add gear-voting logic.

    uint8_t termination = 1;
    unipro_powermode_change(req->gear.tx, req->gear.rx, req->gear.pwrmode,
        req->gear.series, termination);

    transaction->out_msg.hdr->addr = MHB_ADDR_UNIPRO;
    transaction->out_msg.hdr->type = MHB_TYPE_UNIPRO_CONTROL_RSP;
    transaction->out_msg.hdr->result = MHB_RESULT_SUCCESS;
    transaction->send_rsp = true;
    return 0;
}

static int mhb_handle_unipro_status_req(struct mhb_transaction *transaction)
{
    struct mhb_unipro_status_rsp *rsp =
        (struct mhb_unipro_status_rsp *)transaction->out_msg.payload;

    if (transaction->in_msg.payload_length != 0) {
        return -EINVAL;
    }

    transaction->out_msg.hdr->addr = MHB_ADDR_UNIPRO;
    transaction->out_msg.hdr->type = MHB_TYPE_UNIPRO_STATUS_RSP;
    transaction->out_msg.hdr->result = MHB_RESULT_SUCCESS;
    transaction->send_rsp = true;

    /* Basic link-status for now. */
    rsp->status = unipro_p2p_is_link_up() ? 1 : 0;
    rsp->status = cpu_to_le32(rsp->status);

    return 0;
}

static int mhb_handle_unipro_read_attr_req(struct mhb_transaction *transaction)
{
    int ret;
    struct mhb_unipro_read_attr_req *req =
        (struct mhb_unipro_read_attr_req *)transaction->in_msg.payload;
    struct mhb_unipro_read_attr_rsp *rsp =
        (struct mhb_unipro_read_attr_rsp *)transaction->out_msg.payload;
    rsp->value = 0;

    if (transaction->in_msg.payload_length != sizeof(*req)) {
        return -EINVAL;
    }

    req->attribute = le16_to_cpu(req->attribute);
    req->selector = le16_to_cpu(req->selector);

    ret = unipro_attr_read(req->attribute, &rsp->value, req->selector, req->peer);

    rsp->value = le32_to_cpu(rsp->value);

    transaction->out_msg.hdr->addr = MHB_ADDR_UNIPRO;
    transaction->out_msg.hdr->type = MHB_TYPE_UNIPRO_READ_ATTR_RSP;
    transaction->out_msg.hdr->result =
        ret ? MHB_RESULT_UNKNOWN_ERROR : MHB_RESULT_SUCCESS;
    transaction->out_msg.payload_length = sizeof(*rsp);
    transaction->send_rsp = true;
    return 0;
}

static int mhb_handle_unipro_write_attr_req(struct mhb_transaction *transaction)
{
    int ret;
    struct mhb_unipro_write_attr_req *req =
        (struct mhb_unipro_write_attr_req *)transaction->in_msg.payload;

    if (transaction->in_msg.payload_length != sizeof(*req)) {
        return -EINVAL;
    }

    req->attribute = le16_to_cpu(req->attribute);
    req->selector = le16_to_cpu(req->selector);
    req->value = le32_to_cpu(req->value);

    ret = unipro_attr_write(req->attribute, req->value, req->selector, req->peer);

    transaction->out_msg.hdr->addr = MHB_ADDR_UNIPRO;
    transaction->out_msg.hdr->type = MHB_TYPE_UNIPRO_WRITE_ATTR_RSP;
    transaction->out_msg.hdr->result =
        ret ? MHB_RESULT_UNKNOWN_ERROR : MHB_RESULT_SUCCESS;
    transaction->send_rsp = true;
    return 0;

}

static int mhb_handle_unipro(struct mhb_transaction *transaction)
{
    switch (transaction->in_msg.hdr->type) {
    case MHB_TYPE_UNIPRO_CONFIG_REQ:
        return mhb_handle_unipro_config_req(transaction);
    case MHB_TYPE_UNIPRO_CONTROL_REQ:
        return mhb_handle_unipro_control_req(transaction);
    case MHB_TYPE_UNIPRO_STATUS_REQ:
        return mhb_handle_unipro_status_req(transaction);
    case MHB_TYPE_UNIPRO_READ_ATTR_REQ:
        return mhb_handle_unipro_read_attr_req(transaction);
    case MHB_TYPE_UNIPRO_WRITE_ATTR_REQ:
        return mhb_handle_unipro_write_attr_req(transaction);
    default:
        lldbg("ERROR: unknown Unipro event\n");
        return -EINVAL;
    }
}

/* CDSI */
inline struct cdsi_block *cdsi_from_inst(uint8_t inst)
{
    if (inst >= ARRAY_SIZE(g_cdsi_blocks)) {
        lldbg("ERROR: Invalid CDSI instance.\n");
        return NULL;
    }

    return &g_cdsi_blocks[inst];
}

static int mhb_handle_cdsi_config_req(struct mhb_transaction *transaction)
{
    int ret;
    struct mhb_cdsi_config_req *req =
        (struct mhb_cdsi_config_req *)transaction->in_msg.payload;
    struct mhb_cdsi_config *cfg;

    if (transaction->in_msg.payload_length != sizeof(*req)) {
        return -EINVAL;
    }

    uint8_t inst =
      (transaction->in_msg.hdr->addr & MHB_INSTANCE_MASK) >> MHB_INSTANCE_SHIFT;

    struct cdsi_block *cdsi = cdsi_from_inst(inst);
    ret = cdsi != 0 ? 0 : -EINVAL;

    cfg = &req->cfg;

    if (!ret) {
        cdsi->config.direction = cfg->direction ? TSB_CDSI_TX : TSB_CDSI_RX;
        cdsi->config.mode = cfg->mode != 0;
        cdsi->config.rx_num_lanes = cfg->rx_num_lanes;
        cdsi->config.tx_num_lanes = cfg->tx_num_lanes;
        cdsi->config.rx_mbits_per_lane = le32_to_cpu(cfg->rx_mbits_per_lane);
        cdsi->config.tx_mbits_per_lane = le32_to_cpu(cfg->tx_mbits_per_lane);
        cdsi->config.hs_rx_timeout = le32_to_cpu(cfg->hs_rx_timeout);
        cdsi->config.pll_frs = le32_to_cpu(cfg->pll_frs);
        cdsi->config.pll_prd = le32_to_cpu(cfg->pll_prd);
        cdsi->config.pll_fbd = le32_to_cpu(cfg->pll_fbd);
        cdsi->config.framerate = le32_to_cpu(cfg->framerate);
        cdsi->config.width = le32_to_cpu(cfg->width);
        cdsi->config.height = le32_to_cpu(cfg->height);
        cdsi->config.bpp = le32_to_cpu(cfg->bpp);
        cdsi->config.bta_enabled = cfg->bta_enabled != 0;
        cdsi->config.continuous_clock = cfg->continuous_clock != 0;
        cdsi->config.blank_packet_enabled = cfg->blank_packet_enabled != 0;
        cdsi->config.video_mode = cfg->video_mode != 0;
        cdsi->config.color_bar_enabled = cfg->color_bar_enabled != 0;
        cdsi->config.vss_control_payload = le32_to_cpu(cfg->vss_control_payload);
        cdsi->config.keep_alive = cfg->keep_alive;
        cdsi->config.clk_pre = cfg->t_clk_pre;
        cdsi->config.clk_post = cfg->t_clk_post;
        cdsi->config.horizontal_front_porch = cfg->horizontal_front_porch;
        cdsi->config.horizontal_back_porch = cfg->horizontal_back_porch;
        cdsi->config.horizontal_pulse_width = cfg->horizontal_pulse_width;
        cdsi->config.horizontal_sync_skew = cfg->horizontal_sync_skew;
        cdsi->config.horizontal_left_border = cfg->horizontal_left_border;
        cdsi->config.horizontal_right_border = cfg->horizontal_right_border;
        cdsi->config.vertical_front_porch = cfg->vertical_front_porch;
        cdsi->config.vertical_back_porch = cfg->vertical_back_porch;
        cdsi->config.vertical_pulse_width = cfg->vertical_pulse_width;
        cdsi->config.vertical_top_border = cfg->vertical_top_border;
        cdsi->config.vertical_bottom_border = cfg->vertical_bottom_border;
        cdsi->config.vsync_mode = cfg->vsync_mode;

        cdsi->dev = cdsi_initialize(inst, cdsi->config.direction);
        if (cdsi->dev) {
            ret = 0;
        } else {
            lldbg("ERROR: Failed to open CDSI device.\n");
            ret = -EINVAL;
        }
    }

    if (!ret) {
        if (cdsi->config.direction == TSB_CDSI_RX) {
            ret = cdsi_initialize_rx(cdsi->dev, &cdsi->config);
        } else if (cdsi->config.direction == TSB_CDSI_TX) {
            ret = cdsi_initialize_tx(cdsi->dev, &cdsi->config);
        } else {
            lldbg("ERROR: Invalid direction.\n");
            ret = -EINVAL;
            goto error;
        }
    }

    lldbg("reset cport=%d\n", CDSI_INST_TO_CPORT(inst));
    unipro_p2p_reset_connection(CDSI_INST_TO_CPORT(inst));
#if CONFIG_ICE_IPC_SERVER
    lldbg("setup cport=%d\n", CDSI_INST_TO_CPORT(inst));
    unipro_p2p_setup_connection(CDSI_INST_TO_CPORT(inst));
#endif

#if CONFIG_ICE_IPC_CLIENT
    if (!ret) {
        /* Cheat and re-use this transaction.  The unipro response (if any)
           will be in out_msg. */

        /* Swap the CDSI direction for the peer. */
        req->cfg.direction =
            (req->cfg.direction == TSB_CDSI_RX) ? TSB_CDSI_TX : TSB_CDSI_RX;

        lldbg("notify peer: direction: %d\n", req->cfg.direction);
        ret = mhb_unipro_send(transaction);
        if (ret) {
            goto error;
        }
    }
#endif

done:
    transaction->out_msg.hdr->addr = transaction->in_msg.hdr->addr;
    transaction->out_msg.hdr->type = MHB_TYPE_CDSI_CONFIG_RSP;
    transaction->out_msg.hdr->result =
        ret ? MHB_RESULT_UNKNOWN_ERROR : MHB_RESULT_SUCCESS;
    transaction->send_rsp = true;

    return 0;

error:
    lldbg("ERROR: Failed to configure.\n");
    cdsi_uninitialize(cdsi->dev);
    cdsi->dev = NULL;
    goto done;
}

static int mhb_handle_cdsi_control_req(struct mhb_transaction *transaction)
{
    int ret;
    struct mhb_cdsi_control_req *req =
        (struct mhb_cdsi_control_req *)transaction->in_msg.payload;

    uint8_t inst =
      (transaction->in_msg.hdr->addr & MHB_INSTANCE_MASK) >> MHB_INSTANCE_SHIFT;

    struct cdsi_block *cdsi = cdsi_from_inst(inst);
    ret = cdsi != 0 ? 0 : -EINVAL;

    if (!ret) {
        if (cdsi->config.direction == TSB_CDSI_TX) {
            switch (req->command) {
                case MHB_CDSI_COMMAND_NONE:
                    break;
                case MHB_CDSI_COMMAND_START:
                    ret = cdsi_tx_start(cdsi->dev);
                    break;
                case MHB_CDSI_COMMAND_STOP:
                    ret = cdsi_tx_stop(cdsi->dev);
                    break;
                default:
                    lldbg("ERROR: invalid CDSI command.\n");
                    ret = -EINVAL;
                    break;
            }
        } else {
            /* Ignore commands for the RX direction since RX is enabled during
               configuration. */
        }
    }

#if CONFIG_ICE_IPC_CLIENT
    if (!ret) {
        /* Cheat and re-use this transaction.  The unipro response (if any)
           will be in out_msg. */
        ret = mhb_unipro_send(transaction);
    }
#endif

    transaction->out_msg.hdr->addr = transaction->in_msg.hdr->addr;
    transaction->out_msg.hdr->type = MHB_TYPE_CDSI_CONTROL_RSP;
    transaction->out_msg.hdr->result =
        ret ? MHB_RESULT_UNKNOWN_ERROR : MHB_RESULT_SUCCESS;
    transaction->send_rsp = true;
    return 0;
}

static int mhb_handle_cdsi_status_req(struct mhb_transaction *transaction)
{
    transaction->out_msg.hdr->addr = transaction->in_msg.hdr->addr;
    transaction->out_msg.hdr->type = MHB_TYPE_CDSI_STATUS_RSP;
    transaction->out_msg.hdr->result = MHB_RESULT_SUCCESS;
    transaction->send_rsp = true;
    return 0;
}

static int mhb_handle_cdsi_read_cmds_req(struct mhb_transaction *transaction)
{
    int ret;
    size_t n;
    size_t i;
    uint8_t *p;
    size_t l;
    ssize_t count;

    uint8_t inst =
      (transaction->in_msg.hdr->addr & MHB_INSTANCE_MASK) >> MHB_INSTANCE_SHIFT;

    struct cdsi_block *cdsi = cdsi_from_inst(inst);
    if (!cdsi || !cdsi->dev) {
        ret = -EINVAL;
        goto error;
    }

    struct mhb_cdsi_read_cmds_req *req =
                  (struct mhb_cdsi_read_cmds_req *)transaction->in_msg.payload;

    if (transaction->in_msg.payload_length % sizeof(req->cmds[0]) != 0) {
        lldbg("ERROR: invalid size\n");
        ret = -EINVAL;
        goto error;
    }

    n = transaction->in_msg.payload_length / sizeof(req->cmds[0]);
    p = transaction->out_msg.payload;
    l = transaction->out_msg.payload_length;

    for (i = 0; i < n; i++) {
        struct dsi_cmd dsi_cmd;
        struct mhb_cdsi_cmd *src = &req->cmds[i];

        lldbg("[src] i=%d, ct=%d, dt=%d, l=%d, sd=%04x, ld=%08x %08x, l=%d\n",
              i, src->ctype, src->dtype, src->length,
              src->u.spdata, src->u.lpdata[0], src->u.lpdata[1], src->delay);

        /* Convert DCS CMD structures */
        memset(&dsi_cmd, 0, sizeof(dsi_cmd));

        dsi_cmd.ctype = src->ctype;
        dsi_cmd.dt = src->dtype;

        if (dsi_cmd.ctype & MHB_CTYPE_LONG_FLAG) {
            /* long packet */
            dsi_cmd.u.lp.length = src->length;
            dsi_cmd.u.lp.data = src->u.lpdata;
        } else {
            /* short packet */
            dsi_cmd.u.sp.data = src->u.spdata;
        }

        lldbg("[dst] ct=%d, dt=%d, l=%d, sd=%04x ld=%08x %08x\n",
              dsi_cmd.ctype, dsi_cmd.dt, dsi_cmd.u.lp.length,
              dsi_cmd.u.sp.data, dsi_cmd.u.lp.data[0], dsi_cmd.u.lp.data[1]);

        count = dsi_read_cmd(cdsi->dev, &dsi_cmd, p, l);
        if (count < 0) {
            lldbg("ERROR: Failed to write command\n");
            ret = count;
            break;
        }

        p += count;
        l -= count;

        /* Wait, if requested */
        if (src->delay) {
            usleep(src->delay);
        }
    }

error:
    transaction->out_msg.hdr->addr = transaction->in_msg.hdr->addr;
    transaction->out_msg.hdr->type = MHB_TYPE_CDSI_READ_CMDS_RSP;
    transaction->out_msg.hdr->result =
        ret ? MHB_RESULT_UNKNOWN_ERROR : MHB_RESULT_SUCCESS;

    transaction->send_rsp = true;

    return 0;
}

static int mhb_handle_cdsi_write_cmds_req(struct mhb_transaction *transaction)
{
    int ret;

    uint8_t inst =
      (transaction->in_msg.hdr->addr & MHB_INSTANCE_MASK) >> MHB_INSTANCE_SHIFT;

    struct cdsi_block *cdsi = cdsi_from_inst(inst);
    if (!cdsi || !cdsi->dev) {
        ret = -EINVAL;
        goto error;
    }

    struct mhb_cdsi_write_cmds_req *req =
                  (struct mhb_cdsi_write_cmds_req *)transaction->in_msg.payload;

    if (transaction->in_msg.payload_length % sizeof(req->cmds[0]) != 0) {
        lldbg("ERROR: invalid size\n");
        ret = -EINVAL;
        goto error;
    }

    size_t n = transaction->in_msg.payload_length / sizeof(req->cmds[0]);
    size_t i;
    for (i = 0; i < n; i++) {
        struct mhb_cdsi_cmd *src = &req->cmds[i];
        lldbg("[src] i=%d, ct=%d, dt=%d, l=%d, sd=%04x, ld=%08x %08x, l=%d\n",
              i, src->ctype, src->dtype, src->length,
              src->u.spdata, src->u.lpdata[0], src->u.lpdata[1], src->delay);

        /* Convert DCS CMD structures */
        struct dsi_cmd dsi_cmd;
        memset(&dsi_cmd, 0, sizeof(dsi_cmd));

        dsi_cmd.ctype = src->ctype;
        dsi_cmd.dt = src->dtype;

        if (dsi_cmd.ctype & MHB_CTYPE_LONG_FLAG) {
            /* long packet */
            dsi_cmd.u.lp.length = src->length;
            dsi_cmd.u.lp.data = src->u.lpdata;
        } else {
            /* short packet */
            dsi_cmd.u.sp.data = src->u.spdata;
        }

        lldbg("[dst] i=%d, ct=%d, dt=%d, l=%d, sd=%04x ld=%08x %08x\n",
              i, dsi_cmd.ctype, dsi_cmd.dt, dsi_cmd.u.lp.length,
              dsi_cmd.u.sp.data, dsi_cmd.u.lp.data[0], dsi_cmd.u.lp.data[1]);

        ret = dsi_write_cmd(cdsi->dev, &dsi_cmd);
        if (ret) {
            lldbg("ERROR: Failed to write command\n");
            break;
        }

        /* Wait, if requested */
        if (src->delay) {
            usleep(src->delay);
        }
    }

error:
    transaction->out_msg.hdr->addr = transaction->in_msg.hdr->addr;
    transaction->out_msg.hdr->type = MHB_TYPE_CDSI_WRITE_CMDS_RSP;
    transaction->out_msg.hdr->result =
        ret ? MHB_RESULT_UNKNOWN_ERROR : MHB_RESULT_SUCCESS;

    transaction->send_rsp = true;
    return 0;
}

static int mhb_handle_cdsi_unconfig_req(struct mhb_transaction *transaction)
{
    int ret;

    uint8_t inst =
      (transaction->in_msg.hdr->addr & MHB_INSTANCE_MASK) >> MHB_INSTANCE_SHIFT;

    struct cdsi_block *cdsi = cdsi_from_inst(inst);
    if (cdsi && cdsi->dev) {
        cdsi_uninitialize(cdsi->dev);
        cdsi->dev = NULL;
        ret = 0;
    } else {
        ret = -ENODEV;
    }

#if CONFIG_ICE_IPC_CLIENT
    /* Cheat and re-use this transaction.  The unipro response (if any)
       will be in out_msg. */
    ret = mhb_unipro_send(transaction);
#endif

    transaction->out_msg.hdr->addr = transaction->in_msg.hdr->addr;
    transaction->out_msg.hdr->type = MHB_TYPE_CDSI_UNCONFIG_RSP;
    transaction->out_msg.hdr->result =
        ret ? MHB_RESULT_UNKNOWN_ERROR : MHB_RESULT_SUCCESS;
    transaction->send_rsp = true;

    return 0;
}

static int mhb_handle_cdsi(struct mhb_transaction *transaction)
{
    switch (transaction->in_msg.hdr->type) {
    case MHB_TYPE_CDSI_CONFIG_REQ:
        return mhb_handle_cdsi_config_req(transaction);
    case MHB_TYPE_CDSI_CONTROL_REQ:
        return mhb_handle_cdsi_control_req(transaction);
    case MHB_TYPE_CDSI_STATUS_REQ:
        return mhb_handle_cdsi_status_req(transaction);
    case MHB_TYPE_CDSI_READ_CMDS_REQ:
        return mhb_handle_cdsi_read_cmds_req(transaction);
    case MHB_TYPE_CDSI_WRITE_CMDS_REQ:
        return mhb_handle_cdsi_write_cmds_req(transaction);
    case MHB_TYPE_CDSI_UNCONFIG_REQ:
        return mhb_handle_cdsi_unconfig_req(transaction);
    default:
        lldbg("ERROR: unknown CDSI event\n");
        return -EINVAL;
    }
}

/* I2S */
static int mhb_handle_i2s_config_req(struct mhb_transaction *transaction)
{
    int ret;
    struct mhb_i2s_config_req *req =
        (struct mhb_i2s_config_req *)transaction->in_msg.payload;

    struct mhb_i2s_config *cfg = &req->cfg;

    cfg->sample_rate = le32_to_cpu(cfg->sample_rate);

    lldbg("reset cport=%d\n", I2S_INST_TO_CPORT(inst));
    unipro_p2p_reset_connection(I2S_INST_TO_CPORT(inst));
#if CONFIG_ICE_IPC_SERVER
    lldbg("setup cport=%d\n", I2S_INST_TO_CPORT(inst));
    unipro_p2p_setup_connection(I2S_INST_TO_CPORT(inst));
#endif

    // TODO: Implement
    ret = 0;

#if CONFIG_ICE_IPC_CLIENT
    if (!ret) {
        /* Cheat and re-use this transaction.  The unipro response (if any)
           will be in out_msg. */

        ret = mhb_unipro_send(transaction);
    }
#endif

    transaction->out_msg.hdr->addr = transaction->in_msg.hdr->addr;
    transaction->out_msg.hdr->type = MHB_TYPE_I2S_CONFIG_RSP;
    transaction->out_msg.hdr->result =
        ret ? MHB_RESULT_UNKNOWN_ERROR : MHB_RESULT_SUCCESS;
    transaction->send_rsp = true;

    return 0;
}

static int mhb_handle_i2s_control_req(struct mhb_transaction *transaction)
{
    int ret;

    // TODO: Implement
    ret = 0;

#if CONFIG_ICE_IPC_CLIENT
    struct mhb_i2s_control_req *req =
        (struct mhb_i2s_control_req *)transaction->in_msg.payload;

    if (!ret) {
        /* Cheat and re-use this transaction.  The unipro response (if any)
           will be in out_msg. */

        /* Move the peer command to local before sending. */
        if (req->peer_command != MHB_CDSI_COMMAND_NONE) {
            req->local_command = req->peer_command;
            req->peer_command = MHB_CDSI_COMMAND_NONE;
        }

        ret = mhb_unipro_send(transaction);
    }
#endif

    transaction->out_msg.hdr->addr = transaction->in_msg.hdr->addr;
    transaction->out_msg.hdr->type = MHB_TYPE_I2S_CONTROL_RSP;
    transaction->out_msg.hdr->result =
        ret ? MHB_RESULT_UNKNOWN_ERROR : MHB_RESULT_SUCCESS;
    transaction->send_rsp = true;
    return 0;
}

static int mhb_handle_i2s_status_req(struct mhb_transaction *transaction)
{
    // TODO: Implement

    transaction->out_msg.hdr->addr = transaction->in_msg.hdr->addr;
    transaction->out_msg.hdr->type = MHB_TYPE_I2S_STATUS_RSP;
    transaction->out_msg.hdr->result = MHB_RESULT_SUCCESS;
    transaction->send_rsp = true;

    return 0;
}

static int mhb_handle_i2s(struct mhb_transaction *transaction)
{
    switch (transaction->in_msg.hdr->type) {
    case MHB_TYPE_I2S_CONFIG_REQ:
        return mhb_handle_i2s_config_req(transaction);
    case MHB_TYPE_I2S_CONTROL_REQ:
        return mhb_handle_i2s_control_req(transaction);
    case MHB_TYPE_I2S_STATUS_REQ:
        return mhb_handle_i2s_status_req(transaction);
    default:
        lldbg("ERROR: unknown I2S event\n");
        return -EINVAL;
    }
}

/* HSIC */
static int mhb_handle_hsic(struct mhb_transaction *transaction)
{
    switch (transaction->in_msg.hdr->type) {
    default:
        lldbg("ERROR: unknown HSIC event\n");
        return -EINVAL;
    }
}

/* Diag */
static int _mhb_send_diag_local_log_not(struct device *dev)
{
    struct mhb_hdr hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_DIAG;
    hdr.type = MHB_TYPE_DIAG_LOG_NOT;

    return device_mhb_send(dev, &hdr, NULL, 0, 0);
}

int mhb_send_diag_local_log_not(void)
{
    return _mhb_send_diag_local_log_not(g_uart_dev);
}

static int mhb_handle_diag_log_req(struct mhb_transaction *transaction)
{
    transaction->out_msg.hdr->addr = MHB_ADDR_DIAG;
    transaction->out_msg.hdr->type = MHB_TYPE_DIAG_LOG_RSP;
    transaction->out_msg.hdr->result = MHB_RESULT_UNKNOWN_ERROR;
    transaction->send_rsp = true;

#ifdef CONFIG_RAMLOG_SYSLOG
    int fd;
    int ret;

    size_t n = MIN(transaction->out_msg.payload_max, CONFIG_RAMLOG_BUFSIZE);

    fd = open(CONFIG_SYSLOG_DEVPATH, O_RDONLY);
    if (fd < 0)
        goto done;

    ret = read(fd, transaction->out_msg.payload, n);
    if (ret < 0) {
        close(fd);
        goto done;
    }

    close(fd);

    transaction->out_msg.payload_length = ret;
    transaction->out_msg.hdr->result = MHB_RESULT_SUCCESS;

done:
#else
    transaction->out_msg.hdr->result = MHB_RESULT_NONEXISTENT;
#endif

    return 0;
}

static int mhb_handle_diag_last_req(struct mhb_transaction *transaction)
{
    transaction->out_msg.hdr->addr = MHB_ADDR_DIAG;
    transaction->out_msg.hdr->type = MHB_TYPE_DIAG_LAST_RSP;
    transaction->out_msg.hdr->result = MHB_RESULT_UNKNOWN_ERROR;
    transaction->send_rsp = true;

#ifdef CONFIG_RAMLOG_LAST_DMESG
    int fd;
    int ret;

    size_t n = MIN(transaction->out_msg.payload_max, CONFIG_RAMLOG_BUFSIZE);

    fd = open(CONFIG_SYSLOG_LAST_DEVPATH, O_RDONLY);
    if (fd < 0)
        goto done;

    ret = read(fd, transaction->out_msg.payload, n);
    if (ret < 0) {
        close(fd);
        goto done;
    }

    close(fd);

    transaction->out_msg.payload_length = ret;
    transaction->out_msg.hdr->result = MHB_RESULT_SUCCESS;

done:
#else
    transaction->out_msg.hdr->result = MHB_RESULT_NONEXISTENT;
#endif

    return 0;
}

static int mhb_handle_diag_mode_req(struct mhb_transaction *transaction)
{
    uint32_t mode;

    struct mhb_diag_mode_req *req =
        (struct mhb_diag_mode_req *)transaction->in_msg.payload;

    mode = le32_to_cpu(req->mode);
    svc_send_event(SVC_EVENT_TEST_MODE_STARTED, (void *)mode, 0, 0);

    transaction->out_msg.hdr->addr = MHB_ADDR_DIAG;
    transaction->out_msg.hdr->type = MHB_TYPE_DIAG_MODE_RSP;
    transaction->out_msg.hdr->result = MHB_RESULT_SUCCESS;
    transaction->send_rsp = true;
    return 0;
}

static int mhb_handle_diag_loop_req(struct mhb_transaction *transaction)
{
    transaction->out_msg.hdr->addr = MHB_ADDR_DIAG;
        transaction->out_msg.hdr->type = MHB_TYPE_DIAG_LOOP_RSP;
    transaction->out_msg.hdr->result = MHB_RESULT_SUCCESS;
    memcpy(transaction->out_msg.payload, transaction->in_msg.payload,
        transaction->in_msg.payload_length);
    transaction->out_msg.payload_length = transaction->in_msg.payload_length;
    transaction->send_rsp = true;
    return 0;
}

/* All */
static int mhb_handle_diag(struct mhb_transaction *transaction)
{
    switch (transaction->in_msg.hdr->type) {
    case MHB_TYPE_DIAG_LOG_REQ:
        return mhb_handle_diag_log_req(transaction);
    case MHB_TYPE_DIAG_LAST_REQ:
        return mhb_handle_diag_last_req(transaction);
    case MHB_TYPE_DIAG_MODE_REQ:
        return mhb_handle_diag_mode_req(transaction);
    case MHB_TYPE_DIAG_LOOP_REQ:
        return mhb_handle_diag_loop_req(transaction);
    default:
        lldbg("ERROR: unknown diag event\n");
        return -EINVAL;
    }
}

int mhb_handle_msg(struct mhb_transaction *transaction)
{
    uint8_t funcid =
        (transaction->in_msg.hdr->addr >> MHB_FUNC_SHIFT) & MHB_FUNC_BIT;

    switch (funcid) {
    case MHB_FUNC_UNIPRO:
        return mhb_handle_unipro(transaction);
    case MHB_FUNC_CDSI:
        return mhb_handle_cdsi(transaction);
    case MHB_FUNC_I2S:
        return mhb_handle_i2s(transaction);
    case MHB_FUNC_HSIC:
        return mhb_handle_hsic(transaction);
    case MHB_FUNC_DIAG:
        return mhb_handle_diag(transaction);
    default:
        lldbg("ERROR: unknown function\n");
        return -EINVAL;
    }
}

/* UART transport */
static int mhb_uart_handle_msg(struct device *dev, struct mhb_hdr *hdr,
               uint8_t *payload, size_t payload_length)
{
    static uint8_t out_buf[MHB_MAX_MSG_SIZE];

    int ret;
    struct mhb_transaction transaction;

    transaction.dev = dev;
    transaction.in_msg.hdr = hdr;
    transaction.in_msg.payload = payload;
    transaction.in_msg.payload_length = payload_length;
    transaction.out_msg.hdr = (struct mhb_hdr *)out_buf;
    transaction.out_msg.payload = out_buf + sizeof(*transaction.out_msg.hdr);
    transaction.out_msg.payload_length = 0;
    transaction.out_msg.payload_max =
        sizeof(out_buf) - sizeof(*transaction.out_msg.hdr);
    transaction.send_rsp = false;

    ret = mhb_handle_msg(&transaction);

    if (!ret && transaction.send_rsp) {
        ret = device_mhb_send(transaction.dev, transaction.out_msg.hdr,
            transaction.out_msg.payload, transaction.out_msg.payload_length, 0);
    }

    return ret;
}

int mhb_uart_init(void)
{
    g_uart_dev = device_open(DEVICE_TYPE_MHB, 0);

    device_mhb_register_receiver(g_uart_dev, MHB_ADDR_UNIPRO, mhb_uart_handle_msg);
    device_mhb_register_receiver(g_uart_dev, MHB_ADDR_CDSI0, mhb_uart_handle_msg);
    device_mhb_register_receiver(g_uart_dev, MHB_ADDR_CDSI1, mhb_uart_handle_msg);
    device_mhb_register_receiver(g_uart_dev, MHB_ADDR_I2S, mhb_uart_handle_msg);
    device_mhb_register_receiver(g_uart_dev, MHB_ADDR_HSIC, mhb_uart_handle_msg);
    device_mhb_register_receiver(g_uart_dev, MHB_ADDR_DIAG, mhb_uart_handle_msg);

    return 0;
}

/* UniPro transport */
#define IPC_APP_ID_MHB IPC_FOURCC('m', 'h', 'b', '0')

#if CONFIG_ICE_IPC_SERVER
static uint32_t mhb_unipro_handle_msg(void *in_data, uint32_t in_data_len,
                                      void **out_data, uint32_t *out_data_len)
{
    static uint8_t out_buf[MHB_MAX_MSG_SIZE];

    uint32_t ret;
    struct mhb_transaction transaction;

    if (!in_data_len || !in_data) {
        lldbg("ERROR: Invalid input.\n");
        return -EINVAL;
    }

    transaction.dev = NULL; /* No device for UniPro IPC */
    transaction.send_rsp = false;

    transaction.in_msg.hdr = (struct mhb_hdr*)in_data;
    transaction.in_msg.payload_length = in_data_len - sizeof(*transaction.in_msg.hdr);
    if (transaction.in_msg.payload_length > 0) {
        transaction.in_msg.payload =
            ((uint8_t *)in_data) + sizeof(*transaction.in_msg.hdr);
    } else {
        transaction.in_msg.payload = NULL;
    }

    /* Prepare a response in case it is needed. */
    transaction.out_msg.hdr = (struct mhb_hdr *)out_buf;
    transaction.out_msg.payload = out_buf + sizeof(*transaction.out_msg.hdr);
    transaction.out_msg.payload_length = 0;
    transaction.out_msg.payload_max =
        sizeof(out_buf) - sizeof(*transaction.out_msg.hdr);

    ret = mhb_handle_msg(&transaction);
    if (!ret && transaction.send_rsp) {
        *out_data = out_buf;
        *out_data_len =
          sizeof(*transaction.out_msg.hdr) + transaction.out_msg.payload_length;
    } else {
        *out_data = NULL;
        *out_data_len = 0;
    }

    return ret;
}
#endif

#if CONFIG_ICE_IPC_CLIENT

static uint8_t mhb_unipro_tx_buf[MHB_MAX_MSG_SIZE];

static int mhb_unipro_send(struct mhb_transaction *transaction)
{
    int ret;
    uint8_t *p = mhb_unipro_tx_buf;
    void *out_param = NULL;
    uint32_t out_len = 0;

    memcpy(p, transaction->in_msg.hdr, sizeof(*transaction->in_msg.hdr));
    memcpy(p + sizeof(*transaction->in_msg.hdr), transaction->in_msg.payload,
        transaction->in_msg.payload_length);

    ret = ipc_request_sync(IPC_APP_ID_MHB, p,
        sizeof(*transaction->in_msg.hdr) + transaction->in_msg.payload_length,
        &out_param, &out_len, MHB_IPC_TIMEOUT);

    if (out_param) {
        /* Save the response. */
        if (out_len <= sizeof(mhb_unipro_tx_buf) - sizeof(*transaction->out_msg.hdr)) {
            p = out_param;
            memcpy(transaction->out_msg.hdr, p, sizeof(*transaction->out_msg.hdr));
            memcpy(transaction->out_msg.payload, p + sizeof(*transaction->out_msg.hdr), out_len);
        } else {
            lldbg("ERROR: IPC response payload too big.\n");
        }

        release_response(out_param);
    }

    return ret;
}

#endif

int mhb_unipro_init(void)
{
#if CONFIG_ICE_IPC_SERVER
    ipc_init();
    register_ipc_handler(IPC_APP_ID_MHB, mhb_unipro_handle_msg, NULL);
#elif CONFIG_ICE_IPC_CLIENT
    ipc_init();
#endif

    return 0;
}
