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
#include <reglog.h>
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
#include <nuttx/device_gpio_tunnel.h>
#include <nuttx/irq.h>
#include <nuttx/util.h>
#include <nuttx/version.h>
#include <nuttx/wqueue.h>

#if defined(CONFIG_I2S_TUNNEL)
# include <nuttx/i2s_tunnel/i2s_unipro.h>
#endif
#include <nuttx/greybus/tsb_unipro.h>
#include <nuttx/mhb/device_mhb.h>
#include <nuttx/mhb/ipc.h>
#include <nuttx/mhb/mhb_protocol.h>
#include <nuttx/mhb/mhb_utils.h>

#ifdef CONFIG_RAMLOG_SYSLOG
# include <nuttx/syslog/ramlog.h>
#endif

#include <nuttx/unipro/unipro.h>

#ifdef CONFIG_MODS_USB_HCD_ROUTER
#include <nuttx/usbtun/usbtun_hcd_router.h>
#endif
#ifdef CONFIG_MODS_USB_PCD_ROUTER
#include <nuttx/usbtun/usbtun_pcd_router.h>
#endif

#ifdef CONFIG_MODS_HSIC_TEST
int hsic_test_init(void);
void hsic_test_uninit(void);
#endif

#include "sm.h"
#include "gearbox.h"

#define CDSI0_CPORT (16)
#define CDSI1_CPORT (17)

#define CDSI_MAX_INST (2)

#define CDSI_INST_TO_CPORT(x) (x + CDSI0_CPORT)
#define CDSI_CPORT_TO_INST(x) (x - CDSI0_CPORT)

#if defined(CONFIG_I2S_TUNNEL)
# define I2S_CPORT CONFIG_ARCH_CHIP_TSB_I2S_TUNNEL_CPORT_ID

# define I2S_INST_TO_CPORT(x) (I2S_CPORT)
# define I2S_CPORT_TO_INST(x) (I2S_CPORT)

# define MHB_I2S_TUNNEL_RX 0x01
# define MHB_I2S_TUNNEL_TX 0x02
#endif

#define MHB_IPC_TIMEOUT (500)

/* Practical max bps with USBTUN */
#define MHB_HSIC_SPEED_MBPS (200)

static struct device *g_uart_dev;
static struct device *g_gpio_tunnel_dev;
static struct work_s g_mhb_server_wq;

static struct gearbox *g_gearbox;
static int g_gearbox_cdsi0;
static int g_gearbox_cdsi1;
static int g_gearbox_hsic;
static int g_gearbox_i2s;
static int g_gearbox_diag;

struct mhb_unipro_stats g_unipro_stats;

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

#if CONFIG_MHB_IPC_CLIENT
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

static void _mhb_unipro_gear_to_gearbox(struct mhb_unipro_gear *mhb_req,
                                        struct gear_request *gearbox_req)
{
    switch ((mhb_req->pwrmode >> 4) & 0xf) {
    case UNIPRO_FASTAUTO_MODE:
    case UNIPRO_FAST_MODE:
        if (mhb_req->series == 0) {
            /* Series A */
            gearbox_req->rx_max_mbps = mhb_req->rx * 1248 * 0.5;
        } else {
            /* Series B */
            gearbox_req->rx_max_mbps = mhb_req->rx * 1458 * 0.5;
        }
        break;
    case UNIPRO_SLOWAUTO_MODE:
    case UNIPRO_SLOW_MODE:
        switch (mhb_req->rx) {
        case 1:
            gearbox_req->rx_max_mbps = 9* 0.5;
            break;
        case 2:
            gearbox_req->rx_max_mbps = 18* 0.5;
            break;
        case 3:
            gearbox_req->rx_max_mbps = 36* 0.5;
            break;
        case 4:
            gearbox_req->rx_max_mbps = 72* 0.5;
            break;
        }
        break;
    }

    switch ((mhb_req->pwrmode >> 4) & 0xf) {
    case UNIPRO_FASTAUTO_MODE:
    case UNIPRO_SLOWAUTO_MODE:
        gearbox_req->rx_auto = 1;
        break;
    case UNIPRO_FAST_MODE:
    case UNIPRO_SLOW_MODE:
        gearbox_req->rx_auto = 0;
    }

    switch (mhb_req->pwrmode & 0xf) {
    case UNIPRO_FASTAUTO_MODE:
    case UNIPRO_FAST_MODE:
        if (mhb_req->series == 0) {
            /* Series A */
            gearbox_req->tx_max_mbps = mhb_req->tx * 1248 * 0.5;
        } else {
            /* Series B */
            gearbox_req->tx_max_mbps = mhb_req->tx * 1458 * 0.5;
        }
        break;
    case UNIPRO_SLOWAUTO_MODE:
    case UNIPRO_SLOW_MODE:
        switch (mhb_req->tx) {
        case 1:
            gearbox_req->tx_max_mbps = 9 * 0.5;
            break;
        case 2:
            gearbox_req->tx_max_mbps = 18 * 0.5;
            break;
        case 3:
            gearbox_req->tx_max_mbps = 36 * 0.5;
            break;
        case 4:
            gearbox_req->tx_max_mbps = 72 * 0.5;
            break;
        }
        break;
    }

    switch (mhb_req->pwrmode & 0xf) {
    case UNIPRO_FASTAUTO_MODE:
    case UNIPRO_SLOWAUTO_MODE:
        gearbox_req->tx_auto = 1;
        break;
    case UNIPRO_FAST_MODE:
    case UNIPRO_SLOW_MODE:
        gearbox_req->tx_auto = 0;
    }
}

static int mhb_handle_unipro_control_req(struct mhb_transaction *transaction)
{
    struct mhb_unipro_control_req *req =
        (struct mhb_unipro_control_req *)transaction->in_msg.payload;

    if (transaction->in_msg.payload_length != sizeof(*req)) {
        return -EINVAL;
    }

    if (g_gearbox) {
        struct gear_request request;
        _mhb_unipro_gear_to_gearbox(&req->gear, &request);
        gearbox_request(g_gearbox, g_gearbox_diag, &request);
    } else {
        uint8_t termination = 1;
        unipro_powermode_change(req->gear.tx, req->gear.rx, req->gear.pwrmode,
            req->gear.series, termination);
    }

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

static int _mhb_copy_unipro_stats(struct mhb_unipro_stats *stats)
{
    struct mhb_unipro_stats *unipro_stats = &g_unipro_stats;

    irqstate_t flags;
    flags = irqsave();

    stats->phy_lane_err =
        cpu_to_le32(unipro_stats->phy_lane_err);
    stats->pa_lane_reset_tx =
        cpu_to_le32(unipro_stats->pa_lane_reset_tx);
    stats->pa_lane_reset_rx =
        cpu_to_le32(unipro_stats->pa_lane_reset_rx);
    stats->d_nac_received =
        cpu_to_le32(unipro_stats->d_nac_received);
    stats->d_tcx_replay_timer_expired =
        cpu_to_le32(unipro_stats->d_tcx_replay_timer_expired);
    stats->d_afcx_request_timer_expired =
        cpu_to_le32(unipro_stats->d_afcx_request_timer_expired);
    stats->d_fcx_protectiong_timer_expired =
        cpu_to_le32(unipro_stats->d_fcx_protectiong_timer_expired);
    stats->d_crc_error =
        cpu_to_le32(unipro_stats->d_crc_error);
    stats->d_rx_buffer_overflow =
        cpu_to_le32(unipro_stats->d_rx_buffer_overflow);
    stats->d_max_frame_length_exceeded =
        cpu_to_le32(unipro_stats->d_max_frame_length_exceeded);
    stats->d_wrong_sequence_number =
        cpu_to_le32(unipro_stats->d_wrong_sequence_number);
    stats->d_afc_frame_syntax_error =
        cpu_to_le32(unipro_stats->d_afc_frame_syntax_error);
    stats->d_nac_frame_syntax_error =
        cpu_to_le32(unipro_stats->d_nac_frame_syntax_error);
    stats->d_eof_syntax_error =
        cpu_to_le32(unipro_stats->d_eof_syntax_error);
    stats->d_frame_syntax_error =
        cpu_to_le32(unipro_stats->d_frame_syntax_error);
    stats->d_bad_control_symbol_type =
        cpu_to_le32(unipro_stats->d_bad_control_symbol_type);
    stats->d_pa_init_error =
        cpu_to_le32(unipro_stats->d_pa_init_error);
    stats->d_pa_error_ind_received =
        cpu_to_le32(unipro_stats->d_pa_error_ind_received);
    stats->n_unsupported_header_type =
        cpu_to_le32(unipro_stats->n_unsupported_header_type);
    stats->n_bad_device_id_encoding =
        cpu_to_le32(unipro_stats->n_bad_device_id_encoding);
    stats->n_lhdr_trap_packet_dropping =
        cpu_to_le32(unipro_stats->n_lhdr_trap_packet_dropping);
    stats->t_unsupported_header_type =
        cpu_to_le32(unipro_stats->t_unsupported_header_type);
    stats->t_unknown_cport_id =
        cpu_to_le32(unipro_stats->t_unknown_cport_id);
    stats->t_no_connection_rx =
        cpu_to_le32(unipro_stats->t_no_connection_rx);
    stats->t_controlled_segment_dropping =
        cpu_to_le32(unipro_stats->t_controlled_segment_dropping);
    stats->t_bad_tc =
        cpu_to_le32(unipro_stats->t_bad_tc);
    stats->t_e2e_credit_overflow =
        cpu_to_le32(unipro_stats->t_e2e_credit_overflow);
    stats->t_safety_valve_dropping =
        cpu_to_le32(unipro_stats->t_safety_valve_dropping);

    irqrestore(flags);

    return 0;
}

static int _mhb_clear_unipro_stats(void)
{
    irqstate_t flags;
    flags = irqsave();

    memset(&g_unipro_stats, 0, sizeof(g_unipro_stats));

    irqrestore(flags);

    return 0;
}

static int _mhb_send_unipro_stats_not(struct device *dev)
{
    struct mhb_unipro_stats_not not;

    int ret = _mhb_copy_unipro_stats(&not.stats);
    if (ret) {
        return ret;
    }

    struct mhb_hdr hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_UNIPRO;
    hdr.type = MHB_TYPE_UNIPRO_STATS_NOT;

    ret =  device_mhb_send(dev, &hdr, (uint8_t *)&not, sizeof(not), 0);
    if (!ret) {
        ret = _mhb_clear_unipro_stats();
    }

    return ret;
}

int mhb_send_unipro_stats_not(void)
{
    return _mhb_send_unipro_stats_not(g_uart_dev);
}

static int mhb_handle_unipro_stats_req(struct mhb_transaction *transaction)
{
    int ret;

    if (transaction->in_msg.payload_length != 0) {
        return -EINVAL;
    }

    struct mhb_unipro_stats_rsp *rsp =
        (struct mhb_unipro_stats_rsp *)transaction->out_msg.payload;

    ret = _mhb_copy_unipro_stats(&rsp->stats);
    if (!ret) {
        ret = _mhb_clear_unipro_stats();
    }

    transaction->out_msg.hdr->addr = MHB_ADDR_UNIPRO;
    transaction->out_msg.hdr->type = MHB_TYPE_UNIPRO_STATS_RSP;
    transaction->out_msg.hdr->result =
        ret ? MHB_RESULT_UNKNOWN_ERROR : MHB_RESULT_SUCCESS;
    transaction->out_msg.payload_length = sizeof(*rsp);
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
    case MHB_TYPE_UNIPRO_STATS_REQ:
        return mhb_handle_unipro_stats_req(transaction);
    default:
        dbg("ERROR: unknown Unipro event\n");
        return -EINVAL;
    }
}

/* CDSI */
static inline struct cdsi_block *cdsi_from_inst(uint8_t inst)
{
    if (inst >= ARRAY_SIZE(g_cdsi_blocks)) {
        dbg("ERROR: Invalid CDSI instance.\n");
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
    /* CDSI already in use. */
    if (cdsi && cdsi->dev) {
        ret = -EBUSY;
    }

    cfg = &req->cfg;

    if (!ret) {
        cdsi->config.direction = cfg->direction ? TSB_CDSI_TX : TSB_CDSI_RX;
        cdsi->config.mode = cfg->mode != 0;
        cdsi->config.rx_num_lanes = cfg->rx_num_lanes;
        cdsi->config.tx_num_lanes = cfg->tx_num_lanes;
        cdsi->config.rx_bits_per_lane = le32_to_cpu(cfg->rx_bits_per_lane);
        cdsi->config.tx_bits_per_lane = le32_to_cpu(cfg->tx_bits_per_lane);
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
            dbg("ERROR: Failed to open CDSI device.\n");
            ret = -EINVAL;
        }
    }

    if (!ret) {
        if (cdsi->config.direction == TSB_CDSI_RX) {
            ret = cdsi_initialize_rx(cdsi->dev, &cdsi->config);
        } else if (cdsi->config.direction == TSB_CDSI_TX) {
            ret = cdsi_initialize_tx(cdsi->dev, &cdsi->config);
        } else {
            dbg("ERROR: Invalid direction.\n");
            ret = -EINVAL;
            goto error;
        }
    }

    dbg("reset cport=%d\n", CDSI_INST_TO_CPORT(inst));
    unipro_p2p_reset_connection(CDSI_INST_TO_CPORT(inst));
#if CONFIG_MHB_IPC_SERVER
    dbg("setup cport=%d\n", CDSI_INST_TO_CPORT(inst));
    unipro_p2p_setup_connection(CDSI_INST_TO_CPORT(inst));
#endif

    bool use_te = (cfg->mode == 0) && (cfg->video_mode == 0) && (cfg->vsync_mode == 1);
    if (use_te) {
        g_gpio_tunnel_dev = device_open(DEVICE_TYPE_GPIO_TUNNEL_HW, 0);
    }

#if CONFIG_MHB_IPC_CLIENT
    if (!ret) {
        /* Cheat and re-use this transaction.  The unipro response (if any)
           will be in out_msg. */

        /* Swap the CDSI direction for the peer. */
        req->cfg.direction =
            (req->cfg.direction == TSB_CDSI_RX) ? TSB_CDSI_TX : TSB_CDSI_RX;

        dbg("notify peer: direction: %d\n", req->cfg.direction);
        ret = mhb_unipro_send(transaction);
        if (ret) {
            goto error;
        }
    }
#endif

    /* If gearbox is present, initiate a gear shift request. */
    if (g_gearbox && !ret) {
        const int id = (inst == 0) ? g_gearbox_cdsi0 : g_gearbox_cdsi1;

        struct gear_request request;
        const int _auto = cdsi->config.mode == TSB_CDSI_MODE_DSI && !cdsi->config.video_mode;

        /* TSB_CDSI_RX, rx_bits_per_lane, rx_num_lanes, tx_bits_per_lane, and
         * tx_num_lanes are all from the perspective of the CDSI interface.
         *
         * While rx_max_mbps, rx_auto, tx_max_mbps, and tx_auto are from the
         * perspective of UniPro.
         *
         * Furthermore, the gear shift is initiated by the APBE so CDSI and
         * UniPro are from the APBE's perspective.
         *
         * Therefore:
         *   Display on CDSI0: CDSI TX and UniPro RX
         *   Camera on CDSI1: CDSI RX and UniPro TX
         *
         */
        if (cdsi->config.direction == TSB_CDSI_TX) {
            /* Display on CDSI0 */
            request.rx_max_mbps = cdsi->config.tx_bits_per_lane / 1000000 * cdsi->config.tx_num_lanes;
            request.rx_auto = _auto;
            request.tx_max_mbps = GEARBOX_HS_MIN;
            request.tx_auto = _auto;
        } else {
            /* Camera on CDSI1 */
            request.rx_max_mbps = GEARBOX_HS_MIN;
            request.rx_auto = _auto;
            request.tx_max_mbps = cdsi->config.rx_bits_per_lane / 1000000 * cdsi->config.rx_num_lanes;
            request.tx_auto = _auto;
        }

        /* Apply additional factor to max_bps for DSI */
        if (cdsi->config.mode == TSB_CDSI_MODE_DSI) {
            if (request.rx_max_mbps != GEARBOX_HS_MIN)
                request.rx_max_mbps = request.rx_max_mbps / 4 * 5;

            if (request.tx_max_mbps != GEARBOX_HS_MIN)
                request.tx_max_mbps = request.tx_max_mbps / 4 * 5;
        }

        gearbox_request(g_gearbox, id, &request);
    }

done:
    transaction->out_msg.hdr->addr = transaction->in_msg.hdr->addr;
    transaction->out_msg.hdr->type = MHB_TYPE_CDSI_CONFIG_RSP;
    transaction->out_msg.hdr->result =
        ret ? MHB_RESULT_UNKNOWN_ERROR : MHB_RESULT_SUCCESS;
    transaction->send_rsp = true;

    return 0;

error:
    dbg("ERROR: Failed to configure.\n");
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
                    dbg("ERROR: invalid CDSI command.\n");
                    ret = -EINVAL;
                    break;
            }
        } else {
            switch (req->command) {
                case MHB_CDSI_COMMAND_START:
                    ret = cdsi_rx_start(cdsi->dev);
                    break;
                default:
                    /* Ignore commands other than start. */
                    break;
            }
        }
    }

#if CONFIG_MHB_IPC_CLIENT
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
        dbg("ERROR: invalid size\n");
        ret = -EINVAL;
        goto error;
    }

    /* Stop the TX first since it uses the same HW resources. */
    cdsi_tx_stop(cdsi->dev);

    n = transaction->in_msg.payload_length / sizeof(req->cmds[0]);
    p = transaction->out_msg.payload;
    l = transaction->out_msg.payload_length;
    transaction->out_msg.payload_length = 0;

    for (i = 0; i < n; i++) {
        struct dsi_cmd dsi_cmd;
        struct mhb_cdsi_cmd *src = &req->cmds[i];

        vdbg("[src] i=%d, ct=%d, dt=%d, l=%d, sd=%04x, ld=%08x %08x, d=%d\n",
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

        vdbg("[dst] ct=%d, dt=%d, l=%d, sd=%04x ld=%08x %08x\n",
              dsi_cmd.ctype, dsi_cmd.dt, dsi_cmd.u.lp.length,
              dsi_cmd.u.sp.data, dsi_cmd.u.lp.data[0], dsi_cmd.u.lp.data[1]);

        count = dsi_read_cmd(cdsi->dev, &dsi_cmd, p, src->length);
        if (count < 0) {
            dbg("ERROR: Failed to read command\n");
            ret = count;
            break;
        }

        p += count;
        l -= count;
        transaction->out_msg.payload_length += count;

        /* Wait, if requested */
        if (src->delay) {
            usleep(src->delay);
        }
    }

    ret = 0;

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
        dbg("ERROR: invalid size\n");
        ret = -EINVAL;
        goto error;
    }

    size_t n = transaction->in_msg.payload_length / sizeof(req->cmds[0]);
    size_t i;
    for (i = 0; i < n; i++) {
        struct mhb_cdsi_cmd *src = &req->cmds[i];
        vdbg("[src] i=%d, ct=%d, dt=%d, l=%d, sd=%04x, ld=%08x %08x, l=%d\n",
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

        vdbg("[dst] i=%d, ct=%d, dt=%d, l=%d, sd=%04x ld=%08x %08x\n",
              i, dsi_cmd.ctype, dsi_cmd.dt, dsi_cmd.u.lp.length,
              dsi_cmd.u.sp.data, dsi_cmd.u.lp.data[0], dsi_cmd.u.lp.data[1]);

        ret = dsi_write_cmd(cdsi->dev, &dsi_cmd);
        if (ret) {
            dbg("ERROR: Failed to write command\n");
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
        if (cdsi->config.direction == TSB_CDSI_TX) {
            cdsi_uninitialize_tx(cdsi->dev, &cdsi->config);
        } else {
            cdsi_uninitialize_rx(cdsi->dev, &cdsi->config);
        }

        cdsi_uninitialize(cdsi->dev);
        cdsi->dev = NULL;
        ret = 0;
    } else {
        ret = -ENODEV;
    }

    if (g_gpio_tunnel_dev) {
        device_close(g_gpio_tunnel_dev);
        g_gpio_tunnel_dev = NULL;
    }

#if CONFIG_MHB_IPC_CLIENT
    /* Cheat and re-use this transaction.  The unipro response (if any)
       will be in out_msg. */
    ret = mhb_unipro_send(transaction);
#endif

    /* If gearbox is present, release the current gear request. */
    if (g_gearbox) {
        const int id = (inst == 0) ? g_gearbox_cdsi0 : g_gearbox_cdsi1;
        gearbox_request(g_gearbox, id, NULL);
    }

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
        dbg("ERROR: unknown CDSI event\n");
        return -EINVAL;
    }
}

/* I2S */
static int mhb_handle_i2s_config_req(struct mhb_transaction *transaction)
{
#if defined(CONFIG_I2S_TUNNEL)
    int ret;
    struct mhb_i2s_config_req *req =
        (struct mhb_i2s_config_req *)transaction->in_msg.payload;
    struct mhb_i2s_config *cfg = &req->cfg;
    uint8_t flags;
    static const I2S_TUNNEL_I2S_MODE_T mhb_to_tunnel_protocol_tb[] =
    {
        I2S_TUNNEL_I2S_MODE_I2S_STEREO, /* MHB_I2S_PROTOCOL_I2S       */
        I2S_TUNNEL_I2S_MODE_PCM_MONO,   /* MHB_I2S_PROTOCOL_PCM       */
        I2S_TUNNEL_I2S_MODE_LR_STEREO   /* MHB_I2S_PROTOCOL_LR_STEREO */
    };

    ret = i2s_unipro_tunnel_enable(true);
    if (ret == 0) {
        FROM_LE(cfg->sample_rate);
        flags  = (cfg->tx_edge == MHB_I2S_EDGE_RISING ? I2S_TUNNEL_I2S_FLAGS_TX_EDGE_RISING :
                                                        I2S_TUNNEL_I2S_FLAGS_TX_EDGE_FALLING);
        flags |= (cfg->rx_edge == MHB_I2S_EDGE_RISING ? I2S_TUNNEL_I2S_FLAGS_RX_EDGE_RISING :
                                                        I2S_TUNNEL_I2S_FLAGS_RX_EDGE_FALLING);
        flags |= (cfg->wclk_edge == MHB_I2S_EDGE_RISING ? I2S_TUNNEL_I2S_FLAGS_LR_EDGE_RISING :
                                                          I2S_TUNNEL_I2S_FLAGS_LR_EDGE_FALLING);
        flags |= (cfg->clk_role == MHB_I2S_ROLE_MASTER ? I2S_TUNNEL_I2S_FLAGS_SLAVE :
                                                         I2S_TUNNEL_I2S_FLAGS_MASTER);

        if (cfg->protocol >= ARRAY_SIZE(mhb_to_tunnel_protocol_tb)) {
            dbg("Warning: I2S Protocol %d not supported defaulting to I2S Stereo.\n");
            cfg->protocol = I2S_TUNNEL_I2S_MODE_I2S_STEREO;
        }
        ret = i2s_unipro_tunnel_i2s_config(cfg->sample_rate,
                                           mhb_to_tunnel_protocol_tb[cfg->protocol],
                                           cfg->sample_size,
                                           flags);
    }

# if CONFIG_MHB_IPC_CLIENT
    if (!ret) {
        /* Cheat and re-use this transaction.  The unipro response (if any)
           will be in out_msg. */

        /* Swap the I2S direction for the peer if not in loopback mode. */
#  if !defined(CONFIG_I2S_TUNNEL_ROUNDTRIP_LOOPBACK)
        req->cfg.clk_role = (cfg->clk_role == MHB_I2S_ROLE_MASTER ? MHB_I2S_ROLE_SLAVE :
                                                                    MHB_I2S_ROLE_MASTER);
#  endif

        ret = mhb_unipro_send(transaction);
    }
# endif
    i2s_unipro_tunnel_arm(true);
    transaction->out_msg.hdr->addr = MHB_ADDR_I2S;
    transaction->out_msg.hdr->type = MHB_TYPE_I2S_CONFIG_RSP;
    transaction->out_msg.hdr->result =
        ret ? MHB_RESULT_UNKNOWN_ERROR : MHB_RESULT_SUCCESS;
    transaction->send_rsp = true;
#else
    transaction->out_msg.hdr->result = MHB_RESULT_NONEXISTENT;
#endif
    return 0;
}

#if defined(CONFIG_I2S_TUNNEL)
/*
 * Handle starting and stopping i2s tunneling.  A work queue is used to allow
 * both sides to stop as close as possible to one another.
 */
static void mhb_i2s_wq_disarm_disable(void *arg)
{
    int ret;

    ret = i2s_unipro_tunnel_arm(false);
    if (ret == OK) {
        ret = i2s_unipro_tunnel_enable(false);
    }
    if (ret != OK) {
        lldbg("Error: %d when attempting to disableing i2s tunneling.\n", ret);
    }
}

static int mhb_i2s_queue_disarm_disable(void)
{
    return work_queue(HPWORK, &g_mhb_server_wq, mhb_i2s_wq_disarm_disable, NULL, 0);
}

static int mhb_i2s_tunnel_enable_disable(uint8_t status, bool enable)
{
    int ret = OK;

    /*
     * Update the arm and enable functions if status is a 0 and enable is false
     * or status is non zero and enable is true.
     */
    if (((status == 0) ^ (enable == false)) == 0)
    {
        /*
         * When enabling enable, then arm.
         * When disabling disarm, then disable.
         */
        if (enable) {
            ret = i2s_unipro_tunnel_enable(enable);
            if (ret == OK) {
                ret = i2s_unipro_tunnel_arm(enable);
            }
        } else {
            ret = mhb_i2s_queue_disarm_disable();
        }
    }
    return ret;
}
#endif

static int mhb_handle_i2s_control_req(struct mhb_transaction *transaction)
{
#if defined(CONFIG_I2S_TUNNEL)
    static uint8_t tx_rx_status = 0;

    int ret = OK;
    struct mhb_i2s_control_req *req =
        (struct mhb_i2s_control_req *)transaction->in_msg.payload;

# if CONFIG_MHB_IPC_CLIENT
    /*
     * Send the same request to the APBA as soon as possible.   This is done to
     * keep the two sides as in sync as possible.  In this case the local command
     * and peer command are the same.  Please note, the Trigger command must only
     * go to the APBE.  It will handle the timing of starting the APBA in sync with
     * the APBE.
     */
    if (req->command != MHB_I2S_COMMAND_TRIGGER)
    {
        (void)mhb_unipro_send(transaction);
    }
# endif
    switch (req->command) {
        case MHB_I2S_COMMAND_RX_STOP:
            tx_rx_status &= ~MHB_I2S_TUNNEL_RX;
            ret = mhb_i2s_tunnel_enable_disable(tx_rx_status, false);
            break;
        case MHB_I2S_COMMAND_TX_STOP:
            tx_rx_status &= ~MHB_I2S_TUNNEL_TX;
            ret = mhb_i2s_tunnel_enable_disable(tx_rx_status, false);
            break;
        case MHB_I2S_COMMAND_RX_START:
            tx_rx_status |= MHB_I2S_TUNNEL_RX;
            break;
        case MHB_I2S_COMMAND_TX_START:
            tx_rx_status |= MHB_I2S_TUNNEL_TX;
            break;
        case MHB_I2S_COMMAND_ENABLE:
            ret = i2s_unipro_tunnel_enable(true);
            break;
        case MHB_I2S_COMMAND_DISABLE:
            ret = i2s_unipro_tunnel_enable(false);
            break;
        case MHB_I2S_COMMAND_ARM:
            ret = i2s_unipro_tunnel_arm(true);
            break;
        case MHB_I2S_COMMAND_DISARM:
            ret = i2s_unipro_tunnel_arm(false);
            break;
        case MHB_I2S_COMMAND_TRIGGER:
            i2s_unipro_tunnel_start(true);
            break;
        default:
            ret = -ENODEV;
            break;
    }
    transaction->send_rsp = true;
    transaction->out_msg.hdr->addr = MHB_ADDR_I2S;
    transaction->out_msg.hdr->type = MHB_TYPE_I2S_CONTROL_RSP;
    transaction->out_msg.hdr->result =
        ret ? MHB_RESULT_UNKNOWN_ERROR : MHB_RESULT_SUCCESS;
#else
    transaction->out_msg.hdr->result = MHB_RESULT_NONEXISTENT;
#endif
    return 0;
}

static int mhb_handle_i2s_status_req(struct mhb_transaction *transaction)
{
#if defined(CONFIG_I2S_TUNNEL)
    struct i2s_tunnel_info_s *info;
    int ret;

    /* Allow space for both the APBA and ABPE data. */
    transaction->out_msg.payload_length = sizeof(struct i2s_tunnel_info_s)*2;
    if (transaction->out_msg.payload_length < transaction->out_msg.payload_max) {
# if CONFIG_MHB_IPC_CLIENT
        /* Forward the request on to the APBA. */
        mhb_unipro_send(transaction);

        /* Place the APBE data after the APBA data. */
        info = (struct i2s_tunnel_info_s *)&transaction->out_msg.payload[sizeof(struct i2s_tunnel_info_s)];
# else
        /* Place the APBA data before the APBE data. */
        info = (struct i2s_tunnel_info_s *)&transaction->out_msg.payload[0];
# endif
        ret = i2s_unipro_tunnel_info(info);

        if (ret == OK) {
            TO_LE(info->enabled);
            TO_LE(info->is_master);
            TO_LE(info->bclk_rate);
            TO_LE(info->bytes_per_sample);
            TO_LE(info->roundtrip_timer_ticks);
            TO_LE(info->timer_offset);
            TO_LE(info->i2s_tunnel_rx_packets);
            TO_LE(info->i2s_tunnel_packet_size);
            TO_LE(info->i2s_tx_samples_dropped);
            TO_LE(info->i2s_tx_samples_retransmitted);
            TO_LE(info->i2s_tx_buffers_dropped);

            transaction->out_msg.hdr->addr = MHB_ADDR_I2S;
            transaction->out_msg.hdr->type = MHB_TYPE_I2S_STATUS_RSP;
            transaction->out_msg.hdr->result = MHB_RESULT_SUCCESS;
            transaction->send_rsp = true;
            return 0;
        }
    }
    transaction->out_msg.hdr->result = MHB_RESULT_NO_MEMORY;
#else
    transaction->out_msg.hdr->result = MHB_RESULT_NONEXISTENT;
#endif
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
        dbg("ERROR: unknown I2S event\n");
        return -EINVAL;
    }
}

/* HSIC */
static int mhb_handle_hsic_control_req(struct mhb_transaction *transaction)
{
    int ret = 0;

    struct mhb_hsic_control_req *req =
        (struct mhb_hsic_control_req *)transaction->in_msg.payload;

    switch (req->command) {
#ifdef CONFIG_MODS_USB_HCD_ROUTER
    case MHB_HSIC_COMMAND_START:
        ret = usbtun_hcd_router_init();
        break;
    case MHB_HSIC_COMMAND_STOP:
        usbtun_hcd_router_uninit();
        ret = 0;
        break;
#endif
#ifdef CONFIG_MODS_USB_PCD_ROUTER
    case MHB_HSIC_COMMAND_START:
        ret = usbtun_pcd_router_init();
        break;
    case MHB_HSIC_COMMAND_STOP:
        usbtun_pcd_router_uninit();
        ret = 0;
        break;
#endif
#ifdef CONFIG_MODS_HSIC_TEST
    case MHB_HSIC_COMMAND_START:
        ret = hsic_test_init();
        break;
    case MHB_HSIC_COMMAND_STOP:
        hsic_test_uninit();
        break;
#endif
    default:
        dbg("ERROR: HSIC event %d not handled\n", req->command);
        ret = -EINVAL;
        goto snd_resp;
    }

#ifdef CONFIG_MODS_USB_HCD_ROUTER
    /* Configure APBA side */
    ret = mhb_unipro_send(transaction);

    if (ret) {
        dbg("ERROR: Control transaction failure at remote.\n");
        goto snd_resp;
    }
#endif

    if (g_gearbox) {
        if (req->command == MHB_HSIC_COMMAND_START) {
            struct gear_request request;
            request.rx_max_mbps = MHB_HSIC_SPEED_MBPS;
            request.rx_auto = true;
            request.tx_max_mbps = MHB_HSIC_SPEED_MBPS;
            request.tx_auto = true;
            gearbox_request(g_gearbox, g_gearbox_hsic, &request);
        } else {
            gearbox_request(g_gearbox, g_gearbox_hsic, NULL);
        }
    }

snd_resp:
    transaction->out_msg.hdr->addr = transaction->in_msg.hdr->addr;
    transaction->out_msg.hdr->type = MHB_TYPE_HSIC_CONTROL_RSP;
    transaction->out_msg.hdr->result =
        ret ? MHB_RESULT_UNKNOWN_ERROR : MHB_RESULT_SUCCESS;
    transaction->send_rsp = true;

    return 0;
}

static int mhb_handle_hsic(struct mhb_transaction *transaction)
{
    switch (transaction->in_msg.hdr->type) {
    case MHB_TYPE_HSIC_CONTROL_REQ:
        return mhb_handle_hsic_control_req(transaction);
    default:
        dbg("ERROR: unknown HSIC event\n");
        return -EINVAL;
    }
}

static int _mhb_send_hsic_status_not(struct device *dev, bool attached)
{
    struct mhb_hdr hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_HSIC;
    hdr.type = MHB_TYPE_HSIC_STATUS_NOT;
    uint8_t payload = attached ? 1 : 0;

    return device_mhb_send(dev, &hdr, &payload, sizeof(payload), 0);
}

int mhb_send_hsic_status_not(bool attached)
{
    return _mhb_send_hsic_status_not(g_uart_dev, attached);
}

/* Diag */
#if CONFIG_MHB_IPC_SERVER
static int _mhb_send_diag_log_not_uart(struct device *dev, const char *buffer, size_t length)
{
    struct mhb_hdr hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_DIAG;
    hdr.type = MHB_TYPE_DIAG_LOG_NOT;
    hdr.result = MHB_RESULT_SUCCESS;

    return device_mhb_send(dev, &hdr, (const uint8_t *)buffer, length, 0);
}
#endif

#if CONFIG_MHB_IPC_CLIENT
static int _mhb_send_diag_log_not_unipro(struct device *dev, const char *buffer, size_t length)
{
    struct mhb_hdr hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_DIAG;
    hdr.type = MHB_TYPE_DIAG_LOG_NOT;
    hdr.result = MHB_RESULT_SUCCESS;

    struct mhb_transaction transaction;
    memset(&transaction, 0, sizeof(transaction));

    transaction.dev = dev;
    transaction.in_msg.hdr = &hdr;
    transaction.in_msg.payload = (uint8_t *)buffer;
    transaction.in_msg.payload_length = MIN(length, ipc_get_sync_data_sz() - MHB_HDR_SIZE - MHB_CRC_SIZE);
    transaction.out_msg.hdr = NULL;
    transaction.out_msg.payload = NULL;
    transaction.out_msg.payload_length = 0;
    transaction.out_msg.payload_max = 0;
    transaction.send_rsp = false;

    return mhb_unipro_send(&transaction);
}
#endif

int mhb_send_diag_log_not(const char *buffer, size_t length)
{
#if CONFIG_MHB_IPC_SERVER
    return _mhb_send_diag_log_not_uart(g_uart_dev, buffer, length);
#elif CONFIG_MHB_IPC_CLIENT
    return _mhb_send_diag_log_not_unipro(NULL /* dev */, buffer, length);
#endif
}

static int _mhb_handle_local_diag_log_req(struct mhb_transaction *transaction)
{
    transaction->out_msg.hdr->addr = MHB_ADDR_DIAG;
    transaction->out_msg.hdr->type = MHB_TYPE_DIAG_LOG_RSP;
    transaction->out_msg.hdr->result = MHB_RESULT_UNKNOWN_ERROR;
    transaction->send_rsp = true;

#ifdef CONFIG_RAMLOG_SYSLOG
    int fd;
    int ret;

    size_t n = MIN(transaction->out_msg.payload_max - 1 , CONFIG_RAMLOG_BUFSIZE);

    fd = open(CONFIG_SYSLOG_DEVPATH, O_RDONLY);
    if (fd < 0)
        goto done;

    ret = read(fd, transaction->out_msg.payload, n);
    if (ret < 0) {
        close(fd);
        goto done;
    }

    close(fd);

    transaction->out_msg.payload[ret] = '\0';
    transaction->out_msg.payload_length = ret + 1;
    transaction->out_msg.hdr->result = MHB_RESULT_SUCCESS;

done:
#else
    transaction->out_msg.hdr->result = MHB_RESULT_NONEXISTENT;
#endif

    return 0;
}

static int _mhb_handle_peer_diag_log_req(struct mhb_transaction *transaction)
{
    transaction->out_msg.hdr->addr = MHB_ADDR_DIAG|MHB_PEER_MASK;
    transaction->out_msg.hdr->type = MHB_TYPE_DIAG_LOG_RSP;
    transaction->out_msg.hdr->result = MHB_RESULT_UNKNOWN_ERROR;
    transaction->send_rsp = true;

    return 0;
}

static int mhb_handle_diag_log_req(struct mhb_transaction *transaction)
{
    if (transaction->in_msg.hdr->addr & MHB_PEER_MASK) {
        return _mhb_handle_peer_diag_log_req(transaction);
    } else {
        return _mhb_handle_local_diag_log_req(transaction);
    }
}

static int mhb_handle_diag_log_not(struct mhb_transaction *transaction)
{
    /* Forward log notification from UniPro to UART. */
    transaction->in_msg.hdr->addr |= MHB_PEER_MASK;
    return device_mhb_send(g_uart_dev, transaction->in_msg.hdr,
                           transaction->in_msg.payload,
                           transaction->in_msg.payload_length, 0);
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

static int mhb_handle_diag_control_req(struct mhb_transaction *transaction)
{
    struct mhb_diag_control_req *req =
        (struct mhb_diag_control_req *)transaction->in_msg.payload;

    transaction->out_msg.hdr->addr = MHB_ADDR_DIAG;
    transaction->out_msg.hdr->type = MHB_TYPE_DIAG_MODE_RSP;
    transaction->out_msg.hdr->result = MHB_RESULT_SUCCESS;
    switch(req->command) {
#if defined(CONFIG_REGLOG)
        case MHB_DIAG_CONTROL_REGLOG_FIFO:
            reglog_set_mode(REGLOG_MODE_FIFO);
# if defined(CONFIG_MHB_IPC_CLIENT)
            /* Forward the request on to the APBA. */
            mhb_unipro_send(transaction);
# endif
            break;

        case MHB_DIAG_CONTROL_REGLOG_STACK:
            reglog_set_mode(REGLOG_MODE_STACK);
# if defined(CONFIG_MHB_IPC_CLIENT)
            /* Forward the request on to the APBA. */
            mhb_unipro_send(transaction);
# endif
            break;
#endif
        default:
            dbg("ERROR: Unknown diag control: %d\n", req->command);
            transaction->out_msg.hdr->result = MHB_RESULT_NONEXISTENT;
            break;
    }

    transaction->send_rsp = true;
    return 0;
}

static int _mhb_handle_local_diag_reg_log_req(struct mhb_transaction *transaction)
{
#if defined(CONFIG_REGLOG)
    static size_t entries_left = REGLOG_DEPTH;
    size_t max_bytes;
    size_t bytes_read;
    struct reglog_value_s *log_entry;

    transaction->out_msg.hdr->addr = MHB_ADDR_DIAG;
    transaction->out_msg.hdr->type = MHB_TYPE_DIAG_REG_LOG_RSP;
    transaction->send_rsp = true;

    /* If the first time through, advance past the duplicate entries. */
    if (entries_left == REGLOG_DEPTH) {
        (void)reglog_advance_tail();
    }
    /*
     * Limit the data to the max payload for MHB or IPC.  The header and CRC are
     * sent as part of the transaction, but it's size must be rounded up to the
     * next 32 bit boundary.
     */
    max_bytes = ipc_get_sync_data_sz() - MHB_HDR_SIZE - MHB_CRC_SIZE;
    if (transaction->out_msg.payload_max < max_bytes) {
        max_bytes = transaction->out_msg.payload_max;
    }
    /* Update max_bytes to be an even number of log packets. */
    max_bytes = (max_bytes/sizeof(struct reglog_value_s)*sizeof(struct reglog_value_s));
    /* Copy the data into the MHB buffer. */
    bytes_read = 0;
    log_entry = (struct reglog_value_s *)transaction->out_msg.payload;
    do {
        if (reglog_get_entries(log_entry, 1) == 0)
        {
            entries_left = REGLOG_DEPTH;
            break;
        }
        TO_LE(log_entry->time);
        TO_LE(log_entry->addr);
        TO_LE(log_entry->val);
        log_entry++;
        bytes_read += sizeof(struct reglog_value_s);
    } while ((--entries_left) &&
             (max_bytes > bytes_read));
    transaction->out_msg.payload_length = bytes_read;
    transaction->out_msg.hdr->result = MHB_RESULT_SUCCESS;
#else
    transaction->out_msg.hdr->result = MHB_RESULT_NONEXISTENT;
#endif
    return 0;
}

static int _mhb_handle_peer_diag_reg_log_req(struct mhb_transaction *transaction)
{
#if defined(CONFIG_MHB_IPC_CLIENT)
    /* Forward the request to the APBA. */
    transaction->in_msg.hdr->addr &= ~MHB_PEER_MASK;
    transaction->send_rsp = true;
    int ret = mhb_unipro_send(transaction);
    transaction->out_msg.hdr->addr |= MHB_PEER_MASK;
    return ret;
#else
    transaction->out_msg.hdr->result = MHB_RESULT_NONEXISTENT;
    return 0;
#endif
}

static int mhb_handle_diag_reg_log_req(struct mhb_transaction *transaction)
{
    if (transaction->in_msg.hdr->addr & MHB_PEER_MASK) {
        return _mhb_handle_peer_diag_reg_log_req(transaction);
    } else {
        return _mhb_handle_local_diag_reg_log_req(transaction);
    }
}

static int _mhb_send_id_not(struct device *dev, struct mhb_diag_id_not *not)
{
    struct mhb_hdr hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_DIAG;
    hdr.type = MHB_TYPE_DIAG_ID_NOT;
    hdr.result = MHB_RESULT_SUCCESS;

#if CONFIG_MHB_IPC_CLIENT
    struct mhb_transaction transaction;
    memset(&transaction, 0, sizeof(transaction));

    transaction.dev = dev;
    transaction.in_msg.hdr = &hdr;
    transaction.in_msg.payload = (uint8_t *)not;
    transaction.in_msg.payload_length = sizeof(struct mhb_diag_id_not);

    return mhb_unipro_send(&transaction);
#elif CONFIG_MHB_IPC_SERVER
    return device_mhb_send(dev, &hdr, (const uint8_t *)not,
                           sizeof(struct mhb_diag_id_not), 0);
#endif
}

int mhb_send_id_not(void)
{
    struct mhb_diag_id_not not;
    memset(&not, 0, sizeof(not));

    unipro_attr_local_read(DME_DDBL1_MANUFACTURERID, &not.unipro_mid, 0);
    unipro_attr_local_read(DME_DDBL1_PRODUCTID, &not.unipro_pid, 0);
    unipro_attr_local_read(TSB_DME_DDBL2_A, &not.vid, 0);
    unipro_attr_local_read(TSB_DME_DDBL2_B, &not.pid, 0);

    not.major_version = CONFIG_VERSION_MAJOR;
    not.minor_version = CONFIG_VERSION_MINOR;
    strncpy(not.build, CONFIG_VERSION_BUILD, sizeof(not.build) - 1);

#if CONFIG_MHB_IPC_CLIENT
    return _mhb_send_id_not(NULL /* device */, &not);
#elif CONFIG_MHB_IPC_SERVER
    return _mhb_send_id_not(g_uart_dev, &not);
#endif
}

static int mhb_handle_diag_id_not(struct mhb_transaction *transaction)
{
    /* Forward ID notification from UniPro to UART. */
    transaction->in_msg.hdr->addr |= MHB_PEER_MASK;
    return device_mhb_send(g_uart_dev, transaction->in_msg.hdr,
                           transaction->in_msg.payload,
                           transaction->in_msg.payload_length, 0);
}

/* All */
static int mhb_handle_diag(struct mhb_transaction *transaction)
{
    switch (transaction->in_msg.hdr->type) {
    case MHB_TYPE_DIAG_LOG_REQ:
        return mhb_handle_diag_log_req(transaction);
    case MHB_TYPE_DIAG_LOG_NOT:
        return mhb_handle_diag_log_not(transaction);
    case MHB_TYPE_DIAG_LAST_REQ:
        return mhb_handle_diag_last_req(transaction);
    case MHB_TYPE_DIAG_MODE_REQ:
        return mhb_handle_diag_mode_req(transaction);
    case MHB_TYPE_DIAG_LOOP_REQ:
        return mhb_handle_diag_loop_req(transaction);
    case MHB_TYPE_DIAG_CONTROL_REQ:
        return mhb_handle_diag_control_req(transaction);
    case MHB_TYPE_DIAG_REG_LOG_REQ:
        return mhb_handle_diag_reg_log_req(transaction);
    case MHB_TYPE_DIAG_ID_NOT:
        return mhb_handle_diag_id_not(transaction);
    default:
        dbg("ERROR: unknown server diag event: %d\n", transaction->in_msg.hdr->type);
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
        dbg("ERROR: unknown function\n");
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

#if CONFIG_MHB_IPC_SERVER
static uint32_t mhb_unipro_handle_msg(void *in_data, uint32_t in_data_len,
                                      void **out_data, uint32_t *out_data_len)
{
    static uint8_t out_buf[MHB_MAX_MSG_SIZE];

    uint32_t ret;
    struct mhb_transaction transaction;

    if (!in_data_len || !in_data) {
        dbg("ERROR: Invalid input.\n");
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

#if CONFIG_MHB_IPC_CLIENT

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
            if (transaction->out_msg.hdr) {
                memcpy(transaction->out_msg.hdr, p, sizeof(*transaction->out_msg.hdr));
            }
            if (transaction->out_msg.payload) {
                memcpy(transaction->out_msg.payload, p + sizeof(*transaction->out_msg.hdr), out_len);
            }
            transaction->out_msg.payload_length = out_len;
        } else {
            dbg("ERROR: IPC response payload too big.\n");
        }

        release_response(out_param);
    }

    return ret;
}

#endif

int mhb_unipro_init(struct gearbox *gearbox)
{
#if CONFIG_MODS_MHB_SERVER_GEARBOX
    g_gearbox = gearbox;
#endif
    if (g_gearbox) {
        g_gearbox_cdsi0 = gearbox_register(g_gearbox);
        g_gearbox_cdsi1 = gearbox_register(g_gearbox);
        g_gearbox_hsic = gearbox_register(g_gearbox);
        g_gearbox_i2s = gearbox_register(g_gearbox);
        g_gearbox_diag = gearbox_register(g_gearbox);
    }

#if CONFIG_MHB_IPC_SERVER
    ipc_init();
    register_ipc_handler(IPC_APP_ID_MHB, mhb_unipro_handle_msg, NULL);
#elif CONFIG_MHB_IPC_CLIENT
    ipc_init();
#endif
#if defined(CONFIG_I2S_TUNNEL)
    (void)i2s_unipro_tunnel_init();
#endif
    return 0;
}
