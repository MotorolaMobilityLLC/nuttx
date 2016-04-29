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

#include <errno.h>
#include <debug.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <arch/atomic.h>

#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/device.h>
#include <nuttx/device_display.h>
#include <nuttx/device_slave_pwrctrl.h>
#include <nuttx/gpio.h>
#include <nuttx/time.h>

#include <nuttx/mhb/device_mhb.h>
#include <nuttx/mhb/mhb_dsi_display.h>
#include <nuttx/mhb/mhb_protocol.h>

#define MHB_DSI_DISPLAY_INVALID_RESOURCE  0xffffffff

#define MHB_DSI_DISPLAY_POWER_DELAY_US    100000
#define MHB_DSI_DISPLAY_OP_TIMEOUT_NS     2000000000LL /* 2 seconds in ns */

#define MHB_DSI_DISPLAY_CDSI_INSTANCE     0

enum {
    MHB_DSI_DISPLAY_STATE_OFF = 0,
    MHB_DSI_DISPLAY_STATE_APBE_ON,
    MHB_DSI_DISPLAY_STATE_CONFIG_DSI,
    MHB_DSI_DISPLAY_STATE_PANEL_INFO,
    MHB_DSI_DISPLAY_STATE_CONFIG_DCS,
    MHB_DSI_DISPLAY_STATE_STARTING,
    MHB_DSI_DISPLAY_STATE_ON,
    MHB_DSI_DISPLAY_STATE_STOPPING,
    MHB_DSI_DISPLAY_STATE_UNCONFIG_DCS,
    MHB_DSI_DISPLAY_STATE_UNCONFIG_DSI,
    MHB_DSI_DISPLAY_STATE_APBE_OFF,
    MHB_DSI_DISPLAY_STATE_ERROR,
};

static struct mhb_dsi_display
{
    struct device *display_dev;
    struct device *mhb_dev;
    struct device *slave_pwr_ctrl;
    uint32_t gpio_pwr1;
    uint32_t gpio_pwr2;
    uint32_t gpio_pwr3;
    uint32_t gpio_rst1;
    uint32_t gpio_rst2;
    atomic_t host_ready;
    struct mhb_dsi_panel_info panel_info;
    display_notification_cb callback;
    uint8_t cdsi_instance;
    uint8_t state;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
} g_display;

const static struct mhb_cdsi_cmd GET_SUPPLIER_ID[] = {
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_READ0, .length = 2,
        .u = { .spdata = 0x00a1 }, .delay = 10 }, /* supplier_id */
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_READ0, .length = 1,
    .u = { .spdata = 0x00da }, .delay = 10 }, /* id0 */
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_READ0, .length = 1,
        .u = { .spdata = 0x00db }, .delay = 10 }, /* id1 */
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_READ0, .length = 1,
        .u = { .spdata = 0x00dc }, .delay = 10 }, /* id2 */
};

/* operation signaling */
static void _mhb_dsi_display_signal_response(struct mhb_dsi_display *display)
{
    pthread_mutex_lock(&display->mutex);
    pthread_cond_signal(&display->cond);
    pthread_mutex_unlock(&display->mutex);
}

static int _mhb_dsi_display_wait_for_response(struct mhb_dsi_display *display)
{
    int ret;
    struct timespec expires;

    if (clock_gettime(CLOCK_REALTIME, &expires)) {
        return -EBADF;
    }

    uint64_t new_ns = timespec_to_nsec(&expires);
    new_ns += MHB_DSI_DISPLAY_OP_TIMEOUT_NS;
    nsec_to_timespec(new_ns, &expires);

    vdbg("wait start\n");

    pthread_mutex_lock(&display->mutex);
    ret = pthread_cond_timedwait(&display->cond, &display->mutex, &expires);
    pthread_mutex_unlock(&display->mutex);

    if (ret) {
        /* timeout or other erros */
        lldbg("ERROR: wait error %d\n", -ret);
        return -ETIMEDOUT;
    }

    vdbg("wait complete\n");

    return 0;
}

/* GPIOs */
static int _mhb_dsi_display_config_gpios(struct device *dev)
{
    struct device_resource *rsrc;

    struct mhb_dsi_display *display = device_get_private(dev);
    if (!display) {
        return -ENODEV;
    }

    display->gpio_pwr1 = MHB_DSI_DISPLAY_INVALID_RESOURCE;
    rsrc = device_resource_get_by_name(dev,
        DEVICE_RESOURCE_TYPE_GPIO, "pwr1_en");
    if (rsrc) {
        display->gpio_pwr1 = rsrc->start;
        gpio_direction_out(display->gpio_pwr1, 0);
    }

    display->gpio_pwr2 = MHB_DSI_DISPLAY_INVALID_RESOURCE;
    rsrc = device_resource_get_by_name(dev,
        DEVICE_RESOURCE_TYPE_GPIO, "pwr2_en");
    if (rsrc) {
        display->gpio_pwr2 = rsrc->start;
        gpio_direction_out(display->gpio_pwr2, 0);
    }

    display->gpio_pwr3 = MHB_DSI_DISPLAY_INVALID_RESOURCE;
    rsrc = device_resource_get_by_name(dev,
        DEVICE_RESOURCE_TYPE_GPIO, "pwr3_en");
    if (rsrc) {
        display->gpio_pwr3 = rsrc->start;
        gpio_direction_out(display->gpio_pwr3, 0);
    }

    display->gpio_rst1 = MHB_DSI_DISPLAY_INVALID_RESOURCE;
    rsrc = device_resource_get_by_name(dev,
        DEVICE_RESOURCE_TYPE_GPIO, "disp_rst1_n");
    if (rsrc) {
        display->gpio_rst1 = rsrc->start;
        gpio_direction_out(display->gpio_rst1, 0);
    }

    display->gpio_rst2 = MHB_DSI_DISPLAY_INVALID_RESOURCE;
    rsrc = device_resource_get_by_name(dev,
        DEVICE_RESOURCE_TYPE_GPIO, "disp_rst2_n");
    if (rsrc) {
        display->gpio_rst2 = rsrc->start;
        gpio_direction_out(display->gpio_rst2, 0);
    }

    return 0;
}

static void _mhb_dsi_display_power_on(struct mhb_dsi_display *display)
{
    if (display->gpio_pwr1 != MHB_DSI_DISPLAY_INVALID_RESOURCE)
        gpio_direction_out(display->gpio_pwr1, 1);

    if (display->gpio_pwr2 != MHB_DSI_DISPLAY_INVALID_RESOURCE)
        gpio_direction_out(display->gpio_pwr2, 1);

    if (display->gpio_pwr3 != MHB_DSI_DISPLAY_INVALID_RESOURCE)
        gpio_direction_out(display->gpio_pwr3, 1);

    usleep(MHB_DSI_DISPLAY_POWER_DELAY_US);

    if (display->gpio_rst1 != MHB_DSI_DISPLAY_INVALID_RESOURCE)
        gpio_direction_out(display->gpio_rst1, 1);

    if (display->gpio_rst2 != MHB_DSI_DISPLAY_INVALID_RESOURCE)
        gpio_direction_out(display->gpio_rst2, 1);
}

static void _mhb_dsi_display_power_off(struct mhb_dsi_display *display)
{
    if (display->gpio_rst1 != MHB_DSI_DISPLAY_INVALID_RESOURCE)
        gpio_direction_out(display->gpio_rst1, 0);

    if (display->gpio_rst2 != MHB_DSI_DISPLAY_INVALID_RESOURCE)
        gpio_direction_out(display->gpio_rst2, 0);

    if (display->gpio_pwr1 != MHB_DSI_DISPLAY_INVALID_RESOURCE)
        gpio_direction_out(display->gpio_pwr1, 0);

    if (display->gpio_pwr2 != MHB_DSI_DISPLAY_INVALID_RESOURCE)
        gpio_direction_out(display->gpio_pwr2, 0);

    if (display->gpio_pwr3 != MHB_DSI_DISPLAY_INVALID_RESOURCE)
        gpio_direction_out(display->gpio_pwr3, 0);
}

/* Notifications */
static int _mhb_dsi_display_notification(struct mhb_dsi_display *display,
    enum display_notification_event event)
{
    if (!atomic_get(&display->host_ready)) {
        lldbg("ERROR: host not ready for notification.\n");
        return -EAGAIN;
    }

    if (!display->callback) {
        lldbg("ERROR: no callback.\n");
        return -EINVAL;
    }

    display->callback(display->display_dev, event);

    return 0;
}

/* MHB */
static int _mhb_dsi_display_send_config_req(struct mhb_dsi_display *display)
{
    int ret;
    struct mhb_hdr hdr;
    const struct mhb_cdsi_config *cfg = NULL;
    size_t cfg_size = 0;

    ret = _mhb_dsi_display_get_config(display->cdsi_instance,
                &display->panel_info, &cfg, &cfg_size);
    if (ret || !cfg || !cfg_size) {
        lldbg("ERROR: failed to send config.\n");
        return ret;
    }

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_CDSI0;
    hdr.type = MHB_TYPE_CDSI_CONFIG_REQ;

    return device_mhb_send(display->mhb_dev, &hdr, (uint8_t *)cfg, cfg_size, 0);
}

static int _mhb_dsi_display_send_read_panel_info_req(struct mhb_dsi_display *display)
{
    struct mhb_hdr hdr;

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_CDSI0;
    hdr.type = MHB_TYPE_CDSI_READ_CMDS_REQ;

    return device_mhb_send(g_display.mhb_dev, &hdr, (uint8_t *)GET_SUPPLIER_ID, sizeof(GET_SUPPLIER_ID), 0);
}

static int _mhb_dsi_display_send_display_on_req(struct mhb_dsi_display *display)
{
    int ret;
    struct mhb_hdr hdr;
    const struct mhb_cdsi_cmd *cmds = NULL;
    size_t cmds_size = 0;

    ret = _mhb_dsi_display_get_on_commands(display->cdsi_instance,
                &display->panel_info, &cmds, &cmds_size);
    if (ret || !cmds || !cmds_size) {
        lldbg("ERROR: failed to get display on.\n");
        return ret;
    }

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_CDSI0;
    hdr.type = MHB_TYPE_CDSI_WRITE_CMDS_REQ;

    return device_mhb_send(g_display.mhb_dev, &hdr, (uint8_t *)cmds, cmds_size, 0);
}

static int
_mhb_dsi_display_send_display_off_req(struct mhb_dsi_display *display)
{
    int ret;
    struct mhb_hdr hdr;
    const struct mhb_cdsi_cmd *cmds = NULL;
    size_t cmds_size = 0;

    ret = _mhb_dsi_display_get_off_commands(display->cdsi_instance,
            &display->panel_info, &cmds, &cmds_size);
    if (ret || !cmds || !cmds_size) {
        lldbg("ERROR: failed to get display on.\n");
        return ret;
    }

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_CDSI0;
    hdr.type = MHB_TYPE_CDSI_WRITE_CMDS_REQ;

    return device_mhb_send(g_display.mhb_dev, &hdr, (uint8_t *)cmds, cmds_size, 0);
}

static int _mhb_dsi_display_send_control_req(struct mhb_dsi_display *display,
                                             uint8_t command)
{
    struct mhb_hdr hdr;
    struct mhb_cdsi_control_req req;

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_CDSI0;
    hdr.type = MHB_TYPE_CDSI_CONTROL_REQ;

    req.command = command;

    return device_mhb_send(display->mhb_dev, &hdr,
                           (uint8_t *)&req, sizeof(req), 0);
}

static int _mhb_dsi_display_send_unconfig_req(struct mhb_dsi_display *display)
{
    struct mhb_hdr hdr;

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_CDSI0;
    hdr.type = MHB_TYPE_CDSI_UNCONFIG_REQ;

    return device_mhb_send(display->mhb_dev, &hdr, NULL, 0, 0);
}

static int _mhb_dsi_display_mhb_handle_msg(struct device *dev,
    struct mhb_hdr *hdr, uint8_t *payload, size_t payload_length)
{
    /* Assume an error */
    int error = -1;

    struct mhb_dsi_display *display = &g_display;
    if (!display) {
        return -ENODEV;
    }

    vdbg("addr=%x, type=%x, result=%x, payload=%p, length=%zd, old_state=%d\n",
        hdr->addr, hdr->type, hdr->result, payload, payload_length,
        display->state);

    switch (hdr->type) {
    case MHB_TYPE_CDSI_CONFIG_RSP:
        if (display->state == MHB_DSI_DISPLAY_STATE_CONFIG_DSI) {
            /* config-dsi -> panel-info */
            _mhb_dsi_display_send_read_panel_info_req(display);

            display->state = MHB_DSI_DISPLAY_STATE_PANEL_INFO;
            error = 0;
        }
        break;
    case MHB_TYPE_CDSI_WRITE_CMDS_RSP:
        if (display->state == MHB_DSI_DISPLAY_STATE_CONFIG_DCS) {
            /* config-dcs -> starting */
            _mhb_dsi_display_send_control_req(display, MHB_CDSI_COMMAND_START);

            display->state = MHB_DSI_DISPLAY_STATE_STARTING;
            error = 0;
        } else if (display->state == MHB_DSI_DISPLAY_STATE_UNCONFIG_DCS) {
            /* unconfig-dcs -> unconfig-dsi */
            _mhb_dsi_display_send_unconfig_req(display);

            display->state = MHB_DSI_DISPLAY_STATE_UNCONFIG_DSI;
            error = 0;
        }
        break;
    case MHB_TYPE_CDSI_CONTROL_RSP:
        if (display->state == MHB_DSI_DISPLAY_STATE_STARTING) {
            /* starting -> on */
            display->state = MHB_DSI_DISPLAY_STATE_ON;
            _mhb_dsi_display_signal_response(display);
            error = 0;
        } else if (display->state == MHB_DSI_DISPLAY_STATE_STOPPING) {
            /* stopping -> unconfig-dcs */
            _mhb_dsi_display_send_display_off_req(display);

            display->state = MHB_DSI_DISPLAY_STATE_UNCONFIG_DCS;
            error = 0;
        }
        break;
    case MHB_TYPE_CDSI_STATUS_RSP:
        error = 0;
        break;
    case MHB_TYPE_CDSI_UNCONFIG_RSP:
        if (display->state == MHB_DSI_DISPLAY_STATE_UNCONFIG_DSI) {
            /* unconfig-dsi -> off */
            _mhb_dsi_display_power_off(display);

            display->state = MHB_DSI_DISPLAY_STATE_OFF;
            _mhb_dsi_display_signal_response(display);
            error = 0;
        }
        break;
    case MHB_TYPE_CDSI_READ_CMDS_RSP: {
        if (display->state == MHB_DSI_DISPLAY_STATE_PANEL_INFO) {
            /* panel-info -> config-dcs */
            _mhb_dsi_display_send_display_on_req(display);

            memset(&display->panel_info, 0, sizeof(display->panel_info));
            if (hdr->result == MHB_RESULT_SUCCESS &&
                payload_length == sizeof(display->panel_info)) {
                struct mhb_dsi_panel_info *info =
                    (struct mhb_dsi_panel_info *)payload;

                /* Save panel info. */
                display->panel_info.supplier_id = info->supplier_id;
                display->panel_info.id0 = info->id0;
                display->panel_info.id1 = info->id1;
                display->panel_info.id2 = info->id2;
                dbg("panel_info: 0x%x %x %x %x\n",
                    display->panel_info.supplier_id, display->panel_info.id0,
                    display->panel_info.id1, display->panel_info.id2);
            } else {
                dbg("ERROR: DCS read failed.\n");
            }

            display->state = MHB_DSI_DISPLAY_STATE_CONFIG_DCS;
            error = 0;
        }
        break;
    }
    default:
        break;
    }

    if (error) {
        lldbg("ERROR: device failed: state=%d\n", display->state);
        _mhb_dsi_display_notification(display, DISPLAY_NOTIFICATION_EVENT_FAILURE);

        display->state = MHB_DSI_DISPLAY_STATE_ERROR;
        _mhb_dsi_display_signal_response(display);
    }

    vdbg("new_state=%d\n", display->state);

    return 0;
}

/* Device operations */
static int mhb_dsi_display_host_ready(struct device *dev)
{
    struct mhb_dsi_display *display = device_get_private(dev);
    if (!display) {
        return -ENODEV;
    }

    /* If the host sends multiple host ready messages, the value will continue
       to increment. This is fine since there isn't a host not ready. */
    atomic_inc(&g_display.host_ready);

    return 0;
}

static int mhb_dsi_display_get_config_size(struct device *dev, uint32_t *size)
{
    struct mhb_dsi_display *display = device_get_private(dev);
    if (!display) {
        return -ENODEV;
    }

    if (!size) {
        return -EINVAL;
    }

    *size = sizeof(struct display_dsi_config);
    return 0;
}

static void _mhb_dsi_display_convert_dsi_config(struct mhb_dsi_display *display,
    const struct mhb_cdsi_config *src, struct display_dsi_config *dst)
{
    memset(dst, 0, sizeof(*dst));

    dst->manufacturer_id = display->panel_info.supplier_id;

    dst->mode = src->video_mode;
    dst->num_lanes = src->tx_num_lanes;

    dst->width = src->width;
    dst->height = src->height;

    dst->physical_width_dim = 3;
    dst->physical_length_dim = 5;

    dst->framerate = src->framerate;
    dst->bpp = src->bpp;

    dst->clockrate = (uint64_t)src->tx_mbits_per_lane;

    dst->t_clk_pre = src->t_clk_pre;
    dst->t_clk_post = src->t_clk_post;

    dst->continuous_clock = src->continuous_clock;
    dst->eot_mode = DISPLAY_CONFIG_DSI_EOT_MODE_APPEND;
    dst->vsync_mode = DISPLAY_CONFIG_DSI_VSYNC_MODE_NONE;
    dst->traffic_mode = DISPLAY_CONFIG_DSI_TRAFFIC_MODE_BURST;

    dst->virtual_channel_id = 0;
    dst->color_order = DISPLAY_CONFIG_DSI_SWAP_RGB_TO_RGB;
    dst->pixel_packing = DISPLAY_CONFIG_DSI_PIXEL_PACKING_UNPACKED;

    dst->horizontal_front_porch = src->horizontal_front_porch;
    dst->horizontal_sync_pulse_width = src->horizontal_pulse_width;
    dst->horizontal_sync_skew = src->horizontal_sync_skew;
    dst->horizontal_back_porch = src->horizontal_back_porch;
    dst->horizontal_left_border = src->horizontal_left_border;
    dst->horizontal_right_border = src->horizontal_right_border;

    dst->vertical_front_porch = src->vertical_front_porch;
    dst->vertical_sync_pulse_width = src->vertical_pulse_width;
    dst->vertical_back_porch = src->vertical_back_porch;
    dst->vertical_top_border = src->vertical_top_border;
    dst->vertical_bottom_border = src->vertical_bottom_border;
}

static int mhb_dsi_display_get_config(struct device *dev, uint8_t *display_type,
    uint8_t *config_type, uint32_t *size, uint8_t **config)
{
    int ret;
    const struct mhb_cdsi_config *cfg = NULL;
    size_t cfg_size = 0;
    struct display_dsi_config dst;

    struct mhb_dsi_display *display = device_get_private(dev);
    if (!display) {
        return -ENODEV;
    }

    ret = _mhb_dsi_display_get_config(display->cdsi_instance,
                &display->panel_info, &cfg, &cfg_size);
    if (ret || !cfg || !cfg_size) {
        lldbg("ERROR: failed to get config.\n");
        return -EINVAL;
    }

    _mhb_dsi_display_convert_dsi_config(display, cfg, &dst);

    *display_type = DISPLAY_TYPE_DSI;
    *config_type = DISPLAY_CONFIG_TYPE_DSI;
    *size = sizeof(dst);
    *config = (uint8_t *)&dst;

    return 0;
}

static int mhb_dsi_display_set_config(struct device *dev, uint8_t index)
{
    struct mhb_dsi_display *display = device_get_private(dev);
    if (!display) {
        return -ENODEV;
    }

    /* Currently only support one configuration. */
    if (index != 0) {
        dbg("ERROR: Set invalid configuration: index=%d\n", index);
        return -ENOSYS;
    }

    return 0;
}

static int mhb_dsi_display_get_state(struct device *dev, uint8_t *state)
{
    struct mhb_dsi_display *display = device_get_private(dev);
    if (!display) {
        return -ENODEV;
    }

    *state = display->state == MHB_DSI_DISPLAY_STATE_OFF ?
        DISPLAY_STATE_OFF : DISPLAY_STATE_ON;
    return 0;
}

static int mhb_slave_status_callback(struct device *dev, uint32_t slave_status)
{
    int result = -EFAULT;
    struct mhb_dsi_display *display = &g_display;

    vdbg("status=%d, old_state=%d\n", slave_status, display->state);

    switch (slave_status) {
    case MHB_PM_STATUS_PEER_CONNECTED:
        if (display->state == MHB_DSI_DISPLAY_STATE_APBE_ON) {
            /* Send the configuration to the APBE. */
            result = _mhb_dsi_display_send_config_req(display);
            if (result) {
                lldbg("ERROR: send config failed: %d\n", result);
                return result;
            }

            display->state = MHB_DSI_DISPLAY_STATE_CONFIG_DSI;
        }
    }

    vdbg("new_state=%d\n", display->state);

    return result;
}

static int _mhb_dsi_display_apbe_on(struct mhb_dsi_display *display)
{
    int ret;

    if (display->slave_pwr_ctrl) {
        lldbg("ERROR: Already open\n");
        return -EBUSY;
    }

    display->slave_pwr_ctrl = device_open(DEVICE_TYPE_SLAVE_PWRCTRL_HW,
        MHB_ADDR_CDSI0);
    if (!display->slave_pwr_ctrl) {
       lldbg("ERROR: Failed to open\n");
       return -ENODEV;
    }

    ret = device_slave_pwrctrl_register_status_callback(display->slave_pwr_ctrl,
        mhb_slave_status_callback);
    if (ret) {
        lldbg("ERROR: Failed to register\n");
        return ret;
    }

    ret = device_slave_pwrctrl_send_slave_state(display->slave_pwr_ctrl,
        SLAVE_STATE_ENABLED);
    if (ret) {
        lldbg("ERROR: Failed to send state\n");
        return ret;
    }

    return 0;
}

static int _mhb_dsi_display_apbe_off(struct mhb_dsi_display *display)
{
    int ret;

    if (!display->slave_pwr_ctrl) {
        return -ENODEV;
    }

    ret = device_slave_pwrctrl_send_slave_state(display->slave_pwr_ctrl,
        SLAVE_STATE_DISABLED);
    if (ret) {
        lldbg("ERROR: Failed to send state\n");
    }

    display->state = MHB_DSI_DISPLAY_STATE_APBE_OFF;

    ret = device_slave_pwrctrl_unregister_status_callback(
        display->slave_pwr_ctrl, mhb_slave_status_callback);
    if (ret) {
        lldbg("ERROR: Failed to unregister\n");
    }

    device_close(display->slave_pwr_ctrl);
    display->slave_pwr_ctrl = NULL;

    return 0;
}

static int _mhb_dsi_display_set_state_on(struct mhb_dsi_display *display)
{
    int result;

    /* Request APBE on. */
    _mhb_dsi_display_apbe_on(display);
    display->state = MHB_DSI_DISPLAY_STATE_APBE_ON;

    /* Turn panel on. */
    _mhb_dsi_display_power_on(display);

    result = _mhb_dsi_display_wait_for_response(display);
    if (result) {
        lldbg("ERROR: start failed: %d\n", result);
    }

    return result;
}

static int _mhb_dsi_display_set_state_off(struct mhb_dsi_display *display)
{
    int result;

    /* Stop the CDSI stream. */
    result = _mhb_dsi_display_send_control_req(display, MHB_CDSI_COMMAND_STOP);
    if (result) {
        lldbg("ERROR: send control failed: %d\n", result);
        return result;
    }

    display->state = MHB_DSI_DISPLAY_STATE_STOPPING;

    result = _mhb_dsi_display_wait_for_response(display);
    if (result) {
        lldbg("ERROR: stop failed: %d\n", result);
    }

    /* Request APBE off. */
    _mhb_dsi_display_apbe_off(display);

    display->state = MHB_DSI_DISPLAY_STATE_OFF;

    return result;
}

static int mhb_dsi_display_set_state(struct device *dev, uint8_t state)
{
    int result;

    struct mhb_dsi_display *display = device_get_private(dev);
    if (!display) {
        return -ENODEV;
    }

    switch (state) {
        case DISPLAY_STATE_OFF: {
            result = _mhb_dsi_display_set_state_off(display);
            break;
        }
        case DISPLAY_STATE_ON: {
            result = _mhb_dsi_display_set_state_on(display);
            break;
        }
        default:  {
            dbg("ERROR: Invalid display state\n");
            result = -EINVAL;
            break;
        }
    }

    return result;
}

static int mhb_dsi_display_register_callback(struct device *dev,
        display_notification_cb callback)
{
    struct mhb_dsi_display *display = device_get_private(dev);
    if (!display) {
        return -ENODEV;
    }

    if (!callback) {
        return -EINVAL;
    }

    display->callback = callback;
    return 0;
}

static int mhb_dsi_display_unregister_callback(struct device *dev)
{
    struct mhb_dsi_display *display = device_get_private(dev);
    if (!display) {
        return -ENODEV;
    }

    display->callback = NULL;
    return 0;
}

static int mhb_dsi_display_dev_open(struct device *dev)
{
    int result;

    irqstate_t flags;
    flags = irqsave();

    struct mhb_dsi_display *display = device_get_private(dev);
    if (!display) {
        result = -ENODEV;
        goto err_irqrestore;
    }

    if (display->mhb_dev) {
        lldbg("ERROR: already opened.\n");
        result = -EBUSY;
        goto err_irqrestore;
    }

    display->mhb_dev = device_open(DEVICE_TYPE_MHB, MHB_ADDR_CDSI0);
    if (!display->mhb_dev) {
        lldbg("ERROR: failed to open MHB device.\n");
        result = -ENOENT;
        goto err_irqrestore;
    }

    device_mhb_register_receiver(display->mhb_dev, MHB_ADDR_CDSI0,
                                 _mhb_dsi_display_mhb_handle_msg);

    result = 0;

err_irqrestore:
    irqrestore(flags);
    return result;
}

static void mhb_dsi_display_dev_close(struct device *dev) {
    irqstate_t flags;
    flags = irqsave();

    struct mhb_dsi_display *display = device_get_private(dev);
    if (!display) {
        return;
    }

    _mhb_dsi_display_apbe_off(display);

    if (display->mhb_dev) {
        device_mhb_unregister_receiver(display->mhb_dev, MHB_ADDR_CDSI0,
                                       _mhb_dsi_display_mhb_handle_msg);
        device_close(display->mhb_dev);
        display->mhb_dev = NULL;
    }

    irqrestore(flags);
}

static int mhb_dsi_display_probe(struct device *dev)
{
    int result;
    struct mhb_dsi_display *display = &g_display;
    device_set_private(dev, display);

    memset(display, 0, sizeof(*display));
    display->display_dev = dev;
    display->state = MHB_DSI_DISPLAY_STATE_OFF;
    display->cdsi_instance = MHB_DSI_DISPLAY_CDSI_INSTANCE;
    atomic_init(&display->host_ready, 0);
    pthread_mutex_init(&display->mutex, NULL);
    pthread_cond_init(&display->cond, NULL);

    result = _mhb_dsi_display_config_gpios(dev);
    if (result) {
        dbg("ERROR: Failed to configure GPIOS\n");
        return -EBUSY;
    }

    return 0;
}

static void mhb_dsi_display_remove(struct device *dev) {
    struct mhb_dsi_display *display = device_get_private(dev);
    if (display) {
        device_set_private(dev, NULL);
        memset(display, 0, sizeof(*display));
    }
}

const static struct device_display_type_ops mhb_dsi_display_ops = {
    .host_ready = mhb_dsi_display_host_ready,
    .get_config_size = mhb_dsi_display_get_config_size,
    .get_config = mhb_dsi_display_get_config,
    .set_config = mhb_dsi_display_set_config,
    .get_state = mhb_dsi_display_get_state,
    .set_state = mhb_dsi_display_set_state,
    .register_callback = mhb_dsi_display_register_callback,
    .unregister_callback = mhb_dsi_display_unregister_callback,
};

const static struct device_driver_ops mhb_dsi_display_driver_ops = {
    .probe = mhb_dsi_display_probe,
    .remove = mhb_dsi_display_remove,
    .open  = mhb_dsi_display_dev_open,
    .close = mhb_dsi_display_dev_close,
    .type_ops = (struct device_display_type_ops *)&mhb_dsi_display_ops,
};

const struct device_driver dsi_display_driver = {
    .type = DEVICE_TYPE_DISPLAY_HW,
    .name = "mhb_dsi_display",
    .desc = "MHB DSI Display Driver",
    .ops = (struct device_driver_ops *)&mhb_dsi_display_driver_ops,
};
