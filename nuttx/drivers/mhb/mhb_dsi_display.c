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

/* There are four software interfaces, each with their own state variables:
 *   1. Host (host_state)
 *   2. MHB display (state)
 *   3. UniPro link / APBE power control (link_state)
 *   4. Attach (attach_state)
 *
 * The MHB DSI display driver runs in four threads:
 *   1. Greybus thread for handling device operations (device_display_type_ops).
 *   2. MHB UART receiver thread for receiving MHB message callbacks.
 *   3. LP work-queue thread for:
 *       a. receiving attach/detach notifications
 *       b. sending display notification events
 *
 * There are five entry points:
 *   1. Device operations [Greybus thread]
 *   2. MHB receiver callback [MHB UART thread]
 *   3. Attach notifier callback [LP work-queue thread]
 *   4. Work-queue callback for notification event [LP work-queue thread]
 *   5. APBE callback [MHB UART thread or current thread]
 *
 * Each thread is locked using a single semaphore with the MHB_DSI_LOCK() and
 * MHB_DSI_UNLOCK() macros.
 *
 * While registering for the APBE callback, the initial callback occurs
 * immediately in the same thread. Release the lock while registering so that
 * the callback can re-acquire it.
 *
 * The APBE callback can occur on multiple threads: the MHB UART receive
 * thread or the current thread after calling register_slave_state_cb() or
 * send_slave_state(). In order to handle this, the APBE callback only takes
 * the semaphore if it is not already taken. It then only releases it if it
 * took it.
 *
 * Greybus operations are not sent until the device operation returns.  As a
 * result, a device operation that triggers a display notification event needs
 * to delay sending the event until after the operation returns. The LP
 * work-queue is used to queue and send events.
 * NOTE: Currently only one event can be sent at a time. This eliminates the
 *       need to malloc() for each event, but at the price of limited
 *       functionality.
 */

#include <errno.h>
#include <debug.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/device.h>
#include <nuttx/device_display.h>
#include <nuttx/device_slave_pwrctrl.h>
#include <nuttx/gpio.h>
#include <nuttx/time.h>
#include <nuttx/wqueue.h>

#include <nuttx/greybus/mods.h>

#include <nuttx/mhb/device_mhb.h>
#include <nuttx/mhb/mhb_dsi_display.h>
#include <nuttx/mhb/mhb_protocol.h>

/* Config options */
#define MHB_DSI_DISPLAY_READ_PANEL_ID      (0)
#define MHB_DSI_DISPLAY_CONNECT_ON_ATTACH  (1)
#define MHB_DSI_DISPLAY_RECONNECT_ON_ERROR (1)

#define MHB_DSI_DISPLAY_INVALID_RESOURCE  0xffffffff

#define MHB_DSI_DISPLAY_POWER_DELAY_US    100000
#define MHB_DSI_DISPLAY_OP_TIMEOUT_NS     10000000000LL /* 10 seconds in ns */

#define MHB_DSI_DISPLAY_CDSI_INSTANCE     0

#define MHB_DSI_LOCK(sem)   { vdbg("W\n"); sem_wait(sem); vdbg("L\n");  }
#define MHB_DSI_UNLOCK(sem) { sem_post(sem); vdbg("U\n"); }

#define MHB_DSI_DUMP_STATE(display) \
{ \
    dbg("display=%d, attach=%d, link=%d, host=%d\n", \
    display->state, display->attach_state, \
    display->link_state, display->host_state); \
}

enum {
    MHB_DSI_DISPLAY_STATE_OFF = 0,
    MHB_DSI_DISPLAY_STATE_APBE_ON,
    MHB_DSI_DISPLAY_STATE_CONFIG_DSI,
    MHB_DSI_DISPLAY_STATE_PANEL_INFO,
    MHB_DSI_DISPLAY_STATE_CONFIG_DCS,
    MHB_DSI_DISPLAY_STATE_STARTING,
    MHB_DSI_DISPLAY_STATE_ON,
    MHB_DSI_DISPLAY_STATE_BRIGHTNESS,
    MHB_DSI_DISPLAY_STATE_STOPPING,
    MHB_DSI_DISPLAY_STATE_UNCONFIG_DCS,
    MHB_DSI_DISPLAY_STATE_UNCONFIG_DSI,
    MHB_DSI_DISPLAY_STATE_APBE_OFF,
    MHB_DSI_DISPLAY_STATE_ERROR,
};

enum {
    MHB_DSI_DISPLAY_HOST_STATE_NOT_READY = 0,
    MHB_DSI_DISPLAY_HOST_STATE_UNAVAILABLE,
    MHB_DSI_DISPLAY_HOST_STATE_AVAILABLE,
    MHB_DSI_DISPLAY_HOST_STATE_CONNECTED,
    MHB_DSI_DISPLAY_HOST_STATE_DISCONNECTED,
    MHB_DSI_DISPLAY_HOST_STATE_ERROR,
};

static struct mhb_dsi_display
{
    /* Hardware */
    uint32_t gpio_pwr1;
    uint32_t gpio_pwr2;
    uint32_t gpio_pwr3;
    uint32_t gpio_pwr4;
    uint32_t gpio_rst1;
    uint32_t gpio_rst2;

    /* Host/Greybus interface */
    struct device *display_dev;
    uint8_t host_state;
    struct work_s wqueue;
    enum display_notification_event pending_event;
    display_notification_cb callback;

    /* MHB/DSI interface */
    struct device *mhb_dev;
    uint8_t state;
    uint8_t cdsi_instance;
    struct mhb_dsi_panel_info panel_info;
    struct display_dsi_config cfg;

    /* Link/APBE interface */
    struct device *slave_pwr_ctrl;
    uint8_t link_state;

    /* Attach interface */
    enum base_attached_e attach_state;

    sem_t sem;

    /* Used to block the host thread until the on/off request completes. */
    pthread_mutex_t mutex;
    pthread_cond_t cond;
} g_display;

const static struct mhb_cdsi_cmd GET_SUPPLIER_ID[] = {
#if MHB_DSI_DISPLAY_READ_PANEL_ID
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_READ0, .length = 2,
        .u = { .spdata = 0x00a1 }, .delay = 10 }, /* supplier_id */
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_READ0, .length = 1,
        .u = { .spdata = 0x00da }, .delay = 10 }, /* id0 */
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_READ0, .length = 1,
        .u = { .spdata = 0x00db }, .delay = 10 }, /* id1 */
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_READ0, .length = 1,
        .u = { .spdata = 0x00dc }, .delay = 10 }, /* id2 */
#endif
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
        dbg("ERROR: wait error %d\n", -ret);
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

    display->gpio_pwr4 = MHB_DSI_DISPLAY_INVALID_RESOURCE;
    rsrc = device_resource_get_by_name(dev,
        DEVICE_RESOURCE_TYPE_GPIO, "pwr4_en");
    if (rsrc) {
        display->gpio_pwr4 = rsrc->start;
        gpio_direction_out(display->gpio_pwr4, 0);
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

    if (display->gpio_pwr4 != MHB_DSI_DISPLAY_INVALID_RESOURCE)
        gpio_direction_out(display->gpio_pwr4, 1);

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

    if (display->gpio_pwr4 != MHB_DSI_DISPLAY_INVALID_RESOURCE)
        gpio_direction_out(display->gpio_pwr4, 0);
}

/* Notifications */
static int _mhb_dsi_display_notification_cb(struct mhb_dsi_display *display,
    enum display_notification_event event)
{
    int error = -EINVAL;

    dbg("event=%d\n", event);
    MHB_DSI_DUMP_STATE(display);

    switch (event) {
    case DISPLAY_NOTIFICATION_EVENT_AVAILABLE:
        if (display->host_state == MHB_DSI_DISPLAY_HOST_STATE_NOT_READY ||
            display->host_state == MHB_DSI_DISPLAY_HOST_STATE_UNAVAILABLE) {
            display->host_state = MHB_DSI_DISPLAY_HOST_STATE_AVAILABLE;
            error = 0;
        }
        break;
    case DISPLAY_NOTIFICATION_EVENT_CONNECT:
        if (display->host_state == MHB_DSI_DISPLAY_HOST_STATE_AVAILABLE ||
            display->host_state == MHB_DSI_DISPLAY_HOST_STATE_DISCONNECTED) {
            display->host_state = MHB_DSI_DISPLAY_HOST_STATE_CONNECTED;
            error = 0;
        }
        break;
    case DISPLAY_NOTIFICATION_EVENT_DISCONNECT:
        if (display->host_state == MHB_DSI_DISPLAY_HOST_STATE_CONNECTED) {
            display->host_state = MHB_DSI_DISPLAY_HOST_STATE_DISCONNECTED;
            error = 0;
        }
        break;
    case DISPLAY_NOTIFICATION_EVENT_UNAVAILABLE:
        display->host_state = MHB_DSI_DISPLAY_HOST_STATE_UNAVAILABLE;
        error = 0;
        break;

    case DISPLAY_NOTIFICATION_EVENT_FAILURE:
        display->host_state = MHB_DSI_DISPLAY_HOST_STATE_ERROR;
        error = 0;
        break;

    case DISPLAY_NOTIFICATION_EVENT_INVALID:
    default:
        dbg("ERROR: invalid event=%d\n", event);
        break;
    }

    if (error) {
        vdbg("ignore event\n");
        return error;
    }

    MHB_DSI_DUMP_STATE(display);

    if (!display->callback) {
        dbg("ERROR: no callback.\n");
        return -EINVAL;
    }

    display->callback(display->display_dev, event);

    return 0;
}

static void _mhb_dsi_display_wqueue_cb(void *arg)
{
    struct mhb_dsi_display *display = (struct mhb_dsi_display *)arg;

    if (!display) {
        dbg("ERROR: invalid display\n");
        return;
    }

    MHB_DSI_LOCK(&display->sem);

    enum display_notification_event event = display->pending_event;
    display->pending_event = DISPLAY_NOTIFICATION_EVENT_INVALID;

    _mhb_dsi_display_notification_cb(display, event);

    MHB_DSI_UNLOCK(&display->sem);
}

static int _mhb_dsi_display_notification(struct mhb_dsi_display *display,
    enum display_notification_event event, uint32_t delay)
{
    vdbg("event=%d\n", event);
    MHB_DSI_DUMP_STATE(display);

    if (display->pending_event != DISPLAY_NOTIFICATION_EVENT_INVALID) {
        dbg("ERROR: pending event=%d\n", display->pending_event);
    } else {
        display->pending_event = event;
    }

    return work_queue(LPWORK, &display->wqueue, _mhb_dsi_display_wqueue_cb,
                      display, delay);
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
        dbg("ERROR: failed to send config.\n");
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
        dbg("ERROR: failed to get display on.\n");
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
        dbg("ERROR: failed to get display on.\n");
        return ret;
    }

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_CDSI0;
    hdr.type = MHB_TYPE_CDSI_WRITE_CMDS_REQ;

    return device_mhb_send(g_display.mhb_dev, &hdr, (uint8_t *)cmds, cmds_size, 0);
}

static int _mhb_dsi_display_send_brightness_req(struct mhb_dsi_display *display, uint8_t brightness)
{
    struct mhb_cdsi_cmd BRIGHTNESS =
        { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_WRITE1, .length = 2, .u = { .spdata = 0xff51 }, .delay = 0 }; /* brightness_ctrl */

    BRIGHTNESS.u.spdata = (brightness << 8) | 0x51;

    struct mhb_hdr hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_CDSI0;
    hdr.type = MHB_TYPE_CDSI_WRITE_CMDS_REQ;

    return device_mhb_send(g_display.mhb_dev, &hdr, (uint8_t *)&BRIGHTNESS, sizeof(BRIGHTNESS), 0);
}

int mhb_dsi_display_set_brightness(uint8_t brightness)
{
    int result;
    struct mhb_dsi_display *display = &g_display;

    MHB_DSI_LOCK(&display->sem);

    display->state = MHB_DSI_DISPLAY_STATE_BRIGHTNESS;

    /* Set brightness. */
    _mhb_dsi_display_send_brightness_req(display, brightness);

    /* Release the lock before waiting for the response. */
    MHB_DSI_UNLOCK(&display->sem);

    result = _mhb_dsi_display_wait_for_response(display);
    if (result) {
        dbg("ERROR: start failed: %d\n", result);
    }

    return result;
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

    MHB_DSI_LOCK(&display->sem);

    dbg("addr=%x, type=%x, result=%x, payload=%p, length=%zd\n",
        hdr->addr, hdr->type, hdr->result, payload, payload_length);
    MHB_DSI_DUMP_STATE(display);

    switch (hdr->type) {
    case MHB_TYPE_CDSI_CONFIG_RSP:
        if (display->state == MHB_DSI_DISPLAY_STATE_CONFIG_DSI) {
            if (ARRAY_SIZE(GET_SUPPLIER_ID)) {
                /* config-dsi -> panel-info */
                _mhb_dsi_display_send_read_panel_info_req(display);

                display->state = MHB_DSI_DISPLAY_STATE_PANEL_INFO;
                error = 0;
            } else {
                /* state-on complete */
                _mhb_dsi_display_signal_response(display);
                error = 0;
            }
        }
        break;
    case MHB_TYPE_CDSI_WRITE_CMDS_RSP:
        if (display->state == MHB_DSI_DISPLAY_STATE_CONFIG_DCS) {
            /* config-dcs -> starting */
            _mhb_dsi_display_send_control_req(display, MHB_CDSI_COMMAND_START);

            display->state = MHB_DSI_DISPLAY_STATE_STARTING;
            error = 0;
        } else if (display->state == MHB_DSI_DISPLAY_STATE_UNCONFIG_DCS) {
            /* state-blank complete */
            _mhb_dsi_display_signal_response(display);
            error = 0;
        } else if (display->state == MHB_DSI_DISPLAY_STATE_BRIGHTNESS) {
            display->state = MHB_DSI_DISPLAY_STATE_ON;
            /* dcs-brightness complete */
            _mhb_dsi_display_signal_response(display);
            error = 0;
        }
        break;
    case MHB_TYPE_CDSI_CONTROL_RSP:
        if (display->state == MHB_DSI_DISPLAY_STATE_STARTING) {
            /* starting -> on */
            display->state = MHB_DSI_DISPLAY_STATE_ON;
            /* state-unblank complete */
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
            display->state = MHB_DSI_DISPLAY_STATE_OFF;
            /* state-off complete */
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
        dbg("ERROR: device failed: state=%d\n", display->state);
        _mhb_dsi_display_notification(display,
                            DISPLAY_NOTIFICATION_EVENT_FAILURE, 0 /* delay */);

        display->state = MHB_DSI_DISPLAY_STATE_ERROR;
        _mhb_dsi_display_signal_response(display);
    }

    MHB_DSI_DUMP_STATE(display);

    MHB_DSI_UNLOCK(&display->sem);

    return 0;
}

static int _mhb_dsi_display_connect_on_attach(struct mhb_dsi_display *display)
{
    int ret = 0;

#if MHB_DSI_DISPLAY_CONNECT_ON_ATTACH
    MHB_DSI_DUMP_STATE(display);

    if (display->state != MHB_DSI_DISPLAY_STATE_OFF) {
        vdbg("not off\n");
        return -EAGAIN;
    }

    if (display->attach_state != BASE_ATTACHED) {
        vdbg("not attached\n");
        return -EAGAIN;
    }

    _mhb_dsi_display_notification(display,
                            DISPLAY_NOTIFICATION_EVENT_CONNECT, 0 /* delay */);
#endif

    return ret;
}

static int _mhb_dsi_display_reconnect_on_error(struct mhb_dsi_display *display)
{
    int ret = 0;

#if MHB_DSI_DISPLAY_RECONNECT_ON_ERROR
    MHB_DSI_DUMP_STATE(display);

    if (display->state != MHB_DSI_DISPLAY_STATE_OFF &&
        display->state != MHB_DSI_DISPLAY_STATE_ERROR) {
        vdbg("wrong state\n");
        return -EAGAIN;
    }

    if (display->attach_state != BASE_ATTACHED) {
        vdbg("not attached\n");
        return -EAGAIN;
    }

    _mhb_dsi_display_notification(display,
                            DISPLAY_NOTIFICATION_EVENT_CONNECT, 0 /* delay */);
#endif

    return ret;
}

/* Device operations */
static int mhb_dsi_display_host_ready(struct device *dev)
{
    int ret;

    struct mhb_dsi_display *display = device_get_private(dev);
    if (!display) {
        return -ENODEV;
    }

    MHB_DSI_LOCK(&display->sem);

    MHB_DSI_DUMP_STATE(display);

    _mhb_dsi_display_notification(display,
                        DISPLAY_NOTIFICATION_EVENT_AVAILABLE, 0 /* delay */);

    MHB_DSI_UNLOCK(&display->sem);

    return ret;
}

static void _mhb_dsi_display_convert_dsi_config(struct mhb_dsi_display *display,
    const struct mhb_cdsi_config *src, struct display_dsi_config *dst)
{
    memset(dst, 0, sizeof(*dst));

    dst->manufacturer_id = display->panel_info.supplier_id;

    dst->mode = src->video_mode ? DISPLAY_CONFIG_DSI_MODE_VIDEO :
        DISPLAY_CONFIG_DSI_MODE_COMMAND;
    dst->num_lanes = src->rx_num_lanes;

    dst->width = src->width;
    dst->height = src->height;

    dst->physical_width_dim = src->physical_width;
    dst->physical_length_dim = src->physical_height;

    dst->framerate = src->framerate;
    dst->bpp = src->bpp;

    dst->clockrate = (uint64_t)src->rx_bits_per_lane;

    dst->t_clk_pre = src->t_clk_pre;
    dst->t_clk_post = src->t_clk_post;

    dst->continuous_clock = src->continuous_clock;
    dst->eot_mode = src->eot_mode;
    dst->vsync_mode = src->vsync_mode;
    dst->traffic_mode = src->traffic_mode;

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
    struct mhb_dsi_display *display = device_get_private(dev);
    if (!display) {
        return -ENODEV;
    }

    *display_type = DISPLAY_TYPE_DSI;
    *config_type = DISPLAY_CONFIG_TYPE_DSI;
    *size = sizeof(display->cfg);

    if (config) {
        int ret;
        const struct mhb_cdsi_config *cfg = NULL;
        size_t cfg_size = 0;
        struct display_dsi_config *dst = &display->cfg;

        ret = _mhb_dsi_display_get_config(display->cdsi_instance,
                    &display->panel_info, &cfg, &cfg_size);
        if (ret || !cfg || !cfg_size) {
            dbg("ERROR: failed to get config.\n");
            return -EINVAL;
        }

        MHB_DSI_LOCK(&display->sem);

        MHB_DSI_DUMP_STATE(display);

        _mhb_dsi_display_convert_dsi_config(display, cfg, dst);

        dbg("wd=%d (%d %d %d %d %d %d), ht=%d (%d %d %d %d %d), fps=%d, bpp=%d\n",
        dst->width,
        dst->horizontal_back_porch, dst->horizontal_sync_pulse_width,
        dst->horizontal_sync_skew, dst->horizontal_front_porch,
        dst->horizontal_left_border, dst->horizontal_right_border,
        dst->height,
        dst->vertical_front_porch, dst->vertical_sync_pulse_width,
        dst->vertical_back_porch, dst->vertical_top_border,
        dst->vertical_bottom_border,
        dst->framerate, dst->bpp);

        dbg("mid=%d, mode=%d, clk=%d, lanes=%d"
            ", cont=%d, eot=%d, vsync=%d, traffic=%d"
            ", pre=%d, post=%d\n",
            dst->manufacturer_id, dst->mode, dst->clockrate, dst->num_lanes,
            dst->continuous_clock, dst->eot_mode,
            dst->vsync_mode, dst->traffic_mode,
            dst->t_clk_pre, dst->t_clk_post);

        *config = (uint8_t *)dst;

        _mhb_dsi_display_connect_on_attach(display);

        MHB_DSI_UNLOCK(&display->sem);
    }

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

    MHB_DSI_DUMP_STATE(display);

    return 0;
}

static int mhb_dsi_display_get_state(struct device *dev, uint8_t *state)
{
    struct mhb_dsi_display *display = device_get_private(dev);
    if (!display) {
        return -ENODEV;
    }

    MHB_DSI_LOCK(&display->sem);

    MHB_DSI_DUMP_STATE(display);

    switch (display->state) {
    case MHB_DSI_DISPLAY_STATE_UNCONFIG_DSI:
    case MHB_DSI_DISPLAY_STATE_APBE_OFF:
    case MHB_DSI_DISPLAY_STATE_OFF:
    case MHB_DSI_DISPLAY_STATE_ERROR:
        *state = DISPLAY_STATE_OFF;
        break;

    case MHB_DSI_DISPLAY_STATE_APBE_ON:
    case MHB_DSI_DISPLAY_STATE_CONFIG_DSI:
    case MHB_DSI_DISPLAY_STATE_PANEL_INFO:
        *state = DISPLAY_STATE_ON;
        break;

    case MHB_DSI_DISPLAY_STATE_STOPPING:
    case MHB_DSI_DISPLAY_STATE_UNCONFIG_DCS:
        *state = DISPLAY_STATE_BLANK;
        break;

    case MHB_DSI_DISPLAY_STATE_CONFIG_DCS:
    case MHB_DSI_DISPLAY_STATE_STARTING:
    case MHB_DSI_DISPLAY_STATE_ON:
    case MHB_DSI_DISPLAY_STATE_BRIGHTNESS:
        *state = DISPLAY_STATE_UNBLANK;
        break;
    }

    MHB_DSI_UNLOCK(&display->sem);

    return 0;
}

static int mhb_slave_status_callback(struct device *dev, uint32_t slave_status)
{
    int result = -EFAULT;
    struct mhb_dsi_display *display = &g_display;
    bool unlock_when_done;

    /* Lock only if not already locked. */
    vdbg("TW\n");
    result = sem_trywait(&display->sem);
    if (!result) {
        /* success, locked */
        vdbg("L\n");
        unlock_when_done = true;
    } else if (errno == EAGAIN) {
        /* success, already locked */
        unlock_when_done = false;
    } else {
        dbg("ERROR: lock failed: %d\n", -errno);
        return -errno;
    }

    dbg("link=%d\n", slave_status);
    MHB_DSI_DUMP_STATE(display);

    display->link_state = slave_status;

    switch (slave_status) {
    case MHB_PM_STATUS_PEER_CONNECTED:
        if (display->state == MHB_DSI_DISPLAY_STATE_APBE_ON) {
            /* Send the configuration to the APBE. */
            result = _mhb_dsi_display_send_config_req(display);
            if (result) {
                dbg("ERROR: send config failed: %d\n", result);
            } else {
                display->state = MHB_DSI_DISPLAY_STATE_CONFIG_DSI;
            }
        }
        break;
    case MHB_PM_STATUS_PEER_ON:
    case MHB_PM_STATUS_PEER_RESET:
    case MHB_PM_STATUS_PEER_DISCONNECTED:
        if (display->state != MHB_DSI_DISPLAY_STATE_APBE_ON) {
            vdbg("link reset, disconnect\n");

            display->state = MHB_DSI_DISPLAY_HOST_STATE_DISCONNECTED;
            _mhb_dsi_display_notification(display,
                        DISPLAY_NOTIFICATION_EVENT_DISCONNECT, 0 /* delay */);
        }
        break;
    }

    MHB_DSI_DUMP_STATE(display);

    if (unlock_when_done) {
        MHB_DSI_UNLOCK(&display->sem);
        unlock_when_done = false;
    }

    return result;
}

static int _mhb_dsi_display_apbe_on(struct mhb_dsi_display *display)
{
    int ret;

    if (display->slave_pwr_ctrl) {
        dbg("ERROR: Already open\n");
        return -EBUSY;
    }

    display->slave_pwr_ctrl = device_open(DEVICE_TYPE_SLAVE_PWRCTRL_HW,
        MHB_ADDR_CDSI0);
    if (!display->slave_pwr_ctrl) {
       dbg("ERROR: Failed to open\n");
       return -ENODEV;
    }

    ret = device_slave_pwrctrl_register_status_callback(display->slave_pwr_ctrl,
        mhb_slave_status_callback);

    if (ret) {
        dbg("ERROR: Failed to register\n");
        return ret;
    }

    ret = device_slave_pwrctrl_send_slave_state(display->slave_pwr_ctrl,
        SLAVE_STATE_ENABLED);
    if (ret) {
        dbg("ERROR: Failed to send state\n");
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
        dbg("ERROR: Failed to send state\n");
    }

    display->state = MHB_DSI_DISPLAY_STATE_APBE_OFF;

    ret = device_slave_pwrctrl_unregister_status_callback(
        display->slave_pwr_ctrl, mhb_slave_status_callback);
    if (ret) {
        dbg("ERROR: Failed to unregister\n");
    }

    device_close(display->slave_pwr_ctrl);
    display->slave_pwr_ctrl = NULL;

    return 0;
}

static int _mhb_dsi_display_attach(void *arg, const void *data)
{
    if (!arg || !data) {
        return -EINVAL;
    }

    struct mhb_dsi_display *display = (struct mhb_dsi_display *)arg;
    enum base_attached_e state = *((enum base_attached_e *)data);

    MHB_DSI_LOCK(&display->sem);

    dbg("attach=%d\n", state);
    MHB_DSI_DUMP_STATE(display);

    display->attach_state = state;

    switch (state) {
    case BASE_DETACHED:
    case BASE_ATTACHED_OFF:
        /* Turn panel off. */
        _mhb_dsi_display_power_off(display);

        /* Request APBE off. */
        _mhb_dsi_display_apbe_off(display);

        display->state = MHB_DSI_DISPLAY_STATE_OFF;
        display->host_state = MHB_DSI_DISPLAY_HOST_STATE_NOT_READY;
        break;
    case BASE_ATTACHED:
    case BASE_INVALID:
        /* Ignore */
        break;
    }

    MHB_DSI_DUMP_STATE(display);

    MHB_DSI_UNLOCK(&display->sem);

    return 0;
}

static int _mhb_dsi_display_set_state_on(struct mhb_dsi_display *display)
{
    int result;

    MHB_DSI_LOCK(&display->sem);

    /* Request APBE on. */
    _mhb_dsi_display_apbe_on(display);
    display->state = MHB_DSI_DISPLAY_STATE_APBE_ON;

    /* Turn panel on. */
    _mhb_dsi_display_power_on(display);

    /* Release the lock before waiting for the response. */
    MHB_DSI_UNLOCK(&display->sem);

    result = _mhb_dsi_display_wait_for_response(display);
    if (result) {
        dbg("ERROR: start failed: %d\n", result);
    }

    return result;
}

static int _mhb_dsi_display_set_state_unblank(struct mhb_dsi_display *display)
{
    int result;

    MHB_DSI_LOCK(&display->sem);

    /* config-dsi -> config-dcs */
    _mhb_dsi_display_send_display_on_req(display);

    display->state = MHB_DSI_DISPLAY_STATE_CONFIG_DCS;

    /* Release the lock before waiting for the response. */
    MHB_DSI_UNLOCK(&display->sem);

    result = _mhb_dsi_display_wait_for_response(display);
    if (result) {
        dbg("ERROR: start failed: %d\n", result);
    }

    return result;
}

static int _mhb_dsi_display_set_state_blank(struct mhb_dsi_display *display)
{
    int result;

    MHB_DSI_LOCK(&display->sem);

    /* Stop the CDSI stream. */
    result = _mhb_dsi_display_send_control_req(display, MHB_CDSI_COMMAND_STOP);
    if (result) {
        dbg("ERROR: send control failed: %d\n", result);
        return result;
    }

    display->state = MHB_DSI_DISPLAY_STATE_STOPPING;

    /* Release the lock while waiting for the response. */
    MHB_DSI_UNLOCK(&display->sem);

    result = _mhb_dsi_display_wait_for_response(display);
    if (result) {
        dbg("ERROR: stop failed: %d\n", result);
    }

    return result;
}

static int _mhb_dsi_display_set_state_off(struct mhb_dsi_display *display)
{
    int result;

    MHB_DSI_LOCK(&display->sem);

    /* unconfig-dcs -> unconfig-dsi */
    _mhb_dsi_display_send_unconfig_req(display);

    display->state = MHB_DSI_DISPLAY_STATE_UNCONFIG_DSI;

    /* Release the lock while waiting for the response. */
    MHB_DSI_UNLOCK(&display->sem);

    result = _mhb_dsi_display_wait_for_response(display);
    if (result) {
        dbg("ERROR: stop failed: %d\n", result);
    }

    /* Grab the lock again to finish up. */
    MHB_DSI_LOCK(&display->sem);

    /* Turn panel off. */
    _mhb_dsi_display_power_off(display);

    /* Request APBE off. */
    _mhb_dsi_display_apbe_off(display);

    display->state = MHB_DSI_DISPLAY_STATE_OFF;

    /* Reconnect, if appropriate */
    _mhb_dsi_display_reconnect_on_error(display);

    MHB_DSI_UNLOCK(&display->sem);

    return result;
}

static int mhb_dsi_display_set_state(struct device *dev, uint8_t state)
{
    int result;

    struct mhb_dsi_display *display = device_get_private(dev);
    if (!display) {
        return -ENODEV;
    }

    dbg("state=%d\n", state);
    MHB_DSI_DUMP_STATE(display);

    switch (state) {
        case DISPLAY_STATE_OFF: {
            result = _mhb_dsi_display_set_state_off(display);
            break;
        }
        case DISPLAY_STATE_ON: {
            result = _mhb_dsi_display_set_state_on(display);
            break;
        }
        case DISPLAY_STATE_BLANK: {
            result = _mhb_dsi_display_set_state_blank(display);
            break;
        }
        case DISPLAY_STATE_UNBLANK: {
            result = _mhb_dsi_display_set_state_unblank(display);
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

    MHB_DSI_DUMP_STATE(display);

    display->callback = callback;
    return 0;
}

static int mhb_dsi_display_unregister_callback(struct device *dev)
{
    struct mhb_dsi_display *display = device_get_private(dev);
    if (!display) {
        return -ENODEV;
    }

    MHB_DSI_DUMP_STATE(display);

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
        dbg("ERROR: already opened.\n");
        result = -EBUSY;
        goto err_irqrestore;
    }

    display->mhb_dev = device_open(DEVICE_TYPE_MHB, MHB_ADDR_CDSI0);
    if (!display->mhb_dev) {
        dbg("ERROR: failed to open MHB device.\n");
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
    display->host_state = MHB_DSI_DISPLAY_HOST_STATE_NOT_READY;
    display->pending_event = DISPLAY_NOTIFICATION_EVENT_INVALID;
    display->cdsi_instance = MHB_DSI_DISPLAY_CDSI_INSTANCE;
    pthread_mutex_init(&display->mutex, NULL);
    pthread_cond_init(&display->cond, NULL);
    sem_init(&display->sem, 0 /* shared */, 1 /* value */);

    result = _mhb_dsi_display_config_gpios(dev);
    if (result) {
        dbg("ERROR: Failed to configure GPIOS\n");
        return -EBUSY;
    }

    /* Register for attach notifications. This will callback immediately. */
    if (mods_attach_register(_mhb_dsi_display_attach, display)) {
        dbg("ERROR: failed to register attach notifier\n");
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
