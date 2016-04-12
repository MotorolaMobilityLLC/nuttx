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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/config.h>
#include <nuttx/device_cam_ext.h>
#include <nuttx/gpio.h>
#include <nuttx/i2c.h>
#include <nuttx/math.h>

#ifdef CONFIG_REDCARPET_APBE
#include <apps/ice/cdsi.h>
#include <arch/chip/cdsi.h>
#include <arch/chip/cdsi_config.h>
#endif

#include <nuttx/time.h>
#include <nuttx/device.h>
#include <nuttx/unipro/unipro.h>
#include <nuttx/device_slave_pwrctrl.h>
#include <nuttx/mhb/device_mhb.h>
#include <nuttx/mhb/mhb_protocol.h>

#include "greybus/v4l2_camera_ext_ctrls.h"

#include "camera_ext.h"
#include "tc35874x-util.h"

#include "camera_ext_s10p.h"

/*
 * This is the reference greybus camera extension driver for
 * S10 Imager + Toshiba Parallel to CSI-2 bridge chip (TC35874X series).
 *
 */

#define MHB_CDSI_OP_TIMEOUT_MS 800
#define MHB_CDSI_CAM_INSTANCE 0

#define DEBUG_DUMP_REGISTER 1
#define BRIDGE_RESET_DELAY 100000 /* us */

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

typedef enum {
    CAMERA_S10_STATE_OFF = 0,
    CAMERA_S10_STATE_ON,
    CAMERA_S10_STATE_STREAMING,
} dev_state_t;

struct dev_private_s
{
    dev_state_t state;
    struct tc35874x_i2c_dev_info tc_i2c_info;
    struct s10p_i2c_dev_info s10_i2c_info;
    struct device *mhb_device;
    struct device *slave_pwr_ctrl;
    uint8_t cdsi_instance;
    uint8_t bridge_enable;
    uint8_t cam_enable;
    uint8_t cam_int0;
    uint8_t cam_int1;
    uint8_t cam_int2;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
};

//static allocate the singleton instance
static struct dev_private_s s_device;
/////////////////////////////////////////////////////////////////////
#define DEV_TO_PRIVATE(dev_ptr, priv_ptr) struct dev_private_s *priv_ptr =\
        (struct dev_private_s *)device_driver_get_private(dev_ptr)

/* BEGIN - Supported Format definitions */
struct user_data_t {
    uint16_t pll1;
    uint16_t pll2;
    uint16_t fifo_level;
    uint32_t num_csi_lanes;
};

static struct user_data_t _user_data[] = {
    {
        .pll1 = 0x207c,
        .pll2 = 0x0613,
        .fifo_level = 10,
        .num_csi_lanes = 4,
    },
};

static const struct camera_ext_frmival_node _frmival_fhd[] = {
    {
        .numerator = 1,
        .denominator = 30,
        .user_data = &_user_data[0],
    },
};

static const struct camera_ext_frmsize_node _frmsizes_altek[] = {
    {
        .width = 1920,
        .height = 1080,
        .num_frmivals = ARRAY_SIZE(_frmival_fhd),
        .frmival_nodes = _frmival_fhd,
    },
};

static const struct camera_ext_format_node _formats[] = {
    {
        .name = "UYVY",
        .fourcc = V4L2_PIX_FMT_UYVY,
        .depth = 16,
        .num_frmsizes = ARRAY_SIZE(_frmsizes_altek),
        .frmsize_nodes = _frmsizes_altek,
    },
};

static const struct camera_ext_input_node _inputs[] = {
    {
        .name = "Sunny S10P",
        .type = CAM_EXT_INPUT_TYPE_CAMERA,
        .status = 0,
        .num_formats = ARRAY_SIZE(_formats),
        .format_nodes = _formats,
    },
};

static const struct camera_ext_format_db _db = {
    .num_inputs = ARRAY_SIZE(_inputs),
    .input_nodes = _inputs,
};

/* END - Supported Format definitions */

/* Bridge register configurations */
#if DEBUG_DUMP_REGISTER
static int bridge_debug_dump(struct tc35874x_i2c_dev_info *i2c)
{
    /* system  */
    tc35874x_read_reg2(i2c, 0x0002);
    tc35874x_read_reg2(i2c, 0x0006);

    /* PLL */
    tc35874x_read_reg2(i2c, 0x0016);
    tc35874x_read_reg2(i2c, 0x0018);

    /* DPI input control */
    tc35874x_read_reg2(i2c, 0x0006);
    tc35874x_read_reg2(i2c, 0x0008);
    tc35874x_read_reg2(i2c, 0x0022);

    /* CSI Tx Phy */
    tc35874x_read_reg4(i2c, 0x0140);
    tc35874x_read_reg4(i2c, 0x0144);
    tc35874x_read_reg4(i2c, 0x0148);
    tc35874x_read_reg4(i2c, 0x014c);
    tc35874x_read_reg4(i2c, 0x0150);

    /* CSI Tx PPI */
    tc35874x_read_reg4(i2c, 0x0210);
    tc35874x_read_reg4(i2c, 0x0214);
    tc35874x_read_reg4(i2c, 0x0218);
    tc35874x_read_reg4(i2c, 0x021c);
    tc35874x_read_reg4(i2c, 0x0220);
    tc35874x_read_reg4(i2c, 0x0224);
    tc35874x_read_reg4(i2c, 0x0228);
    tc35874x_read_reg4(i2c, 0x022c);
    tc35874x_read_reg4(i2c, 0x0234);
    tc35874x_read_reg4(i2c, 0x0238);
    tc35874x_read_reg4(i2c, 0x0204);

    /* CSI Start */
    tc35874x_read_reg4(i2c, 0x0518);
    tc35874x_read_reg4(i2c, 0x0500);

    return 0;
}
#endif

static int bridge_on(struct tc35874x_i2c_dev_info *i2c, void *data)
{
    int rc = tc35874x_write_reg2(i2c, 0x00e0, 0x0000); /* reset color bar */
    if (!rc) rc = tc35874x_write_reg2(i2c, 0x0004, 0x0004);

    /* system reset */
    if (!rc) rc = tc35874x_write_reg2(i2c, 0x0002, 0x0001);
    if (!rc) usleep(BRIDGE_RESET_DELAY);
    if (!rc) rc = tc35874x_write_reg2(i2c, 0x0002, 0x0000);
    if (!rc) usleep(BRIDGE_RESET_DELAY);

    return rc;
}

static int bridge_off(struct tc35874x_i2c_dev_info *i2c, void *data)
{
    int rc = tc35874x_write_reg2(i2c, 0x0004, 0x0004);
    /* put system in sleep */
    if (!rc) rc = tc35874x_write_reg2(i2c, 0x0002, 0x0001);
    if (rc) usleep(BRIDGE_RESET_DELAY);

    return rc;
}

static int bridge_setup_and_start(struct tc35874x_i2c_dev_info *i2c, void *data)
{
    uint32_t value;
    uint16_t svalue;
    const struct camera_ext_format_user_config *cfg =
        (const struct camera_ext_format_user_config *)data;
    const struct camera_ext_format_node *fmt;
    const struct camera_ext_frmsize_node  *frmsize;
    const struct camera_ext_frmival_node *ival;
    const struct user_data_t *udata;

    fmt = get_current_format_node(&_db, cfg);
    if (fmt == NULL) {
        CAM_ERR("Failed to get current format\n");
        return -1;
    }

    if (fmt->fourcc != V4L2_PIX_FMT_RGB24 &&
        fmt->fourcc != V4L2_PIX_FMT_UYVY) {
        CAM_ERR("Unsupported format 0x%x\n", fmt->fourcc);
        return -1;
    }

    frmsize = get_current_frmsize_node(&_db, cfg);
    if (frmsize == NULL) {
        CAM_ERR("Failed to get current frame size\n");
        return -1;
    }

    ival = get_current_frmival_node(&_db, cfg);
    if (ival == NULL) {
        CAM_ERR("Failed to get current frame interval\n");
        return -1;
    }

    udata = (const struct user_data_t *)ival->user_data;

    if (udata == NULL) {
        CAM_ERR("Failed to get user data\n");
        return -1;
    }

    /* PLL */
    tc35874x_write_reg2(i2c, 0x0016, udata->pll1);
    tc35874x_write_reg2(i2c, 0x0018, udata->pll2);

    /* DPI input control */
    tc35874x_write_reg2(i2c, 0x0006, udata->fifo_level);  /* FIFO level */
    svalue = (fmt->fourcc == V4L2_PIX_FMT_RGB24) ? 0x0030 : 0x0060;
    tc35874x_write_reg2(i2c, 0x0008, svalue);  /* Data format */
    svalue = (fmt->fourcc == V4L2_PIX_FMT_RGB24) ? 3 * frmsize->width : 2 * frmsize->width;
    tc35874x_write_reg2(i2c, 0x0022, svalue);  /* Word count */

    /* CSI Tx Phy */
    tc35874x_write_reg4(i2c, 0x0140, 0x00000000);
    tc35874x_write_reg4(i2c, 0x0144, 0x00000000); /* lane 0 */
    if (udata->num_csi_lanes > 1)
        tc35874x_write_reg4(i2c, 0x0148, 0x00000000); /* lane 1 */
    else
        tc35874x_write_reg4(i2c, 0x0148, 0x00000001); /* lane 1 */

    if (udata->num_csi_lanes > 2)
        tc35874x_write_reg4(i2c, 0x014c, 0x00000000); /* lane 2 */
    else
        tc35874x_write_reg4(i2c, 0x014c, 0x00000001); /* lane 2 */

    if (udata->num_csi_lanes > 3)
        tc35874x_write_reg4(i2c, 0x0150, 0x00000000); /* lane 3 */
    else
        tc35874x_write_reg4(i2c, 0x0150, 0x00000001); /* lane 3 */

    /* CSI Tx PPI */
    tc35874x_write_reg4(i2c, 0x0210, 0x00002C00);
    tc35874x_write_reg4(i2c, 0x0214, 0x00000005);
    tc35874x_write_reg4(i2c, 0x0218, 0x00002004);
    tc35874x_write_reg4(i2c, 0x021c, 0x00000003);
    tc35874x_write_reg4(i2c, 0x0220, 0x00000705);
    tc35874x_write_reg4(i2c, 0x0224, 0x00004988);
    tc35874x_write_reg4(i2c, 0x0228, 0x0000000A);
    tc35874x_write_reg4(i2c, 0x022c, 0x00000004);
    value = ~(0xffffffff << (udata->num_csi_lanes + 1));
    tc35874x_write_reg4(i2c, 0x0234, value);
    tc35874x_write_reg4(i2c, 0x0238, 0x00000001);
    tc35874x_write_reg4(i2c, 0x0204, 0x00000001);

    /* CSI Start */
    tc35874x_write_reg4(i2c, 0x0518, 0x00000001);
    value = 0xa3008081 + ((udata->num_csi_lanes - 1) << 1);
    tc35874x_write_reg4(i2c, 0x0500, value);

    /* Parallel IN start */
    tc35874x_write_reg2(i2c, 0x0032, 0x0000);
    tc35874x_write_reg2(i2c, 0x0004, 0x8164);

#if DEBUG_DUMP_REGISTER
    bridge_debug_dump(i2c);
#endif
    return 0;
}

static int bridge_stop(struct tc35874x_i2c_dev_info *i2c, void *data)
{
    tc35874x_write_reg4(i2c, 0x0518, 0x00000000);

    tc35874x_write_reg2(i2c, 0x0032, 0x8000);
    usleep(BRIDGE_RESET_DELAY);
    tc35874x_write_reg2(i2c, 0x0004, 0x0004);
    tc35874x_write_reg2(i2c, 0x0032, 0xc000);

    return 0;
}

#ifdef CONFIG_REDCARPET_APBE
static struct cdsi_config CDSI_CONFIG =
#else
static struct mhb_cdsi_config CDSI_CONFIG =
#endif
{
    /* Common */
    .direction = 0,
    .mode = 0x01,            /* TSB_CDSI_MODE_CSI */
    .tx_num_lanes = 4,
    .rx_num_lanes = 0,       /* variable */
    .tx_mbits_per_lane = 0,  /* variable */
    .rx_mbits_per_lane = 0,  /* variable */
    /* RX only */
    .hs_rx_timeout = 0xffffffff,
    /* TX only */
    .framerate = 0, /* variable */
    .pll_frs = 0,
    .pll_prd = 0,
    .pll_fbd = 26,
    .width = 0,  /* variable */
    .height = 0, /* variable */
    .bpp = 0,    /* variable */
    .bta_enabled = 0,
    .continuous_clock = 0,
    .blank_packet_enabled = 0,
    .video_mode = 0,
    .color_bar_enabled = 0,
    /* CSI only */
    /* DSI only */
    /* Video Mode only */
    /* Command Mode only */
};

#ifdef CONFIG_REDCARPET_APBE
/* APBA IPC Utility functions */
static struct cdsi_dev *g_cdsi_dev;

static void generic_csi_init(struct cdsi_dev *dev)
{
    cdsi_initialize_rx(dev, &CDSI_CONFIG);
}

const static struct camera_sensor generic_sensor = {
    .cdsi_sensor_init = generic_csi_init,
};
#else

/* MHB Ops */
static int _mhb_cdsi_cam_apbe_state(int state)
{
    static struct device *slave_pwr_ctrl;

    slave_pwr_ctrl = device_open(DEVICE_TYPE_SLAVE_PWRCTRL_HW, MHB_ADDR_CDSI1);
    if (!slave_pwr_ctrl) {
       CAM_ERR("ERROR: Failed to open SLAVE Power Control\n");
       return -ENODEV;
    }

    device_slave_pwrctrl_send_slave_state(slave_pwr_ctrl, state);
    device_close(slave_pwr_ctrl);

    return 0;
}

static int _mhb_cdsi_camera_wait_for_response(struct dev_private_s *cam_device)
{
    int result = -1;
    struct timespec expires;

    if (clock_gettime(CLOCK_REALTIME, &expires)) {
        return -EBADF;
    }

    uint64_t new_ns = timespec_to_nsec(&expires);
    new_ns += MHB_CDSI_OP_TIMEOUT_MS * NSEC_PER_MSEC;
    nsec_to_timespec(new_ns, &expires);

    pthread_mutex_lock(&cam_device->mutex);
    result = pthread_cond_timedwait(&cam_device->cond, &cam_device->mutex, &expires);
    pthread_mutex_unlock(&cam_device->mutex);

    if (result) {
        /* timeout or other erros */
        CAM_ERR("ERROR: wait error %d\n", result);
        return -ETIME;
    }

    return result;
}

static int _mhb_cdsi_send_unipro_cam_mode_req(struct dev_private_s *cam_device,
                                             uint8_t fastmode)
{
    int result = -1;
    struct mhb_hdr hdr;
    struct mhb_unipro_control_req req;

    if (fastmode) {
        req.gear.tx = 2;
        req.gear.rx = 2;
        req.gear.pwrmode = (UNIPRO_FAST_MODE << 4)|UNIPRO_FAST_MODE;
        req.gear.series = UNIPRO_HS_SERIES_A;
    } else {
        req.gear.tx = 1;
        req.gear.rx = 1;
        req.gear.pwrmode = (UNIPRO_SLOW_MODE << 4)|UNIPRO_SLOW_MODE;
        req.gear.series = UNIPRO_HS_SERIES_A;
    }

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_UNIPRO;
    hdr.type = MHB_TYPE_UNIPRO_CONTROL_REQ;

    result = device_mhb_send(cam_device->mhb_device, &hdr, (uint8_t *)&req, sizeof(req), 0);
    if (result) {
        CAM_ERR("ERROR: failed to send unipro %d req. result %d\n", fastmode, result);
        return result;
    }

    result = _mhb_cdsi_camera_wait_for_response(cam_device);
    if (result) {
        CAM_ERR("ERROR: failed waiting for config response result %d\n", result);
    }

    return result;


}

static int _mhb_cdsi_send_config_wait_req(struct dev_private_s *cam_device,
                                          uint8_t *cfg, size_t cfg_size)
{
    struct mhb_hdr hdr;
    int result = -1;

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_CDSI1;
    hdr.type = MHB_TYPE_CDSI_CONFIG_REQ;

    result = device_mhb_send(cam_device->mhb_device, &hdr, (uint8_t *)cfg, cfg_size, 0);
    if (result) {
        CAM_ERR("ERROR: failed to send mhb config req. result %d\n", result);
        return result;
    }

    result = _mhb_cdsi_camera_wait_for_response(cam_device);
    if (result) {
        CAM_ERR("ERROR: failed waiting for config response result %d\n", result);
    }

    return result;
}

static int _mhb_cdsi_send_unconfig_wait_req(struct dev_private_s *cam_device)
{
    struct mhb_hdr hdr;
    int result = 0;

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_CDSI1;
    hdr.type = MHB_TYPE_CDSI_UNCONFIG_REQ;

    result = device_mhb_send(cam_device->mhb_device, &hdr, NULL, 0, 0);
    if (result) {
        CAM_ERR("ERROR: failed to send mhb unconfig req. result %d\n", result);
        return result;
    }

    result = _mhb_cdsi_camera_wait_for_response(cam_device);
    if (result) {
        CAM_ERR("ERROR: failed waiting for unconfig response result %d\n", result);
    }

    return result;
}


static int _mhb_cdsi_control_wait_req(struct dev_private_s *cam_device,
                                    uint8_t command)
{
    struct mhb_hdr hdr;
    struct mhb_cdsi_control_req req;
    int result = 0;

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_CDSI1;
    hdr.type = MHB_TYPE_CDSI_CONTROL_REQ;

    req.command = command;

    result = device_mhb_send(cam_device->mhb_device, &hdr, (uint8_t *)&req, sizeof(req), 0);
    if (result) {
        CAM_ERR("ERROR: failed to send mhb ctrl cmd %d result %d\n", command, result);
        return result;
    }

    result = _mhb_cdsi_camera_wait_for_response(cam_device);
    if (result) {
        CAM_ERR("ERROR: failed waiting for ctrl response cmd %d result %d\n", command, result);
    }

    return result;
}

static void _mhb_cdsi_camera_signal_response(struct dev_private_s *cam_device)
{
    pthread_mutex_lock(&cam_device->mutex);
    pthread_cond_signal(&cam_device->cond);
    pthread_mutex_unlock(&cam_device->mutex);
}

static int _mhb_cdsi_camera_handle_msg(struct device *dev,
    struct mhb_hdr *hdr, uint8_t *payload, size_t payload_length)
{
    /* Assume an error */
    int error = -1;
    CAM_DBG("addr=0x%02x, type=0x%02x, result=0x%02x\n",
            hdr->addr, hdr->type, hdr->result);

    struct dev_private_s *cam_device = &s_device;
    if (!cam_device) {
        return -ENODEV;
    }

    switch (hdr->addr) {
    case MHB_ADDR_UNIPRO:
        switch (hdr->type) {
        case MHB_TYPE_UNIPRO_CONTROL_RSP:
            _mhb_cdsi_camera_signal_response(cam_device);
            error = 0;
            break;
        }
        break;
    case MHB_ADDR_CDSI1:
        switch (hdr->type) {
        case MHB_TYPE_CDSI_UNCONFIG_RSP:
        case MHB_TYPE_CDSI_CONFIG_RSP:
        case MHB_TYPE_CDSI_CONTROL_RSP:
            _mhb_cdsi_camera_signal_response(cam_device);
            error = 0;
            break;
        case MHB_TYPE_CDSI_WRITE_CMDS_RSP:
        case MHB_TYPE_CDSI_READ_CMDS_RSP:
        case MHB_TYPE_CDSI_STATUS_RSP:
            error = 0;
        }
        break;
    }

    if (error) {
        CAM_ERR("ERROR: unexpected rsp: addr=%d type=%d state=%d\n",
                hdr->addr, hdr->type, cam_device->state);
    }

    return 0;
}
#endif

/* CAMERA_EXT operations */
static int _power_on(struct device *dev)
{
    DEV_TO_PRIVATE(dev, dev_priv);

    CAM_DBG("camera_ext_s10 state %d\n", dev_priv->state);

    if (dev_priv->state == CAMERA_S10_STATE_OFF) {
#ifndef CONFIG_REDCARPET_APBE
        /* P1 Config */
        /* Request APBE on. */
        _mhb_cdsi_cam_apbe_state(1);
        usleep(300000); // TODO: Register with apbe-ctrl listener.
        gpio_direction_out(dev_priv->cam_enable, 1);
        gpio_direction_out(dev_priv->bridge_enable, 1);
        usleep(300000);
#endif
    } else {
        CAM_DBG("camera already on %d\n", dev_priv->state);
    }

    if (tc35874x_run_command(&bridge_on, NULL) != 0) {
        CAM_ERR("Failed to run bridge_on commands\n");
        return -1;
    }
    dev_priv->state = CAMERA_S10_STATE_ON;
    return 0;
}

static int _stream_off(struct device *dev);

static int _power_off(struct device *dev)
{
    DEV_TO_PRIVATE(dev, dev_priv);
    CAM_DBG(" state %d\n", dev_priv->state);

    if (dev_priv->state == CAMERA_S10_STATE_STREAMING) {
        CAM_DBG("stop streaming before powering off\n");
        _stream_off(dev);
    }

    /* TODO: REMOVE :Ignore power off for now */
    return 0;

    if (dev_priv->state == CAMERA_S10_STATE_OFF) {
        CAM_ERR("camera already off\n");
    }
    else if (tc35874x_run_command(&bridge_off, NULL) != 0) {
        CAM_ERR("Failed to run bridge_off commands\n");
        return -1;
    }

#ifndef CONFIG_REDCARPET_APBE
    gpio_direction_out(dev_priv->cam_enable, 0);
    gpio_direction_out(dev_priv->bridge_enable, 0);
    /* Request APBE off. */
    _mhb_cdsi_cam_apbe_state(0);
#endif

    dev_priv->state = CAMERA_S10_STATE_OFF;

    return 0;
}

static int _stream_on(struct device *dev)
{
    DEV_TO_PRIVATE(dev, dev_priv);
    CAM_DBG(" state %d\n", dev_priv->state);

    if (dev_priv->state != CAMERA_S10_STATE_ON)
        return -1;

    const struct camera_ext_format_db *db = camera_ext_get_format_db();
    const struct camera_ext_format_user_config *cfg = camera_ext_get_user_config();
    const struct camera_ext_format_node *fmt;
    const struct camera_ext_frmsize_node *frmsize;
    const struct camera_ext_frmival_node *ival;

    fmt = get_current_format_node(db, cfg);
    if (fmt == NULL) {
        CAM_ERR("Failed to get current format\n");
        return -1;
    }

    if (fmt->fourcc != V4L2_PIX_FMT_RGB24 &&
        fmt->fourcc != V4L2_PIX_FMT_UYVY) {
        CAM_ERR("Unsupported format 0x%x\n", fmt->fourcc);
        return -1;
    }

    frmsize = get_current_frmsize_node(db, cfg);
    if (frmsize == NULL) {
        CAM_ERR("Failed to get current frame size\n");
        return -1;
    }

    ival = get_current_frmival_node(db, cfg);
    if (ival == NULL) {
        CAM_ERR("Failed to get current frame interval\n");
        return -1;
    }

    const struct user_data_t *udata;
    udata = (const struct user_data_t *)ival->user_data;
    if (udata == NULL) {
        CAM_ERR("Failed to get user data\n");
        return -1;
    }

    CDSI_CONFIG.width = frmsize->width;
    CDSI_CONFIG.height = frmsize->height;
    CDSI_CONFIG.rx_num_lanes = udata->num_csi_lanes;

    float fps = (float)(ival->denominator) /
        (float)(ival->numerator);
    CDSI_CONFIG.framerate = roundf(fps);

    CDSI_CONFIG.tx_mbits_per_lane = 500000000;
    CDSI_CONFIG.rx_mbits_per_lane = 500000000;

    /* Fill in the rest of CSDI_CONGIG field */
    if (fmt->fourcc == V4L2_PIX_FMT_RGB24) {
        CDSI_CONFIG.bpp = 24;
    } else if (fmt->fourcc == V4L2_PIX_FMT_UYVY) {
        CDSI_CONFIG.bpp = 16;
    }

#ifdef CONFIG_REDCARPET_APBE
    /* start CSI TX on APBA */
    if (cdsi_apba_cam_tx_start(&CDSI_CONFIG) != 0) {
        CAM_ERR("Failed to configure CDSI on APBA\n");
        return -1;
    }

    g_cdsi_dev = csi_initialize((struct camera_sensor *)&generic_sensor, TSB_CDSI1, TSB_CDSI_RX);
    if (!g_cdsi_dev) {
        CAM_ERR("failed to initialize CSI RX\n");
        goto stop_csi_tx;
    }
#else
    /* Send the configuration to the APBE. */
    if (_mhb_cdsi_send_config_wait_req(dev_priv, (uint8_t*)&CDSI_CONFIG, sizeof(CDSI_CONFIG))) {
        CAM_ERR("ERROR: send config failed\n");
        goto stop_csi_tx;
    }
    if (_mhb_cdsi_control_wait_req(dev_priv, MHB_CDSI_COMMAND_START)) {
        CAM_ERR("ERROR: start failed\n");
        goto stop_csi_tx;
    }

    if (_mhb_cdsi_send_unipro_cam_mode_req(dev_priv, 1)) {
            lldbg("ERROR: unipro gear cam req failed\n");
            goto stop_csi_tx;
    }
#endif
    /* setup the bridge chip and start streaming data */
    if (tc35874x_run_command(&bridge_setup_and_start, (void *)cfg) != 0) {
        CAM_ERR("Failed to run setup & start commands\n");
        goto stop_csi_tx;
    }
    dev_priv->state = CAMERA_S10_STATE_STREAMING;

    return 0;

stop_csi_tx:
#ifdef CONFIG_REDCARPET_APBE
    cdsi_apba_cam_tx_stop();
#else
    _mhb_cdsi_send_unipro_cam_mode_req(dev_priv, 0);
    _mhb_cdsi_control_wait_req(dev_priv, MHB_CDSI_COMMAND_STOP);
    _mhb_cdsi_send_unconfig_wait_req(dev_priv);
#endif

    return -1;
}

static int _stream_off(struct device *dev)
{
    DEV_TO_PRIVATE(dev, dev_priv);
    CAM_DBG(" state %d\n", dev_priv->state);

    if (dev_priv->state != CAMERA_S10_STATE_STREAMING)
        return -1;

    if (tc35874x_run_command(&bridge_stop, NULL) != 0)
        return -1;

#ifdef CONFIG_REDCARPET_APBE
    cdsi_apba_cam_tx_stop();

    if (g_cdsi_dev) {
        csi_uninitialize(g_cdsi_dev);
        g_cdsi_dev = NULL;
    }
#else
    _mhb_cdsi_control_wait_req(dev_priv, MHB_CDSI_COMMAND_STOP);
    _mhb_cdsi_send_unconfig_wait_req(dev_priv);
    _mhb_cdsi_send_unipro_cam_mode_req(dev_priv, 0);
#endif

    /* Ling Long's temp fix for stream on/off stuck */
    _power_off(dev);
    _power_on(dev);

    dev_priv->state = CAMERA_S10_STATE_ON;
    return 0;
}

static int int2_irq_event(int irq, FAR void *context)
{
    static uint8_t keycode = 0x01;
    gpio_mask_irq(s_device.cam_int2);
    s10p_report_button(keycode);
    keycode = 0x01 - keycode;
    gpio_clear_interrupt(s_device.cam_int2);
    gpio_unmask_irq(s_device.cam_int2);
    return 0;
}

static int _dev_open(struct device *dev)
{
    struct device_resource *res;
    CAM_DBG("camera_ext_s10\n");

    /* Only initialize I2C once in life */
    if (s_device.tc_i2c_info.i2c == NULL) {
        /* initialize once */
        s_device.tc_i2c_info.i2c = up_i2cinitialize(CONFIG_CAMERA_I2C_BUS);

        if (s_device.tc_i2c_info.i2c == NULL) {
            CAM_ERR("Failed to initialize I2C\n");
            return -1;
        }
        s_device.tc_i2c_info.i2c_addr = CONFIG_CAMERA_TC35874X_I2C_ADDR;

        if (tc35874x_start_i2c_control_thread(&s_device.tc_i2c_info) != 0) {
            CAM_ERR("Failed to start tc35874x i2c ctrl thread\n");
            up_i2cuninitialize(s_device.tc_i2c_info.i2c);
            s_device.tc_i2c_info.i2c = NULL;
            return -1;
        }
    }

    if (s_device.s10_i2c_info.i2c == NULL) {
        /* initialize once */
        s_device.s10_i2c_info.i2c = up_i2cinitialize(CONFIG_CAMERA_I2C_BUS);

        if (s_device.s10_i2c_info.i2c == NULL) {
            CAM_ERR("Failed to initialize I2C\n");
            return -1;
        }
        s_device.s10_i2c_info.i2c_addr = CONFIG_CAMERA_S10_I2C_ADDR;
        s10p_set_i2c(&s_device.s10_i2c_info);

        s_device.state = CAMERA_S10_STATE_OFF;
        s_device.cdsi_instance = MHB_CDSI_CAM_INSTANCE;
        pthread_mutex_init(&s_device.mutex, NULL);
        pthread_cond_init(&s_device.cond, NULL);
    }

    device_driver_set_private(dev, (void*)&s_device);

    camera_ext_register_format_db(&_db);
    camera_ext_register_control_db(&s10p_ctrl_db);

#ifndef CONFIG_REDCARPET_APBE
    res = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO, "CAM_ENABLE");
    if (!res) {
        CAM_ERR("failed to get cam_enable gpio\n");
        return -EINVAL;
    }
    s_device.cam_enable = res->start;

    res = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO, "BRIDGE_ENABLE");
    if (!res) {
        CAM_ERR("failed to get bridge_enable gpio\n");
        return -EINVAL;
    }
    s_device.bridge_enable = res->start;
#endif

    res = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO, "CAM_INT2");
    if (!res) {
        CAM_ERR("failed to get cam_int2 gpio\n");
        return -EINVAL;
    }
    s_device.cam_int2 = res->start;

    res = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO, "CAM_INT1");
    if (!res) {
        CAM_ERR("failed to get cam_int2 gpio\n");
        s_device.cam_int1 = -1;
    } else {
        s_device.cam_int1 = res->start;
    }

    res = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO, "CAM_INT0");
    if (!res) {
        CAM_ERR("failed to get cam_int2 gpio\n");
    } else {
        s_device.cam_int0 = -1;
        s_device.cam_int0 = res->start;
    }

    gpio_direction_in(s_device.cam_int2);
    gpio_mask_irq(s_device.cam_int2);
    set_gpio_triggering(s_device.cam_int2, IRQ_TYPE_EDGE_RISING);
    gpio_irqattach(s_device.cam_int2, int2_irq_event);
    gpio_clear_interrupt(s_device.cam_int2);
    gpio_unmask_irq(s_device.cam_int2);

#ifndef CONFIG_REDCARPET_APBE
    if (s_device.mhb_device) {
        CAM_ERR("ERROR: already opened.\n");
        return 0;
    }

    s_device.mhb_device = device_open(DEVICE_TYPE_MHB, MHB_ADDR_CDSI1);
    if (!s_device.mhb_device) {
        CAM_ERR("ERROR: failed to open MHB device.\n");
        return -ENOENT;
    }

    device_mhb_register_receiver(s_device.mhb_device, MHB_ADDR_CDSI1,
                                 _mhb_cdsi_camera_handle_msg);

    device_mhb_register_receiver(s_device.mhb_device, MHB_ADDR_UNIPRO,
                                 _mhb_cdsi_camera_handle_msg);

#endif

    return 0;
}

static void _dev_close(struct device *dev)
{
    //ensure power off camera
    CAM_DBG(" camera_ext_s10");
    struct dev_private_s *dev_priv = (struct dev_private_s *)
        device_driver_get_private(dev);
    if (dev_priv->state == CAMERA_S10_STATE_STREAMING) {
        CAM_DBG("stop streaming before close\n");
        _stream_off(dev);
    }
    if (dev_priv->state == CAMERA_S10_STATE_ON) {
        CAM_DBG("power off before close\n");
        _power_off(dev);
    }

#ifndef CONFIG_REDCARPET_APBE
    if (dev_priv->mhb_device) {
        device_mhb_unregister_receiver(dev_priv->mhb_device, MHB_ADDR_CDSI1,
                                       _mhb_cdsi_camera_handle_msg);
    }

    device_close(dev_priv->mhb_device);
    dev_priv->mhb_device = NULL;
#endif
}


static struct device_camera_ext_dev_type_ops _camera_ext_type_ops = {
    .register_event_cb = camera_ext_register_event_cb,
    .power_on          = _power_on,
    .power_off         = _power_off,
    .stream_on         = _stream_on,
    .stream_off        = _stream_off,
    .input_enum        = camera_ext_input_enum,
    .input_get         = camera_ext_input_get,
    .input_set         = camera_ext_input_set,
    .format_enum       = camera_ext_format_enum,
    .format_get        = camera_ext_format_get,
    .format_set        = camera_ext_format_set,
    .frmsize_enum      = camera_ext_frmsize_enum,
    .frmival_enum      = camera_ext_frmival_enum,
    .stream_set_parm   = camera_ext_stream_set_parm,
    .stream_get_parm   = camera_ext_stream_get_parm,
    .ctrl_get_cfg      = camera_ext_ctrl_get_cfg,
    .ctrl_get          = camera_ext_ctrl_get,
    .ctrl_set          = camera_ext_ctrl_set,
    .ctrl_try          = camera_ext_ctrl_try,
};

static struct device_driver_ops camera_ext_driver_ops = {
    .open     = _dev_open,
    .close    = _dev_close,
    .type_ops = &_camera_ext_type_ops,
};

struct device_driver cam_ext_s10p_driver = {
    .type = DEVICE_TYPE_CAMERA_EXT_HW,
    .name = "Altek Sunny 10P",
    .desc = "Sunny 10P Camera",
    .ops  = &camera_ext_driver_ops,
};
