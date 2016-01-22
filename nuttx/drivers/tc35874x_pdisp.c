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

#include <errno.h>
#include <debug.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <apps/ice/cdsi.h>
#include <nuttx/config.h>
#include <nuttx/device_cam_ext.h>
#include <nuttx/i2c.h>
#include <nuttx/math.h>

#include <arch/chip/cdsi.h>
#include <arch/chip/cdsi_config.h>

#include "camera_ext.h"
#include "tc35874x-util.h"

/*
 * This is the reference greybus driver for Toshiba Paral to CSI-2 bridge
 * chip (TC35874X series).
 *
 * Driver is implemented to work with Toshiba's evaluation board. GPIOs
 * and regulators control for the chip is done by eval board itself so
 * those are not implemented in this driver.
 *
 * The parameters used in this driver are to work with Raspberry Pi2
 * parallel display output.
 */
#define DEBUG_DUMP_REGISTER 1
#define BRIDGE_RESET_DELAY 100000 /* us */

#define BRIDGE_I2C_ADDR 0x0E

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

/* BEGIN - Supported Format definitions */
struct user_data_t {
    uint16_t pll1;
    uint16_t pll2;
    uint16_t fifo_level;
    uint32_t num_csi_lanes;
};

static struct user_data_t _user_data[] = {
    {
        /* FHD @ 15 fps
         * Input pclk 41 Mhz, hblank 600 pclk */
        .pll1 = 0x30c8,
        .pll2 = 0x0213,
        .fifo_level = 150,
        .num_csi_lanes = 1,
    },
    {
        /* for qHD @ 30 fps
         * Input pclk 23 Mhz, hblank 300 pclk */
        .pll1 = 0x30fa,
        .pll2 = 0x0613,
        .fifo_level = 180,
        .num_csi_lanes = 1,
    },
    {
        /* VGA @ 60 fps
         * Input pclk 27Mhz, hblank 200 pclk */
        .pll1 = 0x3096,
        .pll2 = 0x0213,
        .fifo_level = 120,
        .num_csi_lanes = 1,
    },
};

static const struct camera_ext_frmival_node _frmival_fhd[] = {
    {
        .numerator = 1,
        .denominator = 15,
        .user_data = &_user_data[0],
    },
};

static const struct camera_ext_frmival_node _frmival_qhd[] = {
    {
        .numerator = 1,
        .denominator = 30,
        .user_data = &_user_data[1],
    },
};

static const struct camera_ext_frmival_node _frmival_vga[] = {
    {
        .numerator = 1,
        .denominator = 60,
        .user_data = &_user_data[2],
    },
};

static const struct camera_ext_frmsize_node _frmsizes_rgb888[] = {
    {
        .width = 1920,
        .height = 1080,
        .num_frmivals = ARRAY_SIZE(_frmival_fhd),
        .frmival_nodes = _frmival_fhd,
    },
    {
        .width = 960,
        .height = 540,
        .num_frmivals = ARRAY_SIZE(_frmival_qhd),
        .frmival_nodes = _frmival_qhd,
    },
    {
        .width = 640,
        .height = 480,
        .num_frmivals = ARRAY_SIZE(_frmival_vga),
        .frmival_nodes = _frmival_vga,
    },
};

static const struct camera_ext_format_node _formats[] = {
    {
        .name = "RGB888",
        .fourcc = V4L2_PIX_FMT_RGB24,
        .depth = 24,
        .num_frmsizes = ARRAY_SIZE(_frmsizes_rgb888),
        .frmsize_nodes = _frmsizes_rgb888,
    },
};

static const struct camera_ext_input_node _inputs[] = {
    {
        .name = "TC35874X",
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

typedef enum {
    OFF = 0,
    ON,
    STREAMING
} dev_status_t;

struct dev_private_s
{
    dev_status_t status;
    struct tc35874x_i2c_dev_info i2c_info;
};

#define DEV_TO_PRIVATE(dev_ptr, priv_ptr) struct dev_private_s *priv_ptr =\
        (struct dev_private_s *)device_driver_get_private(dev_ptr)

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

    if (fmt->fourcc != V4L2_PIX_FMT_RGB24) {
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
    tc35874x_write_reg2(i2c, 0x0008, 0x0030);  /* Data format */
    tc35874x_write_reg2(i2c, 0x0022, 3 * frmsize->width);  /* Word count */

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
    tc35874x_write_reg2(i2c, 0x0004, 0x0044);

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

/* APBA IPC Utility functions */
static struct cdsi_dev *g_cdsi_dev;

static struct cdsi_config CDSI_CONFIG = {
    /* Common */
    .mode = TSB_CDSI_MODE_CSI,
    .tx_num_lanes = 4,
    .rx_num_lanes = 0,        /* variable */
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

static void generic_csi_init(struct cdsi_dev *dev)
{
    cdsi_initialize_rx(dev, &CDSI_CONFIG);
}

const static struct camera_sensor generic_sensor = {
    .cdsi_sensor_init = generic_csi_init,
};

/* CAMERA_EXT operations */
static int _power_on(struct device *dev)
{
    DEV_TO_PRIVATE(dev, dev_priv);

    if (dev_priv->status == OFF) {
        if (tc35874x_run_command(&bridge_on, NULL) != 0) {
            CAM_ERR("Failed to run bridge_on commands\n");
        } else {
            dev_priv->status = ON;
            return 0;
        }
    } else {
        CAM_DBG("%s: status %d\n", __func__, dev_priv->status);
    }

    return -1;
}

static int _stream_off(struct device *dev);

static int _power_off(struct device *dev)
{
    DEV_TO_PRIVATE(dev, dev_priv);

    if (dev_priv->status == STREAMING) {
        CAM_DBG("stop streaming before powering off\n");
        _stream_off(dev);
    }

    if (dev_priv->status == OFF) {
        CAM_DBG("camera already off\n");
        return 0;
    }
    if (tc35874x_run_command(&bridge_off, NULL) != 0) {
        CAM_ERR("Failed to run bridge_off commands\n");
        return -1;
    }
    dev_priv->status = OFF;

    return 0;
}

static int _stream_on(struct device *dev)
{
    DEV_TO_PRIVATE(dev, dev_priv);

    if (dev_priv->status != ON)
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

    if (fmt->fourcc != V4L2_PIX_FMT_RGB24) {
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

    /* Fill in the rest of CSDI_CONGIG field */
    if (fmt->fourcc == V4L2_PIX_FMT_RGB24) {
        CDSI_CONFIG.tx_mbits_per_lane = 500000000;
        CDSI_CONFIG.rx_mbits_per_lane = 500000000;
        CDSI_CONFIG.bpp = 24;
    }

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

    /* setup the bridge chip and start streaming data */
    if (tc35874x_run_command(&bridge_setup_and_start, (void *)cfg) != 0) {
        CAM_ERR("Failed to run setup & start commands\n");
        goto stop_csi_tx;
    }
    dev_priv->status = STREAMING;

    return 0;

stop_csi_tx:
    cdsi_apba_cam_tx_stop();

    return -1;
}

static int _stream_off(struct device *dev)
{
    DEV_TO_PRIVATE(dev, dev_priv);

    if (dev_priv->status != STREAMING)
        return -1;

    if (tc35874x_run_command(&bridge_stop, NULL) != 0)
        return -1;

    dev_priv->status = ON;
    cdsi_apba_cam_tx_stop();

    if (g_cdsi_dev) {
        csi_uninitialize(g_cdsi_dev);
        g_cdsi_dev = NULL;
    }

    return 0;
}

static struct camera_ext_ctrl_db _ctrl_db = {
    .num_ctrls = 0,
};

static int _dev_open(struct device *dev)
{
    //static allocate the singleton instance
    static struct dev_private_s s_device;

    s_device.status = OFF;

    /* Only initialize I2C once in life */
    if (s_device.i2c_info.i2c == NULL) {
        /* initialize once */
        s_device.i2c_info.i2c = up_i2cinitialize(0);

        if (s_device.i2c_info.i2c == NULL) {
            CAM_ERR("Failed to initialize I2C\n");
            return -1;
        }
        s_device.i2c_info.i2c_addr = BRIDGE_I2C_ADDR;

        if (tc35874x_start_i2c_control_thread(&s_device.i2c_info) != 0) {
            up_i2cuninitialize(s_device.i2c_info.i2c);
            s_device.i2c_info.i2c = NULL;
        }
    }

    device_driver_set_private(dev, (void*)&s_device);

    camera_ext_register_format_db(&_db);
    camera_ext_register_control_db(&_ctrl_db);

    return 0;
}

static void _dev_close(struct device *dev)
{
    //ensure power off camera
    struct dev_private_s *dev_priv = (struct dev_private_s *)
        device_driver_get_private(dev);
    if (dev_priv->status == STREAMING) {
        CAM_DBG("stop streaming before close\n");
        _stream_off(dev);
    }
    if (dev_priv->status == ON) {
        CAM_DBG("power off before close\n");
        _power_off(dev);
    }
}

static struct device_camera_ext_dev_type_ops _camera_ext_type_ops = {
    .power_on        = _power_on,
    .power_off       = _power_off,
    .stream_on       = _stream_on,
    .stream_off      = _stream_off,
    .input_enum      = camera_ext_input_enum,
    .input_get       = camera_ext_input_get,
    .input_set       = camera_ext_input_set,
    .format_enum     = camera_ext_format_enum,
    .format_get      = camera_ext_format_get,
    .format_set      = camera_ext_format_set,
    .frmsize_enum    = camera_ext_frmsize_enum,
    .frmival_enum    = camera_ext_frmival_enum,
    .stream_set_parm = camera_ext_stream_set_parm,
    .stream_get_parm = camera_ext_stream_get_parm,
    .ctrl_get_cfg    = camera_ext_ctrl_get_cfg,
    .ctrl_get        = camera_ext_ctrl_get,
    .ctrl_set        = camera_ext_ctrl_set,
    .ctrl_try        = camera_ext_ctrl_try,
};

static struct device_driver_ops camera_ext_driver_ops = {
    .open     = _dev_open,
    .close    = _dev_close,
    .type_ops = &_camera_ext_type_ops,
};

struct device_driver cam_ext_tc35874x_pdisp_driver = {
    .type = DEVICE_TYPE_CAMERA_EXT_HW,
    .name = "toshiba_tc35874x_pdisp",
    .desc = "Display Parallel to CSI bridge",
    .ops  = &camera_ext_driver_ops,
};
