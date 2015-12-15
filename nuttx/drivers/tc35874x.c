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
 * This driver provides greybus camera_ext emulation by using internal
 * test pattern generation on TC35874X chip.
 */
#define DEBUG_DUMP_REGISTER 1
#define BRIDGE_RESET_DELAY 100000 /* us */

#define BRIDGE_I2C_ADDR 0x0E

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

/* TODO: move below macro to camera_ext.h */
#define v4l2_fourcc(a, b, c, d) \
    ((uint32_t)(a) | ((uint32_t)(b) << 8)               \
     | ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))

/* BEGIN - Supported Format definitions */
static const struct camera_ext_frmival_node frmival_rgb888_vga[] = {
    {
        .raw_data = {
            .type = cpu_to_le32(CAM_EXT_FRMIVAL_TYPE_DISCRETE),
            .discrete = {
                .numerator = cpu_to_le32(60),
                .denominator = cpu_to_le32(1),
            }
        },
        .user_data = NULL,
    },
};

static const struct camera_ext_frmsize_node _frmsizes_rgb888[] = {
    {
        .raw_data = {
            .type = cpu_to_le32(CAM_EXT_FRMSIZE_TYPE_DISCRETE),
            .discrete = {
                .width = cpu_to_le32(640),
                .height = cpu_to_le32(480),
            },
        },
        .num_frmivals = ARRAY_SIZE(frmival_rgb888_vga),
        .frmival_nodes = frmival_rgb888_vga,
    },
};

static const struct camera_ext_format_node _formats[] = {
    {
        .raw_data = {
            .name = {'R', 'G', 'B', '8', '8', '8', 0},
            .fourcc = cpu_to_le32(v4l2_fourcc('R', 'G', 'B', '3')),
            .depth = cpu_to_le32(24),
        },
        .num_frmsizes = ARRAY_SIZE(_frmsizes_rgb888),
        .frmsize_nodes = _frmsizes_rgb888,
    },
};

static const struct camera_ext_input_node _inputs[] = {
    {
        .raw_data = {
            .index = cpu_to_le32(0),
            .name = {'T', 'C', '3', '5', '8', '7', '4', 'X',  0},
            .type = cpu_to_le32(CAM_EXT_INPUT_TYPE_CAMERA),
            .status = cpu_to_le32(0),
        },
        .num_formats = ARRAY_SIZE(_formats),
        .format_nodes = _formats,
    },
};

static const struct gb_camera_ext_sensor_db _db = {
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
    struct gb_camera_ext_sensor_user_config cfg;
};

#define DEV_TO_PRIVATE(dev_ptr, priv_ptr) struct dev_private_s *priv_ptr =\
        (struct dev_private_s *)device_driver_get_private(dev_ptr)

/* Bridge register configurations */
#if DEBUG_DUMP_REGISTER
static int bridge_debug_dump(struct tc35874x_i2c_dev_info *i2c)
{
    /* system reset */
    tc35874x_read_reg2(i2c, 0x0002);

    /* PLL */
    tc35874x_read_reg2(i2c, 0x0016);
    tc35874x_read_reg2(i2c, 0x0018);

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
    /* put system in sleep */
    if (tc35874x_write_reg2(i2c, 0x0002, 0x0001) != 0)
        return -1;

    usleep(BRIDGE_RESET_DELAY);
    return 0;
}

static void set_color_rgb888(struct tc35874x_i2c_dev_info *i2c, int repeat,
                     uint16_t val0, uint16_t val1, uint16_t val2)
{
    int i;

    for (i = 0; i < repeat; i++) {
        tc35874x_write_reg2(i2c, 0x00e8, val0);
        tc35874x_write_reg2(i2c, 0x00e8, val1);
        tc35874x_write_reg2(i2c, 0x00e8, val2);
    }
}

static void setup_color_bar_rgb888(struct tc35874x_i2c_dev_info *i2c, int width, int height)
{
    tc35874x_write_reg2(i2c, 0x00e0, 0x8000);
    tc35874x_write_reg2(i2c, 0x00e2, 0x0800);  /* Line length = 2048 */
    tc35874x_write_reg2(i2c, 0x00e4, 0x0014);  /* 20 vertival blanking lines */

    int repeat = width / 8 / 2; /* 8 color, set 2 pixel at a time */

    set_color_rgb888(i2c, repeat, 0x0000, 0x0000, 0x0000); /* black */
    set_color_rgb888(i2c, repeat, 0x0000, 0x00ff, 0xff00); /* red */
    set_color_rgb888(i2c, repeat, 0xff00, 0x0000, 0x00ff); /* green */
    set_color_rgb888(i2c, repeat, 0xff00, 0x00ff, 0xffff); /* yellow */
    set_color_rgb888(i2c, repeat, 0x00ff, 0xff00, 0x0000); /* blue */
    set_color_rgb888(i2c, repeat, 0x00ff, 0xffff, 0xff00); /* pink */
    set_color_rgb888(i2c, repeat, 0xffff, 0xff00, 0x00ff); /* cyan */
    set_color_rgb888(i2c, repeat, 0xffff, 0xffff, 0xffff); /* white */

    tc35874x_write_reg2(i2c, 0x00e0, 0xc000 | (height - 1)); /* active line count */
}

static int bridge_setup_and_start(struct tc35874x_i2c_dev_info *i2c, void *data)
{
    struct gb_camera_ext_sensor_user_config *cfg =
        (struct gb_camera_ext_sensor_user_config *)data;
    struct camera_ext_format fmt;
    if (cam_ext_fill_current_format(&_db, cfg, &fmt) != 0) {
        CAM_ERR("Failed to get current format\n");
        return -1;
    }

    /* PLL */
    tc35874x_write_reg2(i2c, 0x0016, 0x5117);
    tc35874x_write_reg2(i2c, 0x0018, 0x0213);

    /* CSI Tx Phy */
    tc35874x_write_reg4(i2c, 0x0140, 0x00000000);
    tc35874x_write_reg4(i2c, 0x0144, 0x00000000);
    tc35874x_write_reg4(i2c, 0x0148, 0x00000000);
    tc35874x_write_reg4(i2c, 0x014c, 0x00000000);
    tc35874x_write_reg4(i2c, 0x0150, 0x00000000);

    /* CSI Tx PPI */
    tc35874x_write_reg4(i2c, 0x0210, 0x00002D00);
    tc35874x_write_reg4(i2c, 0x0214, 0x00000005);
    tc35874x_write_reg4(i2c, 0x0218, 0x00002004);
    tc35874x_write_reg4(i2c, 0x021c, 0x00000003);
    tc35874x_write_reg4(i2c, 0x0220, 0x00000606);
    tc35874x_write_reg4(i2c, 0x0224, 0x00004A00);
    tc35874x_write_reg4(i2c, 0x0228, 0x0000000A);
    tc35874x_write_reg4(i2c, 0x022c, 0x00000004);
    tc35874x_write_reg4(i2c, 0x0234, 0x0000001F);
    tc35874x_write_reg4(i2c, 0x0238, 0x00000001);
    tc35874x_write_reg4(i2c, 0x0204, 0x00000001);

    /* CSI Start */
    tc35874x_write_reg4(i2c, 0x0518, 0x00000001);
    tc35874x_write_reg4(i2c, 0x0500, 0xA30080A7); /* 4 lane */

    /* Format Control */
    tc35874x_write_reg2(i2c, 0x0008, 0x0001); /* Use data type */
    if (fmt.pixelformat == v4l2_fourcc('R', 'G', 'B', '3')) {
        tc35874x_write_reg2(i2c, 0x0050, 0x0024); /* CSI data type RGB888 */
        tc35874x_write_reg2(i2c, 0x0022, le32_to_cpu(fmt.width) * 3); /* byte count per line */
    } else {
        CAM_ERR("Unsupported format = %x\n", fmt.pixelformat);
        return -1;
    }
    setup_color_bar_rgb888(i2c, le32_to_cpu(fmt.width), le32_to_cpu(fmt.height));

#if DEBUG_DUMP_REGISTER
    bridge_debug_dump(i2c);
#endif
    return 0;
}

static int bridge_stop(struct tc35874x_i2c_dev_info *i2c, void *data)
{
    tc35874x_write_reg4(i2c, 0x0518, 0x00000000);
    tc35874x_write_reg2(i2c, 0x00e0, 0x0000);
    return 0;
}

/* APBA IPC Utility functions */
static struct cdsi_dev *g_cdsi_dev;

static struct cdsi_config CDSI_CONFIG = {
    /* Common */
    .mode = TSB_CDSI_MODE_CSI,
    .tx_num_lanes = 4,
    .rx_num_lanes = 4,
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

    struct camera_ext_format fmt;
    const struct camera_ext_frmival_node *ival;
    if (cam_ext_fill_current_format(&_db, &dev_priv->cfg, &fmt) != 0) {
        CAM_ERR("Failed to get current format\n");
        return -1;
    }

    ival = get_current_frmival_node(&_db, &dev_priv->cfg);
    if (ival == NULL) {
        CAM_ERR("Failed to get current frame interval\n");
        return -1;
    }

    /* Fill in the rest of CSDI_CONGIG field */
    if (fmt.pixelformat == v4l2_fourcc('R', 'G', 'B', '3')) {
        CDSI_CONFIG.tx_mbits_per_lane = 500000000;
        CDSI_CONFIG.rx_mbits_per_lane = 500000000;
        CDSI_CONFIG.width = le32_to_cpu(fmt.width);
        CDSI_CONFIG.height = le32_to_cpu(fmt.height);
        CDSI_CONFIG.bpp = 24;
        float fps = (float)(le32_to_cpu(ival->raw_data.discrete.numerator)) /
            (float)(le32_to_cpu(ival->raw_data.discrete.denominator));
        CDSI_CONFIG.framerate = roundf(fps);
    } else {
        CAM_ERR("Unsupported format %x\n", fmt.pixelformat);
        return -1;
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
    if (tc35874x_run_command(&bridge_setup_and_start, &dev_priv->cfg) != 0) {
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

static int _input_enum(struct device *dev,
                              struct camera_ext_input *input)
{
    int index = le32_to_cpu(input->index);

    if (index >= ARRAY_SIZE(_inputs))
        return -EFAULT;

    memcpy(input, &_inputs[index].raw_data, sizeof(*input));
    return 0;
}

static int _input_get(struct device *dev, int *input)
{
    DEV_TO_PRIVATE(dev, dev_priv);

    if (get_current_input_node(&_db, &dev_priv->cfg) == NULL)
        return -EFAULT;

    *input = dev_priv->cfg.input;
    return 0;
}

static int _input_set(struct device *dev, int index)
{
    DEV_TO_PRIVATE(dev, dev_priv);

    if (index >= ARRAY_SIZE(_inputs))
        return -EFAULT;

    dev_priv->cfg.input = index;

    return 0;
}

static int _format_enum(struct device *dev, struct camera_ext_fmtdesc *format)
{
    DEV_TO_PRIVATE(dev, dev_priv);
    struct camera_ext_format_node const *format_node;
    int index = le32_to_cpu(format->index);

    format_node = get_format_node(&_db, dev_priv->cfg.input, index);

    if (format_node == NULL)
        return -EFAULT;

    memcpy(format, &format_node->raw_data, sizeof(*format));

    return 0;
}

static int _format_get(struct device *dev, struct camera_ext_format *format)
{
    DEV_TO_PRIVATE(dev, dev_priv);

    return cam_ext_fill_current_format(&_db, &dev_priv->cfg, format);
}

static int _format_set(struct device *dev, struct camera_ext_format* format)
{
    DEV_TO_PRIVATE(dev, dev_priv);
    struct camera_ext_input_node const *input_node;

    input_node = get_current_input_node(&_db, &dev_priv->cfg);

    return cam_ext_set_current_format(input_node, &dev_priv->cfg, format);
}

static int _frmsize_enum(struct device *dev, struct camera_ext_frmsize* frmsize)
{
    DEV_TO_PRIVATE(dev, dev_priv);
    struct camera_ext_input_node const *input_node;
    int index = le32_to_cpu(frmsize->index);

    input_node = get_current_input_node(&_db, &dev_priv->cfg);

    return cam_ext_frmsize_enum(input_node, index, frmsize);
}

static int _frmival_enum(struct device *dev, struct camera_ext_frmival* frmival)

{
    DEV_TO_PRIVATE(dev, dev_priv);
    struct camera_ext_input_node const *input_node;
    int index = le32_to_cpu(frmival->index);

    input_node = get_current_input_node(&_db, &dev_priv->cfg);

    return cam_ext_frmival_enum(input_node, index, frmival);
}

static int _stream_set_parm(struct device *dev, struct camera_ext_streamparm *parm)
{
    DEV_TO_PRIVATE(dev, dev_priv);
    struct camera_ext_frmival_node const *frmival_node;

    if (dev_priv->status == STREAMING) {
        CAM_ERR("can not update stream param during streaming\n");
        return -EBUSY;
    }

    frmival_node = cam_ext_frmival_set(&_db, &dev_priv->cfg, parm);

    if (frmival_node == NULL) {
        CAM_ERR("failed to apply stream parm\n");
        return -EINVAL;
    }

    return 0;
}

static int _stream_get_parm(struct device *dev, struct camera_ext_streamparm *parm)
{
    DEV_TO_PRIVATE(dev, dev_priv);
    struct camera_ext_frmival_node const *frmival_node;

    frmival_node = get_current_frmival_node(&_db, &dev_priv->cfg);

    if (frmival_node == NULL) {
        CAM_ERR("failed to get current frmival node\n");
        return -EINVAL;
    }

    memset(parm, 0, sizeof(*parm));
    parm->type = cpu_to_le32(CAMERA_EXT_BUFFER_TYPE_VIDEO_CAPTURE);
    parm->capture.timeperframe.numerator= frmival_node->raw_data.discrete.numerator;
    parm->capture.timeperframe.denominator = frmival_node->raw_data.discrete.denominator;

    return 0;
}

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

    /* init default config */
    memset(&s_device.cfg, 0, sizeof(s_device.cfg));

    device_driver_set_private(dev, (void*)&s_device);

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
    .input_enum      = _input_enum,
    .input_get       = _input_get,
    .input_set       = _input_set,
    .format_enum     = _format_enum,
    .format_get      = _format_get,
    .format_set      = _format_set,
    .frmsize_enum    = _frmsize_enum,
    .frmival_enum    = _frmival_enum,
    .stream_set_parm = _stream_set_parm,
    .stream_get_parm = _stream_get_parm,
};

static struct device_driver_ops camera_ext_driver_ops = {
    .open     = _dev_open,
    .close    = _dev_close,
    .type_ops = &_camera_ext_type_ops,
};

struct device_driver cam_ext_tc35874x_driver = {
    .type = DEVICE_TYPE_CAMERA_EXT_HW,
    .name = "toshiba_tc35874x",
    .desc = "pDp to CSI bridge",
    .ops  = &camera_ext_driver_ops,
};
