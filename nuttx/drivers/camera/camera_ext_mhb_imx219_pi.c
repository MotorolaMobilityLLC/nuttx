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

#include <errno.h>
#include <debug.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nuttx/camera/camera_ext.h>
#include <nuttx/camera/camera_ext_meta.h>
#include <nuttx/camera/camera_ext_defs.h>
#include <nuttx/device.h>
#include <nuttx/device_cam_ext.h>
#include <nuttx/device_mhb_cam.h>
#include <nuttx/gpio.h>
#include <nuttx/math.h>
#include <nuttx/mhb/mhb_protocol.h>
#include <nuttx/mhb/mhb_csi_camera.h>

struct dev_private_s
{
    uint8_t rst_n;
    uint8_t led_en;
    uint8_t spi_sel;
};

struct dev_private_s s_data;

#define CAMERA_POWER_DELAY_US (500000)
#define CAMERA_SENSOR_I2C_ADDR 0x10

/*
 * This is the MHB(Mods Hi-Speed Bus) camera driver for
 * Sony IMS220 raw sensor
 *
 */

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

struct cam_i2c_reg_array {
    const uint16_t reg_addr;
    const uint8_t data;
};

struct cam_i2c_reg_setting {
    const uint16_t size;
    struct cam_i2c_reg_array const *regs;
};

static const struct cam_i2c_reg_array init_reg_array[] = {
	{0x0103, 0x01},
};

/* 3280 x 2464 */
static const struct cam_i2c_reg_array res0_array[] = {
    { 0x0103, 0x01, },
    { 0x0100, 0x00, },
    { 0x6620, 0x01, },
    { 0x6621, 0x01, },
    { 0x6622, 0x01, },
    { 0x6623, 0x01, },
    { 0x30EB, 0x0C, },
    { 0x30EB, 0x05, },
    { 0x300A, 0xFF, },
    { 0x300B, 0xFF, },
    { 0x30EB, 0x05, },
    { 0x30EB, 0x09, },
    { 0x0114, 0x01, },
    { 0x0128, 0x00, },
    { 0x012A, 0x18, },
    { 0x012B, 0x00, },
    { 0x0157, 0x00, },
    { 0x015A, 0x09, },
    { 0x015B, 0xBD, },
    { 0x0160, 0x09, },
    { 0x0161, 0xC1, },
    { 0x0162, 0x0F, },
    { 0x0163, 0xA0, },
    { 0x0164, 0x00, },
    { 0x0165, 0x04, },
    { 0x0166, 0x0C, },
    { 0x0167, 0xC3, },
    { 0x0168, 0x00, },
    { 0x0169, 0x04, },
    { 0x016A, 0x09, },
    { 0x016B, 0x93, },
    { 0x016C, 0x0C, },
    { 0x016D, 0xC0, },
    { 0x016E, 0x09, },
    { 0x016F, 0x90, },
    { 0x0170, 0x01, },
    { 0x0171, 0x01, },
    { 0x0174, 0x00, },
    { 0x0175, 0x00, },
    { 0x018C, 0x0A, },
    { 0x018D, 0x0A, },
    { 0x0301, 0x05, },
    { 0x0303, 0x01, },
    { 0x0304, 0x03, },
    { 0x0305, 0x03, },
    { 0x0306, 0x00, },
    { 0x0307, 0x39, },
    { 0x0309, 0x0A, },
    { 0x030B, 0x01, },
    { 0x030C, 0x00, },
    { 0x030D, 0x72, },
    { 0x0624, 0x0C, },
    { 0x0625, 0xD0, },
    { 0x0626, 0x09, },
    { 0x0627, 0xA0, },
    { 0x455E, 0x00, },
    { 0x471E, 0x4B, },
    { 0x4767, 0x0F, },
    { 0x4750, 0x14, },
    { 0x4540, 0x00, },
    { 0x47B4, 0x14, },
    { 0x4713, 0x30, },
    { 0x478B, 0x10, },
    { 0x478F, 0x10, },
    { 0x4793, 0x10, },
    { 0x4797, 0x0E, },
    { 0x479B, 0x0E, },
    { 0x0172, 0x03, },
    { 0x0157, 0xC8, },
    { 0x015A, 0x05, },
    { 0x015B, 0xDC, },
};

#if 0
static const struct cam_i2c_reg_array res1_array[] = {
    { 0x0103, 0x01, },
    { 0x0100, 0x00, },
    { 0x6620, 0x01, },
    { 0x6621, 0x01, },
    { 0x6622, 0x01, },
    { 0x6623, 0x01, },
    { 0x30EB, 0x0C, },
    { 0x30EB, 0x05, },
    { 0x300A, 0xFF, },
    { 0x300B, 0xFF, },
    { 0x30EB, 0x05, },
    { 0x30EB, 0x09, },
    { 0x0114, 0x01, },
    { 0x0128, 0x00, },
    { 0x012A, 0x18, },
    { 0x012B, 0x00, },
    { 0x0157, 0x00, },
    { 0x015A, 0x09, },
    { 0x015B, 0xBD, },
    { 0x0160, 0x06, },
    { 0x0161, 0xE3, },
    { 0x0162, 0x0D, },
    { 0x0163, 0x78, },
    { 0x0164, 0x00, },
    { 0x0165, 0x28, },
    { 0x0166, 0x0C, },
    { 0x0167, 0xA7, },
    { 0x0168, 0x00, },
    { 0x0169, 0x20, },
    { 0x016A, 0x09, },
    { 0x016B, 0x7F, },
    { 0x016C, 0x06, },
    { 0x016D, 0x40, },
    { 0x016E, 0x04, },
    { 0x016F, 0xB0, },
    { 0x0170, 0x01, },
    { 0x0171, 0x01, },
    { 0x0174, 0x01, },
    { 0x0175, 0x01, },
    { 0x018C, 0x0A, },
    { 0x018D, 0x0A, },
    { 0x0301, 0x05, },
    { 0x0303, 0x01, },
    { 0x0304, 0x03, },
    { 0x0305, 0x03, },
    { 0x0306, 0x00, },
    { 0x0307, 0x39, },
    { 0x0309, 0x0A, },
    { 0x030B, 0x01, },
    { 0x030C, 0x00, },
    { 0x030D, 0x72, },
    { 0x455E, 0x00, },
    { 0x471E, 0x4B, },
    { 0x4767, 0x0F, },
    { 0x4750, 0x14, },
    { 0x4540, 0x00, },
    { 0x47B4, 0x14, },
    { 0x4713, 0x30, },
    { 0x478B, 0x10, },
    { 0x478F, 0x10, },
    { 0x4793, 0x10, },
    { 0x4797, 0x0E, },
    { 0x479B, 0x0E, },
    { 0x0172, 0x03, },
    { 0x0157, 0xC8, },
    { 0x015A, 0x05, },
    { 0x015B, 0xDC, },
};

/* 1920 x 1080 */
static const struct cam_i2c_reg_array res2_array[] = {
    { 0x0103, 0x01, },
    { 0x0100, 0x00, },
    { 0x6620, 0x01, },
    { 0x6621, 0x01, },
    { 0x6622, 0x01, },
    { 0x6623, 0x01, },
    { 0x30EB, 0x05, },
    { 0x30EB, 0x0C, },
    { 0x300A, 0xFF, },
    { 0x300B, 0xFF, },
    { 0x30EB, 0x05, },
    { 0x30EB, 0x09, },
    { 0x0114, 0x01, },
    { 0x0128, 0x00, },
    { 0x012A, 0x18, },
    { 0x012B, 0x00, },
    { 0x0157, 0x00, },
    { 0x015A, 0x09, },
    { 0x015B, 0xBD, },
    { 0x0160, 0x06, },
    { 0x0161, 0xE0, },
    { 0x0162, 0x0D, },
    { 0x0163, 0x78, },
    { 0x0164, 0x02, },
    { 0x0165, 0xA8, },
    { 0x0166, 0x0A, },
    { 0x0167, 0x27, },
    { 0x0168, 0x02, },
    { 0x0169, 0xB4, },
    { 0x016A, 0x06, },
    { 0x016B, 0xEB, },
    { 0x016C, 0x07, },
    { 0x016D, 0x80, },
    { 0x016E, 0x04, },
    { 0x016F, 0x38, },
    { 0x0170, 0x01, },
    { 0x0171, 0x01, },
    { 0x0174, 0x00, },
    { 0x0175, 0x00, },
    { 0x018C, 0x0A, },
    { 0x018D, 0x0A, },
    { 0x0301, 0x05, },
    { 0x0303, 0x01, },
    { 0x0304, 0x03, },
    { 0x0305, 0x03, },
    { 0x0306, 0x00, },
    { 0x0307, 0x39, },
    { 0x0309, 0x0A, },
    { 0x030B, 0x01, },
    { 0x030C, 0x00, },
    { 0x030D, 0x72, },
    { 0x455E, 0x00, },
    { 0x471E, 0x4B, },
    { 0x4767, 0x0F, },
    { 0x4750, 0x14, },
    { 0x4540, 0x00, },
    { 0x47B4, 0x14, },
    { 0x4713, 0x30, },
    { 0x478B, 0x10, },
    { 0x478F, 0x10, },
    { 0x4793, 0x10, },
    { 0x4797, 0x0E, },
    { 0x479B, 0x0E, },
    { 0x0172, 0x03, },
    { 0x0160, 0x06, },
    { 0x0161, 0xE3, },
    { 0x0157, 0xC8, },
    { 0x015A, 0x05, },
    { 0x015B, 0xDC, },
};
#endif

static const struct cam_i2c_reg_setting frmival_res0_user_data = {
    .size = ARRAY_SIZE(res0_array),
    .regs = res0_array,
};

#if 0
static const struct cam_i2c_reg_setting frmival_res1_user_data = {
    .size = ARRAY_SIZE(res1_array),
    .regs = res1_array,
};

static const struct cam_i2c_reg_setting frmival_res2_user_data = {
    .size = ARRAY_SIZE(res2_array),
    .regs = res2_array,
};
#endif

static const struct camera_ext_frmival_node frmival_res0[] = {
    {
        .numerator = 1,
        .denominator = 30,
        .user_data = &frmival_res0_user_data,
    },
};

#if 0
static const struct camera_ext_frmival_node frmival_res1[] = {
    {
        .numerator = 1,
        .denominator = 30,
        .user_data = &frmival_res1_user_data,
    },
};

static const struct camera_ext_frmival_node frmival_res2[] = {
    {
        .numerator = 1,
        .denominator = 30,
        .user_data = &frmival_res2_user_data,
    },
};
#endif

// frame sizes for BGGR10
static const struct camera_ext_frmsize_node _cam_frmsizes[] = {
    {
        .width = 3264,
        .height = 2448,
        .num_frmivals = ARRAY_SIZE(frmival_res0),
        .frmival_nodes = frmival_res0,
    },
#if 0
    {
        .width = 1600,
        .height = 1200,
        .num_frmivals = ARRAY_SIZE(frmival_res1),
        .frmival_nodes = frmival_res1,
    },
    {
        .width = 1920,
        .height = 1080,
        .num_frmivals = ARRAY_SIZE(frmival_res2),
        .frmival_nodes = frmival_res2,
    },
#endif
};

// format for camera input
static const struct camera_ext_format_node _cam_formats[] = {
    {
        .name = "BGGR10",
        .fourcc = V4L2_PIX_FMT_SBGGR10,
        .depth = 10,
        .num_frmsizes = ARRAY_SIZE(_cam_frmsizes),
        .frmsize_nodes = _cam_frmsizes,
    },
};

// ov7251 input
static const struct camera_ext_input_node _cam_inputs[] = {
    {
        .name = "imx219-pi",
        .type = CAM_EXT_INPUT_TYPE_CAMERA,
        .status = 0,
        .capabilities = CAMERA_EXT_STREAM_CAP_PREVIEW |
                        CAMERA_EXT_STREAM_CAP_VIDEO |
                        CAMERA_EXT_STREAM_CAP_SNAPSHOT,
        .num_formats = ARRAY_SIZE(_cam_formats),
        .format_nodes = _cam_formats,
    },
};

//root node
const struct camera_ext_format_db mhb_camera_format_db = {
    .num_inputs = ARRAY_SIZE(_cam_inputs),
    .input_nodes = _cam_inputs,
};

extern struct camera_ext_ctrl_db mhb_camera_ctrl_db;

struct mhb_cdsi_config mhb_camera_csi_config =
{
    .direction = 0,
    .mode = 0x01,            /* TSB_CDSI_MODE_CSI */

    .tx_num_lanes = 4,
    .rx_num_lanes = 0,       /* variable */
    .tx_bits_per_lane = 0,  /* variable */
    .rx_bits_per_lane = 0,  /* variable */

    .hs_rx_timeout = 0xffffffff,

    .framerate = 0, /* variable */

    .pll_frs = 0,
    .pll_prd = 0,
    .pll_fbd = 0,

    .width = 0,  /* variable */
    .height = 0, /* variable */
    .bpp = 0,    /* variable */

    .bta_enabled = 0,
    .continuous_clock = 0,
    .blank_packet_enabled = 0,
    .video_mode = 0,
    .color_bar_enabled = 0,
};

/* Device Ops */
static int _mhb_camera_get_csi_config(struct device *dev,
                          void *config)
{
    const struct camera_ext_format_user_config *cfg = camera_ext_get_user_config();
    const struct camera_ext_frmival_node *ival;

    ival = get_current_frmival_node(&mhb_camera_format_db, cfg);
    if (ival == NULL) {
        CAM_ERR("Failed to get current frame interval\n");
        return -1;
    }

    mhb_camera_csi_config.rx_num_lanes = 2;
    mhb_camera_csi_config.framerate = roundf((float)(ival->denominator) /
                                             (float)(ival->numerator));
    mhb_camera_csi_config.tx_bits_per_lane = 500000000;
    mhb_camera_csi_config.rx_bits_per_lane = 900000000;

    *(struct mhb_cdsi_config **)config = &mhb_camera_csi_config;

    return 0;
}

static int _mhb_camera_soc_enable(struct device *dev, uint8_t bootmode)
{
    gpio_direction_out(s_data.led_en, 1);
    usleep(CAMERA_POWER_DELAY_US);

    gpio_direction_out(s_data.rst_n, 1);
    usleep(CAMERA_POWER_DELAY_US);

    uint8_t id0 = 0;
    uint8_t id1 = 0;
    mhb_camera_i2c_write_reg1_16(CAMERA_SENSOR_I2C_ADDR, 0x0103, 1);
    mhb_camera_i2c_read_reg1_16(CAMERA_SENSOR_I2C_ADDR, 0x0000, &id0);
    mhb_camera_i2c_read_reg1_16(CAMERA_SENSOR_I2C_ADDR, 0x0001, &id1);
    lldbg("Sensor ID: %02x %02x\n", id0, id1);

    /* configure init registers */
    size_t num = ARRAY_SIZE(init_reg_array);
    int i;
    int ret;
    for (i = 0; i < num; i++) {
        ret = mhb_camera_i2c_write_reg1_16(CAMERA_SENSOR_I2C_ADDR,
                                        init_reg_array[i].reg_addr,
                                        init_reg_array[i].data);
        if (ret)
            break;
    }
    return 0;
}

static int _mhb_camera_soc_disable(struct device *dev)
{
    gpio_direction_out(s_data.rst_n, 0);
    gpio_direction_out(s_data.led_en, 0);

    usleep(CAMERA_POWER_DELAY_US);

    return 0;
}

static int _mhb_camera_stream_configure(struct device *dev)
{
    const struct camera_ext_format_user_config *cfg = camera_ext_get_user_config();
    const struct camera_ext_frmival_node *ival;

    ival = get_current_frmival_node(&mhb_camera_format_db, cfg);
    if (ival == NULL) {
        CAM_ERR("Failed to get current frame interval\n");
        return -1;
    }


    const struct cam_i2c_reg_setting *udata = ival->user_data;
    if (udata == NULL) {
        CAM_ERR("Failed to get user data\n");
        return -1;
    }

    int i;
    int ret;
    for (i = 0; i < udata->size; i++) {
        ret = mhb_camera_i2c_write_reg1_16(CAMERA_SENSOR_I2C_ADDR,
                                        udata->regs[i].reg_addr,
                                        udata->regs[i].data);
        if (ret)
            break;
    }
    return 0;
}


static int _mhb_camera_stream_enable(struct device *dev)
{
    int ret = start_metadata_task();

    if (!ret) {
        ret = mhb_camera_i2c_write_reg1_16(CAMERA_SENSOR_I2C_ADDR, 0x0100, 0x01);
        if (ret)
            stop_metadata_task();
    }

    return ret;
}

static int _mhb_camera_stream_disable(struct device *dev)
{
    stop_metadata_task();
    return mhb_camera_i2c_write_reg1_16(CAMERA_SENSOR_I2C_ADDR, 0x0100, 0x00);
}

int _mhb_camera_init(struct device *dev)
{
    struct device_resource *res;

    memset(&s_data, 0, sizeof(s_data));

    res = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO, "rst_n");
    if (!res) {
        CAM_ERR("failed to get rst_n gpio\n");
        return -ENODEV;
    }
    s_data.rst_n = res->start;

    res = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO, "led_en");
    if (!res) {
        CAM_ERR("failed to get led_en gpio\n");
        return -ENODEV;
    }
    s_data.led_en = res->start;

    res = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO, "spi_sel");
    if (!res) {
        CAM_ERR("failed to get spi_sel gpio\n");
        return -ENODEV;
    }
    s_data.spi_sel = res->start;

    gpio_direction_out(s_data.spi_sel, 0);
    gpio_direction_out(s_data.rst_n, 0);
    gpio_direction_out(s_data.led_en, 0);

    camera_ext_register_format_db(&mhb_camera_format_db);
    camera_ext_register_control_db(&mhb_camera_ctrl_db);

    init_metadata_task();
    return 0;
}

static struct device_mhb_camera_dev_type_ops _mhb_camera_type_ops = {
    .soc_enable = _mhb_camera_soc_enable,
    .soc_disable = _mhb_camera_soc_disable,
    .stream_configure = _mhb_camera_stream_configure,
    .stream_enable = _mhb_camera_stream_enable,
    .stream_disable = _mhb_camera_stream_disable,
    .get_csi_config = _mhb_camera_get_csi_config,
};

static struct device_driver_ops _mhb_camera_driver_ops = {
    .probe    = &_mhb_camera_init,
    .type_ops = &_mhb_camera_type_ops,
};

struct device_driver imx219_pi_mhb_camera_driver = {
    .type = DEVICE_TYPE_MHB_CAMERA_HW,
    .name = "IMX219_PI",
    .desc = "Raspberry Pi Camera",
    .ops  = &_mhb_camera_driver_ops,
};
