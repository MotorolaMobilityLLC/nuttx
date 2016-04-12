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

#include "camera_ext.h"
#include "camera_ext_csi.h"

static const struct cam_i2c_reg_array init_reg_array[] = {
  {0x0103, 0x01},
};

static const struct cam_i2c_reg_array start_reg_array[] = {
  { 0x0100, 0x01 },
};

static const struct cam_i2c_reg_array stop_reg_array[] = {
  { 0x0100, 0x00 },
};

static const struct cam_i2c_reg_array full_norm_30fps_none_array[] = {
  {0x0101, 0x03},
  {0x0202, 0x09},
  {0x0203, 0xCA},
  {0x0301, 0x05},
  {0x0303, 0x01},
  {0x0305, 0x06},
  {0x0309, 0x05},
  {0x030B, 0x01},
  {0x030C, 0x00},
  {0x030D, 0xA2},
  {0x0340, 0x09},
  {0x0341, 0xCE},
  {0x0342, 0x10},
  {0x0343, 0x68},
  {0x0344, 0x00},
  {0x0345, 0x08},
  {0x0346, 0x00},
  {0x0347, 0x08},
  {0x0348, 0x0C},
  {0x0349, 0xC7},
  {0x034A, 0x09},
  {0x034B, 0x97},
  {0x034C, 0x0C}, //w 0xCC0/3264
  {0x034D, 0xC0},
  {0x034E, 0x09}, //h 0x990/2448
  {0x034F, 0x90},
  {0x0383, 0x01},
  {0x0387, 0x01},
  {0x0390, 0x00},
  {0x0401, 0x00},
  {0x0405, 0x10},
  {0x3020, 0x10},
  {0x3041, 0x15},
  {0x3042, 0x87},
  {0x3089, 0x4F},
  {0x3309, 0x9A},
  {0x3344, 0x57},
  {0x3345, 0x1F},
  {0x3362, 0x0A},
  {0x3363, 0x0A},
  {0x3364, 0x00},
  {0x3368, 0x18},
  {0x3369, 0x00},
  {0x3370, 0x77},
  {0x3371, 0x2F},
  {0x3372, 0x4F},
  {0x3373, 0x2F},
  {0x3374, 0x2F},
  {0x3375, 0x37},
  {0x3376, 0x9F},
  {0x3377, 0x37},
  {0x33C8, 0x00},
  {0x33D4, 0x0C},
  {0x33D5, 0xD0},
  {0x33D6, 0x09},
  {0x33D7, 0xA0},
  {0x4100, 0x0E},
  {0x4108, 0x01},
  {0x4109, 0x7C},
};

static const struct cam_i2c_reg_array half_wide_60fps_bin_array[] = {
  {0x0101, 0x03},
  {0x0202, 0x04},
  {0x0203, 0xE3},
  {0x0301, 0x05},
  {0x0303, 0x01},
  {0x0305, 0x06},
  {0x0309, 0x05},
  {0x030B, 0x01},
  {0x030C, 0x00},
  {0x030D, 0xA2},
  {0x0340, 0x04},
  {0x0341, 0xE7},
  {0x0342, 0x10},
  {0x0343, 0x68},
  {0x0344, 0x00},
  {0x0345, 0x00},
  {0x0346, 0x01},
  {0x0347, 0x34},
  {0x0348, 0x0C},
  {0x0349, 0xCF},
  {0x034A, 0x08},
  {0x034B, 0x6B},
#if 0
  {0x034C, 0x06},
  {0x034D, 0x68},
  {0x034E, 0x03},
  {0x034F, 0x9C},
#else
  {0x034C, 0x06},
  {0x034D, 0x40},
  {0x034E, 0x04},
  {0x034F, 0xB0},
#endif

  {0x0383, 0x01},
  {0x0387, 0x01},
  {0x0390, 0x01},
  {0x0401, 0x00},
  {0x0405, 0x10},
  {0x3020, 0x10},
  {0x3041, 0x15},
  {0x3042, 0x87},
  {0x3089, 0x4F},
  {0x3309, 0x9A},
  {0x3344, 0x57},
  {0x3345, 0x1F},
  {0x3362, 0x0A},
  {0x3363, 0x0A},
  {0x3364, 0x00},
  {0x3368, 0x18},
  {0x3369, 0x00},
  {0x3370, 0x77},
  {0x3371, 0x2F},
  {0x3372, 0x4F},
  {0x3373, 0x2F},
  {0x3374, 0x2F},
  {0x3375, 0x37},
  {0x3376, 0x9F},
  {0x3377, 0x37},
  {0x33C8, 0x00},
  {0x33D4, 0x06},
  {0x33D5, 0x68},
  {0x33D6, 0x03},
  {0x33D7, 0x9C},
  {0x4100, 0x0E},
  {0x4108, 0x01},
  {0x4109, 0x7C},
};

static const struct csi_stream_user_data frmival_1600x1200_user_data = {
    .res_regs = {
        .size = ARRAY_SIZE(half_wide_60fps_bin_array),
        .regs = half_wide_60fps_bin_array,
    },
    .mbps = BW_MEGA_BIT(1600, 1200, 10, 60),
};

// frame rate for 1600x1200 BGGR10
static const struct camera_ext_frmival_node frmival_1600x1200[] = {
    {
        .numerator = 1,
        .denominator = 60,
        .user_data = &frmival_1600x1200_user_data,
    },
};

static const struct csi_stream_user_data frmival_3264x2448_user_data = {
    .res_regs = {
        .size = ARRAY_SIZE(full_norm_30fps_none_array),
        .regs = full_norm_30fps_none_array,
    },
    .mbps = BW_MEGA_BIT(3264, 2448, 10, 30),
};

// frame rate for 3264x2448 BGGR10
static const struct camera_ext_frmival_node frmival_3264x2448[] = {
    {
        .numerator = 1,
        .denominator = 30,
        .user_data = &frmival_3264x2448_user_data,
    },
};

// frame sizes for BGGR10
static const struct camera_ext_frmsize_node imx179_cam_frmsizes[] = {
    {
        .width = 1600,
        .height = 1200,
        .num_frmivals = ARRAY_SIZE(frmival_1600x1200),
        .frmival_nodes = frmival_1600x1200,
    },

    {
        .width = 3264,
        .height = 2448,
        .num_frmivals = ARRAY_SIZE(frmival_3264x2448),
        .frmival_nodes = frmival_3264x2448,
    },
};

// format for camera input
static const struct camera_ext_format_node imx179_cam_formats[] = {
    {
        .name = "BGGR10",
        .fourcc = V4L2_PIX_FMT_SBGGR10,
        .depth = 10,
        .num_frmsizes = ARRAY_SIZE(imx179_cam_frmsizes),
        .frmsize_nodes = imx179_cam_frmsizes,
    },
};

// imx179 input
static const struct camera_ext_input_node imx179_inputs[] = {
    {
        .name = "csi-imx179",
        .type = CAM_EXT_INPUT_TYPE_CAMERA,
        .status = CAM_EXT_IN_ST_HFLIP,
        .capabilities = CAMERA_EXT_STREAM_CAP_PREVIEW
            | CAMERA_EXT_STREAM_CAP_VIDEO,
        .num_formats = ARRAY_SIZE(imx179_cam_formats),
        .format_nodes = imx179_cam_formats,
    },
};

//root node
static const struct camera_ext_format_db imx179_sensor_db = {
    .num_inputs = ARRAY_SIZE(imx179_inputs),
    .input_nodes = imx179_inputs,
};

//not const, user_config is writable
static struct sensor_info imx179 = {
    .i2c_addr = 0x10,
    .init = {
        .size = ARRAY_SIZE(init_reg_array),
        .regs = init_reg_array,
    },

    .start = {
        .size = ARRAY_SIZE(start_reg_array),
        .regs = start_reg_array,
    },

    .stop = {
        .size = ARRAY_SIZE(stop_reg_array),
        .regs = stop_reg_array,
    },

    .sensor_db = &imx179_sensor_db,
};

struct sensor_info * camera_ext_csi_get_board_sensor_info(void) {
    return &imx179;
}
