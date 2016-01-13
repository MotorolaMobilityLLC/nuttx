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

#ifndef CAMERA_EXT_CSI_H
#define CAMERA_EXT_CSI_H

#include <nuttx/i2c.h>
#include <nuttx/util.h>
#include "camera_ext.h"

#define v4l2_fourcc(a, b, c, d) \
    ((uint32_t)(a) | ((uint32_t)(b) << 8) \
    | ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))

typedef enum {
    OFF = 0,
    ON,
    STREAMING
} camera_status_t;

struct camera_dev_s
{
    camera_status_t status;
    struct i2c_dev_s *i2c;
    struct sensor_info *sensor;

    struct camera_ext_ctrl_db ctrl_db;
};

struct gb_camera_ext_v4l_info {
    int camera_ext_inputs_num;
    struct camera_ext_input *camera_ext_inputs;
    int camera_ext_formats_num;
    struct camera_ext_fmtdesc *camera_ext_formats;
    int camera_ext_frmsizes_num;
    struct camera_ext_frmsize *camera_ext_frmsizes;
    int camera_ext_frmivals_num;
    struct camera_ext_frmival *camera_ext_frmivals;
};

struct cam_i2c_reg_array {
    const uint16_t reg_addr;
    const uint16_t data;
};

struct cam_i2c_reg_setting {
    const uint16_t size;
    struct cam_i2c_reg_array const *regs;
};

struct cam_group_settings {
    const uint16_t size;
    struct cam_i2c_reg_setting const *settings;
};

struct csi_stream_user_data {
    //register settings for this resolution
    const struct cam_i2c_reg_setting res_regs;
    //Mega bps for this resolution
    const uint32_t mbps;
};

struct sensor_info {
    const uint16_t i2c_addr;
    const struct cam_i2c_reg_setting init;
    const struct cam_i2c_reg_setting start;
    const struct cam_i2c_reg_setting stop;

    struct camera_ext_format_db const *sensor_db;
};

//each csi camera instance must implement this function
struct sensor_info *camera_ext_csi_get_board_sensor_info(void);

//any camera related drivers can expose their own controls
int register_camera_ext_ctrl_db(struct device *dev,
        const struct camera_ext_ctrl_item **ctrls, size_t num);
#endif
