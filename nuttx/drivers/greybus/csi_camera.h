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

#ifndef CSI_CAMERA_H
#define CSI_CAMERA_H

#define TEST_PATTERN

#include <nuttx/i2c.h>

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
};

struct gb_csi_camera_res {
    int x_output;
    int y_output;
    int line_length;
    int frame_length;
    int op_pixel_clk; //vfe pixel clk required
};

struct gb_csi_camera_info {
    int lane_cnt;
    int settle_cnt;
    int format_fourcc; //cid0 for now
    int res_size;
    struct gb_csi_camera_res res[4];
};

int cam_setup(struct camera_dev_s *cam_dev);
int cam_query_info(struct camera_dev_s *cam_dev,
		struct gb_csi_camera_info *cam_info);
int cam_power_on(struct camera_dev_s *cam_dev);
int cam_power_off(struct camera_dev_s *cam_dev);
int cam_stream_on(struct camera_dev_s *cam_dev);
int cam_stream_off(struct camera_dev_s *cam_dev);
int cam_set_resolution(struct camera_dev_s *cam_dev, int res);

#ifdef TEST_PATTERN
int cam_set_test_pattern(struct camera_dev_s *cam_dev, int test);
#endif

struct cam_i2c_reg_array {
    uint16_t reg_addr;
    uint16_t data;
};

struct cam_i2c_reg_setting {
    uint16_t size;
    struct cam_i2c_reg_array *regs;
};

struct cam_group_settings {
    uint16_t size;
    struct cam_i2c_reg_setting *settings;
};

struct sensor_info {
    uint16_t i2c_addr;
    struct gb_csi_camera_info cam_info;
    struct cam_i2c_reg_setting init;
    struct cam_i2c_reg_setting start;
    struct cam_i2c_reg_setting stop;
    struct cam_group_settings resolutions;
#ifdef TEST_PATTERN
    struct cam_group_settings test;
#endif
};

struct sensor_info *get_board_sensor_info(void);

#endif
