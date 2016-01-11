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

#ifndef CAMERA_EXT_H
#define CAMERA_EXT_H
#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>
#include <arch/byteorder.h>
#include <nuttx/greybus/types.h>
#include <nuttx/camera_ext_defs.h>

#define DEBUG
#include "camera_ext_dbg.h"

#define CAMERA_UNIPRO_CPORT 16
#define IPC_TIMEOUT 500

#define BW_MEGA_BIT(w, h, depth, fps) \
    ((((uint32_t)w*(uint32_t)h*(uint32_t)depth*(uint32_t)fps)>>20) + 1)

#define v4l2_fourcc(a, b, c, d) \
    ((uint32_t)(a) | ((uint32_t)(b) << 8)               \
     | ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))

/* Common V4L2 pixel formats. Note the below is not the exhuausive list */
#define V4L2_PIX_FMT_BGR24   v4l2_fourcc('B', 'G', 'R', '3') /* 24  BGR-8-8-8     */
#define V4L2_PIX_FMT_RGB24   v4l2_fourcc('R', 'G', 'B', '3') /* 24  RGB-8-8-8     */
#define V4L2_PIX_FMT_YUYV    v4l2_fourcc('Y', 'U', 'Y', 'V') /* 16  YUV 4:2:2     */
#define V4L2_PIX_FMT_YVYU    v4l2_fourcc('Y', 'V', 'Y', 'U') /* 16 YVU 4:2:2 */
#define V4L2_PIX_FMT_UYVY    v4l2_fourcc('U', 'Y', 'V', 'Y') /* 16  YUV 4:2:2     */
#define V4L2_PIX_FMT_VYUY    v4l2_fourcc('V', 'Y', 'U', 'Y') /* 16  YUV 4:2:2     */
#define V4L2_PIX_FMT_SBGGR10 v4l2_fourcc('B', 'G', '1', '0') /* 10  BGBG.. GRGR.. */
#define V4L2_PIX_FMT_SGBRG10 v4l2_fourcc('G', 'B', '1', '0') /* 10  GBGB.. RGRG.. */
#define V4L2_PIX_FMT_SGRBG10 v4l2_fourcc('B', 'A', '1', '0') /* 10  GRGR.. BGBG.. */
#define V4L2_PIX_FMT_SRGGB10 v4l2_fourcc('R', 'G', '1', '0') /* 10  RGRG.. GBGB.. */

/*
 * Structures used to store v4l2 format information in a hierachy way.
 */

/* Frame interval for a specific frame size and format.
 * It's a leaf node and may store some user data for the format.
 */
struct camera_ext_fract_node {
    uint32_t numerator;
    uint32_t denominator;
};

struct camera_ext_stepwise_node {
    struct camera_ext_fract_node min;
    struct camera_ext_fract_node max;
    struct camera_ext_fract_node step;
};

struct camera_ext_frmival_node {
    uint32_t type;
    union {
        struct camera_ext_fract_node discrete;
        struct camera_ext_stepwise_node stepwise;
    };

    const void *user_data;
};

static inline int camera_ext_fract_equal(struct camera_ext_fract_node const *l,
                                         struct camera_ext_fract const *r)
{
    //suppose they are irreducible fraction
    return l->numerator == le32_to_cpu(r->numerator)
        && l->denominator == le32_to_cpu(r->denominator);
}

//frame size for a specific format
struct camera_ext_frmsize_discrete_node {
    uint32_t width;
    uint32_t height;
};

struct camera_ext_frmsize_stepwise_node {
    uint32_t min_width;
    uint32_t max_width;
    uint32_t step_width;
    uint32_t min_height;
    uint32_t max_height;
    uint32_t step_height;
};

struct camera_ext_frmsize_node {
    uint32_t type;
    union {
        struct camera_ext_frmsize_discrete_node discrete;
        struct camera_ext_frmsize_stepwise_node stepwise;
    };

    int num_frmivals;
    struct camera_ext_frmival_node const *frmival_nodes;
};

//format for a specific input
struct camera_ext_format_node {
    const char *name;
    uint32_t fourcc;
    uint32_t depth;

    int num_frmsizes;
    struct camera_ext_frmsize_node const *frmsize_nodes;
};

//input of this sensor
struct camera_ext_input_node {
    const char *name;
    uint32_t type;
    uint32_t status;

    int num_formats;
    struct camera_ext_format_node const *format_nodes;
};

/* An array of cam_ext_ctrl_item holds all supported controls' config.
 * ctrl_ids are in ascending order in the array.
 */
struct cam_ext_ctrl_item {
    struct camera_ext_predefined_ctrl_mod_cfg const ctrl_cfg;
    union {
        uint8_t val_8;
        uint16_t val_16;
        uint32_t val;
        void *p_val; //point to val_64 or an array of 8/16/32 bits number
    };

    /* val points to struct camera_ext_val_base */
    int (*g_volatile_ctrl)(void *val);
    int (*s_ctrl)(void *val);
    int (*try_ctrl)(void *val);
};

//format root node
struct gb_camera_ext_sensor_db {
    const int num_inputs;
    struct camera_ext_input_node const *input_nodes;

};

//control db
struct camera_ext_ctrl_db {
    uint32_t num_ctrls;
    struct cam_ext_ctrl_item **ctrls;
};

struct frmsize_user_config {
    uint32_t idx_frmsize;
    //actual frame size if idx points to a step/continuous type
    uint32_t width;
    uint32_t height;
};

struct frmival_user_config {
    uint32_t idx_frmival;
    //actual frame interval if idx points to a step/continuous type
    uint32_t numerator;
    uint32_t denominator;
};

struct gb_camera_ext_sensor_user_config {
    uint32_t input;//index of current input
    uint32_t format; //index of current format
    struct frmsize_user_config frmsize;
    struct frmival_user_config frmival;
};

/* Functions to access Format DB */
bool is_input_valid(struct gb_camera_ext_sensor_db const *db, uint32_t input);
bool is_format_valid(struct gb_camera_ext_sensor_db const *db, uint32_t input, uint32_t format);
bool is_frmsize_valid(struct gb_camera_ext_sensor_db const *db,
                      uint32_t input, uint32_t format, uint32_t frmsize);
bool is_frmsize_valid(struct gb_camera_ext_sensor_db const *db,
                      uint32_t input, uint32_t format, uint32_t frmsize);

struct camera_ext_input_node const *get_current_input_node(
    struct gb_camera_ext_sensor_db const *db,
    struct gb_camera_ext_sensor_user_config *cfg);

struct camera_ext_format_node const *get_current_format_node(
    struct gb_camera_ext_sensor_db const *db,
    struct gb_camera_ext_sensor_user_config *cfg);

struct camera_ext_frmsize_node const *get_current_frmsize_node(
    struct gb_camera_ext_sensor_db const *db,
    struct gb_camera_ext_sensor_user_config *cfg);

struct camera_ext_frmival_node const *get_current_frmival_node(
    struct gb_camera_ext_sensor_db const *db,
    struct gb_camera_ext_sensor_user_config *cfg);

/* Function to fill in GB structure from Format DB*/
int camera_ext_fill_gb_input(struct gb_camera_ext_sensor_db const *db, uint32_t index,
                             struct camera_ext_input *input);

int camera_ext_fill_gb_fmtdesc(struct gb_camera_ext_sensor_db const *db, uint32_t input,
                               uint32_t format, struct camera_ext_fmtdesc *fmt);

int cam_ext_fill_gb_format(struct gb_camera_ext_sensor_db const *db,
                           uint32_t input, uint32_t format,
                           uint32_t frmsize, uint32_t user_width, uint32_t user_height,
                           struct camera_ext_format *fmt);

int cam_ext_fill_gb_frmsize(struct gb_camera_ext_sensor_db const *db, uint32_t input,
                            uint32_t index, struct camera_ext_frmsize* fmsize);

int cam_ext_fill_gb_frmival(struct gb_camera_ext_sensor_db const *db, uint32_t input,
                            uint32_t index, struct camera_ext_frmival* frmival);

int cam_ext_fill_gb_streamparm(struct gb_camera_ext_sensor_db const *db,
                               struct gb_camera_ext_sensor_user_config *cfg,
                               uint32_t capability, uint32_t capturemode,
                               struct camera_ext_streamparm *parm);

/* Functions to update current user setting */
int cam_ext_set_current_format(struct gb_camera_ext_sensor_db const *db,
                               struct gb_camera_ext_sensor_user_config *cfg,
                               struct camera_ext_format *format);

int cam_ext_frmival_set(struct gb_camera_ext_sensor_db const *db,
                        struct gb_camera_ext_sensor_user_config *cfg,
                        struct camera_ext_streamparm *parm);

/* Functions to for v4l2 controls */
int cam_ext_ctrl_get_cfg(struct camera_ext_ctrl_db *ctrl_db, uint32_t idx,
    struct camera_ext_predefined_ctrl_mod_cfg *mod_ctrl_cfg);

int cam_ext_ctrl_get(struct camera_ext_ctrl_db *ctrl_db,
    struct camera_ext_ctrl_val *ctrl_val);
int cam_ext_ctrl_set(struct camera_ext_ctrl_db *ctrl_db,
    struct camera_ext_ctrl_val *ctrl_val);
int cam_ext_ctrl_try(struct camera_ext_ctrl_db *ctrl_db,
    struct camera_ext_ctrl_val *ctrl_val);

int cam_ext_ctrl_array_get(struct camera_ext_ctrl_db *ctrl_db,
    struct camera_ext_ctrl_array_val *ctrl_val);
int cam_ext_ctrl_array_set(struct camera_ext_ctrl_db *ctrl_db,
    struct camera_ext_ctrl_array_val *ctrl_val);
int cam_ext_ctrl_array_try(struct camera_ext_ctrl_db *ctrl_db,
    struct camera_ext_ctrl_array_val *ctrl_val);

#endif
