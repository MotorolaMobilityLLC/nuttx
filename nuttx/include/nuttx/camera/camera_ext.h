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
#include <nuttx/device.h>
#include <nuttx/greybus/types.h>
#include <nuttx/device_cam_ext.h>
#include <nuttx/camera/camera_ext_defs.h>

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
#define V4L2_PIX_FMT_SBGGR8  v4l2_fourcc('B', 'A', '8', '1') /*  8  BGBG.. GRGR.. */
#define V4L2_PIX_FMT_SGBRG8  v4l2_fourcc('G', 'B', 'R', 'G') /*  8  GBGB.. RGRG.. */
#define V4L2_PIX_FMT_SGRBG8  v4l2_fourcc('G', 'R', 'B', 'G') /*  8  GRGR.. BGBG.. */
#define V4L2_PIX_FMT_SRGGB8  v4l2_fourcc('R', 'G', 'G', 'B') /*  8  RGRG.. GBGB.. */
#define V4L2_PIX_FMT_SBGGR10 v4l2_fourcc('B', 'G', '1', '0') /* 10  BGBG.. GRGR.. */
#define V4L2_PIX_FMT_SGBRG10 v4l2_fourcc('G', 'B', '1', '0') /* 10  GBGB.. RGRG.. */
#define V4L2_PIX_FMT_SGRBG10 v4l2_fourcc('B', 'A', '1', '0') /* 10  GRGR.. BGBG.. */
#define V4L2_PIX_FMT_SRGGB10 v4l2_fourcc('R', 'G', '1', '0') /* 10  RGRG.. GBGB.. */

#ifdef CONFIG_ENDIAN_BIG
#error "big-endian unsupported"
#endif

#define le64_to_cpu(v) (v)
#define cpu_to_le64(v) (v)

/*
 * Structures used to store v4l2 format information in a hierachy way.
 */

/* Frame interval for a specific frame size and format.
 * It's a leaf node and may store some user data for the format.
 */
struct camera_ext_frmival_node {
    uint32_t numerator;
    uint32_t denominator;
    const void *user_data;
};

static inline int camera_ext_fract_equal(struct camera_ext_frmival_node const *l,
                                         struct camera_ext_fract const *r)
{
    //suppose they are irreducible fraction
    return l->numerator == le32_to_cpu(r->numerator)
        && l->denominator == le32_to_cpu(r->denominator);
}

//frame size for a specific format
struct camera_ext_frmsize_node {
    uint32_t width;
    uint32_t height;
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
    uint32_t capabilities;

    int num_formats;
    struct camera_ext_format_node const *format_nodes;
};

//control value type
typedef union {
    uint32_t val;
    uint64_t val_64;
    float val_f;
    double val_d;

    uint8_t *p_val_8; //string type
    uint32_t *p_val;
    uint64_t *p_val_64;
    float *p_val_f;
    double *p_val_d;
} camera_ext_ctrl_val_t;

#define CAM_EXT_CTRL_DATA_TYPE_INT    1
#define CAM_EXT_CTRL_DATA_TYPE_BOOL   2
#define CAM_EXT_CTRL_DATA_TYPE_INT64  3
#define CAM_EXT_CTRL_DATA_TYPE_FLOAT  4
#define CAM_EXT_CTRL_DATA_TYPE_DOUBLE 5
#define CAM_EXT_CTRL_DATA_TYPE_STRING 6

//control value config
struct camera_ext_ctrl_val_cfg {
    //if nr_of_elem > 1, value pointer field will be used.
    uint32_t nr_of_elem; /* if > 1, must init p_val_xxx */
    uint32_t elem_type;
};

/*
 * Each supported control will have a MOD side config.
 * Flags are combination of CAMERA_EXT_CTRL_FLAG_NEED_XXX and should be same
 * as predfined controls at phone side. Otherwise it will be rejected by phone.
 * Id and flags are mandatory. Others are optional and indicated by flags.
 */
struct device;
struct camera_ext_ctrl_cfg {
    uint32_t id;
    uint32_t flags; /* tell camera_ext how to pack/unpack data */
    union {
        int64_t min;
        float min_f; /* used to validate set_ctrl, mod side only */
    };
    union {
        int64_t max;
        float max_f; /* used to validate set_ctrl, mod side only */
    };
    uint64_t step;
    uint64_t menu_skip_mask;
    uint32_t array_size; //size of array in below union
    union {
        const uint32_t *dims;
        const int64_t *menu_int;
        const float *menu_float;
    };

    uint16_t ver; //(major << 8) | minor
    struct camera_ext_ctrl_val_cfg val_cfg;
    const camera_ext_ctrl_val_t def;

    //for none volatile contrl, phone only reads its cached value
    //which is init by def and updated by set_ctrl.
    int (*get_volatile_ctrl)(struct device *dev,
            const struct camera_ext_ctrl_cfg *self,
            camera_ext_ctrl_val_t *val);
    int (*set_ctrl)(struct device *dev,
            const struct camera_ext_ctrl_cfg *self,
            const camera_ext_ctrl_val_t *val);
    int (*try_ctrl)(struct device *dev,
            const struct camera_ext_ctrl_cfg *self,
            const camera_ext_ctrl_val_t *val);
};

//format root node
struct camera_ext_format_db {
    const int num_inputs;
    struct camera_ext_input_node const *input_nodes;
};

/* Function to register format db to camera_ext framework */
void camera_ext_register_format_db(const struct camera_ext_format_db *db);

/* Functions to retrieve format db and user config from camera_ext framework */
const struct camera_ext_format_db *camera_ext_get_format_db(void);
struct camera_ext_format_user_config *camera_ext_get_user_config(void);

//control db
struct camera_ext_ctrl_db {
    uint32_t num_ctrls;
    const struct camera_ext_ctrl_cfg **ctrls;
};

void camera_ext_register_control_db(struct camera_ext_ctrl_db *ctrl_db);

struct camera_ext_format_user_config {
    uint32_t input;   //index of current input
    uint32_t format;  //index of current format
    uint32_t frmsize; //index of current frame size
    uint32_t frmival; //index of current frame interval
};

/* Functions to access Format DB */
bool is_input_valid(struct camera_ext_format_db const *db, uint32_t input);
bool is_format_valid(struct camera_ext_format_db const *db, uint32_t input, uint32_t format);
bool is_frmsize_valid(struct camera_ext_format_db const *db,
                      uint32_t input, uint32_t format, uint32_t frmsize);
bool is_frmsize_valid(struct camera_ext_format_db const *db,
                      uint32_t input, uint32_t format, uint32_t frmsize);

struct camera_ext_input_node const *get_current_input_node(
    struct camera_ext_format_db const *db,
    struct camera_ext_format_user_config const *cfg);

struct camera_ext_format_node const *get_current_format_node(
    struct camera_ext_format_db const *db,
    struct camera_ext_format_user_config const *cfg);

struct camera_ext_frmsize_node const *get_current_frmsize_node(
    struct camera_ext_format_db const *db,
    struct camera_ext_format_user_config const *cfg);

struct camera_ext_frmival_node const *get_current_frmival_node(
    struct camera_ext_format_db const *db,
    struct camera_ext_format_user_config const *cfg);

/* Function to fill in GB structure from Format DB*/
int camera_ext_fill_gb_input(struct camera_ext_format_db const *db, uint32_t index,
                             struct camera_ext_input *input);

int camera_ext_fill_gb_fmtdesc(struct camera_ext_format_db const *db, uint32_t input,
                               uint32_t format, struct camera_ext_fmtdesc *fmt);

int cam_ext_fill_gb_format(struct camera_ext_format_db const *db,
                           uint32_t input, uint32_t format, uint32_t frmsize,
                           struct camera_ext_format *fmt);

int cam_ext_fill_gb_frmsize(struct camera_ext_format_db const *db, uint32_t input,
                            uint32_t index, struct camera_ext_frmsize* fmsize);

int cam_ext_fill_gb_frmival(struct camera_ext_format_db const *db, uint32_t input,
                            uint32_t index, struct camera_ext_frmival* frmival);

int cam_ext_fill_gb_streamparm(struct camera_ext_format_db const *db,
                               struct camera_ext_format_user_config const *cfg,
                               uint32_t capability, uint32_t capturemode,
                               struct camera_ext_streamparm *parm);

/* Functions to update current user setting */
int cam_ext_set_current_format(struct camera_ext_format_db const *db,
                               struct camera_ext_format_user_config *cfg,
                               struct camera_ext_format *format);

int cam_ext_frmival_set(struct camera_ext_format_db const *db,
                        struct camera_ext_format_user_config *cfg,
                        struct camera_ext_streamparm *parm);

/* Common format db access functions for drivers to pick up */
int camera_ext_input_enum(struct device *dev, struct camera_ext_input *input);
int camera_ext_input_get(struct device *dev, int *input);
int camera_ext_input_set(struct device *dev, int index);
int camera_ext_format_enum(struct device *dev, struct camera_ext_fmtdesc *format);
int camera_ext_format_get(struct device *dev, struct camera_ext_format *format);
int camera_ext_format_set(struct device *dev, struct camera_ext_format* format);
int camera_ext_frmsize_enum(struct device *dev, struct camera_ext_frmsize* frmsize);
int camera_ext_frmival_enum(struct device *dev, struct camera_ext_frmival* frmival);
int camera_ext_stream_set_parm(struct device *dev, struct camera_ext_streamparm *parm);
int camera_ext_stream_get_parm(struct device *dev, struct camera_ext_streamparm *parm);

/* Common control functions for drivers to pick up */
int camera_ext_ctrl_get_cfg(struct device *dev, uint32_t idx,
        struct camera_ext_predefined_ctrl_mod_cfg *cfg, uint32_t cfg_size);
int camera_ext_ctrl_get(struct device *dev, uint32_t idx, uint8_t *ctrl_val,
    uint32_t ctrl_val_size);
int camera_ext_ctrl_set(struct device *dev, uint32_t idx, uint8_t *ctrl_val,
    uint32_t ctrl_val_size);
int camera_ext_ctrl_try(struct device *dev, uint32_t idx, uint8_t *ctrl_val,
    uint32_t ctrl_val_size);
int camera_ext_set_phone_ver(struct device *dev, uint8_t major, uint8_t minor);
/* get phone camera_ext protocol version. 0 means not connected yet */
uint16_t camera_ext_get_phone_ver(void);

/* Functions to for v4l2 controls */
int cam_ext_ctrl_get_cfg(struct camera_ext_ctrl_db *ctrl_db, uint32_t idx,
    struct camera_ext_predefined_ctrl_mod_cfg *cfg, uint32_t cfg_size);

/* Common event functions for driver to pick up */
int camera_ext_register_event_cb(struct device *dev, camera_ext_event_cb_t cb);

/* Send error event to AP */
int camera_ext_send_error(uint32_t err);

/* Send metadata event to AP*/
int camera_ext_send_metadata(const uint8_t *desc, uint32_t length);

#endif
