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
#include <time.h>
#include <nuttx/config.h>
#include <nuttx/device_cam_ext.h>
#include <nuttx/gpio.h>
#include <nuttx/i2c.h>
#include <nuttx/math.h>
#include <nuttx/mhb/mhb_csi_camera.h>
#include <nuttx/version.h>

#include <nuttx/camera/camera_ext.h>
#include <nuttx/camera/v4l2_camera_ext_ctrls.h>

/* Make a copy and modify for your camera MOD */

static const char model_number_str[CAM_EXT_CTRL_STRING_MAX_LEN] = {
    'M', 'D', 'K', ' ', 'C', 'A', 'M', 'E',
    'R', 'A', ' ', 'I', 'M', 'X', '2', '1',
    '9', '-', 'P', 'I',  0 ,  0 ,  0 ,  0 ,
     0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,
     0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,
     0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,
     0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,
     0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,
};

static int ctrl_val_set(struct device *dev,
        const struct camera_ext_ctrl_cfg *self,
        const camera_ext_ctrl_val_t *val)
{
    int is_array;
    size_t i;
    int retval = 0;

    is_array = self->val_cfg.nr_of_elem > 1;

    CTRL_DBG("control 0x%08x\n", self->id);

    for (i = 0; i < self->val_cfg.nr_of_elem && retval == 0; i++) {
        switch (self->val_cfg.elem_type) {
        case CAM_EXT_CTRL_DATA_TYPE_INT:
        case CAM_EXT_CTRL_DATA_TYPE_BOOL:
            if (!is_array) {
                CTRL_DBG("%d ", val->val);
            } else {
                CTRL_DBG("%d ", val->p_val[i]);
            }
            break;
        case CAM_EXT_CTRL_DATA_TYPE_INT64:
            if (!is_array) {
                CTRL_DBG("%lld ", val->val_64);
            } else {
                CTRL_DBG("%lld ", val->p_val_64[i]);
            }
            break;
        case CAM_EXT_CTRL_DATA_TYPE_FLOAT:
            if (!is_array) {
                CTRL_DBG("%.6f ", val->val_f);
            } else {
                CTRL_DBG("%.6f ", val->p_val_f[i]);
            }
            break;
        case CAM_EXT_CTRL_DATA_TYPE_DOUBLE:
            if (!is_array) {
                CTRL_DBG("%.12f ", val->val_d);
            } else {
                CTRL_DBG("%.12f ", val->p_val_d[i]);
            }
            break;
        case CAM_EXT_CTRL_DATA_TYPE_STRING:
            if (!is_array) {
                CTRL_DBG("%s ", val->p_val_8);
            }
            break;
        default:
            CAM_ERR("error type %d ", self->val_cfg.elem_type);
            return -EINVAL;
        }
    }
    CTRL_DBG("\n");

    return retval;
}

static int ctrl_volatile_get(struct device *dev,
        const struct camera_ext_ctrl_cfg *self,
        camera_ext_ctrl_val_t *val)
{
    int retval = 0;
    uint32_t value = 0x00000001;

    switch (self->id) {
        case CAM_EXT_CID_FIRMWARE_VERSION:
            val->p_val_8 = self->def.p_val_8;
            snprintf(( char *)val->p_val_8,
                     CAM_EXT_CTRL_STRING_MAX_LEN,
                     "VER %d.%d",
                     value >> 16,
                     value & 0xFFFF);
            break;
        default:
            break;
    }

    return retval;
}

static const struct camera_ext_ctrl_cfg ae_antibanding_mode = {
    .id = CAM_EXT_CID_AE_ANTIBANDING_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
        | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .menu_skip_mask = 0x0E,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = CAM_EXT_AE_ANTIBANDING_OFF,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg ae_exposure_compensation = {
    .id = CAM_EXT_CID_AE_EXPOSURE_COMPENSATION,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_STEP
                | CAMERA_EXT_CTRL_FLAG_NEED_MIN
                | CAMERA_EXT_CTRL_FLAG_NEED_MAX
                | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .step = 1,
    .min = -5,
    .max = +5,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 0,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg ae_lock = {
    .id = CAM_EXT_CID_AE_LOCK,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_BOOL,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg ae_mode = {
    .id = CAM_EXT_CID_AE_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .menu_skip_mask = 0xFE,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = CAM_EXT_AE_MODE_OFF,
    },
    .set_ctrl = ctrl_val_set,
};

#define AE_TARGET_FPS_RANGE(lo, hi) ((((uint64_t) hi) << 32) | (uint32_t)lo)
static const uint64_t ae_target_fps_range_items[] = {
    AE_TARGET_FPS_RANGE(10, 30),
};

static const struct camera_ext_ctrl_cfg ae_target_fps_range = {
    .id = CAM_EXT_CID_AE_TARGET_FPS_RANGE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
                | CAMERA_EXT_CTRL_FLAG_NEED_MAX
                | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
                | CAMERA_EXT_CTRL_FLAG_NEED_MENU_INT,
    .max = ARRAY_SIZE(ae_target_fps_range_items) - 1,
    .menu_skip_mask = 0,
    .array_size = ARRAY_SIZE(ae_target_fps_range_items),
    .menu_int = (int64_t*)ae_target_fps_range_items,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg af_mode = {
    .id = CAM_EXT_CID_AF_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .menu_skip_mask = 0xFE,
    .def = {
        .val = CAM_EXT_AF_MODE_OFF,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg awb_mode = {
    .id = CAM_EXT_CID_AWB_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .menu_skip_mask = 0xFC,
    .def = {
        .val = CAM_EXT_AWB_MODE_AUTO,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg effect_mode = {
    .id = CAM_EXT_CID_EFFECT_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .menu_skip_mask = 0xFFE,
    .def = {
        .val = CAM_EXT_EFFECT_MODE_OFF,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg scene_mode = {
    .id = CAM_EXT_CID_SCENE_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .min = 0,
    .menu_skip_mask = 0xFFFFE,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = CAM_EXT_SCENE_MODE_DISABLED,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg video_stabilization = {
    .id = CAM_EXT_CID_VIDEO_STABILIZATION_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .menu_skip_mask = 0x1,
    .def = {
        .val = CAM_EXT_VIDEO_STABILIZATION_MODE_OFF,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg jpeg_gps_location = {
    .id = CAM_EXT_CID_JPEG_GPS_LOCATION,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_DOUBLE,
        .nr_of_elem = 3,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg jpeg_orientation = {
    .id = CAM_EXT_CID_JPEG_ORIENTATION,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .min = 0,
    .max = 3,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = CAM_EXT_JPEG_ORIENTATION_0,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg jpeg_quality = {
    .id = CAM_EXT_CID_JPEG_QUALITY,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .min = 0,
    .max = 100,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 85,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg lens_facing = {
    .id = CAM_EXT_CID_LENS_FACING,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = CAM_EXT_LENS_FACING_BACK,
    },
};

static const struct camera_ext_ctrl_cfg sensor_orientation = {
    .id = CAM_EXT_CID_SENSOR_ORIENTATION,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = CAM_EXT_SENSOR_ORIENTATION_90,
    },
};

static const struct camera_ext_ctrl_cfg flash_mode = {
    .id = CAM_EXT_CID_FLASH_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .menu_skip_mask = 0x6,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const float focal_length_items[] = {
    3.0f,
};

static const struct camera_ext_ctrl_cfg focal_length = {
    .id = CAM_EXT_CID_FOCAL_LENGTH,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_FLOAT
                | CAMERA_EXT_CTRL_FLAG_NEED_DEF
                | CAMERA_EXT_CTRL_FLAG_NEED_MAX
                | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .array_size = ARRAY_SIZE(focal_length_items),
    .menu_float = focal_length_items,
    .max = ARRAY_SIZE(focal_length_items) - 1,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg statistics_info_max_face_count = {
    .id = CAM_EXT_CID_STATISTICS_INFO_MAX_FACE_COUNT,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 1,
    },
};

static const struct camera_ext_ctrl_cfg sensor_exposure_time = {
    .id = CAM_EXT_CID_SENSOR_EXPOSURE_TIME,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MIN
        | CAMERA_EXT_CTRL_FLAG_NEED_MAX
        | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .min = 500000ULL,
    .max = 4000000000ULL,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT64,
        .nr_of_elem = 1,
    },
    .def = {
        .val_64 = 8000000ULL,
    },
    .set_ctrl = ctrl_val_set,
};

#define MAX_AE_REGIONS 1

static const uint32_t ae_regions_dims[] = {4, MAX_AE_REGIONS};

/* no def */
static const struct camera_ext_ctrl_cfg ae_regions = {
    .id = CAM_EXT_CID_AE_REGIONS,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DIMS,
    .array_size = ARRAY_SIZE(ae_regions_dims),
    .dims = ae_regions_dims,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 4 * MAX_AE_REGIONS,
    },
    .set_ctrl = ctrl_val_set,
};

#define MAX_AF_REGIONS 1

static const uint32_t af_regions_dims[] = {4, MAX_AF_REGIONS};

static const struct camera_ext_ctrl_cfg af_regions = {
    .id = CAM_EXT_CID_AF_REGIONS,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DIMS,
    .array_size = ARRAY_SIZE(af_regions_dims),
    .dims = af_regions_dims,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 4 * MAX_AF_REGIONS,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg capture = {
    .id = CAM_EXT_CID_CAPTURE,
    .flags = 0,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg af_mode_ext = {
    .id = CAM_EXT_CID_AF_MODE_EXT,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
        | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .menu_skip_mask = 0xFE,
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg iso = {
    .id = CAM_EXT_CID_ISO,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
        | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .menu_skip_mask = 2,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg nd_filter = {
    .id = CAM_EXT_CID_ND_FILTER,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
        | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .max = 2,
    .step = 1,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg jpeg_sharpness = {
    .id = CAM_EXT_CID_JPEG_SHARPNESS,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg jpeg_contrast = {
    .id = CAM_EXT_CID_JPEG_CONTRAST,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg jpeg_saturation = {
    .id = CAM_EXT_CID_JPEG_SATURATION,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg time_sync = {
    .id = CAM_EXT_CID_TIME_SYNC,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MIN
        | CAMERA_EXT_CTRL_FLAG_NEED_MAX
        | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .min = 0,
    .max = INT64_MAX,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT64,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg jpeg_gps_timestamp = {
    .id = CAM_EXT_CID_JPEG_GPS_TIMESTAMP,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MIN
        | CAMERA_EXT_CTRL_FLAG_NEED_MAX
        | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .min = 0,
    .max = INT64_MAX,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT64,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg jpeg_gps_proc_method = {
    .id = CAM_EXT_CID_JPEG_GPS_PROC_METHOD,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_STRING,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg face_detection = {
    .id = CAM_EXT_CID_FACE_DETECTION,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
        | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .menu_skip_mask = 0x0E,
    .set_ctrl = ctrl_val_set,
};

/* has_uvc, vid, pid */
static const unsigned int uvc_cfg[] = {
    0, 0x0000, 0x0000
};

static const struct camera_ext_ctrl_cfg uvc_snapshot = {
    .id = CAM_EXT_CID_MOD_CAPS_UVC_SNAPSHOT,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 3,
    },
    .def = {
        .p_val = (unsigned int *)uvc_cfg,
    },
};

static const struct camera_ext_ctrl_cfg meta_data_path = {
    .id = CAM_EXT_CID_MOD_META_DATA_PATH,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = CAM_EXT_CID_MOD_META_DATA_PATH_GB,
    },
};

/* line number * smallest frame width >= size of meta data */
static const struct camera_ext_ctrl_cfg meta_data_size = {
    .id = CAM_EXT_CID_MOD_META_DATA_SIZE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 3184, /* meta data size */
    },
};

static const struct camera_ext_ctrl_cfg scaler_max_digital_zoom = {
    .id = CAM_EXT_CID_SCALER_MAX_DIGITAL_ZOOM,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 1,
    },
    .def = {
        .val_f = 1.0f,
    },
};

static const struct camera_ext_ctrl_cfg lens_optical_stabilization_mode = {
    .id = CAM_EXT_CID_LENS_OPTICAL_STABILIZATION_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .menu_skip_mask = 0x02,
    .def = {
        .val = CAM_EXT_CID_LENS_OPTICAL_STABILIZATION_MODE_OFF,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg lens_manual_focus_position = {
    .id = CAM_EXT_CID_MANUAL_FOCUS_POSITION,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MIN
        | CAMERA_EXT_CTRL_FLAG_NEED_MAX,
    .min = 0,
    .max = 100,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg af_trigger = {
    .id = CAM_EXT_CID_AF_TRIGGER,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg awb_lock = {
    .id = CAM_EXT_CID_AWB_LOCK,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_BOOL,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg zoom_lock_1x = {
    .id = CAM_EXT_CID_ZOOM_LOCK_1X,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_BOOL,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg model_number = {
    .id = CAM_EXT_CID_MODEL_NUMBER,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_STRING,
        .nr_of_elem = 1,
    },
    .def = {
        .p_val_8 = (uint8_t *)model_number_str,
    },
};

static char firmware_version_str[CAM_EXT_CTRL_STRING_MAX_LEN];

static const struct camera_ext_ctrl_cfg firmware_version = {
    .id = CAM_EXT_CID_FIRMWARE_VERSION,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_STRING,
        .nr_of_elem = 1,
    },
    .def = {
        .p_val_8 = (uint8_t *)firmware_version_str,
    },
    .get_volatile_ctrl = ctrl_volatile_get,
};

static const struct camera_ext_ctrl_cfg ae_mode_ext = {
    .id = CAM_EXT_CID_AE_MODE_EXT,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
        | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .menu_skip_mask = 0xFE,
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg scene_mode_ext = {
    .id = CAM_EXT_CID_SCENE_MODE_EXT,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
        | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .menu_skip_mask = 0xFE,
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg zoom_limit = {
    .id = CAM_EXT_CID_ZOOM_LIMIT,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .min = 100,
    .max = 100,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 100,
    },
    .set_ctrl = ctrl_val_set,
};

static const float sensor_info_physical_size_def[] = {
    3.67f, 2.76f
};

static const struct camera_ext_ctrl_cfg sensor_info_physical_size = {
    .id = CAM_EXT_CID_SENSOR_INFO_PHYSICAL_SIZE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 2,
    },
    .def = {
        .p_val_f = (float *)sensor_info_physical_size_def,
    },
};

static const uint32_t sensor_info_pixel_array_size_def[2] = {
    3296, 2512,
};

static const struct camera_ext_ctrl_cfg sensor_info_pixel_array_size = {
    .id = CAM_EXT_CID_SENSOR_INFO_PIXEL_ARRAY_SIZE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 2,
    },
    .def = {
        .p_val = (uint32_t *)sensor_info_pixel_array_size_def,
    },
};

static const uint32_t sensor_info_active_array_size_def[4] = {
    8, 24, 3288, 2488,
};

static const struct camera_ext_ctrl_cfg sensor_info_active_array_size = {
    .id = CAM_EXT_CID_SENSOR_INFO_ACTIVE_ARRAY_SIZE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 4,
    },
    .def = {
        .p_val = (uint32_t *)sensor_info_active_array_size_def,
    },
};

static const uint32_t jpeg_available_thumbnail_sizes_items[][2] = {
    {0,   0},
};

static const uint32_t jpeg_available_thumbnail_sizes_dims[] =
            {2, ARRAY_SIZE(jpeg_available_thumbnail_sizes_items)};

static const struct camera_ext_ctrl_cfg jpeg_available_thumbnail_sizes = {
    .id = CAM_EXT_CID_JPEG_AVAILABLE_THUMBNAIL_SIZES,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_DIMS,
    .array_size = ARRAY_SIZE(jpeg_available_thumbnail_sizes_dims),
    .dims = jpeg_available_thumbnail_sizes_dims,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 2 * ARRAY_SIZE(jpeg_available_thumbnail_sizes_items),
    },
    .def = {
        .p_val = (uint32_t *)jpeg_available_thumbnail_sizes_items,
    },
};

static const struct camera_ext_ctrl_cfg jpeg_thumbnail_size_index = {
    .id = CAM_EXT_CID_JPEG_THUMBNAIL_SIZE_INDEX,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .min = 0,
    .max = ARRAY_SIZE(jpeg_available_thumbnail_sizes_items) - 1,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = ARRAY_SIZE(jpeg_available_thumbnail_sizes_items) - 1,
    },
    .set_ctrl = ctrl_val_set,
};

static int group_ind_set(struct device *dev,
        const struct camera_ext_ctrl_cfg *self,
        const camera_ext_ctrl_val_t *val)
{
    return 0;
}

static const struct camera_ext_ctrl_cfg group_ind = {
    .id = CAM_EXT_CID_GROUP_IND,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = group_ind_set,
};

static const struct camera_ext_ctrl_cfg phone_version = {
    .id = CAM_EXT_CID_PHONE_VERSION,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_STRING,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg supplemental_key_mask = {
    .id = CAM_EXT_CID_SUPPLEMENTAL_KEY_MASK,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 0,
    },
};

static const struct camera_ext_ctrl_cfg video_record_hint = {
    .id = CAM_EXT_CID_VIDEO_RECORD_HINT,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_BOOL,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const float raw_to_yuv_gain_def[] = {
    1.0f, 0.8989f, 1.7245f
};

static const struct camera_ext_ctrl_cfg raw_to_yuv_gain = {
    .id = CAM_EXT_CID_RAW_TO_YUV_GAIN,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 3,
    },
    .def = {
        .p_val_f = (float *)raw_to_yuv_gain_def,
    },
};

static const struct camera_ext_ctrl_cfg *_ctrls[] = {
    &ae_antibanding_mode,
    &ae_exposure_compensation,
    &ae_lock,
    &ae_mode,
    &af_mode,
    &ae_target_fps_range,
    &awb_mode,
    &effect_mode,
    &scene_mode,
    &video_stabilization,
    &jpeg_gps_location,
    &jpeg_orientation,
    &jpeg_quality,
    &lens_facing,
    &sensor_orientation,
    &flash_mode,
    &focal_length,
    &statistics_info_max_face_count,
    &sensor_exposure_time,
    &ae_regions,
    &af_regions,
    &capture,
    &af_mode_ext,
    &iso,
    &nd_filter,
    &jpeg_sharpness,
    &jpeg_contrast,
    &jpeg_saturation,
    &time_sync,
    &jpeg_gps_timestamp,
    &jpeg_gps_proc_method,
    &face_detection,
    &uvc_snapshot,
    &meta_data_path,
    &meta_data_size,
    &scaler_max_digital_zoom,
    &lens_optical_stabilization_mode,
    &lens_manual_focus_position,
    &af_trigger,
    &awb_lock,
    &zoom_lock_1x,
    &model_number,
    &firmware_version,
    &ae_mode_ext,
    &scene_mode_ext,
    &zoom_limit,
    &sensor_info_physical_size,
    &sensor_info_pixel_array_size,
    &sensor_info_active_array_size,
    &jpeg_available_thumbnail_sizes,
    &jpeg_thumbnail_size_index,
    &phone_version,
    &supplemental_key_mask,
    &group_ind,
    &video_record_hint,
    &raw_to_yuv_gain,
};

struct camera_ext_ctrl_db mhb_camera_ctrl_db = {
    .num_ctrls = ARRAY_SIZE(_ctrls),
    .ctrls = _ctrls,
};
