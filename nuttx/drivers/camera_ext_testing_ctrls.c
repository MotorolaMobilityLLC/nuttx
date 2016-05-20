#include <errno.h>
#include "camera_ext.h"
#include "camera_ext_csi.h"
#include "greybus/v4l2_camera_ext_ctrls.h"
#include "greybus/camera_ext_dbg.h"

static int ctrl_val_set(struct device *dev,
        const struct camera_ext_ctrl_cfg *self,
        const camera_ext_ctrl_val_t *val)
{
    int is_array;
    size_t i;
    int retval = 0;

    is_array = self->val_cfg.nr_of_elem > 1;
    CAM_DBG("id %x\n", self->id);

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
        default:
            CAM_ERR("error type %d ", self->val_cfg.elem_type);
            retval = -EINVAL;
            break;
        }
    }
    CTRL_DBG("\n");
    return retval;
}

#if 0
/* common function for get control value */
static int ctrl_val_get_volatile(struct device *dev,
        const struct camera_ext_ctrl_cfg *self, camera_ext_ctrl_val_t *val)
{
    CAM_DBG("id %x\n", self->cfg.id);
    return 0;
}
#endif

static const struct camera_ext_ctrl_cfg color_correction_aberration_mode = {
    .id = CAM_EXT_CID_COLOR_CORRECTION_ABERRATION_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
        | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg ae_antibanding_mode = {
    .id = CAM_EXT_CID_AE_ANTIBANDING_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
        | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg ae_exposure_compensation = {
    .id = CAM_EXT_CID_AE_EXPOSURE_COMPENSATION,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_STEP
                | CAMERA_EXT_CTRL_FLAG_NEED_MIN
                | CAMERA_EXT_CTRL_FLAG_NEED_MAX
                | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .step = 2,
    .min = 10,
    .max = 100,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 50,
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
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

#define AE_TARGET_FPS_RANGE(lo, hi) ((((uint64_t) hi) << 32) | (uint32_t)lo)
static const uint64_t ae_target_fps_range_items[] = {
    AE_TARGET_FPS_RANGE(24, 32),
    AE_TARGET_FPS_RANGE(36, 48),
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

static const struct camera_ext_ctrl_cfg awb_mode = {
    .id = CAM_EXT_CID_AWB_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
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
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg control_mode = {
    .id = CAM_EXT_CID_CONTROL_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg scene_mode = {
    .id = CAM_EXT_CID_SCENE_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
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
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg jpeg_gps_location = {
    .id = CAM_EXT_CID_JPEG_GPS_LOCATION,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_DOUBLE,
        .nr_of_elem = 2,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg jpeg_orientation = {
    .id = CAM_EXT_CID_JPEG_ORIENTATION,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg jpeg_quality = {
    .id = CAM_EXT_CID_JPEG_QUALITY,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
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
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg flash_mode = {
    .id = CAM_EXT_CID_FLASH_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const float focal_length_items[] = {
    0.1f, 0.3f, 0.8f, 1.2f, 1.8f, 3.0f,
};

static const struct camera_ext_ctrl_cfg focal_length = {
    .id = CAM_EXT_CID_FOCAL_LENGTH,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_FLOAT
                | CAMERA_EXT_CTRL_FLAG_NEED_DEF
                | CAMERA_EXT_CTRL_FLAG_NEED_MAX
                | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .array_size = ARRAY_SIZE(focal_length_items),
    .menu_float = focal_length_items,
    .menu_skip_mask = 4,
    .max = ARRAY_SIZE(focal_length_items) - 1,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg capabilities = {
    .id = CAM_EXT_CID_CAPABILITIES,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg max_num_output_proc = {
    .id = CAM_EXT_CID_MAX_NUM_OUTPUT_PROC,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 1,
    },
};

static const struct camera_ext_ctrl_cfg max_num_output_proc_stalling = {
    .id = CAM_EXT_CID_MAX_NUM_OUTPUT_PROC_STALLING,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 1, /* max is 1 */
    },
};

static const struct camera_ext_ctrl_cfg max_num_output_raw = {
    .id = CAM_EXT_CID_MAX_NUM_OUTPUT_RAW,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 3,
    },
};

static const struct camera_ext_ctrl_cfg pipleline_max_depth = {
    .id = CAM_EXT_CID_PIPLELINE_MAX_DEPTH,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 5,
    },
};

/* float */
static const struct camera_ext_ctrl_cfg scaler_max_digital_zoom = {
    .id = CAM_EXT_CID_SCALER_MAX_DIGITAL_ZOOM,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 1,
    },
    .def = {
        .val_f = 3.5f,
    },
};

/* read only menu */
static const struct camera_ext_ctrl_cfg scaler_cropping_type = {
    .id = CAM_EXT_CID_SCALER_CROPPING_TYPE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
};

static const uint32_t scaler_crop_region_val_def[4] = {
    100, 200, 300, 400,
};

static const struct camera_ext_ctrl_cfg scaler_crop_region = {
    .id = CAM_EXT_CID_SCALER_CROP_REGION,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 4,
    },
    .def = {
        /* cast type to avoid compile warning */
        .p_val = (uint32_t *)scaler_crop_region_val_def,
    },
    .set_ctrl = ctrl_val_set,
};

static const float sensor_info_physical_size_def[] = {
    2345.13f, 4521.82f
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
    100, 200,
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

static const uint32_t sensor_info_pre_correction_active_array_size_def[4] = {
    10, 20, 100, 300,
};

static const struct camera_ext_ctrl_cfg
        sensor_info_pre_correction_active_array_size = {
    .id = CAM_EXT_CID_SENSOR_INFO_PRE_CORRECTION_ACTIVE_ARRAY_SIZE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 4,
    },
    .def = {
        .p_val = (uint32_t *)sensor_info_pre_correction_active_array_size_def,
    },
};

static const uint32_t sensor_info_active_array_size_def[4] = {
    23, 43, 123, 412,
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

static const struct camera_ext_ctrl_cfg sensor_info_timestamp_source = {
    .id = CAM_EXT_CID_SENSOR_INFO_TIMESTAMP_SOURCE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 10,
    },
};

static const struct camera_ext_ctrl_cfg sensor_orientation = {
    .id = CAM_EXT_CID_SENSOR_ORIENTATION,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
};

static const struct camera_ext_ctrl_cfg statistics_face_detect_mode = {
    .id = CAM_EXT_CID_STATISTICS_FACE_DETECT_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
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
        .val = 3,
    },
};

static const struct camera_ext_ctrl_cfg sync_max_latency = {
    .id = CAM_EXT_CID_SYNC_MAX_LATENCY,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 0, /* max is 1 */
    },
};

static const struct camera_ext_ctrl_cfg control_ae_precapture_trigger = {
    .id = CAM_EXT_CID_CONTROL_AE_PRECATURE_TRIGGER,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg lens_info_focus_distance_calibration = {
    .id = CAM_EXT_CID_LENS_INFO_FOCUS_DISTANCE_CALIBRATION,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 3,
    },
};

static const struct camera_ext_ctrl_cfg lens_info_focus_hyperfocal_distance = {
    .id = CAM_EXT_CID_LENS_INFO_FOCUS_HYPERFOCAL_DISTANCE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 1,
    },
    .def = {
        .val_f = 0.3f,
    },
};

static const struct camera_ext_ctrl_cfg lens_info_minimum_focus_distance = {
    .id = CAM_EXT_CID_LENS_INFO_MINIMUM_FOCUS_DISTANCE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 1,
    },
    .def = {
        .val_f = 0.02f,
    },
};

static const struct camera_ext_ctrl_cfg lens_focus_distance = {
    .id = CAM_EXT_CID_LENS_FOCUS_DISTANCE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 1,
    },
    .def = {
        .val_f = 3.4f,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg lens_optical_stabilization_mode = {
    .id = CAM_EXT_CID_LENS_OPTICAL_STABILIZATION_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 1,
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

static const struct camera_ext_ctrl_cfg reprocess_effective_exposure_factore = {
    .id = CAM_EXT_CID_REPROCESS_EFFECTIVE_EXPOSURE_FACTOR,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 1,
    },
    .def = {
        .val_f = 0.34f,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg reprocess_max_capture_stall = {
    .id = CAM_EXT_CID_REPROCESS_MAX_CAPTURE_STALL,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 4,
    },
};

static const struct camera_ext_ctrl_cfg depth_depth_is_exclusive = {
    .id = CAM_EXT_CID_DEPTH_DEPTH_IS_EXCLUSIVE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_BOOL,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 1,
    },
};

static const struct camera_ext_ctrl_cfg black_level_lock = {
    .id = CAM_EXT_CID_BLACK_LEVEL_LOCK,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_BOOL,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg color_correction_mode = {
    .id = CAM_EXT_CID_COLOR_CORRECTION_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const float color_correction_gains_def[] = {
    1.2f, 56.0f, 21.1f, 21.9f,
};

static const struct camera_ext_ctrl_cfg color_correction_gains = {
    .id = CAM_EXT_CID_COLOR_CORRECTION_GAINS,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 4,
    },
    .def = {
        .p_val_f = (float *)color_correction_gains_def,
    },
    .set_ctrl = ctrl_val_set,
};

static const float color_correction_transform_matrix_def[] = {
    1.0f, 0,    0,
    0,    1.0f, 0,
    0,    0,    1.0f,
};

static const struct camera_ext_ctrl_cfg color_correction_transform = {
    .id = CAM_EXT_CID_COLOR_CORRECTION_TRANSFORM,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 9,
    },
    .def = {
        .p_val_f = (float *)color_correction_transform_matrix_def,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg edge_mode = {
    .id = CAM_EXT_CID_EDGE_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const float lens_apertures_items[] = {
    0.5f, 0.8f, 1.3f, 2.1f, 3.8f, 5.6f,
};

static const struct camera_ext_ctrl_cfg lens_apertures = {
    .id = CAM_EXT_CID_LENS_APERTURES,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_FLOAT
        | CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MAX,
    .array_size = ARRAY_SIZE(lens_apertures_items),
    .menu_float = lens_apertures_items,
    .max = ARRAY_SIZE(lens_apertures_items) - 1,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const float lens_filter_density_items[] = {
    0.8f, 1.9f, 9.4f, 11.4f,
};

static const struct camera_ext_ctrl_cfg lens_filter_density = {
    .id = CAM_EXT_CID_LENS_FILTER_DENSITY,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_FLOAT
        | CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MAX,
    .array_size = ARRAY_SIZE(lens_filter_density_items),
    .menu_float = lens_filter_density_items,
    .max = ARRAY_SIZE(lens_filter_density_items) - 1,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg noise_reduction_mode = {
    .id = CAM_EXT_CID_NOISE_REDUCTION_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg request_max_num_input_stream = {
    .id = CAM_EXT_CID_REQUEST_MAX_NUM_INPUT_STREAM,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 2,
    },
};

static const struct camera_ext_ctrl_cfg rquest_partial_result_count = {
    .id = CAM_EXT_CID_REQUEST_PARTIAL_RESULT_COUNT,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 3,
    },
};

static const struct camera_ext_ctrl_cfg sensor_exposure_time = {
    .id = CAM_EXT_CID_SENSOR_EXPOSURE_TIME,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MIN
        | CAMERA_EXT_CTRL_FLAG_NEED_MAX
        | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .min = 10,
    .max = 100,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT64,
        .nr_of_elem = 1,
    },
    .def = {
        .val_64 = 19,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg sensor_frame_duration = {
    .id = CAM_EXT_CID_SENSOR_FRAME_DURATION,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MAX
        | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .max = 1000,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT64,
        .nr_of_elem = 1,
    },
    .def = {
        .val_64 = 30,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg sensor_sensitivity = {
    .id = CAM_EXT_CID_SENSOR_SENSITIVITY,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 120,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg sensor_info_color_filter_arrangement = {
    .id = CAM_EXT_CID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
};

static const struct camera_ext_ctrl_cfg sensor_max_analog_sensitivity = {
    .id = CAM_EXT_CID_SENSOR_MAX_ANALOG_SENSITIVITY,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 133,
    },
};

static const struct camera_ext_ctrl_cfg shading_mode = {
    .id = CAM_EXT_CID_SHADING_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
        | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg statistics_lens_shading_map_mode = {
    .id = CAM_EXT_CID_STATISTICS_LENS_SHADING_MAP_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
        | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

/* points of tonemap curve */
#define TONEMAP_CURVE_CTRL_POINTS 18

static const uint32_t tonemap_curve_dims[] = {2, 3, TONEMAP_CURVE_CTRL_POINTS};

static const float tonemap_curve_points_def[2 * 3 * TONEMAP_CURVE_CTRL_POINTS] = {
    0.3, 0.1, 0.5, 0.2, 0.9, 1.3,
    1.3, 1.1, 1.5, 1.2, 1.9, 2.3,
    2.3, 2.1, 2.5, 2.2, 2.9, 2.3,
};

static const struct camera_ext_ctrl_cfg tonemap_curve = {
    .id = CAM_EXT_CID_TONEMAP_CURVE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DIMS
                | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .array_size = ARRAY_SIZE(tonemap_curve_dims),
    .dims = tonemap_curve_dims,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 2 * 3 * TONEMAP_CURVE_CTRL_POINTS,
    },
    .def = {
        .p_val_f = (float *)tonemap_curve_points_def,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg tonemap_gamma = {
    .id = CAM_EXT_CID_TONEMAP_GAMMA,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 1,
    },
    .def = {
        .val_f = 3.1f,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg tonemap_mode = {
    .id = CAM_EXT_CID_TONEMAP_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
            | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg tonemap_preset_curve = {
    .id = CAM_EXT_CID_TONEMAP_PRESET_CURVE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg tonemap_max_curve_points = {
    .id = CAM_EXT_CID_TONEMAP_MAX_CURVE_POINSTS,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 100,
    },
};

#define MAX_AE_REGIONS 5

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

#define MAX_AF_REGIONS 3

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

#define MAX_AWB_REGIONS 2

static const uint32_t awb_regions_dims[] = {4, MAX_AWB_REGIONS};

static const struct camera_ext_ctrl_cfg awb_regions = {
    .id = CAM_EXT_CID_AWB_REGIONS,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DIMS,
    .array_size = ARRAY_SIZE(awb_regions_dims),
    .dims = awb_regions_dims,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 4 * MAX_AWB_REGIONS,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg hot_pixel_mode = {
    .id = CAM_EXT_CID_HOT_PIXEL_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
        | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const float lens_intrinsic_calibration_def[5] = {
    2.3f, 32.1f, 223.0f, 12.12f, 131.1f,
};

static const struct camera_ext_ctrl_cfg lens_intrinsic_calibration = {
    .id = CAM_EXT_CID_LENS_INTRINSIC_CALIBRATION,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 5,
    },
    .def = {
        .p_val_f = (float *)lens_intrinsic_calibration_def,
    },
};

static const float lens_pos_rotation_def[7] = {
    0.12f, 0.23f, 21.3f, 32.1f, 0.11f, 3.12f, 5.2f,
};

static const struct camera_ext_ctrl_cfg lens_pos_rotation = {
    .id = CAM_EXT_CID_LENS_POSE_ROTATION,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 7,
    },
    .def = {
        .p_val_f = (float *)lens_pos_rotation_def,
    },
};

static const float lens_pos_translation_def[3] = {
    32.23f, 232.1f, 50.9f,
};

static const struct camera_ext_ctrl_cfg lens_pos_translation = {
    .id = CAM_EXT_CID_LENS_POSE_TRANSLATION,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 3,
    },
    .def = {
        .p_val_f = (float *)lens_pos_translation_def,
    },
};

static const float lens_radial_distortion_def[6] = {
    9.2f, 8.23f, 80.2f, 24.24f, 0.234f, 23.02f,
};

static const struct camera_ext_ctrl_cfg lens_radial_distortion = {
    .id = CAM_EXT_CID_LENS_RADIAL_DISTORTION,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 6,
    },
    .def = {
        .p_val_f = (float *)lens_radial_distortion_def,
    },
};

static const struct camera_ext_ctrl_cfg sensor_test_pattern_data = {
    .id = CAM_EXT_CID_SENSOR_TEST_PATTERN_DATA,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 4,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg sensor_test_pattern_mode = {
    .id = CAM_EXT_CID_SENSOR_TEST_PATTERN_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const uint32_t sensor_black_level_pattern_def[4] = {
    23, 65, 62, 12,
};

static const struct camera_ext_ctrl_cfg sensor_black_level_pattern = {
    .id = CAM_EXT_CID_SENSOR_BLACK_LEVEL_PATTERN,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 4,
    },
    .def = {
        .p_val = (uint32_t *)sensor_black_level_pattern_def,
    },
};

static const float sensor_calibration_transform1_def[9] = {
    1.0f, 0, 0,
    0, 1.0f, 0,
    0, 0, 1.0f,
};

static const struct camera_ext_ctrl_cfg sensor_calibration_transform1 = {
    .id = CAM_EXT_CID_SENSOR_CALIBRATION_TRANSFORM1,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 9,
    },
    .def = {
        .p_val_f = (float *)sensor_calibration_transform1_def,
    },
};

static const float sensor_calibration_transform2_def[9] = {
    4.0f, 0, 0,
    0, 2.0f, 0,
    0, 0, 3.0f,
};

static const struct camera_ext_ctrl_cfg sensor_calibration_transform2 = {
    .id = CAM_EXT_CID_SENSOR_CALIBRATION_TRANSFORM2,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 9,
    },
    .def = {
        .p_val_f = (float *)sensor_calibration_transform2_def,
    },
};

static const float sensor_color_transform1_def[9] = {
    3.4f, 23.1f, 12.2f,
    2.0f, 28.1f, 23.2f,
    90.1f, 33.2f, 99.1f,
};

static const struct camera_ext_ctrl_cfg sensor_color_transform1 = {
    .id = CAM_EXT_CID_SENSOR_COLOR_TRANSFORM1,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 9,
    },
    .def = {
        .p_val_f = (float *)sensor_color_transform1_def,
    },
};

static const float sensor_color_transform2_def[9] = {
    2.4f, 23.1f, 12.2f,
    2.0f, 38.1f, 13.2f,
    60.1f, 33.2f, 19.1f,
};

static const struct camera_ext_ctrl_cfg sensor_color_transform2 = {
    .id = CAM_EXT_CID_SENSOR_COLOR_TRANSFORM2,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 9,
    },
    .def = {
        .p_val_f = (float *)sensor_color_transform2_def,
    },
};

static const float sensor_forward_matrix1_def[9];

static const struct camera_ext_ctrl_cfg sensor_forward_matrix1 = {
    .id = CAM_EXT_CID_SENSOR_FORWARD_MATRIX1,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 9,
    },
    .def = {
        .p_val_f = (float *)sensor_forward_matrix1_def,
    },
};

static const float sensor_forward_matrix2_def[9];

static const struct camera_ext_ctrl_cfg sensor_forward_matrix2 = {
    .id = CAM_EXT_CID_SENSOR_FORWARD_MATRIX2,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 9,
    },
    .def = {
        .p_val_f = (float *)sensor_forward_matrix2_def,
    },
};

static const struct camera_ext_ctrl_cfg sensor_info_lens_shading_applied = {
    .id = CAM_EXT_CID_SENSOR_INFO_LENS_SHADING_APPLIED,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_BOOL,
        .nr_of_elem = 1,
    },
};

static const struct camera_ext_ctrl_cfg sensor_info_white_level = {
    .id = CAM_EXT_CID_SENSOR_INFO_WHITE_LEVEL,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 22,
    },
};

static const struct camera_ext_ctrl_cfg sensor_preference_illuminant1 = {
    .id = CAM_EXT_CID_SENSOR_PREFERENCE_ILLUMINANT1,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 11,
    },
};

static const struct camera_ext_ctrl_cfg sensor_preference_illuminant2 = {
    .id = CAM_EXT_CID_SENSOR_PREFERENCE_ILLUMINANT2,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 12,
    },
};

static const struct camera_ext_ctrl_cfg statistics_hot_pixel_map_mode = {
    .id = CAM_EXT_CID_STATISTICS_HOT_PIXEL_MAP_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_BOOL,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 1,
    },
    .set_ctrl = ctrl_val_set,
};

#define HOT_PIXEL_NUM 3

static const uint32_t hot_pixel_map_dims[] = {2, HOT_PIXEL_NUM};

static const uint32_t hot_pixels[HOT_PIXEL_NUM][2] = {
    {22, 221},
    {242,133},
    {211,521},
};

static const struct camera_ext_ctrl_cfg hot_pixel_map = {
    .id = CAM_EXT_CID_HOT_PIXEL_MAP,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
            | CAMERA_EXT_CTRL_FLAG_NEED_DIMS,
    .dims = hot_pixel_map_dims,
    .array_size = ARRAY_SIZE(hot_pixel_map_dims),
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = HOT_PIXEL_NUM * 2,
    },
    .def = {
        .p_val = (uint32_t *)hot_pixels,
    },
};

static const struct camera_ext_ctrl_cfg start_capture = {
    .id = CAM_EXT_CID_START_CAPTURE,
    .flags = 0,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg abort_capture = {
    .id = CAM_EXT_CID_ABORT_CAPTURE,
    .flags = 0,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg iso = {
    .id = CAM_EXT_CID_ISO,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
        | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
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
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg *testing_ctrls[] = {
    &color_correction_aberration_mode,
    &ae_antibanding_mode,
    &ae_exposure_compensation,
    &ae_lock,
    &ae_mode,
    &ae_target_fps_range,
    &af_mode,
    &af_trigger,
    &awb_lock,
    &awb_mode,
    &effect_mode,
    &control_mode,
    &scene_mode,
    &video_stabilization,
    &jpeg_gps_location,
    &jpeg_orientation,
    &jpeg_quality,
    &lens_facing,
    &flash_mode,
    &focal_length,
    &capabilities,
    &max_num_output_proc,
    &max_num_output_proc_stalling,
    &max_num_output_raw,
    &pipleline_max_depth,
    &scaler_max_digital_zoom,
    &scaler_cropping_type,
    &scaler_crop_region,
    &sensor_info_physical_size,
    &sensor_info_pixel_array_size,
    &sensor_info_pre_correction_active_array_size,
    &sensor_info_active_array_size,
    &sensor_info_timestamp_source,
    &sensor_orientation,
    &statistics_face_detect_mode,
    &statistics_info_max_face_count,
    &sync_max_latency,
    &control_ae_precapture_trigger,
    &lens_info_focus_distance_calibration,
    &lens_info_focus_hyperfocal_distance,
    &lens_info_minimum_focus_distance,
    &lens_focus_distance,
    &lens_optical_stabilization_mode,
    &lens_manual_focus_position,
    &reprocess_effective_exposure_factore,
    &reprocess_max_capture_stall,
    &depth_depth_is_exclusive,
    &black_level_lock,
    &color_correction_mode,
    &color_correction_gains,
    &color_correction_transform,
    &edge_mode,
    &lens_apertures,
    &lens_filter_density,
    &noise_reduction_mode,
    &request_max_num_input_stream,
    &rquest_partial_result_count,
    &sensor_exposure_time,
    &sensor_frame_duration,
    &sensor_sensitivity,
    &sensor_info_color_filter_arrangement,
    &sensor_max_analog_sensitivity,
    &shading_mode,
    &statistics_lens_shading_map_mode,
    &tonemap_curve,
    &tonemap_gamma,
    &tonemap_mode,
    &tonemap_preset_curve,
    &tonemap_max_curve_points,
    &ae_regions,
    &af_regions,
    &awb_regions,
    &hot_pixel_mode,
    &lens_intrinsic_calibration,
    &lens_pos_rotation,
    &lens_pos_translation,
    &lens_radial_distortion,
    &sensor_test_pattern_data,
    &sensor_test_pattern_mode,
    &sensor_black_level_pattern,
    &sensor_calibration_transform1,
    &sensor_calibration_transform2,
    &sensor_color_transform1,
    &sensor_color_transform2,
    &sensor_forward_matrix1,
    &sensor_forward_matrix2,
    &sensor_info_lens_shading_applied,
    &sensor_info_white_level,
    &sensor_preference_illuminant1,
    &sensor_preference_illuminant2,
    &statistics_hot_pixel_map_mode,
    &hot_pixel_map,
    &start_capture,
    &abort_capture,
    &iso,
    &nd_filter,
    &jpeg_sharpness,
    &jpeg_contrast,
    &jpeg_saturation,
    &time_sync,
    &jpeg_gps_timestamp,
    &jpeg_gps_proc_method,
    &face_detection,
};

int camera_ext_tesing_ctrl_init(struct device *dev)
{
    return register_camera_ext_ctrl_db(dev, testing_ctrls,
                ARRAY_SIZE(testing_ctrls));
}

