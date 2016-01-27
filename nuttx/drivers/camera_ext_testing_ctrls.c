#include <errno.h>
#include "camera_ext.h"
#include "camera_ext_csi.h"
#include "greybus/v4l2_camera_ext_ctrls.h"
#include "greybus/camera_ext_dbg.h"

/* Common function for set control value.
 * Just dump the value and store them locally.
 */
#ifdef CONFIG_NSH_CONSOLE
#define CTRL_DBG printf
#else
#define CTRL_DBG
#endif
static int ctrl_val_set(struct device *dev,
        struct camera_ext_ctrl_item *self, struct camera_ext_ctrl_val *val)
{
    int is_array;
    size_t i;
    int retval = 0;

    is_array = val->nr_of_elem > 1;
    CAM_DBG("id %x\n", self->cfg.id);

    for (i = 0; i < val->nr_of_elem && retval == 0; i++) {
        switch (val->elem_type) {
        case CAM_EXT_CTRL_DATA_TYPE_INT:
        case CAM_EXT_CTRL_DATA_TYPE_BOOL:
            if (!is_array) {
                self->val.val = val->val;
                CTRL_DBG("%d ", self->val.val);
            } else {
                self->val.p_val[i] = val->p_val[i];
                CTRL_DBG("%d ", self->val.p_val[i]);
            }
            break;
        case CAM_EXT_CTRL_DATA_TYPE_INT64:
            if (!is_array) {
                self->val.val_64 = val->val_64;
                CTRL_DBG("%lld ", self->val.val_64);
            } else {
                self->val.p_val_64[i] = val->p_val_64[i];
                CTRL_DBG("%lld ", self->val.p_val_64[i]);
            }
            break;
        case CAM_EXT_CTRL_DATA_TYPE_FLOAT:
            if (!is_array) {
                self->val.val_f = val->val_f;
                CTRL_DBG("%.6f ", self->val.val_f);
            } else {
                self->val.p_val_f[i] = val->p_val_f[i];
                CTRL_DBG("%.6f ", self->val.p_val_f[i]);
            }
            break;
        case CAM_EXT_CTRL_DATA_TYPE_DOUBLE:
            if (!is_array) {
                self->val.val_d = val->val_d;
                CTRL_DBG("%.12f ", self->val.val_d);
            } else {
                self->val.p_val_d[i] = val->p_val_d[i];
                CTRL_DBG("%.12f ", self->val.p_val_d[i]);
            }
            break;
        default:
            CAM_ERR("error type %d ", val->elem_type);
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
        struct camera_ext_ctrl_item *self, struct camera_ext_ctrl_val *val)
{
    CAM_DBG("id %x\n", self->cfg.id);
    return 0;
}
#endif

static struct camera_ext_ctrl_item color_correction_aberration_mode = {
    .cfg = {
        .id = CAM_EXT_CID_COLOR_CORRECTION_ABERRATION_MODE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
            | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item ae_antibanding_mode = {
    .cfg = {
        .id = CAM_EXT_CID_AE_ANTIBANDING_MODE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
            | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item ae_exposure_compensation = {
    .cfg = {
        .id = CAM_EXT_CID_AE_EXPOSURE_COMPENSATION,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_STEP
                    | CAMERA_EXT_CTRL_FLAG_NEED_MIN
                    | CAMERA_EXT_CTRL_FLAG_NEED_MAX
                    | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .step = 2,
        .min = 10,
        .max = 100,
        .def = 50,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item ae_lock = {
    .cfg = {
        .id = CAM_EXT_CID_AE_LOCK,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_BOOL,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item ae_mode = {
    .cfg = {
        .id = CAM_EXT_CID_AE_MODE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
            | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    },
    .val = {
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

static struct camera_ext_ctrl_item ae_target_fps_range = {
    .cfg = {
        .id = CAM_EXT_CID_AE_TARGET_FPS_RANGE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
                    | CAMERA_EXT_CTRL_FLAG_NEED_MAX
                    | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
                    | CAMERA_EXT_CTRL_FLAG_NEED_MENU_INT,
        .def = 0,
        .max = ARRAY_SIZE(ae_target_fps_range_items) - 1,
        .menu_skip_mask = 0,
        .array_size = ARRAY_SIZE(ae_target_fps_range_items),
        .menu_int = (int64_t*)ae_target_fps_range_items,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item af_mode = {
    .cfg = {
        .id = CAM_EXT_CID_AF_MODE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
            | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item af_trigger = {
    .cfg = {
        .id = CAM_EXT_CID_AF_TRIGGER,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
            | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item awb_lock = {
    .cfg = {
        .id = CAM_EXT_CID_AWB_LOCK,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_BOOL,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item awb_mode = {
    .cfg = {
        .id = CAM_EXT_CID_AWB_MODE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
            | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item effect_mode = {
    .cfg = {
        .id = CAM_EXT_CID_EFFECT_MODE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
            | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item control_mode = {
    .cfg = {
        .id = CAM_EXT_CID_CONTROL_MODE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
            | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item scene_mode = {
    .cfg = {
        .id = CAM_EXT_CID_SCENE_MODE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
            | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item video_stabilization = {
    .cfg = {
        .id = CAM_EXT_CID_VIDEO_STABILIZATION_MODE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
            | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static double jpeg_gps_loc[2];

static struct camera_ext_ctrl_item jpeg_gps_location = {
    .cfg = {
        .id = CAM_EXT_CID_JPEG_GPS_LOCATION,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_DOUBLE,
        .nr_of_elem = 2,
        /* This sample pre-allocate memory for array type. */
        .p_val_d = jpeg_gps_loc,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item jpeg_orientation = {
    .cfg = {
        .id = CAM_EXT_CID_JPEG_ORIENTATION,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
            | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item jpeg_quality = {
    .cfg = {
        .id = CAM_EXT_CID_JPEG_QUALITY,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item lens_facing = {
    .cfg = {
        .id = CAM_EXT_CID_LENS_FACING,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item flash_mode = {
    .cfg = {
        .id = CAM_EXT_CID_FLASH_MODE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
            | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static float focal_length_items[] = {
    0.1f, 0.3f, 0.8f, 1.2f, 1.8f, 3.0f,
};

static struct camera_ext_ctrl_item focal_length = {
    .cfg = {
        .id = CAM_EXT_CID_FOCAL_LENGTH,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_FLOAT
                    | CAMERA_EXT_CTRL_FLAG_NEED_DEF
                    | CAMERA_EXT_CTRL_FLAG_NEED_MAX
                    | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
        .array_size = ARRAY_SIZE(focal_length_items),
        .menu_float = focal_length_items,
        .menu_skip_mask = 4,
        .def = 1,
        .max = ARRAY_SIZE(focal_length_items) - 1,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item capabilities = {
    .cfg = {
        .id = CAM_EXT_CID_CAPABILITIES,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
            | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item max_num_output_proc = {
    .cfg = {
        .id = CAM_EXT_CID_MAX_NUM_OUTPUT_PROC,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 1,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
};

static struct camera_ext_ctrl_item max_num_output_proc_stalling = {
    .cfg = {
        .id = CAM_EXT_CID_MAX_NUM_OUTPUT_PROC_STALLING,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 1, /* max is 1 */
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
};

static struct camera_ext_ctrl_item max_num_output_raw = {
    .cfg = {
        .id = CAM_EXT_CID_MAX_NUM_OUTPUT_RAW,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 3,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
};

static struct camera_ext_ctrl_item pipleline_max_depth = {
    .cfg = {
        .id = CAM_EXT_CID_PIPLELINE_MAX_DEPTH,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 5,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
};

/* float */
static struct camera_ext_ctrl_item scaler_max_digital_zoom = {
    .cfg = {
        .id = CAM_EXT_CID_SCALER_MAX_DIGITAL_ZOOM,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def_f = 3.5f,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 1,
    },
};

/* read only menu */
static struct camera_ext_ctrl_item scaler_cropping_type = {
    .cfg = {
        .id = CAM_EXT_CID_SCALER_CROPPING_TYPE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
            | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
};

static uint32_t scaler_crop_region_val[4];

static struct camera_ext_ctrl_item scaler_crop_region = {
    .cfg = {
        .id = CAM_EXT_CID_SCALER_CROP_REGION,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 4,
        .p_val = scaler_crop_region_val,
    },
    .set_ctrl = ctrl_val_set,
};

static float sensor_info_physical_size_val[] = {
    2345.13f, 4521.82f
};

static struct camera_ext_ctrl_item sensor_info_physical_size = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_INFO_PHYSICAL_SIZE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def_f = 0.2f,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 2,
        .p_val_f = sensor_info_physical_size_val,
    },
};

static struct camera_ext_ctrl_item sensor_info_pixel_array_size = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_INFO_PIXEL_ARRAY_SIZE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 5,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 2,
    },
};

static struct camera_ext_ctrl_item
        sensor_info_pre_correction_active_array_size = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_INFO_PRE_CORRECTION_ACTIVE_ARRAY_SIZE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 4,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 4,
    },
};

static struct camera_ext_ctrl_item sensor_info_active_array_size = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_INFO_ACTIVE_ARRAY_SIZE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 9,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 4,
    },
};

static struct camera_ext_ctrl_item sensor_info_timestamp_source = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_INFO_TIMESTAMP_SOURCE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 10,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
};

static struct camera_ext_ctrl_item sensor_orientation = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_ORIENTATION,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
};

static struct camera_ext_ctrl_item statistics_face_detect_mode = {
    .cfg = {
        .id = CAM_EXT_CID_STATISTICS_FACE_DETECT_MODE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
            | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item statistics_info_max_face_count = {
    .cfg = {
        .id = CAM_EXT_CID_STATISTICS_INFO_MAX_FACE_COUNT,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 3,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
};

static struct camera_ext_ctrl_item sync_max_latency = {
    .cfg = {
        .id = CAM_EXT_CID_SYNC_MAX_LATENCY,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 0, /* max is 1 */
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
};

static struct camera_ext_ctrl_item control_ae_precapture_trigger = {
    .cfg = {
        .id = CAM_EXT_CID_CONTROL_AE_PRECATURE_TRIGGER,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item lens_info_focus_distance_calibration = {
    .cfg = {
        .id = CAM_EXT_CID_LENS_INFO_FOCUS_DISTANCE_CALIBRATION,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 3,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
};

static struct camera_ext_ctrl_item lens_info_focus_hyperfocal_distance = {
    .cfg = {
        .id = CAM_EXT_CID_LENS_INFO_FOCUS_HYPERFOCAL_DISTANCE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def_f = 0.5f,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 1,
    },
};

static struct camera_ext_ctrl_item lens_info_minimum_focus_distance = {
    .cfg = {
        .id = CAM_EXT_CID_LENS_INFO_MINIMUM_FOCUS_DISTANCE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def_f = 0.02f,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 1,
    },
};

static struct camera_ext_ctrl_item lens_focus_distance = {
    .cfg = {
        .id = CAM_EXT_CID_LENS_FOCUS_DISTANCE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def_f = 3.4f,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item lens_optical_stabilization_mode = {
    .cfg = {
        .id = CAM_EXT_CID_LENS_OPTICAL_STABILIZATION_MODE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
            | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item reprocess_effective_exposure_factore = {
    .cfg = {
        .id = CAM_EXT_CID_REPROCESS_EFFECTIVE_EXPOSURE_FACTOR,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def_f = 0.34f,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item reprocess_max_capture_stall = {
    .cfg = {
        .id = CAM_EXT_CID_REPROCESS_MAX_CAPTURE_STALL,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 4,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
};

static struct camera_ext_ctrl_item depth_depth_is_exclusive = {
    .cfg = {
        .id = CAM_EXT_CID_DEPTH_DEPTH_IS_EXCLUSIVE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 1,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_BOOL,
        .nr_of_elem = 1,
    },
};

static struct camera_ext_ctrl_item black_level_lock = {
    .cfg = {
        .id = CAM_EXT_CID_BLACK_LEVEL_LOCK,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 0,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_BOOL,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item color_correction_mode = {
    .cfg = {
        .id = CAM_EXT_CID_COLOR_CORRECTION_MODE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 1,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static float color_correction_gains_val[] = {
    1.2f, 56.0f, 21.1f, 21.9f,
};

static struct camera_ext_ctrl_item color_correction_gains = {
    .cfg = {
        .id = CAM_EXT_CID_COLOR_CORRECTION_GAINS,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 4,
        .p_val_f = color_correction_gains_val,
    },
    .set_ctrl = ctrl_val_set,
};

static float color_correction_transform_matrix[] = {
    1.0f, 0,    0,
    0,    1.0f, 0,
    0,    0,    1.0f,
};

static struct camera_ext_ctrl_item color_correction_transform = {
    .cfg = {
        .id = CAM_EXT_CID_COLOR_CORRECTION_TRANSFORM,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 2.1f,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 9,
        .p_val_f = color_correction_transform_matrix,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item edge_mode = {
    .cfg = {
        .id = CAM_EXT_CID_EDGE_MODE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
            | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static float lens_apertures_items[] = {
    0.5f, 0.8f, 1.3f, 2.1f, 3.8f, 5.6f,
};

static struct camera_ext_ctrl_item lens_apertures = {
    .cfg = {
        .id = CAM_EXT_CID_LENS_APERTURES,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_FLOAT
            | CAMERA_EXT_CTRL_FLAG_NEED_DEF
            | CAMERA_EXT_CTRL_FLAG_NEED_MAX,
        .array_size = ARRAY_SIZE(lens_apertures_items),
        .menu_float = lens_apertures_items,
        .max = ARRAY_SIZE(lens_apertures_items) - 1,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static float lens_filter_density_items[] = {
    0.8f, 1.9f, 9.4f, 11.4f,
};

static struct camera_ext_ctrl_item lens_filter_density = {
    .cfg = {
        .id = CAM_EXT_CID_LENS_FILTER_DENSITY,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_FLOAT
            | CAMERA_EXT_CTRL_FLAG_NEED_DEF
            | CAMERA_EXT_CTRL_FLAG_NEED_MAX,
        .array_size = ARRAY_SIZE(lens_filter_density_items),
        .menu_float = lens_filter_density_items,
        .max = ARRAY_SIZE(lens_filter_density_items) - 1,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item noise_reduction_mode = {
    .cfg = {
        .id = CAM_EXT_CID_NOISE_REDUCTION_MODE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
            | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item request_max_num_input_stream = {
    .cfg = {
        .id = CAM_EXT_CID_REQUEST_MAX_NUM_INPUT_STREAM,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 2,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
};

static struct camera_ext_ctrl_item rquest_partial_result_count = {
    .cfg = {
        .id = CAM_EXT_CID_REQUEST_PARTIAL_RESULT_COUNT,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 3,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
};

static struct camera_ext_ctrl_item sensor_exposure_time = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_EXPOSURE_TIME,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_MIN
            | CAMERA_EXT_CTRL_FLAG_NEED_MAX
            | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .min = 0,
        .max = 100,
        .def = 19,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT64,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item sensor_frame_duration = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_FRAME_DURATION,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_MAX
            | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .max = 1000,
        .def = 30,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT64,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item sensor_sensitivity = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_SENSITIVITY,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 120,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item sensor_info_color_filter_arrangement = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
};

static struct camera_ext_ctrl_item sensor_max_analog_sensitivity = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_MAX_ANALOG_SENSITIVITY,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 133,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
};

static struct camera_ext_ctrl_item shading_mode = {
    .cfg = {
        .id = CAM_EXT_CID_SHADING_MODE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
                | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item statistics_lens_shading_map_mode = {
    .cfg = {
        .id = CAM_EXT_CID_STATISTICS_LENS_SHADING_MAP_MODE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
                | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

/* points of tonemap curve */
#define TONEMAP_CURVE_CTRL_POINTS 18

static uint32_t tonemap_curve_dims[] = {2, 3, TONEMAP_CURVE_CTRL_POINTS};

/* in real impl, control value maybe stored somewhere else */
static float tonemap_curve_points[2 * 3 * TONEMAP_CURVE_CTRL_POINTS] = {
};

static struct camera_ext_ctrl_item tonemap_curve = {
    .cfg = {
        .id = CAM_EXT_CID_TONEMAP_CURVE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DIMS
                    | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def_f = 0.2f,
        .array_size = ARRAY_SIZE(tonemap_curve_dims),
        .dims = tonemap_curve_dims,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 2 * 3 * TONEMAP_CURVE_CTRL_POINTS,
        .p_val_f = tonemap_curve_points,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item tonemap_gamma = {
    .cfg = {
        .id = CAM_EXT_CID_TONEMAP_GAMMA,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 3.1f,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item tonemap_mode = {
    .cfg = {
        .id = CAM_EXT_CID_TONEMAP_MODE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
                | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item tonemap_preset_curve = {
    .cfg = {
        .id = CAM_EXT_CID_TONEMAP_PRESET_CURVE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item tonemap_max_curve_points = {
    .cfg = {
        .id = CAM_EXT_CID_TONEMAP_MAX_CURVE_POINSTS,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 100,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
};

#define MAX_AE_REGIONS 5

static uint32_t ae_regions_dims[] = {4, MAX_AE_REGIONS};

static uint32_t ae_regions_val[MAX_AE_REGIONS][4] = {
    {10, 20, 110, 201},
    {15, 22, 120, 202},
    {20, 24, 130, 203},
    {25, 25, 140, 204},
    {30, 28, 150, 205},
};

static struct camera_ext_ctrl_item ae_regions = {
    .cfg = {
        .id = CAM_EXT_CID_AE_REGIONS,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DIMS,
        .array_size = ARRAY_SIZE(ae_regions_dims),
        .dims = ae_regions_dims,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = sizeof(ae_regions_val)/sizeof(uint32_t),
        .p_val = &ae_regions_val[0][0],
    },
    .set_ctrl = ctrl_val_set,
};

#define MAX_AF_REGIONS 3

static uint32_t af_regions_dims[] = {4, MAX_AF_REGIONS};

static uint32_t af_regions_val[MAX_AF_REGIONS][4] = {
    {0, 0, 100, 100},
    {120, 120, 200, 210},
    {80, 90, 110, 130},
};

static struct camera_ext_ctrl_item af_regions = {
    .cfg = {
        .id = CAM_EXT_CID_AF_REGIONS,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DIMS,
        .array_size = ARRAY_SIZE(af_regions_dims),
        .dims = af_regions_dims,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = sizeof(af_regions_val)/sizeof(uint32_t),
        .p_val = &af_regions_val[0][0],
    },
    .set_ctrl = ctrl_val_set,
};

#define MAX_AWB_REGIONS 2

static uint32_t awb_regions_dims[] = {4, MAX_AWB_REGIONS};

static uint32_t awb_regions_val[MAX_AWB_REGIONS][4] = {
    {3, 2, 111, 222},
    {5, 21, 131, 542},
};

static struct camera_ext_ctrl_item awb_regions = {
    .cfg = {
        .id = CAM_EXT_CID_AWB_REGIONS,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DIMS,
        .array_size = ARRAY_SIZE(awb_regions_dims),
        .dims = awb_regions_dims,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = sizeof(awb_regions_val)/sizeof(uint32_t),
        .p_val = &awb_regions_val[0][0],
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item hot_pixel_mode = {
    .cfg = {
        .id = CAM_EXT_CID_HOT_PIXEL_MODE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
            | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item lens_intrinsic_calibration = {
    .cfg = {
        .id = CAM_EXT_CID_LENS_INTRINSIC_CALIBRATION,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def_f = 0.4f,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 5,
    },
};

static struct camera_ext_ctrl_item lens_pos_rotation = {
    .cfg = {
        .id = CAM_EXT_CID_LENS_POSE_ROTATION,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def_f = 0.3f,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 7,
    },
};

static struct camera_ext_ctrl_item lens_pos_translation = {
    .cfg = {
        .id = CAM_EXT_CID_LENS_POSE_TRANSLATION,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 0.21f,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 3,
    },
};

static struct camera_ext_ctrl_item lens_radial_distortion = {
    .cfg = {
        .id = CAM_EXT_CID_LENS_RADIAL_DISTORTION,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 0.33f,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 6,
    },
};

static uint32_t sensor_test_pattern_data_val[4];

static struct camera_ext_ctrl_item sensor_test_pattern_data = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_TEST_PATTERN_DATA,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 4,
        .p_val = sensor_test_pattern_data_val,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item sensor_test_pattern_mode = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_TEST_PATTERN_MODE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

static struct camera_ext_ctrl_item sensor_black_level_pattern = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_BLACK_LEVEL_PATTERN,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 1,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 4,
    },
};

static struct camera_ext_ctrl_item sensor_calibration_transform1 = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_CALIBRATION_TRANSFORM1,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def_f = 1.2f,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 9,
    },
};

static struct camera_ext_ctrl_item sensor_calibration_transform2 = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_CALIBRATION_TRANSFORM2,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def_f = 3.3f,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 9,
    },
};

static struct camera_ext_ctrl_item sensor_color_transform1 = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_COLOR_TRANSFORM1,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def_f = 11.0f,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 9,
    },
};

static struct camera_ext_ctrl_item sensor_color_transform2 = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_COLOR_TRANSFORM2,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def_f = 2.3f,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 9,
    },
};

static struct camera_ext_ctrl_item sensor_forward_matrix1 = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_FORWARD_MATRIX1,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def_f = 0.1f,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 9,
    },
};

static struct camera_ext_ctrl_item sensor_forward_matrix2 = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_FORWARD_MATRIX2,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def_f = 0.4f,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 9,
    },
};

static struct camera_ext_ctrl_item sensor_info_lens_shading_applied = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_INFO_LENS_SHADING_APPLIED,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_BOOL,
        .nr_of_elem = 1,
    },
};

static struct camera_ext_ctrl_item sensor_info_white_level = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_INFO_WHITE_LEVEL,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 22,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
};

static struct camera_ext_ctrl_item sensor_preference_illuminant1 = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_PREFERENCE_ILLUMINANT1,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 11,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
};

static struct camera_ext_ctrl_item sensor_preference_illuminant2 = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_PREFERENCE_ILLUMINANT2,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 12,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
};

static struct camera_ext_ctrl_item statistics_hot_pixel_map_mode = {
    .cfg = {
        .id = CAM_EXT_CID_STATISTICS_HOT_PIXEL_MAP_MODE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 1,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_BOOL,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set,
};

#define HOT_PIXEL_NUM 3

static uint32_t hot_pixel_map_dims[] = {2, HOT_PIXEL_NUM};

static uint32_t hot_pixels[HOT_PIXEL_NUM][2] = {
    {22, 221},
    {242,133},
    {211,521},
};

static struct camera_ext_ctrl_item hot_pixel_map = {
    .cfg = {
        .id = CAM_EXT_CID_HOT_PIXEL_MAP,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
                | CAMERA_EXT_CTRL_FLAG_NEED_DIMS,
        .def = 1, /* TODO: support def[] */
        .dims = hot_pixel_map_dims,
        .array_size = ARRAY_SIZE(hot_pixel_map_dims),
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = sizeof(hot_pixels)/sizeof(uint32_t),
    },
};

const struct camera_ext_ctrl_item *testing_ctrls[] = {
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
};

int camera_ext_tesing_ctrl_init(struct device *dev)
{
    return register_camera_ext_ctrl_db(dev, testing_ctrls,
                ARRAY_SIZE(testing_ctrls));
}

