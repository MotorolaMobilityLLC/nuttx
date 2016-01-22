#include "camera_ext.h"
#include "camera_ext_csi.h"
#include "greybus/v4l2_camera_ext_ctrls.h"
#include "greybus/camera_ext_dbg.h"

static int color_correction_aberration_mode_set_ctrl(struct device *dev,
        struct camera_ext_ctrl_item *self, struct camera_ext_ctrl_val *val)
{
    CAM_DBG("%d\n", val->val);
    self->val.val = val->val;
    return 0;
}

struct camera_ext_ctrl_item color_correction_aberration_mode = {
    .cfg = {
        .id = CAM_EXT_CID_COLOR_CORRECTION_ABERRATION_MODE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
                    | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .menu_skip_mask = 0, /* never mask def */
        .def = 1,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = color_correction_aberration_mode_set_ctrl,
};

static int ae_exposure_compensation_set_ctrl(struct device *dev,
        struct camera_ext_ctrl_item *self, struct camera_ext_ctrl_val *val)
{
    CAM_DBG("%d\n", val->val);
    self->val.val = val->val;
    return 0;
}

struct camera_ext_ctrl_item ae_exposure_compensation = {
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
    .set_ctrl = ae_exposure_compensation_set_ctrl,
};

#define AE_TARGET_FPS_RANGE(lo, hi) ((((uint64_t) hi) << 32) | (uint32_t)lo)
static const uint64_t ae_target_fps_range_items[] = {
    AE_TARGET_FPS_RANGE(24, 32),
    AE_TARGET_FPS_RANGE(36, 48),
};

static int ae_target_fps_range_set_ctrl(struct device *dev,
        struct camera_ext_ctrl_item *self, struct camera_ext_ctrl_val *val)
{
    CAM_DBG("%d\n", val->val);
    self->val.val = val->val;
    return 0;
}

struct camera_ext_ctrl_item ae_target_fps_range = {
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
    .set_ctrl = ae_target_fps_range_set_ctrl,
};

static int awb_lock_set_ctrl(struct device *dev,
        struct camera_ext_ctrl_item *self, struct camera_ext_ctrl_val *val)
{
    CAM_DBG("%d\n", val->val);
    self->val.val = val->val;
    return 0;
}

struct camera_ext_ctrl_item awb_lock = {
    .cfg = {
        .id = CAM_EXT_CID_AWB_LOCK,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 1,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_BOOL,
        .nr_of_elem = 1,
    },
    .set_ctrl = awb_lock_set_ctrl,
};

static double jpeg_gps_loc[2];

static int jpeg_gps_location_set_ctrl(struct device *dev,
            struct camera_ext_ctrl_item *self, struct camera_ext_ctrl_val *val)
{
    CAM_DBG("%.12f, %.12f\n", val->p_val_d[0], val->p_val_d[1]);
    self->val.p_val_d[0] = val->p_val_d[0];
    self->val.p_val_d[1] = val->p_val_d[1];
    return 0;
}

struct camera_ext_ctrl_item jpeg_gps_location = {
    .cfg = {
        .id = CAM_EXT_CID_JPEG_GPS_LOCATION,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_DOUBLE,
        .nr_of_elem = 2,
        .p_val_d = jpeg_gps_loc,
    },
    .set_ctrl = jpeg_gps_location_set_ctrl,
};

static float focal_length_items[] = {
    0.1f, 0.3f, 0.8f, 1.2f, 1.8f, 3.0f,
};

static int focal_length_set_ctrl(struct device *dev,
        struct camera_ext_ctrl_item *self, struct camera_ext_ctrl_val *val)
{
    CAM_DBG("%d\n", val->val);
    self->val.val = val->val;
    return 0;
}

struct camera_ext_ctrl_item focal_length = {
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
    .set_ctrl = focal_length_set_ctrl,
};

static float sensor_info_physical_size_val[] = {
    2345.13f, 4521.82f
};

static int sensor_info_physical_size_get_volatile_ctrl(struct device *dev,
            struct camera_ext_ctrl_item *self, struct camera_ext_ctrl_val *val)
{
    CAM_DBG("enter\n");
    val->elem_type = self->val.elem_type,
    val->nr_of_elem = self->val.nr_of_elem,
    val->p_val_f = self->val.p_val_f;

    return 0;
}

static struct camera_ext_ctrl_item sensor_info_physical_size = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_INFO_PHYSICAL_SIZE,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def_f = 0.2f, /* the def value will be [0.2f, 0.2f] */
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 2,
        .p_val_f = sensor_info_physical_size_val,
    },
    .get_volatile_ctrl = sensor_info_physical_size_get_volatile_ctrl,
};

static int lens_focus_distance_set_ctrl(struct device *dev,
            struct camera_ext_ctrl_item *self, struct camera_ext_ctrl_val *val)
{
    CAM_DBG("%.6f\n", val->val_f);
    self->val.val_f = val->val_f;
    return 0;
}

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
    .set_ctrl = lens_focus_distance_set_ctrl,
};

static float color_correction_transform_matrix[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1},
};

static int color_correction_transform_set_ctrl(struct device *dev,
            struct camera_ext_ctrl_item *self, struct camera_ext_ctrl_val *val)
{
    int i,j;
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++) {
            CAM_DBG("%.6f\n", val->p_val_f[i * 3 + j]);
            self->val.p_val_f[i * 3 + j] = val->p_val_f[i * 3 + j];
        }
    return 0;
}

static struct camera_ext_ctrl_item color_correction_transform = {
    .cfg = {
        .id = CAM_EXT_CID_COLOR_CORRECTION_TRANSFORM,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def_f = 1.0f,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 9,
        .p_val_f = &color_correction_transform_matrix[0][0],
    },
    .set_ctrl = color_correction_transform_set_ctrl,
};

static float lens_apertures_items[] = {
    12.1f, 23.2f, 34.5f, 45.8f, 56.3f, 67.0f, 78.1f,
};

static int lens_apertures_set_ctrl(struct device *dev,
            struct camera_ext_ctrl_item *self, struct camera_ext_ctrl_val *val)
{
    CAM_DBG("%d\n", val->val);
    self->val.val = val->val;
    return 0;
}

static struct camera_ext_ctrl_item lens_apertures = {
    .cfg = {
        .id = CAM_EXT_CID_LENS_APERTURES,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_FLOAT
                    | CAMERA_EXT_CTRL_FLAG_NEED_DEF
                    | CAMERA_EXT_CTRL_FLAG_NEED_MAX,
        .def = 2,
        .array_size = ARRAY_SIZE(lens_apertures_items),
        .max = ARRAY_SIZE(lens_apertures_items) - 1,
        .menu_float = lens_apertures_items,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = lens_apertures_set_ctrl,
};

static int sensor_exposure_time_set_ctrl(struct device *dev,
            struct camera_ext_ctrl_item *self, struct camera_ext_ctrl_val *val)
{
    CAM_DBG("%lld\n", val->val_64);
    self->val.val_64 = val->val_64;
    return 0;
}

static struct camera_ext_ctrl_item sensor_exposure_time = {
    .cfg = {
        .id = CAM_EXT_CID_SENSOR_EXPOSURE_TIME,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_MIN
                    | CAMERA_EXT_CTRL_FLAG_NEED_MAX
                    | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def = 232,
        .min = 100,
        .max = 1000,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT64,
        .nr_of_elem = 1,
    },
    .set_ctrl = sensor_exposure_time_set_ctrl,
};

/* points of tonemap curve */
#define TONEMAP_CURVE_CTRL_POINTS 18

static uint32_t tonemap_curve_dims[] = {2, 3, TONEMAP_CURVE_CTRL_POINTS};

/* in real impl, control value maybe stored somewhere else */
static float tonemap_curve_points[2 * 3 * TONEMAP_CURVE_CTRL_POINTS] = {
};

static int tonemap_curve_set_ctrl(struct device *dev,
            struct camera_ext_ctrl_item *self, struct camera_ext_ctrl_val *val)
{
    int i, j;
    float *src = val->p_val_f;
    float *tgt = self->val.p_val_f;

    CAM_DBG("enter\n");
    for (i = 0; i < TONEMAP_CURVE_CTRL_POINTS; i++) {
        CAM_DBG("(%.6f, %.6f) (%.6f, %.6f) (%.6f, %.6f)\n",
            src[0], src[1], src[2],
            src[3], src[4], src[5]);
        for (j = 0; j < 6; j++) *src++ = *tgt++;
    }
    return 0;
}

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
    .set_ctrl = tonemap_curve_set_ctrl,
};

static float lens_pos_rotation_value[7] = {
};

static int lens_pos_rotation_set_ctrl(struct device *dev,
            struct camera_ext_ctrl_item *self, struct camera_ext_ctrl_val *val)
{
    int i;

    CAM_DBG("enter\n");
    for (i = 0; i < 7; i++) {
        CAM_DBG("%.6f\n", val->p_val_f[i]);
        self->val.p_val_f[i] = val->p_val_f[i];
    }

    return 0;
}

static struct camera_ext_ctrl_item lens_pos_rotation = {
    .cfg = {
        .id = CAM_EXT_CID_LENS_POSE_ROTATION,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
        .def_f = 0.3f, /* TODO: support array default */
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_FLOAT,
        .nr_of_elem = 7,
        .p_val_f = lens_pos_rotation_value,
    },
    .set_ctrl = lens_pos_rotation_set_ctrl,
};

/* 12 hot pixels */
static uint32_t hot_pixel_map_dims[] = {2, 12};

static uint32_t hot_pixel_map_coord[12][2] = {
    {100,200}, {120,240},
    {108,250}, {150,210},
    {130,256}, {303,240},
    {162,290}, {130,260},
    {155,299}, {260,200},
    {160,255}, {407,230},
};

static int hot_pixel_map_get_voloatile_ctrl(struct device *dev,
        struct camera_ext_ctrl_item *self, struct camera_ext_ctrl_val *val)
{
    CAM_DBG("enter\n");
    val->elem_type = self->val.elem_type;
    val->nr_of_elem = self->val.nr_of_elem;
    val->p_val = self->val.p_val;

    return 0;
}

static struct camera_ext_ctrl_item hot_pixel_map = {
    .cfg = {
        .id = CAM_EXT_CID_HOT_PIXEL_MAP,
        .flags = CAMERA_EXT_CTRL_FLAG_NEED_DIMS,
        .array_size = ARRAY_SIZE(hot_pixel_map_dims),
        .dims = hot_pixel_map_dims,
    },
    .val = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 2 * 12,
        .p_val = &hot_pixel_map_coord[0][0],
    },
    .get_volatile_ctrl = hot_pixel_map_get_voloatile_ctrl,
};

const struct camera_ext_ctrl_item *testing_ctrls[] = {
    &color_correction_aberration_mode,
    &ae_exposure_compensation,
    &ae_target_fps_range,
    &awb_lock,
    &jpeg_gps_location,
    &focal_length,
    &sensor_info_physical_size,
    &lens_focus_distance,
    &color_correction_transform,
    &lens_apertures,
    &sensor_exposure_time,
    &tonemap_curve,
    &lens_pos_rotation,
    &hot_pixel_map,
};

int camera_ext_tesing_ctrl_init(struct device *dev)
{
    return register_camera_ext_ctrl_db(dev, testing_ctrls,
                ARRAY_SIZE(testing_ctrls));
}
