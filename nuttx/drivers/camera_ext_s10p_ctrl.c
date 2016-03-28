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

#include <apps/ice/cdsi.h>
#include <nuttx/config.h>
#include <nuttx/device_cam_ext.h>
#include <nuttx/gpio.h>
#include <nuttx/i2c.h>
#include <nuttx/math.h>

#include "greybus/v4l2_camera_ext_ctrls.h"

#include "camera_ext.h"

#include "camera_ext_s10p.h"

#ifdef CONFIG_NSH_CONSOLE
#define CTRL_DBG printf
#else
#define CTRL_DBG
#endif

#define REG_JPEG_ORIEN_W      0x0000
#define REG_JPEG_QUALITY_W    0x0001
#define REG_JPEG_ORIEN_R      0x0004
#define REG_JPEG_QUALITY_R    0x0005
#define REG_TIME_SYNC_STEP1   0x0008
#define REG_TIME_SYNC_STEP2   0x0010

#define REG_FLASH_MODE_W      0x1001
#define REG_AE_LOCK_W         0x1002
#define REG_ANTI_FLICK_W      0x1003
#define REG_FLASH_MODE_R      0x1005
#define REG_AE_LOCK_R         0x1006
#define REG_ANTI_FLICK_R      0x1007
#define REG_AE_REGION_STEP1   0x1008
#define REG_AE_REGION_STEP2   0x1010
#define REG_FOCUS_MODE_W      0x1014
#define REG_AF_LOCK_W         0x1016
#define REG_FOCUS_MODE_R      0x1018
#define REG_AF_LOCK_R         0x101A
#define REG_AF_REGION_STEP1   0x101C
#define REG_AF_REGION_STEP2   0x1020
#define REG_EV_COMP_W         0x1024
#define REG_EV_COM__R         0x1028

#define REG_ISO_W             0x2000
#define REG_WB_W              0x2001
#define REG_COLOR_W           0x2002
#define REG_ISO_R             0x2004
#define REG_WB_R              0x2005
#define REG_COLOR_R           0x2006
#define REG_SHARPNESS_W       0x2008
#define REG_CONTRAST_W        0x2009
#define REG_SATURATION_W      0x200A
#define REG_SHARPNESS_R       0x200C
#define REG_CONTRAST_R        0x200D
#define REG_SATURATION_R      0x200E

#define REG_CAPTURE_W         0x4008
#define REG_VIDEO_W           0x4009

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

static struct s10p_i2c_dev_info *i2c_info;

static int i2c_read(struct s10p_i2c_dev_info *i2c,
                    uint8_t *addr, int addr_len,
                    uint8_t *data, int data_len)
{
    struct i2c_msg_s msg[2];

    msg[0].addr   = i2c->i2c_addr;
    msg[0].flags  = 0;
    msg[0].buffer = addr;
    msg[0].length = addr_len;

    msg[1].addr   = i2c->i2c_addr;
    msg[1].flags  = I2C_M_READ;
    msg[1].buffer = data;
    msg[1].length = data_len;

    int ret = I2C_TRANSFER(i2c->i2c, &msg[0], 1);
    if (ret != 0) {
        CAM_ERR("i2c read step 1 transfer failed %d\n", ret);
        return -1;
    }
    ret = I2C_TRANSFER(i2c->i2c, &msg[1], 1);
    if (ret != 0) {
        CAM_ERR("i2c read step 2 transfer failed %d\n", ret);
        return -1;
    }

    return 0;
}

/* 1 bytes value register read */
uint8_t s10p_read_reg1(struct s10p_i2c_dev_info* i2c, uint16_t regaddr)
{
    uint8_t addr[2];
    uint8_t data[1];
    uint8_t value = 0;

    addr[0] = regaddr & 0xFF;
    addr[1] = (regaddr >> 8) & 0xFF;

    memset(data, 0, sizeof(data));

    if (i2c_read(i2c, addr, sizeof(addr), data, sizeof(data)) == 0) {
        value = data[0];
        CAM_DBG("read: 0x%04x -> 0x%02x\n", regaddr, value);
    }

    return value;
}

/* 2 bytes value register read */
uint16_t s10p_read_reg2(struct s10p_i2c_dev_info* i2c, uint16_t regaddr)
{
    uint8_t addr[2];
    uint8_t data[2];
    uint16_t value = 0;

    addr[0] = regaddr & 0xFF;
    addr[1] = (regaddr >> 8) & 0xFF;

    memset(data, 0, sizeof(data));

    if (i2c_read(i2c, addr, sizeof(addr), data, sizeof(data)) == 0) {
        value = (data[1] << 8) + data[0];
        CAM_DBG("read: 0x%04x -> 0x%04x\n", regaddr, value);
    }

    return value;
}

/* 4 bytes value register read */
uint32_t s10p_read_reg4(struct s10p_i2c_dev_info* i2c, uint16_t regaddr)
{
    uint8_t addr[2];
    uint8_t data[4];
    uint32_t value = 0;

    addr[0] = regaddr & 0xFF;
    addr[1] = (regaddr >> 8) & 0xFF;

    memset(data, 0, sizeof(data));

    if (i2c_read(i2c, addr, sizeof(addr), data, sizeof(data)) == 0) {
        value = (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + data[0];
        CAM_DBG("read: 0x%04x -> 0x%08x\n", regaddr, value);
    }

    return value;
}

static int i2c_write(struct s10p_i2c_dev_info *i2c, uint8_t *addr, int addr_len)
{
    struct i2c_msg_s msg;

    msg.addr   = i2c->i2c_addr;
    msg.flags  = 0;
    msg.buffer = addr;
    msg.length = addr_len;

    int ret = I2C_TRANSFER(i2c->i2c, &msg, 1);
    if (ret != 0) {
        CAM_ERR("i2c write transfer failed %d\n", ret);
        return -1;
    }

    return 0;
}

/* 1 bytes value register write */
int s10p_write_reg1(struct s10p_i2c_dev_info *i2c, uint16_t regaddr, uint8_t data)
{
    uint8_t addr[3];

    CAM_DBG("write 0x%02x to addr 0x%04x\n", data, regaddr);
    addr[0] = regaddr & 0xFF;
    addr[1] = (regaddr >> 8) & 0xFF;
    addr[2] = data & 0xFF;

    return i2c_write(i2c, addr, sizeof(addr));
}

/* 2 bytes value register write */
int s10p_write_reg2(struct s10p_i2c_dev_info *i2c, uint16_t regaddr, uint16_t data)
{
    uint8_t addr[4];

    CAM_DBG("write 0x%04x to addr 0x%04x\n", data, regaddr);
    addr[0] = regaddr & 0xFF;
    addr[1] = (regaddr >> 8) & 0xFF;
    addr[2] = data & 0xFF;
    addr[3] = (data >> 8) & 0xFF;

    return i2c_write(i2c, addr, sizeof(addr));
}

/* 4 bytes value register write */
int s10p_write_reg4(struct s10p_i2c_dev_info *i2c, uint16_t regaddr, uint32_t data)
{
    uint8_t addr[6];

    CAM_DBG("write 0x%08x to addr 0x%04x\n", data, regaddr);
    addr[0] = regaddr & 0xFF;
    addr[1] = (regaddr >> 8) & 0xFF;
    addr[2] = data & 0xFF;
    addr[3] = (data >> 8) & 0xFF;
    addr[4] = (data >> 16) & 0xFF;
    addr[5] = (data >> 24) & 0xFF;

    return i2c_write(i2c, addr, sizeof(addr));
}

void s10p_set_i2c(struct s10p_i2c_dev_info *i2c)
{
    i2c_info = i2c;
}

static int ctrl_val_set(struct device *dev,
        const struct camera_ext_ctrl_cfg *self,
        const camera_ext_ctrl_val_t *val)
{
    int is_array;
    size_t i;
    int retval = 0;

    is_array = self->val_cfg.nr_of_elem > 1;
    switch (self->id) {
        case CAM_EXT_CID_AE_ANTIBANDING_MODE:
            s10p_write_reg1(i2c_info, REG_ANTI_FLICK_W, 0x00);  //mbing
            break;
        case CAM_EXT_CID_AE_EXPOSURE_COMPENSATION:
            s10p_write_reg2(i2c_info, REG_EV_COMP_W, val->val & 0xFFFF);
            break;
        case CAM_EXT_CID_AE_LOCK:
            s10p_write_reg1(i2c_info, REG_AE_LOCK_W, 0x00);   //mbing
            break;
        case CAM_EXT_CID_AE_MODE:
            break;
        case CAM_EXT_CID_AF_MODE:
            break;
        case CAM_EXT_CID_AWB_MODE:
            break;
        case CAM_EXT_CID_EFFECT_MODE:
            break;
        case CAM_EXT_CID_CONTROL_MODE:
            break;
        case CAM_EXT_CID_SCENE_MODE:
            break;
        case CAM_EXT_CID_VIDEO_STABILIZATION_MODE:
            break;
        case CAM_EXT_CID_JPEG_GPS_LOCATION:
            break;
        case CAM_EXT_CID_JPEG_ORIENTATION:
            s10p_write_reg1(i2c_info, REG_JPEG_ORIEN_W, val->val & 0xFF);
            CAM_DBG("wrote jpeg orientation %d\n", val->val);
            break;
        case CAM_EXT_CID_JPEG_QUALITY:
            break;
        case CAM_EXT_CID_STATISTICS_FACE_DETECT_MODE:
            break;
        case CAM_EXT_CID_STATISTICS_INFO_MAX_FACE_COUNT:
            break;
        case CAM_EXT_CID_SENSOR_EXPOSURE_TIME:
            break;
        case CAM_EXT_CID_AE_REGIONS:
            break;
        case CAM_EXT_CID_AF_REGIONS:
            break;
        case CAM_EXT_CID_START_CAPTURE:
            if (val->val != 0)
                s10p_write_reg1(i2c_info, REG_CAPTURE_W, val->val & 0xFF);
            CAM_DBG("Capture Image %x\n", val->val);
            break;
        case CAM_EXT_CID_ABORT_CAPTURE:
            break;
        case CAM_EXT_CID_ISO:
            break;
        case CAM_EXT_CID_ND_FILTER:
            break;
        case CAM_EXT_CID_JPEG_SHARPNESS:
            break;
        case CAM_EXT_CID_JPEG_CONTRAST:
            break;
        case CAM_EXT_CID_JPEG_SATURATION:
            break;
        case CAM_EXT_CID_TIME_SYNC:
            break;
        case CAM_EXT_CID_JPEG_GPS_TIMESTAMP:
            break;
        default:
            break;
    }

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
        case CAM_EXT_CTRL_DATA_TYPE_STRING:
            if (!is_array) {
                CAM_DBG("%s ", val->p_val_8);
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
    .step = 333,
    .min = -3000,
    .max = 3000,
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
        .nr_of_elem = 3,
    },
    .set_ctrl = ctrl_val_set,
};

static const struct camera_ext_ctrl_cfg jpeg_orientation = {
    .id = CAM_EXT_CID_JPEG_ORIENTATION,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .min = 0,
    .max = 8,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
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

static const struct camera_ext_ctrl_cfg *_ctrls[] = {
    &ae_antibanding_mode,
    &ae_exposure_compensation,
    &ae_lock,
    &ae_mode,
    &af_mode,
    &ae_target_fps_range,
    &awb_mode,
    &effect_mode,
    &control_mode,
    &scene_mode,
    &video_stabilization,
    &jpeg_gps_location,
    &jpeg_orientation,
    &jpeg_quality,
    &focal_length,
    &statistics_face_detect_mode,
    &statistics_info_max_face_count,
    &sensor_exposure_time,
    &ae_regions,
    &af_regions,
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

struct camera_ext_ctrl_db s10p_ctrl_db = {
    .num_ctrls = ARRAY_SIZE(_ctrls),
    .ctrls = _ctrls,
};
