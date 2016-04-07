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

#define S10_I2C_RETRY  5

static const uint8_t s10_iso[] = {
    ISO_AUTO,
    ISO_AUTO,
    ISO_100,
    ISO_200,
    ISO_400,
    ISO_800,
    ISO_1600,
    ISO_3200,
};

static const uint8_t s10_wb[] = {
    WB_AUTO,
    WB_AUTO,
    WB_INCANDESCENT,
    WB_FLUORESCENT,
    WB_DAYLIGHT,
    WB_CLOUDY_DAYLIGHT,
    WB_TWILIGHT,
    WB_SHADE,
};

static const uint8_t s10_effect[] = {
    NONE,
    MONO,
    NEGATIVE,
    SOLARIZE,
    SEPIA,
    POSTERIZE,
    WHITEBOARD,
    BLACKBOARD,
    AQUA,
};

static const uint8_t s10_sharpness[] = {
    SHARPNESS_2M,
    SHARPNESS_1M,
    SHARPNESS_0,
    SHARPNESS_1P,
    SHARPNESS_2P,
};

static const uint8_t s10_contrast[] = {
    CONTRAST_2M,
    CONTRAST_1M,
    CONTRAST_0,
    CONTRAST_1P,
    CONTRAST_2P,
};

static const uint8_t s10_saturation[] = {
    SATURATION_2M,
    SATURATION_1M,
    SATURATION_0,
    SATURATION_1P,
    SATURATION_2P,
};

static const uint8_t s10_scene[] = {
    SCENE_AUTO,
    SCENE_AUTO,
    SCENE_ACTION,
    SCENE_PORTRAIT,
    SCENE_LANDSCAPE,
    SCENE_NIGHT,
    SCENE_NIGHT_PORTRAIT,
    SCENE_THEATRE,
    SCENE_BEACH,
    SCENE_SNOW,
    SCENE_SUNSET,
    SCENE_STEADYPHOTO,
    SCENE_FIREWORKS,
    SCENE_SPORTS,
    SCENE_PARTY,
    SCENE_CANDLELIGHT,
    SCENE_BARCODE,
    SCENE_HDR,
};

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

    int ret = S10_I2C_RETRY;
    while (I2C_TRANSFER(i2c->i2c, &msg[0], 1) != 0)
    {
        usleep(1000 * 100);
        if (--ret < 0) break;
    }
    if (ret < 0) {
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

    int ret = S10_I2C_RETRY;
    while (I2C_TRANSFER(i2c->i2c, &msg, 1) != 0)
    {
        usleep(1000 * 100);
        if (--ret < 0) break;
    }
    if (ret < 0) {
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
    uint8_t value_8;
    int32_t value_s;
    size_t i;
    int retval = 0;

    is_array = self->val_cfg.nr_of_elem > 1;
    switch (self->id) {
        case CAM_EXT_CID_AE_ANTIBANDING_MODE:
            break;
        case CAM_EXT_CID_AE_EXPOSURE_COMPENSATION:
            break;
        case CAM_EXT_CID_AE_LOCK:
            break;
        case CAM_EXT_CID_AE_MODE:
            value_8 = FLASH_AUTO;
            if ((val->val & 0xFF) == CAM_EXT_AE_MODE_ON)
            {
                value_8 = FLASH_OFF;
            }
            else if ((val->val & 0xFF) == CAM_EXT_AE_MODE_ON_ALWAYS_FLASH)
            {
                value_8 = FLASH_ON;
            }
            s10p_write_reg1(i2c_info, REG_FLASH_MODE_W, value_8);
            break;
        case CAM_EXT_CID_AWB_MODE:
            value_8 = val->val & 0xFF;
            s10p_write_reg1(i2c_info, REG_WB_W, s10_wb[value_8]);
            break;
        case CAM_EXT_CID_EFFECT_MODE:
            value_8 = val->val & 0xFF;
            if (value_8 >= sizeof(s10_effect))
                value_8 = CAM_EXT_EFFECT_MODE_OFF;
            s10p_write_reg1(i2c_info, REG_COLOR_W, s10_effect[value_8]);
            break;
        case CAM_EXT_CID_CONTROL_MODE:
            break;
        case CAM_EXT_CID_SCENE_MODE:
            value_8 = val->val & 0xFF;
            s10p_write_reg1(i2c_info, REG_SCENEMODE_W, s10_scene[value_8]);
            break;
        case CAM_EXT_CID_VIDEO_STABILIZATION_MODE:
            break;
        case CAM_EXT_CID_JPEG_GPS_LOCATION:
            break;
        case CAM_EXT_CID_JPEG_ORIENTATION:
            value_8 =  JPEG_ORIENTATION_TOP_LEFT;
            if ((val->val & 0x03) == 0)
            {
                value_8 = JPEG_ORIENTATION_TOP_RIGHT;
            }
            else if ((val->val & 0x03) == 2)
            {
                value_8 = JPEG_ORIENTATION_BOTTOM_LEFT;
            }
            s10p_write_reg1(i2c_info, REG_JPEG_ORIEN_W, value_8);
            break;
        case CAM_EXT_CID_JPEG_QUALITY:
            value_8 = val->val & 0xFF;
            if (value_8 > 100) value_8 = 100;
            s10p_write_reg1(i2c_info, REG_JPEG_QUALITY_W, value_8);
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
        case CAM_EXT_CID_ISO:
            value_8 = val->val & 0xFF;
            if (value_8 >= sizeof(s10_iso))
                value_8 = CAM_EXT_ISO_AUTO;
            s10p_write_reg1(i2c_info, REG_ISO_W, s10_iso[value_8]);
            break;
        case CAM_EXT_CID_ND_FILTER:
            s10p_write_reg1(i2c_info, REG_ND_FILTER_W, val->val & 0x03);
            break;
        case CAM_EXT_CID_JPEG_SHARPNESS:
            value_s = (int32_t)val->val;
            value_s += sizeof(s10_sharpness)/2;
            if ((value_s < 0) || value_s > (sizeof(s10_sharpness) -1))
                value_s = sizeof(s10_sharpness)/2;
            s10p_write_reg1(i2c_info, REG_SHARPNESS_W, s10_sharpness[value_s]);
            break;
        case CAM_EXT_CID_JPEG_CONTRAST:
            value_s = (int32_t)val->val;
            value_s += sizeof(s10_contrast)/2;
            if ((value_s < 0) || value_s > (sizeof(s10_contrast) -1))
                value_s = sizeof(s10_contrast)/2;
            s10p_write_reg1(i2c_info, REG_CONTRAST_W, s10_contrast[value_s]);
            break;
        case CAM_EXT_CID_JPEG_SATURATION:
            value_s = (int32_t)val->val;
            value_s += sizeof(s10_saturation)/2;
            if ((value_s < 0) || value_s > (sizeof(s10_saturation) -1))
                value_s = sizeof(s10_saturation)/2;
            s10p_write_reg1(i2c_info, REG_SATURATION_W, s10_saturation[value_s]);
            break;
        case CAM_EXT_CID_FACE_DETECTION:
            value_8 = 0;
            if (val->val != CAM_EXT_STATISTICS_FACE_DETECT_MODE_OFF)
                value_8 = 1;
            s10p_write_reg1(i2c_info, REG_FACE_DET_W, value_8);
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

static int ctrl_val_set_focus(struct device *dev,
        const struct camera_ext_ctrl_cfg *self,
        const camera_ext_ctrl_val_t *val)
{
    uint8_t value;
    uint8_t value_8 = val->val & 0xFF;
    if (self->id == CAM_EXT_CID_AF_MODE_EXT)
    {
        if (value_8 == CAM_EXT_AF_MODE_EXT_NULL)
            return 0;
        value_8 += CAM_EXT_AF_MODE_MAX;
    }

    if (value_8 == CAM_EXT_AF_MODE_OFF)
    {
        s10p_write_reg1(i2c_info, REG_FOCUS_MODE_W, FOCUS_MANUAL);
    }
    else
    {
        s10p_write_reg1(i2c_info, REG_FOCUS_MODE_W, FOCUS_AUTO);
        if ((value_8 == CAM_EXT_AF_MODE_CONTINUOUS_VIDEO) ||
            (value_8 == CAM_EXT_AF_MODE_CONTINUOUS_PICTURE))
		{
            s10p_write_reg1(i2c_info, REG_CONT_FOCUS_W, 1);
        }
        else
        {
            s10p_write_reg1(i2c_info, REG_CONT_FOCUS_W, 0);
            value = FOCUS_RANGE_NORMAL;
            if (value_8 == CAM_EXT_AF_MODE_MACRO) value = FOCUS_RANGE_MACRO;
            if (value_8 == CAM_EXT_AF_MODE_MAX + CAM_EXT_AF_MODE_EXT_INFINITY)
                value = FOCUS_RANGE_INFINITY;
            if (value_8 == CAM_EXT_AF_MODE_MAX + CAM_EXT_AF_MODE_EXT_FIXED)
                value = FOCUS_RANGE_HYPER_F;
            s10p_write_reg1(i2c_info, REG_FOCUS_RANGE_W, value);
        }
    }

    return 0;
}

static int ctrl_val_set_capture(struct device *dev,
        const struct camera_ext_ctrl_cfg *self,
        const camera_ext_ctrl_val_t *val)
{
    static uint32_t video_recording = 0;
    uint32_t capture;
    uint8_t value;

    capture = val->val;
    if ((capture & CAM_EXT_CID_CAPTURE_VIDEO_RECORD) != video_recording)
    {
        value = 0;
        if (video_recording == 0)
        {
            video_recording = CAM_EXT_CID_CAPTURE_VIDEO_RECORD;
            value = 1;
        }
        else
        {
            video_recording = 0;
        }
        s10p_write_reg1(i2c_info, REG_VIDEO_W, value);
    }

    if (((capture & CAM_EXT_CID_CAPTURE_STILL_CAPTURE)  |
	     (capture & CAM_EXT_CID_CAPTURE_VIDEO_SNAPSHOT) |
		 (capture & CAM_EXT_CID_CAPTURE_ZSL_CAPTURE)) != 0)
    {
        value = IMG_JPG;
        if ((capture & CAM_EXT_CID_CAPTURE_RAW) != 0)
        {
            value = IMG_RAW;
            if ((capture & CAM_EXT_CID_CAPTURE_JPG) != 0)
                value = IMG_ALL;
        }
        s10p_write_reg1(i2c_info, REG_IMG_FORMAT_W, value);
        s10p_write_reg1(i2c_info, REG_CAPTURE_W, 1);
    }

    CAM_DBG("Capture Image %x\n", val->val);
	return 0;
}

static int ctrl_val_set_time_sync(struct device *dev,
        const struct camera_ext_ctrl_cfg *self,
        const camera_ext_ctrl_val_t *val)
{
    uint8_t value[8];
    time_t time_in_sec;
    FAR struct tm *time;

    time_in_sec = (time_t)(val->val_64 / 1000000);
    time = gmtime(&time_in_sec);

    value[0] = REG_TIME_SYNC_STEP & 0xFF;
    value[1] = (REG_TIME_SYNC_STEP >> 8) & 0xFF;
    value[2] = (uint8_t)(time->tm_year - 80);
    value[3] = (uint8_t)time->tm_mon;
    value[4] = (uint8_t)time->tm_mday;
    value[5] = (uint8_t)time->tm_hour;
    value[6] = (uint8_t)time->tm_min;
    value[7] = (uint8_t)time->tm_sec;

    i2c_write(i2c_info, value, sizeof(value));

    return 0;
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
    .def = {
        .val = CAM_EXT_AE_MODE_ON_AUTO_FLASH,
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
    .set_ctrl = ctrl_val_set_focus,
};

static const struct camera_ext_ctrl_cfg awb_mode = {
    .id = CAM_EXT_CID_AWB_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .menu_skip_mask = 0x01,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
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
    .min = 0,
    .menu_skip_mask = 0x02,
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
    .max = 3,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .def = {
        .val = 1,
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

static const struct camera_ext_ctrl_cfg flash_mode = {
    .id = CAM_EXT_CID_FLASH_MODE,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
        | CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
    .menu_skip_mask = 0x2,
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
        .val = 8,
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

static const struct camera_ext_ctrl_cfg capture = {
    .id = CAM_EXT_CID_CAPTURE,
    .flags = 0,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set_capture,
};

static const struct camera_ext_ctrl_cfg af_mode_ext = {
    .id = CAM_EXT_CID_AF_MODE_EXT,
    .flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
        | CAMERA_EXT_CTRL_FLAG_NEED_DEF,
    .val_cfg = {
        .elem_type = CAM_EXT_CTRL_DATA_TYPE_INT,
        .nr_of_elem = 1,
    },
    .set_ctrl = ctrl_val_set_focus,
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
    .set_ctrl = ctrl_val_set_time_sync,
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

/* has_uvc, vid, pid */
static const unsigned int uvc_cfg[] = {
    1, 0x143C, 0x7722
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
        .val = CAM_EXT_CID_MOD_META_DATA_PATH_NONE,
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
        .val = 0, /* 0 line */
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
    &control_mode,
    &scene_mode,
    &video_stabilization,
    &jpeg_gps_location,
    &jpeg_orientation,
    &jpeg_quality,
    &flash_mode,
    &focal_length,
    &statistics_face_detect_mode,
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
};

struct camera_ext_ctrl_db s10p_ctrl_db = {
    .num_ctrls = ARRAY_SIZE(_ctrls),
    .ctrls = _ctrls,
};
