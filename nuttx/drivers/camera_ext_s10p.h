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

#define REG_JPEG_ORIEN_W      0x0000
#define REG_JPEG_QUALITY_W    0x0001
#define REG_TIME_SYNC_STEP    0x0008

#define REG_FLASH_MODE_W      0x1001
#define REG_AE_LOCK_W         0x1002
#define REG_ANTI_FLICK_W      0x1003
#define REG_FOCUS_MODE_W      0x1014
#define REG_CONT_FOCUS_W      0x1015
#define REG_FOCUS_RANGE_W     0x1017
#define REG_EV_COMP_W         0x1024
#define REG_ROI_REGION_W      0x102C

#define REG_ISO_W             0x2000
#define REG_WB_W              0x2001
#define REG_COLOR_W           0x2002
#define REG_SHARPNESS_W       0x2008
#define REG_CONTRAST_W        0x2009
#define REG_SATURATION_W      0x200A

#define REG_IMG_FORMAT_W      0x4000
#define REG_FACE_DET_W        0x4002
#define REG_SCENEMODE_W       0x4003
#define REG_CAPTURE_W         0x4008
#define REG_VIDEO_W           0x4009
#define REG_ND_FILTER_W       0x400E

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define JPEG_ORIENTATION_TOP_LEFT       1
#define JPEG_ORIENTATION_TOP_RIGHT      6
#define JPEG_ORIENTATION_BOTTOM_LEFT    8

#define FLASH_OFF    0
#define FLASH_AUTO   1
#define FLASH_ON     2

#define FOCUS_AUTO   0
#define FOCUS_MANUAL 2

#define FOCUS_RANGE_NORMAL   0x01
#define FOCUS_RANGE_MACRO    0x02
#define FOCUS_RANGE_HYPER_F  0x05
#define FOCUS_RANGE_INFINITY 0x0D

#define ISO_AUTO     0x00
#define ISO_100      0x04
#define ISO_200      0x09
#define ISO_400      0x0D
#define ISO_800      0x10
#define ISO_1600     0x13
#define ISO_3200     0x16

#define WB_AUTO              0x00
#define WB_CLOUDY_DAYLIGHT   0x01
#define WB_DAYLIGHT          0x02
#define WB_FLUORESCENT       0x03
#define WB_INCANDESCENT      0x04
#define WB_SHADE             0x05
#define WB_TWILIGHT          0x06

#define AQUA         0x00
#define BLACKBOARD   0x01
#define MONO         0x02
#define NEGATIVE     0x03
#define NONE         0x04
#define POSTERIZE    0x05
#define SEPIA        0x06
#define SOLARIZE     0x07
#define WHITEBOARD   0x08

#define SHARPNESS_2P     0x04
#define SHARPNESS_1P     0x02
#define SHARPNESS_0      0x00
#define SHARPNESS_1M     0x01
#define SHARPNESS_2M     0x03

#define CONTRAST_2P      0x05
#define CONTRAST_1P      0x02
#define CONTRAST_0       0x00
#define CONTRAST_1M      0x01
#define CONTRAST_2M      0x04

#define SATURATION_2P    0x04
#define SATURATION_1P    0x02
#define SATURATION_0     0x00
#define SATURATION_1M    0x01
#define SATURATION_2M    0x03

#define SCENE_ACTION          0x00
#define SCENE_AUTO            0x01
#define SCENE_BARCODE         0x02
#define SCENE_BEACH           0x03
#define SCENE_CANDLELIGHT     0x04
#define SCENE_FIREWORKS       0x05
#define SCENE_HDR             0x06
#define SCENE_LANDSCAPE       0x07
#define SCENE_NIGHT           0x08
#define SCENE_NIGHT_PORTRAIT  0x09
#define SCENE_PARTY           0x0A
#define SCENE_PORTRAIT        0x0B
#define SCENE_SNOW            0x0C
#define SCENE_SPORTS          0x0D
#define SCENE_STEADYPHOTO     0x0E
#define SCENE_SUNSET          0x0F
#define SCENE_THEATRE         0x10

#define IMG_JPG          0x00
#define IMG_RAW          0x01
#define IMG_ALL          0x02

#define ND_FILTER_AUTO   0x00
#define ND_FILTER_OFF    0x01
#define ND_FILTER_ON     0x02

struct s10p_i2c_dev_info {
    struct i2c_dev_s *i2c;
    uint16_t i2c_addr;
};

extern struct camera_ext_ctrl_db s10p_ctrl_db;

uint8_t s10p_read_reg1(struct s10p_i2c_dev_info* i2c, uint16_t regaddr);
uint16_t s10p_read_reg2(struct s10p_i2c_dev_info* i2c, uint16_t regaddr);
uint32_t s10p_read_reg4(struct s10p_i2c_dev_info* i2c, uint16_t regaddr);
int s10p_write_reg1(struct s10p_i2c_dev_info *i2c, uint16_t regaddr, uint8_t data);
int s10p_write_reg2(struct s10p_i2c_dev_info *i2c, uint16_t regaddr, uint16_t data);
int s10p_write_reg4(struct s10p_i2c_dev_info *i2c, uint16_t regaddr, uint32_t data);
void s10p_set_i2c(struct s10p_i2c_dev_info *i2c);

int s10p_report_button(uint8_t);
