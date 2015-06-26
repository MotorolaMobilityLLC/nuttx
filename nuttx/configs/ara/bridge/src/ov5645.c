/*
 * Copyright (c) 2015 Google Inc.
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <debug.h>
#include <stdint.h>
#include <errno.h>
#include <nuttx/gpio.h>
#include <nuttx/i2c.h>
#include <nuttx/util.h>
#include <arch/tsb/gpio.h>
#include <arch/board/ov5645.h>

#define OV5645_RESET        3
#define OV5645_PWDN         4

#define OV5645_APB3_I2C0    0
#define OV5645_I2C_ADDR     0x3c

struct camera_param {
    uint16_t addr;
    uint16_t data;
};

/* YCbCr initial setting */
static struct camera_param ov5645_default_regs_init[] = {
    /* initial setting, Sysclk = 56Mhz, MIPI 2 lane 224MBps */
    {0x3103, 0x11}, /* select PLL input clock */
    {0x3008, 0x82}, /* software reset */
    {0x3008, 0x42}, /* software standby */
    {0x3103, 0x03}, /* clo0xfrom, 0xpl,L */
    {0x3503, 0x07}, /* AGC manual, AEC manual */
    {0x3002, 0x1c}, /* system reset */
    {0x3006, 0xc3}, /* clock enable */
    {0x300e, 0x45}, /* MIPI 2 lane */
    {0x3017, 0x40}, /* Frex, CSK input, Vsync output */
    {0x3018, 0x00}, /* GPIO input */
    {0x302e, 0x0b},
    {0x3037, 0x13}, /* PLL */
    {0x3108, 0x01}, /* PLL */
    {0x3611, 0x06},
    {0x3612, 0xab},
    {0x3614, 0x50},
    {0x3618, 0x04},
    {0x3034, 0x18}, /* PLL, MIPI 8-bit mode */
    {0x3035, 0x21}, /* PLL */
    {0x3036, 0x70}, /* PLL */
    {0x3500, 0x00}, /* exposure = 0x100 */
    {0x3501, 0x01}, /* exposure */
    {0x3502, 0x00}, /* exposure */
    {0x350a, 0x00}, /* gain = 0x3f */
    {0x350b, 0x3f}, /* gain */
    {0x3600, 0x09},
    {0x3601, 0x43},
    {0x3620, 0x33},
    {0x3621, 0xe0},
    {0x3622, 0x01},
    {0x3630, 0x2d},
    {0x3631, 0x00},
    {0x3632, 0x32},
    {0x3633, 0x52},
    {0x3634, 0x70},
    {0x3635, 0x13},
    {0x3636, 0x03},
    {0x3702, 0x6e},
    {0x3703, 0x52},
    {0x3704, 0xa0},
    {0x3705, 0x33},
    {0x3708, 0x66},
    {0x3709, 0x12},
    {0x370b, 0x61},
    {0x370c, 0xc3},
    {0x370f, 0x10},
    {0x3715, 0x08},
    {0x3717, 0x01},
    {0x371b, 0x20},
    {0x3731, 0x22},
    {0x3739, 0x70},
    {0x3901, 0x0a},
    {0x3905, 0x02},
    {0x3906, 0x10},
    {0x3719, 0x86},
    {0x3800, 0x00}, /* HS = 0 */
    {0x3801, 0x00}, /* HS */
    {0x3802, 0x00}, /* VS = 6 */
    {0x3803, 0x06}, /* VS */
    {0x3804, 0x0a}, /* HW = 2623 */
    {0x3805, 0x3f}, /* HW */
    {0x3806, 0x07}, /* VH = 1949 */
    {0x3807, 0x9d}, /* VH */
    {0x3808, 0x05}, /* DVPHO = 1280 */
    {0x3809, 0x00}, /* DVPHO */
    {0x380a, 0x03}, /* DVPVO = 960 */
    {0x380b, 0xc0}, /* DVPVO */
    {0x380c, 0x07}, /* HTS = 1896 */
    {0x380d, 0x68}, /* HTS */
    {0x380e, 0x03}, /* VTS = 984 */
    {0x380f, 0xd8}, /* VTS */
    {0x3810, 0x00}, /* H OFF = 16 */
    {0x3811, 0x10}, /* H OFF */
    {0x3812, 0x00}, /* V OFF = 6 */
    {0x3813, 0x06}, /* V OFF */
    {0x3814, 0x31}, /* X INC */
    {0x3815, 0x31}, /* Y INC */
    {0x3820, 0x47}, /* flip on, V bin on */
    {0x3821, 0x07}, /* mirror on, H bin on */
    {0x3824, 0x01}, /* PLL */
    {0x3826, 0x03},
    {0x3828, 0x08},
    {0x3a02, 0x03}, /* nigt mode ceiling = 984 */
    {0x3a03, 0xd8}, /* nigt mode ceiling */
    {0x3a08, 0x01}, /* B50 */
    {0x3a09, 0xf8}, /* B50 */
    {0x3a0a, 0x01}, /* B60 */
    {0x3a0b, 0xa4}, /* B60 */
    {0x3a0e, 0x02}, /* max 50 */
    {0x3a0d, 0x02}, /* max 60 */
    {0x3a14, 0x03}, /* 50Hz max exposure = 984 */
    {0x3a15, 0xd8}, /* 50Hz max exposure */
    {0x3a18, 0x01}, /* gain ceiling = 31.5x */
    {0x3a19, 0xf8}, /* gain ceiling */
    /* 50Hz/60Hz auto detect */
    {0x3c01, 0x34},
    {0x3c04, 0x28},
    {0x3c05, 0x98},
    {0x3c07, 0x07},
    {0x3c09, 0xc2},
    {0x3c0a, 0x9c},
    {0x3c0b, 0x40},
    {0x3c01, 0x34},
    {0x4001, 0x02}, /* BLC start line */
    {0x4004, 0x02}, /* B0xline, 0xnu,mber */
    {0x4005, 0x18}, /* BLC update by gain change */
    {0x4300, 0x30}, /* YUV 422, YUYV */
    {0x4514, 0x00},
    {0x4520, 0xb0},
    {0x460b, 0x37},
    {0x460c, 0x20},
    /* MIPI timing */
    {0x4818, 0x01},
    {0x481d, 0xf0},
    {0x481f, 0x50},
    {0x4823, 0x70},
    {0x4831, 0x14},
    {0x4837, 0x10}, /* global timing */
    {0x5000, 0xa7}, /* Lenc/raw gamma/BPC/WPC/color interpolation on */
    {0x5001, 0x83}, /* SDE on, scale off, UV adjust off, color matrix/AWB on */
    {0x501d, 0x00},
    {0x501f, 0x00}, /* select ISP YUV 422 */
    {0x503d, 0x00},
    {0x505c, 0x30},
    /* AWB control */
    {0x5181, 0x59},
    {0x5183, 0x00},
    {0x5191, 0xf0},
    {0x5192, 0x03},
    /* AVG control */
    {0x5684, 0x10},
    {0x5685, 0xa0},
    {0x5686, 0x0c},
    {0x5687, 0x78},
    {0x5a00, 0x08},
    {0x5a21, 0x00},
    {0x5a24, 0x00},
    {0x3008, 0x02}, /* wake 0xfrom, 0xso,ftware standby */
    {0x3503, 0x00}, /* AGC auto, AEC auto */
    /* AWB control */
    {0x5180, 0xff},
    {0x5181, 0xf2},
    {0x5182, 0x00},
    {0x5183, 0x14},
    {0x5184, 0x25},
    {0x5185, 0x24},
    {0x5186, 0x09},
    {0x5187, 0x09},
    {0x5188, 0x0a},
    {0x5189, 0x75},
    {0x518a, 0x52},
    {0x518b, 0xea},
    {0x518c, 0xa8},
    {0x518d, 0x42},
    {0x518e, 0x38},
    {0x518f, 0x56},
    {0x5190, 0x42},
    {0x5191, 0xf8},
    {0x5192, 0x04},
    {0x5193, 0x70},
    {0x5194, 0xf0},
    {0x5195, 0xf0},
    {0x5196, 0x03},
    {0x5197, 0x01},
    {0x5198, 0x04},
    {0x5199, 0x12},
    {0x519a, 0x04},
    {0x519b, 0x00},
    {0x519c, 0x06},
    {0x519d, 0x82},
    {0x519e, 0x38},
    /* matrix */
    {0x5381, 0x1e},
    {0x5382, 0x5b},
    {0x5383, 0x08},
    {0x5384, 0x0b},
    {0x5385, 0x84},
    {0x5386, 0x8f},
    {0x5387, 0x82},
    {0x5388, 0x71},
    {0x5389, 0x11},
    {0x538a, 0x01},
    {0x538b, 0x98},
    /* CIP */
    {0x5300, 0x08}, /* sharpen MT th1 */
    {0x5301, 0x30}, /* sharpen MT th2 */
    {0x5302, 0x10}, /* sharpen MT off1 */
    {0x5303, 0x00}, /* sharpen MT off2 */
    {0x5304, 0x08}, /* DNS th1 */
    {0x5305, 0x30}, /* DNS th2 */
    {0x5306, 0x08}, /* DNS off1 */
    {0x5307, 0x16}, /* DNS off2 */
    {0x5309, 0x08}, /* sharpen TH th1 */
    {0x530a, 0x30}, /* sharpen TH th2 */
    {0x530b, 0x04}, /* sharpen TH off1 */
    {0x530c, 0x06}, /* sharpen TH off2 */
    /* Gamma */
    {0x5480, 0x01}, /* bias on */
    {0x5481, 0x0e}, /* Y yst 00 */
    {0x5482, 0x18},
    {0x5483, 0x2b},
    {0x5484, 0x52},
    {0x5485, 0x65},
    {0x5486, 0x71},
    {0x5487, 0x7d},
    {0x5488, 0x87},
    {0x5489, 0x91},
    {0x548a, 0x9a},
    {0x548b, 0xaa},
    {0x548c, 0xb8},
    {0x548d, 0xcd},
    {0x548e, 0xdd},
    {0x548f, 0xea}, /* Y yst 0E */
    {0x5490, 0x1d}, /* Y yst 0F */
    /* SDE */
    {0x5580, 0x06},
    {0x5583, 0x40},
    {0x5584, 0x30},
    {0x5589, 0x10},
    {0x558a, 0x00},
    {0x558b, 0xf8},
    /* LENC */
    {0x5800, 0x3f},
    {0x5801, 0x16},
    {0x5802, 0x0e},
    {0x5803, 0x0d},
    {0x5804, 0x17},
    {0x5805, 0x3f},
    {0x5806, 0x0b},
    {0x5807, 0x06},
    {0x5808, 0x04},
    {0x5809, 0x04},
    {0x580a, 0x06},
    {0x580b, 0x0b},
    {0x580c, 0x09},
    {0x580d, 0x03},
    {0x580e, 0x00},
    {0x580f, 0x00},
    {0x5810, 0x03},
    {0x5811, 0x08},
    {0x5812, 0x0a},
    {0x5813, 0x03},
    {0x5814, 0x00},
    {0x5815, 0x00},
    {0x5816, 0x04},
    {0x5817, 0x09},
    {0x5818, 0x0f},
    {0x5819, 0x08},
    {0x581a, 0x06},
    {0x581b, 0x06},
    {0x581c, 0x08},
    {0x581d, 0x0c},
    {0x581e, 0x3f},
    {0x581f, 0x1e},
    {0x5820, 0x12},
    {0x5821, 0x13},
    {0x5822, 0x21},
    {0x5823, 0x3f},
    {0x5824, 0x68},
    {0x5825, 0x28},
    {0x5826, 0x2c},
    {0x5827, 0x28},
    {0x5828, 0x08},
    {0x5829, 0x48},
    {0x582a, 0x64},
    {0x582b, 0x62},
    {0x582c, 0x64},
    {0x582d, 0x28},
    {0x582e, 0x46},
    {0x582f, 0x62},
    {0x5830, 0x60},
    {0x5831, 0x62},
    {0x5832, 0x26},
    {0x5833, 0x48},
    {0x5834, 0x66},
    {0x5835, 0x44},
    {0x5836, 0x64},
    {0x5837, 0x28},
    {0x5838, 0x66},
    {0x5839, 0x48},
    {0x583a, 0x2c},
    {0x583b, 0x28},
    {0x583c, 0x26},
    {0x583d, 0xae},
    {0x5025, 0x00},
    {0x3a0f, 0x38}, /* AEC in H */
    {0x3a10, 0x30}, /* AEC in L */
    {0x3a1b, 0x38}, /* AEC out H */
    {0x3a1e, 0x30}, /* AEC out L */
    {0x3a11, 0x70}, /* control zone H */
    {0x3a1f, 0x18}, /* control zone L */
    {0x3008, 0x02}, /* software enable */
};

/**
 * @brief i2c read for camera sensor
 * @param dev dev pointer to structure of i2c device data
 * @param addr address of i2c to read
 * @param data pointer of data to read in (It reads a single byte)
 * @return zero for success or non-zero on any faillure
 */
int camera_i2c_read(struct i2c_dev_s *dev, uint16_t addr, uint8_t *data)
{
    uint8_t cmd[2] = {0x00, 0x00};
    uint8_t buf = 0x00;
    int ret = 0;
    struct i2c_msg_s msg[] = {
        {
            .addr = OV5645_I2C_ADDR,
            .flags = 0,
            .buffer = cmd,
            .length = 2,
        }, {
            .addr = OV5645_I2C_ADDR,
            .flags = I2C_M_READ,
            .buffer = &buf,
            .length = 1,
        }
    };

    cmd[0] = (addr >> 8) & 0xFF;
    cmd[1] = addr & 0xFF;

    ret = I2C_TRANSFER(dev, msg, 2);
    if (ret != OK) {
        printf("%s: read 0x%04x fail\n", __func__, addr);
        return -EIO;
    }

    *data = buf;
    return 0;
}

/**
 * @brief i2c write for camera sensor
 * @param dev dev pointer to structure of i2c device data
 * @param addr address of i2c to write
 * @param data data to write
 * @return zero for success or non-zero on any faillure
 */
int camera_i2c_write(struct i2c_dev_s *dev, uint16_t addr, uint8_t data)
{
    uint8_t cmd[3] = {0x00, 0x00, 0x00};
    int ret = 0;
    struct i2c_msg_s msg[] = {
        {
            .addr = OV5645_I2C_ADDR,
            .flags = 0,
            .buffer = cmd,
            .length = 3,
        },
    };

    cmd[0] = (addr >> 8) & 0xFF;
    cmd[1] = addr & 0xFF;
    cmd[2] = data;

    ret = I2C_TRANSFER(dev, msg, 1);
    if (ret != OK) {
        printf("%s: write 0x%04x fail\n", __func__, addr);
        return -EIO;
    }

    return 0;
}

/**
 * @brief ov5645 sensor initialization function
 * @param mode mode of sensor to initialize
 * @return zero for success or non-zero on any faillure
 */
int ov5645_init(int mode)
{
    struct i2c_dev_s *cam_i2c = NULL;
    struct camera_param *initialize_table;
    uint8_t id[2] = {0, 0};
    int i = 0, count = 0;
    int ret;

    switch (mode) {
        case 0:
            initialize_table = ov5645_default_regs_init;
            count = ARRAY_SIZE(ov5645_default_regs_init);
            break;
        default:
            printf("unsupported mode!\n");
            return -EINVAL;
    }

    tsb_gpio_initialize();

    /* sensor powerup */
    tsb_gpio_direction_out(NULL, OV5645_PWDN, 0); /* shutdown -> L */
    tsb_gpio_direction_out(NULL, OV5645_RESET, 0); /* reset -> L */
    usleep(5000);

    tsb_gpio_direction_out(NULL, OV5645_PWDN, 1); /* shutdown -> H */
    usleep(1000);

    tsb_gpio_direction_out(NULL, OV5645_RESET, 1); /* reset -> H */
    usleep(1000);

    cam_i2c = up_i2cinitialize(OV5645_APB3_I2C0);
    if (!cam_i2c) {
        printf("could not init I2C!\n", __func__);
        return -EIO;
    }

    /* sensor initialize */
    /* get sensor id */
    camera_i2c_read(cam_i2c, 0x300a, &id[1]); /* sensor id (high) */
    camera_i2c_read(cam_i2c, 0x300b, &id[0]); /* sensor id (low) */
    printf("Sensor ID : 0x%04X\n", (id[1] << 8) | id[0]);

    /* fill initialize command*/
    for (i = 0; i < count; i++) {
        ret = camera_i2c_write(cam_i2c,
                               initialize_table->addr,
                               initialize_table->data);
        if (ret) {
            printf("fill ov5645 initialize command failed");
            return -EIO;
        }
        usleep(50);
        initialize_table++;
    }

    return 0;
}
