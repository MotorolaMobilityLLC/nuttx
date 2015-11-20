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

#include <nuttx/config.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nuttx/i2c.h>
#include <nuttx/gpio.h>

#include "csi_camera.h"

#define GPIO_APBE_CAM_PWR_EN (3) /* output to camera (CAM_DVDD_EN) */
#define GPIO_APBE_CAM_RST_N  (5) /* output to camera (CAM_RST_N) */

#define CAMERA_POWER_DELAY_US (100000)

#ifndef DEBUG
#define DEBUG
#endif

#ifdef DEBUG
#define CHK_PTR(p) do { \
    if (!p) { \
        lldbg("%s:null pointer %s\n", __func__, #p); \
        return -1; \
    } \
}while(0)
#else
#define CHK_PTR(p)
#endif
static int cci_write_regs(struct i2c_dev_s *i2c_dev, uint16_t i2c_addr,
        struct cam_i2c_reg_array* items, uint16_t size);

int cam_setup(struct camera_dev_s *cam_dev)
{
    CHK_PTR(cam_dev);
    struct sensor_info *sensor = get_board_sensor_info();
    if (sensor == NULL) {
        return -1;
    }
    cam_dev->status = OFF;
    cam_dev->sensor = sensor;
    return 0;
}

int cam_query_info(struct camera_dev_s *cam_dev,
        struct gb_csi_camera_info *cam_info)
{
    CHK_PTR(cam_dev);
    CHK_PTR(cam_info);
    memcpy(cam_info, &cam_dev->sensor->cam_info, sizeof(*cam_info));
    return 0;
}

static int cam_init(struct camera_dev_s *cam_dev)
{
    CHK_PTR(cam_dev);
    return cci_write_regs(cam_dev->i2c, cam_dev->sensor->i2c_addr,
               cam_dev->sensor->init.regs, cam_dev->sensor->init.size);
}

int cam_power_on(struct camera_dev_s *cam_dev)
{
    int retval;
    CHK_PTR(cam_dev);
    if (cam_dev->status == OFF) {
        gpio_direction_out(GPIO_APBE_CAM_PWR_EN, 1); /* power on */
        usleep(CAMERA_POWER_DELAY_US);
        gpio_direction_out(GPIO_APBE_CAM_RST_N, 1);  /* reset not asserted */
        cam_dev->i2c = up_i2cinitialize(0);

        retval = cam_init(cam_dev);
        if (retval == 0) {
            //go standby
            retval = cci_write_regs(cam_dev->i2c, cam_dev->sensor->i2c_addr,
                cam_dev->sensor->stop.regs, cam_dev->sensor->stop.size);
        }
        if (retval == 0) cam_dev->status = ON;
        return retval;
    } else {
        lldbg("%s: status %d\n", __func__, cam_dev->status);
        return -1;
    }
}

int cam_power_off(struct camera_dev_s *cam_dev)
{
    CHK_PTR(cam_dev);
    if (cam_dev->status != OFF) {
        gpio_direction_out(GPIO_APBE_CAM_PWR_EN, 0); /* power off */
        gpio_direction_out(GPIO_APBE_CAM_RST_N, 0);  /* reset asserted */
        up_i2cuninitialize(cam_dev->i2c);
        cam_dev->status = OFF;
        cam_dev->i2c = NULL;
    }
    return 0;
}

int cam_stream_on(struct camera_dev_s *cam_dev)
{
    int retval;
    CHK_PTR(cam_dev);
    if (cam_dev->status != ON) return -1;
    retval = cci_write_regs(cam_dev->i2c, cam_dev->sensor->i2c_addr,
               cam_dev->sensor->start.regs, cam_dev->sensor->start.size);
    if (retval == 0) {
        cam_dev->status = STREAMING;
    }
    return retval;
}

int cam_stream_off(struct camera_dev_s *cam_dev)
{
    int retval;
    CHK_PTR(cam_dev);
    if (cam_dev->status != STREAMING) return -1;
    retval = cci_write_regs(cam_dev->i2c, cam_dev->sensor->i2c_addr,
               cam_dev->sensor->stop.regs, cam_dev->sensor->stop.size);
    if (retval == 0) {
        cam_dev->status = ON;
    }
    return retval;
}

int cam_set_resolution(struct camera_dev_s *cam_dev, int res)
{
    CHK_PTR(cam_dev);
    if (cam_dev->status == OFF) return -1;
    if (res < 0 || res >= cam_dev->sensor->resolutions.size) {
        lldbg("%s: %d is out of range\n", __func__, res);
        return -1;
    }
    return cci_write_regs(cam_dev->i2c, cam_dev->sensor->i2c_addr,
               cam_dev->sensor->resolutions.settings[res].regs,
               cam_dev->sensor->resolutions.settings[res].size);
}

#ifdef TEST_PATTERN
int cam_set_test_pattern(struct camera_dev_s *cam_dev, int test)
{
    CHK_PTR(cam_dev);
    if (cam_dev->status == OFF) return -1;
    if (test < 0 || test >= cam_dev->sensor->test.size) {
        lldbg("%s: %d is out of range\n", __func__, test);
        return -1;
    }
    return cci_write_regs(cam_dev->i2c, cam_dev->sensor->i2c_addr,
               cam_dev->sensor->test.settings[test].regs,
               cam_dev->sensor->test.settings[test].size);
}
#endif

static int cci_write_regs(struct i2c_dev_s *i2c_dev, uint16_t i2c_addr,
        struct cam_i2c_reg_array* items, uint16_t size)
{
    struct i2c_msg_s msg;
    uint8_t data[3];
    uint16_t regaddr;
    int i, ret;
    for (i=0; i<size; i++) {
        regaddr = items[i].reg_addr;
        lldbg("reg = %x data = %d\n", regaddr, items[i].data);
        data[0] = (regaddr >> 8) & 0xFF;
        data[1] = regaddr & 0xFF;
        data[2] = items[i].data;

        /* Set up to write the address */
        msg.addr   = i2c_addr;
        msg.flags  = 0;
        msg.buffer = &data[0];
        msg.length = 3;

        /* Read the register data.  The returned value is the number messages
         * completed.
         */
        ret = I2C_TRANSFER(i2c_dev, &msg, 1);
        if (ret < 0) {
            lldbg("i2c write failed at item %d, res = %d\n", i, ret);
            return ret;
        }
    }
    return 0;
}
