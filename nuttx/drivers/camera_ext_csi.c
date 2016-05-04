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

#include <errno.h>
#include <debug.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <apps/ice/cdsi.h>

#include <arch/chip/cdsi.h>
#include <arch/chip/cdsi_config.h>

#include <nuttx/config.h>
#include <nuttx/device_cam_ext.h>
#include <nuttx/gpio.h>
#include <nuttx/i2c.h>
#include <nuttx/math.h>

#include "camera_ext.h"
#include "camera_ext_csi.h"

#define GPIO_APBE_CAM_PWR_EN (3) /* output to camera (CAM_DVDD_EN) */
#define GPIO_APBE_CAM_RST_N  (5) /* output to camera (CAM_RST_N) */

#define CAMERA_POWER_DELAY_US (100000)

static struct cdsi_dev *g_cdsi_dev;

static int cci_write_regs(struct i2c_dev_s *i2c_dev, uint16_t i2c_addr,
        const struct cam_i2c_reg_array* items, uint16_t size);


static int cam_setup(struct camera_dev_s *cam_dev)
{
    struct sensor_info *sensor = camera_ext_csi_get_board_sensor_info();
    if (sensor == NULL) {
        return -1;
    }
    cam_dev->status = OFF;
    cam_dev->sensor = sensor;
    return 0;
}

static inline int cam_init(struct camera_dev_s *cam_dev)
{
    return cci_write_regs(cam_dev->i2c, cam_dev->sensor->i2c_addr,
               cam_dev->sensor->init.regs, cam_dev->sensor->init.size);
}

static int cam_power_on(struct device *dev)
{
    int retval;
    struct camera_dev_s *cam_dev = (struct camera_dev_s *)
            device_driver_get_private(dev);

    if (cam_dev->status == OFF) {
        //TODO: these gpio configs are changing among boards
        //move these gpio config to board level
        gpio_direction_out(GPIO_APBE_CAM_PWR_EN, 1); /* power on */
        usleep(CAMERA_POWER_DELAY_US);
        gpio_direction_out(GPIO_APBE_CAM_RST_N, 1);  /* reset not asserted */

        //TODO:should be moved up to board level initialization block.
        //There could be a case there is non-camera related I2C device
        //hanging around on the same bus.
        cam_dev->i2c = up_i2cinitialize(0);

        retval = cam_init(cam_dev);
        if (retval == 0) {
            //go standby
            retval = cci_write_regs(cam_dev->i2c, cam_dev->sensor->i2c_addr,
                cam_dev->sensor->stop.regs, cam_dev->sensor->stop.size);
        }

        if (retval == 0) {
            cam_dev->status = ON;
        }
        return retval;
    } else {
        CAM_DBG("%s: status %d\n", __func__, cam_dev->status);
        return -1;
    }
}

static int csi_cam_stream_off(struct device *dev);

static int cam_power_off(struct device *dev)
{
    struct camera_dev_s *cam_dev = (struct camera_dev_s *)
            device_driver_get_private(dev);
    if (cam_dev->status == STREAMING) {
        CAM_DBG("stop streaming before off\n");
        csi_cam_stream_off(dev);
    }
    if (cam_dev->status != OFF) {
        gpio_direction_out(GPIO_APBE_CAM_PWR_EN, 0); /* power off */
        gpio_direction_out(GPIO_APBE_CAM_RST_N, 0);  /* reset asserted */
        //TODO:should be moved up to board level initialization block.
        //There could be a case there is non-camera related I2C device
        //hanging around on the same bus.
        up_i2cuninitialize(cam_dev->i2c);
        cam_dev->status = OFF;
        cam_dev->i2c = NULL;
    } else {
        CAM_DBG("camera already off\n");
    }
    return 0;
}

static int cam_set_resolution(struct camera_dev_s *cam_dev,
        const struct cam_i2c_reg_setting *res)
{
    if (cam_dev->status == OFF) return -1;
    return cci_write_regs(cam_dev->i2c, cam_dev->sensor->i2c_addr,
               res->regs, res->size);
}

static struct cdsi_config CDSI_CONFIG = {
    .mode = TSB_CDSI_MODE_CSI,

    .tx_num_lanes = 4,
    .rx_num_lanes = 4,
    .tx_bits_per_lane = 518*1000*1000,
    .rx_bits_per_lane = 518*1000*1000,

    .hs_rx_timeout = 0xffffffff,

    .framerate = 0, /* variable */

    .pll_frs = 0,
    .pll_prd = 0,
    .pll_fbd = 26,

    .width = 0,  /* variable */
    .height = 0, /* variable */
    .bpp = 10,

    .bta_enabled = 0,
    .continuous_clock = 0,
    .blank_packet_enabled = 0,
    .video_mode = 0,
    .color_bar_enabled = 0,
};

static void generic_csi_init(struct cdsi_dev *dev)
{
    cdsi_initialize_rx(dev, &CDSI_CONFIG);
}

const static struct camera_sensor generic_sensor = {
    .cdsi_sensor_init = generic_csi_init,
};

static int csi_cam_stream_on(struct device *dev)
{
    const struct camera_ext_format_db *db = camera_ext_get_format_db();
    const struct camera_ext_format_user_config *cfg = camera_ext_get_user_config();
    const struct camera_ext_frmsize_node  *frmsize;
    struct camera_ext_frmival_node const *frmival;
    struct camera_dev_s *cam_dev = (struct camera_dev_s *)
            device_driver_get_private(dev);

    if (cam_dev->status != ON) return -1;

    frmsize = get_current_frmsize_node(db, cfg);
    if (frmsize == NULL) {
        CAM_ERR("Failed to get current frame size\n");
        return -EINVAL;
    }

    frmival = get_current_frmival_node(db, cfg);
    if (frmival == NULL) {
        CAM_ERR("Failed to get current frame interval\n");
        return -EINVAL;
    }

    struct csi_stream_user_data const *user_data = frmival->user_data;
    if (user_data == NULL) {
        CAM_ERR("User data not fond for the configuration\n");
        return -EINVAL;
    }

    if (cam_set_resolution(cam_dev, &user_data->res_regs) != 0) {
        CAM_ERR("invalid current stream config\n");
        return -EINVAL;
    }

    //start camera tx on apba
    if (cdsi_apba_cam_tx_start(&CDSI_CONFIG) != 0) {
        CAM_ERR("Failed to start CDSI TX on APBA\n");
        return -EINVAL;
    }

    //start stream on camera sensor
    if (cci_write_regs(cam_dev->i2c, cam_dev->sensor->i2c_addr,
                       cam_dev->sensor->start.regs, cam_dev->sensor->start.size) != 0) {
        CAM_DBG("Failed to start stream on sensor\n");
        goto stop_csi_tx;
    }

    CDSI_CONFIG.width = frmsize->width;
    CDSI_CONFIG.height = frmsize->height;
    float fps = (float)frmival->denominator / (float)frmival->numerator;
    CDSI_CONFIG.framerate = roundf(fps);

    g_cdsi_dev = csi_initialize((struct camera_sensor *)&generic_sensor, TSB_CDSI1, TSB_CDSI_RX);
    if (!g_cdsi_dev) {
        CAM_DBG("failed to initialize CSI RX\n");
        goto stop_sensor;
    }

    cam_dev->status = STREAMING;

    return 0;

stop_sensor:
    cci_write_regs(cam_dev->i2c, cam_dev->sensor->i2c_addr,
                   cam_dev->sensor->stop.regs, cam_dev->sensor->stop.size);
stop_csi_tx:
    cdsi_apba_cam_tx_stop();

    return -EINVAL;
}

static int csi_cam_stream_off(struct device *dev)
{
    int retval;
    struct camera_dev_s *cam_dev = (struct camera_dev_s *)
            device_driver_get_private(dev);

    if (cam_dev->status != STREAMING) return -1;

    retval = cci_write_regs(cam_dev->i2c, cam_dev->sensor->i2c_addr,
               cam_dev->sensor->stop.regs, cam_dev->sensor->stop.size);
    if (retval == 0) {
        cam_dev->status = ON;
        retval = cdsi_apba_cam_tx_stop();

        if (g_cdsi_dev) {
            csi_uninitialize(g_cdsi_dev);
            g_cdsi_dev = NULL;
        }
    }

    return retval;
}

static int cci_write_regs(struct i2c_dev_s *i2c_dev, uint16_t i2c_addr,
        const struct cam_i2c_reg_array* items, uint16_t size)
{
    struct i2c_msg_s msg;
    uint8_t data[3];
    uint16_t regaddr;
    int i, ret;
    for (i=0; i<size; i++) {
        regaddr = items[i].reg_addr;
        CAM_DBG("reg = 0x%02x data = 0x%02x\n", regaddr, items[i].data);
        data[0] = (regaddr >> 8) & 0xFF;
        data[1] = regaddr & 0xFF;
        data[2] = items[i].data;

        msg.addr   = i2c_addr;
        msg.flags  = 0;
        msg.buffer = &data[0];
        msg.length = 3;

        ret = I2C_TRANSFER(i2c_dev, &msg, 1);
        if (ret < 0) {
            CAM_ERR("i2c write failed at item %d, res = %d\n", i, ret);
            return ret;
        }
    }
    return 0;
}

#ifdef CONFIG_CAMERA_EXT_TEST_CTRLS
extern int camera_ext_tesing_ctrl_init(struct device *dev);
#endif

static int csi_cam_dev_open(struct device *dev)
{
    int retval;
    //static allocate the singleton instance
    static struct camera_dev_s camdev;
    retval = cam_setup(&camdev);
    if (retval == 0) {
        device_driver_set_private(dev, (void*)&camdev);
    } else {
        CAM_ERR("failed to open device\n");
    }
    /* Notify all sub devices to register their controls.
     * sensor_init_ctrl(cam_dev);
     * actuator_init_ctrl(cam_dev);
     * ...
     */
#ifdef CONFIG_CAMERA_EXT_TEST_CTRLS
    if (retval == 0)
        retval = camera_ext_tesing_ctrl_init(dev);
#endif

    camera_ext_register_format_db(camdev.sensor->sensor_db);
    camera_ext_register_control_db(&camdev.ctrl_db);
    return retval;
}

static void csi_cam_dev_close(struct device *dev)
{
    //ensure power off camera
    struct camera_dev_s *cam_dev = (struct camera_dev_s *)
            device_driver_get_private(dev);
    if (cam_dev->status == STREAMING) {
        CAM_DBG("stop streaming before close\n");
        csi_cam_stream_off(dev);
    }
    if (cam_dev->status == ON) {
        CAM_DBG("power off before close\n");
        cam_power_off(dev);
    }
}

static struct device_camera_ext_dev_type_ops csi_camera_ext_type_ops = {
    .register_event_cb = camera_ext_register_event_cb,
    .power_on          = cam_power_on,
    .power_off         = cam_power_off,
    .stream_on         = csi_cam_stream_on,
    .stream_off        = csi_cam_stream_off,
    .input_enum        = camera_ext_input_enum,
    .input_get         = camera_ext_input_get,
    .input_set         = camera_ext_input_set,
    .format_enum       = camera_ext_format_enum,
    .format_get        = camera_ext_format_get,
    .format_set        = camera_ext_format_set,
    .frmsize_enum      = camera_ext_frmsize_enum,
    .frmival_enum      = camera_ext_frmival_enum,
    .stream_set_parm   = camera_ext_stream_set_parm,
    .stream_get_parm   = camera_ext_stream_get_parm,
    .ctrl_get_cfg      = camera_ext_ctrl_get_cfg,
    .ctrl_get          = camera_ext_ctrl_get,
    .ctrl_set          = camera_ext_ctrl_set,
    .ctrl_try          = camera_ext_ctrl_try,
};

static struct device_driver_ops camera_ext_driver_ops = {
    .open     = csi_cam_dev_open,
    .close    = csi_cam_dev_close,
    .type_ops = &csi_camera_ext_type_ops,
};

struct device_driver cam_ext_csi_driver = {
    .type = DEVICE_TYPE_CAMERA_EXT_HW,
    .name = "camera_extension_csi",
    .desc = "driver for csi camera",
    .ops  = &camera_ext_driver_ops,
};

int register_camera_ext_ctrl_db(struct device *dev,
        const struct camera_ext_ctrl_cfg **ctrls, size_t num)
{
    struct camera_dev_s *cam_dev = (struct camera_dev_s *)
            device_driver_get_private(dev);

    cam_dev->ctrl_db.num_ctrls += num;
    cam_dev->ctrl_db.ctrls = realloc(cam_dev->ctrl_db.ctrls,
        sizeof(struct camera_ext_ctrl_cfg *) * cam_dev->ctrl_db.num_ctrls);
    if (cam_dev->ctrl_db.ctrls == NULL) {
        cam_dev->ctrl_db.num_ctrls = 0;
        CAM_ERR("OOM\n");
        return -ENOMEM;
    }

    const struct camera_ext_ctrl_cfg **tail =
                &cam_dev->ctrl_db.ctrls[cam_dev->ctrl_db.num_ctrls - num];
    memcpy(tail, ctrls, sizeof(struct camera_ext_ctrl_cfg *) * num);
    return 0;
}
