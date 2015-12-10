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

const static struct cdsi_config CAMERA_CONFIG = {
    /* TODO: some fields like mbits_per_lane, width, height etc. should
     *       be derived from configuration chosen by the user.
     */
    /* Common */
    .mode = TSB_CDSI_MODE_CSI,
    .mbits_per_lane = 518*1000*1000,
    .num_lanes = 4,
    /* RX only */
    .hs_rx_timeout = 0xffffffff,
    /* TX only */
    .framerate = 24,
    .pll_frs = 0,
    .pll_prd = 0,
    .pll_fbd = 26,
    .width = 3264,
    .height = 2510,
    .bpp = 10,
    .bta_enabled = 0,
    .continuous_clock = 0,
    .blank_packet_enabled = 0,
    .video_mode = 0,
    .color_bar_enabled = 0,
    /* CSI only */
    /* DSI only */
    /* Video Mode only */
    /* Command Mode only */
};

static void generic_csi_init(struct cdsi_dev *dev)
{
    cdsi_initialize_rx(dev, &CAMERA_CONFIG);
}

const static struct camera_sensor generic_sensor = {
    .cdsi_sensor_init = generic_csi_init,
};

static int csi_cam_stream_on(struct device *dev)
{
    int retval;
    struct camera_ext_frmival_node const *frmival_node;
    struct camera_dev_s *cam_dev = (struct camera_dev_s *)
            device_driver_get_private(dev);

    if (cam_dev->status != ON) return -1;

    frmival_node = get_current_frmival_node(cam_dev->sensor->sensor_db,
            &cam_dev->sensor->user_config);

    retval = -EINVAL;
    if (frmival_node != NULL) {
        struct csi_stream_user_data const *user_data = frmival_node->user_data;
        retval = cam_set_resolution(cam_dev, &user_data->res_regs);
    } else {
        CAM_ERR("invalid current stream config\n");
        return -EINVAL;
    }

    //start apa camera tx
    if (retval == 0) {
        retval = cdsi_apba_cam_tx_start(&CAMERA_CONFIG);
        CAM_DBG("start apba cam tx result %d\n", retval);
    }

    //start stream
    if (retval == 0) {
        retval = cci_write_regs(cam_dev->i2c, cam_dev->sensor->i2c_addr,
               cam_dev->sensor->start.regs, cam_dev->sensor->start.size);
    }

    g_cdsi_dev = csi_initialize((struct camera_sensor *)&generic_sensor, TSB_CDSI1, TSB_CDSI_RX);
    if (!g_cdsi_dev) {
        CAM_DBG("failed to initialize CSI RX\n");
        retval = -1;
    }

    if (retval == 0) {
        cam_dev->status = STREAMING;
    }
    return retval;
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

static int csi_cam_input_enum(struct device *dev,
        struct camera_ext_input *input)
{
    struct camera_dev_s *cam_dev = (struct camera_dev_s *)
            device_driver_get_private(dev);
    int index = le32_to_cpu(input->index);
    struct camera_ext_input_node const *input_node;

    input_node = get_input_node(cam_dev->sensor->sensor_db, index);

    if (input_node == NULL) {
        CAM_DBG("no such input: %d\n", index);
        return -EFAULT;
    }

    memcpy(input, &input_node->raw_data, sizeof(*input));
    return 0;
}

static int csi_cam_input_get(struct device *dev, int *input)
{
    struct camera_dev_s *cam_dev = (struct camera_dev_s *)
            device_driver_get_private(dev);

    struct camera_ext_input_node const *input_node;
    struct gb_camera_ext_sensor_user_config *cfg;

    cfg = &cam_dev->sensor->user_config;
    input_node = get_current_input_node(cam_dev->sensor->sensor_db, cfg);

    if (input_node == NULL) {
        CAM_ERR("invalid current config\n");
        return -EFAULT;
    }

    *input = cfg->input;
    return 0;
}

static int csi_cam_input_set(struct device *dev, int index)
{
    struct camera_dev_s *cam_dev = (struct camera_dev_s *)
            device_driver_get_private(dev);
    struct gb_camera_ext_sensor_db const *db = cam_dev->sensor->sensor_db;

    if (index < 0 || index >= db->num_inputs) {
        CAM_DBG("v4l input index %d out of range\n", index);
        return -EFAULT;
    }

    cam_dev->sensor->user_config.input = index;
    return 0;
}

static int csi_cam_format_enum(struct device *dev,
        struct camera_ext_fmtdesc *format)
{
    struct camera_dev_s *cam_dev = (struct camera_dev_s *)
            device_driver_get_private(dev);
    struct gb_camera_ext_sensor_user_config *cfg;
    struct camera_ext_format_node const *format_node;
    int index = le32_to_cpu(format->index);

    cfg = &cam_dev->sensor->user_config;
    format_node = get_format_node(cam_dev->sensor->sensor_db,
        cfg->input, index);

    if (format_node == NULL) {
        CAM_DBG("no such format: %d\n", index);
        return -EFAULT;
    }

    memcpy(format, &format_node->raw_data, sizeof(*format));
    return 0;
}

static int csi_cam_format_get(struct device *dev,
        struct camera_ext_format *format)
{
    struct camera_dev_s *cam_dev = (struct camera_dev_s *)
            device_driver_get_private(dev);
    return cam_ext_fill_current_format(cam_dev->sensor->sensor_db,
            &cam_dev->sensor->user_config, format);
}

static int csi_cam_format_set(struct device *dev,
        struct camera_ext_format* format)
{
    int retval;
    struct camera_dev_s *cam_dev = (struct camera_dev_s *)
            device_driver_get_private(dev);

    struct gb_camera_ext_sensor_db const *db;
    struct gb_camera_ext_sensor_user_config *cfg;
    struct camera_ext_input_node const *input_node;

    db = cam_dev->sensor->sensor_db;
    cfg = &cam_dev->sensor->user_config;
    input_node = &db->input_nodes[cfg->input];
    retval = cam_ext_set_current_format(input_node, cfg, format);

    return retval;
}

static int csi_cam_frmsize_enum(struct device *dev,
        struct camera_ext_frmsize* frmsize)
{
    struct camera_dev_s *cam_dev = (struct camera_dev_s *)
            device_driver_get_private(dev);

    struct gb_camera_ext_sensor_db const *db;
    struct gb_camera_ext_sensor_user_config *cfg;
    struct camera_ext_input_node const *input_node;
    int index = le32_to_cpu(frmsize->index);

    db = cam_dev->sensor->sensor_db;
    cfg = &cam_dev->sensor->user_config;
    input_node = &db->input_nodes[cfg->input];

    return cam_ext_frmsize_enum(input_node, index, frmsize);
}

static int csi_cam_frmival_enum(struct device *dev,
        struct camera_ext_frmival* frmival)

{
    struct camera_dev_s *cam_dev = (struct camera_dev_s *)
            device_driver_get_private(dev);

    struct gb_camera_ext_sensor_db const *db;
    struct gb_camera_ext_sensor_user_config *cfg;
    struct camera_ext_input_node const *input_node;
    int index = le32_to_cpu(frmival->index);

    db = cam_dev->sensor->sensor_db;
    cfg = &cam_dev->sensor->user_config;
    input_node = &db->input_nodes[cfg->input];

    return cam_ext_frmival_enum(input_node, index, frmival);
}

static int csi_cam_stream_set_parm(struct device *dev,
        struct camera_ext_streamparm *parm)
{
    int retval;
    struct camera_dev_s *cam_dev = (struct camera_dev_s *)
            device_driver_get_private(dev);
    struct gb_camera_ext_sensor_db const *db;
    struct gb_camera_ext_sensor_user_config *cfg;
    struct camera_ext_frmival_node const *frmival_node;

    if (cam_dev->status == STREAMING) {
        CAM_ERR("can not update stream param during streaming\n");
        return -EBUSY;
    }

    db = cam_dev->sensor->sensor_db;
    cfg = &cam_dev->sensor->user_config;
    frmival_node = cam_ext_frmival_set(db, cfg, parm);

    if (frmival_node == NULL) {
        CAM_ERR("failed to apply stream parm\n");
        return -EINVAL;
    }

    //write the resolution setttings
    struct csi_stream_user_data const *user_data;
    user_data = frmival_node->user_data;
    retval = cam_set_resolution(cam_dev, &user_data->res_regs);
    return retval;
}

static int csi_cam_stream_get_parm(struct device *dev,
        struct camera_ext_streamparm *parm)
{
    __le32 frmival_type;
    struct camera_dev_s *cam_dev = (struct camera_dev_s *)
            device_driver_get_private(dev);
    struct gb_camera_ext_sensor_db const *db;
    struct gb_camera_ext_sensor_user_config *cfg;
    struct camera_ext_frmival_node const *frmival_node;

    db = cam_dev->sensor->sensor_db;
    cfg = &cam_dev->sensor->user_config;
    frmival_node = get_current_frmival_node(db, cfg);

    if (frmival_node == NULL) {
        CAM_ERR("failed to get current frmival node\n");
        return -EINVAL;
    }

    memset(parm, 0, sizeof(*parm));
    parm->type = cpu_to_le32(CAMERA_EXT_BUFFER_TYPE_VIDEO_CAPTURE);
    frmival_type = frmival_node->raw_data.type;
    if (frmival_type == le32_to_cpu(CAM_EXT_FRMIVAL_TYPE_DISCRETE)) {
        parm->capture.timeperframe.numerator=
                frmival_node->raw_data.discrete.numerator;
        parm->capture.timeperframe.denominator =
                frmival_node->raw_data.discrete.denominator;
    } else {
        //for step/continuous type, frame rate stored in user config
        parm->capture.timeperframe.numerator = cfg->frmival.numerator;
        parm->capture.timeperframe.denominator = cfg->frmival.denominator;
    }
    parm->capture.capability = cpu_to_le32(CAMERA_EXT_MODE_HIGHQUALITY);
    parm->capture.capturemode = cpu_to_le32(CAMERA_EXT_CAP_TIMEPERFRAME);
    return 0;
}

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
    .power_on        = cam_power_on,
    .power_off       = cam_power_off,
    .stream_on       = csi_cam_stream_on,
    .stream_off      = csi_cam_stream_off,
    .input_enum      = csi_cam_input_enum,
    .input_get       = csi_cam_input_get,
    .input_set       = csi_cam_input_set,
    .format_enum     = csi_cam_format_enum,
    .format_get      = csi_cam_format_get,
    .format_set      = csi_cam_format_set,
    .frmsize_enum    = csi_cam_frmsize_enum,
    .frmival_enum    = csi_cam_frmival_enum,
    .stream_set_parm = csi_cam_stream_set_parm,
    .stream_get_parm = csi_cam_stream_get_parm,
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
