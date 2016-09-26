/*
 * Copyright (c) 2015 Google Inc. All rights reserved.
 * Copyright (C) 2016 Motorola Mobility, LLC. All rights reserved.
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
 *
 * Author: Fabien Parent <fparent@baylibre.com>
 * Author: Alexandre Bailon <abailon@baylibre.com>
 */

#include <nuttx/config.h>
#include <nuttx/usb.h>
#include <nuttx/gpio.h>
#include <nuttx/arch.h>
#include <nuttx/i2c.h>
#include <arch/byteorder.h>

#include <string.h>
#include <debug.h>

#define HUB_RESET_ASSERTION_TIME_IN_USEC    5 /* us */
#define HUB_RESET_DEASSERTION_TIME_IN_MSEC  300 /* ms */

struct i2c_dev_info {
    struct i2c_dev_s *i2c;
    uint16_t i2c_addr;
};

struct dev_private_s {
    uint8_t rst_n;
    struct i2c_dev_info i2c_info;
};

struct dev_private_s s_data;

#define HUB_I2C_ADDR 0x2D
#define MAX_SMBUS_DATA_LEN 12

#define HUB_I2C_BUS 2

static bool s_hsic_uplink = false;

void usb3813_set_hsic_uplink(bool uplink)
{
    s_hsic_uplink = uplink;
}

static int smbus_read(struct i2c_dev_info *i2c, uint8_t *data, uint8_t data_len)
{
    struct i2c_msg_s msg[2];

    /* use 0x0004 for read per spec */
    uint16_t tmp = cpu_to_be16(0x0004);
    msg[0].addr   = i2c->i2c_addr;
    msg[0].flags  = 0;
    msg[0].buffer = (uint8_t *)&tmp;
    msg[0].length = 2;

    msg[1].addr   = i2c->i2c_addr;
    msg[1].flags  = I2C_M_READ;
    msg[1].buffer = data;
    msg[1].length = data_len;

    int ret = I2C_TRANSFER(i2c->i2c, msg, 2);
    if (ret != 0) {
        lldbg("SmBus read transfer failed %d\n", ret);
        return -1;
    }
    return 0;
}

static int smbus_write(struct i2c_dev_info *i2c, uint8_t *data, uint8_t data_len)
{
    struct i2c_msg_s msg;
    uint8_t tmp[MAX_SMBUS_DATA_LEN + 2];

    if (data_len > (MAX_SMBUS_DATA_LEN))
        return -1;

    /* use address 0x0000 per spec */
    tmp[0] = 0;
    tmp[1] = 0;

    int i;
    for (i = 0; i < data_len; i++) {
        tmp[i + 2] = data[i];
    }

    msg.addr   = i2c->i2c_addr;
    msg.flags  = 0;
    msg.buffer = tmp;
    msg.length = data_len + 2;

    int ret = I2C_TRANSFER(i2c->i2c, &msg, 1);
    if (ret != 0) {
        lldbg("SMBus write transfer failed %d\n", ret);
        return -1;
    }

    return 0;
}

static int send_config_register_access(struct i2c_dev_info *i2c)
{
    struct i2c_msg_s msg;
    uint8_t tmp[3];

    /* special i2c command - 0x9937 to request read/write operation*/
    tmp[0] = 0x99;
    tmp[1] = 0x37;
    tmp[2] = 0x00;

    msg.addr   = i2c->i2c_addr;
    msg.flags  = 0;
    msg.buffer = tmp;
    msg.length = 3;

    int ret = I2C_TRANSFER(i2c->i2c, &msg, 1);
    if (ret != 0) {
        lldbg("SMBus config register access failed %d\n", ret);
        return -1;
    }

    return 0;
}

static int send_usb_attach(struct i2c_dev_info *i2c)
{
    struct i2c_msg_s msg;
    uint8_t tmp[3];

    /* special i2c command - 0xaa55 to change hub state to normal from cfg */
    tmp[0] = 0xaa;
    tmp[1] = 0x55;
    tmp[2] = 0x00;

    msg.addr   = i2c->i2c_addr;
    msg.flags  = 0;
    msg.buffer = tmp;
    msg.length = 3;

    int ret = I2C_TRANSFER(i2c->i2c, &msg, 1);
    if (ret != 0) {
        lldbg("SMBus USB attach failed %d\n", ret);
        return -1;
    }

    return 0;
}

static int read_config(struct i2c_dev_info *i2c, uint16_t address,
                       uint8_t *data, uint8_t data_len)
{
    uint8_t tmp[5];
    uint8_t tmp_data[MAX_SMBUS_DATA_LEN + 1];

    if (data_len > MAX_SMBUS_DATA_LEN)
        return -1;

    tmp[0] = 4;  /* number of bytes to write to memory */
    tmp[1] = 1;  /* read configuration register */
    tmp[2] = data_len;  /* bytes reading */
    tmp[3] = (address >> 8) & 0xFF;  /* M address of data to read */
    tmp[4] = address & 0xFF;         /* L address of data to read */

    if (smbus_write(i2c, tmp, sizeof(tmp)) < 0)
        return -1;

    if (send_config_register_access(i2c) < 0)
        return -1;

    if (smbus_read(i2c, tmp_data, data_len + 1) < 0)
        return -1;

    memcpy(data, tmp_data + 1, data_len);
    return 0;
}

static int write_config(struct i2c_dev_info *i2c, uint16_t address,
                       uint8_t *data, uint8_t data_len)
{
    uint8_t tmp[MAX_SMBUS_DATA_LEN + 5 + 1]; /* hdr & final zero */

    if (data_len > MAX_SMBUS_DATA_LEN) {
        lldbg("SMBus write data too big. (len=%u)\n", data_len);
        return -1;
    }

    tmp[0] = 4 + data_len;  /* number of bytes to write to memory */
    tmp[1] = 0;             /* wtie configuration register */
    tmp[2] = data_len;      /* bytes writing */
    tmp[3] = (address >> 8) & 0xFF;  /* M address of data to read */
    tmp[4] = address & 0xFF;         /* L address of data to read */

    int i;
    for (i = 0; i < data_len; i++) {
        tmp[i + 5] = data[i];
    }
    tmp[i + 5] = 0x00;

    if (smbus_write(i2c, tmp, 5 + data_len + 1) < 0) {
        return -1;
    }
    return send_config_register_access(i2c);
}

/**
 * Init the USB3813 hub
 *
 * Activate the GPIO line
 *
 * @param dev Device
 * @return 0 if successful
 */
static int usb3813_open(struct device *dev)
{
    gpio_activate(s_data.rst_n);

    s_data.i2c_info.i2c = up_i2cinitialize(HUB_I2C_BUS);
    s_data.i2c_info.i2c_addr = HUB_I2C_ADDR;

    return 0;
}

/**
 * Deinit the USB3813 hub
 *
 * Deactivate the GPIO line
 *
 * @param dev Device
 */
static void usb3813_close(struct device *dev)
{
    gpio_deactivate(s_data.rst_n);

    up_i2cuninitialize(s_data.i2c_info.i2c);
}

/**
 * Hold the usb hub under reset
 *
 * @param dev Device
 * @return 0 if successful
 */
static int usb3813_hold_reset(struct device *dev)
{
    gpio_direction_out(s_data.rst_n, 0);
    up_udelay(HUB_RESET_ASSERTION_TIME_IN_USEC);
    return 0;
}

/**
 * Release the usb hub from reset
 *
 * @param dev Device
 * @return 0 if successful
 */
static int usb3813_release_reset(struct device *dev)
{
    gpio_direction_out(s_data.rst_n, 1);
    up_mdelay(HUB_RESET_DEASSERTION_TIME_IN_MSEC);

    uint16_t vid, pid, devid;
    if (read_config(&s_data.i2c_info, 0x3000, (uint8_t *)&vid, sizeof(vid)) == 0)
        lldbg("HUB VID %0x\n", vid);

    if (read_config(&s_data.i2c_info, 0x3002, (uint8_t *)&pid, sizeof(pid)) == 0)
        lldbg("HUB PID %0x\n", pid);

    if (read_config(&s_data.i2c_info, 0x3004, (uint8_t *)&devid, sizeof(devid)) == 0)
        lldbg("HUB DEVICE ID %0x\n", devid);

    uint8_t conn_cfg;
    if (read_config(&s_data.i2c_info, 0x318E, &conn_cfg, 1) != 0) {
        lldbg("Failed to read current connection config\n");
        return -1;
    }

    if (s_hsic_uplink) {
        conn_cfg |= 0x03;
        if (write_config(&s_data.i2c_info, 0x318E, &conn_cfg, 1) != 0) {
            lldbg("Failed to write connection config\n");
            return -1;
        }
    }

    return send_usb_attach(&s_data.i2c_info);
}

static int usb3813_probe(struct device *dev)
{
    struct device_resource *res;

    memset(&s_data, 0, sizeof(s_data));

    res = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO, "hub_rst_n");
    if (!res) {
        lldbg("failed to get rst_n gpio\n");
        return -ENODEV;
    }
    s_data.rst_n = res->start;

    return 0;
}

/**
 * Reset the USB Hub
 *
 * @param dev Device
 * @return 0 if successful
 */
static int usb3813_reset(struct device *dev)
{
    int retval;

    retval = usb3813_hold_reset(dev);
    if (!retval) {
        return retval;
    }

    retval = usb3813_release_reset(dev);

    return retval;
}

static struct device_hsic_type_ops usb3813_type_ops = {
    .reset = usb3813_reset,
    .hold_reset = usb3813_hold_reset,
    .release_reset = usb3813_release_reset,
};

static struct device_driver_ops usb3813_driver_ops = {
    .probe = usb3813_probe,
    .open = usb3813_open,
    .close = usb3813_close,
    .type_ops = &usb3813_type_ops,
};

struct device_driver usb3813_driver = {
    .type = DEVICE_TYPE_HSIC_DEVICE,
    .name = "usb3813",
    .desc = "USB3813 HSIC Hub Driver",
    .ops = &usb3813_driver_ops,
};
