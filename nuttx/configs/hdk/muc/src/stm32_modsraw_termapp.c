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
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <arch/board/mods.h>
#include <apps/netutils/cJSON.h>
#include <nuttx/greybus/debug.h>
#include <nuttx/arch.h>
#include <nuttx/device.h>
#include <nuttx/device_raw.h>
#include <nuttx/gpio.h>
#include <nuttx/time.h>

#include "hdk.h"

/* max data buffer/column count */
#define MODTERM_MAX_BUF_LENGTH   128

struct modterm_info {
    struct device *gDevice;                 /* device handle for this driver */
    int fd;                                 /* file descriptor handle */
    raw_send_callback gCallback;            /* callback to send back messages */
    pthread_t rx_thread;                    /* rx thread to read from usart */
    uint8_t post_buf[MODTERM_MAX_BUF_LENGTH]; /* buffer for sending back data */
};

static struct modterm_info *info;

/**
 * rx thread to read data from uart and send back over raw
 * in JSON format.
 */
static void *modterm_rx_thread(void *data)
{
    int ret = 0;
    char *output;
    cJSON *root, *fmt_term;
#ifndef CONFIG_TERMAPP_ROCK_SEVEN
    int i, data_size;
    uint8_t rx_buf[MODTERM_MAX_BUF_LENGTH];
#endif

    if (!info) {
        return (void *)(-EINVAL);
    }

    /* poll read for data */
    while (1) {
        /* initialize the buffers and data_size */
        memset(info->post_buf,0,sizeof(info->post_buf));
#ifdef CONFIG_TERMAPP_ROCK_SEVEN
        /* blocking read */
        ret = read(info->fd, &info->post_buf[0], sizeof(info->post_buf));

        llvdbg("send %d bytes over raw\n", ret);
#else
        memset(rx_buf, 0, sizeof(rx_buf));
        data_size = 0;
        /* read uart data till carriage return */
        while(rx_buf[0] != 0xd) {
            /* blocking read */
            ret = read(info->fd, &rx_buf[0], sizeof(rx_buf));

            /* copy all the characters that were buffered in read */
            for(i=0; i<ret; i++) {
                info->post_buf[data_size] = rx_buf[i];
                data_size++;
                if (data_size > MODTERM_MAX_BUF_LENGTH) {
                    break;
                }
            }
        }

        llvdbg("send %d bytes over raw\n", data_size);
#endif

        if (info->gCallback) {
            /* build JSON objects for terminal */
            root=cJSON_CreateObject();
            cJSON_AddItemToObject(root, "term", fmt_term=cJSON_CreateObject());
            cJSON_AddStringToObject(fmt_term,"data", (const char *)&info->post_buf[0]);
#ifdef CONFIG_TERMAPP_ROCK_SEVEN
            cJSON_AddNumberToObject(fmt_term,"size", ret);
#else
            cJSON_AddNumberToObject(fmt_term,"size", data_size);
#endif
            cJSON_AddStringToObject(fmt_term,"sender", "mod");

            /* generate JSON for terminal */
            output = cJSON_PrintUnformatted(root);
            llvdbg("output: %x, size: %d\n", *output, strlen(output));
            if (!output) {
                return (void *)(-EINVAL);
            }

            /* send JSON format data over raw */
            info->gCallback(info->gDevice, strlen(output), (uint8_t *)output);
            free(output);
            cJSON_Delete(root);
        }
    }

    dbg("modterm_rx_thread done: ret=%d\n", ret);
    return (void *)ret;
}


/**
 * Process the JSON format data received over raw.
 */
static int modterm_raw_recv(struct device *dev, uint32_t len, uint8_t data[])
{
    struct modterm_info *mt_info = NULL;
    int i, array_size;
    char *data_recv;
    int data_size;
    char *cmd_data;
    char *port;
    uint8_t pin;
    uint8_t level;
    int gpio_direction;
    char *direction;
    char *output;
    cJSON *resp_root, *fmt_command, *fmt_gpios;
    cJSON *root, *term, *gpios, *command, *gpio_item;

    if (len == 0)
        return -EINVAL;

    if (!dev || !device_get_private(dev)) {
        return -ENODEV;
    }
    mt_info = device_get_private(dev);

    /* Parse the received data */
    root = cJSON_Parse((const char *)&data[0]);
    if(!root)
        return -EINVAL;

    /* get the term object containing terminal data */
    term = cJSON_GetObjectItem(root,"term");
    if (term) {
        /* get the received terminal data */
        data_recv = cJSON_GetObjectItem(term,"data")->valuestring;
        /* get the data size */
        data_size = cJSON_GetObjectItem(term,"size")->valueint;
#ifdef CONFIG_TERMAPP_ROCK_SEVEN
        if (*(data_recv + (data_size - 1)) == 0xa)
            *(data_recv + (data_size - 1)) = 0xd;
#endif

        llvdbg("wrote %d bytes to uart\n", data_size);
        /* write the data to uart */
        write(mt_info->fd, data_recv, data_size);
    }

    /* get the gpio object containing gpio data */
    gpios = cJSON_GetObjectItem(root,"gpios");
    if (gpios) {
        array_size = cJSON_GetArraySize(gpios);
        llvdbg("array size = %d\n", array_size);

        /* check for command object */
        command = cJSON_GetObjectItem(root,"command");
        if (command) {
            /* read the command string */
            cmd_data = cJSON_GetObjectItem(command,"data")->valuestring;
            llvdbg("command = %s\n", cmd_data);
            /* process the read command */
            if (!strcmp(cmd_data, "read")) {
                if (info->gCallback) {
                    /* prepare the response with command object */
                    resp_root=cJSON_CreateObject();
                    cJSON_AddItemToObject(resp_root, "command",
                                            fmt_command=cJSON_CreateObject());
                    /* response command is status */
                    cJSON_AddStringToObject(fmt_command,"data", "status");
                    cJSON_AddStringToObject(fmt_command,"sender", "mod");
                    /* create gpio array object for response */
                    cJSON_AddItemToObject(resp_root, "gpios", fmt_gpios=cJSON_CreateArray());

                    /* read the gpio items from received gpio object */
                    for(i=0; i<array_size; i++) {
                        gpio_item = cJSON_GetArrayItem(gpios, i);
                        if (!gpio_item) {
                            dbg("gpio_item not available %d\n", i);
                            continue;
                        }
                        /* read the gpio port and pin from received array */
                        port = cJSON_GetObjectItem(gpio_item,"port")->valuestring;
                        pin = cJSON_GetObjectItem(gpio_item,"pin")->valueint;

                        /* create gpio array item with port and pin for response */
                        cJSON *gpio_arrayitem = cJSON_CreateObject();
                        cJSON_AddStringToObject(gpio_arrayitem,"port", port);
                        cJSON_AddNumberToObject(gpio_arrayitem,"pin", pin);

                        /* write the level for the port:pin gpio */
                        cJSON_AddNumberToObject(gpio_arrayitem,"level",
                                    gpio_get_value(CALC_GPIO_NUM(*port,pin)));

                        /* write the direction for the port:pin gpio */
                        gpio_direction = gpio_get_direction(CALC_GPIO_NUM(*port,pin));
                        if (gpio_direction == 1)
                            cJSON_AddStringToObject(gpio_arrayitem,"direction", "in");
                        else if (gpio_direction == 0)
                            cJSON_AddStringToObject(gpio_arrayitem,"direction", "out");
                        else
                            cJSON_AddStringToObject(gpio_arrayitem,"direction", "unk");
                        cJSON_AddItemToArray(fmt_gpios, gpio_arrayitem);
                    }

                    /* generate JSON for command and gpios */
                    output = cJSON_PrintUnformatted(resp_root);
                    if (!output) {
                        return (-EINVAL);
                    }
                    llvdbg("output: %x, size: %d\n", *output, strlen(output));

                    /* send JSON format data over raw */
                    info->gCallback(info->gDevice, strlen(output), (uint8_t *)output);
                    free(output);
                    cJSON_Delete(resp_root);
                }
            } else if (!strcmp(cmd_data, "write")) {
                /* read the gpio items from received gpio object */
                for(i=0; i<array_size; i++) {
                    gpio_item = cJSON_GetArrayItem(gpios, i);
                    if (!gpio_item) {
                        dbg("gpio_item not available %d\n", i);
                        continue;
                    }
                    /* read the gpio port, pin, level from received array */
                    port = cJSON_GetObjectItem(gpio_item,"port")->valuestring;
                    pin = cJSON_GetObjectItem(gpio_item,"pin")->valueint;
                    level = cJSON_GetObjectItem(gpio_item,"level")->valueint;

                    /* read the direction and set the gpio direction
                     * and level.
                     */
                    direction = cJSON_GetObjectItem(gpio_item,
                                            "direction")->valuestring;
                    if (!strcmp(direction, "in")) {
                        gpio_direction_in(CALC_GPIO_NUM(*port,pin));
                    }

                    if (!strcmp(direction, "out")) {
                        gpio_direction_out(CALC_GPIO_NUM(*port,pin), level);
                    }
                    llvdbg("item:%d, port:%s, pin:%d level:%d, direction:%s\n",
                            i, port, pin, level, direction);
                }
            }
        }
    }

    return 0;
}

static int modterm_raw_register_callback(struct device *dev,
        raw_send_callback callback)
{
    struct modterm_info *mt_info = NULL;

    if (!dev || !device_get_private(dev)) {
        return -ENODEV;
    }

    mt_info = device_get_private(dev);
    mt_info->gCallback = callback;
    return 0;
}

static int modterm_raw_unregister_callback(struct device *dev)
{
    struct modterm_info *mt_info = NULL;

    if (!dev || !device_get_private(dev)) {
        return -ENODEV;
    }

    mt_info = device_get_private(dev);
    mt_info->gCallback = NULL;
    return 0;
}

static int modterm_raw_probe(struct device *dev)
{
    int ret;

    if (!dev) {
        return -EINVAL;
    }

    gb_debug("%s:\n",__func__);
    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    info->gDevice = dev;
    device_set_private(dev, info);

    info->fd = open(TERMAPP_RAW_UART2_DEVPATH, O_RDWR);
    if (info->fd < 0) {
        dbg("open %s failed: %d\n", TERMAPP_RAW_UART2_DEVPATH, errno);
        free(info);
        return -ENODEV;
    }

    ret = pthread_create(&info->rx_thread, NULL, modterm_rx_thread, NULL);
    if (ret) {
        dbg("ERROR: Failed to create rx thread: %s.\n", strerror(errno));
        info->rx_thread = 0;
        free(info);
        return ret;
    }

    return 0;
}

static struct device_raw_type_ops modterm_raw_type_ops = {
    .recv = modterm_raw_recv,
    .register_callback = modterm_raw_register_callback,
    .unregister_callback = modterm_raw_unregister_callback,
};

static struct device_driver_ops termapp_driver_ops = {
    .probe = modterm_raw_probe,
    .type_ops = &modterm_raw_type_ops,
};

struct device_driver mods_raw_termapp_driver = {
    .type = DEVICE_TYPE_RAW_HW,
    .name = "mods_raw_termapp",
    .desc = "Termapp Raw Interface",
    .ops = &termapp_driver_ops,
};
