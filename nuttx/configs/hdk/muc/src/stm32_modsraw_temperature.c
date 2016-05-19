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
#include <sys/ioctl.h>
#include <arch/byteorder.h>
#include <arch/board/board.h>

#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/clock.h>
#include <nuttx/gpio.h>
#include <nuttx/greybus/debug.h>
#include <nuttx/device.h>
#include <nuttx/device_raw.h>
#include <nuttx/power/pm.h>
#include <nuttx/time.h>
#include <nuttx/wqueue.h>
#include <arch/board/mods.h>


#include "hdk.h"
#include "aes.h"

#ifdef CONFIG_STM32_ADC1
/* The number of ADC channels in the conversion list */
#define ADC1_NCHANNELS 1

/* Identifying number of each ADC channel
 * Temperature output is available on PC3, which corresponds
 * to ADC1 channel 4 (GPIO_ADC1_IN4).
 */
static const uint8_t  g_chanlist[ADC1_NCHANNELS] = {4};

/* Configurations of pins used by each ADC channel */
static const uint32_t g_pinlist[ADC1_NCHANNELS]  = {GPIO_ADC1_IN4};
#endif


#define TEMP_RAW_NAME     "TEMPERATURE"
#define TEMP_RAW_DRIVER_VERSION   0x01
#define TEMP_RAW_MAX_LATENCY      2000    /* milliSeconds */

/* Command IDs */
#define TEMP_RAW_COMMAND_INVALID  0x00
#define TEMP_RAW_COMMAND_INFO     0x01
#define TEMP_RAW_COMMAND_ON       0x02
#define TEMP_RAW_COMMAND_OFF      0x03
#define TEMP_RAW_COMMAND_DATA     0x04
#define TEMP_RAW_COMMAND_CHALLENGE      0x05
#define TEMP_RAW_COMMAND_CHLGE_RESP     0x06

#define TEMP_RAW_COMMAND_RESP_MASK    0x80

/* AES */
#define TEMP_RAW_AES_KEY            "moto-temp-apk"
#define TEMP_RAW_AES_BLOCK_SIZE     16
#define TEMP_RAW_AES_CLIENT_ADDS    777

/* AES key, encrypted and decrypted data */
struct temp_raw_aes {
    char    key[TEMP_RAW_AES_BLOCK_SIZE];
    uint8_t input[TEMP_RAW_AES_BLOCK_SIZE];
    uint8_t encrypted_data[TEMP_RAW_AES_BLOCK_SIZE];
    uint8_t decrypted_data[TEMP_RAW_AES_BLOCK_SIZE];
};

struct temp_raw_info {
    struct device *gDevice;         /* device handle for this driver */
    uint16_t interval;              /* reporting interval */
    raw_send_callback gCallback;    /* callback to send back messages */
    uint8_t client_verified;        /* flag to indicate client was verified */
    struct temp_raw_aes tr_aes;     /* device aes structure */
};

static struct work_s data_report_work; /* work queue for data reporting */

/*
 * Message structure between this driver and client.
 * |   1-byte   |  1-byte           |   N bytes         |
 * |   CMD ID   |  PAYLOAD SIZE (N) |   PAYLOAD  DATA   |
 */
struct temp_raw_msg {
    uint8_t     cmd_id;
    uint8_t     size;
    uint8_t     payload[];
} __packed;

/* device attributes */
struct temp_raw_attr {
    uint8_t     version;        /* device driver version */
    uint8_t     reserved;       /* reserved */
    uint16_t    max_interval;   /* max report interval device supports */
    uint8_t     name[64];       /* name of the device */
} __packed;

/**
 * Initialize ADC1 interface and register the ADC driver.
 */
static int adc_devinit(void)
{
#ifdef CONFIG_STM32_ADC1
    static bool initialized = false;
    struct adc_dev_s *adc;
    int ret;
    int i;

    /* Check if we have already initialized */
    if (!initialized)
    {
        /* Configure the pins as analog inputs for the selected channels */
        for (i = 0; i < ARRAY_SIZE(g_pinlist); i++)
        {
            stm32_configgpio(g_pinlist[i]);
        }

        /* Call stm32_adcinitialize() to get an instance of the ADC interface */
        adc = stm32_adcinitialize(1, g_chanlist, ARRAY_SIZE(g_chanlist));
        if (adc == NULL)
        {
            adbg("ERROR: Failed to get ADC interface\n");
            return -ENODEV;
        }

        /* Register the ADC driver */
        ret = adc_register(TEMP_RAW_ADC_DEVPATH, adc);
        if (ret < 0)
        {
            adbg("adc_register failed: %d\n", ret);
            return ret;
        }

        /* Now we are initialized */
        adbg("adc initialized: %s\n", TEMP_RAW_ADC_DEVPATH);
        initialized = true;
    }

    return OK;
#else
    return -ENOSYS;
#endif
}

/**
 * Call the base raw device to send our data
 */
static int temp_raw_send(struct device *dev, uint32_t len, uint8_t data[])
{
    struct temp_raw_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return -ENODEV;
    }
    info = device_get_private(dev);

    if (info->gCallback) {
        info->gCallback(info->gDevice, len, data);
    }
    return OK;
}

/**
 * build and send message with payload data.
 */
static int temp_raw_build_send_mesg(struct device *dev, uint8_t cmd_id,
                        uint8_t size, uint8_t *data, uint8_t resp)
{
    int ret = 0;
    uint8_t *resp_msg;
    struct temp_raw_msg  *sresp;
    int resp_size = sizeof(struct temp_raw_msg) + size;

    if (size == 0) return -EINVAL;

    /* allocate memory for response message */
    resp_msg = zalloc(resp_size);
    if (!resp_msg) {
        return -ENOMEM;
    }

    /* fill in the message structure */
    sresp = (struct temp_raw_msg *)resp_msg;
    sresp->cmd_id = cmd_id;
    if (resp)
        sresp->cmd_id |= TEMP_RAW_COMMAND_RESP_MASK;
    sresp->size = size;
    /* copy payload data and send the message */
    memcpy((void *)&sresp->payload[0], data, size);
    ret = temp_raw_send(dev, resp_size, resp_msg);

    free(resp_msg);

    return ret;
}

/**
 * worker function for the scheduled work queue.
 * open ADC device and trigger a sample reading.
 */
static void temp_raw_worker(void *arg)
{
    int ret;
    int adc_fd;
    struct adc_msg_s sample;
    struct temp_raw_info *info = NULL;

    info = arg;

    adc_fd = open(TEMP_RAW_ADC_DEVPATH, O_RDONLY);
    if (adc_fd < 0)
    {
        dbg("open %s failed: %d\n", TEMP_RAW_ADC_DEVPATH, errno);
        goto errout;
    }

    ret = ioctl(adc_fd, ANIOC_TRIGGER, 0);
    if (ret < 0)
    {
        dbg("ANIOC_TRIGGER ioctl failed: %d\n", errno);
    }

    read(adc_fd, &sample, sizeof(sample));
    close(adc_fd);

    temp_raw_build_send_mesg(info->gDevice, TEMP_RAW_COMMAND_DATA,
                sizeof(sample.am_data), (uint8_t *)&sample.am_data, 0);

    /* cancel any work and reset ourselves */
    if (!work_available(&data_report_work))
        work_cancel(LPWORK, &data_report_work);

    /* schedule work */
    work_queue(LPWORK, &data_report_work,
                temp_raw_worker, info, MSEC2TICK(info->interval));
errout:
    return;
}

/**
 * We got data from the device (core) side.
 */
static int temp_raw_recv(struct device *dev, uint32_t len, uint8_t data[])
{
    struct temp_raw_attr sattr;
    struct temp_raw_msg *smsg;
    struct temp_raw_info *info = NULL;
    struct timespec ts;
    uint64_t time_ns;
    int i, retval;

    if (!dev || !device_get_private(dev)) {
        return -ENODEV;
    }
    info = device_get_private(dev);

    smsg = (struct temp_raw_msg *)&data[0];

    if ((len == 0) || (len < (sizeof(struct temp_raw_msg))))
        return -EINVAL;

    switch (smsg->cmd_id) {
        case TEMP_RAW_COMMAND_INFO:
            llvdbg("TEMP_RAW_COMMAND_INFO\n");
            memset(&sattr, 0, sizeof(sattr));

            sattr.version = TEMP_RAW_DRIVER_VERSION;
            memcpy(&sattr.name[0], TEMP_RAW_NAME, sizeof(TEMP_RAW_NAME));
            sattr.max_interval = cpu_to_le16(TEMP_RAW_MAX_LATENCY);

            temp_raw_build_send_mesg(dev, TEMP_RAW_COMMAND_INFO, sizeof(sattr),
                         (uint8_t *)&sattr, 1);

            /*
             * build and send challenge message:
             * 1. Send AES ECB 128-bit encrypted current time in nanoseconds.
             * 2. client decrypts, adds TEMP_RAW_AES_CLIENT_ADDS, encrypts
             *    and sends back the adjusted time.
             * 3. Decrpyt and match the adjusted time. Allow ON operation on
             *    match.
             */
            up_rtc_gettime(&ts);
            time_ns = timespec_to_nsec(&ts);

            /*
             * time_ns is 8 bytes, since AES block size is
             * TEMP_RAW_AES_BLOCK_SIZE, pad the data for
             * AES/ECB/PKCS5Padding
             */
            memcpy(&info->tr_aes.input[0], &time_ns, sizeof(time_ns));
            for (i=sizeof(time_ns); i<TEMP_RAW_AES_BLOCK_SIZE; i++)
            {
                info->tr_aes.input[i] = TEMP_RAW_AES_BLOCK_SIZE - sizeof(time_ns);
            }

            /* Encrypt padded time_ns */
            AES128_ECB_encrypt(&info->tr_aes.input[0],
                                (const uint8_t *)&info->tr_aes.key[0],
                                &info->tr_aes.encrypted_data[0]);

            /* send the challenge */
            temp_raw_build_send_mesg(dev, TEMP_RAW_COMMAND_CHALLENGE,
                                sizeof(info->tr_aes.encrypted_data),
                                (uint8_t *)&info->tr_aes.encrypted_data[0], 0);
            break;

        case TEMP_RAW_COMMAND_CHLGE_RESP:
            llvdbg("TEMP_RAW_COMMAND_CHLGE_RESP");
            /* Decrypt the adjusted time data */
            AES128_ECB_decrypt(&smsg->payload[0],
                                (const uint8_t *)&info->tr_aes.key[0],
                                &info->tr_aes.decrypted_data[0]);

            /*
             * compare the sent data with received adjusted data.
             * allow ON operation upon match
             */
            memcpy(&time_ns, &info->tr_aes.input[0], sizeof(time_ns));
            time_ns += TEMP_RAW_AES_CLIENT_ADDS;
            retval = memcmp(&info->tr_aes.decrypted_data[0],
                            &time_ns, sizeof(time_ns));
            if (!retval) {
                info->client_verified = 1;
                /* Initialize ADC */
                adc_devinit();
            } else {
                info->client_verified = 0;
                dbg("Client verification failed: %d\n", retval);
            }

            /* respond with the result */
            temp_raw_build_send_mesg(dev, TEMP_RAW_COMMAND_CHLGE_RESP,
                                        sizeof(retval),
                                        (uint8_t *)&retval, 1);
            break;

        case TEMP_RAW_COMMAND_ON:
            llvdbg("TEMP_RAW_COMMAND_ON %d\n", smsg->size);
            if (info->client_verified == 0)
                break;
            if (smsg->size != sizeof(sattr.max_interval))
                break;

            info->interval = le16_to_cpu(*((uint16_t *)&smsg->payload[0]));
            llvdbg("info->interval: 0x%x\n", info->interval);

            gpio_set_value(GPIO_MODS_DEMO_ENABLE, 1);
            if (info->interval > TEMP_RAW_MAX_LATENCY)
                info->interval = TEMP_RAW_MAX_LATENCY;

            /* cancel any work and reset ourselves */
            if (!work_available(&data_report_work))
                work_cancel(LPWORK, &data_report_work);

            /* schedule work */
            work_queue(LPWORK, &data_report_work,
                        temp_raw_worker, info, 0);

            break;

        case TEMP_RAW_COMMAND_OFF:
            llvdbg("TEMP_RAW_COMMAND_OFF\n");
            /* cancel any work and reset ourselves */
            if (!work_available(&data_report_work))
                work_cancel(LPWORK, &data_report_work);

            gpio_set_value(GPIO_MODS_DEMO_ENABLE, 0);

            break;

        default:
            dbg("Invalid command: %d\n", smsg->cmd_id);
            break;
    }

    return 0;
}

/*
 * Prepare callback from power management.
 * This driver returns non-zero value when data
 * reporting is enabled.
 */
#ifdef CONFIG_PM
static int pm_prepare(struct pm_callback_s *cb, enum pm_state_e state)
{
    /*
    * Do not allow IDLE when work is scheduled
    */
    if ((state >= PM_IDLE) && (!work_available(&data_report_work)))
        return -EIO;

    return OK;
}

static struct pm_callback_s pm_callback =
{
  .prepare = pm_prepare,
};
#endif

/**
 * register the send callback function.
 */
static int temp_raw_register_callback(struct device *dev,
        raw_send_callback callback)
{
    struct temp_raw_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return -ENODEV;
    }
    info = device_get_private(dev);

    info->gCallback = callback;
    return 0;
}

/**
 * unregister the send callback function.
 */
static int temp_raw_unregister_callback(struct device *dev)
{
    struct temp_raw_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return -ENODEV;
    }
    info = device_get_private(dev);

    info->gCallback = NULL;
    return 0;
}

/**
 * probe function called upon device registration.
 */
static int temp_raw_probe(struct device *dev)
{
    struct temp_raw_info *info;

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    info->client_verified = 0;
    info->gDevice = dev;
    device_set_private(dev, info);

    /* Enable temperature device */
    gpio_direction_out(GPIO_MODS_DEMO_ENABLE, 0);

    sprintf(&info->tr_aes.key[0], TEMP_RAW_AES_KEY);

#ifdef CONFIG_PM
    if (pm_register(&pm_callback) != OK)
    {
        dbg("Failed register to power management!\n");
    }
#endif

    llvdbg("Probe complete\n");

    return 0;
}

/**
 * remove function called when device is unregistered.
 */
static void temp_raw_remove(struct device *dev)
{
    struct temp_raw_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    free(info);
    device_set_private(dev, NULL);
}

static struct device_raw_type_ops temp_raw_type_ops = {
    .recv = temp_raw_recv,
    .register_callback = temp_raw_register_callback,
    .unregister_callback = temp_raw_unregister_callback,
};

static struct device_driver_ops temperature_driver_ops = {
    .probe = temp_raw_probe,
    .remove = temp_raw_remove,
    .type_ops = &temp_raw_type_ops,
};

struct device_driver mods_raw_temperature_driver = {
    .type = DEVICE_TYPE_RAW_HW,
    .name = "mods_raw_temperature",
    .desc = "Temperature sensor Raw Interface",
    .ops = &temperature_driver_ops,
};
