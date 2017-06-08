/*
 * Copyright (c) 2017 Motorola Mobility, LLC.
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

#include <debug.h>
#include <errno.h>
#include <reglog.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>

#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/i2c.h>
#include <nuttx/lib.h>
#include <nuttx/kmalloc.h>
#include <nuttx/pthread.h>
#include <nuttx/device.h>
#include <nuttx/device_sensors_ext.h>
#include <nuttx/greybus/debug.h>
#include <nuttx/power/pm.h>
#include <nuttx/rtc.h>
#include <nuttx/time.h>
#include <nuttx/util.h>

#include <arch/byteorder.h>

#define TMP007_DEVICE_NAME  "TMP007"
#define TMP007_VENDOR_NAME  "TI"

#define TMP007_REG_DEVICE_TEMP_RESULT (0x01)
#define TMP007_REG_CONFIG (0x02)
    #define TMP007_REG_CONFIG_SW_RESET (0x8000)
#define TMP007_REG_OBJ_TEMP_RESULT (0x03)
    #define TMP007_REG_OBJ_TEMP_RESULT_INVALID (0x0001)
#define TMP007_REG_MANUFACTURER_ID (0x1e)
#define TMP007_REG_DEVICE_ID (0x1f)

struct tmp007_event {
    uint16_t time_delta;
    int32_t value;
} __packed;

struct tmp007_info {
    struct i2c_dev_s *i2c_dev;
    pthread_t thread;

    struct report_info_data *rinfo_data;
    sensors_ext_event_callback callback;
    size_t payload_size;

    uint32_t sampling_period_us;
    uint32_t max_report_latency_us;

    /* event_times and event_values are separate arrays to improve array packing. */
    uint64_t event_times[CONFIG_SENSOR_TMP007_BATCH_SIZE];
    int16_t event_values[CONFIG_SENSOR_TMP007_BATCH_SIZE];
    size_t event_count;

    int opened:1;
    int running:1;
    uint8_t sensor_id;
};

/* Globals */
static struct tmp007_info *g_tmp007;
static pthread_mutex_t g_tmp007_mutex = PTHREAD_MUTEX_INITIALIZER;

#define TMP007_STAMP() \
do { \
    vdbg("%d\n", __LINE__); \
} while (0)

#define TMP007_LOCK() \
do { \
    /*vdbg("LOCK: %d\n", __LINE__);*/ \
    pthread_mutex_lock(&g_tmp007_mutex); \
    /*vdbg("LOCKED: %d\n", __LINE__);*/ \
} while (0)

#define TMP007_UNLOCK() \
do { \
    /*vdbg("UNLOCK: %d\n", __LINE__);*/ \
    pthread_mutex_unlock(&g_tmp007_mutex); \
    /*vdbg("UNLOCKED: %d\n", __LINE__);*/ \
} while (0)

/* I2C */
int tmp007_i2c_open(struct tmp007_info *info)
{
    vdbg("info=%p\n", info);

    info->i2c_dev = up_i2cinitialize(CONFIG_SENSOR_TMP007_I2C_BUS);
    if (!info->i2c_dev) {
        vdbg("ERROR: Failed to initialize i2c device");
        return -EIO;
    }

    I2C_SETFREQUENCY(info->i2c_dev, CONFIG_SENSOR_TMP007_I2C_BUS_SPEED);
    I2C_SETADDRESS(info->i2c_dev, CONFIG_SENSOR_TMP007_I2C_ADDRESS, 7);

    return OK;
}

static int tmp007_i2c_close(struct tmp007_info *info)
{
    vdbg("info=%p\n", info);

    if (info->i2c_dev) {
        (void)up_i2cuninitialize(info->i2c_dev);
        info->i2c_dev = NULL;
    }

    return OK;
}

static int tmp007_i2c_reg16_read(struct tmp007_info *info, uint8_t dev_addr,
                                            uint8_t reg_addr, uint16_t *value)
{
    int ret;
    struct i2c_msg_s msgs[2];

    msgs[0].addr   = dev_addr;
    msgs[0].flags  = 0;
    msgs[0].buffer = &reg_addr;
    msgs[0].length = sizeof(reg_addr);

    msgs[1].addr   = dev_addr;
    msgs[1].flags  = I2C_M_READ;
    msgs[1].buffer = (uint8_t *)value;
    msgs[1].length = sizeof(*value);

    ret = I2C_TRANSFER(info->i2c_dev, msgs, ARRAY_SIZE(msgs));
    if (ret == OK) {
        *value = be16_to_cpu(*value);
    }

    vdbg("info=%p, addr=0x%x, reg_addr=0x%x, value=0x%x\n",
            info, dev_addr, reg_addr, *value);

    return ret;
}

static int tmp007_i2c_reg16_write(struct tmp007_info *info, uint16_t dev_addr,
                                            uint8_t reg_addr, uint16_t value)
{
    int result;
    struct i2c_msg_s msgs[1];
    uint8_t wbuf[3] = { reg_addr };

    vdbg("info=%p, addr=0x%x, reg_addr=0x%x, value=0x%x\n",
            info, dev_addr, reg_addr, value);

    value = cpu_to_be16(value);
    wbuf[1] = value & 0xff;
    wbuf[2] = value >> 8;

    msgs[0].addr   = dev_addr;
    msgs[0].flags  = 0;
    msgs[0].buffer = wbuf;
    msgs[0].length = ARRAY_SIZE(wbuf);

    result = I2C_TRANSFER(info->i2c_dev, msgs, ARRAY_SIZE(msgs));
    return result == ARRAY_SIZE(msgs) ? OK : -EIO;
}

static int tmp007_report_alloc_locked(struct tmp007_info *info)
{
    vdbg("info=%p\n", info);

    info->payload_size = ((sizeof(struct report_info) +
            CONFIG_SENSOR_TMP007_BATCH_SIZE * sizeof(struct tmp007_event)));

    info->rinfo_data = zalloc(sizeof(struct report_info_data) + info->payload_size);
    if (!info->rinfo_data) {
        return -ENOMEM;
    }

    return OK;
}

static void tmp007_report_free_locked(struct tmp007_info *info)
{
    vdbg("info=%p\n", info);

    if (info->rinfo_data) {
        free(info->rinfo_data);
        info->rinfo_data = NULL;
    }
}

static int tmp007_report_locked(struct tmp007_info *info, uint8_t flags)
{
    struct report_info *rinfo;
    struct tmp007_event *dst;
    struct timespec ts;
    size_t i;

    vdbg("info=%p, flags=%x\n", info, flags);

    up_rtc_gettime(&ts);
    vdbg("report at %u.%03u\n", ts.tv_sec, (ts.tv_nsec / 1000000));

    info->rinfo_data->num_sensors_reporting = 1;

    rinfo = info->rinfo_data->reportinfo;
    rinfo->id = info->sensor_id;
    rinfo->reference_time = timespec_to_nsec(&ts);
    rinfo->flags = flags;
    rinfo->readings = info->event_count;

    dst = (struct tmp007_event *)&rinfo->data_payload[0];

    for (i = 0; i < info->event_count; i++) {
        // FIXME: handle wrap when subtracting times.
        dst[i].time_delta = ((rinfo->reference_time - info->event_times[i]) / 1000);
        dst[i].value = info->event_values[i];
        vdbg("0x%x, %d\n", dst[i].value, dst[i].value);

#if CONFIG_DEBUG_VERBOSE
        {
            int64_t t0 = dst[i].value;
            t0 *= 3125;
            t0 /= 100000;

            vdbg("%d C at %u us\n", ((int32_t)t0), dst[i].time_delta);
        }
#endif
    }

    if (info->callback) {
        info->callback(info->sensor_id, info->rinfo_data, info->payload_size);
    }

    return OK;
}

/* Thread */
static void *tmp007_thread(void *arg)
{
    struct tmp007_info *info = arg;
    uint16_t manufacturer, device;
    int ret;

    vdbg("thread started: info=%p\n", info);

    TMP007_LOCK();
    tmp007_i2c_open(info);

    tmp007_i2c_reg16_write(info, CONFIG_SENSOR_TMP007_I2C_ADDRESS,
        TMP007_REG_CONFIG, TMP007_REG_CONFIG_SW_RESET);

    TMP007_UNLOCK();

    /* Give the sensor a chance to reset. */
    usleep(250*1000);

    TMP007_LOCK();

    tmp007_i2c_reg16_read(info, CONFIG_SENSOR_TMP007_I2C_ADDRESS,
        TMP007_REG_MANUFACTURER_ID, &manufacturer);
    tmp007_i2c_reg16_read(info, CONFIG_SENSOR_TMP007_I2C_ADDRESS,
        TMP007_REG_DEVICE_ID, &device);
    vdbg("manufacturer=0x%04x, device=0x%04x\n", manufacturer, device);

    TMP007_UNLOCK();

    while (1) {
        struct timespec ts;
        int16_t value;

        vdbg("start loop\n");

        TMP007_LOCK();

        if (!info->running) {
            vdbg("thread stopping\n");
            ret = OK;
            TMP007_UNLOCK();
            break;
        }

        ret = tmp007_i2c_reg16_read(info, CONFIG_SENSOR_TMP007_I2C_ADDRESS,
                            TMP007_REG_OBJ_TEMP_RESULT, (uint16_t *)&value);
        if (ret == OK && ((value & TMP007_REG_OBJ_TEMP_RESULT_INVALID) == 0)) {
            /* The temperature is in bits 16:2. */
            value >>= 2;

            up_rtc_gettime(&ts);
            info->event_times[info->event_count] = timespec_to_nsec(&ts);
            info->event_values[info->event_count] = value;
            info->event_count++;

            if (info->event_count >= CONFIG_SENSOR_TMP007_BATCH_SIZE) {
                tmp007_report_locked(info, 0 /* flags */);
                info->event_count = 0;
            }
        }

        TMP007_UNLOCK();

        vdbg("sampling_period_us=%d\n", info->sampling_period_us);
        usleep(info->sampling_period_us);

        vdbg("end loop\n");
    }

    TMP007_LOCK();
    tmp007_i2c_close(info);
    TMP007_UNLOCK();

    vdbg("thread done\n");
    return (void *)ret;
}

static int tmp007_start_thread_locked(struct tmp007_info *info)
{
    int ret;

    vdbg("info=%p\n", info);

    if (info->thread) {
        vdbg("thread already started\n");
        return OK;
    }

    info->running = 1;

    ret = pthread_create(&info->thread, NULL, tmp007_thread, info);
    if (ret) {
        vdbg("ERROR: Failed to create thread: %s.\n", strerror(errno));
        info->thread = 0;
    }

    return ret;
}

static int tmp007_stop_thread_locked(struct tmp007_info *info)
{
    int ret;
    pthread_addr_t join_value;

    vdbg("info=%p\n", info);

    if (!info->thread) {
        vdbg("thread not started\n");
        return OK;
    }

    info->running = 0;

    /* Unlock the mutex so the thread can complete and join. */
    TMP007_UNLOCK();

    /* The thread might be sleeping, give it enough time to wake up first. */
    usleep(info->sampling_period_us);

    ret = pthread_join(info->thread, &join_value);
    vdbg("thread=%p, start=%p, join_value=%p, ret=%d, errno=%d\n",
            info->thread, tmp007_thread, join_value, ret, errno);
    if (ret != ESRCH) {
        ret = pthread_kill(info->thread, 9);
        vdbg("kill ret=%d, errno=%d\n", ret, errno);
    }

    /* Relock the thread before returning. */
    TMP007_LOCK();

    info->thread = 0;

    /* This might not have been closed if the thread was killed. */
    tmp007_i2c_close(info);

    return ret;
}

/* Ops */
static int tmp007_op_get_sensor_info(struct device *dev, uint8_t sensor_id,
                                                    struct sensor_info *sinfo)
{
    struct tmp007_info *info;
    int ret;

    vdbg("dev=%p, sensor_id=%d\n", dev, sensor_id);

    if (!dev) {
        return -EINVAL;
    }

    info = device_get_private(dev);
    if (!info) {
        return -ENOENT;
    }

    TMP007_LOCK();

    sinfo->type = AMBIENT_TEMPERATURE_ID;

    sinfo->version = 1;

    sinfo->name_len = sizeof(TMP007_DEVICE_NAME);
    memcpy(sinfo->name, TMP007_DEVICE_NAME, sinfo->name_len);

    sinfo->vendor_len = sizeof(TMP007_VENDOR_NAME);
    memcpy(sinfo->vendor, TMP007_VENDOR_NAME, sinfo->vendor_len);

    sinfo->string_type_len = 0; /* Skip string-type if type is set. */

    sinfo->channels = 1;
    sinfo->max_range = 1000; /* 0 - 65 C */
    sinfo->resolution = 1; /* 0.03125 C */

    sinfo->min_delay = 100*1000; /* us */
    sinfo->max_delay = 10*1000*1000; /* us */

    sinfo->fifo_mec = 0; /* FIFO max-event count */
    sinfo->fifo_rec = 0; /* FIFO reserved-event count */

    sinfo->flags = SENSOR_EXT_FLAG_CONTINUOUS_MODE;

    /* Read as 0.03125 C, convert to C. */
    sinfo->scale_int = 0;
    sinfo->offset_int = 0;
    sinfo->scale_nano = 31250000;
    sinfo->offset_nano = 0;

    sinfo->power = 0; /* Power is 270 uA ~= 0 mA. */

    ret = OK;

    TMP007_UNLOCK();

    return ret;
}

static int tmp007_op_start_reporting(struct device *dev, uint8_t sensor_id,
                    uint64_t sampling_period_ns, uint64_t max_report_latency_ns)
{
    struct tmp007_info *info;
    int ret;

    vdbg("dev=%p, sensor_id=%d, sampling_period_ns=%lld, max_report_latency_ns=%lld\n",
            dev, sensor_id, sampling_period_ns, max_report_latency_ns);

    if (!dev) {
        return -EINVAL;
    }

    info = device_get_private(dev);
    if (!info) {
        return -ENOENT;
    }

    TMP007_LOCK();

    info->sampling_period_us = (uint32_t)(sampling_period_ns / 1000);
    info->max_report_latency_us = (uint32_t)(max_report_latency_ns / 1000);
    tmp007_start_thread_locked(info);

    ret = OK;

    TMP007_UNLOCK();

    return ret;
}

static int tmp007_op_stop_reporting(struct device *dev, uint8_t sensor_id)
{
    struct tmp007_info *info;
    int ret;

    vdbg("dev=%p, sensor_id=%d\n", dev, sensor_id);

    if (!dev) {
        return -EINVAL;
    }

    info = device_get_private(dev);
    if (!info) {
        return -ENOENT;
    }

    TMP007_LOCK();

    tmp007_stop_thread_locked(info);

    ret = OK;

    TMP007_UNLOCK();

    return ret;
}

static int tmp007_op_flush(struct device *dev, uint8_t sensor_id)
{
    struct tmp007_info *info;
    int ret;

    vdbg("dev=%p, sensor_id=%d\n", dev, sensor_id);

    if (!dev) {
        return -EINVAL;
    }

    info = device_get_private(dev);
    if (!info) {
        return -ENOENT;
    }

    TMP007_LOCK();

    ret = tmp007_report_locked(info,
            REPORT_INFO_FLAG_FLUSHING | REPORT_INFO_FLAG_FLUSH_COMPLETE);

    TMP007_UNLOCK();

    return ret;
}

static int tmp007_op_register_callback(struct device *dev,
                                       uint8_t sensor_id,
                                       sensors_ext_event_callback callback)
{
    struct tmp007_info *info;
    int ret;

    vdbg("dev=%p, sensor_id=%d\n", dev, sensor_id);

    if (!dev) {
        return -EINVAL;
    }

    info = device_get_private(dev);
    if (!info) {
        return -ENOENT;
    }

    TMP007_LOCK();

    info->callback = callback;
    info->sensor_id = sensor_id;

    ret = OK;

    TMP007_UNLOCK();

    return ret;
}

#if CONFIG_PM
static int pm_prepare(struct pm_callback_s *cb, enum pm_state_e state)
{
    struct tmp007_info *info = g_tmp007;
    int ret;

    TMP007_LOCK();

    /* Prevent PM sleep if any sensors are running. */
    ret = ((state >= PM_IDLE) && info->running) ? -EIO : OK;

    TMP007_UNLOCK();

    return ret;
}

static struct pm_callback_s pm_callback =
{
    .prepare = pm_prepare,
};
#endif

static int tmp007_dev_open(struct device *dev)
{
    struct tmp007_info *info;
    int ret;

    vdbg("dev=%p\n", dev);

    if (!dev) {
        return -EINVAL;
    }

    info = device_get_private(dev);
    if (!info) {
        return -ENOENT;
    }

    TMP007_LOCK();

    if (info->opened) {
        vdbg("ERROR: Already opened.\n");
        ret = -EINVAL;
        goto error;
    }

    memset(info, 0, sizeof(*info));
    info->opened = 1;

    ret = tmp007_report_alloc_locked(info);
    if (ret) {
        goto error;
    }

    ret = OK;

error:
    TMP007_UNLOCK();

    return ret;
}

static void tmp007_dev_close(struct device *dev)
{
    struct tmp007_info *info;

    vdbg("dev=%p\n", dev);

    if (!dev) {
        return;
    }

    info = device_get_private(dev);
    if (!info) {
        return;
    }

    TMP007_LOCK();

    if (!info->opened) {
        vdbg("ERROR: Not opened.\n");
        goto error;
    }

    info->opened = 0;

    tmp007_stop_thread_locked(info);

    tmp007_report_free_locked(info);

error:
    TMP007_UNLOCK();
}

static int tmp007_dev_probe(struct device *dev)
{
    struct tmp007_info *info = g_tmp007;
    int ret;

    vdbg("dev=%p\n", dev);

    if (!dev) {
        return -EINVAL;
    }

    TMP007_LOCK();

    if (info) {
        ret = -EINVAL;
        goto error;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        ret = -ENOMEM;
        goto error;
    }

#if CONFIG_PM
    if (pm_register(&pm_callback) != OK) {
        vdbg("ERROR: Failed register to power management!\n");
    }
#endif

    g_tmp007 = info;

    device_set_private(dev, info);

    ret = OK;

error:
    TMP007_UNLOCK();

    return ret;
}

static void tmp007_dev_remove(struct device *dev)
{
    struct tmp007_info *info = NULL;

    vdbg("dev=%p\n", dev);

    if (!dev) {
        return;
    }

    info = device_get_private(dev);
    if (!info) {
        return;
    }

    TMP007_LOCK();

    device_set_private(dev, NULL);

    tmp007_stop_thread_locked(info);

    free(info);
    g_tmp007 = NULL;

    TMP007_UNLOCK();
}

static struct device_sensors_ext_type_ops tmp007_type_ops = {
    .get_sensor_info = tmp007_op_get_sensor_info,
    .start_reporting = tmp007_op_start_reporting,
    .flush = tmp007_op_flush,
    .stop_reporting = tmp007_op_stop_reporting,
    .register_callback = tmp007_op_register_callback,
};

static struct device_driver_ops tmp007_driver_ops = {
    .probe    = tmp007_dev_probe,
    .remove   = tmp007_dev_remove,
    .open     = tmp007_dev_open,
    .close    = tmp007_dev_close,
    .type_ops = &tmp007_type_ops,
};

const struct device_driver sensor_tmp007_driver = {
    .type   = DEVICE_TYPE_SENSORS_HW,
    .name   = "TMP007",
    .desc   = "TMP007",
    .ops    = &tmp007_driver_ops,
};
