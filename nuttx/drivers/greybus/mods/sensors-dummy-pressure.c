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

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <arch/atomic.h>
#include <nuttx/clock.h>
#include <nuttx/lib.h>
#include <nuttx/util.h>
#include <nuttx/kmalloc.h>
#include <nuttx/pthread.h>
#include <nuttx/device.h>
#include <nuttx/device_sensors_ext.h>
#include <nuttx/greybus/debug.h>
#include <nuttx/greybus/mods-ctrl.h>
#include <nuttx/power/pm.h>
#include <nuttx/rtc.h>
#include <nuttx/time.h>

#define SENSOR_PRESSURE_FLAG_OPEN       BIT(0)

#define PRESSURE_DEVICE_NAME  "Barometer"
#define PRESSURE_VENDOR_NAME  "Moto"
#define PRESSURE_VERSION      3
#define PRESSURE_SAMPLE_RATE  100
#define PRESSURE_SCALE_INT    1
#define PRESSURE_SCALE_NANO   3
#define PRESSURE_OFFSET_INT    0
#define PRESSURE_OFFSET_NANO   4
#define PRESSURE_CHANNEL_SIZE  1
#define PRESSURE_MAX_RANGE     1024 * 16
#define PRESSURE_MIN_DELAY     20000
#define PRESSURE_MAX_DELAY     1000000
#define PRESSURE_FIFO_REC     0
#define PRESSURE_FIFO_MEC     0
#define PRESSURE_FLAGS        SENSOR_EXT_FLAG_CONTINUOUS_MODE
#define REPORTING_SENSORS   1

struct sensor_pressure_info {
    struct device *dev;
    uint32_t flags;
    sensors_ext_event_callback callback;
    uint8_t sensor_id;
    uint64_t max_report_latency;
    int latency_ms;
    pthread_t tx_thread;
    pthread_mutex_t run_mutex;
    bool should_run;
    uint32_t clk_cntr;
};

/*
 * event fields for pressure sensor in bytes
 * [ time_delta (2) | data_value x (4) | data_value y (4) |data_value z (4) ]
*/
struct sensor_event_data {
    uint16_t    time_delta;
    uint32_t    data_value[PRESSURE_CHANNEL_SIZE];
}__packed;

#define BATCH_PROCESS_ENABLED

#ifdef BATCH_PROCESS_ENABLED
#define PRESSURE_READING_NUM 2
#else
#define PRESSURE_READING_NUM 1
#endif
#define TX_PROCESSING_DELAY     1 /* processing and tx delay (mS)*/

static int data = 1000;
static atomic_t txn;       /* Flag to indicate reporting in progress */

static void *sensor_tx_thread(void *arg)
{
    struct sensor_pressure_info *info;
    struct report_info *rinfo;
    struct report_info_data *rinfo_data;
    struct sensor_event_data *event_data;
    struct timespec ts;
    uint16_t payload_size;
    uint16_t readings_count = PRESSURE_READING_NUM;
    uint32_t cntr;

    info = arg;
#ifdef BATCH_PROCESS_ENABLED
    if (info->max_report_latency > 0) {
        readings_count = PRESSURE_READING_NUM;
    } else {
        readings_count = 1;
    }
#endif
    payload_size = (readings_count * sizeof(struct sensor_event_data))
                    + (REPORTING_SENSORS * sizeof(struct report_info));

    rinfo_data = malloc(sizeof(struct report_info_data) + payload_size);
    if (!rinfo_data)
        return (void *)-ENOMEM;

    while(1) {
        memset(rinfo_data, 0, sizeof(struct report_info_data) + payload_size);

        if (info->callback) {
            rinfo_data->num_sensors_reporting = REPORTING_SENSORS;
            rinfo = rinfo_data->reportinfo;
            rinfo->id = info->sensor_id;
            rinfo->flags = 0;
            event_data = (struct sensor_event_data *)&rinfo->data_payload[0];
            up_rtc_gettime(&ts);
            rinfo->reference_time = timespec_to_nsec(&ts);
            rinfo->readings = readings_count;
#ifdef BATCH_PROCESS_ENABLED
            /*
             * Batch sensor data values and its time_deltas
             * until max fifo event count
            */
            if (info->max_report_latency > 0) {
                uint32_t time_delta;
                /* Multiple sensor event data */
                for (time_delta = 0; time_delta <= 2000; time_delta += 2000) {
                    cntr = mods_control_get_rtc_clock_counter();
                    if (cntr != info->clk_cntr) {
                        info->clk_cntr = cntr;
                        if (time_delta > 0) {
                            /* clock changed between events send what we have */
                            goto send_data;
                        }
                    }
                    event_data->time_delta = time_delta;
                    event_data->data_value[0] = data++;  /* read data */
                    event_data++;
                }
            } else {
                /* Single sensor event data */
                cntr = mods_control_get_rtc_clock_counter();
                if (cntr != info->clk_cntr)
                    info->clk_cntr = cntr;
                event_data->time_delta = 0;
                event_data->data_value[0] = data++;
            }
send_data:
#else
            /* Single sensor event data */
            cntr = mods_control_get_rtc_clock_counter();
            if (cntr != info->clk_cntr)
                info->clk_cntr = cntr;
            event_data->num_sensors_reporting = 1;
            event_data->time_delta = 0;
            event_data->data_value[0] = data++;
#endif
            gb_debug("report sensor: %d\n", rinfo->id);
            info->callback(info->sensor_id, rinfo_data, payload_size);
        }

        if (!info->should_run)
            break;

        up_mdelay(info->latency_ms);
    }

    free(rinfo_data);

    return NULL;
}

static int sensor_pressure_op_get_sensor_info(struct device *dev,
    uint8_t sensor_id, struct sensor_info *sinfo)
{
    struct sensor_pressure_info *info = NULL;

    gb_debug("%s: dummy %d\n",__func__, sensor_id);
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);
    info->sensor_id = sensor_id;

    sinfo->version = PRESSURE_VERSION;
    sinfo->max_range = PRESSURE_MAX_RANGE;
    sinfo->resolution = 1;
    sinfo->min_delay = PRESSURE_MIN_DELAY;
    sinfo->max_delay = PRESSURE_MAX_DELAY;
#ifdef BATCH_PROCESS_ENABLED
    /*
     * Example to calculate number of sensor events that can be batched in the
     * payload. Each event consists of time_delta and sensor data fields.
     * Reading size provides the number of sensor data fields reported.
    */
    sinfo->fifo_rec = PRESSURE_READING_NUM;
    sinfo->fifo_mec = sinfo->fifo_rec;
#else
    sinfo->fifo_rec = PRESSURE_FIFO_REC;
    sinfo->fifo_mec = PRESSURE_FIFO_MEC;
#endif
    sinfo->flags = PRESSURE_FLAGS;
    sinfo->scale_int = PRESSURE_SCALE_INT;
    sinfo->scale_nano = PRESSURE_SCALE_NANO;
    sinfo->offset_int = PRESSURE_OFFSET_INT;
    sinfo->offset_nano = PRESSURE_OFFSET_NANO;
    sinfo->channels = PRESSURE_CHANNEL_SIZE;
    sinfo->type = PRESSURE_ID;
    sinfo->name_len = sizeof(PRESSURE_DEVICE_NAME);
    memcpy(&sinfo->name[0], PRESSURE_DEVICE_NAME, sinfo->name_len);
    sinfo->vendor_len = sizeof(PRESSURE_VENDOR_NAME);
    memcpy(&sinfo->vendor[0], PRESSURE_VENDOR_NAME, sinfo->vendor_len);
    sinfo->string_type_len = 0;

    return OK;
}

static int sensor_pressure_op_start_reporting(struct device *dev,
    uint8_t sensor_id, uint64_t sampling_period, uint64_t max_report_latency)
{
    struct sensor_pressure_info *info = NULL;
    int ret;

    gb_debug("%s: %d\n",__func__, sensor_id);

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    info->max_report_latency = max_report_latency;
    info->latency_ms = sampling_period/1000000;
    if (info->latency_ms > TX_PROCESSING_DELAY) {
        info->latency_ms -= TX_PROCESSING_DELAY;
    }
    data = 1000;
    info->should_run = true;

    if (!info->tx_thread) {
        ret = pthread_create(&info->tx_thread, NULL, sensor_tx_thread, info);
        if (ret) {
            lldbg("ERROR: Failed to create tx thread: %s.\n", strerror(errno));
            info->tx_thread = 0;
            info->should_run = false;
            return ret;
        }
    }
    atomic_inc(&txn);

    return OK;
}

static int sensor_pressure_op_flush(struct device *dev, uint8_t id)
{
    struct sensor_event_data *event_data;
    struct sensor_pressure_info *info;
    struct report_info_data *rinfo_data;
    struct report_info *rinfo;
    struct timespec ts;
    uint16_t payload_size;
    uint16_t readings_count = PRESSURE_READING_NUM;

    gb_debug("%s:\n", __func__);

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

#ifdef BATCH_PROCESS_ENABLED
    if (info->max_report_latency > 0) {
        readings_count = PRESSURE_READING_NUM;
    } else {
        readings_count = 1;
    }
#endif
    payload_size = (readings_count * sizeof(struct sensor_event_data))
                    + (REPORTING_SENSORS * sizeof(struct report_info));

    if (info->callback) {
        rinfo_data = malloc(sizeof(struct report_info) + payload_size);
        if (!rinfo_data)
            return -ENOMEM;

        rinfo_data->num_sensors_reporting = REPORTING_SENSORS;
        rinfo = rinfo_data->reportinfo;
        rinfo->id = info->sensor_id;
        event_data = (struct sensor_event_data *)&rinfo->data_payload[0];

        up_rtc_gettime(&ts);
        rinfo->reference_time = timespec_to_nsec(&ts);
        gb_debug("[%u.%03u]\n", ts.tv_sec, (ts.tv_nsec / 1000000));
        rinfo->flags = REPORT_INFO_FLAG_FLUSHING | REPORT_INFO_FLAG_FLUSH_COMPLETE;
        rinfo->readings = readings_count;
#ifdef BATCH_PROCESS_ENABLED
        /*
         * Batch sensor data values and its time_deltas
         * until max fifo event count
        */
        if (info->max_report_latency > 0) {
            /* Multiple sensor event data */
            event_data->time_delta = 0;
            event_data->data_value[0] = data++;

            event_data++;
            event_data->time_delta = 2000;
            event_data->data_value[0] = data++;
        } else {
            /* Single sensor event data */
            event_data->time_delta = 0;
            event_data->data_value[0] = data++;
        }
#else
        /* Single sensor event data */
        event_data->time_delta = 0;
        event_data->data_value[0] = data++;
#endif
        info->callback(info->sensor_id, rinfo_data, payload_size);

        free(rinfo_data);
    }

    return OK;
}

static void sensor_pressure_kill_pthread(struct sensor_pressure_info *info)
{
    info->should_run = false;

    if (info->tx_thread) {
        while(pthread_mutex_trylock(&info->run_mutex)) {
            pthread_kill(info->tx_thread, 9);
            usleep(10000);
        }
        pthread_mutex_unlock(&info->run_mutex);
        pthread_join(info->tx_thread, NULL);
        info->tx_thread = 0;
    }
}

static int sensor_pressure_op_stop_reporting(struct device *dev,
    uint8_t sensor_id)
{
    struct sensor_pressure_info *info = NULL;

    gb_debug("%s:\n",__func__);

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);
    sensor_pressure_kill_pthread(info);

    atomic_dec(&txn);

    return OK;
}

static int sensor_pressure_op_register_callback(struct device *dev,
                                        uint8_t se_id,
                                        sensors_ext_event_callback callback)
{
    struct sensor_pressure_info *info = NULL;

    gb_debug("%s:\n", __func__);
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    info->callback = callback;
    info->sensor_id = se_id;

    return 0;
}

#ifdef CONFIG_PM
static int pm_prepare(struct pm_callback_s *cb, enum pm_state_e state)
{
    /*
    * Do not allow IDLE when work is scheduled
    */
    if ((state >= PM_IDLE) && (atomic_get(&txn)))
        return -EIO;

    return OK;
}

static struct pm_callback_s pm_callback =
{
  .prepare = pm_prepare,
};
#endif

static int sensor_pressure_dev_open(struct device *dev)
{
    struct sensor_pressure_info *info = NULL;

    gb_debug("%s:\n", __func__);
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (info->flags & SENSOR_PRESSURE_FLAG_OPEN) {
        return -EBUSY;
    }

    info->flags |= SENSOR_PRESSURE_FLAG_OPEN;

    return 0;
}

static void sensor_pressure_dev_close(struct device *dev)
{
    struct sensor_pressure_info *info = NULL;

    gb_debug("%s:\n",__func__);

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);
    sensor_pressure_kill_pthread(info);

    if (!(info->flags & SENSOR_PRESSURE_FLAG_OPEN)) {
        return;
    }

    info->flags &= ~SENSOR_PRESSURE_FLAG_OPEN;
}

static int sensor_pressure_dev_probe(struct device *dev)
{
    struct sensor_pressure_info *info;

    gb_debug("%s:\n",__func__);
    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    info->dev = dev;
    device_set_private(dev, info);
    info->clk_cntr = mods_control_get_rtc_clock_counter();

    atomic_init(&txn, 0);
    pthread_mutex_init(&info->run_mutex, NULL);

#ifdef CONFIG_PM
    if (pm_register(&pm_callback) != OK)
    {
        dbg("Failed register to power management!\n");
    }
#endif

    return 0;
}

static void sensor_pressure_dev_remove(struct device *dev)
{
    struct sensor_pressure_info *info = NULL;

    gb_debug("%s:\n",__func__);

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);
    sensor_pressure_kill_pthread(info);

    if (info->flags & SENSOR_PRESSURE_FLAG_OPEN) {
        sensor_pressure_dev_close(dev);
    }
    info->flags = 0;

    free(info);
    device_set_private(dev, NULL);
}

static struct device_sensors_ext_type_ops sensor_pressure_type_ops = {
    .get_sensor_info = sensor_pressure_op_get_sensor_info,
    .start_reporting = sensor_pressure_op_start_reporting,
    .flush = sensor_pressure_op_flush,
    .stop_reporting = sensor_pressure_op_stop_reporting,
    .register_callback = sensor_pressure_op_register_callback,
};

static struct device_driver_ops sensor_pressure_driver_ops = {
    .probe    = sensor_pressure_dev_probe,
    .remove   = sensor_pressure_dev_remove,
    .open     = sensor_pressure_dev_open,
    .close    = sensor_pressure_dev_close,
    .type_ops = &sensor_pressure_type_ops,
};

const struct device_driver sensor_dummy_pressure_driver = {
    .type   = DEVICE_TYPE_SENSORS_HW,
    .name   = "sensors_ext_dummy_pressure",
    .desc   = "DUMMY PRESSURE SENSOR",
    .ops    = &sensor_pressure_driver_ops,
};
