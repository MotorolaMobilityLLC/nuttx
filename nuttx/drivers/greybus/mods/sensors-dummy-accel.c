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
#include <nuttx/clock.h>
#include <nuttx/lib.h>
#include <nuttx/util.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/device.h>
#include <nuttx/device_sensors_ext.h>
#include <nuttx/greybus/debug.h>
#include <nuttx/rtc.h>
#include <nuttx/time.h>


#define SENSOR_ACCEL_FLAG_OPEN       BIT(0)

#define ACCEL_DEVICE_NAME  "ACCEL"
#define ACCEL_SAMPLE_RATE  100
#define ACCEL_SCALE_INT    0
#define ACCEL_SCALE_NANO   976563
#define ACCEL_OFFSET_INT    0
#define ACCEL_OFFSET_NANO   0
#define ACCEL_READING_SIZE  3
#define ACCEL_MAX_RANGE     1024 * 16
#define ACCEL_MIN_DELAY     1000
#define ACCEL_MAX_DELAY     1000000
#define ACCEL_FIFO_REC      0
#define ACCEL_FIFO_MEC      0
#define ACCEL_FLAGS        SENSOR_EXT_FLAG_CONTINUOUS_MODE


struct sensor_accel_info {
    struct device *dev;
    uint32_t flags;
    sensors_ext_event_callback callback;
    struct work_s data_report_work;
    uint8_t sensor_id;
};

/*
 * event fields for accelerometer in bytes
 * [ time_delta (2) | data_value x (4) | data_value y (4) |data_value z (4) ]
*/
struct sensor_event_data {
    uint16_t    time_delta;
    uint32_t    data_value[ACCEL_READING_SIZE];
};

static void accel_data_report_worker(void *arg)
{
    struct sensor_accel_info *info;
    struct report_info *rinfo;
    struct sensor_event_data *event_data;
    struct timespec ts;
    uint64_t time_ns;

    info = arg;
    if (info->callback) {
        rinfo = malloc(sizeof(struct report_info));
        if (!rinfo)
            goto out;
        event_data = (struct sensor_event_data *)&rinfo->data_payload[0];

        up_rtc_gettime(&ts);
        gb_debug("[%u.%03u]\n", ts.tv_sec, (ts.tv_nsec / 1000000));
        time_ns = timespec_to_nsec(&ts);
        gb_debug("Time %u\n", time_ns);
        memcpy((void *)&rinfo->reference_time, &time_ns, sizeof(uint64_t));

#ifdef BATCH_PROCESS_ENABLED
        /*
         * Batch sensor data values and its time_deltas
         * until max fifo event count
        */
#else
        /* Single sensor event data */
        rinfo->readings = 1;
        event_data->time_delta = 0;
        event_data->data_value[0] = ts.tv_sec;
        event_data->data_value[1] = 3;
        event_data->data_value[2] = (uint32_t)ts.tv_nsec;
#endif
        info->callback(info->sensor_id, rinfo);

        free(rinfo);
    }

out:
    gb_debug("report DATA\n");

     /* cancel any work and reset ourselves */
    if (!work_available(&info->data_report_work))
        work_cancel(LPWORK, &info->data_report_work);

    /* if not already scheduled, schedule start */
    if (work_available(&info->data_report_work))
        work_queue(LPWORK, &info->data_report_work, accel_data_report_worker, info, MSEC2TICK(2000));
}

static int sensor_accel_op_get_sensor_info(struct device *dev,
    uint8_t sensor_id, struct sensor_info *sinfo)
{
    gb_debug("%s:\n", __func__);
    sinfo->max_range = ACCEL_MAX_RANGE;
    sinfo->resolution = 1;
    sinfo->min_delay = ACCEL_MIN_DELAY;
    sinfo->max_delay = ACCEL_MAX_DELAY;
#ifdef BATCH_PROCESS_ENABLED
    /*
     * Example to calculate number of sensor events that can be batched in the
     * payload. Each event consists of time_delta and sensor data fields.
     * Reading size provides the number of sensor data fields reported.
    */
    sinfo->fifo_rec = (SENSOR_DATA_PAYLOAD_SIZE / (sizeof(struct sensor_event_data)));
    sinfo->fifo_mec = sinfo->fifo_rec;
#else
    sinfo->fifo_rec = ACCEL_FIFO_REC;
    sinfo->fifo_mec = ACCEL_FIFO_MEC;
#endif
    sinfo->flag = ACCEL_FLAGS;
    sinfo->scale = ACCEL_SCALE_NANO;
    sinfo->offset = ACCEL_OFFSET_NANO;
    sinfo->reading_size = ACCEL_READING_SIZE;
    sinfo->type = ACCELEROMETER_ID;
    memcpy(&sinfo->name[0], ACCEL_DEVICE_NAME, sizeof(ACCEL_DEVICE_NAME));
    return OK;
}

static int sensor_accel_op_start_reporting(struct device *dev,
    uint8_t sensor_id, uint64_t sampling_period, uint64_t max_report_latency)
{
    struct sensor_accel_info *info = NULL;

    gb_debug("%s:\n",__func__);

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    /* cancel any work and reset ourselves */
    if (!work_available(&info->data_report_work))
        work_cancel(LPWORK, &info->data_report_work);

    /* if not already scheduled, schedule start */
    if (work_available(&info->data_report_work))
        work_queue(LPWORK, &info->data_report_work, accel_data_report_worker, info, 0);

    return OK;
}

static int sensor_accel_op_get_report(struct device *dev,
    uint8_t id, struct report_info *rinfo)
{
    struct sensor_event_data *event_data;
    struct timespec ts;
    uint64_t time_ns;
    gb_debug("%s:\n", __func__);

    if (!dev || !rinfo) {
        return -EINVAL;
    }
    rinfo->readings = 1;
    event_data = (struct sensor_event_data *)&rinfo->data_payload[0];
    up_rtc_gettime(&ts);
    gb_debug("[%u.%03u]\n", ts.tv_sec, (ts.tv_nsec / 1000000));
    time_ns = timespec_to_nsec(&ts);
    gb_debug("Time %ju\n", time_ns);
    memcpy((void *)&rinfo->reference_time, &time_ns, sizeof(uint64_t));
    event_data->time_delta = 0;
    event_data->data_value[0] = ts.tv_sec;
    event_data->data_value[1] = 3;
    event_data->data_value[2] = (uint32_t)ts.tv_nsec;

    return OK;
}

static int sensor_accel_op_stop_reporting(struct device *dev,
    uint8_t sensor_id)
{
    struct sensor_accel_info *info = NULL;

    gb_debug("%s:\n",__func__);

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    /* cancel any work and reset ourselves */
    if (!work_available(&info->data_report_work))
        work_cancel(LPWORK, &info->data_report_work);

    return OK;
}

static int sensor_accel_op_register_callback(struct device *dev,
                                        uint8_t se_id,
                                        sensors_ext_event_callback callback)
{
    struct sensor_accel_info *info = NULL;

    gb_debug("%s:\n", __func__);
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    info->callback = callback;
    info->sensor_id = se_id;

    return 0;
}


static int sensor_accel_dev_open(struct device *dev)
{
    struct sensor_accel_info *info = NULL;

    gb_debug("%s:\n", __func__);
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (info->flags & SENSOR_ACCEL_FLAG_OPEN) {
        return -EBUSY;
    }

    info->flags |= SENSOR_ACCEL_FLAG_OPEN;

    return 0;
}

static void sensor_accel_dev_close(struct device *dev)
{
    struct sensor_accel_info *info = NULL;

    gb_debug("%s:\n",__func__);

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    /* cancel any pending events */
    if (!work_available(&info->data_report_work))
        work_cancel(LPWORK, &info->data_report_work);

    if (!(info->flags & SENSOR_ACCEL_FLAG_OPEN)) {
        return;
    }

    info->flags &= ~SENSOR_ACCEL_FLAG_OPEN;
}

static int sensor_accel_dev_probe(struct device *dev)
{
    struct sensor_accel_info *info;

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

    return 0;
}

static void sensor_accel_dev_remove(struct device *dev)
{
    struct sensor_accel_info *info = NULL;

    gb_debug("%s:\n",__func__);

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    if (info->flags & SENSOR_ACCEL_FLAG_OPEN) {
        sensor_accel_dev_close(dev);
    }
    info->flags = 0;

    free(info);
    device_set_private(dev, NULL);
}

static struct device_sensors_ext_type_ops sensor_accel_type_ops = {
    .get_sensor_info = sensor_accel_op_get_sensor_info,
    .start_reporting = sensor_accel_op_start_reporting,
    .get_report = sensor_accel_op_get_report,
    .stop_reporting = sensor_accel_op_stop_reporting,
    .register_callback = sensor_accel_op_register_callback,
};

static struct device_driver_ops sensor_accel_driver_ops = {
    .probe    = sensor_accel_dev_probe,
    .remove   = sensor_accel_dev_remove,
    .open     = sensor_accel_dev_open,
    .close    = sensor_accel_dev_close,
    .type_ops = &sensor_accel_type_ops,
};

const struct device_driver sensor_dummy_accel_driver = {
    .type   = DEVICE_TYPE_SENSORS_HW,
    .name   = "sensors_ext_accel",
    .desc   = "ACCEL SENSOR",
    .ops    = &sensor_accel_driver_ops,
};
