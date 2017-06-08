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


#include "bme680.h"
#include "bme680_calculations.h"

#define DEBUG_ENABLE_ALL (0)

#define SENSOR_DEVICE_NAME  "BME680"
#define SENSOR_VENDOR_NAME  "Bosch"

/* Types */
enum {
    SUB_SENSOR_TEMPERATURE = 0,
    SUB_SENSOR_PRESSURE    = 1,
    SUB_SENSOR_HUMIDITY    = 2,
    SUB_SENSOR_GAS         = 3,

    SUB_SENSOR_MAX,
};

struct sub_sensor {
    struct bme680_info *info;
    const struct sub_sensor_ops *ops;
    sensors_ext_event_callback callback;
    int running;
    uint32_t sampling_period_us;
    uint32_t max_report_latency_us;
    size_t sub_id; /* Index into sensors[SUB_SENSOR_MAX]. */
    uint8_t sensor_id; /* Assigned by device manager. */
    size_t payload_size;
    struct report_info_data *report_info_data;
    void *event_data;
};

struct sub_sensor_ops {
    int (*get_sensor_info)(struct sub_sensor *sub_sensor,
        struct sensor_info *sinfo);
    int (*start_reporting)(struct sub_sensor *sub_sensor,
        uint64_t sampling_period_ns, uint64_t max_report_latency_ns);
    int (*stop_reporting)(struct sub_sensor *sub_sensor);
    int (*report)(struct sub_sensor *sub_sensor);
    int (*flush)(struct sub_sensor *sub_sensor);
    int (*alloc_report)(struct sub_sensor *sub_sensor);
    void (*free_report)(struct sub_sensor *sub_sensor);
};

struct bme680_info {
    struct i2c_dev_s *i2c_dev;
    uint8_t *buf;
    size_t buf_len;
    struct bme680_t adapter; /* Adapter to the open-source BME680 driver. */
    int probed; /* Count of number of devices that are probed. */
    int opened; /* Count of number of devices that are opened. */
    int running; /* Count of number of devices that are running. */
    int config_changed; /* True if configuration has changed. */
    pthread_t thread;
    uint32_t latency_us; /* Current thread polling rate. */
    struct sub_sensor sub_sensors[SUB_SENSOR_MAX];
};

/* Globals */
static struct bme680_info *g_bme680;
static pthread_mutex_t g_bme680_mutex = PTHREAD_MUTEX_INITIALIZER;

#define BME680_STAMP() \
do { \
    vdbg("%d\n", __LINE__); \
} while (0)

#define BME680_LOCK() \
do { \
    /*vdbg("LOCK: %d\n", __LINE__);*/ \
    pthread_mutex_lock(&g_bme680_mutex); \
    /*vdbg("LOCKED: %d\n", __LINE__);*/ \
} while (0)

#define BME680_UNLOCK() \
do { \
    /*vdbg("UNLOCK: %d\n", __LINE__);*/ \
    pthread_mutex_unlock(&g_bme680_mutex); \
    /*vdbg("UNLOCKED: %d\n", __LINE__);*/ \
} while (0)

/* TEMPERATURE */
#define TSS_READINGS (1)
#define TSS_NUM_SENSORS_REPORTING (1)

struct tss_event_data {
    uint16_t    time_delta;
    uint32_t    data_value;
} __packed;

static int tss_get_sensor_info(struct sub_sensor *sub_sensor,
                                                    struct sensor_info *sinfo)
{
    vdbg("sub_sensor_id=%d\n", sub_sensor->sensor_id);

    sinfo->type = AMBIENT_TEMPERATURE_ID;

    sinfo->version = 1;

    sinfo->name_len = sizeof(SENSOR_DEVICE_NAME);
    memcpy(sinfo->name, SENSOR_DEVICE_NAME, sinfo->name_len);

    sinfo->vendor_len = sizeof(SENSOR_VENDOR_NAME);
    memcpy(sinfo->vendor, SENSOR_VENDOR_NAME, sinfo->vendor_len);

    sinfo->string_type_len = 0; /* Skip string-type if type is set. */

    sinfo->channels = 1;
    sinfo->max_range = 100; /* 0 - 65 C */
    sinfo->resolution = 1; /* 0.01 */

    sinfo->min_delay = 100*1000; /* us */
    sinfo->max_delay = 10*1000*1000; /* us */

    sinfo->fifo_mec = 0;    /* FIFO max-event count */
    sinfo->fifo_rec = 0;    /* FIFO reserved-event count */

    sinfo->flags = SENSOR_EXT_FLAG_CONTINUOUS_MODE;

    /* Read as hC, convert to C. */
    sinfo->scale_int = 0;
    sinfo->offset_int = 0;
    sinfo->scale_nano = 10*1000*1000;
    sinfo->offset_nano = 0;

    sinfo->power = 0; /* Power is 1.8 to 3.6 uA ~= 0 mA. */

    return OK;
}

static int tss_start_reporting(struct sub_sensor *sub_sensor,
                    uint64_t sampling_period_ns, uint64_t max_report_latency_ns)
{
    vdbg("sub_sensor_id=%d\n", sub_sensor->sensor_id);

    sub_sensor->running = 1;
    sub_sensor->sampling_period_us = (uint32_t)(sampling_period_ns / 1000);
    sub_sensor->max_report_latency_us = (uint32_t)(max_report_latency_ns / 1000);

    return OK;
}

static int tss_stop_reporting(struct sub_sensor *sub_sensor)
{
    vdbg("sub_sensor_id=%d\n", sub_sensor->sensor_id);

    sub_sensor->sampling_period_us = 0;
    sub_sensor->max_report_latency_us = 0;
    sub_sensor->running = 0;

    return OK;
}

static int _tss_report(struct sub_sensor *sub_sensor, uint8_t flags)
{
    struct timespec ts;
    struct report_info *rinfo;

    vdbg("sub_sensor_id=%d\n", sub_sensor->sensor_id);

    up_rtc_gettime(&ts);

    rinfo = sub_sensor->report_info_data->reportinfo;
    rinfo->reference_time = timespec_to_nsec(&ts);
    rinfo->flags = flags;

    vdbg("%d %u.%03u\n",
        ((struct tss_event_data *)sub_sensor->event_data)->data_value,
        ts.tv_sec, (ts.tv_nsec / 1000000));

    if (sub_sensor->callback) {
        sub_sensor->callback(sub_sensor->sensor_id,
            sub_sensor->report_info_data, sub_sensor->payload_size);
    }

    return OK;
}

static int tss_report(struct sub_sensor *sub_sensor)
{
    return _tss_report(sub_sensor, 0);
}

static int tss_flush(struct sub_sensor *sub_sensor)
{
    return OK;
}

static int tss_alloc_report(struct sub_sensor *sub_sensor)
{
    struct report_info_data *report_info_data;
    struct report_info *report_info;

    vdbg("sub_sensor=%p, sub_id=%d\n", sub_sensor, sub_sensor->sub_id);

    sub_sensor->payload_size =
        (TSS_NUM_SENSORS_REPORTING * sizeof(struct report_info) +
        (TSS_READINGS * sizeof(struct tss_event_data)));

    report_info_data = zalloc(sizeof(struct report_info_data) +
                                                    sub_sensor->payload_size);
    if (!report_info_data) {
        return -ENOMEM;
    }

    report_info_data->num_sensors_reporting = TSS_NUM_SENSORS_REPORTING;

    report_info = report_info_data->reportinfo;
    report_info->readings = TSS_READINGS;

    sub_sensor->report_info_data = report_info_data;
    sub_sensor->event_data = report_info->data_payload;

    return OK;
}

static void tss_free_report(struct sub_sensor *sub_sensor)
{
    free(sub_sensor->report_info_data);
    sub_sensor->payload_size = 0;
    sub_sensor->report_info_data = NULL;
    sub_sensor->event_data = NULL;
}

/* PRESSURE */
#define PSS_READINGS (1)
#define PSS_NUM_SENSORS_REPORTING (1)

struct pss_event_data {
    uint16_t    time_delta;
    uint32_t    data_value;
} __packed;

static int pss_get_sensor_info(struct sub_sensor *sub_sensor,
                                                    struct sensor_info *sinfo)
{
    vdbg("sub_sensor_id=%d\n", sub_sensor->sensor_id);

    sinfo->type = PRESSURE_ID;

    sinfo->version = 1;

    sinfo->name_len = sizeof(SENSOR_DEVICE_NAME);
    memcpy(sinfo->name, SENSOR_DEVICE_NAME, sinfo->name_len);

    sinfo->vendor_len = sizeof(SENSOR_VENDOR_NAME);
    memcpy(sinfo->vendor, SENSOR_VENDOR_NAME, sinfo->vendor_len);

    sinfo->string_type_len = 0; /* Skip string-type if type is set. */

    sinfo->channels = 1;
    sinfo->max_range = 110000; /* 300 - 11000 hPa */
    sinfo->resolution = 1; /* 0.18% */

    sinfo->min_delay = 10*1000; /* us */
    sinfo->max_delay = 10*1000*1000; /* us */

    sinfo->fifo_mec = 0;    /* FIFO max-event count */
    sinfo->fifo_rec = 0;    /* FIFO reserved-event count */

    sinfo->flags = SENSOR_EXT_FLAG_CONTINUOUS_MODE;

    /* Read as Pa, convert to hPa. */
    sinfo->scale_int = 0;
    sinfo->offset_int = 0;
    sinfo->scale_nano = 10*1000*1000;
    sinfo->offset_nano = 0;

    sinfo->power = 0; /* Power is 1.8 to 3.6 uA ~= 0 mA. */

    return OK;
}

static int pss_start_reporting(struct sub_sensor *sub_sensor,
                               uint64_t sampling_period_ns,
                               uint64_t max_report_latency_ns)
{
    vdbg("sub_sensor_id=%d\n", sub_sensor->sensor_id);

    sub_sensor->running = 1;
    sub_sensor->sampling_period_us = (uint32_t)(sampling_period_ns / 1000);
    sub_sensor->max_report_latency_us = (uint32_t)(max_report_latency_ns / 1000);

    return OK;
}

static int pss_stop_reporting(struct sub_sensor *sub_sensor)
{
    vdbg("sub_sensor_id=%d\n", sub_sensor->sensor_id);

    sub_sensor->sampling_period_us = 0;
    sub_sensor->max_report_latency_us = 0;
    sub_sensor->running = 0;

    return OK;
}

static int _pss_report(struct sub_sensor *sub_sensor, uint8_t flags)
{
    struct timespec ts;
    struct report_info *rinfo;

    vdbg("sub_sensor_id=%d\n", sub_sensor->sensor_id);

    up_rtc_gettime(&ts);

    rinfo = sub_sensor->report_info_data->reportinfo;
    rinfo->reference_time = timespec_to_nsec(&ts);
    rinfo->flags = flags;

    vdbg("%d %u.%03u\n",
        ((struct pss_event_data *)sub_sensor->event_data)->data_value,
        ts.tv_sec, (ts.tv_nsec / 1000000));

    if (sub_sensor->callback) {
        sub_sensor->callback(sub_sensor->sensor_id,
            sub_sensor->report_info_data, sub_sensor->payload_size);
    }

    return OK;
}

static int pss_report(struct sub_sensor *sub_sensor)
{
    return _pss_report(sub_sensor, 0);
}

static int pss_flush(struct sub_sensor *sub_sensor)
{
    return OK;
}

static int pss_alloc_report(struct sub_sensor *sub_sensor)
{
    struct report_info_data *report_info_data;
    struct report_info *report_info;

    vdbg("sub_sensor=%p, sub_id=%d\n", sub_sensor, sub_sensor->sub_id);

    sub_sensor->payload_size =
        (PSS_NUM_SENSORS_REPORTING * sizeof(struct report_info) +
        (PSS_READINGS * sizeof(struct pss_event_data)));

    report_info_data = zalloc(sizeof(struct report_info_data) +
                                                    sub_sensor->payload_size);
    if (!report_info_data) {
        return -ENOMEM;
    }

    report_info_data->num_sensors_reporting = PSS_NUM_SENSORS_REPORTING;

    report_info = report_info_data->reportinfo;
    report_info->readings = PSS_READINGS;

    sub_sensor->report_info_data = report_info_data;
    sub_sensor->event_data = report_info->data_payload;

    return OK;
}

static void pss_free_report(struct sub_sensor *sub_sensor)
{
    free(sub_sensor->report_info_data);
    sub_sensor->payload_size = 0;
    sub_sensor->report_info_data = NULL;
    sub_sensor->event_data = NULL;
}

/* HUMIDITY */
#define HSS_READINGS (1)
#define HSS_NUM_SENSORS_REPORTING (1)

struct hss_event_data {
    uint16_t    time_delta;
    uint32_t    data_value;
} __packed;

static int hss_get_sensor_info(struct sub_sensor *sub_sensor,
                                                    struct sensor_info *sinfo)
{
    vdbg("sub_sensor_id=%d\n", sub_sensor->sensor_id);

    sinfo->type = RELATIVE_HUMIDITY_ID;

    sinfo->version = 1;

    sinfo->name_len = sizeof(SENSOR_DEVICE_NAME);
    memcpy(sinfo->name, SENSOR_DEVICE_NAME, sinfo->name_len);

    sinfo->vendor_len = sizeof(SENSOR_VENDOR_NAME);
    memcpy(sinfo->vendor, SENSOR_VENDOR_NAME, sinfo->vendor_len);

    sinfo->string_type_len = 0; /* Skip string-type if type is set. */

    sinfo->channels = 1;
    sinfo->max_range = 10000; /* 0 - 100%rH in %mrH */
    sinfo->resolution = 1; /* 1.5 - 3% */

    sinfo->min_delay = 100*1000; /* us */
    sinfo->max_delay = 10*1000*1000; /* us */

    sinfo->fifo_mec = 0;    /* FIFO max-event count */
    sinfo->fifo_rec = 0;    /* FIFO reserved-event count */

    sinfo->flags = SENSOR_EXT_FLAG_CONTINUOUS_MODE;

    /* Read as %mrH, convert to %rH. */
    sinfo->scale_int = 0;
    sinfo->offset_int = 0;
    sinfo->scale_nano = 1*1000*1000;
    sinfo->offset_nano = 0;

    sinfo->power = 0; /* Power is 1.8 to 3.6 uA ~= 0 mA. */

    return OK;
}

static int hss_start_reporting(struct sub_sensor *sub_sensor,
                               uint64_t sampling_period_ns,
                               uint64_t max_report_latency_ns)
{
    vdbg("sub_sensor_id=%d\n", sub_sensor->sensor_id);

    sub_sensor->running = 1;
    sub_sensor->sampling_period_us = (uint32_t)(sampling_period_ns / 1000);
    sub_sensor->max_report_latency_us = (uint32_t)(max_report_latency_ns / 1000);

    return OK;
}

static int hss_stop_reporting(struct sub_sensor *sub_sensor)
{
    vdbg("sub_sensor_id=%d\n", sub_sensor->sensor_id);

    sub_sensor->sampling_period_us = 0;
    sub_sensor->max_report_latency_us = 0;
    sub_sensor->running = 0;

    return OK;
}

static int _hss_report(struct sub_sensor *sub_sensor, uint8_t flags)
{
    struct timespec ts;
    struct report_info *rinfo;

    vdbg("sub_sensor_id=%d\n", sub_sensor->sensor_id);

    up_rtc_gettime(&ts);

    rinfo = sub_sensor->report_info_data->reportinfo;
    rinfo->reference_time = timespec_to_nsec(&ts);
    rinfo->flags = flags;

    vdbg("%d %u.%03u\n",
        ((struct pss_event_data *)sub_sensor->event_data)->data_value,
        ts.tv_sec, (ts.tv_nsec / 1000000));

    if (sub_sensor->callback) {
        sub_sensor->callback(sub_sensor->sensor_id,
            sub_sensor->report_info_data, sub_sensor->payload_size);
    }

    return OK;
}

static int hss_report(struct sub_sensor *sub_sensor)
{
    return _hss_report(sub_sensor, 0);
}

static int hss_flush(struct sub_sensor *sub_sensor)
{
    return OK;
}

static int hss_alloc_report(struct sub_sensor *sub_sensor)
{
    struct report_info_data *report_info_data;
    struct report_info *report_info;

    vdbg("sub_sensor=%p, sub_id=%d\n", sub_sensor, sub_sensor->sub_id);

    sub_sensor->payload_size =
        (HSS_NUM_SENSORS_REPORTING * sizeof(struct report_info) +
        (HSS_READINGS * sizeof(struct hss_event_data)));

    report_info_data = zalloc(sizeof(struct report_info_data) +
                                                    sub_sensor->payload_size);
    if (!report_info_data) {
        return -ENOMEM;
    }

    report_info_data->num_sensors_reporting = HSS_NUM_SENSORS_REPORTING;

    report_info = report_info_data->reportinfo;
    report_info->readings = HSS_READINGS;

    sub_sensor->report_info_data = report_info_data;
    sub_sensor->event_data = report_info->data_payload;

    return OK;
}

static void hss_free_report(struct sub_sensor *sub_sensor)
{
    free(sub_sensor->report_info_data);
    sub_sensor->payload_size = 0;
    sub_sensor->report_info_data = NULL;
    sub_sensor->event_data = NULL;
}

/* GAS */
#define GSS_READINGS (1)
#define GSS_NUM_SENSORS_REPORTING (1)

struct gss_event_data {
    uint16_t    time_delta;
    uint32_t    data_values[BME680_MAX_PROFILES];
} __packed;

static int gss_get_sensor_info(struct sub_sensor *sub_sensor,
                                                    struct sensor_info *sinfo)
{
    vdbg("sub_sensor_id=%d\n", sub_sensor->sensor_id);

    sinfo->type = 65536;

    sinfo->version = 1;

    sinfo->name_len = sizeof(SENSOR_DEVICE_NAME);
    memcpy(sinfo->name, SENSOR_DEVICE_NAME, sinfo->name_len);

    sinfo->vendor_len = sizeof(SENSOR_VENDOR_NAME);
    memcpy(sinfo->vendor, SENSOR_VENDOR_NAME, sinfo->vendor_len);

    #define GAS_SENSOR_STRING_TYPE "motorola.sensor.gas"
    sinfo->string_type_len = sizeof(GAS_SENSOR_STRING_TYPE);
    memcpy(sinfo->string_type, GAS_SENSOR_STRING_TYPE, sinfo->string_type_len);

    sinfo->channels = BME680_MAX_PROFILES;
    sinfo->max_range = 13e6; /* 178 - 12800000 Ohms */
    sinfo->resolution = 1; /* 0.08% ~= 15 Ohms */

    sinfo->min_delay = 250*1000; /* us */
    sinfo->max_delay = 10*1000*1000; /* us */

    sinfo->fifo_mec = 0;    /* FIFO max-event count */
    sinfo->fifo_rec = 0;    /* FIFO reserved-event count */

    sinfo->flags = SENSOR_EXT_FLAG_CONTINUOUS_MODE;

    /* Read as Ohms, do not convert. */
    sinfo->scale_int = 1;
    sinfo->offset_int = 0;
    sinfo->scale_nano = 0;
    sinfo->offset_nano = 0;

    sinfo->power = 0; /* Power is 1.8 to 3.6 uA ~= 0 mA. */

    return OK;
}

static int gss_start_reporting(struct sub_sensor *sub_sensor,
                               uint64_t sampling_period_ns,
                               uint64_t max_report_latency_ns)
{
    vdbg("sub_sensor_id=%d\n", sub_sensor->sensor_id);

    sub_sensor->running = 1;
    sub_sensor->sampling_period_us = (uint32_t)(sampling_period_ns / 1000);
    sub_sensor->max_report_latency_us = (uint32_t)(max_report_latency_ns / 1000);

    return OK;
}

static int gss_stop_reporting(struct sub_sensor *sub_sensor)
{
    vdbg("sub_sensor_id=%d\n", sub_sensor->sensor_id);

    sub_sensor->sampling_period_us = 0;
    sub_sensor->max_report_latency_us = 0;
    sub_sensor->running = 0;

    return OK;
}

static int _gss_report(struct sub_sensor *sub_sensor, uint8_t flags)
{
    struct timespec ts;
    struct report_info *rinfo;

    vdbg("sub_sensor_id=%d\n", sub_sensor->sensor_id);

    up_rtc_gettime(&ts);

    rinfo = sub_sensor->report_info_data->reportinfo;
    rinfo->reference_time = timespec_to_nsec(&ts);
    rinfo->flags = flags;

    vdbg("%d %u.%03u\n",
        ((struct gss_event_data *)sub_sensor->event_data)->data_values[0],
        ts.tv_sec, (ts.tv_nsec / 1000000));

    if (sub_sensor->callback) {
        sub_sensor->callback(sub_sensor->sensor_id,
            sub_sensor->report_info_data, sub_sensor->payload_size);
    }

    return OK;
}

static int gss_report(struct sub_sensor *sub_sensor)
{
    return _gss_report(sub_sensor, 0);
}

static int gss_flush(struct sub_sensor *sub_sensor)
{
    return OK;
}

static int gss_alloc_report(struct sub_sensor *sub_sensor)
{
    struct report_info_data *report_info_data;
    struct report_info *report_info;

    vdbg("sub_sensor=%p, sub_id=%d\n", sub_sensor, sub_sensor->sub_id);

    sub_sensor->payload_size =
        (GSS_NUM_SENSORS_REPORTING * sizeof(struct report_info) +
        (GSS_READINGS * sizeof(struct gss_event_data)));

    report_info_data = zalloc(sizeof(struct report_info_data) +
                                                    sub_sensor->payload_size);
    if (!report_info_data) {
        return -ENOMEM;
    }

    report_info_data->num_sensors_reporting = GSS_NUM_SENSORS_REPORTING;

    report_info = report_info_data->reportinfo;
    report_info->readings = GSS_READINGS;

    sub_sensor->report_info_data = report_info_data;
    sub_sensor->event_data = report_info->data_payload;

    return OK;
}

static void gss_free_report(struct sub_sensor *sub_sensor)
{
    free(sub_sensor->report_info_data);
    sub_sensor->payload_size = 0;
    sub_sensor->report_info_data = NULL;
    sub_sensor->event_data = NULL;
}

const static struct sub_sensor_ops sub_sensor_ops[SUB_SENSOR_MAX] =
{
    /* Temperature */
    {
        .get_sensor_info = tss_get_sensor_info,
        .start_reporting = tss_start_reporting,
        .stop_reporting = tss_stop_reporting,
        .report = tss_report,
        .flush = tss_flush,
        .alloc_report = tss_alloc_report,
        .free_report = tss_free_report,
    },
    /* Pressure */
    {
        .get_sensor_info = pss_get_sensor_info,
        .start_reporting = pss_start_reporting,
        .stop_reporting = pss_stop_reporting,
        .report = pss_report,
        .flush = pss_flush,
        .alloc_report = pss_alloc_report,
        .free_report = pss_free_report,
    },
    /* Humidity */
    {
        .get_sensor_info = hss_get_sensor_info,
        .start_reporting = hss_start_reporting,
        .stop_reporting = hss_stop_reporting,
        .report = hss_report,
        .flush = hss_flush,
        .alloc_report = hss_alloc_report,
        .free_report = hss_free_report,
    },
    /* Gas */
    {
        .get_sensor_info = gss_get_sensor_info,
        .start_reporting = gss_start_reporting,
        .stop_reporting = gss_stop_reporting,
        .report = gss_report,
        .flush = gss_flush,
        .alloc_report = gss_alloc_report,
        .free_report = gss_free_report,
    },
};

/* I2C */
int bme680_i2c_open(struct bme680_info *bme680_info)
{
    vdbg("\n");

    bme680_info->buf_len = 64;
    bme680_info->buf = zalloc(bme680_info->buf_len);
    if (!bme680_info->buf) {
        return -ENOMEM;
    }

    bme680_info->i2c_dev = up_i2cinitialize(CONFIG_SENSOR_BME680_I2C_BUS);
    if (!bme680_info->i2c_dev) {
        vdbg("ERROR: Failed to initialize i2c device");

        /* Clean-up the buffer. */
        free(bme680_info->buf);
        bme680_info->buf = NULL;
        bme680_info->buf_len = 0;

        return -EIO;
    }

    I2C_SETFREQUENCY(bme680_info->i2c_dev, CONFIG_SENSOR_BME680_I2C_BUS_SPEED);
    I2C_SETADDRESS(bme680_info->i2c_dev, CONFIG_SENSOR_BME680_I2C_ADDRESS, 7);

    return OK;
}

static int bme680_i2c_close(struct bme680_info *bme680_info)
{
    vdbg("\n");

    if (bme680_info->buf) {
        free(bme680_info->buf);
        bme680_info->buf = NULL;
        bme680_info->buf_len = 0;
    }

    if (bme680_info->i2c_dev) {
        (void)up_i2cuninitialize(bme680_info->i2c_dev);
        bme680_info->i2c_dev = NULL;
    }

    return OK;
}

static s8 _bme_i2c_sensor_read(u8 dev_addr, u8 reg_addr, u8 *reg_data_ptr,
                                                                    u8 data_len)
{
    struct bme680_info *bme680_info = g_bme680;

    vdbg("addr=0x%x, reg_addr=0x%x, len=%d\n", dev_addr, reg_addr, data_len);

    int result;
    struct i2c_msg_s msgs[2];

    msgs[0].addr   = dev_addr;
    msgs[0].flags  = 0;
    msgs[0].buffer = &reg_addr;
    msgs[0].length = sizeof(reg_addr);

    msgs[1].addr   = dev_addr;
    msgs[1].flags  = I2C_M_READ;
    msgs[1].buffer = reg_data_ptr;
    msgs[1].length = data_len;

    result = I2C_TRANSFER(bme680_info->i2c_dev, msgs, ARRAY_SIZE(msgs));

    return result;
}

static s8 _bme_i2c_sensor_write(u8 dev_addr, u8 reg_addr, u8 *reg_data_ptr,
                                                                    u8 data_len)
{
    struct bme680_info *bme680_info = g_bme680;
    int result;
    struct i2c_msg_s msg;

    vdbg("addr=0x%x, reg_addr=0x%x, len=%d\n", dev_addr, reg_addr, data_len);

    /* If the buffer is not big enough, allocate a new one. */
    if ((data_len + 1) > bme680_info->buf_len) {
        if (bme680_info->buf) {
            free(bme680_info->buf);
        }

        bme680_info->buf_len = data_len + 1;

        /* Force the buf_len to be multiples of 16. */
        bme680_info->buf_len += 0x10;
        bme680_info->buf_len &= 0x0f;

        bme680_info->buf = zalloc(bme680_info->buf_len);
        if (!bme680_info->buf) {
            return -ENOMEM;
        }
    }

    bme680_info->buf[0] = reg_addr;
    memcpy(bme680_info->buf+1, reg_data_ptr, data_len);

    msg.addr   = dev_addr;
    msg.flags  = 0;
    msg.buffer = bme680_info->buf;
    msg.length = sizeof(reg_addr) + data_len;

    result = I2C_TRANSFER(bme680_info->i2c_dev, &msg, 1);
    return result;
}

/* Adapter to open-source BME680 driver */
static int bme680_adapter_init(struct bme680_info *info)
{
    struct bme680_t *adapter = &info->adapter;
    enum bme680_return_type com_rslt = BME680_COMM_RES_ERROR;

    adapter->bme680_bus_read = _bme_i2c_sensor_read;
    adapter->bme680_bus_write = _bme_i2c_sensor_write;
    adapter->bme680_burst_read = NULL; /* Not used */
    adapter->delay_msec = NULL; /* Not used */

    adapter->interface = BME680_I2C_INTERFACE;
    adapter->dev_addr = CONFIG_SENSOR_BME680_I2C_ADDRESS;

    /* get chip id and calibration parameter */
    com_rslt = bme680_init(adapter);
    if (com_rslt != BME680_COMM_RES_OK) {
        return -ENODEV;
    }

    if (BME680_CHIP_ID != adapter->chip_id) {
        return -EINVAL;
    }

    return OK;
}

static int bme680_adapter_config(struct bme680_info *info)
{
    struct bme680_t *bme680 = &info->adapter;
    enum bme680_return_type result = BME680_COMM_RES_ERROR;
    struct bme680_sens_conf set_conf_sensor;
    struct bme680_heater_conf set_heatr_conf_sensor;

    set_conf_sensor.filter = BME680_FILTER_COEFF_127;

    /* Set reading interval ~ the polling rate. */
    // TODO: This is not optimal, but setting the reading rate slower
    //       causes us to miss readings.
    if (info->latency_us >= 1*1000*1000) {
        set_conf_sensor.odr = BME680_ODR_1000MS;
    } else if (info->latency_us >= 500*1000) {
        set_conf_sensor.odr = BME680_ODR_500MS;
    } else if (info->latency_us >= 250*1000) {
        set_conf_sensor.odr = BME680_ODR_250MS;
    } else if (info->latency_us >= 125*1000) {
        set_conf_sensor.odr = BME680_ODR_125MS;
    } else if (info->latency_us >= 62*1000) {
        set_conf_sensor.odr = BME680_ODR_62_5MS;
    } else if (info->latency_us >= 20*1000) {
        set_conf_sensor.odr = BME680_ODR_20MS;
    } else if (info->latency_us >= 10*1000) {
        set_conf_sensor.odr = BME680_ODR_10MS;
    } else {
        set_conf_sensor.odr = BME680_ODR_0_59MS;
    }

    if (DEBUG_ENABLE_ALL || info->sub_sensors[SUB_SENSOR_TEMPERATURE].running) {
        set_conf_sensor.osrs_temp = BME680_OSRS_1X;
    } else {
        set_conf_sensor.osrs_temp = BME680_OSRS_NONE;
    }

    if (DEBUG_ENABLE_ALL || info->sub_sensors[SUB_SENSOR_PRESSURE].running) {
        set_conf_sensor.osrs_pres = BME680_OSRS_16X;
    } else {
        set_conf_sensor.osrs_pres = BME680_OSRS_NONE;
    }

    if (DEBUG_ENABLE_ALL || info->sub_sensors[SUB_SENSOR_HUMIDITY].running) {
        set_conf_sensor.osrs_hum = BME680_OSRS_4X;
    } else {
        set_conf_sensor.osrs_hum = BME680_OSRS_NONE;
    }

    if (DEBUG_ENABLE_ALL || info->sub_sensors[SUB_SENSOR_GAS].running) {
        set_conf_sensor.heatr_ctrl = BME680_HEATR_CTRL_ENABLE;
        set_conf_sensor.run_gas =  BME680_RUN_GAS_ENABLE;
        set_conf_sensor.nb_conv = BME680_NB_CONV_10;
    } else {
        set_conf_sensor.heatr_ctrl = BME680_HEATR_CTRL_DISABLE;
        set_conf_sensor.run_gas =  BME680_RUN_GAS_DISABLE;
        set_conf_sensor.nb_conv = BME680_NB_CONV_SKIPPED;
    }

    result = bme680_set_sensor_config(&set_conf_sensor, bme680);
    if (result != BME680_COMM_RES_OK) {
        vdbg("ERROR: set sensor config: %d\n", result);
        return -EIO;
    }

    if (DEBUG_ENABLE_ALL || info->sub_sensors[SUB_SENSOR_GAS].running) {
        size_t i;
        for (i = 0; i < BME680_MAX_PROFILES; i++) {
            set_heatr_conf_sensor.heater_temp[i] = 202 + 22 * i; /* 202 - 400 C */
            set_heatr_conf_sensor.heatr_idacv[i] = 1; /* ((v + 1) / 8) mA */
            set_heatr_conf_sensor.heatr_dur[i] = 1; /* 1 shared duration (see below) */
        }
        set_heatr_conf_sensor.heatr_dur_shared = 250; /* ms */
        set_heatr_conf_sensor.profile_cnt = BME680_MAX_PROFILES;

        /* activate heater configuration */
        result = bme680_set_gas_heater_config(&set_heatr_conf_sensor, BME680_PARALLEL_MODE, bme680);
        if (result != BME680_COMM_RES_OK) {
            vdbg("ERROR: set gas heater config: %d\n", result);
            return -EIO;
        }
    }

    /* Set power mode as forced mode */
    result = bme680_set_power_mode(BME680_PARALLEL_MODE, bme680);
    if (result != BME680_COMM_RES_OK) {
        vdbg("ERROR: set sensor mode: %d\n", result);
        return -EIO;
    }

    return OK;
}

static int bme680_adapter_read(struct bme680_info *info)
{
    struct bme680_t *bme680 = &info->adapter;
    struct bme680_uncomp_field_data uncompensated_data;

    bme680_get_uncomp_data(&uncompensated_data, 1, BME680_ALL, bme680);
    if (!uncompensated_data.status.new_data) {
        vdbg("data not ready\n");
        return -EAGAIN;
    }

    vdbg("meas_index=%d\n", uncompensated_data.status.meas_index);

    if (DEBUG_ENABLE_ALL || info->sub_sensors[SUB_SENSOR_TEMPERATURE].running) {
        struct tss_event_data *src = info->sub_sensors[SUB_SENSOR_TEMPERATURE].event_data;
        src->data_value = bme680_compensate_temperature_int32(uncompensated_data.temp_adcv, bme680);
        vdbg("%d hC\n", src->data_value);
#if CONFIG_REGLOG
        reglog_log(uncompensated_data.status.meas_index, src->data_value);
#endif
    }

    if (DEBUG_ENABLE_ALL || info->sub_sensors[SUB_SENSOR_PRESSURE].running) {
        struct pss_event_data *src = info->sub_sensors[SUB_SENSOR_PRESSURE].event_data;
        src->data_value = bme680_compensate_pressure_int32(uncompensated_data.pres_adcv, bme680);
        vdbg("%d Pa\n", src->data_value);
#if CONFIG_REGLOG
        reglog_log(uncompensated_data.status.meas_index, src->data_value);
#endif
    }

    if (DEBUG_ENABLE_ALL || info->sub_sensors[SUB_SENSOR_HUMIDITY].running) {
        struct hss_event_data *src = info->sub_sensors[SUB_SENSOR_HUMIDITY].event_data;
        src->data_value = bme680_compensate_humidity_int32(uncompensated_data.hum_adcv, bme680);
        vdbg("%d rH\n", src->data_value);
#if CONFIG_REGLOG
        reglog_log(uncompensated_data.status.meas_index, src->data_value);
#endif
    }

    if (DEBUG_ENABLE_ALL || info->sub_sensors[SUB_SENSOR_GAS].running) {
        if (uncompensated_data.status.gas_valid && uncompensated_data.status.heatr_stab) {
            struct gss_event_data *src = info->sub_sensors[SUB_SENSOR_GAS].event_data;
            if (uncompensated_data.status.gas_meas_index < ARRAY_SIZE(src->data_values)) {
                src->data_values[uncompensated_data.status.gas_meas_index] = bme680_calculate_gas_int32(uncompensated_data.gas_res_adcv, uncompensated_data.gas_range, bme680);
                vdbg("gas_meas_index=%d\n", uncompensated_data.status.gas_meas_index);
                vdbg("%d Ohm at %d C\n", src->data_values[uncompensated_data.status.gas_meas_index], (202 + 22 * uncompensated_data.status.gas_meas_index));
#if CONFIG_REGLOG
                reglog_log(uncompensated_data.status.meas_index,
                    src->data_values[uncompensated_data.status.gas_meas_index]);
#endif
            }
        }
    }

    return OK;
}

/* Core */
static struct sub_sensor *sensor_from_sub_id(struct bme680_info *info,
                                                    uint8_t sub_id)
{
    if (sub_id >= ARRAY_SIZE(info->sub_sensors)) {
        vdbg("ERROR: Invalid sub_id\n");
        return NULL;
    }

    return &info->sub_sensors[sub_id];
}

static int sub_id_from_rsrc_id(struct device *dev)
{
    struct device_resource *rsrc_sub_id = device_resource_get_by_name(dev,
        DEVICE_RESOURCE_TYPE_REGS, "sub_id");
    if (!rsrc_sub_id) {
        vdbg("ERROR: Failed to get sub_id\n");
        return -1;
    } else if (rsrc_sub_id->start >= SUB_SENSOR_MAX) {
        vdbg("ERROR: Invalid sub_id\n");
        return -1;
    }

    return rsrc_sub_id->start;
}

static void *bme680_thread(void *arg)
{
    struct bme680_info *info = arg;
    int ret;

    vdbg("thread started\n");

    BME680_LOCK();

    bme680_i2c_open(info);
    bme680_adapter_init(info);

    BME680_UNLOCK();

    while (1) {
        size_t i;

        vdbg("start loop\n");

        BME680_LOCK();

        /* Update configuration */
        if (info->config_changed) {
            vdbg("config changing\n");

            /* Put the sensor in sleep mode. */
            enum bme680_return_type result =
                    bme680_set_power_mode(BME680_SLEEP_MODE, &info->adapter);
            if (result != BME680_COMM_RES_OK) {
                vdbg("ERROR: config failed\n");
                break;
            }

            /* Update latency_us */
            info->latency_us = 0xffffffff;
            for (i = 0; i < ARRAY_SIZE(info->sub_sensors); i++) {
                struct sub_sensor *sub_sensor = &info->sub_sensors[i];
                /* Use the lowest non-zero sampling_period_us. */
                if (sub_sensor->sampling_period_us &&
                    sub_sensor->sampling_period_us < info->latency_us) {
                    info->latency_us = sub_sensor->sampling_period_us;
                }
            }

            bme680_adapter_config(info);

            info->config_changed = 0;

            vdbg("config changed\n");
        }

        /* This are counted in the loop. */
        int tmp_running = 0;

        ret = bme680_adapter_read(info);
        for (i = 0; i < ARRAY_SIZE(info->sub_sensors); i++) {
            struct sub_sensor *sub_sensor = &info->sub_sensors[i];
            if (sub_sensor->running) {
                /* Count the number of running sensors. */
                tmp_running++;

                /* Notify sub sensors if data is available. */
                if (ret == OK) {
                    if (sub_sensor->ops->report) {
                        sub_sensor->ops->report(sub_sensor);
                    }
                }
            }
        }

        info->running = tmp_running;

        BME680_UNLOCK();

        if (tmp_running == 0) {
            vdbg("thread stopping\n");
            break;
        }

        vdbg("latency_us=%d\n", info->latency_us);
        usleep(info->latency_us);

        vdbg("end loop\n");
    }

    BME680_LOCK();

    bme680_i2c_close(info);

    BME680_UNLOCK();

    vdbg("thread done\n");
    return (void *)OK;
}

static int bme680_start_thread_locked(struct bme680_info *info)
{
    int ret;

    vdbg("\n");

    if (info->thread) {
        vdbg("thread already started\n");
        return OK;
    }

    ret = pthread_create(&info->thread, NULL, bme680_thread, info);
    if (ret) {
        vdbg("ERROR: Failed to create thread: %s.\n", strerror(errno));
        info->thread = 0;
    }

    return ret;
}

static int bme680_stop_thread_locked(struct bme680_info *info)
{
    int ret;
    pthread_addr_t join_value;

    if (!info->thread) {
        vdbg("thread not started\n");
        return OK;
    }

    /* Unlock the mutex so the thread can complete and join. */
    BME680_UNLOCK();

    /* The thread might be sleeping, give it enough time to wake up first. */
    usleep(info->latency_us);

    ret = pthread_join(info->thread, &join_value);
    vdbg("thread=%p, start=%p, join_value=%p, ret=%d, errno=%d\n", info->thread,
                                        bme680_thread, join_value, ret, errno);
    if (ret != ESRCH) {
        ret = pthread_kill(info->thread, 9);
        vdbg("kill ret=%d, errno=%d\n", ret, errno);
    }

    /* Relock the thread before returning. */
    BME680_LOCK();

    info->thread = 0;

    return ret;
}

static int bme680_num_running_locked(struct bme680_info *info)
{
    size_t i;
    int tmp_running = 0;

    for (i = 0; i < ARRAY_SIZE(info->sub_sensors); i++) {
        struct sub_sensor *sub_sensor = &info->sub_sensors[i];
        tmp_running += (sub_sensor->running ? 1 : 0);
    }

    info->running = tmp_running;

    return tmp_running;
}

static int bme680_update_thread_locked(struct bme680_info *info)
{
    int running = bme680_num_running_locked(info);

    if (running) {
        bme680_start_thread_locked(info);
    } else {
        bme680_stop_thread_locked(info);
    }

    return OK;
}

static int bme680_op_get_sensor_info(struct device *dev, uint8_t sensor_id,
                                                    struct sensor_info *sinfo)
{
    struct sub_sensor *sub_sensor;
    int ret;

    vdbg("dev=%p, sensor_id=%d\n", dev, sensor_id);

    if (!dev) {
        return -EINVAL;
    }

    sub_sensor = device_get_private(dev);
    if (!sub_sensor) {
        return -ENOENT;
    }

    BME680_LOCK();

    if (sub_sensor->ops->get_sensor_info) {
        ret = sub_sensor->ops->get_sensor_info(sub_sensor, sinfo);
    } else {
        ret = -ENODEV;
    }

    BME680_UNLOCK();

    return ret;
}

static int bme680_op_start_reporting(struct device *dev, uint8_t sensor_id,
                    uint64_t sampling_period_ns, uint64_t max_report_latency_ns)
{
    struct sub_sensor *sub_sensor;
    int ret;

    vdbg("dev=%p, sensor_id=%d, period_ns=%lld, max_report_latency_ns=%lld\n",
                    dev, sensor_id, sampling_period_ns, max_report_latency_ns);

    if (!dev) {
        return -EINVAL;
    }

    sub_sensor = device_get_private(dev);
    if (!sub_sensor) {
        return -ENOENT;
    }

    BME680_LOCK();

    if (sub_sensor->ops->start_reporting) {
        ret = sub_sensor->ops->start_reporting(sub_sensor, sampling_period_ns,
                                                        max_report_latency_ns);
    } else {
        ret = -ENODEV;
    }

    sub_sensor->info->config_changed = 1;

    bme680_update_thread_locked(sub_sensor->info);

    BME680_UNLOCK();

    return ret;
}

static int bme680_op_stop_reporting(struct device *dev, uint8_t sensor_id)
{
    struct sub_sensor *sub_sensor;
    int ret;

    vdbg("dev=%p, sensor_id=%d\n", dev, sensor_id);

    if (!dev) {
        return -EINVAL;
    }

    sub_sensor = device_get_private(dev);
    if (!sub_sensor) {
        return -ENOENT;
    }

    BME680_LOCK();

    sub_sensor->info->config_changed = 1;

    if (sub_sensor->ops->stop_reporting) {
        ret = sub_sensor->ops->stop_reporting(sub_sensor);
    } else {
        ret = -ENODEV;
    }

    bme680_update_thread_locked(sub_sensor->info);

    BME680_UNLOCK();

    return ret;
}

static int bme680_op_flush(struct device *dev, uint8_t sensor_id)
{
    struct sub_sensor *sub_sensor;
    int ret;

    vdbg("dev=%p, sensor_id=%d\n", dev, sensor_id);

    if (!dev) {
        return -EINVAL;
    }

    sub_sensor = device_get_private(dev);
    if (!sub_sensor) {
        return -ENOENT;
    }

    BME680_LOCK();

    if (sub_sensor->ops->flush) {
        ret = sub_sensor->ops->flush(sub_sensor);
    } else {
        ret = -ENODEV;
    }

    BME680_UNLOCK();

    return ret;
}

static int bme680_op_register_callback(struct device *dev,
                                       uint8_t sensor_id,
                                       sensors_ext_event_callback callback)
{
    struct sub_sensor *sub_sensor;
    int ret;

    vdbg("dev=%p, sensor_id=%d\n", dev, sensor_id);

    if (!dev) {
        return -EINVAL;
    }

    sub_sensor = device_get_private(dev);
    if (!sub_sensor) {
        return -ENOENT;
    }

    BME680_LOCK();

    sub_sensor->callback = callback;
    sub_sensor->sensor_id = sensor_id;
    sub_sensor->report_info_data->reportinfo[0].id = sensor_id;

    ret = OK;

    BME680_UNLOCK();

    return ret;
}

#if CONFIG_PM
static int pm_prepare(struct pm_callback_s *cb, enum pm_state_e state)
{
    struct bme680_info *info = g_bme680;
    int ret;

    BME680_LOCK();

    /* Prevent PM sleep if any sensors are running. */
    ret = ((state >= PM_IDLE) && info->running) ? -EIO : OK;

    BME680_UNLOCK();

    return ret;
}

static struct pm_callback_s pm_callback =
{
    .prepare = pm_prepare,
};
#endif

static int bme680_dev_open(struct device *dev)
{
    struct sub_sensor *sub_sensor;

    vdbg("dev=%p\n", dev);

    if (!dev) {
        return -EINVAL;
    }

    sub_sensor = device_get_private(dev);
    if (!sub_sensor) {
        return -ENOENT;
    }

    BME680_LOCK();

    if (sub_sensor->info->opened == 0) {
        vdbg("first opened\n");
    }

    sub_sensor->info->opened++;

    BME680_UNLOCK();


    return 0;
}

static void bme680_dev_close(struct device *dev)
{
    struct sub_sensor *sub_sensor;

    vdbg("dev=%p\n", dev);

    if (!dev) {
        return;
    }

    sub_sensor = device_get_private(dev);
    if (!sub_sensor) {
        return;
    }

    BME680_LOCK();

    /* Guarantee that the sub_sensor was stopped so the thread can be updated
     * correctly.
    */
    sub_sensor->running = 0;

    sub_sensor->info->opened--;
    if (sub_sensor->info->opened == 0) {
        vdbg("last closed\n");
        bme680_stop_thread_locked(sub_sensor->info);
    }

    BME680_UNLOCK();
}

static int bme680_dev_probe(struct device *dev)
{
    struct bme680_info *info = g_bme680;
    int info_alloc = 0;
    int ret;

    vdbg("dev=%p\n", dev);

    if (!dev) {
        return -EINVAL;
    }

    BME680_LOCK();

    /* Core initialization. */
    if (!info) {
        info = zalloc(sizeof(*info));
        if (!info) {
            BME680_UNLOCK();
            return -ENOMEM;
        }

        info_alloc = 1;

        info->probed = 0;
        info->opened = 0;
        info->running = 0;
        info->config_changed = 1;

#if CONFIG_PM
        if (pm_register(&pm_callback) != OK) {
            vdbg("ERROR: Failed register to power management!\n");
        }
#endif
    }

    /* Sub-sensor initialization. */
    int sub_id = sub_id_from_rsrc_id(dev);
    if (sub_id < 0) {
        ret = -EINVAL;
        goto error;
    }

    struct sub_sensor *sub_sensor = sensor_from_sub_id(info, sub_id);
    if (!sub_sensor) {
        ret = -ENODEV;
        goto error;
    }

    if (sub_sensor->ops) {
        vdbg("ERROR: sub-sensor already probed.\n");
        ret = -EINVAL;
        goto error;
    }

    sub_sensor->info = info;
    sub_sensor->ops = &sub_sensor_ops[sub_id];
    sub_sensor->sub_id = sub_id;
    sub_sensor->ops->alloc_report(sub_sensor);
    if (!sub_sensor->event_data) {
        vdbg("ERROR: no memory for event data.\n");
        ret = -ENOMEM;
        goto error;
    }

    info->probed++;

    device_set_private(dev, sub_sensor);

    g_bme680 = info;

    ret = OK;

error:
    /* Only free info if it was allocated in this failed probe. */
    if (ret != OK && info_alloc) {
        free(info);
    }

    BME680_UNLOCK();

    return ret;
}

static void bme680_dev_remove(struct device *dev)
{
    struct sub_sensor *sub_sensor;

    vdbg("dev=%p\n", dev);

    if (!dev) {
        return;
    }

    sub_sensor = device_get_private(dev);
    if (!sub_sensor) {
        return;
    }

    BME680_LOCK();

    device_set_private(dev, NULL);

    /* Sub-sensor clean-up. */
    sub_sensor->ops = NULL;
    sub_sensor->sub_id = 0;
    sub_sensor->sensor_id = 0;
    sub_sensor->running = 0;

    sub_sensor->ops->free_report(sub_sensor);
    sub_sensor->event_data = NULL;

    bme680_stop_thread_locked(sub_sensor->info);

    sub_sensor->info->probed--;
    if (sub_sensor->info->probed == 0) {
        /* This is the last probed driver, clean up. */
        bme680_dev_close(dev);

        free(sub_sensor->info);
        g_bme680 = NULL;
    }

    BME680_UNLOCK();
}

static struct device_sensors_ext_type_ops bme680_type_ops = {
    .get_sensor_info = bme680_op_get_sensor_info,
    .start_reporting = bme680_op_start_reporting,
    .flush = bme680_op_flush,
    .stop_reporting = bme680_op_stop_reporting,
    .register_callback = bme680_op_register_callback,
};

static struct device_driver_ops bme680_driver_ops = {
    .probe    = bme680_dev_probe,
    .remove   = bme680_dev_remove,
    .open     = bme680_dev_open,
    .close    = bme680_dev_close,
    .type_ops = &bme680_type_ops,
};

const struct device_driver sensor_bme860_driver = {
    .type   = DEVICE_TYPE_SENSORS_HW,
    .name   = "BME680",
    .desc   = "BME680",
    .ops    = &bme680_driver_ops,
};
