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
#include <stdio.h>
#include <arch/atomic.h>
#include <nuttx/clock.h>
#include <nuttx/lib.h>
#include <nuttx/util.h>
#include <nuttx/kmalloc.h>
#include <nuttx/pthread.h>
#include <nuttx/device.h>
#include <nuttx/device_sensors_ext.h>
#include <nuttx/greybus/debug.h>
#include <nuttx/power/pm.h>
#include <nuttx/rtc.h>
#include <nuttx/time.h>
#include <nuttx/i2c.h>


#define SENSOR_ACCEL_FLAG_OPEN       BIT(0)

/* Accelerometer attributes */
#define ACCEL_DEVICE_NAME  "Accel-LSM9SD1"
#define ACCEL_VENDOR_NAME  "Moto"
#define ACCEL_VERSION      1
#define ACCEL_SAMPLE_RATE  100
#define ACCEL_SCALE_INT    0
#define ACCEL_SCALE_NANO   3
#define ACCEL_OFFSET_INT    0
#define ACCEL_OFFSET_NANO   0
#define ACCEL_CHANNEL_SIZE  3
#define ACCEL_MAX_RANGE     1024 * 16
#define ACCEL_MIN_DELAY     4000
#define ACCEL_MAX_DELAY     1000000
#define ACCEL_FIFO_REC      0
#define ACCEL_FIFO_MEC      0
#define ACCEL_FLAGS        SENSOR_EXT_FLAG_CONTINUOUS_MODE
#define REPORTING_SENSORS   1


/* acceleromter register offsets */
#define ACT_THS             0x04
#define ACT_DUR             0x05
#define INT_GEN_CFG_XL      0x06
#define INT_GEN_THS_X_XL    0x07
#define INT_GEN_THS_Y_XL    0x08
#define INT_GEN_THS_Z_XL    0x09
#define INT_GEN_DUR_XL      0x0A
#define REFERENCE_G         0x0B
#define INT1_CTRL           0x0C
#define INT2_CTRL           0x0D
#define WHO_AM_I_XG         0x0F
#define CTRL_REG1_G         0x10
#define CTRL_REG2_G         0x11
#define CTRL_REG3_G         0x12
#define ORIENT_CFG_G        0x13
#define INT_GEN_SRC_G       0x14
#define OUT_TEMP_L          0x15
#define OUT_TEMP_H          0x16
#define STATUS_REG_0        0x17
#define OUT_X_L_G           0x18
#define OUT_X_H_G           0x19
#define OUT_Y_L_G           0x1A
#define OUT_Y_H_G           0x1B
#define OUT_Z_L_G           0x1C
#define OUT_Z_H_G           0x1D
#define CTRL_REG4           0x1E
#define CTRL_REG5_XL        0x1F
#define CTRL_REG6_XL        0x20
#define CTRL_REG7_XL        0x21
#define CTRL_REG8           0x22
#define CTRL_REG9           0x23
#define CTRL_REG10          0x24
#define INT_GEN_SRC_XL      0x26
#define STATUS_REG_1        0x27
#define OUT_X_L_XL          0x28
#define OUT_X_H_XL          0x29
#define OUT_Y_L_XL          0x2A
#define OUT_Y_H_XL          0x2B
#define OUT_Z_L_XL          0x2C
#define OUT_Z_H_XL          0x2D
#define FIFO_CTRL           0x2E
#define FIFO_SRC            0x2F
#define INT_GEN_CFG_G       0x30
#define INT_GEN_THS_XH_G    0x31
#define INT_GEN_THS_XL_G    0x32
#define INT_GEN_THS_YH_G    0x33
#define INT_GEN_THS_YL_G    0x34
#define INT_GEN_THS_ZH_G    0x35
#define INT_GEN_THS_ZL_G    0x36
#define INT_GEN_DUR_G       0x37

/* CTRL_REG6_XL/FS_XL, Full scale selection of accel*/
enum accel_scale
{
    A_SCALE_2G,  /* 00: +/- 2g  */
    A_SCALE_16G, /* 01: +/- 16g */
    A_SCALE_4G,  /* 10: +/- 4g  */
    A_SCALE_8G   /* 11: +/- 8g  */
};

/* CTRL_REG6_XL/ODR_XL, output data rates of the accel */
typedef enum accel_odr
{
    A_POWER_DOWN,   /* Power-down mode (0x0) */
    A_ODR_10,       /* 10 Hz (0x1) */
    A_ODR_50,       /* 50 Hz (0x2) */
    A_ODR_119,      /* 119 Hz (0x3) */
    A_ODR_238,      /* 238 Hz (0x4) */
    A_ODR_476,      /* 476 Hz (0x5) */
    A_ODR_952       /* 952 Hz (0x6) */
}accel_odr;

/* CTRL_REG6_XL/BW_XL bandwiths for low-pass filter of the accel */
enum accel_bw
{
    A_BW_AUTO_SCALE = 0x0,  /* Automatic BW scaling (0x0) */
    A_BW_408 = 0x4,         /* 408 Hz (0x4) */
    A_BW_211 = 0x5,         /* 211 Hz (0x5) */
    A_BW_105 = 0x6,         /* 105 Hz (0x6) */
    A_BW_50 = 0x7           /* 50 Hz (0x7) */
};

int16_t ax_raw, ay_raw, az_raw; /* x, y, and z axis readings of the accel */

struct sensor_accel_info {
    struct device *dev;
    struct i2c_dev_s *i2c;
    uint32_t flags;
    sensors_ext_event_callback callback;
    uint8_t sensor_id;
    int a_scale;
    int scale_resolution;
    int latency_ms;
    pthread_t tx_thread;
    bool should_run;
};

/*
 * event fields for accelerometer in bytes
 * [ time_delta (2) | data_value x (4) | data_value y (4) |data_value z (4) ]
*/
struct sensor_event_data {
    uint16_t    time_delta;
    uint32_t    data_value[ACCEL_CHANNEL_SIZE];
} __packed;

#ifdef BATCH_PROCESS_ENABLED
#define ACCEL_READING_NUM 2
#else
#define ACCEL_READING_NUM 1
#endif
#define TX_PROCESSING_DELAY     2 /* processing and tx delay (mS)*/


static atomic_t txn;       /* Flag to indicate reporting in progress */

static uint8_t i2c_reg_read(struct sensor_accel_info *info,
                             uint8_t reg, uint8_t *regdata, int len)
{
    uint8_t buf[1];
    int ret = 0;
    int index = 3;
    buf[0] = reg;
    do {
        ret = I2C_WRITEREAD(info->i2c, buf, 1, regdata, len);
    } while (ret != 0 && index--);
    return ret;
}

void calc_aresolution(struct sensor_accel_info *info)
{
/* Possible accelerometer scales(register settings):
 * 2g (000), 4g (001), 6g (010) 8g (011), 16g (100)
 */
    switch (info->a_scale)
    {
        case A_SCALE_2G:
            info->scale_resolution = 598550;
            break;
        case A_SCALE_4G:
            info->scale_resolution = 1197101;
            break;
        case A_SCALE_8G:
            info->scale_resolution = 2394202; /* (8 * 9.80665)/32768 */
            break;
        case A_SCALE_16G:
            info->scale_resolution = 4788404;
            break;
    }
}

void read_accel(struct sensor_accel_info *info)
{
    int ret;

    /* data to be read from the accel */
    uint8_t odata[6];

    ret = i2c_reg_read(info, OUT_X_L_XL, odata, sizeof(odata));
    if (!ret) {
        /* Reassemble the data and convert to g */
        ax_raw = odata[0] | (odata[1] << 8);
        ay_raw = odata[2] | (odata[3] << 8);
        az_raw = odata[4] | (odata[5] << 8);
    } else {
        gb_error("%s: read failed %d\n",__func__, ret);
    }
}

void set_accel_scale(struct sensor_accel_info *info)
{
    uint8_t cmd[2] = {
        CTRL_REG6_XL,
        0
    };

    i2c_reg_read(info, CTRL_REG6_XL, &cmd[1], 1);

    /* mask out the accel scale bits: */
    cmd[1] &= 0xFF^(0x3 << 3);
    /* shift in our new scale bits: */
    cmd[1] |= info->a_scale << 3;

    /* Write the accelscale out to the accel */
    I2C_WRITE(info->i2c, cmd, sizeof(cmd));

    /* calculate a new aRes, which relies on a_scale being set correctly: */
    calc_aresolution(info);
}

void set_accel_ODR(struct sensor_accel_info *info, accel_odr aRate)
{
    uint8_t cmd[2] = {
        CTRL_REG6_XL,
        0
    };

    i2c_reg_read(info, CTRL_REG6_XL, &cmd[1], 1);

    /* mask out the accel odr bits: */
    cmd[1] &= 0xFF^(0x7 << 5);
    /* shift in our new odr bits: */
    cmd[1] |= aRate << 5;

    /* Write the accelodr out to the accel */
    I2C_WRITE(info->i2c, cmd, sizeof(cmd));
}

static void *sensor_tx_thread(void *arg)
{
    struct sensor_accel_info *info;
    struct report_info *rinfo;
    struct report_info_data *rinfo_data;
    struct sensor_event_data *event_data;
    struct timespec ts;
    uint16_t payload_size;

    info = arg;
    payload_size = (ACCEL_READING_NUM * sizeof(struct sensor_event_data))
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
            read_accel(info);
#ifdef BATCH_PROCESS_ENABLED
            /*
             * Batch sensor data values and its time_deltas
             * until max fifo event count
            */
#else
            /* Single sensor event data */
            rinfo->readings = ACCEL_READING_NUM;
            event_data->time_delta = 0;
            event_data->data_value[0] = ax_raw;
            event_data->data_value[1] = ay_raw;
            event_data->data_value[2] = az_raw;
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

static int sensor_accel_op_get_sensor_info(struct device *dev,
    uint8_t sensor_id, struct sensor_info *sinfo)
{
    struct sensor_accel_info *info = NULL;

    gb_debug("%s: %d\n",__func__, sensor_id);
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);
    info->sensor_id = sensor_id;

    sinfo->version = ACCEL_VERSION;
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
    sinfo->fifo_rec = ACCEL_READING_NUM;
    sinfo->fifo_mec = sinfo->fifo_rec;
#else
    sinfo->fifo_rec = ACCEL_FIFO_REC;
    sinfo->fifo_mec = ACCEL_FIFO_MEC;
#endif
    sinfo->flags = ACCEL_FLAGS;
    sinfo->scale_int = ACCEL_SCALE_INT;
    sinfo->scale_nano = info->scale_resolution;
    sinfo->offset_int = ACCEL_OFFSET_INT;
    sinfo->offset_nano = ACCEL_OFFSET_NANO;
    sinfo->channels = ACCEL_CHANNEL_SIZE;
    sinfo->type = ACCELEROMETER_ID;
    sinfo->name_len = sizeof(ACCEL_DEVICE_NAME);
    memcpy(&sinfo->name[0], ACCEL_DEVICE_NAME, sizeof(ACCEL_DEVICE_NAME));
    sinfo->vendor_len = sizeof(ACCEL_VENDOR_NAME);
    memcpy(&sinfo->vendor[0], ACCEL_VENDOR_NAME, sizeof(ACCEL_VENDOR_NAME));
    sinfo->string_type_len = 0;

    return OK;
}

static int sensor_accel_op_start_reporting(struct device *dev,
    uint8_t sensor_id, uint64_t sampling_period, uint64_t max_report_latency)
{
    struct sensor_accel_info *info = NULL;
    uint32_t sampling_ms;
    accel_odr aRate;
    int ret;

    gb_debug("%s:\n",__func__);

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    sampling_ms = sampling_period/1000000;

    gb_info("%s: requested sampling period %d mS\n", __func__, sampling_ms);

    do {

        if(sampling_ms <= 4) {
            aRate = A_ODR_238;
            info->latency_ms = 4;
            break;
        }

        if(sampling_ms <= 8) {
            aRate = A_ODR_119;
            info->latency_ms = 8;
            break;
        }

        if(sampling_ms <= 20) {
            aRate = A_ODR_119;
            info->latency_ms = sampling_ms;
            break;
        }

        if(sampling_ms < 100) {
            aRate = A_ODR_50;
            info->latency_ms = sampling_ms;
            break;
        }

        if(sampling_ms >= 100) {
            aRate = A_ODR_10;
            info->latency_ms = sampling_ms;
            break;
        }

        aRate = A_ODR_119;
        info->latency_ms = 18;
    } while(0);

    if (info->latency_ms > TX_PROCESSING_DELAY) {
        info->latency_ms -= TX_PROCESSING_DELAY;
    }
    gb_info("%s: set sampling period %d mS\n", __func__, info->latency_ms);
    set_accel_ODR(info, aRate);
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

static int sensor_accel_op_flush(struct device *dev, uint8_t id)
{
    struct sensor_event_data *event_data;
    struct sensor_accel_info *info;
    struct report_info_data *rinfo_data;
    struct report_info *rinfo;
    struct timespec ts;
    uint16_t payload_size;

    payload_size = (ACCEL_READING_NUM * sizeof(struct sensor_event_data))
                    + (REPORTING_SENSORS * sizeof(struct report_info));

    gb_debug("%s:\n", __func__);

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

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
        read_accel(info);
#ifdef BATCH_PROCESS_ENABLED
        /*
         * Batch sensor data values and its time_deltas
         * until max fifo event count
        */
#else
        /* Single sensor event data */
        rinfo->readings = ACCEL_READING_NUM;
        event_data->time_delta = 0;
        event_data->data_value[0] = ax_raw;
        event_data->data_value[1] = ay_raw;
        event_data->data_value[2] = az_raw;
#endif
        info->callback(info->sensor_id, rinfo_data, payload_size);

        free(rinfo_data);
    }

    return OK;
}

static void sensor_accel_kill_pthread(struct sensor_accel_info *info)
{
    info->should_run = false;

    if (info->tx_thread) {
        pthread_kill(info->tx_thread, 9);
        pthread_join(info->tx_thread, NULL);
        info->tx_thread = 0;
    }
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
    sensor_accel_kill_pthread(info);

    set_accel_ODR(info, A_POWER_DOWN);
    atomic_dec(&txn);

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
    sensor_accel_kill_pthread(info);
    set_accel_ODR(info, A_POWER_DOWN);

    if (!(info->flags & SENSOR_ACCEL_FLAG_OPEN)) {
        return;
    }

    info->flags &= ~SENSOR_ACCEL_FLAG_OPEN;
}

static int sensor_accel_dev_probe(struct device *dev)
{
    struct sensor_accel_info *info;
    int ret = 0;
    uint8_t rbuf[0];
    struct device_resource *i2c_bus;

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

    i2c_bus = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_REGS,
            "i2c_bus");
    if (i2c_bus) {
        info->i2c = up_i2cinitialize(i2c_bus->start);
        if (!info->i2c) {
            dbg("failed to init i2c\n");
            ret = -ENODEV;
            goto init_done;
        }
    } else {
        dbg("i2c bus not configured\n");
            ret = -ENODEV;
            goto init_done;
    }

    ret = I2C_SETADDRESS(info->i2c, 0x6A, 7);
    if (ret) {
        dbg("failed to set i2c address\n");
        goto init_done;
    }

    I2C_SETFREQUENCY(info->i2c, 100000);

    atomic_init(&txn, 0);

    rbuf[0] = 0;
    i2c_reg_read(info, WHO_AM_I_XG, &rbuf[0], 1);

    info->a_scale = A_SCALE_2G;

    uint8_t buf[4] = {
        CTRL_REG5_XL,
        0x38,       /* Enable all axis and don't decimate data in out Registers */
        (A_POWER_DOWN << 5) | (A_BW_AUTO_SCALE),   /* auto BW */
        0           /* Default resolution mode and filtering settings */
    };

    /* Write the data to the accel control registers */
    I2C_WRITE(info->i2c, buf, sizeof(buf));

    set_accel_scale(info);

#ifdef CONFIG_PM
    if (pm_register(&pm_callback) != OK)
    {
        dbg("Failed register to power management!\n");
    }
#endif

    return 0;

init_done:
    return ret;
}

static void sensor_accel_dev_remove(struct device *dev)
{
    struct sensor_accel_info *info = NULL;

    gb_debug("%s:\n",__func__);

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);
    sensor_accel_kill_pthread(info);

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
    .flush = sensor_accel_op_flush,
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

const struct device_driver sensor_lsm9sd1_accel_driver = {
    .type   = DEVICE_TYPE_SENSORS_HW,
    .name   = "sensors_ext_accel",
    .desc   = "ACCEL SENSOR",
    .ops    = &sensor_accel_driver_ops,
};
