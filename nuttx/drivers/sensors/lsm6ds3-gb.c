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
#include <arch/board/mods.h>
#include <arch/atomic.h>
#include <nuttx/gpio.h>
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
#define SENSOR_GYRO_FLAG_OPEN        BIT(1)
#define SENSOR_TEMP_FLAG_OPEN        BIT(2)
#define LSM6DS3_I2C_ADDR             0x6A

/* Accelerometer attributes */
#define ACCEL_DEVICE_NAME  "Accel-LSM6DS3"
#define ACCEL_VENDOR_NAME  "Moto"
#define ACCEL_VERSION      1
#define ACCEL_SAMPLE_RATE  100
#define ACCEL_SCALE_INT    0
#define ACCEL_SCALE_NANO   3
#define ACCEL_OFFSET_INT    0
#define ACCEL_OFFSET_NANO   0
#define TAG_CHANNEL_SIZE    3
#define ACCEL_MAX_RANGE     1024 * 16
#define ACCEL_MIN_DELAY     2000
#define ACCEL_MAX_DELAY     1000000
#define ACCEL_FIFO_REC      0
#define ACCEL_FIFO_MEC      0
#define ACCEL_FLAGS         SENSOR_EXT_FLAG_CONTINUOUS_MODE
#define REPORTING_SENSORS   1

/* Gyroscope attributes */
#define GYRO_DEVICE_NAME  "Gyro-LSM6DS3"
#define GYRO_VENDOR_NAME  "Moto"
#define GYRO_VERSION      1
#define GYRO_SAMPLE_RATE  100
#define GYRO_SCALE_INT    0
#define GYRO_SCALE_NANO   3
#define GYRO_OFFSET_INT    0
#define GYRO_OFFSET_NANO   0
#define GYRO_MAX_RANGE     1024 * 16
#define GYRO_MIN_DELAY     2000
#define GYRO_MAX_DELAY     1000000
#define GYRO_FIFO_REC      0
#define GYRO_FIFO_MEC      0
#define GYRO_FLAGS         SENSOR_EXT_FLAG_CONTINUOUS_MODE

/* Temperature attributes */
#define TEMP_DEVICE_NAME  "Temp-LSM6DS3"
#define TEMP_VENDOR_NAME  "Moto"
#define TEMP_VERSION      1
#define TEMP_SAMPLE_RATE  100
#define TEMP_SCALE_INT    1
#define TEMP_SCALE_NANO   0
#define TEMP_OFFSET_INT    0
#define TEMP_OFFSET_NANO   0
#define TEMP_CHANNEL_SIZE  3
#define TEMP_MAX_RANGE     1024 * 16
#define TEMP_MIN_DELAY     2000
#define TEMP_MAX_DELAY     1000000
#define TEMP_FIFO_REC      0
#define TEMP_FIFO_MEC      0
#define TEMP_FLAGS         SENSOR_EXT_FLAG_CONTINUOUS_MODE

/* acceleromter register offsets */
#define FUNC_CFG_ACCESS     0x01
#define SYNC_TIME_FRAME     0x04
#define FIFO_CTRL1          0x06
#define FIFO_CTRL2          0x07
#define FIFO_CTRL3          0x08
#define FIFO_CTRL4          0x09
#define FIFO_CTRL5          0x0A
#define ORIENT_CFG_G        0x0B
#define INT1_CTRL           0x0D
#define INT2_CTRL           0x0E
#define WHO_AM_I_XG         0x0F
#define CTRL_REG1_XL        0x10
#define CTRL_REG2_G         0x11
#define CTRL_REG3_C         0x12
#define CTRL_REG4_C         0x13
#define CTRL_REG5_C         0x14
#define CTRL_REG6_C         0x15
#define CTRL_REG7_G         0x16
#define CTRL_REG8_XL        0x17
#define CTRL_REG9_XL        0x18
#define CTRL_REG10_C        0x19
#define MASTER_CONFIG       0x1A
#define WAKE_UP_SRC         0x1B
#define TAP_SRC             0x1C
#define D6D_SRC             0x1D
#define STATUS_REG          0x1E
#define OUT_TEMP_L          0x20
#define OUT_TEMP_H          0x21
#define OUT_X_L_G           0x22
#define OUT_X_H_G           0x23
#define OUT_Y_L_G           0x24
#define OUT_Y_H_G           0x25
#define OUT_Z_L_G           0x26
#define OUT_Z_H_G           0x27
#define OUT_X_L_XL          0x28
#define OUT_X_H_XL          0x29
#define OUT_Y_L_XL          0x2A
#define OUT_Y_H_XL          0x2B
#define OUT_Z_L_XL          0x2C
#define OUT_Z_H_XL          0x2D


#define INT_GEN_CFG_XL      0x06
#define INT_GEN_THS_X_XL    0x07
#define INT_GEN_THS_Y_XL    0x08
#define INT_GEN_THS_Z_XL    0x09
#define INT_GEN_DUR_XL      0x0A

#define CTRL_REG3_G         0x12
#define INT_GEN_SRC_G       0x14
#define STATUS_REG_0        0x17
#define CTRL_REG4           0x1E
#define CTRL_REG5_XL        0x1F
#define CTRL_REG6_XL        0x20
#define CTRL_REG7_XL        0x21
#define CTRL_REG8           0x22
#define CTRL_REG9           0x23
#define CTRL_REG10          0x24
#define INT_GEN_SRC_XL      0x26
#define STATUS_REG_1        0x27

#define FIFO_SRC            0x2F
#define INT_GEN_CFG_G       0x30
#define INT_GEN_THS_XH_G    0x31
#define INT_GEN_THS_XL_G    0x32
#define INT_GEN_THS_YH_G    0x33
#define INT_GEN_THS_YL_G    0x34
#define INT_GEN_THS_ZH_G    0x35
#define INT_GEN_THS_ZL_G    0x36
#define INT_GEN_DUR_G       0x37

#define WHO_AM_I_LSM6DS3    0x69

/* CTRL_REG6_XL/FS_XL, Full scale selection of accel*/
enum accel_scale
{
    A_SCALE_2G,  /* 00: +/- 2g  */
    A_SCALE_16G, /* 01: +/- 16g */
    A_SCALE_4G,  /* 10: +/- 4g  */
    A_SCALE_8G   /* 11: +/- 8g  */
};

/* CTRL_REG2_G/FS_G, Full scale selection of gyro */
enum gyro_scale
{
    G_SCALE_245dps,   /* 00: +/- 245dps */
    G_SCALE_500dps,   /* 01: +/- 500dps */
    G_SCALE_1000dps,  /* 10: +/- 1000dps  */
    G_SCALE_2000dps   /* 11: +/- 2000dps  */
};

/* CTRL_REG6_XL/ODR_XL, output data rates of the accel */
typedef enum lsm6ds3_odr
{
    A_POWER_DOWN,   /* Power-down mode (0x0) */
    A_ODR_12P5,     /* 12.5 Hz (0x1) */
    A_ODR_26,       /* 26 Hz (0x2) */
    A_ODR_52,       /* 52 Hz (0x3) */
    A_ODR_104,      /* 104 Hz (0x4) */
    A_ODR_208,      /* 208 Hz (0x5) */
    A_ODR_416,      /* 416 Hz (0x6) */
    A_ODR_833,      /* 833 Hz (0x7) */
    A_ODR_1660,     /* 1.66 kHz (0x8) */
    A_ODR_3330,     /* 3.33 kHz (0x9) */
    A_ODR_6660      /* 6.66 kHz (0xA) */
}lsm6ds3_odr;

typedef enum lsm6ds3_int
{
    INT1_DRDY_XL = 1,
    INT1_DRDY_G = 2,
}lsm6ds3_int;

/* CTRL_REG1_XL/BW_XL bandwiths for low-pass filter of the accel */
enum accel_bw
{
    A_BW_AUTO_SCALE = 0x0,  /* Automatic BW scaling (0x0), TBD */
    A_BW_400 = 0x0,         /* 400 Hz (0x0) */
    A_BW_200 = 0x1,         /* 200 Hz (0x1) */
    A_BW_100 = 0x2,         /* 100 Hz (0x2) */
    A_BW_50 = 0x3           /* 50 Hz (0x3) */
};

static int16_t ax_raw, ay_raw, az_raw; /* x, y, and z axis readings of the accel */
static int16_t gx_raw, gy_raw, gz_raw; /* x, y, and z axis readings of the gyro */
static int16_t temp_raw; /* reading of the temperature */

struct lsm6ds3_sensor_info {
    struct device *dev;
    struct i2c_dev_s *i2c;
    uint32_t flags;
    sensors_ext_event_callback callback;
    uint8_t sensor_id;
    int a_scale;
    int g_scale;
    int a_scale_resolution;
    int g_scale_resolution;
    int latency_ms;
};

/*
 * event fields for accelerometer in bytes
 * [ time_delta (2) | data_value x (4) | data_value y (4) |data_value z (4) ]
*/
struct sensor_event_data {
    uint16_t    time_delta;
    uint32_t    data_value[TAG_CHANNEL_SIZE];
} __packed;

#ifdef BATCH_PROCESS_ENABLED
#define ACCEL_READING_NUM 2
#define GYRO_READING_NUM 2
#define TEMP_READING_NUM 2
#else
#define ACCEL_READING_NUM 1
#define GYRO_READING_NUM 1
#define TEMP_READING_NUM 1
#endif

#define TX_PROCESSING_DELAY     3 /* processing and tx delay (mS)*/
#define LSM6DS3_TOTAL_SENSORS   3

enum lsm6ds3_sensor_type
{
    LSM6DS3_ACCEL_ID = 0x1,
    LSM6DS3_GYRO_ID = 0x2,
    LSM6DS3_TEMP_ID = 0x4
};

// sensor[i][j] means sensor_id is i, its actual sensor id is j
static int sensor_list[LSM6DS3_TOTAL_SENSORS][2] = {{-1, -1}, {-1, -1}, {-1, -1}};
static pthread_t tx_thread = 0;
static int g_users = 0;
static atomic_t txn;       /* Flag to indicate reporting in progress */
static int g_latency_ms = -1;
static sem_t g_thread_sem;

static uint8_t i2c_reg_read(struct lsm6ds3_sensor_info *info,
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

void calc_aresolution(struct lsm6ds3_sensor_info *info)
{
/* Possible accelerometer scales(register settings):
 * 2g (000), 4g (001), 6g (010) 8g (011), 16g (100)
 */
    switch (info->a_scale)
    {
        case A_SCALE_2G:
            info->a_scale_resolution = 598550;
            break;
        case A_SCALE_4G:
            info->a_scale_resolution = 1197101;
            break;
        case A_SCALE_8G:
            info->a_scale_resolution = 2394202; /* (8 * 9.80665)/32768 */
            break;
        case A_SCALE_16G:
            info->a_scale_resolution = 4788404;
            break;
    }
}

void calc_gresolution(struct lsm6ds3_sensor_info *info)
{
/* Possible gryo scales(register settings):
 * 245dps (00), 500 (01), 1000 (10) 2000 (11)
 */
    switch (info->g_scale)
    {
        case G_SCALE_245dps:
            info->g_scale_resolution = 7576806;
            break;
        case G_SCALE_500dps:
            info->g_scale_resolution = 15258789;
            break;
        case G_SCALE_1000dps:
            info->g_scale_resolution = 30517578; /* (1000 dps)/32768 */
            break;
        case G_SCALE_2000dps:
            info->g_scale_resolution = 61035156;
            break;
    }
}

void read_temp_accel_gyro(struct lsm6ds3_sensor_info *info)
{
    int ret;
    int16_t u, v, w;

    /* data to be read from the temp/accel/gyro */
    uint8_t odata[14];

    ret = i2c_reg_read(info, OUT_TEMP_L, odata, sizeof(odata));
    if (!ret) {
        /* Reassemble the data and convert to g */
        temp_raw = odata[0] | (odata[1] << 8);

        u = odata[2] | (odata[3] << 8);
        v = odata[4] | (odata[5] << 8);
        w = odata[6] | (odata[7] << 8);

        gx_raw = v;
        gy_raw = u;
        gz_raw = - w;

        u = odata[8] | (odata[9] << 8);
        v = odata[10] | (odata[11] << 8);
        w = odata[12] | (odata[13] << 8);

        ax_raw = v;
        ay_raw = u;
        az_raw = - w;

    } else {
        dbg("%s: read failed %d\n",__func__, ret);
    }
}

void set_accel_scale(struct lsm6ds3_sensor_info *info)
{
    uint8_t cmd[2] = {
        CTRL_REG1_XL,
        0
    };

    i2c_reg_read(info, CTRL_REG1_XL, &cmd[1], 1);

    /* mask out the accel scale bits: */
    cmd[1] &= 0xFF^(0x3 << 2);
    /* shift in our new scale bits: */
    cmd[1] |= info->a_scale << 2;

    /* Write the accelscale out to the accel */
    I2C_WRITE(info->i2c, cmd, sizeof(cmd));

    /* calculate a new aRes, which relies on a_scale being set correctly: */
    calc_aresolution(info);
}

void set_gyro_scale(struct lsm6ds3_sensor_info *info)
{
    uint8_t cmd[2] = {
        CTRL_REG2_G,
        0
    };

    i2c_reg_read(info, CTRL_REG2_G, &cmd[1], 1);

    /* mask out the gyro scale bits: */
    cmd[1] &= 0xFF^(0x3 << 2);
    /* shift in our new scale bits: */
    cmd[1] |= info->g_scale << 2;

    /* Write the gyro full-scale out to the gyro */
    I2C_WRITE(info->i2c, cmd, sizeof(cmd));

    /* calculate a new aRes, which relies on scale being set correctly: */
    calc_gresolution(info);
}

void set_accel_ODR(struct lsm6ds3_sensor_info *info, lsm6ds3_odr aRate)
{
    uint8_t cmd[2] = {
        CTRL_REG1_XL,
        0
    };

    i2c_reg_read(info, CTRL_REG1_XL, &cmd[1], 1);

    /* mask out the accel odr bits: */
    cmd[1] &= 0xFF^(0xF << 4);
    /* shift in our new odr bits: */
    cmd[1] |= aRate << 4;

    /* Write the accelodr out to the accel */
    I2C_WRITE(info->i2c, cmd, sizeof(cmd));
}

void set_gyro_ODR(struct lsm6ds3_sensor_info *info, lsm6ds3_odr aRate)
{
    uint8_t cmd[2] = {
        CTRL_REG2_G,
        0
    };

    i2c_reg_read(info, CTRL_REG2_G, &cmd[1], 1);

    /* mask out the gyro odr bits: */
    cmd[1] &= 0xFF^(0xF << 4);
    /* shift in our new odr bits: */
    cmd[1] |= aRate << 4;

    /* Write the gyro odr out to the gyro */
    I2C_WRITE(info->i2c, cmd, sizeof(cmd));
}

static void *sensor_tx_thread(void *arg)
{
    struct lsm6ds3_sensor_info *info;
    struct report_info *rinfo;
    struct report_info_data *rinfo_data;
    struct sensor_event_data *event_data;
    struct timespec ts;
    uint16_t payload_size;
    int index;

    info = arg;
    payload_size = (ACCEL_READING_NUM * sizeof(struct sensor_event_data))
                    + (REPORTING_SENSORS * sizeof(struct report_info));

    rinfo_data = malloc(sizeof(struct report_info_data) + payload_size);
    if (!rinfo_data)
        return (void *)-ENOMEM;

    while(1) {
        sem_wait(&g_thread_sem);
        memset(rinfo_data, 0, sizeof(struct report_info_data) + payload_size);

        if (info->callback) {
            rinfo_data->num_sensors_reporting = REPORTING_SENSORS;
            rinfo = rinfo_data->reportinfo;
            rinfo->flags = 0;
            event_data = (struct sensor_event_data *)&rinfo->data_payload[0];

            up_rtc_gettime(&ts);
            rinfo->reference_time = timespec_to_nsec(&ts);
            read_temp_accel_gyro(info);
            for (index = 0; index < LSM6DS3_TOTAL_SENSORS; index++){
                if (sensor_list[index][1] == LSM6DS3_ACCEL_ID){
                    rinfo->id = sensor_list[index][0];
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
                } else if (sensor_list[index][1] == LSM6DS3_GYRO_ID){
                    rinfo->id = sensor_list[index][0];
#ifdef BATCH_PROCESS_ENABLED
                    /*
                     * Batch sensor data values and its time_deltas
                     * until max fifo event count
                    */
#else
                    /* Single sensor event data */
                    rinfo->readings = GYRO_READING_NUM;
                    event_data->time_delta = 0;
                    event_data->data_value[0] = gx_raw;
                    event_data->data_value[1] = gy_raw;
                    event_data->data_value[2] = gz_raw;
#endif
                } else if (sensor_list[index][1] == LSM6DS3_TEMP_ID){
                    rinfo->id = sensor_list[index][0];
#ifdef BATCH_PROCESS_ENABLED
                    /*
                     * Batch sensor data values and its time_deltas
                     * until max fifo event count
                    */
#else
                    /* Single sensor event data */
                    rinfo->readings = TEMP_READING_NUM;
                    event_data->time_delta = 0;
                    event_data->data_value[0] = temp_raw;
                    event_data->data_value[1] = 0;
                    event_data->data_value[2] = 0;
#endif
                }
                vdbg("report sensor: %d, %lld\n", rinfo->id, (long long)rinfo->reference_time);
                info->callback(rinfo->id, rinfo_data, payload_size);
            }
        }

        if (g_users == 0)
            break;

        // TODO the delay time will be the minimal of all the enabled sensors
        //up_mdelay(g_latency_ms);
    }

    free(rinfo_data);

    return NULL;
}

static void lsm6d3s_register_isr(int16_t irq, xcpt_t isr)
{
    gpio_direction_in(irq);
    set_gpio_triggering(irq, IRQ_TYPE_EDGE_RISING);
    gpio_irqattach(irq, isr);
}

int lsm6d3s_isr(int irq, void *context)
{
    sem_post(&g_thread_sem);
    return OK;
}

void enable_INT1(struct lsm6ds3_sensor_info *info, lsm6ds3_int intx)
{
    uint8_t cmd[2] = {
        INT1_CTRL,
        0
    };

    i2c_reg_read(info, INT1_CTRL, &cmd[1], 1);

    /* if INT1_DRDY_G is set, we make Accel reuse this interrupt */
    if ((cmd[1] & INT1_DRDY_G) == INT1_DRDY_G)
        return;
    /* set the new signal on interrupt 1 */
    cmd[1] = intx;

    /* Write the accelodr out to the accel */
    I2C_WRITE(info->i2c, cmd, sizeof(cmd));

}

static int sensor_accel_op_get_sensor_info(struct device *dev,
    uint8_t sensor_id, struct sensor_info *sinfo)
{
    struct lsm6ds3_sensor_info *info = NULL;

    vdbg("%s: %d\n",__func__, sensor_id);
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
    sinfo->scale_nano = info->a_scale_resolution;
    sinfo->offset_int = ACCEL_OFFSET_INT;
    sinfo->offset_nano = ACCEL_OFFSET_NANO;
    sinfo->channels = TAG_CHANNEL_SIZE;
    sinfo->type = ACCELEROMETER_ID;
    sinfo->name_len = sizeof(ACCEL_DEVICE_NAME);
    memcpy(&sinfo->name[0], ACCEL_DEVICE_NAME, sizeof(ACCEL_DEVICE_NAME));
    sinfo->vendor_len = sizeof(ACCEL_VENDOR_NAME);
    memcpy(&sinfo->vendor[0], ACCEL_VENDOR_NAME, sizeof(ACCEL_VENDOR_NAME));
    sinfo->string_type_len = 0;

    return OK;
}

static int sensor_gyro_op_get_sensor_info(struct device *dev,
    uint8_t sensor_id, struct sensor_info *sinfo)
{
    struct lsm6ds3_sensor_info *info = NULL;

    vdbg("%s: %d\n",__func__, sensor_id);
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);
    info->sensor_id = sensor_id;

    sinfo->version = GYRO_VERSION;
    sinfo->max_range = GYRO_MAX_RANGE;
    sinfo->resolution = 1;
    sinfo->min_delay = GYRO_MIN_DELAY;
    sinfo->max_delay = GYRO_MAX_DELAY;
#ifdef BATCH_PROCESS_ENABLED
    /*
     * Example to calculate number of sensor events that can be batched in the
     * payload. Each event consists of time_delta and sensor data fields.
     * Reading size provides the number of sensor data fields reported.
    */
    sinfo->fifo_rec = GYRO_READING_NUM;
    sinfo->fifo_mec = sinfo->fifo_rec;
#else
    sinfo->fifo_rec = GYRO_FIFO_REC;
    sinfo->fifo_mec = GYRO_FIFO_MEC;
#endif
    sinfo->flags = GYRO_FLAGS;
    sinfo->scale_int = GYRO_SCALE_INT;
    sinfo->scale_nano = info->g_scale_resolution;
    sinfo->offset_int = GYRO_OFFSET_INT;
    sinfo->offset_nano = GYRO_OFFSET_NANO;
    sinfo->channels = TAG_CHANNEL_SIZE;
    sinfo->type = GYROSCOPE_ID;
    sinfo->name_len = sizeof(GYRO_DEVICE_NAME);
    memcpy(&sinfo->name[0], GYRO_DEVICE_NAME, sizeof(GYRO_DEVICE_NAME));
    sinfo->vendor_len = sizeof(GYRO_VENDOR_NAME);
    memcpy(&sinfo->vendor[0], GYRO_VENDOR_NAME, sizeof(GYRO_VENDOR_NAME));
    sinfo->string_type_len = 0;

    return OK;
}

static int sensor_temp_op_get_sensor_info(struct device *dev,
    uint8_t sensor_id, struct sensor_info *sinfo)
{
    struct lsm6ds3_sensor_info *info = NULL;

    vdbg("%s: %d\n",__func__, sensor_id);
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);
    info->sensor_id = sensor_id;

    sinfo->version = TEMP_VERSION;
    sinfo->max_range = TEMP_MAX_RANGE;
    sinfo->resolution = 1;
    sinfo->min_delay = TEMP_MIN_DELAY;
    sinfo->max_delay = TEMP_MAX_DELAY;
#ifdef BATCH_PROCESS_ENABLED
    /*
     * Example to calculate number of sensor events that can be batched in the
     * payload. Each event consists of time_delta and sensor data fields.
     * Reading size provides the number of sensor data fields reported.
    */
    sinfo->fifo_rec = TEMP_READING_NUM;
    sinfo->fifo_mec = sinfo->fifo_rec;
#else
    sinfo->fifo_rec = TEMP_FIFO_REC;
    sinfo->fifo_mec = TEMP_FIFO_MEC;
#endif
    sinfo->flags = TEMP_FLAGS;
    sinfo->scale_int = TEMP_SCALE_INT;
    sinfo->scale_nano = 488281;//sensitivity of +16 LSB/degC, 10^9 * 16/32768
    sinfo->offset_int = TEMP_OFFSET_INT;
    sinfo->offset_nano = TEMP_OFFSET_NANO;
    sinfo->channels = TAG_CHANNEL_SIZE;
    sinfo->type = AMBIENT_TEMPERATURE_ID;
    sinfo->name_len = sizeof(TEMP_DEVICE_NAME);
    memcpy(&sinfo->name[0], TEMP_DEVICE_NAME, sizeof(TEMP_DEVICE_NAME));
    sinfo->vendor_len = sizeof(TEMP_VENDOR_NAME);
    memcpy(&sinfo->vendor[0], TEMP_VENDOR_NAME, sizeof(TEMP_VENDOR_NAME));
    sinfo->string_type_len = 0;

    return OK;
}

static int sensor_accel_op_start_reporting(struct device *dev,
    uint8_t sensor_id, uint64_t sampling_period, uint64_t max_report_latency)
{
    struct lsm6ds3_sensor_info *info = NULL;
    uint32_t sampling_ms;
    lsm6ds3_odr aRate;
    int ret;

    vdbg("%s:\n",__func__);

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    sampling_ms = sampling_period/1000000;

    vdbg("%s: requested sampling period %d mS\n", __func__, sampling_ms);

    do {

        if(sampling_ms <= 3) {
            aRate = A_ODR_416;
            info->latency_ms = 3;
            break;
        }

        if(sampling_ms <= 5) {
            aRate = A_ODR_208;
            info->latency_ms = 5;
            break;
        }

        if(sampling_ms <= 10) {
            aRate = A_ODR_104;
            info->latency_ms = sampling_ms;
            break;
        }

        if(sampling_ms <= 20) {
            aRate = A_ODR_52;
            info->latency_ms = sampling_ms;
            break;
        }

        if(sampling_ms < 80) {
            aRate = A_ODR_26;
            info->latency_ms = sampling_ms;
            break;
        }

        if(sampling_ms >= 80) {
            aRate = A_ODR_12P5;
            info->latency_ms = sampling_ms;
            break;
        }

        aRate = A_ODR_208;
        info->latency_ms = 5;
    } while(0);

    if (info->latency_ms > TX_PROCESSING_DELAY) {
        info->latency_ms -= TX_PROCESSING_DELAY;
    }
    if (g_latency_ms == -1 || info->latency_ms < g_latency_ms)
        g_latency_ms = info->latency_ms;
    vdbg("%s: set sampling period %d mS\n", __func__, g_latency_ms);
    lsm6d3s_register_isr(GPIO_MODS_IMU_INT1, lsm6d3s_isr);
    enable_INT1(info, INT1_DRDY_XL);
    set_accel_ODR(info, aRate);
    g_users++; // new thread user arrives
    sensor_list[g_users-1][0] = sensor_id;
    sensor_list[g_users-1][1] = LSM6DS3_ACCEL_ID;
    if (!tx_thread && (g_users == 1)) {
        ret = pthread_create(&tx_thread, NULL, sensor_tx_thread, info);
        if (ret) {
            lldbg("ERROR: Failed to create tx thread: %s.\n", strerror(errno));
            tx_thread = 0;
            g_users = 0;
            g_latency_ms = -1;
            return ret;
        }
    }
    atomic_inc(&txn);

    return OK;
}

static int sensor_gyro_op_start_reporting(struct device *dev,
    uint8_t sensor_id, uint64_t sampling_period, uint64_t max_report_latency)
{
    struct lsm6ds3_sensor_info *info = NULL;
    uint32_t sampling_ms;
    lsm6ds3_odr aRate;
    int ret;

    vdbg("%s:\n",__func__);

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    sampling_ms = sampling_period/1000000;

    vdbg("%s: requested sampling period %d mS\n", __func__, sampling_ms);

    do {

        if(sampling_ms <= 3) {
            aRate = A_ODR_416;
            info->latency_ms = 3;
            break;
        }

        if(sampling_ms <= 5) {
            aRate = A_ODR_208;
            info->latency_ms = 5;
            break;
        }

        if(sampling_ms <= 10) {
            aRate = A_ODR_104;
            info->latency_ms = sampling_ms;
            break;
        }

        if(sampling_ms <= 20) {
            aRate = A_ODR_52;
            info->latency_ms = sampling_ms;
            break;
        }

        if(sampling_ms < 80) {
            aRate = A_ODR_26;
            info->latency_ms = sampling_ms;
            break;
        }

        if(sampling_ms >= 80) {
            aRate = A_ODR_12P5;
            info->latency_ms = sampling_ms;
            break;
        }

        aRate = A_ODR_208;
        info->latency_ms = 5;
    } while(0);

    if (info->latency_ms > TX_PROCESSING_DELAY) {
        info->latency_ms -= TX_PROCESSING_DELAY;
    }
    if (g_latency_ms == -1 || info->latency_ms < g_latency_ms)
        g_latency_ms = info->latency_ms;
    vdbg("%s: set sampling period %d mS\n", __func__, g_latency_ms);
    lsm6d3s_register_isr(GPIO_MODS_IMU_INT1, lsm6d3s_isr);
    enable_INT1(info, INT1_DRDY_G);
    set_gyro_ODR(info, aRate);
    g_users++; // new thread user arrives
    sensor_list[g_users-1][0] = sensor_id;
    sensor_list[g_users-1][1] = LSM6DS3_GYRO_ID;

    if (!tx_thread) {
        ret = pthread_create(&tx_thread, NULL, sensor_tx_thread, info);
        if (ret) {
            lldbg("ERROR: Failed to create tx thread: %s.\n", strerror(errno));
            tx_thread = 0;
            g_users = 0;
            g_latency_ms = -1;
            return ret;
        }
    }
    atomic_inc(&txn);

    return OK;
}

static int sensor_temp_op_start_reporting(struct device *dev,
    uint8_t sensor_id, uint64_t sampling_period, uint64_t max_report_latency)
{
    struct lsm6ds3_sensor_info *info = NULL;
    uint32_t sampling_ms;
    int ret;

    vdbg("%s:\n",__func__);

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    sampling_ms = sampling_period/1000000;

    vdbg("%s: requested sampling period %d mS\n", __func__, sampling_ms);

    info->latency_ms = sampling_ms;

    if (info->latency_ms > TX_PROCESSING_DELAY) {
        info->latency_ms -= TX_PROCESSING_DELAY;
    }

    if (g_latency_ms == -1 || info->latency_ms < g_latency_ms)
        g_latency_ms = info->latency_ms;
    g_users++; // new thread user arrives
    sensor_list[g_users-1][0] = sensor_id;
    sensor_list[g_users-1][1] = LSM6DS3_TEMP_ID;

    if (!tx_thread) {
        ret = pthread_create(&tx_thread, NULL, sensor_tx_thread, info);
        if (ret) {
            lldbg("ERROR: Failed to create tx thread: %s.\n", strerror(errno));
            tx_thread = 0;
            g_users = 0;
            g_latency_ms = -1;
            return ret;
        }
    }
    atomic_inc(&txn);

    return OK;
}

static int sensor_accel_op_flush(struct device *dev, uint8_t id)
{
    struct sensor_event_data *event_data;
    struct lsm6ds3_sensor_info *info;
    struct report_info_data *rinfo_data;
    struct report_info *rinfo;
    struct timespec ts;
    uint16_t payload_size;

    payload_size = (ACCEL_READING_NUM * sizeof(struct sensor_event_data))
                    + (REPORTING_SENSORS * sizeof(struct report_info));

    vdbg("%s:\n", __func__);

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
        vdbg("[%u.%03u]\n", ts.tv_sec, (ts.tv_nsec / 1000000));
        rinfo->flags = REPORT_INFO_FLAG_FLUSHING | REPORT_INFO_FLAG_FLUSH_COMPLETE;
        read_temp_accel_gyro(info);
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

static int sensor_gyro_op_flush(struct device *dev, uint8_t id)
{
    struct sensor_event_data *event_data;
    struct lsm6ds3_sensor_info *info;
    struct report_info_data *rinfo_data;
    struct report_info *rinfo;
    struct timespec ts;
    uint16_t payload_size;

    payload_size = (GYRO_READING_NUM * sizeof(struct sensor_event_data))
                    + (REPORTING_SENSORS * sizeof(struct report_info));

    vdbg("%s:\n", __func__);

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
        vdbg("[%u.%03u]\n", ts.tv_sec, (ts.tv_nsec / 1000000));
        rinfo->flags = REPORT_INFO_FLAG_FLUSHING | REPORT_INFO_FLAG_FLUSH_COMPLETE;
        read_temp_accel_gyro(info);
#ifdef BATCH_PROCESS_ENABLED
        /*
         * Batch sensor data values and its time_deltas
         * until max fifo event count
        */
#else
        /* Single sensor event data */
        rinfo->readings = GYRO_READING_NUM;
        event_data->time_delta = 0;
        event_data->data_value[0] = gx_raw;
        event_data->data_value[1] = gy_raw;
        event_data->data_value[2] = gz_raw;
#endif
        info->callback(info->sensor_id, rinfo_data, payload_size);

        free(rinfo_data);
    }

    return OK;
}

static int sensor_temp_op_flush(struct device *dev, uint8_t id)
{
    struct sensor_event_data *event_data;
    struct lsm6ds3_sensor_info *info;
    struct report_info_data *rinfo_data;
    struct report_info *rinfo;
    struct timespec ts;
    uint16_t payload_size;

    payload_size = (TEMP_READING_NUM * sizeof(struct sensor_event_data))
                    + (REPORTING_SENSORS * sizeof(struct report_info));

    vdbg("%s:\n", __func__);

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
        vdbg("[%u.%03u]\n", ts.tv_sec, (ts.tv_nsec / 1000000));
        rinfo->flags = REPORT_INFO_FLAG_FLUSHING | REPORT_INFO_FLAG_FLUSH_COMPLETE;
        read_temp_accel_gyro(info);
#ifdef BATCH_PROCESS_ENABLED
        /*
         * Batch sensor data values and its time_deltas
         * until max fifo event count
        */
#else
        /* Single sensor event data */
        rinfo->readings = TEMP_READING_NUM;
        event_data->time_delta = 0;
        event_data->data_value[0] = temp_raw;
#endif
        info->callback(info->sensor_id, rinfo_data, payload_size);

        free(rinfo_data);
    }

    return OK;
}

static void sensor_kill_pthread(struct lsm6ds3_sensor_info *info)
{
    if (g_users > 0)
        g_users--;
    if (tx_thread && g_users == 0) {
        pthread_kill(tx_thread, 9);
        pthread_join(tx_thread, NULL);
        tx_thread = 0;
        g_latency_ms = -1;
    }
}

static int sensor_accel_op_stop_reporting(struct device *dev,
    uint8_t sensor_id)
{
    struct lsm6ds3_sensor_info *info = NULL;
    int index;

    vdbg("%s:\n",__func__);

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);
    for (index = 0; index < LSM6DS3_TOTAL_SENSORS; index++){
        if (sensor_list[index][1] == LSM6DS3_ACCEL_ID){
            sensor_list[index][0] = -1;
            sensor_list[index][1] = -1;
        }
    }
    sensor_kill_pthread(info);

    set_accel_ODR(info, A_POWER_DOWN);
    atomic_dec(&txn);

    return OK;
}

static int sensor_gyro_op_stop_reporting(struct device *dev,
    uint8_t sensor_id)
{
    struct lsm6ds3_sensor_info *info = NULL;
    int index;

    vdbg("%s:\n",__func__);

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);
    for (index = 0; index < LSM6DS3_TOTAL_SENSORS; index++){
        if (sensor_list[index][1] == LSM6DS3_GYRO_ID){
            sensor_list[index][0] = -1;
            sensor_list[index][1] = -1;
        }
    }
    sensor_kill_pthread(info);
    set_gyro_ODR(info, A_POWER_DOWN);
    atomic_dec(&txn);

    return OK;
}

static int sensor_temp_op_stop_reporting(struct device *dev,
    uint8_t sensor_id)
{
    struct lsm6ds3_sensor_info *info = NULL;
    int index;

    vdbg("%s:\n",__func__);

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);
    for (index = 0; index < LSM6DS3_TOTAL_SENSORS; index++){
        if (sensor_list[index][1] == LSM6DS3_TEMP_ID){
            sensor_list[index][0] = -1;
            sensor_list[index][1] = -1;
        }
    }
    sensor_kill_pthread(info);
    atomic_dec(&txn);

    return OK;
}

static int sensor_op_register_callback(struct device *dev,
                                        uint8_t se_id,
                                        sensors_ext_event_callback callback)
{
    struct lsm6ds3_sensor_info *info = NULL;

    vdbg("%s:\n", __func__);
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
    struct lsm6ds3_sensor_info *info = NULL;

    vdbg("%s:\n", __func__);
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (info->flags & SENSOR_ACCEL_FLAG_OPEN) {
        return -EBUSY;
    }
    sem_init(&g_thread_sem, 0, 0);

    info->flags |= SENSOR_ACCEL_FLAG_OPEN;

    return 0;
}

static int sensor_gyro_dev_open(struct device *dev)
{
    struct lsm6ds3_sensor_info *info = NULL;

    vdbg("%s:\n", __func__);
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (info->flags & SENSOR_GYRO_FLAG_OPEN) {
        return -EBUSY;
    }
    sem_init(&g_thread_sem, 0, 0);

    info->flags |= SENSOR_GYRO_FLAG_OPEN;

    return 0;
}

static int sensor_temp_dev_open(struct device *dev)
{
    struct lsm6ds3_sensor_info *info = NULL;

    vdbg("%s:\n", __func__);
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (info->flags & SENSOR_TEMP_FLAG_OPEN) {
        return -EBUSY;
    }

    info->flags |= SENSOR_TEMP_FLAG_OPEN;

    return 0;
}

static void sensor_accel_dev_close(struct device *dev)
{
    struct lsm6ds3_sensor_info *info = NULL;

    vdbg("%s:\n",__func__);

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);
    sensor_kill_pthread(info);
    set_accel_ODR(info, A_POWER_DOWN);

    if (!(info->flags & SENSOR_ACCEL_FLAG_OPEN)) {
        return;
    }

    info->flags &= ~SENSOR_ACCEL_FLAG_OPEN;
}

static void sensor_gyro_dev_close(struct device *dev)
{
    struct lsm6ds3_sensor_info *info = NULL;

    vdbg("%s:\n",__func__);

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);
    sensor_kill_pthread(info);
    set_gyro_ODR(info, A_POWER_DOWN);

    if (!(info->flags & SENSOR_GYRO_FLAG_OPEN)) {
        return;
    }

    info->flags &= ~SENSOR_GYRO_FLAG_OPEN;
}

static void sensor_temp_dev_close(struct device *dev)
{
    struct lsm6ds3_sensor_info *info = NULL;

    vdbg("%s:\n",__func__);

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);
    sensor_kill_pthread(info);

    if (!(info->flags & SENSOR_TEMP_FLAG_OPEN)) {
        return;
    }

    info->flags &= ~SENSOR_TEMP_FLAG_OPEN;
}

static int sensor_dev_probe(struct device *dev)
{
    struct lsm6ds3_sensor_info *info;
    int ret = 0;
    uint8_t rbuf[1];
    struct device_resource *i2c_bus;

    vdbg("%s:\n",__func__);
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

    ret = I2C_SETADDRESS(info->i2c, LSM6DS3_I2C_ADDR, 7);
    if (ret) {
        dbg("failed to set i2c address\n");
        goto init_done;
    }

    I2C_SETFREQUENCY(info->i2c, 400000);

    atomic_init(&txn, 0);

    rbuf[0] = 0;
    i2c_reg_read(info, WHO_AM_I_XG, &rbuf[0], 1);
    vdbg("WHO_AM_I register is 0x%2x.\n", rbuf[0]);
    if (rbuf[0] != WHO_AM_I_LSM6DS3)
    {
        dbg("Not LSM6DS3 chipset, WHO_AM_I register is 0x%2x.\n", rbuf[0]);
    }

    info->a_scale = A_SCALE_2G;
    info->g_scale = G_SCALE_245dps;

    uint8_t buf[3] = {
        CTRL_REG9_XL,
        0x38,       /* Enable all accel axis */
        0x38        /* Enable all gyro axis */
    };

    /* Write the data to the accel and gyro control registers */
    I2C_WRITE(info->i2c, buf, sizeof(buf));

    buf[0] = CTRL_REG1_XL;
    buf[1] = (A_POWER_DOWN << 4) | (A_BW_AUTO_SCALE);   /* auto BW */
    buf[2] = (A_POWER_DOWN << 4); /* gyro */
    /* Write the data to the accel and gyro control registers */
    I2C_WRITE(info->i2c, buf, sizeof(buf));

    set_accel_scale(info);
    set_gyro_scale(info);

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
    struct lsm6ds3_sensor_info *info = NULL;

    vdbg("%s:\n",__func__);

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);
    sensor_kill_pthread(info);

    if (info->flags & SENSOR_ACCEL_FLAG_OPEN) {
        sensor_accel_dev_close(dev);
    }
    info->flags = 0;

    free(info);
    device_set_private(dev, NULL);
}

static void sensor_gyro_dev_remove(struct device *dev)
{
    struct lsm6ds3_sensor_info *info = NULL;

    vdbg("%s:\n",__func__);

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);
    sensor_kill_pthread(info);

    if (info->flags & SENSOR_GYRO_FLAG_OPEN) {
        sensor_gyro_dev_close(dev);
    }
    info->flags = 0;

    free(info);
    device_set_private(dev, NULL);
}

static void sensor_temp_dev_remove(struct device *dev)
{
    struct lsm6ds3_sensor_info *info = NULL;

    vdbg("%s:\n",__func__);

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);
    sensor_kill_pthread(info);

    if (info->flags & SENSOR_TEMP_FLAG_OPEN) {
        sensor_temp_dev_close(dev);
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
    .register_callback = sensor_op_register_callback,
};

static struct device_driver_ops sensor_accel_driver_ops = {
    .probe    = sensor_dev_probe,
    .remove   = sensor_accel_dev_remove,
    .open     = sensor_accel_dev_open,
    .close    = sensor_accel_dev_close,
    .type_ops = &sensor_accel_type_ops,
};

const struct device_driver sensor_lsm6ds3_accel_driver = {
    .type   = DEVICE_TYPE_SENSORS_HW,
    .name   = "sensors_ext_accel",
    .desc   = "ACCEL SENSOR",
    .ops    = &sensor_accel_driver_ops,
};

static struct device_sensors_ext_type_ops sensor_gyro_type_ops = {
    .get_sensor_info = sensor_gyro_op_get_sensor_info,
    .start_reporting = sensor_gyro_op_start_reporting,
    .flush = sensor_gyro_op_flush,
    .stop_reporting = sensor_gyro_op_stop_reporting,
    .register_callback = sensor_op_register_callback,
};

static struct device_driver_ops sensor_gyro_driver_ops = {
    .probe    = sensor_dev_probe,
    .remove   = sensor_gyro_dev_remove,
    .open     = sensor_gyro_dev_open,
    .close    = sensor_gyro_dev_close,
    .type_ops = &sensor_gyro_type_ops,
};

const struct device_driver sensor_lsm6ds3_gyro_driver = {
    .type   = DEVICE_TYPE_SENSORS_HW,
    .name   = "sensors_ext_gyro",
    .desc   = "GYRO SENSOR",
    .ops    = &sensor_gyro_driver_ops,
};

static struct device_sensors_ext_type_ops sensor_temp_type_ops = {
    .get_sensor_info = sensor_temp_op_get_sensor_info,
    .start_reporting = sensor_temp_op_start_reporting,
    .flush = sensor_temp_op_flush,
    .stop_reporting = sensor_temp_op_stop_reporting,
    .register_callback = sensor_op_register_callback,
};

static struct device_driver_ops sensor_temp_driver_ops = {
    .probe    = sensor_dev_probe,
    .remove   = sensor_temp_dev_remove,
    .open     = sensor_temp_dev_open,
    .close    = sensor_temp_dev_close,
    .type_ops = &sensor_temp_type_ops,
};

const struct device_driver sensor_lsm6ds3_temp_driver = {
    .type   = DEVICE_TYPE_SENSORS_HW,
    .name   = "sensors_ext_temp",
    .desc   = "TEMP SENSOR",
    .ops    = &sensor_temp_driver_ops,
};
