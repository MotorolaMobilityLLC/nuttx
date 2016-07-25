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

#include <nuttx/clock.h>
#include <nuttx/config.h>
#include <nuttx/device.h>
#include <nuttx/device_battery_level.h>
#include <nuttx/device_battery_temp.h>
#include <nuttx/device_battery_good.h>
#include <nuttx/device_battery_voltage.h>
#include <nuttx/gpio.h>
#include <nuttx/i2c.h>
#include <nuttx/kmalloc.h>
#include <nuttx/power/battery.h>
#include <nuttx/power/pm.h>
#include <nuttx/util.h>
#include <nuttx/wqueue.h>

#include <sys/types.h>

#include <debug.h>
#include <errno.h>
#include <limits.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdint.h>

#include "max17050_config.h"
#include "max17050_thermistor.h"

#define MAX17050_I2C_ADDR               0x36

#define MAX17050_REG_STATUS             0x00
#define MAX17050_REG_VALRT              0x01
#define MAX17050_REG_TALRT              0x02
#define MAX17050_REG_SALRT              0x03
#define MAX17050_REG_REP_CAP            0x05
#define MAX17050_REG_REP_SOC            0x06
#define MAX17050_REG_TEMP               0x08
#define MAX17050_REG_VCELL              0x09
#define MAX17050_REG_CURRENT            0x0A
#define MAX17050_REG_AVERAGE_CURRENT    0x0B
#define MAX17050_REG_REM_CAP            0x0F
#define MAX17050_REG_FULL_CAP           0x10
#define MAX17050_REG_Q_RESIDUAL_00      0x12
#define MAX17050_REG_FULL_SOC_THR       0x13
#define MAX17050_REG_CYCLES             0x17
#define MAX17050_REG_DESIGN_CAP         0x18
#define MAX17050_REG_MAX_VOLT           0x1B
#define MAX17050_REG_CONFIG             0x1D
#define MAX17050_REG_ICHG_TERM          0x1E
#define MAX17050_REG_CONFIG_VER         0x20    /* Reserved Register */
#define MAX17050_REG_DEV_NAME           0x21
#define MAX17050_REG_Q_RESIDUAL_10      0x22
#define MAX17050_REG_FULL_CAP_NOM       0x23
#define MAX17050_REG_LEARN_CFG          0x28
#define MAX17050_REG_FILTER_CFG         0x29
#define MAX17050_REG_RELAX_CFG          0x2A
#define MAX17050_REG_MISC_CFG           0x2B
#define MAX17050_REG_TGAIN              0x2C
#define MAX17050_REG_TOFF               0x2D
#define MAX17050_REG_CGAIN              0x2E
#define MAX17050_REG_COFF               0x2F
#define MAX17050_REG_Q_RESIDUAL_20      0x32
#define MAX17050_REG_IAVG_EMPTY         0x36
#define MAX17050_REG_RCOMP0             0x38
#define MAX17050_REG_TEMPCO             0x39
#define MAX17050_REG_V_EMPTY            0x3A
#define MAX17050_REG_Q_RESIDUAL_30      0x42
#define MAX17050_REG_DQ_ACC             0x45
#define MAX17050_REG_DP_ACC             0x46
#define MAX17050_REG_VFSOC0             0x48
#define MAX17050_REG_QH0                0x4C
#define MAX17050_REG_QH                 0x4D
#define MAX17050_REG_VFSOC0_QH0_ACCESS  0x60
#define MAX17050_REG_MODEL_ACCESS_1     0x62
#define MAX17050_REG_MODEL_ACCESS_2     0x63
#define MAX17050_REG_CELL_DATA_START    0x80
#define MAX17050_REG_VFSOC              0xFF

#define MAX17050_IC_VERSION             0x00AC

#define MAX17050_STATUS_POR             BIT(1)  /* Power-On Reset */
#define MAX17050_STATUS_BST             BIT(3)  /* Battery Status */
#define MAX17050_STATUS_VMN             BIT(8)  /* Min Valrt */
#define MAX17050_STATUS_TMN             BIT(9)  /* Min Talrt */
#define MAX17050_STATUS_SMN             BIT(10) /* Min SOCalrt */
#define MAX17050_STATUS_VMX             BIT(12) /* Max Valrt */
#define MAX17050_STATUS_TMX             BIT(13) /* Max Talrt */
#define MAX17050_STATUS_SMX             BIT(14) /* Max SOCalrt */

#define MAX17050_CONFIG_AEN             BIT(2)  /* Alert enable */

#define MAX17050_MODEL_UNLOCK_1         0x59
#define MAX17050_MODEL_UNLOCK_2         0xC4
#define MAX17050_MODEL_LOCK_1           0x00
#define MAX17050_MODEL_LOCK_2           0x00
#define MAX17050_VFSOC0_QH0_UNLOCK      0x80
#define MAX17050_VFSOC0_QH0_LOCK        0x00
#define MAX17050_CYCLES_INIT_VALUE      96
#define MAX17050_REM_CAP_DIV            25600
#define MAX17050_DQ_ACC_DIV             16
#define MAX17050_DP_ACC                 0x0C80

#define MAX17050_CGAIN_REVERSED_SENSE_R 0xC000
#define MAX17050_CGAIN_DEFAULT          0x4000

/* Priority to report to PM framework when reporting activity */
#define PM_ACTIVITY                     10

/* Min battery voltage for fuel gauge to be functional */
#define MAX17050_MIN_BATTERY_VOLTAGE    2500    /* in mV */

/* Delay afer fuel gauge power up */
#define MAX17050_DELAY_AFTER_POWERUP    750     /* in mS */

/* Fuel gauge interrupt is only used by the device drivers */
#define MAX17050_USE_ALRT               (defined(CONFIG_BATTERY_TEMP_DEVICE_MAX17050)       ||\
                                         defined (CONFIG_BATTERY_LEVEL_DEVICE_MAX17050)     ||\
                                         defined (CONFIG_BATTERY_VOLTAGE_DEVICE_MAX17050))

/* Delay to wait for the ALERT line to deassert after conditions go away */
#define MAX17050_ALERT_DEASSERT_DELAY   500     /* in mS */

/* Polling interval if ALERT interrupt is not used */
#define MAX17050_ALERT_POLLING_INTERVAL 1000    /* in mS */

extern const struct max17050_config max17050_cfg;

struct max17050_dev_s
{
    /* The common part of the battery driver visible to the upper-half driver */

    FAR const struct battery_operations_s *ops;
    sem_t batsem;

    /* Data fields specific to the lower half MAX17050 driver follow */

    FAR struct i2c_dev_s *i2c;
    bool initialized;
    struct work_s batt_good_work;
    struct device *batt_good_dev;
#if MAX17050_USE_ALRT
    struct work_s alrt_work;
    int alrt_gpio;
    bool min_max_valrt;
    bool min_max_talrt;
    bool min_max_salrt;
    int min_valrt;
    int max_valrt;
    int min_talrt;
    int max_talrt;
    int min_salrt;
    int max_salrt;
#endif
#ifdef CONFIG_BATTERY_TEMP_DEVICE_MAX17050
    sem_t battery_temp_sem;
    batt_temp_limits temp_limits_cb;
    void *temp_limits_arg;
    batt_temp_available temp_available_cb;
    void *temp_available_arg;
#endif
#ifdef CONFIG_BATTERY_LEVEL_DEVICE_MAX17050
    sem_t battery_level_sem;
    batt_level_limits level_limits_cb;
    void *level_limits_arg;
    batt_level_available level_available_cb;
    void *level_available_arg;
    bool max_soc_alrt;
    int eoc_counter;
    struct work_s state_changed_work;
    struct work_s eoc_work;
#endif
#ifdef CONFIG_BATTERY_VOLTAGE_DEVICE_MAX17050
    sem_t battery_voltage_sem;
    batt_voltage_limits voltage_limits_cb;
    void *voltage_limits_arg;
    batt_voltage_available voltage_available_cb;
    void *voltage_available_arg;
#endif
};

/* Forward declarations */
static void max17050_set_initialized(FAR struct max17050_dev_s *priv, bool state);
#if MAX17050_USE_ALRT
static void max17050_alrt_worker(FAR void *arg);
#endif
#if defined(CONFIG_BATTERY_LEVEL_DEVICE_MAX17050) && CONFIG_BATTERY_LEVEL_DEVICE_MAX17050_EMPTY_VOLTAGE
static void max17050_battery_level_set_empty(struct max17050_dev_s *priv);
static void max17050_battery_level_empty_detection(struct max17050_dev_s *priv, bool enable);
#endif

/*
 * Global pointer is needed because Nuttx IRQs do not provide a pointer to
 * driver private data
 */
static struct max17050_dev_s* g_max17050_dev;

/* Battery status that is set externally */
static enum battery_status_e g_status = BATTERY_DISCHARGING;

#if MAX17050_USE_ALRT && defined (CONFIG_PM)
static int pm_prepare(struct pm_callback_s *cb, enum pm_state_e state)
{
#ifdef CONFIG_BATTERY_LEVEL_DEVICE_MAX17050
    /* Do not allow STANDBY while charging */
    if (state >= PM_STANDBY && g_status == BATTERY_CHARGING)
        return -EBUSY;
#endif
    return OK;
}

static struct pm_callback_s pm_callback =
{
  .prepare = pm_prepare,
};
#endif

#ifdef CONFIG_BATTERY_LEVEL_DEVICE_MAX17050
static void max17050_battery_level_state_changed_worker(FAR void *arg);
#endif

int battery_set_status(enum battery_status_e status)
{
    /* Will use only BATTERY_CHARGING and BATTERY_DISCHARGING states */
    if (status != BATTERY_CHARGING)
        status = BATTERY_DISCHARGING;

    if (g_status != status) {
        g_status = status;
#ifdef CONFIG_BATTERY_LEVEL_DEVICE_MAX17050
        if (work_available(&g_max17050_dev->state_changed_work))
            work_queue(LPWORK, &g_max17050_dev->state_changed_work,
                max17050_battery_level_state_changed_worker, g_max17050_dev, 0);
#endif
    };

    return 0;
}

static int max17050_reg_read(FAR struct max17050_dev_s *priv, uint8_t reg)
{
    uint16_t reg_val;
    int ret;

    ret = I2C_WRITEREAD(priv->i2c, (uint8_t *)&reg, sizeof(reg),
                        (uint8_t *)&reg_val, sizeof(reg_val));
    return ret ? ret : reg_val;
}

static int max17050_reg_read_retry(FAR struct max17050_dev_s *priv, uint8_t reg)
{
    int tries = 8;
    uint16_t reg_val;
    int ret;
    do {
        ret = I2C_WRITEREAD(priv->i2c, (uint8_t *)&reg, sizeof(reg),
                        (uint8_t *)&reg_val, sizeof(reg_val));
        if (!ret) {
            break;
        }
    } while (--tries);
    return ret ? ret : reg_val;
}

static int max17050_reg_write(FAR struct max17050_dev_s *priv, uint8_t reg, uint16_t val)
{
    uint8_t buf[3];

    buf[0] = reg;
    buf[1] = val & 0xFF;
    buf[2] = val >> 8;

    return I2C_WRITE(priv->i2c, buf, sizeof(buf));
}

static int max17050_reg_write_retry(FAR struct max17050_dev_s *priv, uint16_t reg, uint16_t val)
{
    int tries = 8;
    int ret;

    do {
        ret = max17050_reg_write(priv, reg, val);
        if (!ret) {
            break;
        }
    } while (--tries);

    return ret;
}

static int max17050_reg_write_verify(FAR struct max17050_dev_s *priv, uint16_t reg, uint16_t val)
{
    int tries = 8;

    do {
        max17050_reg_write(priv, reg, val);
        if (max17050_reg_read(priv, reg) == val) {
            break;
        } else {
            continue;
        }
    } while (--tries);

    return tries? 0 : -EIO;
}

static int max17050_reg_modify(FAR struct max17050_dev_s *priv, uint16_t reg,
                               uint16_t mask, uint16_t set)
{
    int ret;
    uint16_t new_value;

    ret = max17050_reg_read(priv, reg);
    if (ret < 0)
        return ret;

    new_value = ret & ~mask;
    new_value |= set & mask;

    return max17050_reg_write(priv, reg, new_value);
}

static int max17050_reg_modify_retry(FAR struct max17050_dev_s *priv, uint16_t reg,
                               uint16_t mask, uint16_t set)
{
    int tries = 8;
    int ret;
    do {
        ret = max17050_reg_modify(priv, reg, mask, set);
        if (!ret) {
            break;
        }
    } while (--tries);
    return ret;
}

static int max17050_online(struct battery_dev_s *dev, bool *status)
{
    FAR struct max17050_dev_s *priv = (FAR struct max17050_dev_s *)dev;
    int ret;

    if (!priv->initialized)
        return -EAGAIN;

    ret = max17050_reg_read_retry(priv, MAX17050_REG_STATUS);
    if (ret < 0)
        return -ENODEV;

    // BST is 0 when battery is present and 1 when battery is removed
    *status = (ret & MAX17050_STATUS_BST) != MAX17050_STATUS_BST;

    return OK;
}

static int max17050_voltage(struct battery_dev_s *dev, b16_t *voltage)
{
    FAR struct max17050_dev_s *priv = (FAR struct max17050_dev_s *)dev;
    int ret;

    if (!priv->initialized)
        return -EAGAIN;

    ret = max17050_reg_read_retry(priv, MAX17050_REG_VCELL);
    if (ret < 0)
        return -ENODEV;

    *voltage = ret * 625 / 8;

    return OK;
}

static int max17050_capacity(struct battery_dev_s *dev, b16_t *capacity)
{
    FAR struct max17050_dev_s *priv = (FAR struct max17050_dev_s *)dev;
    b16_t value;
    int ret;

    if (!priv->initialized)
        return -EAGAIN;

    ret = max17050_reg_read_retry(priv, MAX17050_REG_REP_SOC);
    if (ret < 0)
        return -ENODEV;

    /* high byte with a resolution of 1.0% */
    value = ret >> 8;

    /* count in low byte, round to the nearest integer */
    if (ret & 0x80)
        *capacity = value + 1;
    else
        *capacity = value;

    return OK;
}

static int max17050_max_voltage(struct battery_dev_s *dev, b16_t *max_voltage)
{
    /* The MAX17050_REG_MAX_VOLT returns the maximum voltage that the IC has
       seen thus far. Instead we want to return the max_voltage per design */

    *max_voltage = max17050_cfg.voltage_max_design; // units of mV

    return OK;
}

static int max17050_temperature(struct battery_dev_s *dev, b16_t *temperature)
{
    FAR struct max17050_dev_s *priv = (FAR struct max17050_dev_s *)dev;
    int ret;

    if (!priv->initialized)
        return -EAGAIN;

    ret = max17050_reg_read_retry(priv, MAX17050_REG_TEMP);
    if (ret < 0)
        return -ENODEV;

    *temperature = ret;
    /* The value is signed. */
    if (*temperature & 0x8000) {
        *temperature = (0x7fff & ~*temperature) + 1;
        *temperature *= -1;
    }

    /* The value is converted into deci-centigrade scale */
    /* Units of LSB = 1 / 256 degree Celsius */
    *temperature = *temperature * 10 / 256;

#ifdef CONFIG_BATTERY_MAX17050_THERMISTOR_ACCURATE_TEMP
    *temperature = max17050_thermistor_fuelgauge_to_real(*temperature);
#endif

    return OK;
}

static int max17050_get_current(struct battery_dev_s *dev, bool average, b16_t *current)
{
    FAR struct max17050_dev_s *priv = (FAR struct max17050_dev_s *)dev;
    uint8_t reg = average ? MAX17050_REG_AVERAGE_CURRENT : MAX17050_REG_CURRENT;
    int ret;

    if (!priv->initialized)
        return -EAGAIN;

    ret = max17050_reg_read_retry(priv, reg);
    if (ret < 0)
        return -ENODEV;

    *current = ret;
    if (*current & 0x8000) {
        /* Negative */
        *current = ~*current & 0x7fff;
        *current += 1;
        *current *= -1;
    }
    *current *= 1562500 / max17050_cfg.sns_resistor;

    return OK;
}

static int max17050_current(struct battery_dev_s *dev, b16_t *current)
{
    return max17050_get_current(dev, false, current);
}

static int max17050_full_capacity(struct battery_dev_s *dev, b16_t *capacity)
{
    FAR struct max17050_dev_s *priv = (FAR struct max17050_dev_s *)dev;
    int ret;

    if (!priv->initialized)
        return -EAGAIN;

    ret = max17050_reg_read_retry(priv, MAX17050_REG_FULL_CAP);
    if (ret < 0)
        return -ENODEV;

    *capacity = ret * (5000000 / max17050_cfg.sns_resistor);

    return OK;
}

static int max17050_state(struct battery_dev_s *dev, int *status)
{
    FAR struct max17050_dev_s *priv = (FAR struct max17050_dev_s *)dev;
    int ret;
    b16_t value;

    if (priv->initialized) {
        ret = max17050_capacity(dev, &value);
        if (ret == 0 && value >= 100) {
            *status = BATTERY_FULL;
            return OK;
        }
    }

    *status = g_status;

    return OK;
}

static const struct battery_operations_s max17050_ops =
{
    .state = max17050_state,
    .online = max17050_online,
    .voltage = max17050_voltage,
    .capacity = max17050_capacity,
    .max_voltage = max17050_max_voltage,
    .temperature = max17050_temperature,
    .current = max17050_current,
    .full_capacity = max17050_full_capacity,
};

#if MAX17050_USE_ALRT
static int max17050_enable_alrt(FAR struct max17050_dev_s *priv, bool state)
{
    uint16_t set = state ? MAX17050_CONFIG_AEN : ~MAX17050_CONFIG_AEN;

    return max17050_reg_modify_retry(priv, MAX17050_REG_CONFIG, MAX17050_CONFIG_AEN,
                               set);
}
#endif

#define WRITE_RETRY(priv, reg, val)                             \
    do {                                                        \
        int ret = max17050_reg_write_retry(priv, reg, val);     \
        if (ret) {                                              \
            dbg("err %d write to %x\n", ret, reg);              \
            return ret;                                         \
        }                                                       \
    } while(0)

#define WRITE_VERIFY(priv, reg, val)                            \
    do {                                                        \
        int ret = max17050_reg_write_verify(priv, reg, val);    \
        if (ret) {                                              \
            dbg("err %d write_verify to %x\n", ret, reg);       \
            return ret;                                         \
        }                                                       \
    } while(0)

static int max17050_por_perform_init_config(FAR struct max17050_dev_s *priv)
{
    WRITE_RETRY(priv, MAX17050_REG_CONFIG, max17050_cfg.config);
    WRITE_RETRY(priv, MAX17050_REG_FILTER_CFG, max17050_cfg.filter_cfg);
    WRITE_RETRY(priv, MAX17050_REG_RELAX_CFG, max17050_cfg.relax_cfg);
    WRITE_RETRY(priv, MAX17050_REG_LEARN_CFG, max17050_cfg.learn_cfg);
    WRITE_RETRY(priv, MAX17050_REG_FULL_SOC_THR, max17050_cfg.full_soc_thr);

    return 0;
}

static int max17050_por_unlock_model(FAR struct max17050_dev_s *priv)
{
    WRITE_RETRY(priv, MAX17050_REG_MODEL_ACCESS_1, MAX17050_MODEL_UNLOCK_1);
    WRITE_RETRY(priv, MAX17050_REG_MODEL_ACCESS_2, MAX17050_MODEL_UNLOCK_2);

    return 0;
}

static int max17050_por_lock_model(FAR struct max17050_dev_s *priv)
{
    WRITE_RETRY(priv, MAX17050_REG_MODEL_ACCESS_1, MAX17050_MODEL_LOCK_1);
    WRITE_RETRY(priv, MAX17050_REG_MODEL_ACCESS_2, MAX17050_MODEL_LOCK_2);

    return 0;
}

static int max17050_por_write_cell_data(FAR struct max17050_dev_s *priv)
{
    int i;

    for (i = 0; i < MAX17050_CELL_DATA_SIZE; i++)
        WRITE_RETRY(priv, MAX17050_REG_CELL_DATA_START + i, max17050_cfg.cell_data[i]);

    return 0;
}

static void max17050_por_read_cell_data(FAR struct max17050_dev_s *priv, uint16_t cell_data[])
{
    int i;

    for (i = 0; i < MAX17050_CELL_DATA_SIZE; i++)
        cell_data[i] = max17050_reg_read_retry(priv, MAX17050_REG_CELL_DATA_START + i);
}

static bool max17050_por_cell_data_correct(FAR struct max17050_dev_s *priv)
{
    uint16_t cell_data[MAX17050_CELL_DATA_SIZE];
    int i;

    max17050_por_read_cell_data(priv, cell_data);
    for (i = 0; i <  MAX17050_CELL_DATA_SIZE; i++) {
        if (cell_data[i] != max17050_cfg.cell_data[i]) {
            dbg("mismatch: cell_data[%d]=0x%x\n", i, cell_data[i]);
            return false;
        }
    }

    return true;
}

static bool max17050_por_model_locked(FAR struct max17050_dev_s *priv)
{
    uint16_t cell_data[MAX17050_CELL_DATA_SIZE];
    int i;

    max17050_por_read_cell_data(priv, cell_data);
    for (i = 0; i <  MAX17050_CELL_DATA_SIZE; i++) {
        if (cell_data[i] != 0) {
            dbg("model failed to lock\n");
            return false;
        }
    }
    return true;
}

static int max17050_por_load_model(FAR struct max17050_dev_s *priv)
{
    int tries = 8;

    do {
        if (max17050_por_unlock_model(priv) < 0)
            continue;
        if (max17050_por_write_cell_data(priv) < 0)
            continue;
        if (max17050_por_cell_data_correct(priv))
            break;
    } while(--tries);

    if (!tries)
        return -EIO;

    tries = 8;

    do {
        if (max17050_por_lock_model(priv) < 0)
            continue;
        if (max17050_por_model_locked(priv))
            break;
    } while(--tries);

    return tries ? 0 : -EIO;
}

static int max17050_por_write_custom_params(FAR struct max17050_dev_s *priv)
{
    WRITE_VERIFY(priv, MAX17050_REG_RCOMP0, max17050_cfg.rcomp0);
    WRITE_VERIFY(priv, MAX17050_REG_TEMPCO, max17050_cfg.tempco);

    WRITE_RETRY(priv, MAX17050_REG_ICHG_TERM, max17050_cfg.ichg_term);
    WRITE_RETRY(priv, MAX17050_REG_TGAIN, max17050_cfg.tgain);
    WRITE_RETRY(priv, MAX17050_REG_TOFF, max17050_cfg.toff);

    WRITE_VERIFY(priv, MAX17050_REG_Q_RESIDUAL_00, max17050_cfg.qr_table_00);
    WRITE_VERIFY(priv, MAX17050_REG_Q_RESIDUAL_10, max17050_cfg.qr_table_10);
    WRITE_VERIFY(priv, MAX17050_REG_Q_RESIDUAL_20, max17050_cfg.qr_table_20);
    WRITE_VERIFY(priv, MAX17050_REG_Q_RESIDUAL_30, max17050_cfg.qr_table_30);

#ifdef CONFIG_BATTERY_MAX17050_EMPTY_CFG
    WRITE_RETRY(priv, MAX17050_REG_IAVG_EMPTY, max17050_cfg.iavg_empty);
    WRITE_RETRY(priv, MAX17050_REG_V_EMPTY, max17050_cfg.v_empty);
#endif
    return 0;
}

static int max17050_por_update_full_capacity_params(FAR struct max17050_dev_s *priv)
{
    WRITE_VERIFY(priv, MAX17050_REG_FULL_CAP, max17050_cfg.capacity);
    WRITE_RETRY(priv, MAX17050_REG_DESIGN_CAP, max17050_cfg.vf_fullcap);
    WRITE_VERIFY(priv, MAX17050_REG_FULL_CAP_NOM, max17050_cfg.vf_fullcap);

    return 0;
}

static int max17050_por_write_vfsoc_and_qh0(FAR struct max17050_dev_s *priv)
{
    int vfsoc = max17050_reg_read_retry(priv, MAX17050_REG_VFSOC);
    int qh = max17050_reg_read_retry(priv, MAX17050_REG_QH);

    if (vfsoc < 0 || qh < 0)
         return -EIO;

    WRITE_RETRY(priv, MAX17050_REG_VFSOC0_QH0_ACCESS, MAX17050_VFSOC0_QH0_UNLOCK);
    WRITE_VERIFY(priv, MAX17050_REG_VFSOC0, vfsoc);
    WRITE_RETRY(priv, MAX17050_REG_QH0, qh);
    WRITE_RETRY(priv, MAX17050_REG_VFSOC0_QH0_ACCESS, MAX17050_VFSOC0_QH0_LOCK);

    return 0;
}

static int max17050_por_advance_to_cc_mode(FAR struct max17050_dev_s *priv)
{
    WRITE_VERIFY(priv, MAX17050_REG_CYCLES, MAX17050_CYCLES_INIT_VALUE);
    return 0;
}

static int max17050_por_load_new_capacity_params(FAR struct max17050_dev_s *priv)
{
    int vfsoc = max17050_reg_read_retry(priv, MAX17050_REG_VFSOC);
    int remcap = vfsoc * max17050_cfg.vf_fullcap / MAX17050_REM_CAP_DIV;
    int repcap = remcap * max17050_cfg.capacity / max17050_cfg.vf_fullcap;
    int dq_acc = max17050_cfg.vf_fullcap / 16;

    if (vfsoc < 0)
         return -EIO;

    WRITE_VERIFY(priv, MAX17050_REG_REM_CAP, remcap);
    WRITE_VERIFY(priv, MAX17050_REG_REP_CAP, repcap);
    WRITE_VERIFY(priv, MAX17050_REG_DP_ACC, MAX17050_DP_ACC);
    WRITE_VERIFY(priv, MAX17050_REG_DQ_ACC, dq_acc);
    WRITE_VERIFY(priv, MAX17050_REG_FULL_CAP, max17050_cfg.capacity);
    WRITE_RETRY(priv, MAX17050_REG_DESIGN_CAP, max17050_cfg.vf_fullcap);
    WRITE_VERIFY(priv, MAX17050_REG_FULL_CAP_NOM, max17050_cfg.vf_fullcap);
    WRITE_RETRY(priv, MAX17050_REG_REP_SOC, vfsoc);

    return 0;
}

static int max17050_por_init(FAR struct max17050_dev_s *priv)
{
    /* Workaround for h/w where sense reistor is connected in reverse to the
     * fuel gauge CSN and CSP inputs
     */
#ifdef CONFIG_BATTERY_MAX17050_REVERSED_SENSE_RESISTOR
    WRITE_RETRY(priv, MAX17050_REG_CGAIN, MAX17050_CGAIN_REVERSED_SENSE_R);
#else
    WRITE_RETRY(priv, MAX17050_REG_CGAIN, MAX17050_CGAIN_DEFAULT);
#endif

    // Set COFF and MiscRate register
#ifdef CONFIG_BATTERY_MAX17050_SENSE_TRACE_CORRECTION
    WRITE_VERIFY(priv, MAX17050_REG_COFF, CONFIG_BATTERY_MAX17050_COFF_CFG);
    WRITE_VERIFY(priv, MAX17050_REG_MISC_CFG, CONFIG_BATTERY_MAX17050_MISC_CFG);
#endif

    if (max17050_por_perform_init_config(priv))
        return -EIO;

    if (max17050_por_load_model(priv))
        return -EIO;

    if (max17050_por_write_custom_params(priv))
        return -EIO;

    if (max17050_por_update_full_capacity_params(priv))
        return -EIO;

    // Wait for VFSOC to be calculated with the new configuration
    usleep(350000);

    if (max17050_por_write_vfsoc_and_qh0(priv))
        return -EIO;

    if (max17050_por_advance_to_cc_mode(priv))
        return -EIO;

    if (max17050_por_load_new_capacity_params(priv))
        return -EIO;

    // Clear POR bit to indicate successful initialization
    if (!max17050_reg_modify_retry(priv, MAX17050_REG_STATUS, MAX17050_STATUS_POR, 0)) {
        int status = max17050_reg_read_retry(priv, MAX17050_REG_STATUS);
        if (status >= 0 && !(status & MAX17050_STATUS_POR)) {
            return 0;
        }
    }

    dbg("Failed to clear POR bit\n");
    return -EIO;
}

static void *max17050_por(void *v)
{
    FAR struct max17050_dev_s *priv = v;
    static sem_t sem = SEM_INITIALIZER(1);

    /* Theoretically, another por maybe scheduled before another one finished */
    while (sem_wait(&sem) != OK) {
        if (errno == EINVAL) {
            return NULL;
        }
    }

    if (!max17050_por_init(priv)) {
        // Ready to report battery status
        max17050_set_initialized(priv, true);
        // Set Config Version
        if (max17050_reg_write(priv, MAX17050_REG_CONFIG_VER, max17050_cfg.version)) {
            dbg("Failed to set config version\n");
        }
        dbg("Power-On Reset complete\n");
    } else {
        dbg("Power-On Reset failed\n");
    }

    sem_post(&sem);
    return NULL;
}

#if MAX17050_USE_ALRT
static int max17050_set_voltage_alert(FAR struct max17050_dev_s *priv, int min,
                                      int max)
{
#if defined(CONFIG_BATTERY_LEVEL_DEVICE_MAX17050) && CONFIG_BATTERY_LEVEL_DEVICE_MAX17050_EMPTY_VOLTAGE
    static bool empty_battery_detection;

    /* Set empty battery limit when MIN VALRT is not used by voltage driver */
    if (max == INT_MIN) { /* max value used only by empty battery detection */
        empty_battery_detection = (min != INT_MIN);
        max = priv->max_valrt;
        if (empty_battery_detection && priv->min_valrt != INT_MIN)
            return 0;
    } else if (min == INT_MIN && empty_battery_detection)
        min = CONFIG_BATTERY_LEVEL_DEVICE_MAX17050_EMPTY_VOLTAGE;
#endif

    /* Remember the limits that will be programmed */
    priv->min_valrt = min;
    priv->max_valrt = max;

    uint16_t valrt_min = (min == INT_MIN ? 0x00 : min / 20);
    uint16_t valrt_max = (max == INT_MAX ? 0xff : max / 20);
    uint16_t valrt = valrt_min | valrt_max << 8;

    return max17050_reg_write(priv, MAX17050_REG_VALRT, valrt);
}

#define TALRT_MIN       -128
#define TALRT_MIN_DIS   0x80
#define TALRT_MAX       127
#define TALRT_MAX_DIS   0x7F
static int max17050_set_temperature_alert(FAR struct max17050_dev_s *priv,
                                          int min, int max)
{
    uint16_t talrt_min;
    uint16_t talrt_max;
    uint16_t talrt;

    /* IC limits are in 2's compliment format, 8 bits per limit */
    if (min == INT_MIN)
        talrt_min = TALRT_MIN_DIS;
    else if (min < TALRT_MIN || min > TALRT_MAX)
        return -EINVAL;
    else if (min >= 0)
        talrt_min = min;
    else {
        talrt_min = -min;
        talrt_min = (~talrt_min + 1) & 0x00FF;
    }

    if (max == INT_MAX)
        talrt_max = TALRT_MAX_DIS;
    else if (max < TALRT_MIN || max > TALRT_MAX)
        return -EINVAL;
    else if (max >= 0)
        talrt_max = max;
    else {
        talrt_max = -max;
        talrt_max = (~talrt_max + 1) & 0x00FF;
    }

    /* Remember the limits that will be programmed */
    priv->min_talrt = min;
    priv->max_talrt = max;

    talrt = talrt_min | talrt_max << 8;

    return max17050_reg_write(priv, MAX17050_REG_TALRT, talrt);
}

static int max17050_set_soc_alert(FAR struct max17050_dev_s *priv, int min,
                                  int max)
{
    uint16_t salrt_min = (min == INT_MIN ? 0x00 : min );
    uint16_t salrt_max = (max == INT_MAX ? 0xff : max );
    uint16_t salrt = salrt_min | salrt_max << 8;

    /* Remember the limits that will be programmed */
    priv->min_salrt = min;
    priv->max_salrt = max;

    return max17050_reg_write(priv, MAX17050_REG_SALRT, salrt);
}
#endif

#ifdef CONFIG_BATTERY_TEMP_DEVICE_MAX17050
static void max17050_do_temp_limits_cb(FAR struct max17050_dev_s *priv, bool min)
{
    while (sem_wait(&priv->battery_temp_sem) != OK) {
        if (errno == EINVAL) {
            return;
        }
    }

    if (priv->initialized && priv->temp_limits_cb)
        priv->temp_limits_cb(priv->temp_limits_arg, min);

    sem_post(&priv->battery_temp_sem);
}
#endif

#ifdef CONFIG_BATTERY_LEVEL_DEVICE_MAX17050
static void max17050_do_level_limits_cb(FAR struct max17050_dev_s *priv, bool min)
{
    while (sem_wait(&priv->battery_level_sem) != OK) {
        if (errno == EINVAL) {
            return;
        }
    }

    if (priv->initialized && priv->level_limits_cb)
        priv->level_limits_cb(priv->level_limits_arg, min);

    sem_post(&priv->battery_level_sem);
}
#endif

#ifdef CONFIG_BATTERY_VOLTAGE_DEVICE_MAX17050
static void max17050_do_voltage_limits_cb(FAR struct max17050_dev_s *priv, bool min)
{
    while (sem_wait(&priv->battery_voltage_sem) != OK) {

        if (errno == EINVAL)
            return;
    }

    if (priv->initialized && priv->voltage_limits_cb) {
        priv->voltage_limits_cb(priv->voltage_limits_arg, min);
    }

    sem_post(&priv->battery_voltage_sem);
}
#endif

#if MAX17050_USE_ALRT
static int max17050_alrt_work_delay(struct max17050_dev_s *priv,
                                    bool empty_battery)
{
    if (priv->alrt_gpio >= 0)
        return empty_battery ?
            2 * MAX17050_ALERT_DEASSERT_DELAY : MAX17050_ALERT_DEASSERT_DELAY;
    else
        return MAX17050_ALERT_POLLING_INTERVAL;
}

static void max17050_queue_alrt_work(FAR struct max17050_dev_s *priv, int ms)
{
    if (work_available(&priv->alrt_work))
        work_queue(LPWORK, &g_max17050_dev->alrt_work, max17050_alrt_worker,
                   priv, MSEC2TICK(ms));
}

/*
 * Fuel gauge may erroneously assert both min and max alerts at the same time in
 * some corner cases. If these conditions are detected, disable the alerts to
 * clear the erroneous state and reprogram the thresholds back after at least
 * 351 mS.
 * status: The output value is "-1" if ALRT line is not asserted, otherwise the
 * value is non negative.
 */
static void max17050_min_max_alrt(struct max17050_dev_s *priv, int *status)
{
    /* Thresholds before alerts are disabled */
    static int min_valrt, max_valrt;
    static int min_talrt, max_talrt;
    static int min_salrt, max_salrt;
    /* Flags indicating which alarms need to be reprogrammed */
    bool valrt = priv->min_max_valrt;
    bool talrt = priv->min_max_talrt;
    bool salrt = priv->min_max_salrt;

    *status = -1;

    /* Reprogram alarms if both min and max were previously detected */
    if (valrt) {
        max17050_set_voltage_alert(priv, min_valrt, max_valrt);
        priv->min_max_valrt = false;
    }

    if (talrt) {
        max17050_set_temperature_alert(priv, min_talrt, max_talrt);
        priv->min_max_talrt = false;
    }

    if (salrt) {
        max17050_set_soc_alert(priv, min_salrt, max_salrt);
        priv->min_max_salrt = false;
    }

    /* Process only if ALRT line is asserted */
    if (priv->alrt_gpio >= 0 && gpio_get_value(priv->alrt_gpio))
        return;

    *status = max17050_reg_read_retry(priv, MAX17050_REG_STATUS);

    if (*status < 0) {
        *status = 0; /* To force rescheduling alrt worker */
        return;
    }

    if (valrt)
         /* Skip processing since new thresholds were just programmed */
        *status &= ~(MAX17050_STATUS_VMN | MAX17050_STATUS_VMX);
    else if (*status & MAX17050_STATUS_VMN && *status & MAX17050_STATUS_VMX) {
        dbg("Simultaneous min and max VALRT\n");
        priv->min_max_valrt = true;
         /* Skip processing since both min and max are asserted */
        *status &= ~(MAX17050_STATUS_VMN | MAX17050_STATUS_VMX);
         /* Remember current thresholds and disable the alerts to clear them */
        min_valrt = priv->min_valrt;
        max_valrt = priv->max_valrt;
        max17050_set_voltage_alert(priv, INT_MIN, INT_MAX);
    }

    if (talrt)
         /* Skip processing since new thresholds were just programmed */
        *status &= ~(MAX17050_STATUS_TMN | MAX17050_STATUS_TMX);
    else if (*status & MAX17050_STATUS_TMN && *status & MAX17050_STATUS_TMX) {
        dbg("Simultaneous min and max TALRT\n");
        priv->min_max_talrt = true;
         /* Skip processing since both min and max are asserted */
        *status &= ~(MAX17050_STATUS_TMN | MAX17050_STATUS_TMX);
         /* Remember current thresholds and disable the alerts to clear them */
        min_talrt = priv->min_talrt;
        max_talrt = priv->max_talrt;
        max17050_set_temperature_alert(priv, INT_MIN, INT_MAX);
    }

    if (salrt)
         /* Clear bits to skip processing */
        *status &= ~(MAX17050_STATUS_SMN | MAX17050_STATUS_SMX);
    else if (*status & MAX17050_STATUS_SMN && *status & MAX17050_STATUS_SMX) {
        dbg("Simultaneous min and max SALRT\n");
        priv->min_max_salrt = true;
         /* Skip processing since both min and max are asserted */
        *status &= ~(MAX17050_STATUS_SMN | MAX17050_STATUS_SMX);
         /* Remember current thresholds and disable the alerts to clear them */
        min_salrt = priv->min_salrt;
        max_salrt = priv->max_salrt;
        max17050_set_soc_alert(priv, INT_MIN, INT_MAX);
    }
}

static void max17050_alrt_worker(FAR void *arg)
{
    FAR struct max17050_dev_s *priv = arg;
    int status;
    bool empty_battery = false;

    /* Process only if initialized */
    if (!priv->initialized)
        return;

    max17050_min_max_alrt(priv, &status);
    if (status < 0)
        return;

    if (status > 0) {
#ifdef CONFIG_BATTERY_LEVEL_DEVICE_MAX17050
 #if CONFIG_BATTERY_LEVEL_DEVICE_MAX17050_EMPTY_VOLTAGE
        if (status & MAX17050_STATUS_VMN && priv->min_valrt ==
                        CONFIG_BATTERY_LEVEL_DEVICE_MAX17050_EMPTY_VOLTAGE) {
            max17050_battery_level_set_empty(priv);
            empty_battery = true;
            max17050_do_level_limits_cb(priv, true);
            /* Skip checking voltage alerts in battery voltage driver */
            status &= ~(MAX17050_STATUS_VMN | MAX17050_STATUS_VMX);
        } else if (status & MAX17050_STATUS_SMN)
            max17050_do_level_limits_cb(priv, true);
        else if (status & MAX17050_STATUS_SMX)
            max17050_do_level_limits_cb(priv, false);
 #else
        if (status & MAX17050_STATUS_SMN)
            max17050_do_level_limits_cb(priv, true);
        else if (status & MAX17050_STATUS_SMX)
            max17050_do_level_limits_cb(priv, false);
 #endif
#endif
#ifdef CONFIG_BATTERY_VOLTAGE_DEVICE_MAX17050
        if (status & MAX17050_STATUS_VMN)
            max17050_do_voltage_limits_cb(priv, true);
        else if (status & MAX17050_STATUS_VMX)
            max17050_do_voltage_limits_cb(priv, false);
#endif
#ifdef CONFIG_BATTERY_TEMP_DEVICE_MAX17050
        if (status & MAX17050_STATUS_TMN)
            max17050_do_temp_limits_cb(priv, true);
        else if (status & MAX17050_STATUS_TMX)
            max17050_do_temp_limits_cb(priv, false);
#endif
   }

    /* It takes up to 351 mS for the IC to de-assert the ALRT line after the
     * condition goes away. Run this worker again since ALRT line will remain
     * asserted if another condition becomes active within this window.
     * If SOC is forced to 0% after empty battery is detected, it may take
     * another 351 mS for the IC to process it. So, double the delay to avoid
     * false SOC alerts.
     */
    max17050_queue_alrt_work(priv, max17050_alrt_work_delay(priv, empty_battery));
}

static int max17050_alrt_isr(int irq, void *context)
{
    if (work_available(&g_max17050_dev->alrt_work)) {
        work_queue(LPWORK, &g_max17050_dev->alrt_work,
                    max17050_alrt_worker, g_max17050_dev, 0);
        pm_activity(PM_ACTIVITY);
    };

    return OK;
}
#endif

static void max17050_set_initialized(FAR struct max17050_dev_s *priv, bool state)
{
    if (state == priv->initialized)
        return;
#ifdef CONFIG_BATTERY_TEMP_DEVICE_MAX17050
    while (sem_wait(&priv->battery_temp_sem) != OK) {
        if (errno == EINVAL) {
            return;
        }
    }
#endif
#ifdef CONFIG_BATTERY_LEVEL_DEVICE_MAX17050
    while (sem_wait(&priv->battery_level_sem) != OK) {
        if (errno == EINVAL) {
        #ifdef CONFIG_BATTERY_TEMP_DEVICE_MAX17050
            sem_post(&priv->battery_temp_sem);
        #endif
            return;
        }
    }
#endif
#ifdef CONFIG_BATTERY_VOLTAGE_DEVICE_MAX17050
    while (sem_wait(&priv->battery_voltage_sem) != OK) {
       if (errno == EINVAL) {
       #ifdef CONFIG_BATTERY_TEMP_DEVICE_MAX17050
          sem_post(&priv->battery_temp_sem);
       #endif
       #ifdef CONFIG_BATTERY_LEVEL_DEVICE_MAX17050
          sem_post(&priv->battery_level_sem);
       #endif
           return;
       }
    }
#endif

    priv->initialized = state;
#if MAX17050_USE_ALRT
    if (!state) {
        priv->min_max_valrt = false;
        priv->min_max_talrt = false;
        priv->min_max_salrt = false;
    }
#endif
#ifdef CONFIG_BATTERY_TEMP_DEVICE_MAX17050
    if (priv->temp_available_cb)
        priv->temp_available_cb(priv->temp_available_arg, state);

    if (!state)
        /* Disable */
        max17050_set_temperature_alert(priv, INT_MIN, INT_MAX);

    sem_post(&priv->battery_temp_sem);
#endif
#ifdef CONFIG_BATTERY_LEVEL_DEVICE_MAX17050
    if (priv->level_available_cb)
        priv->level_available_cb(priv->level_available_arg, state);

    if (!state) {
        /* Disable */
        max17050_set_soc_alert(priv, INT_MIN, INT_MAX);
 #if CONFIG_BATTERY_LEVEL_DEVICE_MAX17050_EMPTY_VOLTAGE
        max17050_battery_level_empty_detection(priv, false);
        max17050_battery_level_set_empty(priv);
 #endif
    }

    sem_post(&priv->battery_level_sem);
#endif
#ifdef CONFIG_BATTERY_VOLTAGE_DEVICE_MAX17050
    if (priv->voltage_available_cb)
        priv->voltage_available_cb(priv->voltage_available_arg, state);

    if (!state) {
        /* Disable */
        max17050_set_voltage_alert(priv, INT_MIN, INT_MAX);
    }

    sem_post(&priv->battery_voltage_sem);
#endif /* #ifdef CONFIG_BATTERY_VOLTAGE_DEVICE_MAX17050 */

#if MAX17050_USE_ALRT
    if (state) {
        /* Make sure alerts are enabled and queue alert worker in case alert set
           in available callback is already active */
        max17050_enable_alrt(priv, true);
        max17050_queue_alrt_work(priv, max17050_alrt_work_delay(priv, false));
    } else
        max17050_enable_alrt(priv, false);
#endif

}

static bool max17050_new_config(FAR struct max17050_dev_s *priv)
{
    int ret;

    ret = max17050_reg_read_retry(priv, MAX17050_REG_CONFIG_VER);
    if (ret < 0)
        return false;

    return (max17050_cfg.version != ret);
}

static void max17050_batt_is_good_worker(FAR void *arg)
{
    FAR struct max17050_dev_s *priv = arg;
    pthread_t por_thread;
    int ret;

    ret = max17050_reg_read_retry(priv, MAX17050_REG_DEV_NAME);
    if (ret < 0) {
        max17050_set_initialized(priv, false);
        return;
    }

    if (ret != MAX17050_IC_VERSION) {
        max17050_set_initialized(priv, false);
        return;
    }

#ifdef CONFIG_BATTERY_MAX17050_SENSE_TRACE_CORRECTION
    ret = max17050_reg_read_retry(priv, MAX17050_REG_COFF);
    if (ret < 0) {
        max17050_set_initialized(priv, false);
        return;
    }
    if (ret != CONFIG_BATTERY_MAX17050_COFF_CFG) {
        max17050_reg_modify_retry(priv, MAX17050_REG_STATUS,
              MAX17050_STATUS_POR, MAX17050_STATUS_POR);
    } else {
       ret = max17050_reg_read_retry(priv, MAX17050_REG_MISC_CFG);
       if (ret < 0) {
           max17050_set_initialized(priv, false);
           return;
       }
       if (ret != CONFIG_BATTERY_MAX17050_MISC_CFG) {
           max17050_reg_modify_retry(priv, MAX17050_REG_STATUS,
              MAX17050_STATUS_POR, MAX17050_STATUS_POR);
       }
    }
#endif

    ret = max17050_reg_read_retry(priv, MAX17050_REG_STATUS);
    if (ret < 0) {
        max17050_set_initialized(priv, false);
        return;
    }

    // Configure device after Power-On Reset or with the new configuration
    if (ret & MAX17050_STATUS_POR || max17050_new_config(priv)) {
        max17050_set_initialized(priv, false);
        pthread_create(&por_thread, NULL, max17050_por, priv);
        pthread_detach(por_thread);
    } else
        max17050_set_initialized(priv, true);
}

static void max17050_batt_is_bad_worker(FAR void *arg)
{
    max17050_set_initialized(arg, false);
}

static void max17050_battery_good(void *arg, bool good)
{
    FAR struct max17050_dev_s *priv = arg;
    worker_t worker = good ? max17050_batt_is_good_worker :
                             max17050_batt_is_bad_worker;
    int delay = good ? MSEC2TICK(MAX17050_DELAY_AFTER_POWERUP) : 0;

    if (work_cancel(LPWORK, &priv->batt_good_work) == 0)
        work_queue(LPWORK, &priv->batt_good_work, worker, priv, delay);

    return;
}

FAR struct battery_dev_s *max17050_initialize(FAR struct i2c_dev_s *i2c,
                                              uint32_t frequency,
                                              int alrt_gpio)
{
    FAR struct max17050_dev_s *priv;
    int ret;
    /*
     * Sanity check configuration and bomb out if supplied invalid values.
     * SNS resistor value must be non-zero to avoid dividing by zero when
     * calcuating current.
     */
    if (!max17050_cfg.sns_resistor)
        return NULL;

    priv = (FAR struct max17050_dev_s *)kmm_zalloc(sizeof(*priv));
    if (priv) {
        sem_init(&priv->batsem, 0, 1);
        priv->ops = &max17050_ops;
        priv->i2c = i2c;

        ret = I2C_SETADDRESS(i2c, MAX17050_I2C_ADDR, 7);
        if (ret)
            goto err;

        I2C_SETFREQUENCY(i2c, frequency);

#if MAX17050_USE_ALRT
        priv->alrt_gpio = alrt_gpio;
        /* Disable alerts */
        max17050_set_soc_alert(priv, INT_MIN, INT_MAX);
        max17050_set_voltage_alert(priv, INT_MIN, INT_MAX);
        max17050_set_temperature_alert(priv, INT_MIN, INT_MAX);
        max17050_enable_alrt(priv, false);
#endif

#ifdef CONFIG_BATTERY_TEMP_DEVICE_MAX17050
        sem_init(&priv->battery_temp_sem, 0, 1);
#endif

#ifdef CONFIG_BATTERY_LEVEL_DEVICE_MAX17050
        sem_init(&priv->battery_level_sem, 0, 1);
 #if CONFIG_BATTERY_LEVEL_DEVICE_MAX17050_EMPTY_VOLTAGE
        priv->min_valrt = INT_MIN;
        priv->max_valrt = INT_MAX;
 #endif
#endif

#ifdef CONFIG_BATTERY_VOLTAGE_DEVICE_MAX17050
        sem_init(&priv->battery_voltage_sem, 0, 1);
#endif

        g_max17050_dev = priv;

       /* If the host can run when battery voltage is below the min level
        * needed for a fuel gauge to operate, an external device such as
        * a battery voltage comparator needs to be used to detect when fuel
        * gauge is functional.
        */
        priv->batt_good_dev = device_open(DEVICE_TYPE_BATTERY_GOOD_HW, 0);
        if (priv->batt_good_dev) {
            ret = device_batt_good_register_callback(priv->batt_good_dev,
                                                     max17050_battery_good,
                                                     priv);
            if (ret) {
                g_max17050_dev = NULL;
                device_close(priv->batt_good_dev);
                goto err;
            }
        } else
            /* If batt_good device is not used, check for POR right away */
            work_queue(LPWORK, &priv->batt_good_work,
                       max17050_batt_is_good_worker, priv,
                       MSEC2TICK(MAX17050_DELAY_AFTER_POWERUP));

#if MAX17050_USE_ALRT
        /* Setup ALRT IRQ */
        if (alrt_gpio >= 0) {
            gpio_irqattach(alrt_gpio, max17050_alrt_isr);
            set_gpio_triggering(alrt_gpio, IRQ_TYPE_EDGE_FALLING);
        }
 #ifdef CONFIG_PM
        if (pm_register(&pm_callback) != OK)
        {
            dbg("Failed register to power management!\n");
        }
 #endif
#endif
    }

    return (FAR struct battery_dev_s *)priv;

err:
    kmm_free(priv);
    return NULL;
}

#ifdef CONFIG_BATTERY_TEMP_DEVICE_MAX17050
int max17050_battery_temp_open(struct device *dev)
{
    /* Make sure the core driver is initialized */
    if (!g_max17050_dev)
        return -ENODEV;

    device_set_private(dev, g_max17050_dev);

    return 0;
}

void max17050_battery_temp_close(struct device *dev)
{
    FAR struct max17050_dev_s *priv = device_get_private(dev);

    while (sem_wait(&priv->battery_temp_sem) != OK) {
        if (errno == EINVAL) {
            return;
        }
    }

    /* Disable temp alerts */
    (void) max17050_set_temperature_alert(priv, INT_MIN, INT_MAX);

    /* Unregister callbacks */
    priv->temp_limits_cb     = NULL;
    priv->temp_limits_arg    = NULL;
    priv->temp_available_cb  = NULL;
    priv->temp_available_arg = NULL;

    device_set_private(dev, NULL);

    sem_post(&priv->battery_temp_sem);
}

int max17050_battery_temp_register_limits_cb(struct device *dev, batt_temp_limits cb,
                                             void *arg)
{
    FAR struct max17050_dev_s *priv = device_get_private(dev);

    if (priv->temp_limits_cb)
        return -EEXIST;

    while (sem_wait(&priv->battery_temp_sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    priv->temp_limits_cb  = cb;
    priv->temp_limits_arg = arg;

    sem_post(&priv->battery_temp_sem);

    return 0;
}

int max17050_battery_temp_register_available_cb(struct device *dev,
                                                batt_temp_available cb,
                                                void *arg)
{
    FAR struct max17050_dev_s *priv = device_get_private(dev);

    if (priv->temp_available_cb)
        return -EEXIST;

    while (sem_wait(&priv->battery_temp_sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    priv->temp_available_cb  = cb;
    priv->temp_available_arg = arg;
    cb(arg, priv->initialized);
    if (priv->initialized)
        /* In case temp alert set in available callback is already active */
        max17050_queue_alrt_work(priv, max17050_alrt_work_delay(priv, false));

    sem_post(&priv->battery_temp_sem);

    return 0;
}

int max17050_battery_temp_get_temperature(struct device *dev, int *temperature)
{
    struct battery_dev_s *battery_dev = device_get_private(dev);
    b16_t t;
    int ret;

    ret = max17050_temperature(battery_dev, &t);
    if (ret)
        return ret;

    /* Convert deci-centigrades to centigrades */
    *temperature = t / 10;

    return 0;
}

int max17050_battery_temp_set_limits(struct device *dev, int min, int max)
{
    FAR struct max17050_dev_s *priv = device_get_private(dev);

#ifdef CONFIG_BATTERY_MAX17050_THERMISTOR_ACCURATE_TEMP
    /* min and max will be 1x Celsius. */
    if (min != INT_MIN)
        min = max17050_thermistor_real_to_fuelgauge(min*10)/10;
    if (max != INT_MAX)
        max = max17050_thermistor_real_to_fuelgauge(max*10)/10;
#endif

    return max17050_set_temperature_alert(priv, min, max);
}

static struct device_batt_temp_type_ops max17050_battery_temp_type_ops = {
    .register_limits_cb = max17050_battery_temp_register_limits_cb,
    .register_available_cb = max17050_battery_temp_register_available_cb,
    .get_temperature = max17050_battery_temp_get_temperature,
    .set_limits = max17050_battery_temp_set_limits,
};

static struct device_driver_ops max17050_battery_temp_driver_ops = {
    .open = max17050_battery_temp_open,
    .close = max17050_battery_temp_close,
    .type_ops = &max17050_battery_temp_type_ops,
};

struct device_driver max17050_battery_temp_driver = {
    .type = DEVICE_TYPE_BATTERY_TEMP_HW,
    .name = "max17050_battery_temp",
    .desc = "Battery temperature monitoring with MAX17050",
    .ops = &max17050_battery_temp_driver_ops,
};
#endif

#ifdef CONFIG_BATTERY_LEVEL_DEVICE_MAX17050
static int max17050_battery_level_open(struct device *dev)
{
    /* Make sure the core driver is initialized */
    if (!g_max17050_dev)
        return -ENODEV;

    device_set_private(dev, g_max17050_dev);

    return 0;
}

static void max17050_battery_level_close(struct device *dev)
{
    FAR struct max17050_dev_s *priv = device_get_private(dev);

    while (sem_wait(&priv->battery_level_sem) != OK) {
        if (errno == EINVAL) {
            return;
        }
    }

    /* Disable soc alerts */
    (void) max17050_set_soc_alert(priv, INT_MIN, INT_MAX);

#if CONFIG_BATTERY_LEVEL_DEVICE_MAX17050_EMPTY_VOLTAGE
    /* Disable empty battery detection */
    (void) max17050_battery_level_empty_detection(priv, false);
#endif

    /* Unregister callbacks */
    priv->level_limits_cb     = NULL;
    priv->level_limits_arg    = NULL;
    priv->level_available_cb  = NULL;
    priv->level_available_arg = NULL;

    device_set_private(dev, NULL);

    sem_post(&priv->battery_level_sem);
}

static int max17050_battery_level_register_limits_cb(struct device *dev,
                                                     batt_level_limits cb,
                                                     void *arg)
{
    FAR struct max17050_dev_s *priv = device_get_private(dev);

    if (priv->level_limits_cb)
        return -EEXIST;

    while (sem_wait(&priv->battery_level_sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    priv->level_limits_cb  = cb;
    priv->level_limits_arg = arg;

    sem_post(&priv->battery_level_sem);

    return 0;
}

static int max17050_battery_level_register_available_cb(struct device *dev,
                                                        batt_level_available cb,
                                                        void *arg)
{
    FAR struct max17050_dev_s *priv = device_get_private(dev);

    if (priv->level_available_cb)
        return -EEXIST;

    while (sem_wait(&priv->battery_level_sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    priv->level_available_cb  = cb;
    priv->level_available_arg = arg;
    cb(arg, priv->initialized);
    if (priv->initialized)
        /* In case level alert set in available callback is already active */
        max17050_queue_alrt_work(priv, max17050_alrt_work_delay(priv, false));

    sem_post(&priv->battery_level_sem);

    return 0;
}

static int max17050_battery_level_get_capacity(struct device *dev, int *capacity)
{
    struct battery_dev_s *battery_dev = device_get_private(dev);
    b16_t c;
    int ret;

    ret = max17050_capacity(battery_dev, &c);
    if (ret)
        return ret;

    *capacity = c;

    return 0;
}

#define MAX17050_EOC_COUNT  (15)
#define MAX17050_EOC_DELAY  MSEC2TICK(1000)
static int max17050_average_current(struct battery_dev_s *dev, b16_t *current)
{
    return max17050_get_current(dev, true, current);
}

static int max17050_voltage_capacity(struct battery_dev_s *dev, b16_t *capacity)
{
    FAR struct max17050_dev_s *priv = (FAR struct max17050_dev_s *)dev;
    int ret;

    if (!priv->initialized)
        return -EAGAIN;

    ret = max17050_reg_read(priv, MAX17050_REG_VFSOC);
    if (ret < 0)
        return -ENODEV;

    *capacity = ret >> 8;

    return OK;
}

static int max17050_battery_level_charge_complete(struct max17050_dev_s *priv)
{
    int repcap = max17050_reg_read(priv, MAX17050_REG_REP_CAP);
#ifdef CONFIG_BATTERY_MAX17050_SENSE_TRACE_CORRECTION
    int full_charge_design = max17050_cfg.capacity;
    int repcap_max = full_charge_design + full_charge_design * 3 / 100;
    int repcap_min = full_charge_design / 2;
#endif

    /* Set FullCap to RepCap to force 100% */
    if (repcap < 0)
        return repcap;

#ifdef CONFIG_BATTERY_MAX17050_SENSE_TRACE_CORRECTION
    if (repcap > repcap_max) {
        WRITE_VERIFY(priv, MAX17050_REG_REP_CAP, repcap_max);
        WRITE_VERIFY(priv, MAX17050_REG_FULL_CAP, repcap_max);
    } else if(repcap < repcap_min) {
        WRITE_VERIFY(priv, MAX17050_REG_REP_CAP, repcap_min);
        WRITE_VERIFY(priv, MAX17050_REG_FULL_CAP, repcap_min);
    } else {
        WRITE_VERIFY(priv, MAX17050_REG_FULL_CAP, repcap);
    }
#else
    WRITE_VERIFY(priv, MAX17050_REG_FULL_CAP, repcap);
#endif

    return 0;
}

static bool max17050_battery_level_eoc_conditions(struct battery_dev_s *dev)
{
    int current, a_current, v_capacity;

    if (max17050_current(dev, &current) < 0)
        return false;

    current /= 1000; /* uA to mA */

    if (max17050_average_current(dev, &a_current) < 0)
        return false;

    a_current /= 1000; /* uA to mA */

    if (max17050_voltage_capacity(dev, &v_capacity) < 0)
        return false;

    return (current    >= CONFIG_BATTERY_LEVEL_DEVICE_MAX17050_MIN_ITERM &&
            current    <= CONFIG_BATTERY_LEVEL_DEVICE_MAX17050_MAX_ITERM &&
            a_current  >= CONFIG_BATTERY_LEVEL_DEVICE_MAX17050_MIN_ITERM &&
            a_current  <= CONFIG_BATTERY_LEVEL_DEVICE_MAX17050_MAX_ITERM &&
            v_capacity >= CONFIG_BATTERY_LEVEL_DEVICE_MAX17050_MIN_VFSOC);
}

static void max17050_battery_level_eoc_worker(FAR void *arg)
{
    FAR struct max17050_dev_s *priv = arg;

    /* Conditions must be met several times in a row */
    if (max17050_battery_level_eoc_conditions(arg)) {
        if (++priv->eoc_counter == MAX17050_EOC_COUNT) {
            max17050_battery_level_charge_complete(arg);
        }
    } else {
        priv->eoc_counter = 0;
    }

    /* Make callback after SOC became 100% */
    if (priv->eoc_counter > MAX17050_EOC_COUNT && priv->max_soc_alrt)
         max17050_do_level_limits_cb(priv, false);

    work_queue(LPWORK, &priv->eoc_work, max17050_battery_level_eoc_worker, priv,
               MAX17050_EOC_DELAY);
}

static void max17050_battery_level_state_changed_worker(FAR void *arg)
{
    FAR struct max17050_dev_s *priv = arg;

    /* Detect end-of-charging conditions while charging */
    if (g_status == BATTERY_CHARGING) {
        if (work_available(&priv->eoc_work)) {
            priv->eoc_counter = 0;
            work_queue(LPWORK, &priv->eoc_work,
                       max17050_battery_level_eoc_worker, priv, 0);
        }
    } else {
        work_cancel(LPWORK, &priv->eoc_work);
    }
}

#if CONFIG_BATTERY_LEVEL_DEVICE_MAX17050_EMPTY_VOLTAGE
static void max17050_battery_level_empty_detection(struct max17050_dev_s *priv, bool enable)
{

    max17050_set_voltage_alert(priv,
        enable ? CONFIG_BATTERY_LEVEL_DEVICE_MAX17050_EMPTY_VOLTAGE : INT_MIN,
        INT_MIN /* this value indicates that request came from empty battery detector */);
}
#endif

#define MIN_SOC_ALRT    1    /* No IRQ for 0% since SOC cannot be less than 0% */
static int max17050_battery_level_set_limits(struct device *dev, int min, int max)
{
    FAR struct max17050_dev_s *priv = device_get_private(dev);

    if (min != INT_MIN && min < DEVICE_BATTERY_LEVEL_EMPTY)
        return -EINVAL;

    if (max != INT_MAX && max > DEVICE_BATTERY_LEVEL_FULL)
        return -EINVAL;

    /* Use end of charging detection for DEVICE_BATTERY_LEVEL_FULL */
    priv->max_soc_alrt = (max == DEVICE_BATTERY_LEVEL_FULL);
    if (priv->max_soc_alrt)
        max = INT_MAX;

#if CONFIG_BATTERY_LEVEL_DEVICE_MAX17050_EMPTY_VOLTAGE
    /* Enable empty battery detection if MIN ALRT is enabled */
    max17050_battery_level_empty_detection(priv, min != INT_MIN);

    /* Use empty battery detection for DEVICE_BATTERY_LEVEL_EMPTY */
    if (min == DEVICE_BATTERY_LEVEL_EMPTY)
        min = INT_MIN;
#else
    /* Use the MIN_SOC_ALRT to detect empty battery */
    if (min == DEVICE_BATTERY_LEVEL_EMPTY)
        min = MIN_SOC_ALRT;
#endif

    return max17050_set_soc_alert(priv, min, max);
}

#if CONFIG_BATTERY_LEVEL_DEVICE_MAX17050_EMPTY_VOLTAGE
static void max17050_battery_level_set_empty(struct max17050_dev_s *priv)
{
    /* Set RepCap to 0 mAh to force RepSOC to become 0% */
    max17050_reg_write_verify(priv, MAX17050_REG_REP_CAP, 0);
}
#endif

static struct device_batt_level_type_ops max17050_battery_level_type_ops = {
    .register_limits_cb = max17050_battery_level_register_limits_cb,
    .register_available_cb = max17050_battery_level_register_available_cb,
    .get_capacity = max17050_battery_level_get_capacity,
    .set_limits = max17050_battery_level_set_limits,
};

static struct device_driver_ops max17050_battery_level_driver_ops = {
    .open = max17050_battery_level_open,
    .close = max17050_battery_level_close,
    .type_ops = &max17050_battery_level_type_ops,
};

struct device_driver max17050_battery_level_driver = {
    .type = DEVICE_TYPE_BATTERY_LEVEL_HW,
    .name = "max17050_battery_level",
    .desc = "Battery level monitoring with MAX17050",
    .ops = &max17050_battery_level_driver_ops,
};
#endif

#ifdef CONFIG_BATTERY_VOLTAGE_DEVICE_MAX17050
static int max17050_battery_voltage_register_limits_cb(struct device *dev, batt_voltage_limits cb, void *arg)
{
    FAR struct max17050_dev_s *priv = device_get_private(dev);

    if (priv->voltage_limits_cb)
        return -EEXIST;

    while (sem_wait(&priv->battery_voltage_sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    priv->voltage_limits_cb  = cb;
    priv->voltage_limits_arg = arg;

    sem_post(&priv->battery_voltage_sem);

    return 0;
}

static int max17050_battery_voltage_register_available_cb(struct device *dev,
                                                batt_voltage_available cb,
                                                void *arg) {

    FAR struct max17050_dev_s *priv = device_get_private(dev);

    if (priv->voltage_available_cb){
        return -EEXIST;
    }
    while (sem_wait(&priv->battery_voltage_sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }
    priv->voltage_available_cb  = cb;
    priv->voltage_available_arg = arg;
    cb(arg, priv->initialized);
    if (priv->initialized)
        /* In case voltage alert set in available callback is already active */
        max17050_queue_alrt_work(priv, max17050_alrt_work_delay(priv, false));
    sem_post(&priv->battery_voltage_sem);
    return 0;
}

static int max17050_battery_voltage_get_voltage(struct device *dev, int *voltage)
{
    struct battery_dev_s *battery_dev = device_get_private(dev);
    b16_t v;
    int ret;

    ret = max17050_voltage(battery_dev, &v);
    if (ret)
        return ret;

    /* Convert uV to mV */
    *voltage = v / 1000;

    return 0;
}

static int max17050_battery_voltage_set_limits(struct device *dev, int min, int max){

    FAR struct max17050_dev_s *priv = device_get_private(dev);

    return max17050_set_voltage_alert(priv, min, max);
}


static int max17050_battery_voltage_open(struct device *dev)
{
    /* Make sure the core driver is initialized */
    if (!g_max17050_dev)
        return -ENODEV;

    device_set_private(dev, g_max17050_dev);

    return 0;
}

static void max17050_battery_voltage_close(struct device *dev)
{
    FAR struct max17050_dev_s *priv = device_get_private(dev);

    while (sem_wait(&priv->battery_voltage_sem) != OK) {
        if (errno == EINVAL) {
            return;
        }
    }

    /* Disable voltage alerts */
    (void) max17050_set_voltage_alert(priv, INT_MIN, INT_MAX);

    /* Unregister callbacks */
    priv->voltage_limits_cb     = NULL;
    priv->voltage_limits_arg    = NULL;
    priv->voltage_available_cb  = NULL;
    priv->voltage_available_arg = NULL;

    device_set_private(dev, NULL);

    sem_post(&priv->battery_voltage_sem);
}

static struct device_batt_voltage_type_ops max17050_battery_voltage_type_ops = {
    .voltage_register_limits_cb = max17050_battery_voltage_register_limits_cb,
    .voltage_register_available_cb = max17050_battery_voltage_register_available_cb,
    .get_voltage = max17050_battery_voltage_get_voltage,
    .set_voltage_limits =  max17050_battery_voltage_set_limits,
};


static struct device_driver_ops max17050_battery_voltage_driver_ops = {
    .open = max17050_battery_voltage_open,
    .close = max17050_battery_voltage_close,
    .type_ops = &max17050_battery_voltage_type_ops,
};


struct device_driver max17050_battery_voltage_driver = {
    .type = DEVICE_TYPE_BATTERY_VOLTAGE_HW,
    .name = "max17050_battery_voltage",
    .desc = "Battery voltage monitoring with MAX17050",
    .ops = &max17050_battery_voltage_driver_ops,
};
#endif /* #ifdef CONFIG_BATTERY_VOLTAGE_DEVICE_MAX17050 */
