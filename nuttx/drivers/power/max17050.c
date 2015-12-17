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

#include <nuttx/config.h>
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
#include <stdbool.h>
#include <stdint.h>

#include "max17050_config.h"

#define MAX17050_I2C_ADDR               0x36

#define MAX17050_REG_STATUS             0x00
#define MAX17050_REG_VALRT              0x01
#define MAX17050_REG_REP_CAP            0x05
#define MAX17050_REG_REP_SOC            0x06
#define MAX17050_REG_TEMP               0x08
#define MAX17050_REG_VCELL              0x09
#define MAX17050_REG_CURRENT            0x0A
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
#define MAX17050_REG_TGAIN              0x2C
#define MAX17050_REG_TOFF               0x2D
#define MAX17050_REG_CGAIN              0x2E
#define MAX17050_REG_Q_RESIDUAL_20      0x32
#define MAX17050_REG_RCOMP0             0x38
#define MAX17050_REG_TEMPCO             0x39
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
#define MAX17050_STATUS_VMX             BIT(12) /* Max Valrt */

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

/* Priority to report to PM framework when reporting activity */
#define PM_ACTIVITY                     10

/* Min battery voltage for fuel gauge to be functional */
#define MAX17050_MIN_BATTERY_VOLTAGE    2500    /* in mV */

/* Delay afer fuel gauge power up */
#define MAX17050_DELAY_AFTER_POWERUP    500000  /* in uS */

extern const struct max17050_config max17050_cfg;

struct max17050_dev_s
{
    /* The common part of the battery driver visible to the upper-half driver */

    FAR const struct battery_operations_s *ops;
    sem_t batsem;

    /* Data fields specific to the lower half MAX17050 driver follow */

    FAR struct i2c_dev_s *i2c;
    bool initialized;
    struct work_s i2c_available_work;
    struct work_s alrt_work;
    bool use_voltage_alrt;
};


/*
 * Global pointer is needed because Nuttx IRQs do not provide a pointer to
 * driver private data
 */
static struct max17050_dev_s* g_max17050_dev;

static int max17050_reg_read(FAR struct max17050_dev_s *priv, uint8_t reg)
{
    uint16_t reg_val;
    int ret;

    ret = I2C_WRITEREAD(priv->i2c, (uint8_t *)&reg, sizeof(reg),
                        (uint8_t *)&reg_val, sizeof(reg_val));
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

static int max17050_online(struct battery_dev_s *dev, bool *status)
{
    FAR struct max17050_dev_s *priv = (FAR struct max17050_dev_s *)dev;
    int ret;

    if (!priv->initialized)
        return -EAGAIN;

    ret = max17050_reg_read(priv, MAX17050_REG_STATUS);
    if (ret < 0)
        return -ENODEV;

    // BST is 0 when battery is present and 1 when battery is removed
    *status = (ret & MAX17050_STATUS_BST) != MAX17050_STATUS_BST;

    dbg("%d\n", *status);
    return OK;
}

static int max17050_voltage(struct battery_dev_s *dev, b16_t *voltage)
{
    FAR struct max17050_dev_s *priv = (FAR struct max17050_dev_s *)dev;
    int ret;

    if (!priv->initialized)
        return -EAGAIN;

    ret = max17050_reg_read(priv, MAX17050_REG_VCELL);
    if (ret < 0)
        return -ENODEV;

    *voltage = ret * 625 / 8;

    dbg("%d uV\n", *voltage);
    return OK;
}

static int max17050_capacity(struct battery_dev_s *dev, b16_t *capacity)
{
    FAR struct max17050_dev_s *priv = (FAR struct max17050_dev_s *)dev;
    int ret;

    if (!priv->initialized)
        return -EAGAIN;

    ret = max17050_reg_read(priv, MAX17050_REG_REP_SOC);
    if (ret < 0)
        return -ENODEV;

    *capacity = ret >> 8;

    dbg("%d percent\n", *capacity);
    return OK;
}

static int max17050_max_voltage(struct battery_dev_s *dev, b16_t *max_voltage)
{
    /* The MAX17050_REG_MAX_VOLT returns the maximum voltage that the IC has
       seen thus far. Instead we want to return the max_voltage per design */

    *max_voltage = max17050_cfg.voltage_max_design; // units of mV

    dbg("%d mV\n", *max_voltage);
    return OK;
}

static int max17050_temperature(struct battery_dev_s *dev, b16_t *temperature)
{
    FAR struct max17050_dev_s *priv = (FAR struct max17050_dev_s *)dev;
    int ret;

    if (!priv->initialized)
        return -EAGAIN;

    ret = max17050_reg_read(priv, MAX17050_REG_TEMP);
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

    dbg("%d\n", *temperature);
    return OK;
}

static int max17050_current(struct battery_dev_s *dev, b16_t *current)
{
    FAR struct max17050_dev_s *priv = (FAR struct max17050_dev_s *)dev;
    int ret;

    if (!priv->initialized)
        return -EAGAIN;

    ret = max17050_reg_read(priv, MAX17050_REG_CURRENT);
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

    dbg("%d uA\n", *current);
    return OK;
}

static int max17050_full_capacity(struct battery_dev_s *dev, b16_t *capacity)
{
    FAR struct max17050_dev_s *priv = (FAR struct max17050_dev_s *)dev;
    int ret;

    if (!priv->initialized)
        return -EAGAIN;

    ret = max17050_reg_read(priv, MAX17050_REG_FULL_CAP);
    if (ret < 0)
        return -ENODEV;

    *capacity = ret * (5000000 / max17050_cfg.sns_resistor);

    dbg("%d uAh\n", *capacity);
    return OK;
}

static int max17050_state(struct battery_dev_s *dev, int *status)
{
    FAR struct max17050_dev_s *priv = (FAR struct max17050_dev_s *)dev;
    int ret;
    b16_t value;

    if (!priv->initialized)
        return -EAGAIN;

    ret = max17050_capacity(dev, &value);
    if (ret < 0) {
        *status = BATTERY_UNKNOWN;
        return -ENODEV;
    }

    // TODO: Currently treating 100% capacity as full. Should it be based on
    // capacity or when input current drops below certain threshold?
    if (value >= 100) {
        *status = BATTERY_FULL;
        return OK;
    }

    ret = max17050_current(dev, &value);
    if (ret < 0) {
        *status = BATTERY_UNKNOWN;
        return ret;
    }

    // Positive current means the battery is charging
    *status = (value > 0) ? BATTERY_CHARGING : BATTERY_DISCHARGING;

    dbg("%d\n", *status);
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

#define WRITE(priv, reg, val)                                   \
    do {                                                        \
        int ret = max17050_reg_write(priv, reg, val);           \
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
    WRITE(priv, MAX17050_REG_CONFIG, max17050_cfg.config);
    WRITE(priv, MAX17050_REG_FILTER_CFG, max17050_cfg.filter_cfg);
    WRITE(priv, MAX17050_REG_RELAX_CFG, max17050_cfg.relax_cfg);
    WRITE(priv, MAX17050_REG_LEARN_CFG, max17050_cfg.learn_cfg);
    WRITE(priv, MAX17050_REG_FULL_SOC_THR, max17050_cfg.full_soc_thr);

    return 0;
}

static int max17050_por_unlock_model(FAR struct max17050_dev_s *priv)
{
    WRITE(priv, MAX17050_REG_MODEL_ACCESS_1, MAX17050_MODEL_UNLOCK_1);
    WRITE(priv, MAX17050_REG_MODEL_ACCESS_2, MAX17050_MODEL_UNLOCK_2);

    return 0;
}

static int max17050_por_lock_model(FAR struct max17050_dev_s *priv)
{
    WRITE(priv, MAX17050_REG_MODEL_ACCESS_1, MAX17050_MODEL_LOCK_1);
    WRITE(priv, MAX17050_REG_MODEL_ACCESS_2, MAX17050_MODEL_LOCK_2);

    return 0;
}

static int max17050_por_write_cell_data(FAR struct max17050_dev_s *priv)
{
    int i;

    for (i = 0; i < MAX17050_CELL_DATA_SIZE; i++)
        WRITE(priv, MAX17050_REG_CELL_DATA_START + i, max17050_cfg.cell_data[i]);

    return 0;
}

static void max17050_por_read_cell_data(FAR struct max17050_dev_s *priv, uint16_t cell_data[])
{
    int i;

    for (i = 0; i < MAX17050_CELL_DATA_SIZE; i++)
        cell_data[i] = max17050_reg_read(priv, MAX17050_REG_CELL_DATA_START + i);
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

    WRITE(priv, MAX17050_REG_ICHG_TERM, max17050_cfg.ichg_term);
    WRITE(priv, MAX17050_REG_TGAIN, max17050_cfg.tgain);
    WRITE(priv, MAX17050_REG_TOFF, max17050_cfg.toff);

    WRITE_VERIFY(priv, MAX17050_REG_Q_RESIDUAL_00, max17050_cfg.qr_table_00);
    WRITE_VERIFY(priv, MAX17050_REG_Q_RESIDUAL_10, max17050_cfg.qr_table_10);
    WRITE_VERIFY(priv, MAX17050_REG_Q_RESIDUAL_20, max17050_cfg.qr_table_20);
    WRITE_VERIFY(priv, MAX17050_REG_Q_RESIDUAL_30, max17050_cfg.qr_table_30);

    return 0;
}

static int max17050_por_update_full_capacity_params(FAR struct max17050_dev_s *priv)
{
    WRITE_VERIFY(priv, MAX17050_REG_FULL_CAP, max17050_cfg.capacity);
    WRITE(priv, MAX17050_REG_DESIGN_CAP, max17050_cfg.vf_fullcap);
    WRITE_VERIFY(priv, MAX17050_REG_FULL_CAP_NOM, max17050_cfg.capacity);

    return 0;
}

static int max17050_por_write_vfsoc_and_qh0(FAR struct max17050_dev_s *priv)
{
    int vfsoc = max17050_reg_read(priv, MAX17050_REG_VFSOC);
    int qh = max17050_reg_read(priv, MAX17050_REG_QH);

    WRITE(priv, MAX17050_REG_VFSOC0_QH0_ACCESS, MAX17050_VFSOC0_QH0_UNLOCK);
    WRITE_VERIFY(priv, MAX17050_REG_VFSOC0, vfsoc);
    WRITE(priv, MAX17050_REG_QH0, qh);
    WRITE(priv, MAX17050_REG_VFSOC0_QH0_ACCESS, MAX17050_VFSOC0_QH0_LOCK);

    return 0;
}

static int max17050_por_advance_to_cc_mode(FAR struct max17050_dev_s *priv)
{
    WRITE_VERIFY(priv, MAX17050_REG_CYCLES, MAX17050_CYCLES_INIT_VALUE);
    return 0;
}

static int max17050_por_load_new_capacity_params(FAR struct max17050_dev_s *priv)
{
    int vfsoc = max17050_reg_read(priv, MAX17050_REG_VFSOC);
    int remcap = vfsoc * max17050_cfg.vf_fullcap / 25600;
    int repcap = remcap * max17050_cfg.capacity / max17050_cfg.vf_fullcap;
    int dq_acc = max17050_cfg.vf_fullcap / 16;

    WRITE_VERIFY(priv, MAX17050_REG_REM_CAP, remcap);
    WRITE_VERIFY(priv, MAX17050_REG_REP_CAP, repcap);
    WRITE_VERIFY(priv, MAX17050_REG_DP_ACC, MAX17050_DP_ACC);
    WRITE_VERIFY(priv, MAX17050_REG_DQ_ACC, dq_acc);
    WRITE_VERIFY(priv, MAX17050_REG_FULL_CAP, max17050_cfg.capacity);
    WRITE(priv, MAX17050_REG_DESIGN_CAP, max17050_cfg.vf_fullcap);
    WRITE_VERIFY(priv, MAX17050_REG_FULL_CAP_NOM, max17050_cfg.vf_fullcap);
    WRITE(priv, MAX17050_REG_REP_SOC, vfsoc);

    return 0;
}

static int max17050_por_init(FAR struct max17050_dev_s *priv)
{
    /* Workaround for h/w where sense reistor is connected in reverse to the
     * fuel gauge CSN and CSP inputs
     */
#ifdef CONFIG_BATTERY_MAX17050_REVERSED_SENSE_RESISTOR
    WRITE(priv, MAX17050_REG_CGAIN, MAX17050_CGAIN_REVERSED_SENSE_R);
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
    if (!max17050_reg_modify(priv, MAX17050_REG_STATUS, MAX17050_STATUS_POR, 0)) {
        int status = max17050_reg_read(priv, MAX17050_REG_STATUS);
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
        priv->initialized = true;
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

static int max17050_set_voltage_alert(FAR struct max17050_dev_s *priv, int min,
                                      int max)
{
    uint8_t valrt_min = (min == INT_MIN ? 0x00 : min / 20);
    uint8_t valrt_max = (max == INT_MAX ? 0xff : max / 20);
    uint16_t valrt = valrt_min | valrt_max << 8;

    return max17050_reg_write(priv, MAX17050_REG_VALRT, valrt);
}

static void max17050_alrt_worker(FAR void *arg)
{
    FAR struct max17050_dev_s *priv = arg;
    pthread_t por_thread;
    int status;

    status = max17050_reg_read(priv, MAX17050_REG_STATUS);
    if (status >= 0) {
        if (status & MAX17050_STATUS_VMX) {
            max17050_set_voltage_alert(priv, INT_MIN, INT_MAX);
            pthread_create(&por_thread, NULL, max17050_por, priv);
            pthread_detach(por_thread);
        }
    }
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

static bool max17050_new_config(FAR struct max17050_dev_s *priv)
{
    int ret;

    ret = max17050_reg_read(priv, MAX17050_REG_CONFIG_VER);
    if (ret < 0)
        return false;

    return (max17050_cfg.version != ret);
}

static void max17050_check_por(FAR struct max17050_dev_s *priv)
{
    pthread_t por_thread;
    int ret;

    // Wait in case the device just powered up
    usleep(MAX17050_DELAY_AFTER_POWERUP);

    ret = max17050_reg_read(priv, MAX17050_REG_DEV_NAME);
    if (ret < 0)
        return;

    if (ret != MAX17050_IC_VERSION)
        return;

    ret = max17050_reg_read(priv, MAX17050_REG_STATUS);
    if (ret < 0)
        return;

    // Configure device after Power-On Reset or with the new configuration
    if (ret & MAX17050_STATUS_POR || max17050_new_config(priv)) {
        if (priv->use_voltage_alrt) {
            max17050_set_voltage_alert(priv, INT_MIN,
                                       MAX17050_MIN_BATTERY_VOLTAGE);
            max17050_reg_modify(priv, MAX17050_REG_CONFIG,
                                MAX17050_CONFIG_AEN, MAX17050_CONFIG_AEN);
        } else {
            pthread_create(&por_thread, NULL, max17050_por, priv);
            pthread_detach(por_thread);
        }
    } else
        priv->initialized = true;

}

static void max17050_i2c_available_worker(FAR void *arg)
{
    FAR struct max17050_dev_s *priv = arg;

    priv->initialized = false;
    max17050_check_por(priv);
}

static int max17050_i2c_available_isr(int irq, void *context)
{
    if (work_available(&g_max17050_dev->i2c_available_work)) {
        work_queue(LPWORK, &g_max17050_dev->i2c_available_work,
                   max17050_i2c_available_worker, g_max17050_dev, 0);
        pm_activity(PM_ACTIVITY);
    };

    return OK;
}

FAR struct battery_dev_s *max17050_initialize(FAR struct i2c_dev_s *i2c,
                                              uint32_t frequency,
                                              int alrt_gpio,
                                              int reg_gpio)
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

        g_max17050_dev = priv;

       /*
        * When reg line is asserted, fuel gauge can communicate over I2C, but
        * it is not necessarily fully functional. It becomes fully functional
        * when battery voltage raises above some min threshold.  Use reg_gpio
        * irq to set up fuel gauge Valrt min irq and perform por init after
        * Valrt irq goes off. If the gpios are not utilized, schedule por init
        * right away if the chip requires initialization.
        */
        if (alrt_gpio >= 0) {
            gpio_irqattach(alrt_gpio, max17050_alrt_isr);
            set_gpio_triggering(alrt_gpio, IRQ_TYPE_EDGE_FALLING);

            if (reg_gpio >= 0) {
                priv->use_voltage_alrt = true;
                gpio_irqattach(reg_gpio, max17050_i2c_available_isr);
                set_gpio_triggering(reg_gpio, IRQ_TYPE_EDGE_RISING);
                if (gpio_get_value(reg_gpio) && work_available(&priv->i2c_available_work))
                    work_queue(LPWORK, &priv->i2c_available_work,
                               max17050_i2c_available_worker, priv, 0);
            } else {
                work_queue(LPWORK, &priv->i2c_available_work,
                           max17050_i2c_available_worker, priv, 0);
            }
        } else {
            work_queue(LPWORK, &priv->i2c_available_work,
                       max17050_i2c_available_worker, priv, 0);
        }
    }

    return (FAR struct battery_dev_s *)priv;

err:
    kmm_free(priv);
    return NULL;
}
