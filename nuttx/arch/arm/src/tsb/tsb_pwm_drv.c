/**
 * Copyright (c) 2015 Google, Inc.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <nuttx/lib.h>
#include <nuttx/kmalloc.h>
#include <nuttx/device.h>
#include <nuttx/device_pwm.h>
#include <nuttx/list.h>

#include "up_arch.h"
#include "tsb_scm.h"
#include "tsb_pwm.h"

#define TSB_GENERATOR_COUNTS    2

static uint32_t pwm_pclk[TSB_GENERATOR_COUNTS] = {
    /** For PWM0 */
    TSB_PWM_CLK,

    /** For PWM1 */
    TSB_PWM_CLK
};

/**
 * Data struct for a generator.
 *
 * This struct will be created dynamic when activate and deleted in deactivate.
 */
struct generator_info {
    /** Chain to generator linking list. */
    struct list_head list;

    /** Number for this generator. */
    uint8_t which;

    /** Specific generator of register base. */
    uint32_t gntr_base;

    /** Generator status. */
    uint8_t gntr_flag;

    /** The frequency of the pwm clock (after divider). */
    uint32_t pclk;

    /** The value for pulse of iteration output. */
    uint16_t pulse_count;
};

/**
 * Data struct for PWM controller.
 *
 * This struct store the controller private data for operation management.
 */
struct pwm_ctlr_info {
    /** Generator linking list. */
    struct list_head pwm_list;

    /** Device driver handler. */
    struct device *dev;

    /** Device driver status. */
    uint32_t flags;

    /** Irq number. */
    uint32_t pwm_irq;

    /** Start address of PWM controller registers. */
    uint32_t reg_base;

    /** Supported generators number. */
    uint16_t gntr_counts;

    /**
     * Caller of interrupt Callback handler with state pointer, or interrupt
     * mask value.
     */
    void (*handle)(void *state);

    uint32_t int_state;

    /** Power/clock refere count. */
    uint32_t refcount;

    /** Only one thread can access operation at a time */
    sem_t op_mutex;

    /** Only one thread can access controller of power at a time */
    sem_t pwr_mutex;
};

/** A struct pointer for interrupt routine using. */
static struct device *saved_dev;

/**
 * @brief Write value to register.
 *
 * This function write value to register by basic register write function.
 *
 * @param base Base address of this Controller.
 * @param addr Specific register of offset.
 * @param val The content will be write for Specific register.
 */
static void tsb_pwm_write(uint32_t base, uint32_t addr, uint32_t val)
{
    putreg32(val, base + addr);
}

/**
 * @brief Read value from register.
 *
 * This function returns register content by basic register read function.
 *
 * @param base Base address of this Controller.
 * @param addr Specific register of offset.
 *
 * @return Returns content for a specific register.
 */
static uint32_t tsb_pwm_read(uint32_t base, uint32_t addr)
{
    return getreg32(base + addr);
}

/**
 * @brief Checking common parameter.
 *
 * This function do parameters check and then return status result.
 *
 * @param dev Pointer to the device structure for PWM controller.
 * @param which Specific PWM generator device number.
 *
 * @return 0: Success, error code on failure
 */
static int valid_param(struct device *dev, uint32_t which)
{
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    if (which > ((struct pwm_ctlr_info *)device_get_private(dev))->gntr_counts) {
        return -EINVAL;
    }

    return 0;
}

/**
 * @brief Find device info from linking list.
 *
 * This function find an activated generator_info pointer from driver of
 * linking list.
 *
 * @param info Pointer to the pwm_ctlr_info structure.
 * @param which Specific PWM generator device number.
 *
 * @return return Generator_info struct pointer.
 */
static struct generator_info *get_gntr_info(struct pwm_ctlr_info *info,
                                            uint8_t which)
{
    struct list_head *iter;
    struct generator_info *dev_info = NULL;

    list_foreach(&info->pwm_list, iter) {
        dev_info = list_entry(iter, struct generator_info, list);
        if (dev_info->which == which && (dev_info->gntr_flag &
            TSB_PWM_FLAG_ACTIVED)) {
            return dev_info;
        }
    }

    return NULL;
}

/**
 * @brief Stops a specific generator of toggling.
 *
 * This function stops a specific generator of toggling and sets flag to
 * identify its “disabled” state. If it was not enabled yet, return -EIO error
 * back to the caller. Regardless, stop toggling operation will be completed.
 *
 * @param dev Pointer to the device structure for PWM controller.
 * @param which Specific PWM generator device number.
 *
 * @return 0: Success, error code on failure.
 */
static int tsb_pwm_op_disable(struct device *dev, uint16_t which)
{
    struct pwm_ctlr_info *info = NULL;
    struct generator_info *dev_info = NULL;
    uint32_t reg_cr;
    int ret = 0;

    if (valid_param(dev, which)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->op_mutex);

    dev_info = get_gntr_info(info, which);
    if (!dev_info) {
        ret = -EIO;
        goto err_disable;
    }

    /* Check whether or not this generator has been enabled */
    if (!(dev_info->gntr_flag & TSB_PWM_FLAG_ENABLED)) {
        ret = -EIO;
        goto err_disable;
    }

    reg_cr = tsb_pwm_read(dev_info->gntr_base, TSB_PWM_CR);
    tsb_pwm_write(dev_info->gntr_base, TSB_PWM_CR, reg_cr & ~PWM_CR_ENB);

    dev_info->gntr_flag &= ~TSB_PWM_FLAG_ENABLED;

err_disable:
    sem_post(&info->op_mutex);

    return ret;
}

/**
 * @brief Setup the power state of controller.
 *
 * This function can be called by activate()or by caller directly to setup the
 * power state of controller. If it has not been configured yet, this function
 * will configure power/clock of the PWM controller and then increment the
 * reference count.
 *
 * @param dev Pointer to the device structure for PWM controller.
 *
 * @return 0: Success, error code on failure.
 */
static int tsb_pwm_op_setup(struct device *dev)
{
    struct pwm_ctlr_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->pwr_mutex);

    if (!info->refcount) {
        /* Enable Clock */
        tsb_clk_enable(TSB_CLK_PWMODP);
        tsb_clk_enable(TSB_CLK_PWMODS);

        /* Reset */
        tsb_reset(TSB_RST_PWMODP);
        tsb_reset(TSB_RST_PWMODS);
    }

    info->refcount++;

    sem_post(&info->pwr_mutex);

    return 0;
}

/**
 * @brief Shutdown the power state of controller.
 *
 * This function can be called by deactivate() or by caller directly to setup
 * power state of the controller. If the force_off is set to be true, this
 * function disable power/clock of the controller immediately without checking
 * the reference count. If the force_off is set to be false, this function
 * decrements the reference count.
 * If the reference count become ‘0’, disable power and clock of the PWM
 * controller.
 *
 * @param dev Pointer to the device structure for PWM controller.
 * @param force true for force power off, 0 for checking reference count before
 *              power off.
 *
 * @return 0: Success, error code on failure.
 */
static int tsb_pwm_op_shutdown(struct device *dev, bool force_off)
{
    struct pwm_ctlr_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->pwr_mutex);

    if (force_off) {
        info->refcount = 0;
    } else if (info->refcount) {
        info->refcount--;
    }

    if (info->refcount == 0) {
        tsb_clk_disable(TSB_CLK_PWMODP);
        tsb_clk_disable(TSB_CLK_PWMODS);
    }

    sem_post(&info->pwr_mutex);

    return 0;
}

/**
 * @brief Check device driver open state.
 *
 * This function to check device driver state by pwm_ctlr_info of flags value.
 *
 * @param info Pointer to the pwm_ctlr_info structure.
 *
 * @return true for opened, false for closed
 */
static bool tsb_pwm_device_is_open(struct pwm_ctlr_info *info)
{
    return (info->flags & TSB_PWM_FLAG_OPENED) ? true : false;
}

/**
 * @brief Get the number of generators supported.
 *
 * This function returns the count parameter with the number of generators
 * supported in the system.
 *
 * @param dev Pointer to the device structure for PWM controller.
 * @param count A pointer to the variable of maximum number of supported
 *              generators.
 *
 * @return 0: Success, error code on failure.
*/
static int tsb_pwm_op_count(struct device *dev, uint16_t *count)
{
    struct pwm_ctlr_info *info = NULL;

    if (!dev || !device_get_private(dev) || !count) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->op_mutex);

    info->gntr_counts = TSB_GENERATOR_COUNTS;
    *count = info->gntr_counts;

    sem_post(&info->op_mutex);

    return 0;
}

/**
 * @brief Activates the specific PWM generator.
 *
 * This function allocates resource to store internal data for specific
 * generator and sets flag to identify its ”activated” state.
 * Finally, it calls setup() function to setup new power state in the PWM
 * controller.
 *
 * @param dev Pointer to the device structure for PWM controller.
 * @param which Specific PWM generator device number.
 *
 * @return 0: Success, error code on failure.
 */
static int tsb_pwm_op_activate(struct device *dev, uint16_t which)
{
    struct pwm_ctlr_info *info = NULL;
    struct generator_info *dev_info = NULL;
    struct device_resource *r;
    char string[10];
    int ret = 0;

    if (valid_param(dev, which)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->op_mutex);

    dev_info = get_gntr_info(info, which);
    if (dev_info) {
        ret = -EBUSY;
        goto err_active;
    }

    dev_info = zalloc(sizeof(*dev_info));
    if (!dev_info) {
        ret = -ENOMEM;
        goto err_active;
    }

    /* Get register base by specific generator of name */
    sprintf(string, "pwm%u", which);
    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_REGS, string);
    if (!r) {
        ret = -EINVAL;
        goto err_resc;
    }

    dev_info->gntr_base = (uint32_t)r->start;

    /* Get max support frequency by specific generator of name */
    dev_info->pclk = ((uint32_t *)device_get_init_data(dev))[which];

    /* Turn PWM controller power and clock ON */
    tsb_pwm_op_setup(dev);

    /* Set LVS to 0 when freq = 0 */
    tsb_pwm_write(dev_info->gntr_base, TSB_PWM_CR, PWM_CR_DIV(TSB_PWM_DIV));

    dev_info->which = which;
    dev_info->gntr_flag |= TSB_PWM_FLAG_ACTIVED;

    list_add(&info->pwm_list, &dev_info->list);

    sem_post(&info->op_mutex);

    return ret;

err_resc:
    free(dev_info);
err_active:
    sem_post(&info->op_mutex);

    return ret;
}

/**
 * @brief Deactivates a specific PWM generator.
 *
 * If it was enabled, stops the generator and frees the resources allocated by
 * the “activate” function. Finally, it calls shutdown() to setup new power
 * state in the PWM controller.
 *
 * @param dev Pointer to the device structure for PWM controller.
 * @param which Specific PWM generator device number.
 *
 * @return 0: Success, error code on failure.
 */
static int tsb_pwm_op_deactivate(struct device *dev, uint16_t which)
{
    struct pwm_ctlr_info *info = NULL;
    struct generator_info *dev_info = NULL;
    uint32_t reg_cr;
    int ret = 0;

    if (valid_param(dev, which)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->op_mutex);

    /* If no one find, just return */
    dev_info = get_gntr_info(info, which);
    if (!dev_info) {
        goto no_deactivated;
    }

    if (dev_info->gntr_flag & TSB_PWM_FLAG_ENABLED) {
        reg_cr = tsb_pwm_read(dev_info->gntr_base, TSB_PWM_CR);
        tsb_pwm_write(dev_info->gntr_base, TSB_PWM_CR, reg_cr & ~PWM_CR_ENB);
    }

    /* Turn PWM controller power/clock OFF if no one activated*/
    tsb_pwm_op_shutdown(dev, false);

    list_del(&dev_info->list);
    free(dev_info);

no_deactivated:
    sem_post(&info->op_mutex);

    return ret;
}

/**
 * @brief Configure a specific generator of duty/peroid.
 *
 * This function makes use of duty and period parameters to configure the
 * target generator and sets flag to identify its “configured”’ state.  If it
 * was not activated, return -EIO. If value of duty and period are out of
 * range or value of duty is larger than period, it returns -EINVAL error code.
 * The caller should call activate() before calling this function.
 *
 * @param dev Pointer to the device structure for PWM controller.
 * @param which Specific PWM generator device number.
 *
 * @return 0: Success, error code on failure.
*/
static int tsb_pwm_op_config(struct device *dev, uint16_t which, uint32_t duty,
                             uint32_t period)
{
    struct pwm_ctlr_info *info = NULL;
    struct generator_info *dev_info = NULL;
    uint32_t reg_cr;
    uint32_t set_freq;
    uint32_t set_duty;
    int ret = 0;

    if (valid_param(dev, which) || period == 0 || duty == 0) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->op_mutex);

    dev_info = get_gntr_info(info, which);
    if (!dev_info) {
        ret = -EIO;
        goto err_config;
    }

    /*
     * Because input period and duty is nanosecond, so 1000000000 divided by
     * PCLK will get the minimum unit in nanosecond.
     */
    set_freq = period / (1000000000 / dev_info->pclk);
    if (set_freq <= 1) {
        ret = -EINVAL;
        goto err_config;
    }

    set_duty = duty / (1000000000 / dev_info->pclk);
    if (set_duty > set_freq) {
        ret = -EINVAL;
        goto err_config;
    }

    reg_cr = tsb_pwm_read(dev_info->gntr_base, TSB_PWM_CR) & ~PWM_CR_ENB;
    tsb_pwm_write(dev_info->gntr_base, TSB_PWM_CR, reg_cr);
    dev_info->gntr_flag &= ~TSB_PWM_FLAG_ENABLED;

    tsb_pwm_write(dev_info->gntr_base, TSB_PWM_FREQ, set_freq);
    tsb_pwm_write(dev_info->gntr_base, TSB_PWM_DUTY, set_duty);

    tsb_pwm_write(dev_info->gntr_base, TSB_PWM_ITERATION, 0);

    dev_info->gntr_flag |= TSB_PWM_FLAG_CONFIGURED;

err_config:
    sem_post(&info->op_mutex);

    return ret;
}

/**
 * @brief Start a specific generator of toggling.
 *
 * This function starts a specific generator of toggling by configured
 * frequency and duty. It also sets flag to identify its “enabled” state. If it
 * was not activated and configured, return -EIO. The caller should call
 * config() before calling this function.
 *
 * @param dev Pointer to the device structure for PWM controller.
 * @param which Specific PWM generator device number.
 *
 * @return 0: Success, error code on failure.
*/
static int tsb_pwm_op_enable(struct device *dev, uint16_t which)
{
    struct pwm_ctlr_info *info = NULL;
    struct generator_info *dev_info = NULL;
    uint32_t reg_cr;
    int ret = 0;

    if (valid_param(dev, which)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->op_mutex);

    dev_info = get_gntr_info(info, which);
    if (!dev_info) {
        ret = -EIO;
        goto err_enable;
    }

    /* Check whether or not this generator has been configured */
    if (!(dev_info->gntr_flag & TSB_PWM_FLAG_CONFIGURED)) {
        ret = -EIO;
        goto err_enable;
    }

    /* Start the time */
    reg_cr = tsb_pwm_read(dev_info->gntr_base, TSB_PWM_CR);
    tsb_pwm_write(dev_info->gntr_base, TSB_PWM_CR,
                  reg_cr | PWM_CR_UPD | PWM_CR_ENB);

    dev_info->gntr_flag |= TSB_PWM_FLAG_ENABLED;

err_enable:
    sem_post(&info->op_mutex);

    return ret;
}

/**
 * @brief Set PWM output of polarity.
 *
 * This function sets PWM output of polarity by polarity parameter. If it was
 * not activated yet, return -EIO. The caller should call activate() before
 * calling this function.
 *
 * @param dev Pointer to the device structure for PWM controller.
 * @param which Specific PWM generator device number.
 * @param polarity The active signal of polarity, 0 for normal, 1 for inverted.
 *
 * @return 0: Success, error code on failure.
 */
static int tsb_pwm_op_set_polarity(struct device *dev, uint16_t which,
                                   uint8_t polarity)
{
    struct pwm_ctlr_info *info = NULL;
    struct generator_info *dev_info = NULL;
    uint32_t reg_cr;
    int ret = 0;

    if (valid_param(dev, which) || polarity > 1) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->op_mutex);

    dev_info = get_gntr_info(info, which);
    if (!dev_info) {
        ret = -EIO;
        goto err_set_polarity;
    }

    reg_cr = tsb_pwm_read(dev_info->gntr_base, TSB_PWM_CR);
    if (polarity) {
        reg_cr |= PWM_CR_POL;
    } else {
        reg_cr &= ~PWM_CR_POL;
    }

    tsb_pwm_write(dev_info->gntr_base, TSB_PWM_CR, reg_cr);

err_set_polarity:
    sem_post(&info->op_mutex);

    return ret;
}

/**
 * @brief Set pulses output synchronization.
 *
 * This function make generators that has be configured in same frequency to
 * start pulse at same time.
 *
 * @param dev Pointer to the device structure for PWM controller.
 * @param enable 0 for stop, 1 for start.
 *
 * @return 0: Success, error code on failure.
 */
static int tsb_pwm_op_sync(struct device *dev, bool enable)
{
    struct pwm_ctlr_info *info = NULL;
    uint32_t reg_cr;
    int ret = 0;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->op_mutex);

    if (info->refcount) {
        reg_cr = tsb_pwm_read(info->reg_base, TSB_PWM_GENB);
        if (enable) {
            tsb_pwm_write(info->reg_base, TSB_PWM_GENB, reg_cr |
                          PWM_GENB_GENB0);
        } else {
            tsb_pwm_write(info->reg_base, TSB_PWM_GENB,
                          reg_cr & ~PWM_GENB_GENB0);
        }
    } else {
        ret = -EIO;
    }

    sem_post(&info->op_mutex);

    return ret;
}

/**
 * @brief Set generator to output specific waveform mode.
 *
 * This function request a specific generator to output specific waveform by
 * mode (see pwm_mode) parameter. If it was not activated yet, return -EIO.
 * If the mode is not supported, return -EINVAL. The caller should call
 * activate() before calling this function.
 *
 * @param dev Pointer to the device structure for PWM controller.
 * @param which Specific PWM generator device number.
 * @param mode The mode for generator output.
 * @param param For mode 0, totally iteration times.
 *              For mode 1, false is low level, true is high level.
 *              For mode 2, true to start sync.
 *
 * @return 0: Success, error code on failure.
 */
static int tsb_pwm_op_set_mode(struct device *dev, uint16_t which,
                               uint32_t mode, void *param)
{
    struct pwm_ctlr_info *info = NULL;
    struct generator_info *dev_info = NULL;
    uint32_t reg_cr;
    int ret = 0;
    bool op_state = false;

    if (valid_param(dev, which)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->op_mutex);

    dev_info = get_gntr_info(info, which);
    if (!dev_info) {
        ret = -EIO;
        goto err_set_mode;
    }

    switch(mode) {
    case PWM_PULSECOUNT_MODE:
        dev_info->pulse_count = *(uint16_t *)param;
        if (dev_info->pulse_count > 0) {
            tsb_pwm_write(dev_info->gntr_base, TSB_PWM_ITERATION,
                          dev_info->pulse_count);
        }
        break;
    case PWM_STOP_LEVEL_MODE:
        op_state = *(bool *)param;
        reg_cr = tsb_pwm_read(dev_info->gntr_base, TSB_PWM_CR);
        if (op_state) {
            reg_cr |= PWM_CR_LVS;
        } else {
            reg_cr &= ~PWM_CR_LVS;
        }

        tsb_pwm_write(dev_info->gntr_base, TSB_PWM_CR, reg_cr);
        tsb_pwm_write(dev_info->gntr_base, TSB_PWM_FREQ, 0);
        dev_info->gntr_flag |= TSB_PWM_FLAG_CONFIGURED;
        break;
    case PWM_SYNC_MODE:
        op_state = *(bool *)param;
        reg_cr = tsb_pwm_read(dev_info->gntr_base, TSB_PWM_CR);
        if (op_state) {
            if (dev_info->gntr_flag & TSB_PWM_FLAG_ENABLED) {
                ret = -EIO;
                goto err_set_mode;
            }
            if (!(dev_info->gntr_flag & TSB_PWM_FLAG_CONFIGURED)) {
                ret = -EIO;
                goto err_set_mode;
            }
            reg_cr |= (PWM_CR_ENBSEL | PWM_CR_UPD) ;
            dev_info->gntr_flag |= TSB_PWM_FLAG_ENABLED;
        } else {
            reg_cr &= ~PWM_CR_ENBSEL;
        }
        tsb_pwm_write(dev_info->gntr_base, TSB_PWM_CR, reg_cr);
        break;
    default:
        break;
    }

err_set_mode:
    sem_post(&info->op_mutex);

    return ret;
}

/**
 * @brief Store up-layer call back handler.
 *
 * When caller want to monitor PWM of interrupt states, it will call this
 * function to register callback handler for driver of interrupt routine to
 * notify caller when masked interrupt occurred, the masked bit depend on mask
 * value.
 * If the callback pointer is NULL and mask has value, this function use mask
 * value to unmask bit in TSB_PWM_INTMASK register.
 * The caller should call activate() before calling this function to ensure the
 * power/clock is turn on.
 *
 * @param dev Pointer to the device structure for PWM controller.
 * @param mask Required interrupt monitor state.
 * @param callback Pointer to a callback handler. If this pointer is NULL, this
 *                 function will set the interrupt mask only, otherwise, store
 *                 the callback handler.
 * @param state Pointer to a variable for interrupt status.
 *
 * @return 0: Success, error code on failure.
 */
static int tsb_pwm_op_intr_callback(struct device *dev, uint32_t mask,
                                    void (*callback)(void *state))
{
    struct pwm_ctlr_info *info = NULL;
    uint32_t reg_cr;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->refcount) {
        if (!callback) {
            /* If no callback handler, that mean user want to disable interrupt
             * by masked bit.
             */
            reg_cr = tsb_pwm_read(info->reg_base, TSB_PWM_INTMASK);
            reg_cr |= mask;
            tsb_pwm_write(info->reg_base, TSB_PWM_INTMASK, reg_cr);
        } else {
            info->handle = callback;

            reg_cr = tsb_pwm_read(info->reg_base, TSB_PWM_INTCONFIG);
            reg_cr |= mask;
            tsb_pwm_write(info->reg_base, TSB_PWM_INTCONFIG, reg_cr);

            reg_cr = tsb_pwm_read(info->reg_base, TSB_PWM_INTMASK);
            reg_cr &= ~mask;
            tsb_pwm_write(info->reg_base, TSB_PWM_INTMASK, reg_cr);
        }
    }

    return 0;
}

/**
 * @brief Interrupt event process.
 *
 * When a specific condition match the INTMASK setting, this function be called
 * by IRQ routine to handle specific interrupt status. If caller has registered
 * callback handler, the function calls handler and return with INTSTATUS
 * value.
 *
 * @param irq IRQ number.
 * @param context Pointer to ARM of interrupt registers.
 *
 * @return 0: Success.
 */
static int tsb_pwm_irq_handler(int irq, void *context)
{
    struct pwm_ctlr_info *info = NULL;

    if (!device_get_private(saved_dev)) {
        return ERROR;
    }

    info = device_get_private(saved_dev);

    info->int_state = tsb_pwm_read(info->reg_base, TSB_PWM_INTSTATUS);

    if (info->handle) {
        info->handle(&info->int_state);
    }

    tsb_pwm_write(info->reg_base, TSB_PWM_INTSTATUS, info->int_state);

    return OK;
}

/**
 * @brief Device driver open function.
 *
 * When the caller is preparing to use this driver, it will call this function.
 *
 * @param dev Pointer to the device structure for PWM controller.
 *
 * @return 0: Success, error code on failure.
 */
static int tsb_pwm_dev_open(struct device *dev)
{
    struct pwm_ctlr_info *info = NULL;
    int ret = 0;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->op_mutex);

    if (tsb_pwm_device_is_open(info)) {
        ret = -EBUSY;
        goto err_open;
    }

    up_enable_irq(info->pwm_irq);

    info->flags = TSB_PWM_FLAG_OPENED;

err_open:
    sem_post(&info->op_mutex);

    return ret;
}

/**
 * @brief Device driver close function.
 *
 * When caller is no longer use this driver, it will call this function.
 *
 * @param dev Pointer to the device structure for PWM controller.
 */
static void tsb_pwm_dev_close(struct device *dev)
{
    struct pwm_ctlr_info *info = NULL;
    struct generator_info *dev_info = NULL;
    uint32_t reg_cr;
    int i;

    if (!dev || !device_get_private(dev)) {
        return;
    }

    info = device_get_private(dev);

    sem_wait(&info->op_mutex);

    if (!tsb_pwm_device_is_open(info)) {
        goto err_close;
    }

    up_disable_irq(info->pwm_irq);

    for (i = 0; i < info->gntr_counts; i++) {
        dev_info = get_gntr_info(info, i);
        if (!dev_info) {
            continue;
        } else {
            /* If any of the generator was enabled, stop its output. */
            if (dev_info->gntr_flag & TSB_PWM_FLAG_ENABLED) {
                reg_cr = tsb_pwm_read(dev_info->gntr_base, TSB_PWM_CR);
                tsb_pwm_write(dev_info->gntr_base, TSB_PWM_CR,
                              reg_cr & ~PWM_CR_ENB);
            }

            /*
             * If pointer is valid, it means activated, so free allocated
             * resource too.
             */
            list_del(&dev_info->list);
            free(dev_info);
        }
    }

    /* Finally, shutdown power and clock. */
    tsb_pwm_op_shutdown(dev, true);

    info->flags &= ~TSB_PWM_FLAG_OPENED;

err_close:
    sem_post(&info->op_mutex);
}

/**
 * @brief Device driver probe function.
 *
 * The device core calls this function to register PWM driver during system
 * boot up.
 *
 * @param dev Pointer to the device structure for PWM controller.
 *
 * @return 0: Success, error code on failure.
 */
static int tsb_pwm_dev_probe(struct device *dev)
{
    struct pwm_ctlr_info *info = NULL;
    struct device_resource *r;
    irqstate_t flags;
    int ret = 0;

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
       return -ENOMEM;
    }

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_REGS, "pwm_base");
    if (!r) {
        ret = -EINVAL;
        lldbg("get pwm_base error!\n");
        goto err_resc;
    }

    info->reg_base = (uint32_t)r->start;

    flags = irqsave();

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_IRQ, "pwmintr");
    if (!r) {
        ret = -EINVAL;
        goto err_irq;
    }

    info->pwm_irq = (uint32_t)r->start;

    ret = irq_attach(info->pwm_irq, tsb_pwm_irq_handler);
    if (ret) {
        ret = -EINTR;
        goto err_irq;
    }

    irqrestore(flags);

    /** For PWM1 */
    ret = tsb_request_pinshare(TSB_PIN_GPIO9 | TSB_PIN_UART_CTSRTS);
    if (ret) {
        lowsyslog("PWM: cannot get ownership of PWM pins\n");
        goto err_req_pinshare;
    }

    tsb_clr_pinshare(TSB_PIN_GPIO9);
    tsb_clr_pinshare(TSB_PIN_UART_CTSRTS);

    info->dev = dev;
    device_set_private(dev, info);
    device_set_init_data(dev, pwm_pclk);
    saved_dev = dev;

    info->pwm_list.prev = &info->pwm_list;
    info->pwm_list.next = &info->pwm_list;

    sem_init(&info->op_mutex, 0, 1);
    sem_init(&info->pwr_mutex, 0, 1);

    return ret;

err_irq:
    irqrestore(flags);
err_req_pinshare:
    irq_detach(info->pwm_irq);
err_resc:
    free(info);
    return ret;
}

/**
 * @brief Device driver remove function.
 *
 * The device core calls this function when system unregister PWM driver.
 *
 * @param dev Pointer to the device structure for PWM controller.
 *
 */
static void tsb_pwm_dev_remove(struct device *dev)
{
    struct pwm_ctlr_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return;
    }

    tsb_release_pinshare(TSB_PIN_GPIO9 | TSB_PIN_UART_CTSRTS);

    info = device_get_private(dev);

    sem_destroy(&info->op_mutex);
    sem_destroy(&info->pwr_mutex);

    irq_detach(info->pwm_irq);

    free(info);
    device_set_private(dev, NULL);
}

static struct device_pwm_type_ops tsb_pwm_type_ops = {
    /** Get number of generator supported in system */
    .count        = tsb_pwm_op_count,

    /** Get register base, add to link and record assign status */
    .activate     = tsb_pwm_op_activate,

    /** Release link and unassigned */
    .deactivate   = tsb_pwm_op_deactivate,

    /** Start pulse output by configured freq, duty or mode */
    .enable       = tsb_pwm_op_enable,

    /** Stop pulse output */
    .disable      = tsb_pwm_op_disable,

    /** Set Freq and duty for specific pulse output */
    .config       = tsb_pwm_op_config,

    /** Set a pulse of polarity output */
    .set_polarity = tsb_pwm_op_set_polarity,

    /** sync mode, pulse count mode, pattern mode.. etc. */
    .set_mode     = tsb_pwm_op_set_mode,

    /** Enable controller power/clock */
    .setup        = tsb_pwm_op_setup,

    /** Disable controller power/clock if need */
    .shutdown           = tsb_pwm_op_shutdown,

    /** Start generators of pulse concurrently */
    .sync_output        = tsb_pwm_op_sync,

    /**
     * Provide the caller to register interrupt callback handler or mask bit
     * to disable interrupt.
     */
    .pwm_intr_callback  = tsb_pwm_op_intr_callback,
};

static struct device_driver_ops tsb_pwm_driver_ops = {
    .probe          = tsb_pwm_dev_probe,
    .remove         = tsb_pwm_dev_remove,
    .open           = tsb_pwm_dev_open,
    .close          = tsb_pwm_dev_close,
    .type_ops       = &tsb_pwm_type_ops,
};

struct device_driver tsb_pwm_driver = {
    .type       = DEVICE_TYPE_PWM_HW,
    .name       = "tsb_pwm",
    .desc       = "TTSB PWM Driver",
    .ops        = &tsb_pwm_driver_ops,
};
