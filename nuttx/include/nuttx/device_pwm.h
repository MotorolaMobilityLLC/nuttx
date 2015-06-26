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

#ifndef __INCLUDE_NUTTX_DEVICE_PWM_H
#define __INCLUDE_NUTTX_DEVICE_PWM_H

#include <stdbool.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/util.h>
#include <nuttx/device.h>

#define DEVICE_TYPE_PWM_HW                 "pwm"

/**
 * PWM device driver operations.
 */
struct device_pwm_type_ops {
    /** PWM get count function pointer. */
    int (*count)(struct device *dev, uint16_t *count);

    /** PWM activate generator function pointer. */
    int (*activate)(struct device *dev, uint16_t which);

    /** PPWM deactivate generator function pointer. */
    int (*deactivate)(struct device *dev, uint16_t which);

    /** PWM config generator function pointer. */
    int (*config)(struct device *dev, uint16_t which, uint32_t duty,
                  uint32_t period);

    /** PWM enable generator function pointer. */
    int (*enable)(struct device *dev, uint16_t  which);

    /** PWM disable generator function pointer. */
    int (*disable)(struct device *dev, uint16_t  which);

    /** PWM set generator of polarity function pointer. */
    int (*set_polarity)(struct device *dev, uint16_t  which, uint8_t polarity);

    /** PWM set generator output mode function pointer. */
    int (*set_mode)(struct device *dev, uint16_t  which, uint32_t mode);

    /** PWM setup power/clock of PWM controller function pointer. */
    int (*setup)(struct device *dev);

    /** PWM shutdown power/clock of controller function pointer. */
    int (*shutdown)(struct device *dev, bool force_off);

    /**
     * PWM interrupt callback handling() to notify caller and return status
     * value for further caller and return status value for further processing.
     */
    int (*pwm_intr_callback)(struct device *dev, void (*callback)(void));
};

/**
 * Definition for supported output mode.
 */
enum pwm_mode {
    /** mode 0, finite pulse output */
    PWM_PULSECOUNT_MODE,

    /** mode 1, pulse toggling in high level output */
    PWM_STOP_HIGH_MODE,

    /** mode 2, pulse toggling in low level output. */
    PWM_STOP_LOW_MODE,

    /**
     * mode 3, Stat multiple pwm generators to concurrently output with same
     * FREQUENCY.
     */
    PWM_SYNC_MODE,
};

/**
 * @brief Get the number of generators supported from PWM controlelr.
 *
 * @param dev Opened device driver handle.
 * @param count A pointer to the variable of maximum number of supported
 *              generators.
 */
static inline int device_pwm_request_count(struct device *dev, uint16_t *count)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.pwm);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.pwm->count) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.pwm->count(dev, count);
}

/**
 * @brief Activating a specific generator that system supported.
 *
 * @param dev Opened device driver handle.
 * @param pwm_no The number of Specific generator for operating.
 */
static inline int device_pwm_request_activate(struct device *dev,
                                              uint16_t pwm_no)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.pwm);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.pwm->activate) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.pwm->activate(dev, pwm_no);
}

/**
 * @brief Deativating a specific generator that been actived previously.
 *
 * @param dev Opened device driver handle.
 * @param pwm_no The number of Specific generator for operating.
 */
static inline int device_pwm_request_deactivate(struct device *dev,
                                                uint16_t pwm_no)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.pwm);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.pwm->deactivate) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.pwm->deactivate(dev, pwm_no);
}

/**
 * @brief Configure specific generator for a particular duty cycle and period.
 *
 * @param dev Opened device driver handle.
 * @param pwm_no The number of Specific generator for operating.
 * @param duty Active time (in nanoseconds).
 * @param period Sum of active and deactive time (in nanoseconds).
 */
static inline int device_pwm_request_config(struct device *dev,
                                            uint16_t pwm_no,
                                            uint32_t duty, uint32_t period)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.pwm);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.pwm->config) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.pwm->config(dev, pwm_no, duty, period);
}

/**
 * @brief Configure specific generator for a particular polarity.
 *
 * @param dev Opened device driver handle.
 * @param pwm_no The number of Specific generator for operating.
 * @param polarity 0 for normal, 1 for inverted.
 */
static inline int device_pwm_request_set_polarity(struct device *dev,
                                                  uint16_t pwm_no,
                                                  uint8_t polarity)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.pwm);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.pwm->set_polarity) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.pwm->set_polarity(dev, pwm_no, polarity);
}

/**
 * @brief Enable a specific generator to start toggling.
 *
 * @param dev Opened device driver handle.
 * @param pwm_no The number of Specific generator for operating.
 */
static inline int device_pwm_request_enable(struct device *dev,
                                            uint16_t pwm_no)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.pwm);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.pwm->enable) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.pwm->enable(dev, pwm_no);
}

/**
 * @brief Disable a specific generator toggling.
 *
 * @param dev Opened device driver handle.
 * @param pwm_no The number of Specific generator for operating.
 */
static inline int device_pwm_request_disable(struct device *dev,
                                             uint16_t pwm_no)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.pwm);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.pwm->disable) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.pwm->disable(dev, pwm_no);
}

/**
 * @brief Set generator output mode.
 *
 * This function to set specific generator output in particular mode by 'mode'
 * parameter.
 *
 * @param dev Opened device driver handle.
 * @param pwm_no The number of Specific generator for operating.
 * @param mode A mode number will be set, refer to enum pwm_mode.
 */
static inline int device_pwm_request_set_mode(struct device *dev,
                                              uint16_t pwm_no, uint32_t mode)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.pwm);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.pwm->set_mode) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.pwm->set_mode(dev, pwm_no, mode);
}

/**
 * @brief Setup power/clock of PWM controller.
 *
 * @param dev Opened device driver handle.
 */
static inline int device_pwm_request_setup(struct device *dev)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.pwm);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.pwm->setup) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.pwm->setup(dev);
}

/**
 * @brief Shutdown power/clock of PWM controller.
 *
 * @param dev Opened device driver handle.
 */
static inline int device_pwm_request_shutdown(struct device *dev, bool off)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.pwm);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.pwm->shutdown) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.pwm->shutdown(dev, off);
}
#endif
