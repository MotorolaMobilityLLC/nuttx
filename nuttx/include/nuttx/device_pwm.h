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
    int (*set_mode)(struct device *dev, uint16_t  which, uint32_t mode,
                    void *param);

    /** PWM setup power/clock of PWM controller function pointer. */
    int (*setup)(struct device *dev);

    /** PWM shutdown power/clock of controller function pointer. */
    int (*shutdown)(struct device *dev, bool force_off);

    /** PWM multiple generator of concurrent output. */
    int (*sync_output)(struct device *dev, bool enable);

    /**
     * PWM interrupt callback handling() to notify caller and return status
     * value for further caller and return status value for further processing.
     */
    int (*pwm_intr_callback)(struct device *dev, uint32_t mask,
                             void (*callback)(void *state));
};

/**
 * Definition for supported output mode.
 */
enum pwm_mode {
    /** mode 0, finite pulse output */
    PWM_PULSECOUNT_MODE,

    /**
     * mode 1, true for high level signal, false for low level signal when
     * FREQ = 0.
     */
    PWM_STOP_LEVEL_MODE,

    /**
     * mode 2, Stat multiple pwm generators to concurrently output with same
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
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwm)->count) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwm)->count(dev, count);
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
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwm)->activate) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwm)->activate(dev, pwm_no);
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
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwm)->deactivate) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwm)->deactivate(dev, pwm_no);
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
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwm)->config) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwm)->config(dev, pwm_no, duty, period);
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
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwm)->set_polarity) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwm)->set_polarity(dev, pwm_no, polarity);
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
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwm)->enable) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwm)->enable(dev, pwm_no);
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
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwm)->disable) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwm)->disable(dev, pwm_no);
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
 * @param param For mode 0, totally iteration times.
 *              For mode 1, false is low level, true is high level.
 *              For mode 2, true to start sync.
 */
static inline int device_pwm_request_set_mode(struct device *dev,
                                              uint16_t pwm_no,
                                              uint32_t mode, void *param)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwm)->set_mode) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwm)->set_mode(dev, pwm_no, mode, param);
}

/**
 * @brief Setup power/clock of PWM controller.
 *
 * @param dev Opened device driver handle.
 */
static inline int device_pwm_request_setup(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwm)->setup) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwm)->setup(dev);
}

/**
 * @brief Shutdown power/clock of PWM controller.
 *
 * @param dev Opened device driver handle.
 * @param off true for force power off, 0 for checking reference count before
 *            power off.
 */
static inline int device_pwm_request_shutdown(struct device *dev, bool off)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwm)->shutdown) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwm)->shutdown(dev, off);
}


/**
 * @brief Enable generate waveforms with the same FREQUENCY.
 *
 * @param dev Opened device driver handle.
 * @param enable True for enable, false for disable.
 */
static inline int device_pwm_request_sync(struct device *dev, bool enable)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwm)->sync_output) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwm)->sync_output(dev, enable);
}

/**
 * @brief Callback function register.
 *
 * @param dev Opened device driver handle.
 * @param mask Required interrupt active bite.
 * @param callback Pointer to a callback handler.
 * @param state Pointer to a variable of interrupt status.
 */
static inline int device_pwm_request_callback(struct device *dev,
                                              uint32_t mask,
                                              void (*callback)(void *state))
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwm)->pwm_intr_callback) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwm)->pwm_intr_callback(dev, mask,
                                                              callback);
}

#endif
