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

#ifndef __INCLUDE_NUTTX_DEVICE_PTP_H
#define __INCLUDE_NUTTX_DEVICE_PTP_H

#include <nuttx/device.h>

#include <errno.h>
#include <stdint.h>

#define DEVICE_TYPE_PTP_HW  "ptp"

/**
 * The Mod capability to send power from its internal power source to the Core.
 */
enum ptp_int_snd_functionality {
    /** The power is never sent to the Core. */
    PTP_INT_SND_NEVER           = 0x00,
    /** The Core may request the power at any time. */
    PTP_INT_SND_SUPPLEMENTAL    = 0x01,
    /** The Core should not request the power unless the battery is low. */
    PTP_INT_SND_LOW_BATT_SAVER  = 0x02,
};

/**
 * The Mod capability to charge its internal power source with the power
 * received from the Core.
 */
enum ptp_int_rcv_functionality {
    /** The source is never charged by the power from the Core. */
    PTP_INT_RCV_NEVER           = 0x00,
    /** The Core should charge the Mod before charging itself. */
    PTP_INT_RCV_FIRST           = 0x01,
    /** The Core should charge itself before charging the Mod. */
    PTP_INT_RCV_SECOND          = 0x02,
    /** The Core should alternate between charging itself and the Mod. */
    PTP_INT_RCV_PARALLEL        = 0x03,
};

/**
 * The Mod support of the external charging sources.
 */
enum ptp_ext_functionality {
    /** The Mod does not support external charging sources. */
    PTP_EXT_NONE                = 0x00,
    /** The Mod supports external charging sources. */
    PTP_EXT_SUPPORTED           = 0x01,
};

/**
 * Current flow direction.
 */
enum ptp_current_flow {
    /** No power transfers between the Core and Mod. */
    PTP_CURRENT_OFF             = 0x00,
    /** The Core transfers power to the Mod. */
    PTP_CURRENT_TO_MOD          = 0x01,
    /** The Mod transfers power to the Core. */
    PTP_CURRENT_FROM_MOD        = 0x02,
};

/**
 * External charging sources.
 */
enum ptp_ext_power {
    /** No sources are present. */
    PTP_EXT_POWER_NOT_PRESENT            = 0x00,
    /** Source(s) are present. */
    PTP_EXT_POWER_PRESENT                = 0x01,    /* removed in ver 00.02 */
    /** Wireless charging source is present. */
    PTP_EXT_POWER_WIRELESS_PRESENT       = 0x02,    /* added in ver 00.02 */
    /** Wired charging source is present. */
    PTP_EXT_POWER_WIRED_PRESENT          = 0x03,    /* added in ver 00.02 */
    /** Wired and wireless charging sources are present. */
    PTP_EXT_POWER_WIRED_WIRELESS_PRESENT = 0x04,    /* added in ver 00.02 */

    PTP_EXT_POWER_DONGLE_PRESENT = 0x05,    /* added in ver 00.02 */
};

/* Last version 'PTP_EXT_POWER_PRESENT' response was supported */
#define PTP_EXT_POWER_PRESENT_LAST_SUPPORTED_MAJOR  0x00
#define PTP_EXT_POWER_PRESENT_LAST_SUPPORTED_MINOR  0x01

/**
 * Power that is available to be sent to the Core upon request.
 */
enum ptp_power_available {
    /** No power is available. */
    PTP_POWER_AVAILABLE_NONE    = 0x00,
    /** The power from external charging source will be sent. */
    PTP_POWER_AVAILABLE_EXT     = 0x01,
    /** The power from internal source (aka battery) will be sent. */
    PTP_POWER_AVAILABLE_INT     = 0x02,
};

/**
 * Source of the power that is being sent to the Core.
 */
enum ptp_power_source {
    /** No power is being sent to the Core. */
    PTP_POWER_SOURCE_NONE       = 0x00,
    /** The Mod battery power is being sent to the Core. */
    PTP_POWER_SOURCE_BATTERY    = 0x01,
    /** The power for a wired charger is being sent to the Core. */
    PTP_POWER_SOURCE_WIRED      = 0x02,
    /** The power for a wireless charger is being sent to the Core. */
    PTP_POWER_SOURCE_WIRELESS   = 0x03,
    /** The power for a wireless charger is being sent to the Core. */
    PTP_POWER_SOURCE_DONGLE     = 0x04,
};

/**
 * The Mod internal power source charging needs.
 */
enum ptp_power_required {
    /** The source does not need/cannot be charged. */
    PTP_POWER_NOT_REQUIRED      = 0x00,
    /** The source needs to be charged. */
    PTP_POWER_REQUIRED          = 0x01
};

/**
 * The change in the Mod state.
 */
enum ptp_change {
    /** The External charging source(s) presence changed. */
    POWER_PRESENT,
    /** The Internal power source charging needs changed. */
    POWER_REQUIRED,
    /** The Power availability changed. */
    POWER_AVAILABLE,
};

/**
 * @brief Change notification callback
 * @param change The change in the Mod state.
 * @return 0 on success, negative errno on error.
 */
typedef int (*ptp_changed)(enum ptp_change change);

struct device_ptp_type_ops {
    /** Set the current flow between the Mod and Core. */
    int (*set_current_flow)(struct device *dev, uint8_t direction);
    /** Get the current flow between the Mod and Core. */
    int (*get_current_flow)(struct device *dev, uint8_t *direction);
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    /** External power present on the Mod. */
    int (*ext_power_present)(struct device *dev, uint8_t *present);
#endif
#if !defined (CONFIG_GREYBUS_PTP_INT_SND_NEVER) || defined (CONFIG_GREYBUS_PTP_EXT_SUPPORTED)
    /** Max amount of current that the Core can pull from the Mod. */
    int (*get_max_output_current)(struct device *dev, uint32_t *current);
    /** Power available to be sent from the Mod to the Core. */
    /** Set the max amount of voltage the Mod can supply to the Core. */
    int (*set_max_output_voltage)(struct device *dev, uint32_t voltage);
    /** Voltage of the power supplied to the Core. */
    int (*get_output_voltage)(struct device *dev, uint32_t *voltage);
    int (*power_available)(struct device *dev, uint8_t *available);
    /** Source of the power being sent from the Mod to the Core. */
    int (*power_source)(struct device *dev, uint8_t *source);
#endif
#ifndef CONFIG_GREYBUS_PTP_INT_RCV_NEVER
    /** Set the max amount of current the Mod can pull from the Core. */
    int (*set_max_input_current)(struct device *dev, uint32_t current);
    /** The max voltage the Core can supply to the Mod. */
    int (*get_max_input_voltage)(struct device *dev, uint32_t *voltage);
    /** Voltage of the power supplied to the Mod. */
    int (*set_input_voltage)(struct device *dev, uint32_t voltage);
    /** The Mod need for its battery to be charged. */
    int (*power_required)(struct device *dev, uint8_t *required);
#endif
    /** Register callback to be notified when power presence, needs, availability changes. */
    int (*register_callback)(struct device *dev, ptp_changed cb);
};

/**
 * @brief Set the direction of the current flow.
 * @param dev Pointer to structure of device data.
 * @param direction Direction of the current flow. The possible values are
 *                  defined in the enum ptp_current_flow.
 * @return 0 on success, negative errno on error.
 */
static inline int device_ptp_set_current_flow(struct device *dev,
                                              uint8_t direction)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->set_current_flow) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->set_current_flow(dev, direction);
}

/**
 * @brief Get the direction of the current flow.
 * @param dev Pointer to structure of device data.
 * @param direction Direction of the current flow. The possible values are
 *                  defined in the enum ptp_current_flow.
 * @return 0 on success, negative errno on error.
 */
static inline int device_ptp_get_current_flow(struct device *dev,
                                              uint8_t *direction)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->get_current_flow) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->get_current_flow(dev, direction);
}

#if defined (CONFIG_GREYBUS_PTP_EXT_SUPPORTED)
/**
 * @brief Get the presence of external charging power source
 * @param dev Pointer to structure of device data.
 * @param present The output is the external charging power source(s) currently
 *                present. The possible values are defined in the
 *                enum ptp_ext_power.
 * @return 0 on success, negative errno on error.
 */
static inline int device_ptp_ext_power_present(struct device *dev,
                                               uint8_t *present)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->ext_power_present) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->ext_power_present(dev, present);
}
#endif

#if !defined (CONFIG_GREYBUS_PTP_INT_SND_NEVER) || defined (CONFIG_GREYBUS_PTP_EXT_SUPPORTED)
/**
 * @brief Get the max amount of current the Mod can supply to the Core
 * @param dev pointer to structure of device data.
 * @param current The output is the value of current in microamps.
 * @return 0 on success, negative errno on error.
 */
static inline int device_ptp_get_max_output_current(struct device *dev,
                                                    uint32_t *current)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->get_max_output_current) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->get_max_output_current(dev, current);
}

/**
 * @brief Set the max voltage the Mod can supply to the Core.
 * @param dev Pointer to structure of device data.
 * @param voltage The value of voltage in microvolts.
 * @return 0 on success, negative errno on error.
 */
static inline int device_ptp_set_max_output_voltage(struct device *dev,
                                                    uint32_t voltage)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->set_max_output_voltage) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->set_max_output_voltage(dev, voltage);
}

/**
 * @brief Get the voltage of the power supplied to the Core.
 * @param dev Pointer to structure of device data.
 * @param voltage The value of voltage in microvolts.
 * @return 0 on success, negative errno on error.
 */
static inline int device_ptp_get_output_voltage(struct device *dev,
                                                uint32_t *voltage)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->get_output_voltage) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->get_output_voltage(dev, voltage);
}

/**
 * @brief Availability of power to be sent to the Core
 * @param dev pointer to structure of device data.
 * @param available The output indicates if the Mod can provide power to the
 *                  Core and whether an external or internal power source will
 *                  be used to supply power. The possible values are defined in
 *                  the enum ptp_power_available.
 * @return 0 on success, negative errno on error.
 */
static inline int device_ptp_power_available(struct device *dev,
                                            uint8_t *available)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->power_available) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->power_available(dev, available);
}

/**
 * @brief Source of the power that is being supplied to the Core
 * @param dev Pointer to structure of device data.
 * @param source The output is the power source. The possible values are defined
 *               in the enum ptp_power_source.
 * @return 0 on success, negative errno on error.
 */
static inline int device_ptp_power_source(struct device *dev,
                                            uint8_t *source)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->power_source) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->power_source(dev, source);
}
#endif

#ifndef CONFIG_GREYBUS_PTP_INT_RCV_NEVER
/**
 * @brief Set the maximum amount of current that can be pulled from the Core
 * @param dev Pointer to structure of device data.
 * @param current The value of current in microamps.
 * @return 0 on success, negative errno on error.
 */
static inline int device_ptp_set_max_input_current(struct device *dev,
                                                   uint32_t current)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->set_max_input_current) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->set_max_input_current(dev, current);
}

/**
 * @brief Get the max voltage the Core can supply to the Mod.
 * @param dev Pointer to structure of device data.
 * @param voltage The value of voltage in microvolts.
 * @return 0 on success, negative errno on error.
 */
static inline int device_ptp_get_max_input_voltage(struct device *dev,
                                                   uint32_t *voltage)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->get_max_input_voltage) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->get_max_input_voltage(dev, voltage);
}

/**
 * @brief Set the voltage of the power supplied to the Mod.
 * @param dev Pointer to structure of device data.
 * @param voltage The value of voltage in microvolts.
 * @return 0 on success, negative errno on error.
 */
static inline int device_ptp_set_input_voltage(struct device *dev,
                                               uint32_t voltage)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->set_input_voltage) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->set_input_voltage(dev, voltage);
}

/**
 * @brief Get the Mod power needs
 * @param dev Pointer to structure of device data.
 * @param current The output indicates if the Mod internal power source can be
 *                charged. The possible values are defined in the enum
 *                ptp_power_required.
 * @return 0 on success, negative errno on error.
 */
static inline int device_ptp_power_required(struct device *dev,
                                            uint8_t *required)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->power_required) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->power_required(dev, required);
}
#endif

/**
 * @brief Register notification callback
 * @param dev Pointer to structure of device data.
 * @param cb Callback function executed when there is a change in external power
 *           power presence, or power availability and needs.
 * @return 0 on success, negative errno on error.
 */
static inline int device_ptp_register_callback(struct device *dev,
                                               ptp_changed cb)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->register_callback) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->register_callback(dev, cb);
}
#endif /* __INCLUDE_NUTTX_DEVICE_PTP_H */
