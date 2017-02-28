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

#ifndef __INCLUDE_NUTTX_EXT_POWER_H
#define __INCLUDE_NUTTX_EXT_POWER_H

#include <errno.h>

/**
 * Exeternal power source ID/types: 0 .. (EXT_POWER_NUMBER_OF_SOURCES - 1).
 * DEVICE_TYPE_EXT_POWER_HW devices (nuttx/device_ext_power.h) that need to be
 * tracked by the notifier must use these IDs as instance numbers (device IDs).
 * Since devices must use unique IDs, only one instance of each type of an
 * external power source can be tracked by this notifier.
 */
typedef enum ext_power_source {
    EXT_POWER_WIRED,
    EXT_POWER_WIRELESS,
    EXT_POWER_DONGLE,
    EXT_POWER_NUMBER_OF_SOURCES
} ext_power_source_e;

/** State of external power device changed notification callback
 *
 * @param arg value provided during callback registration
 * @param dev external power devices. The power source IDs are the indices to
 *            the array.
 */
typedef void (*ext_power_notification)(void *arg, struct device *const dev[]);

/**
 * @brief Register external power sources state notification callback
 *
 * @param callback function to run when state of external power sources changes.
 *                 The first callback is executed within the register call to
 *                 report intitial state.
 * @param arg user provided value that will be returned in callback
 * @return 0 on success, negative errno on error
 */
#ifdef CONFIG_EXT_POWER
int ext_power_register_callback(ext_power_notification callback, void *arg);
#else
static int ext_power_register_callback(ext_power_notification callback, void *arg)
{
    return -ENODEV;
}
#endif

/**
 * @brief Set maximum output voltage of external power source
 *
 * @param source external power source.
 * @param voltage maximum output voltage in mV.
 * @return 0 on success, negative errno on error.
 */
#ifdef CONFIG_EXT_POWER
int ext_power_set_max_output_voltage(ext_power_source_e source, int voltage);
#else
static int ext_power_set_max_output_voltage(ext_power_source_e source, int voltage)
{
    return -ENODEV;
}
#endif
#endif /* __INCLUDE_NUTTX_EXT_POWER_H */
