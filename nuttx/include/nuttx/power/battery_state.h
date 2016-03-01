/*
 * Copyright (C) 2015 Motorola Mobility, LLC.
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

#ifndef __INCLUDE_NUTTX_POWER_BATTERY_STATE_H
#define __INCLUDE_NUTTX_POWER_BATTERY_STATE_H

/*
 * Battery temperature levels
 */
enum batt_temp_e {
    BATTERY_TEMP_NORMAL,
    BATTERY_TEMP_REDUCED_CHARGING,
    BATTERY_TEMP_NO_CHARGING,
    BATTERY_TEMP_COOL_DOWN,   /* no charging or discharging */
    BATTERY_TEMP_UNAVAILABLE, /* charging is allowed, but no discharging */
};

/*
 * Battery capacity levels
 */
enum batt_level_e {
    BATTERY_LEVEL_EMPTY,
    BATTERY_LEVEL_LOW,
    BATTERY_LEVEL_NORMAL,
    BATTERY_LEVEL_FULL,
};

/*
 * Battery charge zone
 */
enum batt_voltage_e {
    BATTERY_QUICK_CHARGE,
    BATTERY_SLOW_CHARGE,
};

/*
 * Battery state
 */
struct batt_state_s {
    enum batt_temp_e temp;
    enum batt_level_e level;
    enum batt_voltage_e voltage;
};

/* The type of the battery state callback function */
typedef void (*batt_callback_t)(void *arg, const struct batt_state_s *batt);

/****************************************************************************
 * Name: battery_state_init
 *
 * Description:
 *   Initialize battery state changes notifications.
 *
 * Input Parameter:
 *   (None)
 *
 * Returned Value:
 *   0 on success or negative errno on failure
 *
 ****************************************************************************/

int battery_state_init(void);

/****************************************************************************
 * Name: battery_state_register
 *
 * Description:
 *   Register for battery state changes notifications.
 *
 * Input Parameters:
 *   callback - Function to call on changes in battery state.
 *   arg      - Parameter to pass to callback function.
 *
 * Returned Value:
 *   0 on success or negative errno on failure
 *
 ****************************************************************************/

int battery_state_register(batt_callback_t callback, void *arg);

/****************************************************************************
 * Name: battery_state_set_level
 *
 * Description:
 *   Set battery level.
 *
 * Input Parameters:
 *   level - battery level
 *
 * Returned Value:
 *   0 on success or negative errno on failure
 *
 ****************************************************************************/

int battery_state_set_level(enum batt_level_e level);

/****************************************************************************
 * Name: battery_state_set_temp
 *
 * Description:
 *   Set battery temperature.
 *
 * Input Parameters:
 *   temp - battery temperature
 *
 * Returned Value:
 *   0 on success or negative errno on failure
 *
 ****************************************************************************/

int battery_state_set_temp(enum batt_temp_e temp);

/****************************************************************************
 * Name: battery_state_set_voltage
 *
 * Description:
 *   Set battery charge voltage.
 *
 * Input Parameters:
 *   charge zone - quick, slow, stop charge
 *
 * Returned Value:
 *   0 on success or negative errno on failure
 *
 ****************************************************************************/

int battery_state_set_voltage(enum batt_voltage_e voltage);

#endif /* __INCLUDE_NUTTX_POWER_BATTERY_STATE_H */
