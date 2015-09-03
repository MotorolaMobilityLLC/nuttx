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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <nuttx/device_battery.h>

/**
 * Show battery technology.
 */
static void do_show_technology(struct device *dev)
{
    uint32_t tech = 0;
    int result = 0;
    char *strtech = NULL;

    result = device_battery_technology(dev, &tech);
    if(!result) {
        switch(tech){
            case BATTERY_TECH_NIMH:
                strtech = "BATTERY_TECH_NIMH";
                break;
            case BATTERY_TECH_LION:
                strtech = "BATTERY_TECH_LION";
                break;
            case BATTERY_TECH_LIPO:
                strtech = "BATTERY_TECH_LIPO";
                break;
            case BATTERY_TECH_LIFE:
                strtech = "BATTERY_TECH_LIFE";
                break;
            case BATTERY_TECH_NICD:
                strtech = "BATTERY_TECH_NICD";
                break;
            case BATTERY_TECH_LIMN:
                strtech = "BATTERY_TECH_LIMN";
                break;
            case BATTERY_TECH_UNKNOWN:
            default:
                strtech = "BATTERY_TECH_UNKNOWN";
                break;
        }
        printf("battery technology: tech = %u, %s \n", tech, strtech);
    } else {
        printf("get_technology failed: %d\n", result);
    }
}

/**
 * Show battery charging status.
 */
static void do_show_status(struct device *dev)
{
    uint16_t status = 0;
    int result = 0;
    char *strstatus = NULL;

    result = device_battery_status(dev, &status);
    if(!result) {
        switch(status){
            case BATTERY_STATUS_CHARGING:
                strstatus = "BATTERY_STATUS_CHARGING";
                break;
            case BATTERY_STATUS_DISCHARGING:
                strstatus = "BATTERY_STATUS_DISCHARGING";
                break;
            case BATTERY_STATUS_NOT_CHARGING:
                strstatus = "BATTERY_STATUS_NOT_CHARGING";
                break;
            case BATTERY_STATUS_FULL:
                strstatus = "BATTERY_STATUS_FULL";
                break;
            case BATTERY_STATUS_UNKNOWN:
            default:
                strstatus = "BATTERY_STATUS_UNKNOWN";
                break;
            }
        printf("battery status = %u, %s \n", status, strstatus);
    } else {
        printf("get_status failed: %d\n", result);
    }
}

/**
 * Show battery max voltage.
 */
static void do_show_max_voltage(struct device *dev)
{
    uint32_t voltage = 0;
    int result = 0;

    result = device_battery_max_voltage(dev, &voltage);
    if(!result) {
        printf("battery max voltage is %u uV\n", voltage);
    } else {
        printf("get_max_voltage failed: %d\n", result);
    }
}

/**
 * Show battery percent capacity.
 */
static void do_show_percent_capacity(struct device *dev)
{
    uint32_t capacity = 0;
    int result = 0;

    result = device_battery_percent_capacity(dev, &capacity);
    if(!result) {
        printf("battery percent capacity = %u \n", capacity);
    } else {
        printf("get_percentage_capacity failed: %d\n", result);
    }
}

/**
 * Show battery temperature.
 */
static void do_show_temperature(struct device *dev)
{
    int temp = 0;
    int result = 0;

    result = device_battery_temperature(dev, &temp);
    if(!result) {
        printf("battery temperature is %d in 0.1 Celsius\n", temp);
    } else {
        printf("get_temperature failed: %d\n", result);
    }
}

/**
 * Show battery voltage.
 */
static void do_show_voltage(struct device *dev)
{
    uint32_t voltage = 0;
    int result = 0;

    result = device_battery_voltage(dev, &voltage);
    if(!result) {
        printf("battery voltage is %u uV\n", voltage);
    } else {
        printf("get_voltage failed: %d\n", result);
    }
}

/**
 * Show battery current.
 */
static void do_show_current(struct device *dev)
{
    int current = 0;
    int result = 0;

    result = device_battery_current(dev, &current);
    if(!result) {
        printf("battery current is %d uA\n", current);
    } else {
        printf("get_current failed: %d\n", result);
    }
}

/**
 * Show battery capacity.
 */
static void do_show_capacity(struct device *dev)
{
    uint32_t capacity = 0;
    int result = 0;

    result = device_battery_total_capacity(dev, &capacity);
    if(!result) {
        printf("battery capacity is %u mAh\n", capacity);
    } else {
        printf("get_capacity failed: %d\n", result);
    }
}

/**
 * Show battery shutdown temperature.
 */
static void do_show_shutdown_temp(struct device *dev)
{
    int temp = 0;
    int result = 0;

    result = device_battery_shutdown_temp(dev, &temp);
    if(!result) {
        printf("battery shutdown temperature is %d in 0.1 Celsius\n", temp);
    } else {
        printf("get_shutdown_temperature failed: %d\n", result);
    }
}

/**
 * Show all battery info.
 */
static void do_all_test(struct device *dev)
{
    do_show_technology(dev);

    do_show_status(dev);

    do_show_voltage(dev);

    do_show_max_voltage(dev);

    do_show_percent_capacity(dev);

    do_show_current(dev);

    do_show_capacity(dev);

    do_show_temperature(dev);

    do_show_shutdown_temp(dev);

    printf("===================================\n");
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[]) {
#else
int battery_test_main(int argc, char *argv[]) {
#endif
    struct device *dev = NULL;

    dev = device_open(DEVICE_TYPE_BATTERY_DEVICE, 0);
    if (!dev) {
        printf("Failed to open battery driver \n");
        return EXIT_FAILURE;
    }

    do_all_test(dev);

    device_close(dev);
    dev = NULL;

    return 0;
}
