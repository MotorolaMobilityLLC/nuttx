/*
 * Copyright (c) 2015 Google Inc.
 * All rights reserved.
 * Author: Benoit Cousson <bcousson@baylibre.com>
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

#ifndef __BATTERY_GB_H__
#define __BATTERY_GB_H__

#include <nuttx/greybus/types.h>

/* Greybus battery request types */
#define GB_BATTERY_TYPE_INVALID             0x00
#define GB_BATTERY_TYPE_PROTOCOL_VERSION    0x01
#define GB_BATTERY_TYPE_TECHNOLOGY          0x02
#define GB_BATTERY_TYPE_STATUS              0x03
#define GB_BATTERY_TYPE_MAX_VOLTAGE         0x04
#define GB_BATTERY_TYPE_PERCENT_CAPACITY    0x05
#define GB_BATTERY_TYPE_TEMPERATURE         0x06
#define GB_BATTERY_TYPE_VOLTAGE             0x07
#define GB_BATTERY_TYPE_CURRENT             0x08
#define GB_BATTERY_TYPE_CAPACITY            0x09
#define GB_BATTERY_TYPE_SHUTDOWN_TEMP       0x0a

/* Should match up with battery types in linux/power_supply.h */
#define GB_BATTERY_TECH_UNKNOWN             0x0000
#define GB_BATTERY_TECH_NIMH                0x0001
#define GB_BATTERY_TECH_LION                0x0002
#define GB_BATTERY_TECH_LIPO                0x0003
#define GB_BATTERY_TECH_LIFE                0x0004
#define GB_BATTERY_TECH_NICD                0x0005
#define GB_BATTERY_TECH_LIMN                0x0006

struct gb_battery_technology_response {
    __le32  technology;
};

/* Should match up with battery status in linux/power_supply.h */
#define GB_BATTERY_STATUS_UNKNOWN           0x0000
#define GB_BATTERY_STATUS_CHARGING          0x0001
#define GB_BATTERY_STATUS_DISCHARGING       0x0002
#define GB_BATTERY_STATUS_NOT_CHARGING      0x0003
#define GB_BATTERY_STATUS_FULL              0x0004

struct gb_battery_status_response {
    __le16  status;
};

struct gb_battery_max_voltage_response {
    __le32  voltage;
};

struct gb_battery_capacity_response {
    __le32  capacity;
};

struct gb_battery_temperature_response {
    __le32  temperature;
};

struct gb_battery_voltage_response {
    __le32  voltage;
};

struct gb_battery_current_response {
    __le32  current;
};

/* version request has no payload */
struct gb_battery_proto_version_response {
    __u8    major;
    __u8    minor;
};

#endif /* __BATTERY_GB_H__ */
