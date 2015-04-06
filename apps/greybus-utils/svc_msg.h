/*
 * Copyright (c) 2014-2015 Google Inc.
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

#ifndef __SVC_MSG_H
#define __SVC_MSG_H

#include <stdint.h>

typedef uint64_t __le64;
typedef uint8_t __u8;

#include <nuttx/greybus/greybus.h>
#include <nuttx/greybus/types.h>

#include <apps/greybus-utils/debug.h>

#define GREYBUS_VERSION_MAJOR	0x00
#define GREYBUS_VERSION_MINOR	0x01

/* SVC message header + 2 bytes of payload */
#define HP_BASE_SIZE            sizeof(struct svc_msg_header) + 2
#define HS_PAYLOAD_SIZE         (sizeof(struct svc_function_handshake))
#define HS_MSG_SIZE             (sizeof(struct svc_msg_header) +        \
                                        HS_PAYLOAD_SIZE)

#define APID_PAYLOAD_SIZE       (sizeof(struct svc_function_unipro_management))
#define APID_MSG_SIZE           (sizeof(struct svc_msg_header) +        \
                                        APID_PAYLOAD_SIZE)
#define LU_PAYLOAD_SIZE         (sizeof(struct svc_function_unipro_management))
#define LU_MSG_SIZE             (sizeof(struct svc_msg_header) +        \
                                        LU_PAYLOAD_SIZE)

#define HS_VALID(m)							\
	((m->handshake.version_major == GREYBUS_VERSION_MAJOR) &&	\
	(m->handshake.version_minor == GREYBUS_VERSION_MINOR) &&	\
	(m->handshake.handshake_type == SVC_HANDSHAKE_AP_HELLO))

#ifdef CONFIG_ENDIAN_BIG
#define htole16(x) ((LSBYTE(x) << 8) | (MSBYTE(x) & 0xFF))
#define le16toh(x) ((LSBYTE(x) << 8) | (MSBYTE(x) & 0xFF))
#else
#define htole16(x) (x)
#define le16toh(x) (x)
#endif

enum gb_state {
    GBEMU_IDLE = 0,
    GBEMU_HS_COMPLETE = 1,
};

#pragma pack(push, 1)

enum svc_function_id {
    SVC_FUNCTION_HANDSHAKE = 0x00,
    SVC_FUNCTION_UNIPRO_NETWORK_MANAGEMENT = 0x01,
    SVC_FUNCTION_HOTPLUG = 0x02,
    SVC_FUNCTION_POWER = 0x03,
    SVC_FUNCTION_EPM = 0x04,
    SVC_FUNCTION_SUSPEND = 0x05,
};

enum svc_msg_type {
    SVC_MSG_DATA = 0x00,
    SVC_MSG_ERROR = 0xff,
};

struct svc_msg_header {
    __u8 function_id;           /* enum svc_function_id */
    __u8 message_type;
    __le16 payload_length;
};

enum svc_function_handshake_type {
    SVC_HANDSHAKE_SVC_HELLO = 0x00,
    SVC_HANDSHAKE_AP_HELLO = 0x01,
    SVC_HANDSHAKE_MODULE_HELLO = 0x02,
};

struct svc_function_handshake {
    __u8 version_major;
    __u8 version_minor;
    __u8 handshake_type;        /* enum svc_function_handshake_type */
};

struct svc_function_unipro_set_route {
    __u8 device_id;
};

struct svc_function_unipro_link_up {
    __u8 interface_id;
    __u8 device_id;
};

struct svc_function_ap_id {
    __u8 interface_id;
    __u8 device_id;
};

enum svc_function_management_event {
    SVC_MANAGEMENT_AP_ID = 0x00,
    SVC_MANAGEMENT_LINK_UP = 0x01,
    SVC_MANAGEMENT_SET_ROUTE = 0x02,
};

struct svc_function_unipro_management {
    __u8 management_packet_type;        /* enum svc_function_management_event */
    union {
        struct svc_function_ap_id ap_id;
        struct svc_function_unipro_link_up link_up;
        struct svc_function_unipro_set_route set_route;
    };
};

enum svc_function_hotplug_event {
    SVC_HOTPLUG_EVENT = 0x00,
    SVC_HOTUNPLUG_EVENT = 0x01,
};

struct svc_function_hotplug {
    __u8 hotplug_event;         /* enum svc_function_hotplug_event */
    __u8 interface_id;
    __u8 data[0];
};

enum svc_function_power_type {
    SVC_POWER_BATTERY_STATUS = 0x00,
    SVC_POWER_BATTERY_STATUS_REQUEST = 0x01,
};

enum svc_function_battery_status {
    SVC_BATTERY_UNKNOWN = 0x00,
    SVC_BATTERY_CHARGING = 0x01,
    SVC_BATTERY_DISCHARGING = 0x02,
    SVC_BATTERY_NOT_CHARGING = 0x03,
    SVC_BATTERY_FULL = 0x04,
};

struct svc_function_power_battery_status {
    __le16 charge_full;
    __le16 charge_now;
    __u8 status;                /* enum svc_function_battery_status */
};

struct svc_function_power_battery_status_request {
};

/* XXX
 * Each interface block carries power, so it's possible these things
 * are associated with each UniPro device and not just the interface.
 * For now it's safe to assume it's per-interface.
 */
struct svc_function_power {
    __u8 power_type;            /* enum svc_function_power_type */
    __u8 interface_id;
    union {
        struct svc_function_power_battery_status status;
        struct svc_function_power_battery_status_request request;
    };
};

enum svc_function_epm_command_type {
    SVC_EPM_ENABLE = 0x00,
    SVC_EPM_DISABLE = 0x01,
};

/* EPM's are associated with the interface */
struct svc_function_epm {
    __u8 epm_command_type;      /* enum svc_function_epm_command_type */
    __u8 module_id;
};

enum svc_function_suspend_command_type {
    SVC_SUSPEND_FIXME_1 = 0x00, // FIXME
    SVC_SUSPEND_FIXME_2 = 0x01,
};

/* We'll want independent control for multi-interface block interfaces */
struct svc_function_suspend {
    __u8 suspend_command_type;  /* enum function_suspend_command_type */
    __u8 device_id;
};

struct svc_msg {
    struct svc_msg_header header;
    union {
        struct svc_function_handshake handshake;
        struct svc_function_unipro_management management;
        struct svc_function_hotplug hotplug;
        struct svc_function_power power;
        struct svc_function_epm epm;
        struct svc_function_suspend suspend;
    };
};

#pragma pack(pop)

#endif                          /* __SVC_MSG_H */
