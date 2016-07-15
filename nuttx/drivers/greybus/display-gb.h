/*
 * Copyright (c) 2015-2016 Motorola Mobility, LLC.
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

#ifndef _GREYBUS_MODS_DISPLAY__
#define _GREYBUS_MODS_DISPLAY__

#include <nuttx/greybus/greybus.h>

/* Greybus Display operation types */
#define GB_MODS_DISPLAY_TYPE_PROTOCOL_VERSION         0x01
#define GB_MODS_DISPLAY_TYPE_HOST_READY               0x02
#define GB_MODS_DISPLAY_TYPE_GET_CONFIG_SIZE          0x03
#define GB_MODS_DISPLAY_TYPE_GET_CONFIG               0x04
#define GB_MODS_DISPLAY_TYPE_SET_CONFIG               0x05
#define GB_MODS_DISPLAY_TYPE_GET_STATE                0x06
#define GB_MODS_DISPLAY_TYPE_SET_STATE                0x07
#define GB_MODS_DISPLAY_TYPE_NOTIFICATION             0x08

#define GB_MODS_DISPLAY_VERSION_MAJOR              0
#define GB_MODS_DISPLAY_VERSION_MINOR              3

#define GB_MODS_DISPLAY_DISPLAY_TYPE_INVALID          0x00
#define GB_MODS_DISPLAY_DISPLAY_TYPE_DSI              0x01
#define GB_MODS_DISPLAY_DISPLAY_TYPE_DP               0x02

#define GB_MODS_DISPLAY_CONFIG_TYPE_INVALID           0x00
#define GB_MODS_DISPLAY_CONFIG_TYPE_EDID_1P3          0x01
#define GB_MODS_DISPLAY_CONFIG_DSI                    0x02
#define GB_MODS_DISPLAY_CONFIG_TYPE_EDID_DOWNSTREAM   0x03

#define GB_MODS_DISPLAY_STATE_OFF                     0x00
#define GB_MODS_DISPLAY_STATE_ON                      0x01
#define GB_MODS_DISPLAY_STATE_BLANK                   0x02
#define GB_MODS_DISPLAY_STATE_UNBLANK                 0x03

#define GB_MODS_DISPLAY_NOTIFICATION_EVENT_INVALID     0x00
#define GB_MODS_DISPLAY_NOTIFICATION_EVENT_FAILURE     0x01
#define GB_MODS_DISPLAY_NOTIFICATION_EVENT_AVAILABLE   0x02
#define GB_MODS_DISPLAY_NOTIFICATION_EVENT_UNAVAILABLE 0x03
#define GB_MODS_DISPLAY_NOTIFICATION_EVENT_CONNECT     0x04
#define GB_MODS_DISPLAY_NOTIFICATION_EVENT_DISCONNECT  0x05

/* Check for operation support */
#define GB_MODS_DISPLAY_VERSION_SUPPORTS(major, minor, name) \
    ((major > GB_MODS_DISPLAY_SUPPORT_##name##_MAJOR) || \
    (major == GB_MODS_DISPLAY_SUPPORT_##name##_MAJOR && \
    minor >= GB_MODS_DISPLAY_SUPPORT_##name##_MINOR))

/**
 * New in version 0.2
 *   added value to enum display_config_type
 *     DISPLAY_CONFIG_TYPE_EDID_DOWNSTREAM
 */
#define GB_MODS_DISPLAY_SUPPORT_CONFIG_TYPE_EDID_DOWNSTREAM_MAJOR 0
#define GB_MODS_DISPLAY_SUPPORT_CONFIG_TYPE_EDID_DOWNSTREAM_MINOR 2

/**
 * New in version 0.3
 *   added values to GB_MODS_DISPLAY_STATE_* for blanking and unblanking
 */
#define GB_MODS_DISPLAY_SUPPORT_STATE_BLANKING_MAJOR 0
#define GB_MODS_DISPLAY_SUPPORT_STATE_BLANKING_MINOR 3

/**
 * Greybus Display Protocol Version Request
 */
struct gb_mods_display_proto_version_request {
    __u8 major;
    __u8 minor;
} __packed;

/**
 * Greybus Display Protocol Version Response
 */
struct gb_mods_display_proto_version_response {
    __u8 major;
    __u8 minor;
} __packed;

/**
 * Greybus Display Get Display Config Size
 */
struct gb_mods_display_get_display_config_size_response {
    __le32 size;
} __packed;

/**
 * Greybus Display Get Display Config
 */

struct gb_mods_display_get_display_config_response {
    __u8 display_type;
    __u8 config_type;
    __u8 reserved[2];
    __u8 config_data[0];
} __packed;

/**
 * Greybus Display Set Display Config
 */

struct gb_mods_display_set_display_config_request {
    __u8 index;
} __packed;

/**
 * Greybus Display Set Display State
 */

struct gb_mods_display_set_display_state_request {
    __u8 state;
} __packed;

/**
 * Greybus Display Get Display State
 */
struct gb_mods_display_get_display_state_response {
    __u8 state;
} __packed;

/**
 * Greybus Display Notification
 */

struct gb_mods_display_notification_request {
    __u8  event;
} __packed;

#endif
