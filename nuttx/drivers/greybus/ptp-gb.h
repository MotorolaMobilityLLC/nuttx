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

#ifndef __PTP_GB_H__
#define __PTP_GB_H__

#include <nuttx/greybus/types.h>

#define GB_PTP_TYPE_PROTOCOL_VERSION        0x01
#define GB_PTP_TYPE_GET_FUNCTIONALITY       0x02
#define GB_PTP_TYPE_SET_CURRENT_FLOW        0x03
#define GB_PTP_TYPE_SET_MAX_INPUT_CURRENT   0x04
#define GB_PTP_TYPE_EXT_POWER_CHANGED       0x05
#define GB_PTP_TYPE_EXT_POWER_PRESENT       0x06
#define GB_PTP_TYPE_POWER_REQUIRED_CHANGED  0x07
#define GB_PTP_TYPE_POWER_REQUIRED          0x08
#define GB_PTP_TYPE_POWER_AVAILABLE_CHANGED 0x09    /* added in ver 00.02 */
#define GB_PTP_TYPE_POWER_AVAILABLE         0x0A    /* added in ver 00.02 */
#define GB_PTP_TYPE_POWER_SOURCE            0x0B    /* added in ver 00.02 */
#define GB_PTP_TYPE_GET_MAX_OUTPUT_CURRENT  0x0C    /* added in ver 00.02 */
#define GB_PTP_TYPE_GET_CURRENT_FLOW        0x0D    /* added in ver 00.03 */
#define GB_PTP_TYPE_SET_MAX_OUTPUT_VOLTAGE  0x0E    /* added in ver 00.03 */
#define GB_PTP_TYPE_GET_OUTPUT_VOLTAGE      0x0F    /* added in ver 00.03 */
#define GB_PTP_TYPE_GET_MAX_INPUT_VOLTAGE   0x10    /* added in ver 00.03 */
#define GB_PTP_TYPE_SET_INPUT_VOLTAGE       0x11    /* added in ver 00.03 */

/* Check for operation support */
#define GB_PTP_SUPPORTS(major, minor, name) \
    ((major > GB_PTP_SUPPORT_##name##_MAJOR) || \
    (major == GB_PTP_SUPPORT_##name##_MAJOR && \
    minor >= GB_PTP_SUPPORT_##name##_MINOR))

/* Added in ver 00.02 */
#define GB_PTP_SUPPORT_POWER_AVAILABLE_CHANGED_MAJOR    0x00
#define GB_PTP_SUPPORT_POWER_AVAILABLE_CHANGED_MINOR    0x02

struct gb_ptp_proto_version_request {
    __u8 major;
    __u8 minor;
} __packed;

struct gb_ptp_proto_version_response {
    __u8 major;
    __u8 minor;
} __packed;

struct gb_ptp_get_functionality_response {
    __u8 int_snd;
    __u8 int_rcv;
    __le32 unused;
    __u8 ext;
} __packed;

struct gb_ptp_set_current_flow_request {
    __u8 direction;
} __packed;

struct gb_ptp_get_current_flow_response {
    __u8 direction;
} __packed;

struct gb_ptp_get_max_output_current_response {
    __le32 current;
} __packed;

struct gb_ptp_set_max_input_current_request {
    __le32 current;
} __packed;

struct gb_ptp_ext_power_present_response {
    __u8 present;
} __packed;

struct gb_ptp_power_required_response {
    __u8 required;
} __packed;

struct gb_ptp_power_available_response {
    __u8 available;
} __packed;

struct gb_ptp_power_source_response {
    __u8 source;
} __packed;

struct gb_ptp_set_max_output_voltage_request {
    __le32 voltage;
} __packed;

struct gb_ptp_get_output_voltage_response {
    __le32 voltage;
} __packed;

struct gb_ptp_get_max_input_voltage_response {
    __le32 voltage;
} __packed;

struct gb_ptp_set_input_voltage_request {
    __le32 voltage;
} __packed;

#endif /* __PTP_GB_H__ */
