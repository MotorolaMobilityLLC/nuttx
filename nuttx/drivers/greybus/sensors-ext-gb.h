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

#ifndef __SENSORS_EXT_GB_H__
#define __SENSORS_EXT_GB_H__

#include <nuttx/greybus/types.h>

#define GB_SENSORS_EXT_TYPE_INVALID             0x00
#define GB_SENSORS_EXT_TYPE_PROTOCOL_VERSION    0x01
#define GB_SENSORS_EXT_TYPE_SENSOR_COUNT        0x02
#define GB_SENSORS_EXT_TYPE_SENSOR_INFO         0x03
#define GB_SENSORS_EXT_TYPE_START_REPORTING     0x04
#define GB_SENSORS_EXT_TYPE_FLUSH               0x05
#define GB_SENSORS_EXT_TYPE_STOP_REPORTING      0x06
#define GB_SENSORS_EXT_TYPE_EVENT               0x07


struct gb_sensors_ext_proto_version_response {
    __u8 major;
    __u8 minor;
} __packed;

struct gb_sensors_ext_sensor_count_response {
    __u8     count;
} __packed;

struct gb_sensors_ext_sensor_info_request {
    __u8    id;
} __packed;

struct gb_sensors_ext_sensor_info_response {
    __le32    version;
    __le32    type;
    __le32    max_range;
    __le32    resolution;
    __le32    power;
    __le32    min_delay;
    __le32    max_delay;
    __le32    fifo_rec;
    __le32    fifo_mec;
    __le32    flags;
    __le32    scale_int;
    __le32    scale_nano;
    __le32    offset_int;
    __le32    offset_nano;
    __u8      channels;
    __u8      reserved[3];
    __le16    name_len;
    __u8      name[128];
    __le16    vendor_len;
    __u8      vendor[128];
    __le16    string_type_len;
    __u8      string_type[256];
} __packed;

/* this request has no response payload */
struct gb_sensors_ext_start_reporting_request {
    __u8    id;
    __u8    reserved[3];
    __le64  sampling_period;
    __le64  max_report_latency;
} __packed;

/* this request has no response payload */
struct gb_sensors_ext_flush_request {
    __u8    id;
} __packed;

/* flush response and event report request */
struct gb_sensors_ext_event_report {
    __u8    id;
    __u8    flags;
    __le16  readings;
    __le64  reference_time;
    __u8    data_payload[];
} __packed;

struct gb_sensors_ext_report_data {
    __u8    num_sensors_reporting;
    __u8    reserved;
    struct gb_sensors_ext_event_report event_report[];
} __packed;

/* this request has no response payload */
struct gb_sensors_ext_stop_reporting_request {
    __u8    id;
} __packed;

#endif /* __SENSORS_EXT_GB_H__ */
