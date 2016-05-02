/**
 * Copyright (c) 2015 Google Inc.
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
 *
 * @author Mark Greer
 */

#ifndef __I2S_GB_H__
#define __I2S_GB_H__

#include <nuttx/util.h>

#define GB_I2S_MGMT_TYPE_PROTOCOL_VERSION               0x01
#define GB_I2S_MGMT_TYPE_GET_SUPPORTED_CONFIGURATIONS   0x02
#define GB_I2S_MGMT_TYPE_SET_CONFIGURATION              0x03
#define GB_I2S_MGMT_TYPE_SET_SAMPLES_PER_MESSAGE        0x04
#define GB_I2S_MGMT_TYPE_GET_PROCESSING_DELAY           0x05
#define GB_I2S_MGMT_TYPE_SET_START_DELAY                0x06
#define GB_I2S_MGMT_TYPE_ACTIVATE_CPORT                 0x07
#define GB_I2S_MGMT_TYPE_DEACTIVATE_CPORT               0x08
#define GB_I2S_MGMT_TYPE_REPORT_EVENT                   0x09
#define GB_I2S_MGMT_TYPE_ACTIVATE_PORT                  0x0a
#define GB_I2S_MGMT_TYPE_DEACTIVATE_PORT                0x0b
#define GB_I2S_MGMT_TYPE_START                          0x0c
#define GB_I2S_MGMT_TYPE_STOP                           0x0d

#define GB_I2S_DATA_TYPE_PROTOCOL_VERSION               0x01
#define GB_I2S_DATA_TYPE_SEND_DATA                      0x02

#define GB_I2S_PCM_FMT_8                    BIT(0)
#define GB_I2S_PCM_FMT_16                   BIT(1)
#define GB_I2S_PCM_FMT_24                   BIT(2)
#define GB_I2S_PCM_FMT_32                   BIT(3)
#define GB_I2S_PCM_FMT_64                   BIT(4)

#define GB_I2S_PCM_RATE_5512                BIT(0)
#define GB_I2S_PCM_RATE_8000                BIT(1)
#define GB_I2S_PCM_RATE_11025               BIT(2)
#define GB_I2S_PCM_RATE_16000               BIT(3)
#define GB_I2S_PCM_RATE_22050               BIT(4)
#define GB_I2S_PCM_RATE_32000               BIT(5)
#define GB_I2S_PCM_RATE_44100               BIT(6)
#define GB_I2S_PCM_RATE_48000               BIT(7)
#define GB_I2S_PCM_RATE_64000               BIT(8)
#define GB_I2S_PCM_RATE_88200               BIT(9)
#define GB_I2S_PCM_RATE_96000               BIT(10)
#define GB_I2S_PCM_RATE_176400              BIT(11)
#define GB_I2S_PCM_RATE_192000              BIT(12)

#define GB_I2S_BYTE_ORDER_NA                        BIT(0)
#define GB_I2S_BYTE_ORDER_BE                        BIT(1)
#define GB_I2S_BYTE_ORDER_LE                        BIT(2)

#define GB_I2S_SPATIAL_LOCATION_FL                  BIT(0)
#define GB_I2S_SPATIAL_LOCATION_FR                  BIT(1)
#define GB_I2S_SPATIAL_LOCATION_FC                  BIT(2)
#define GB_I2S_SPATIAL_LOCATION_LFE                 BIT(3)
#define GB_I2S_SPATIAL_LOCATION_BL                  BIT(4)
#define GB_I2S_SPATIAL_LOCATION_BR                  BIT(5)
#define GB_I2S_SPATIAL_LOCATION_FLC                 BIT(6)
#define GB_I2S_SPATIAL_LOCATION_FRC                 BIT(7)
#define GB_I2S_SPATIAL_LOCATION_C                   BIT(8) /* BC in USB Spec */
#define GB_I2S_SPATIAL_LOCATION_SL                  BIT(9)
#define GB_I2S_SPATIAL_LOCATION_SR                  BIT(10)
#define GB_I2S_SPATIAL_LOCATION_TC                  BIT(11)
#define GB_I2S_SPATIAL_LOCATION_TFL                 BIT(12)
#define GB_I2S_SPATIAL_LOCATION_TFC                 BIT(13)
#define GB_I2S_SPATIAL_LOCATION_TFR                 BIT(14)
#define GB_I2S_SPATIAL_LOCATION_TBL                 BIT(15)
#define GB_I2S_SPATIAL_LOCATION_TBC                 BIT(16)
#define GB_I2S_SPATIAL_LOCATION_TBR                 BIT(17)
#define GB_I2S_SPATIAL_LOCATION_TFLC                BIT(18)
#define GB_I2S_SPATIAL_LOCATION_TFRC                BIT(19)
#define GB_I2S_SPATIAL_LOCATION_LLFE                BIT(20)
#define GB_I2S_SPATIAL_LOCATION_RLFE                BIT(21)
#define GB_I2S_SPATIAL_LOCATION_TSL                 BIT(22)
#define GB_I2S_SPATIAL_LOCATION_TSR                 BIT(23)
#define GB_I2S_SPATIAL_LOCATION_BC                  BIT(24)
#define GB_I2S_SPATIAL_LOCATION_BLC                 BIT(25)
#define GB_I2S_SPATIAL_LOCATION_BRC                 BIT(26)
#define GB_I2S_SPATIAL_LOCATION_RD                  BIT(31)

#define GB_I2S_PROTOCOL_PCM                         BIT(0)
#define GB_I2S_PROTOCOL_I2S                         BIT(1)
#define GB_I2S_PROTOCOL_LR_STEREO                   BIT(2)

#define GB_I2S_ROLE_MASTER                          BIT(0)
#define GB_I2S_ROLE_SLAVE                           BIT(1)

#define GB_I2S_POLARITY_NORMAL                      BIT(0)
#define GB_I2S_POLARITY_REVERSED                    BIT(1)

#define GB_I2S_EDGE_RISING                          BIT(0)
#define GB_I2S_EDGE_FALLING                         BIT(1)

#define GB_I2S_EVENT_INVALID           0x1
#define GB_I2S_EVENT_NONE              0x2
#define GB_I2S_EVENT_UNSPECIFIED       0x3
#define GB_I2S_EVENT_TX_COMPLETE       0x4
#define GB_I2S_EVENT_RX_COMPLETE       0x5
#define GB_I2S_EVENT_UNDERRUN          0x6
#define GB_I2S_EVENT_OVERRUN           0x7
#define GB_I2S_EVENT_CLOCKING          0x8
#define GB_I2S_EVENT_DATA_LEN          0x9

#define GB_I2S_MGMT_PORT_TYPE_RECEIVER              0x1
#define GB_I2S_MGMT_PORT_TYPE_TRANSMITTER           0x2

struct gb_i2s_configuration {
    __le32  sample_frequency;
    __u8    num_channels;
    __u8    bytes_per_channel;
    __u8    byte_order;
    __u8    pad;
    __le32  spatial_locations;
    __le32  ll_protocol;
    __u8    ll_mclk_role;
    __u8    ll_bclk_role;
    __u8    ll_wclk_role;
    __u8    ll_wclk_polarity;
    __u8    ll_wclk_change_edge;
    __u8    ll_data_tx_edge;
    __u8    ll_data_rx_edge;
    __u8    ll_data_offset;
};

struct gb_i2s_config_masks {
    __le32  sample_frequency;
    __u8    num_channels;
    __le32  format;
    __u8    protocol;
    __u8    wclk_polarity;
    __u8    wclk_change_edge;
    __u8    data_tx_edge;
    __u8    data_rx_edge;
} __packed;

/* version request has no payload */
struct gb_i2s_proto_version_response {
    __u8    major;
    __u8    minor;
}__packed;

/* get supported configurations request has no payload */
struct gb_i2s_get_supported_configurations_response {
    __u8    config_count;
    __u8    pad[3];
    struct gb_i2s_configuration config[0];
}__packed;

struct gb_i2s_set_configuration_request {
    struct gb_i2s_configuration config;
} __packed;
/* set configuration response has no payload */

struct gb_i2s_get_config_masks_response {
    struct gb_i2s_config_masks config;
}__packed;

struct gb_i2s_set_config_masks_request {
    struct gb_i2s_config_masks config;
} __packed;

struct gb_i2s_set_samples_per_message_request {
    __le16  samples_per_message;
} __packed;
/* set samples per message response has no payload */

/* get processing request delay has no payload */
struct gb_i2s_get_processing_delay_response {
    __le32  microseconds;
} __packed;

struct gb_i2s_set_start_delay_request {
    __le32  microseconds;
} __packed;
/* set start delay response has no payload */

struct gb_i2s_activate_cport_request {
    __le16  cport;
} __packed;
/* activate cport response has no payload */

struct gb_i2s_deactivate_cport_request {
    __le16  cport;
} __packed;
/* deactivate cport response has no payload */

struct gb_i2s_report_event_request {
    __u8    event;
} __packed;
/* report event response has no payload */

struct gb_i2s_activate_port_request {
    __u8    port_type;
} __packed;
/* activate port response has no payload */

struct gb_i2s_deactivate_port_request {
    __u8    port_type;
} __packed;
/* deactivate port response has no payload */

struct gb_i2s_start_request {
    __u8    port_type;
} __packed;
/* start response has no payload */

struct gb_i2s_stop_request {
    __u8    port_type;
} __packed;
/* stop response has no payload */

struct gb_i2s_send_data_request {
    __le32  sample_number;
    __le32  size;
    __u8    data[0];
} __packed;
/* send data has no response at all */

#endif /* __I2S_GB_H__ */
