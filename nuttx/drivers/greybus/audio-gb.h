/*
 * Copyright (c) 2015 Motorola Mobility LLC.
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

#ifndef __AUDIO_GB_H__
#define __AUDIO_GB_H__

#include <nuttx/util.h>

#define GB_AUDIO_PROTOCOL_VERSION       0x01

/* Commands */
#define GB_AUDIO_GET_VOLUME_DB_RANGE        0x02
#define GB_AUDIO_GET_SUPPORTED_USE_CASES    0x03
#define GB_AUDIO_SET_USE_CASE       0x04
#define GB_AUDIO_SET_VOLUME     0x05
#define GB_AUDIO_SET_SYSTEM_VOLUME      0x06
#define GB_AUDIO_GET_SUPPORTED_DEVICES      0x07
#define GB_AUDIO_DEVICES_REPORT_EVENT   0x08
#define GB_AUDIO_ENABLE_DEVICES   0x09

/* use cases bit mask*/

/* Music */
#define GB_AUDIO_MUSIC_USE_CASE         BIT(0x1)
/* voice call */
#define GB_AUDIO_VOICE_CALL_SPKR_USE_CASE   BIT(0x2)
/* Low latency stream use cases (ringer, alarm, system sounds ..) which demand min processing delay*/
#define GB_AUDIO_LOW_LATENCY_USE_CASE   BIT(0x3)

/* audio output devices bit mask */
#define GB_AUDIO_DEVICE_OUT_LOUDSPEAKER BIT(0)
#define GB_AUDIO_DEVICE_OUT_HEADSET     BIT(1)
#define GB_AUDIO_DEVICE_OUT_LINE        BIT(2)
#define GB_AUDIO_DEVICE_OUT_HDMI        BIT(3)
#define GB_AUDIO_DEVICE_OUT_EARPIECE    BIT(4)

/* audio input devices bit mask */
#define GB_AUDIO_DEVICE_IN_MIC                   BIT(0)
#define GB_AUDIO_DEVICE_IN_HEADSET_MIC           BIT(1)
#define GB_AUDIO_DEVICE_IN_CAMCORDER_MIC         BIT(2)
#define GB_AUDIO_DEVICE_IN_EC_REF                BIT(3)
#define GB_AUDIO_DEVICE_IN_FM_TUNER              BIT(4)
#define GB_AUDIO_DEVICE_IN_MIC_EC                BIT(5)
#define GB_AUDIO_DEVICE_IN_MIC_ECNS              BIT(6)
#define GB_AUDIO_DEVICE_IN_MIC_NS                BIT(7)

/* version request has no payload */
struct gb_audio_proto_version_response {
    __u8    major;
    __u8    minor;
};

/* Audio bundle will indicate supported use cases to help
 * host make routing decisions.
 * Remote audio bundle should support Music use case at minimum.
 * Remote audio bundle will indicate support for voice call speaker phone
 * use case if it supports either of the three cases
 *  1) echo cancellation on chip,
 *  2) capable of looping back output signal to host
 *  3) no non linear processing, in this case HOST can use internal signal for echo cancellation
 * Remote audio bundle will indicate support for low latency use case if it supports max processing
 * delay required.
*/

struct gb_audio_get_supported_usecases_response {
    __u8                    use_cases;
};

struct gb_aud_vol_range {
    __le32 min;
    __le32 step;
};

struct gb_aud_devices {
    __le32 in_devices;
    __le32 out_devices;
};

/* get volume range min, max and volume step in DB */
struct gb_audio_get_volume_db_range_response {
      struct gb_aud_vol_range vol_range;
};

/* get current volume step of the audio device */
struct gb_audio_get_volume_step_response {
    __le32                  vol_step;
};

/* set current use case */
struct gb_audio_set_use_case_request {
    __u8                    use_case;
};

/* set volume */
struct gb_audio_set_volume_db_request {
    __le32                  vol_step;
};

/* set system volume peak db */
struct gb_audio_set_system_volume_db_request {
    __le32                  vol_db;
};

/* get available audio output devices */
struct gb_audio_get_devices_response {
    struct gb_aud_devices    devices;
};

/* enable audio devices */
struct gb_audio_enable_devices_request {
    struct gb_aud_devices    devices;
};

/* report available audio devices */
struct gb_audio_report_devices_request {
    struct gb_aud_devices    devices;
};
#endif /* __AUDIO_GB_H__ */
