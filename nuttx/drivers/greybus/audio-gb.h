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
#define GB_AUDIO_GET_VOLUME_DB_RANGE       0x02
#define GB_AUDIO_GET_SUPPORTED_USE_CASES   0x03
#define GB_AUDIO_SET_CAPTURE_USE_CASE      0x04
#define GB_AUDIO_SET_PLAYBACK_USE_CASE     0x05
#define GB_AUDIO_SET_VOLUME                0x06
#define GB_AUDIO_SET_SYSTEM_VOLUME         0x07
#define GB_AUDIO_GET_SUPPORTED_DEVICES     0x08
#define GB_AUDIO_DEVICES_REPORT_EVENT      0x09
#define GB_AUDIO_ENABLE_DEVICES	           0x0a
#define GB_AUDIO_GET_SPEAKER_PRESET_EQ     0x0b
#define GB_AUDIO_GET_MIC_PARAMS            0x0c

/* use cases bit mask*/

/* Playback use cases bit mask*/


#define GB_AUDIO_PLAYBACK_NONE_USE_CASE              BIT(0)
/* Music */
#define GB_AUDIO_PLAYBACK_MUSIC_USE_CASE             BIT(1)
/* Voice call */
#define GB_AUDIO_PLAYBACK_VOICE_CALL_SPKR_USE_CASE   BIT(2)
/* Ringer  */
#define GB_AUDIO_PLAYBACK_RINGTONE_USE_CASE          BIT(3)
/* System sounds */
#define GB_AUDIO_PLAYBACK_SONIFICATION_USE_CASE      BIT(4)


/* Capture use cases bit mask*/

/* Default setting capture */
#define GB_AUDIO_CAPTURE_DEFAULT_USE_CASE     BIT(0)
/* Voice  */
#define GB_AUDIO_CAPTURE_VOICE_USE_CASE       BIT(1)
/* Unprocessed pcm capture */
#define GB_AUDIO_CAPTURE_RAW_USE_CASE         BIT(2)
/* Camcorder */
#define GB_AUDIO_CAPTURE_CAMCORDER_USE_CASE   BIT(3)

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

/* Size of motorola provided camcorder tuning parameters
 * binary file. If the size of this parameters is changed, make
 * sure it is less than maximum greybus message size, current
 * API is defined to send the parameters in one greybus message,
 * It does not break it up in to chunks.
 */
#define GB_AUDIO_MIC_PARAMS_SIZE   1996

/* version request has no payload */
struct gb_audio_proto_version_response {
    __u8    major;
    __u8    minor;
};

/* Audio bundle will indicate supported use cases for
 * capture and playback. Host will make routing decisions,
 * for capture based on the supported use cases.
*/
struct gb_aud_usecases {
    __le32 playback_usecases;
    __le32 capture_usecases;
} __packed;

struct gb_audio_get_supported_usecases_response {
	struct gb_aud_usecases aud_use_cases;
} __packed;

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

/* Set current playback/capture use case.
 * can set unsupported use cases too, its up
 * to the mod to use the info or not to do
 * use case specific tuning.
 */
struct gb_audio_set_use_case_request {
    __le32                  use_case;
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

/* get preset eq for speaker */
struct gb_audio_get_speaker_preset_eq_response {
    __le32                  preset_eq;
};

/* get mic tuning parameters */
struct gb_audio_get_mic_params_response {
    u8  params[GB_AUDIO_MIC_PARAMS_SIZE];
};

#endif /* __AUDIO_GB_H__ */
