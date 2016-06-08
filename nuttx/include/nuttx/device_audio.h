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

#ifndef __INCLUDE_NUTTX_DEVICE_AUDIO_H
#define __INCLUDE_NUTTX_DEVICE_AUDIO_H

#include <errno.h>

#include <nuttx/util.h>
#include <nuttx/device.h>

#define DEVICE_TYPE_MUC_AUD_HW   "aud_dev"
#define DEVICE_AUDIO_GB_ID    0
#define DEVICE_AUDIO_MHB_ID    1


#define DEV_AUDIO_PLAYBACK_NONE_USE_CASE              BIT(0)
/* Music */
#define DEV_AUDIO_PLAYBACK_MUSIC_USE_CASE             BIT(1)
/* Voice call */
#define DEV_AUDIO_PLAYBACK_VOICE_CALL_SPKR_USE_CASE   BIT(2)
/* Ringer  */
#define DEV_AUDIO_PLAYBACK_RINGTONE_USE_CASE          BIT(3)
/* System sounds */
#define DEV_AUDIO_PLAYBACK_SONIFICATION_USE_CASE      BIT(4)


/* Capture use cases bit mask*/

/* Default setting capture */
#define DEV_AUDIO_CAPTURE_DEFAULT_USE_CASE     BIT(0)
/* Voice  */
#define DEV_AUDIO_CAPTURE_VOICE_USE_CASE       BIT(1)
/* Unprocessed pcm capture */
#define DEV_AUDIO_CAPTURE_RAW_USE_CASE         BIT(2)
/* Camcorder */
#define DEV_AUDIO_CAPTURE_CAMCORDER_USE_CASE   BIT(3)

/* audio output devices bit mask */
#define DEV_AUDIO_DEVICE_OUT_LOUDSPEAKER BIT(0)
#define DEV_AUDIO_DEVICE_OUT_HEADSET     BIT(1)
#define DEV_AUDIO_DEVICE_OUT_LINE        BIT(2)
#define DEV_AUDIO_DEVICE_OUT_HDMI        BIT(3)
#define DEV_AUDIO_DEVICE_OUT_EARPIECE    BIT(4)

/* audio input devices bit mask */
#define DEV_AUDIO_DEVICE_IN_MIC                   BIT(0)
#define DEV_AUDIO_DEVICE_IN_HEADSET_MIC           BIT(1)
#define DEV_AUDIO_DEVICE_IN_CAMCORDER_MIC         BIT(2)
#define DEV_AUDIO_DEVICE_IN_EC_REF                BIT(3)
#define DEV_AUDIO_DEVICE_IN_FM_TUNER              BIT(4)
#define DEV_AUDIO_DEVICE_IN_MIC_EC                BIT(5)
#define DEV_AUDIO_DEVICE_IN_MIC_ECNS              BIT(6)
#define DEV_AUDIO_DEVICE_IN_MIC_NS                BIT(7)

/* speaker eq preset*/
#define DEV_AUDIO_SPEAKER_PRESET_EQ_NONE                 0
 /* EQ and Bass Enhancement optimized to boost an approximate
  * frequency range of 400Hz - 800Hz.
  */
#define DEV_AUDIO_SPEAKER_PRESET_EQ_SMALL                1
/* EQ and Bass Enhancement optimized to boost an approximate
 * frequency range of 150Hz - 300Hz.
 */
#define DEV_AUDIO_SPEAKER_PRESET_EQ_MEDIUM               2
 /* EQ and Bass Enhancement optimized to boost a frequency range
  * below 150Hz.
  */
#define DEV_AUDIO_SPEAKER_PRESET_EQ_LARGE                3

struct device_aud_vol_range {
    int min;
    int step;
};

struct device_aud_devices {
    uint32_t in_devices;
    uint32_t out_devices;
};

struct device_aud_usecases {
    uint32_t playback_usecases;
    uint32_t capture_usecases;
};

struct device_aud_pcm_config {
    uint32_t    format;     /* DEVICE_I2S_PCM_FMT_* */
    uint32_t    rate;       /* DEVICE_I2S_PCM_RATE_* */
    uint8_t     channels;
};

struct device_aud_i2s_config {
    uint32_t    mclk_freq;          /* mclk frequency generated/required */
    uint8_t     protocol;           /* DEVICE_I2S_PROTOCOL_* */
    uint8_t     wclk_polarity;      /* DEVICE_I2S_POLARITY_* */
    uint8_t     wclk_change_edge;   /* DEVICE_I2S_EDGE_* */
    uint8_t     data_rx_edge;       /* DEVICE_I2S_EDGE_* */
    uint8_t     data_tx_edge;       /* DEVICE_I2S_EDGE_* */
};

/* Add new dai config structures here when we
 * support more than just I2S.
 */
typedef enum {
    DEVICE_AUDIO_DAI_I2S_TYPE,
} device_audio_dai_type;

struct device_aud_dai_config {
    device_audio_dai_type dai_type;
    union dai_config {
        struct device_aud_i2s_config i2s_dai;
    } config;
};


typedef int (*report_devices_cb)(struct device *dev, struct device_aud_devices *devices);


struct device_aud_dev_type_ops {
    int (*get_volume_db_range)(struct device *dev,
                      struct device_aud_vol_range *vol_range);
    int (*get_supported_use_cases)(struct device *dev,struct device_aud_usecases *use_cases);
    int (*set_current_playback_use_case)(struct device *dev, uint32_t use_case);
    int (*set_current_capture_use_case)(struct device *dev, uint32_t use_case);
    int (*set_volume)(struct device *dev, uint32_t vol_step);
    int (*set_sys_volume)(struct device *dev, int vol_db);
    int (*get_supp_devices)(struct device *dev, struct device_aud_devices  *devices);
    int (*enable_devices)(struct device *dev, struct device_aud_devices  *devices);
    int (*get_supp_config)(struct device *dev, struct device_aud_pcm_config *pcm,
                           struct device_aud_dai_config *dai);
    int (*set_config)(struct device *dev, struct device_aud_pcm_config *pcm,
                           struct device_aud_dai_config *dai);
    int (*get_spkr_preset_eq)(struct device *dev, int *preset_eq);
    int (*get_mic_params)(struct device *dev, uint8_t *params, int len);
    int (*rx_dai_start)(struct device *dev);
    int (*tx_dai_start)(struct device *dev);
    int (*rx_dai_stop)(struct device *dev);
    int (*tx_dai_stop)(struct device *dev);
    int (*register_callback)(struct device *dev, report_devices_cb cb);
    int (*unregister_callback)(struct device *dev);
};

/**
 * @brief get volume range,
 * @param dev pointer to structure of device data.
 * @param vol_range output structure containing min volume and volume step size
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_get_volume_db_range(struct device *dev,
                                         struct device_aud_vol_range *vol_range)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->get_volume_db_range)
       return DEVICE_DRIVER_GET_OPS(dev,aud_dev)->get_volume_db_range(dev, vol_range);

    return -ENOSYS;

}

/**
 * @brief get supported use cases
 * @param dev pointer to structure of device data.
 * @param use_cases output structure containing playback and capture use cases
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_get_supported_use_cases(struct device *dev,
                                         struct device_aud_usecases *use_cases)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

     if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->get_supported_use_cases)
         return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->get_supported_use_cases(dev, use_cases);


    return -ENOSYS;

}

/**
 * @brief set current playback use case that is active on the core
 * @param dev pointer to structure of device data.
 * @param use_case playback use case DEV_AUDIO_PLAYBACK_*
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_set_playback_use_case(struct device *dev,
                                                                 uint32_t use_case)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

     if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->set_current_playback_use_case)
        return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->set_current_playback_use_case(dev,
                                         use_case);

    return -ENOSYS;

}

/**
 * @brief set current capture use case that is active on the core
 * @param dev pointer to structure of device data.
 * @param use_case capture use case DEV_AUDIO_CAPTURE_*
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_set_capture_use_case(struct device *dev,
                                                                 uint32_t use_case)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

     if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->set_current_capture_use_case)
        return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->set_current_capture_use_case(dev,
                                        use_case);

    return -ENOSYS;

}

/**
 * @brief set volume on the playback stream
 * @param dev pointer to structure of device data.
 * @param vol_step volume step number to which volume level to be set
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_set_volume(struct device *dev,
                                                uint32_t vol_step)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->set_volume)
        return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->set_volume(dev, vol_step);

    return -ENOSYS;
}

/**
 * @brief set current system volume on the playback stream
 * @param dev pointer to structure of device data.
 * @param vol_db volume level/atenuation applied on the current active playback
 * stream in (db*100) units
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_set_sys_volume(struct device *dev,
                                                int vol_db)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->set_sys_volume)
        return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->set_sys_volume(dev, vol_db);

    return -ENOSYS;
}

/**
 * @brief get supported audio devices
 * @param dev pointer to structure of device data.
 * @param devices output structure containing supported in and out devices.
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_get_supp_devices(struct device *dev,
                                           struct device_aud_devices *devices)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->get_supp_devices)
        return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->get_supp_devices(dev, devices);

    return -ENOSYS;
}

/**
 * @brief enable audio devices
 * @param dev pointer to structure of device data.
 * @param devices structures containing in and out devices bit masks to be enabled
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_enable_devices(struct device *dev,
                                            struct device_aud_devices *devices)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->enable_devices)
        return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->enable_devices(dev, devices);

    return -ENOSYS;
}

/**
 * @brief get supported DAI and PCM configurations
 * @param dev pointer to structure of device data.
 * @param pcm output structure containing rate, format and max channels supported
 * @param dai output structure containing supported low level DAI configurations.
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_get_config(struct device *dev,
                                           struct device_aud_pcm_config *pcm,
                                           struct device_aud_dai_config *dai)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->get_supp_config)
        return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->get_supp_config(dev, pcm, dai);

    return -ENOSYS;
}

/**
 * @brief set supported DAI and PCM configurations
 * @param dev pointer to structure of device data.
 * @param pcm structure containing rate, format and channels configuration used
 * @param dai structure containing low level DAI configurations used
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_set_config(struct device *dev,
                                           struct device_aud_pcm_config *pcm,
                                           struct device_aud_dai_config *dai)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->set_config)
        return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->set_config(dev, pcm, dai);

    return -ENOSYS;
}

/**
 * @brief start receiving playback stream
 * @param dev pointer to structure of device data.
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_rx_dai_start(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->rx_dai_start)
        return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->rx_dai_start(dev);

    return -ENOSYS;
}

/**
 * @brief start transmitting capture stream
 * @param dev pointer to structure of device data.
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_tx_dai_start(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->tx_dai_start)
        return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->tx_dai_start(dev);

    return -ENOSYS;
}

/**
 * @brief stop receiving playback stream
 * @param dev pointer to structure of device data.
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_rx_dai_stop(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->rx_dai_stop)
        return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->rx_dai_stop(dev);

    return -ENOSYS;
}

/**
 * @brief stop transmitting capture stream
 * @param dev pointer to structure of device data.
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_tx_dai_stop(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->tx_dai_stop)
        return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->tx_dai_stop(dev);

    return -ENOSYS;
}

/**
 * @brief register callback method used to report supported audio devices
 * @param dev pointer to structure of device data.
 * @param cb report_devices_cb type callback method
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_register_callback(struct device *dev,
                                 report_devices_cb cb )
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->register_callback)
        return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->register_callback(dev, cb);

    return -ENOSYS;
}

/**
 * @brief unregister callback
 * @param dev pointer to structure of device data.
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_unregister_callback(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->unregister_callback)
        return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->unregister_callback(dev);

    return -ENOSYS;
}

/**
 * @brief get eq preset profile that will be applied on the playback stream
 * for speaker device
 * @param dev pointer to structure of device data.
 * @param preset_eq output EQ DEV_AUDIO_SPEAKER_PRESET_EQ_* type
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_get_spkr_preset_eq(struct device *dev,
                                                         int *preset_eq)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->get_spkr_preset_eq)
        return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->get_spkr_preset_eq(dev,
                                                                    preset_eq);

    return -ENOSYS;
}

/**
 * @brief get mic tuning parameters that will applied on the captured
 * multimedia stream. These tuning params will be provided my motorola
 * and is not applicable for voice call use cases.
 * @param dev pointer to structure of device data.
 * @param params output mic tuning parameters
 * @param len size of mic tuning parameters
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_get_mic_params(struct device *dev,
                                                  uint8_t *params, int len)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->get_mic_params)
        return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->get_mic_params(dev,
                                                             params, len);

    return -ENOSYS;
}
#endif
