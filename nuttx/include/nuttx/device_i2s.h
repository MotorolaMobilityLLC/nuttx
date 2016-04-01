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

#ifndef __INCLUDE_NUTTX_DEVICE_I2S_H
#define __INCLUDE_NUTTX_DEVICE_I2S_H

#include <errno.h>

#include <nuttx/util.h>
#include <nuttx/device.h>

#define DEVICE_TYPE_I2S_HW                      "i2s"

#define DEVICE_I2S_PCM_FMT_8                    BIT(0)
#define DEVICE_I2S_PCM_FMT_16                   BIT(1)
#define DEVICE_I2S_PCM_FMT_24                   BIT(2)
#define DEVICE_I2S_PCM_FMT_32                   BIT(3)
#define DEVICE_I2S_PCM_FMT_64                   BIT(4)

#define DEVICE_I2S_PCM_RATE_5512                BIT(0)
#define DEVICE_I2S_PCM_RATE_8000                BIT(1)
#define DEVICE_I2S_PCM_RATE_11025               BIT(2)
#define DEVICE_I2S_PCM_RATE_16000               BIT(3)
#define DEVICE_I2S_PCM_RATE_22050               BIT(4)
#define DEVICE_I2S_PCM_RATE_32000               BIT(5)
#define DEVICE_I2S_PCM_RATE_44100               BIT(6)
#define DEVICE_I2S_PCM_RATE_48000               BIT(7)
#define DEVICE_I2S_PCM_RATE_64000               BIT(8)
#define DEVICE_I2S_PCM_RATE_88200               BIT(9)
#define DEVICE_I2S_PCM_RATE_96000               BIT(10)
#define DEVICE_I2S_PCM_RATE_176400              BIT(11)
#define DEVICE_I2S_PCM_RATE_192000              BIT(12)

#define DEVICE_I2S_BYTE_ORDER_NA                BIT(0)
#define DEVICE_I2S_BYTE_ORDER_BE                BIT(1)
#define DEVICE_I2S_BYTE_ORDER_LE                BIT(2)

#define DEVICE_I2S_SPATIAL_LOCATION_FL          BIT(0)
#define DEVICE_I2S_SPATIAL_LOCATION_FR          BIT(1)
#define DEVICE_I2S_SPATIAL_LOCATION_FC          BIT(2)
#define DEVICE_I2S_SPATIAL_LOCATION_LFE         BIT(3)
#define DEVICE_I2S_SPATIAL_LOCATION_BL          BIT(4)
#define DEVICE_I2S_SPATIAL_LOCATION_BR          BIT(5)
#define DEVICE_I2S_SPATIAL_LOCATION_FLC         BIT(6)
#define DEVICE_I2S_SPATIAL_LOCATION_FRC         BIT(7)
#define DEVICE_I2S_SPATIAL_LOCATION_C           BIT(8) /* BC in USB Spec */
#define DEVICE_I2S_SPATIAL_LOCATION_SL          BIT(9)
#define DEVICE_I2S_SPATIAL_LOCATION_SR          BIT(10)
#define DEVICE_I2S_SPATIAL_LOCATION_TC          BIT(11)
#define DEVICE_I2S_SPATIAL_LOCATION_TFL         BIT(12)
#define DEVICE_I2S_SPATIAL_LOCATION_TFC         BIT(13)
#define DEVICE_I2S_SPATIAL_LOCATION_TFR         BIT(14)
#define DEVICE_I2S_SPATIAL_LOCATION_TBL         BIT(15)
#define DEVICE_I2S_SPATIAL_LOCATION_TBC         BIT(16)
#define DEVICE_I2S_SPATIAL_LOCATION_TBR         BIT(17)
#define DEVICE_I2S_SPATIAL_LOCATION_TFLC        BIT(18)
#define DEVICE_I2S_SPATIAL_LOCATION_TFRC        BIT(19)
#define DEVICE_I2S_SPATIAL_LOCATION_LLFE        BIT(20)
#define DEVICE_I2S_SPATIAL_LOCATION_RLFE        BIT(21)
#define DEVICE_I2S_SPATIAL_LOCATION_TSL         BIT(22)
#define DEVICE_I2S_SPATIAL_LOCATION_TSR         BIT(23)
#define DEVICE_I2S_SPATIAL_LOCATION_BC          BIT(24)
#define DEVICE_I2S_SPATIAL_LOCATION_BLC         BIT(25)
#define DEVICE_I2S_SPATIAL_LOCATION_BRC         BIT(26)
#define DEVICE_I2S_SPATIAL_LOCATION_RD          BIT(31)

#define DEVICE_I2S_PROTOCOL_PCM                 BIT(0)
#define DEVICE_I2S_PROTOCOL_I2S                 BIT(1)
#define DEVICE_I2S_PROTOCOL_LR_STEREO           BIT(2)

#define DEVICE_I2S_ROLE_MASTER                  BIT(0)
#define DEVICE_I2S_ROLE_SLAVE                   BIT(1)

#define DEVICE_I2S_POLARITY_NORMAL              BIT(0)
#define DEVICE_I2S_POLARITY_REVERSED            BIT(1)

#define DEVICE_I2S_EDGE_RISING                  BIT(0)
#define DEVICE_I2S_EDGE_FALLING                 BIT(1)

enum device_i2s_event {
    DEVICE_I2S_EVENT_INVALID,
    DEVICE_I2S_EVENT_NONE,
    DEVICE_I2S_EVENT_UNSPECIFIED, /* Catch-all */
    DEVICE_I2S_EVENT_TX_COMPLETE,
    DEVICE_I2S_EVENT_RX_COMPLETE,
    DEVICE_I2S_EVENT_UNDERRUN,
    DEVICE_I2S_EVENT_OVERRUN,
    DEVICE_I2S_EVENT_CLOCKING,
    DEVICE_I2S_EVENT_DATA_LEN,
};

typedef void (*device_i2s_callback)(struct ring_buf *rb,
                                    enum device_i2s_event event, void *arg);
typedef int (*device_i2s_notification_callback)(struct device *dev,
                                    enum device_i2s_event event);

struct device_i2s_configuration {
    uint32_t    sample_frequency;
    uint8_t     num_channels;
    uint8_t     bytes_per_channel;
    uint8_t     byte_order;
    uint8_t     pad;
    uint32_t    spatial_locations;
    uint32_t    ll_protocol;
    uint8_t     ll_mclk_role;
    uint8_t     ll_bclk_role;
    uint8_t     ll_wclk_role;
    uint8_t     ll_wclk_polarity;
    uint8_t     ll_wclk_change_edge;
    uint8_t     ll_data_tx_edge;
    uint8_t     ll_data_rx_edge;
    uint8_t     ll_data_offset;
    uint8_t     ll_pad;
};

struct device_i2s_pcm {
    uint32_t    format;     /* DEVICE_I2S_PCM_FMT_* */
    uint32_t    rate;       /* DEVICE_I2S_PCM_RATE_* */
    uint8_t     channels;
};

/*
 * Low-level DAI communication capabilities of a driver.
 * Most capabilities are bitfields. To find matching capabilities
 * between drivers AND the bitfields returned from get_caps.
 */
struct device_i2s_dai {
    uint32_t    mclk_freq;          /* mclk frequency generated/required */
    uint8_t     protocol;           /* DEVICE_I2S_PROTOCOL_* */
    uint8_t     wclk_polarity;      /* DEVICE_I2S_POLARITY_* */
    uint8_t     wclk_change_edge;   /* DEVICE_I2S_EDGE_* */
    uint8_t     data_rx_edge;       /* DEVICE_I2S_EDGE_* */
    uint8_t     data_tx_edge;       /* DEVICE_I2S_EDGE_* */
};

struct device_i2s_type_ops {
    int (*get_caps)(struct device *dev, uint8_t clk_role,
                    struct device_i2s_pcm *pcm, struct device_i2s_dai *dai);
    int (*set_config)(struct device *dev, uint8_t clk_role,
                      struct device_i2s_pcm *pcm, struct device_i2s_dai *dai);
    int (*get_supported_configurations)(struct device *dev,
                                        uint16_t *configuration_count,
                                        const struct device_i2s_configuration
                                                             *configurations[]);
    int (*set_configuration)(struct device *dev,
                             struct device_i2s_configuration *configuration);
    int (*get_delay_receiver)(struct device *dev, uint32_t *delay);
    int (*prepare_receiver)(struct device *dev, struct ring_buf *rx_rb,
                            device_i2s_callback callback, void *arg);
    int (*start_receiver)(struct device *dev);
    int (*stop_receiver)(struct device *dev);
    int (*start_receiver_port)(struct device *dev);
    int (*stop_receiver_port)(struct device *dev);
    int (*shutdown_receiver)(struct device *dev);
    int (*get_delay_transmitter)(struct device *dev, uint32_t *delay);
    int (*prepare_transmitter)(struct device *dev, struct ring_buf *tx_rb,
                               device_i2s_callback callback, void *arg);
    int (*start_transmitter)(struct device *dev);
    int (*stop_transmitter)(struct device *dev);
    int (*start_transmitter_port)(struct device *dev);
    int (*stop_transmitter_port)(struct device *dev);
    int (*shutdown_transmitter)(struct device *dev);
    int (*register_callback)(struct device *dev, device_i2s_notification_callback cb);
    int (*unregister_callback)(struct device *dev);
};

/**
 * @brief Get device's capabilities with specified PCM values
 * @param dev I2S device being queried
 * @param clk_role Specifies if query is for clock master or slave
 * @param pcm PCM parameters being tested
 * @param dai DAI parameters that support specified PCM parameters
 * @return 0: Delay value returned successfully
 *         -errno: Cause of failure
 */
static inline int device_i2s_get_caps(struct device *dev, uint8_t clk_role,
                                      struct device_i2s_pcm *pcm,
                                      struct device_i2s_dai *dai)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, i2s)->get_caps)
        return DEVICE_DRIVER_GET_OPS(dev, i2s)->get_caps(dev, clk_role, pcm,
                                                         dai);

    return -ENOSYS;
}

static inline int device_i2s_get_supported_configurations(
                        struct device *dev,
                        uint16_t *configuration_count,
                        const struct device_i2s_configuration *configurations[])
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, i2s)->get_supported_configurations)
        return DEVICE_DRIVER_GET_OPS(dev, i2s)->get_supported_configurations(
                                     dev, configuration_count, configurations);

    return -ENOSYS;
}

/**
 * @brief Set supported configuration
 * @param dev I2S device whose configuration is set
 * @param configuration Address of configuration data
 * @return 0: Configuration set successfully
 *         -errno: Cause of failure
 */
static inline int device_i2s_set_configuration(struct device *dev,
                                 struct device_i2s_configuration *configuration)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, i2s)->set_configuration)
        return DEVICE_DRIVER_GET_OPS(dev, i2s)->set_configuration(dev,
                                                                 configuration);

    return -ENOSYS;
}

/**
 * @brief Set current PCM and DAI configuration
 * @param dev I2S device being set
 * @param clk_role Specifies that device is clock master or slave
 * @param pcm Set PCM parameter values
 * @param dai Set DAI parameter values
 * @return 0: Delay value returned successfully
 *         -errno: Cause of failure
 */
static inline int device_i2s_set_config_mask(struct device *dev, uint8_t clk_role,
                                        struct device_i2s_pcm *pcm,
                                        struct device_i2s_dai *dai)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, i2s)->set_config)
        return DEVICE_DRIVER_GET_OPS(dev, i2s)->set_config(dev, clk_role, pcm,
                                                           dai);

    return -ENOSYS;
}

/**
 * @brief Prepare to receive audio data
 * @param dev I2S device to receive audio data
 * @param callback Callback called when ring buffer entry filled
 * @param arg Argument to pass to callback routine
 * @return 0: Successfully prepared to receive audio data
 *         -errno: Cause of failure
 */
static inline int device_i2s_prepare_receiver(struct device *dev,
                                              struct ring_buf *rx_rb,
                                              device_i2s_callback callback,
                                              void *arg)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, i2s)->prepare_receiver)
        return DEVICE_DRIVER_GET_OPS(dev, i2s)->prepare_receiver(dev, rx_rb,
                                                                 callback, arg);

    return -ENOSYS;
}

/**
 * @brief Get receiver start-up delay
 * @param dev I2S device to get delay of
 * @param delay Address to write delay value
 * @return 0: Delay value returned successfully
 *         -errno: Cause of failure
 */
static inline int device_i2s_get_delay_receiver(struct device *dev,
                                                uint32_t *delay)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, i2s)->get_delay_receiver)
        return DEVICE_DRIVER_GET_OPS(dev, i2s)->get_delay_receiver(dev, delay);

    return -ENOSYS;
}

/**
 * Start I2S receiver.  Should be called whenever an empty ring buffer
 * entry is made available to the I2S driver (unless you *know* that the
 * ring buffer is not full).
 *
 * @brief Start receiving audio data
 * @param dev I2S device to receive audio data
 * @return 0: Successfully started receiving audio data
 *         -errno: Cause of failure
 */
static inline int device_i2s_start_receiver(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, i2s)->start_receiver)
        return DEVICE_DRIVER_GET_OPS(dev, i2s)->start_receiver(dev);

    return -ENOSYS;
}

/**
 * @brief Stop receiving audio data
 * @param dev I2S device to stop receiving audio data
 * @return 0: Successfully stopped receiving audio data
 *         -errno: Cause of failure
 */
static inline int device_i2s_stop_receiver(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, i2s)->stop_receiver)
        return DEVICE_DRIVER_GET_OPS(dev, i2s)->stop_receiver(dev);

    return -ENOSYS;
}

/**
 * Start I2S receiver port.  Should be called whenever host requests receiver
 * port to be started, this does not start reception of audio data, this only
 * activates phyiscal I2S port
 *
 * @brief Start receiver port
 * @param dev I2S device to start receiver port
 * @return 0: Successfully started receiver port
 *         -errno: Cause of failure
 */
static inline int device_i2s_start_receiver_port(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, i2s)->start_receiver_port)
        return DEVICE_DRIVER_GET_OPS(dev, i2s)->start_receiver_port(dev);

    return -ENOSYS;
}

/**
 * @brief Stop I2S receiver port
 * @param dev I2S device to stop receiver I2S port
 * @return 0: Successfully stopped receiver I2S port
 *         -errno: Cause of failure
 */
static inline int device_i2s_stop_receiver_port(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, i2s)->stop_receiver_port)
        return DEVICE_DRIVER_GET_OPS(dev, i2s)->stop_receiver_port(dev);

    return -ENOSYS;
}

/**
 * @brief Shutdown audio data receiver (counterpart to preparing receiver)
 * @param dev I2S device whose receiver is shutdown
 * @return 0: Successfully shutdown receiver
 *         -errno: Cause of failure
 */
static inline int device_i2s_shutdown_receiver(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, i2s)->shutdown_receiver)
        return DEVICE_DRIVER_GET_OPS(dev, i2s)->shutdown_receiver(dev);

    return -ENOSYS;
}

/**
 * @brief Get transmitter start-up delay
 * @param dev I2S device to get delay of
 * @param delay Address to write delay value
 * @return 0: Delay value returned successfully
 *         -errno: Cause of failure
 */
static inline int device_i2s_get_delay_transmitter(struct device *dev,
                                                   uint32_t *delay)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, i2s)->get_delay_transmitter)
        return DEVICE_DRIVER_GET_OPS(dev, i2s)->get_delay_transmitter(dev,
                                                                      delay);

    return -ENOSYS;
}

/**
 * @brief Prepare to transmit audio data
 * @param dev I2S device to transmit audio data
 * @param callback Callback called when ring buffer entry emptied
 * @param arg Argument to pass to callback routine
 * @return 0: Successfully prepared to transmit audio data
 *         -errno: Cause of failure
 */
static inline int device_i2s_prepare_transmitter(struct device *dev,
                                                 struct ring_buf *tx_rb,
                                                 device_i2s_callback callback,
                                                 void *arg)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, i2s)->prepare_transmitter)
        return DEVICE_DRIVER_GET_OPS(dev, i2s)->prepare_transmitter(dev, tx_rb,
                                                                   callback,
                                                                   arg);
    return -ENOSYS;
}

/**
 * Start I2S transmitter.  Should be called whenever a non-empty ring buffer
 * entry is made available to the I2S driver.
 *
 * @brief Start transmitting audio data (call when ring buffer entried filled)
 * @param dev I2S device to transmit audio data
 * @return 0: Successfully started transmitting audio data
 *         -errno: Cause of failure
 */
static inline int device_i2s_start_transmitter(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, i2s)->start_transmitter)
        return DEVICE_DRIVER_GET_OPS(dev, i2s)->start_transmitter(dev);

    return -ENOSYS;
}

/**
 * @brief Stop transmitting audio data
 * @param dev I2S device to stop transmitting audio data
 * @return 0: Successfully stopped transmitting audio data
 *         -errno: Cause of failure
 */
static inline int device_i2s_stop_transmitter(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, i2s)->stop_transmitter)
        return DEVICE_DRIVER_GET_OPS(dev, i2s)->stop_transmitter(dev);

    return -ENOSYS;
}

/**
 * Start I2S transmitter port. Should be called whenever host requests transmitter
 * port to be started, this does not start transmission of audio data, this only
 * activates phyiscal I2S port
 *
 * @brief Start I2S Transmitter port
 * @param dev I2S device to start port
 * @return 0: Successfully started port
 *         -errno: Cause of failure
 */
static inline int device_i2s_start_transmitter_port(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, i2s)->start_transmitter_port)
        return DEVICE_DRIVER_GET_OPS(dev, i2s)->start_transmitter_port(dev);

    return -ENOSYS;
}

/**
 * @brief Stop transmitter port
 * @param dev I2S device to stop transmitter port
 * @return 0: Successfully stopped port
 *         -errno: Cause of failure
 */
static inline int device_i2s_stop_transmitter_port(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, i2s)->stop_transmitter_port)
        return DEVICE_DRIVER_GET_OPS(dev, i2s)->stop_transmitter_port(dev);

    return -ENOSYS;
}

/**
 * @brief Shutdown audio data transmitter (counterpart to preparing transmitter)
 * @param dev I2S device whose transmitter is shutdown
 * @return 0: Successfully shutdown transmitter
 *         -errno: Cause of failure
 */
static inline int device_i2s_shutdown_transmitter(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, i2s)->shutdown_transmitter)
        return DEVICE_DRIVER_GET_OPS(dev, i2s)->shutdown_transmitter(dev);

    return -ENOSYS;
}

/**
 * @brief i2s register_callback() wrap function
 *
 * @param dev pointer to structure of device data
 * @param callback callback function for notify event
 * @return 0 on success, negative errno on error
 */
static inline int device_i2s_register_callback(struct device *dev,
        device_i2s_notification_callback cb)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, i2s)->register_callback) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, i2s)->register_callback(dev, cb);
}


/**
 * @brief i2s unregister_callback() wrap function
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static inline int device_i2s_unregister_callback(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, i2s)->unregister_callback) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, i2s)->unregister_callback(dev);
}
#endif /* __INCLUDE_NUTTX_DEVICE_I2S_H */
