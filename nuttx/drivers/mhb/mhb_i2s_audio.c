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
#include <errno.h>
#include <debug.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/util.h>
#include <nuttx/time.h>

#include <nuttx/device.h>
#include <nuttx/device_i2s.h>
#include <nuttx/device_audio.h>
#include <nuttx/device_slave_pwrctrl.h>
#include <nuttx/mhb/device_mhb.h>
#include <nuttx/mhb/mhb_utils.h>
#include <nuttx/i2s_tunnel/i2s_unipro.h>
#include <nuttx/mhb/mhb_protocol.h>
#include <nuttx/wqueue.h>

#define MHB_I2S_OP_TIMEOUT_MS     2100

#define MHB_I2S_RX_ACTIVE    BIT(0)
#define MHB_I2S_TX_ACTIVE    BIT(1)
#define MHB_I2S_LOG_TUNNEL_STATS 1
#define MHB_I2S_STATE_PENDING 0x80

struct mhb_response {
    struct mhb_hdr hdr;
    uint8_t *payload;
    size_t payload_length;
};

enum mhb_i2s_audio_state {
    MHB_I2S_AUDIO_OFF,
    MHB_I2S_AUDIO_START,
    MHB_I2S_AUDIO_CONFIG,
    MHB_I2S_AUDIO_START_STREAM,
    MHB_I2S_AUDIO_STOP_STREAM,
};

struct mhb_i2s_audio {
    struct device *i2s_dev;
    struct device *audio_dev;
    struct device *mhb_dev;
    struct device_i2s_pcm common_pcm_cfg;
    struct device_i2s_dai common_dai_cfg;
    uint8_t i2s_status;
    enum mhb_i2s_audio_state state;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    sem_t lock;
    struct mhb_response last_rsp;
    device_i2s_notification_callback callback;
    struct device *slave_pwr_ctrl;
    int slave_state;
    struct mhb_i2s_config config;
    struct device_aud_pcm_config set_codec_pcm_cfg;
    struct device_aud_dai_config set_codec_dai_cfg;
};

static struct work_s mhb_i2s_work;
static struct mhb_i2s_audio mhb_i2s;

/*
 * I2S configuration supported by TSB, as we dont expect this to change
 * config is harcoded here, in future if we need we can query APBE to
 * to get supported configurations
 */
static const struct device_i2s_pcm tsb_i2s_pcm = {
    .rate = (DEVICE_I2S_PCM_RATE_16000 | DEVICE_I2S_PCM_RATE_48000),
    .format = DEVICE_I2S_PCM_FMT_16 | DEVICE_I2S_PCM_FMT_24,
    .channels = 2,
};

static const struct device_i2s_dai tsb_i2s_dai = {
    .protocol = DEVICE_I2S_PROTOCOL_I2S | DEVICE_I2S_PROTOCOL_LR_STEREO,
    .wclk_change_edge = DEVICE_I2S_EDGE_FALLING | DEVICE_I2S_EDGE_RISING,
    .data_rx_edge = DEVICE_I2S_EDGE_RISING,
    .data_tx_edge = DEVICE_I2S_EDGE_FALLING,
    .wclk_polarity = DEVICE_I2S_POLARITY_NORMAL | DEVICE_I2S_POLARITY_REVERSED,
};

/* operation signalling */
static void _mhb_i2s_signal_response(struct mhb_i2s_audio *i2s)
{
    pthread_mutex_lock(&i2s->mutex);
    pthread_cond_signal(&i2s->cond);
    pthread_mutex_unlock(&i2s->mutex);
}

static int _mhb_i2s_wait_for_response(struct mhb_i2s_audio *i2s )
{
    int ret;
    struct timespec expires;

    if (clock_gettime(CLOCK_REALTIME, &expires)) {
        return -EBADF;
    }

    uint64_t new_ns = timespec_to_nsec(&expires);
    new_ns += MHB_I2S_OP_TIMEOUT_MS * NSEC_PER_MSEC;
    nsec_to_timespec(new_ns, &expires);
    sem_post(&i2s->lock);
    pthread_mutex_lock(&i2s->mutex);
    llvdbg("waiting for response ..\n");
    ret = pthread_cond_timedwait(&i2s->cond, &i2s->mutex, &expires);
    pthread_mutex_unlock(&i2s->mutex);
    sem_wait(&i2s->lock);

    if (ret) {
        /* timeout or other errors */
        lldbg("ERROR: wait error %d\n", ret);
        return -ETIME;
    }

    return 0;
}

#ifdef MHB_I2S_LOG_TUNNEL_STATS
static void mhb_print_i2s_status(const char *header, struct i2s_tunnel_info_s *info)
{
    FROM_LE(info->enabled);
    FROM_LE(info->is_master);
    FROM_LE(info->bclk_rate);
    FROM_LE(info->bytes_per_sample);
    FROM_LE(info->roundtrip_timer_ticks);
    FROM_LE(info->timer_offset);
    FROM_LE(info->i2s_tunnel_rx_packets);
    FROM_LE(info->i2s_tunnel_packet_size);
    FROM_LE(info->i2s_tx_samples_dropped);
    FROM_LE(info->i2s_tx_samples_retransmitted);
    FROM_LE(info->i2s_tx_buffers_dropped);

    lldbg("%s:\n", header);
    lldbg("  Enabled:                      %u\n", (info->enabled));
    lldbg("  Master:                       %u\n", (info->is_master));
    lldbg("  Clock Rate:                   %u\n", (info->bclk_rate));
    lldbg("  Bytes per sample:             %u\n", (info->bytes_per_sample));
    lldbg("  Round trip 48MHz timer ticks: %u\n", (info->roundtrip_timer_ticks));
    lldbg("  Timer offset ticks:           %d\n", (info->timer_offset));
    lldbg("  Packets received:             %u\n", (info->i2s_tunnel_rx_packets));
    lldbg("  Packet size:                  %u\n", (info->i2s_tunnel_packet_size));
    lldbg("  Samples removed:              %u\n", (info->i2s_tx_samples_dropped));
    lldbg("  Samples repeated:             %u\n", (info->i2s_tx_samples_retransmitted));
    lldbg("  Packets dropped:              %u\n", (info->i2s_tx_buffers_dropped));
}
#endif

static int mhb_i2s_send_config_req(struct mhb_i2s_audio *i2s, struct mhb_i2s_config *config)
{
    struct mhb_hdr hdr;

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_I2S;
    hdr.type = MHB_TYPE_I2S_CONFIG_REQ;

    return device_mhb_send(i2s->mhb_dev, &hdr, (uint8_t *)config,
                   sizeof(struct mhb_i2s_config), 0);

}

static void  mhb_i2s_config_worker(void *data)
{
    struct mhb_i2s_audio *i2s = &mhb_i2s;
    int slave_status = *((uint32_t *)data);
    int result;

    lldbg("status=%d, old_state=%d\n", slave_status, i2s->slave_state);

    sem_wait(&i2s->lock);

    if (i2s->slave_state == slave_status)
        goto out;

    if (slave_status == MHB_PM_STATUS_PEER_CONNECTED &&
                          ((i2s->state & MHB_I2S_STATE_PENDING) ||
                          (i2s->state == MHB_I2S_AUDIO_CONFIG) ||
                          (i2s->state == MHB_I2S_AUDIO_START_STREAM)))
    {
        i2s->state |= MHB_I2S_STATE_PENDING;
        if (i2s->audio_dev) {
            lldbg("Configure codec from pending state\n");
            result = device_audio_set_config(i2s->audio_dev, &i2s->set_codec_pcm_cfg,
                              &i2s->set_codec_dai_cfg);
            if (result) {
                lldbg("ERROR: failed configuring audio codec\n");
                goto out;
            }
        }
        mhb_i2s_send_config_req(i2s, &i2s->config);
    }

out:
    i2s->slave_state = slave_status;
    sem_post(&i2s->lock);
}

static int mhb_i2s_audio_slave_status_callback(struct device *dev,
                                                   uint32_t slave_status)
{
    work_queue(HPWORK, &mhb_i2s_work, mhb_i2s_config_worker, &slave_status, 0);

    return 0;
}

static int _mhb_i2s_audio_apbe_on(struct mhb_i2s_audio *i2s)
{
    int ret;

    ret = device_slave_pwrctrl_send_slave_state(i2s->slave_pwr_ctrl,
        SLAVE_STATE_ENABLED);
    if (ret) {
        lldbg("ERROR: Failed to send state\n");
        return ret;
    }

    return 0;
}

static int _mhb_i2s_audio_apbe_off(struct mhb_i2s_audio *i2s)
{
    int ret;

    if (!i2s->slave_pwr_ctrl) {
        return -ENODEV;
    }

    ret = device_slave_pwrctrl_send_slave_state(i2s->slave_pwr_ctrl,
        SLAVE_STATE_DISABLED);
    if (ret) {
        lldbg("ERROR: Failed to send state\n");
    }

    return 0;
}

static int mhb_i2s_send_control_req(struct mhb_i2s_audio *i2s,  uint8_t command)
{
    struct mhb_hdr hdr;
    struct mhb_i2s_control_req req;

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_I2S;
    hdr.type = MHB_TYPE_I2S_CONTROL_REQ;
    req.command = command;

    return device_mhb_send(i2s->mhb_dev, &hdr,
                           (uint8_t *)&req, sizeof(req), 0);
}

static int mhb_i2s_convert_gb_rate(uint32_t gb_sample_rate, uint32_t *mhb_rate)
{
    int ret = 0;

    switch(gb_sample_rate) {
        case DEVICE_I2S_PCM_RATE_5512:
            *mhb_rate = 5512;
            break;
        case DEVICE_I2S_PCM_RATE_8000:
            *mhb_rate = 8000;
            break;
        case DEVICE_I2S_PCM_RATE_11025:
            *mhb_rate = 11025;
            break;
        case DEVICE_I2S_PCM_RATE_16000:
            *mhb_rate = 16000;
            break;
        case DEVICE_I2S_PCM_RATE_22050:
            *mhb_rate = 22050;
            break;
        case DEVICE_I2S_PCM_RATE_32000:
            *mhb_rate = 32000;
            break;
        case DEVICE_I2S_PCM_RATE_44100:
            *mhb_rate = 44100;
            break;
        case DEVICE_I2S_PCM_RATE_48000:
            *mhb_rate = 48000;
            break;
        case DEVICE_I2S_PCM_RATE_96000:
            *mhb_rate = 96000;
            break;
        case DEVICE_I2S_PCM_RATE_192000:
            *mhb_rate = 192000;
            break;
        default:
            ret = -EINVAL;
    }

    return ret;
}

static int mhb_i2s_convert_gb_format(uint8_t gb_format, uint8_t *mhb_sample_size)
{
    int ret = 0;

    switch (gb_format) {
        case DEVICE_I2S_PCM_FMT_8:
            *mhb_sample_size = 8;
            break;
        case DEVICE_I2S_PCM_FMT_16:
            *mhb_sample_size = 16;
            break;
        case DEVICE_I2S_PCM_FMT_24:
            *mhb_sample_size = 24;
            break;
        case DEVICE_I2S_PCM_FMT_32:
            *mhb_sample_size = 32;
            break;
        case DEVICE_I2S_PCM_FMT_64:
            *mhb_sample_size = 64;
            break;
        default:
            ret = -EINVAL;
    }

    return ret;
}

static int mhb_i2s_convert_gb_edge(uint8_t gb_edge, uint8_t *mhb_edge)
{
    int ret = 0;

    switch (gb_edge) {
        case DEVICE_I2S_EDGE_RISING:
            *mhb_edge = MHB_I2S_EDGE_RISING;
            break;
        case DEVICE_I2S_EDGE_FALLING:
            *mhb_edge = MHB_I2S_EDGE_FALLING;
            break;
        default:
            ret = EINVAL;
    }

    return ret;
}

static int mhb_i2s_convert_gb_protocol(uint8_t gb_protocol, uint8_t *mhb_protocol)
{
    int ret = 0;

    switch (gb_protocol) {
        case DEVICE_I2S_PROTOCOL_I2S:
            *mhb_protocol = MHB_I2S_PROTOCOL_I2S;
            break;
        case DEVICE_I2S_PROTOCOL_LR_STEREO:
            *mhb_protocol = MHB_I2S_PROTOCOL_LR_STEREO;
            break;
        case DEVICE_I2S_PROTOCOL_PCM:
            *mhb_protocol = MHB_I2S_PROTOCOL_PCM;
            break;
        default:
            ret = EINVAL;
    }

    return ret;
}

static int mhb_i2s_convert_gb_clk_polarity(uint8_t gb_polarity, uint8_t *mhb_polarity)
{
    int ret = 0;

    switch (gb_polarity) {
        case DEVICE_I2S_POLARITY_NORMAL:
            *mhb_polarity = MHB_I2S_WCLK_POLARITY_NORMAL;
            break;
        case DEVICE_I2S_POLARITY_REVERSED:
            *mhb_polarity = MHB_I2S_WCLK_POLARITY_REVERSE;
            break;
        default:
            ret = EINVAL;
    }

    return ret;
}

static int mhb_i2s_convert_gb_clk_role(uint8_t gb_clk_role, uint8_t *mhb_clk_role)
{
    int ret = 0;

    switch (gb_clk_role) {
        case DEVICE_I2S_ROLE_SLAVE:
            *mhb_clk_role = MHB_I2S_ROLE_SLAVE;
            break;
        case DEVICE_I2S_ROLE_MASTER:
            *mhb_clk_role = MHB_I2S_ROLE_MASTER;
            break;
        default:
            ret = EINVAL;
    }

    return ret;
}

static int mhb_i2s_convert_config(struct device_i2s_dai *i2s_dai,
                           struct device_i2s_pcm  *i2s_pcm, uint8_t clk_role,
                           struct mhb_i2s_config *i2s_cfg)
{
   int result;

   i2s_cfg->num_channels = i2s_pcm->channels;

   result = mhb_i2s_convert_gb_rate(i2s_pcm->rate, &i2s_cfg->sample_rate);
   if (result) {
        lldbg("ERROR: invalid gb sample rate\n");
        return result;
    }
   result = mhb_i2s_convert_gb_format(i2s_pcm->format, &i2s_cfg->sample_size);
   if (result) {
        lldbg("ERROR: invalid gb pcm format\n");
        return result;
    }
   result = mhb_i2s_convert_gb_edge(i2s_dai->wclk_change_edge, &i2s_cfg->wclk_edge);
   if (result) {
        lldbg("ERROR: invalid gb ws edge\n");
        return result;
    }
   result = mhb_i2s_convert_gb_edge(i2s_dai->data_rx_edge, &i2s_cfg->rx_edge);
   if (result) {
        lldbg("ERROR: invalid gb rx edge\n");
        return result;
    }
   result = mhb_i2s_convert_gb_edge(i2s_dai->data_tx_edge, &i2s_cfg->tx_edge);
   if (result) {
        lldbg("ERROR: invalid gb tx edge\n");
        return result;
    }
    result = mhb_i2s_convert_gb_protocol(i2s_dai->protocol, &i2s_cfg->protocol);
    if (result) {
        lldbg("ERROR: invalid gb protocol\n");
        return result;
    }
    result = mhb_i2s_convert_gb_clk_polarity(i2s_dai->wclk_polarity,
                              &i2s_cfg->wclk_polarity);
    if (result) {
        lldbg("ERROR: invalid gb protocol\n");
        return result;
    }

    result = mhb_i2s_convert_gb_clk_role(clk_role,
                              &i2s_cfg->clk_role);
    if (result) {
        lldbg("ERROR: invalid gb clock rule\n");
        return result;
    }

    return 0;
}

static int mhb_i2s_op_set_config(struct device *dev, uint8_t clk_role,
                           struct device_i2s_pcm *i2s_pcm,
                           struct device_i2s_dai  *i2s_dai)
{
    struct mhb_i2s_config i2s_cfg;
    struct mhb_response *rsp;
    struct mhb_i2s_audio *i2s = device_get_private(dev);
    int result;

    if (!i2s) {
        lldbg("error no dev\n");
        return -ENODEV;
    }

    sem_wait(&i2s->lock);
    if (i2s->i2s_status) {
        i2s->state = MHB_I2S_AUDIO_CONFIG;
        llvdbg("bridge already cfg'ed and streaming return\n");
        result = 0;
        goto err;
    }

    /* check if config is supported */
    if (!ONE_BIT_IS_SET(i2s_dai->protocol) ||
          !ONE_BIT_IS_SET(i2s_dai->wclk_polarity) ||
          !ONE_BIT_IS_SET(i2s_dai->wclk_change_edge) ||
          !ONE_BIT_IS_SET(i2s_dai->data_rx_edge) ||
          !ONE_BIT_IS_SET(i2s_dai->data_tx_edge) ||
          !ONE_BIT_IS_SET(i2s_pcm->rate) ||
          !ONE_BIT_IS_SET(i2s_pcm->format)) {
        lldbg("%s() i2s configuration more than 1 bit set\n", __func__);
        result = -ERANGE;
        goto err;
    }

    if (i2s_pcm->channels < 0 || i2s_pcm->channels > 2) {
         lldbg("%s() i2s channels configuration invalid\n", __func__);
         result = -ERANGE;
         goto err;
    }

     /* Check that bit field settings match our capabilities */
    if (((i2s_dai->protocol & i2s->common_dai_cfg.protocol) !=
                                              i2s_dai->protocol)||
         ((i2s_dai->wclk_polarity & i2s->common_dai_cfg.wclk_polarity) !=
                                              i2s_dai->wclk_polarity) ||
         ((i2s_dai->wclk_change_edge & i2s->common_dai_cfg.wclk_change_edge) !=
                                              i2s_dai->wclk_change_edge) ||
         ((i2s_dai->data_rx_edge & i2s->common_dai_cfg.data_rx_edge) !=
                                              i2s_dai->data_rx_edge) ||
         ((i2s_dai->data_tx_edge & i2s->common_dai_cfg.data_tx_edge) !=
                                              i2s_dai->data_tx_edge ) ||
         ((i2s_pcm->rate & i2s->common_pcm_cfg.rate) !=
                                              i2s_pcm->rate) ||
         ((i2s_pcm->format & i2s->common_pcm_cfg.format) !=
                                             i2s_pcm->format)) {
         lldbg("%s() failed unsupported i2s config", __func__);
         result = -EINVAL;
         goto err;
    }

    result = mhb_i2s_convert_config(i2s_dai, i2s_pcm, clk_role, &i2s_cfg);
    if (result) {
        lldbg("ERROR: send config req failed: %d\n", result);
        goto err;
    }

    /* set pcm and dai config to audio codec */
    if (i2s->audio_dev) {
        memset(&i2s->set_codec_pcm_cfg, 0, sizeof(&i2s->set_codec_pcm_cfg));
        memset(&i2s->set_codec_dai_cfg, 0, sizeof(&i2s->set_codec_dai_cfg));
        i2s->set_codec_pcm_cfg.rate = i2s_pcm->rate;
        i2s->set_codec_pcm_cfg.format = i2s_pcm->format;
        i2s->set_codec_pcm_cfg.channels = i2s_pcm->channels;
        i2s->set_codec_dai_cfg.dai_type = DEVICE_AUDIO_DAI_I2S_TYPE;
        i2s->set_codec_dai_cfg.config.i2s_dai.wclk_polarity = i2s_dai->wclk_polarity;
        i2s->set_codec_dai_cfg.config.i2s_dai.wclk_change_edge = i2s_dai->wclk_change_edge;
        i2s->set_codec_dai_cfg.config.i2s_dai.data_rx_edge = i2s_dai->data_rx_edge;
        i2s->set_codec_dai_cfg.config.i2s_dai.data_tx_edge = i2s_dai->data_tx_edge;
        i2s->set_codec_dai_cfg.config.i2s_dai.protocol = i2s_dai->protocol;
        result = device_audio_set_config(i2s->audio_dev, &i2s->set_codec_pcm_cfg,
                                  &i2s->set_codec_dai_cfg);
        if (result) {
            lldbg("ERROR: failed configuring audio codec\n");
            goto err;
        }
    }
    if (i2s->slave_state == MHB_PM_STATUS_PEER_CONNECTED &&
                                        i2s->state == MHB_I2S_AUDIO_START) {
        result =  mhb_i2s_send_config_req(i2s, &i2s_cfg);
        if (result) {
            lldbg("ERROR: send config req failed: %d\n", result);
            goto err;
        }
        llvdbg("waiting for config rsp ..\n");
        result = _mhb_i2s_wait_for_response(i2s);
        if (result) {
            lldbg("ERROR: send config req timed out: %d\n", result);
            goto err;
        } else {
            rsp = &i2s->last_rsp;

            if(rsp->hdr.type == MHB_TYPE_I2S_CONFIG_RSP &&
                    rsp->hdr.result == MHB_RESULT_SUCCESS) {
                llvdbg("mhb i2s: configured successfully\n");
            }
        }
        i2s->state = MHB_I2S_AUDIO_CONFIG;
    } else {
       /* else wait for slave connection, config req will
        * be sent when slave is connected.
        */
        llvdbg("mhb i2s: slave connection pending");
        i2s->state = (MHB_I2S_AUDIO_CONFIG | MHB_I2S_STATE_PENDING);
    }
    /* save config */
    memcpy(&i2s->config, &i2s_cfg, sizeof(struct mhb_i2s_config));

    sem_post(&i2s->lock);
    return 0;
err:
    i2s->state = MHB_I2S_AUDIO_OFF;
    sem_post(&i2s->lock);
    return result;
}

static int mhb_i2s_op_get_capabilties(struct device *dev, uint8_t role,
                           struct device_i2s_pcm *i2s_pcm,
                           struct device_i2s_dai  *i2s_dai)
{
    struct mhb_i2s_audio *i2s = device_get_private(dev);

    if (!i2s)
        return -ENODEV;

    memcpy(i2s_dai, &i2s->common_dai_cfg, sizeof(struct device_i2s_dai));
    memcpy(i2s_pcm, &i2s->common_pcm_cfg, sizeof(struct device_i2s_pcm));

    return 0;
}

static int mhb_i2s_op_start_rx_stream(struct device *dev)
{
    struct mhb_i2s_audio *i2s = device_get_private(dev);
    struct mhb_response *rsp;
    int result = 0;

    if (!i2s)
        return -ENODEV;

    sem_wait(&i2s->lock);
    if (i2s->i2s_status & MHB_I2S_RX_ACTIVE)
        goto out;
    /* Its possible for phone to request start playback stream again
     * before sending stop command, use the saved config and resend config
     * request to the bridge in this case to stay consistent with the state
     * machine on the bridge side.
     */
    if (i2s->slave_state == MHB_PM_STATUS_PEER_CONNECTED
               && i2s->state == MHB_I2S_AUDIO_STOP_STREAM) {
        result =  mhb_i2s_send_config_req(i2s, &i2s->config);
        if (result) {
            lldbg("ERROR: send config req failed: %d\n", result);
            goto out;
        }
        llvdbg("waiting for config rsp ..\n");
        result = _mhb_i2s_wait_for_response(i2s);
        if (result) {
            lldbg("ERROR: send config req timed out: %d\n", result);
            goto out;
        } else {
            rsp = &i2s->last_rsp;

            if(rsp->hdr.type == MHB_TYPE_I2S_CONFIG_RSP &&
                    rsp->hdr.result == MHB_RESULT_SUCCESS) {
                i2s->state = MHB_I2S_AUDIO_CONFIG;
                llvdbg("mhb i2s: reconfigured successfully\n");
            }
        }
    }

    if (i2s->slave_state == MHB_PM_STATUS_PEER_CONNECTED  &&
                                (i2s->state == MHB_I2S_AUDIO_CONFIG ||
                                i2s->state == MHB_I2S_AUDIO_START_STREAM)) {
        result = mhb_i2s_send_control_req(i2s, MHB_I2S_COMMAND_RX_START);
        if (result) {
            lldbg("ERROR: failed to send control req to start rx\n");
            goto out;
        }

        result = _mhb_i2s_wait_for_response(i2s);
         if (result) {
            lldbg("ERROR: send control req timed out: %d\n", result);
            goto out;
        }
        rsp = &i2s->last_rsp;
        if (rsp->hdr.type == MHB_TYPE_I2S_CONTROL_RSP &&
                            rsp->hdr.result == MHB_RESULT_SUCCESS) {
            /* trigger audio codec to start receiving */
            if (i2s->audio_dev)
                device_audio_rx_dai_start(i2s->audio_dev);
        } else {
           result = -EIO;
           goto out;
        }
        i2s->state = MHB_I2S_AUDIO_START_STREAM;
    } else
        i2s->state = (MHB_I2S_AUDIO_START_STREAM | MHB_I2S_STATE_PENDING);

    i2s->i2s_status |= MHB_I2S_RX_ACTIVE;

out:
    sem_post(&i2s->lock);
    return result;
}

static int mhb_i2s_op_start_tx_stream(struct device *dev)
{
    struct mhb_i2s_audio *i2s = device_get_private(dev);
    struct mhb_response *rsp;
    int result = 0;

    if (!i2s)
        return -ENODEV;

    sem_wait(&i2s->lock);
    if (i2s->i2s_status & MHB_I2S_TX_ACTIVE)
        goto out;

    if ((i2s->slave_state == MHB_PM_STATUS_PEER_CONNECTED) &&
                                (i2s->state == MHB_I2S_AUDIO_CONFIG ||
                                i2s->state == MHB_I2S_AUDIO_START_STREAM)) {
        result = mhb_i2s_send_control_req(i2s, MHB_I2S_COMMAND_TX_START);
        if (result) {
            lldbg("ERROR: failed to send control req to start rx\n");
            goto out;
        }
        result = _mhb_i2s_wait_for_response(i2s);
         if (result) {
            lldbg("ERROR: send control req timed out: %d\n", result);
             goto out;
        }
        rsp = &i2s->last_rsp;
        if (rsp->hdr.type == MHB_TYPE_I2S_CONTROL_RSP &&
                            rsp->hdr.result == MHB_RESULT_SUCCESS) {

            /* trigger audio codec to start transmitting */
            if (i2s->audio_dev)
                device_audio_tx_dai_start(i2s->audio_dev);
        } else {
           lldbg("ERROR: send control req rsp failed %d\n", rsp->hdr.result);
           result = -EIO;
           goto out;
        }
        i2s->state = MHB_I2S_AUDIO_START_STREAM;
    } else
        i2s->state = (MHB_I2S_AUDIO_START_STREAM | MHB_I2S_STATE_PENDING);

    i2s->i2s_status |= MHB_I2S_TX_ACTIVE;

out:
    sem_post(&i2s->lock);
    return 0;
}

static int mhb_i2s_op_stop_tx_stream(struct device *dev)
{
    struct mhb_i2s_audio *i2s = device_get_private(dev);
    struct mhb_response *rsp;
    int result = 0;

    if (!i2s)
        return -ENODEV;

    sem_wait(&i2s->lock);
    if (!(i2s->i2s_status & MHB_I2S_TX_ACTIVE))
        goto out;

    if (i2s->state != MHB_I2S_AUDIO_START_STREAM) {
        i2s->i2s_status &= ~MHB_I2S_TX_ACTIVE;
        goto out;
    }
   /* trigger audio codec to stop transmitting */
   if (i2s->audio_dev)
        device_audio_tx_dai_stop(i2s->audio_dev);

    result = mhb_i2s_send_control_req(i2s, MHB_I2S_COMMAND_TX_STOP);
    if (result) {
        lldbg("ERROR: failed to send control req to stop tx\n");
        goto out;
    }
    result = _mhb_i2s_wait_for_response(i2s);
     if (result) {
        lldbg("ERROR: send control req timed out: %d\n", result);
        goto out;
    }
    rsp = &i2s->last_rsp;
    if (rsp->hdr.type == MHB_TYPE_I2S_CONTROL_RSP &&
                        rsp->hdr.result == MHB_RESULT_SUCCESS) {
        i2s->i2s_status &= ~MHB_I2S_TX_ACTIVE;
    }

out:
    i2s->state = MHB_I2S_AUDIO_STOP_STREAM;
    sem_post(&i2s->lock);
    return result;
}

static int mhb_i2s_op_stop_rx_stream(struct device *dev)
{
    struct mhb_i2s_audio *i2s = device_get_private(dev);
    struct mhb_response *rsp;
    int result = 0;

    if (!i2s)
        return -ENODEV;

    sem_wait(&i2s->lock);
    if (!(i2s->i2s_status & MHB_I2S_RX_ACTIVE))
        goto out;

    if (i2s->state != MHB_I2S_AUDIO_START_STREAM) {
        i2s->i2s_status &= ~MHB_I2S_RX_ACTIVE;
        goto out;
    }

   /* trigger audio codec to stop receiving */
   if (i2s->audio_dev)
        device_audio_rx_dai_stop(i2s->audio_dev);

    result = mhb_i2s_send_control_req(i2s, MHB_I2S_COMMAND_RX_STOP);
    if (result) {
        lldbg("ERROR: failed to send control req to stop rx\n");
        goto out;
    }
    result = _mhb_i2s_wait_for_response(i2s);
     if (result) {
        lldbg("ERROR: send control req timed out: %d\n", result);
        goto out;
    }
    rsp = &i2s->last_rsp;
    if (rsp->hdr.type == MHB_TYPE_I2S_CONTROL_RSP &&
                        rsp->hdr.result == MHB_RESULT_SUCCESS) {
        i2s->i2s_status &= ~MHB_I2S_RX_ACTIVE;
    }

out:
    i2s->state = MHB_I2S_AUDIO_STOP_STREAM;
    sem_post(&i2s->lock);
    return 0;
}

static int mhb_i2s_op_start_rx(struct device *dev)
{
    struct mhb_i2s_audio *i2s = device_get_private(dev);
    int result = 0;

    if (!i2s)
        return -ENODEV;

    sem_wait(&i2s->lock);

    if (i2s->state == MHB_I2S_AUDIO_START)
        goto out;

    i2s->state = MHB_I2S_AUDIO_START;

    result = _mhb_i2s_audio_apbe_on(i2s);
    if (result)
        lldbg("ERROR: failed start mhb i2s\n");

out:
    sem_post(&i2s->lock);
    return result;
}

static int mhb_i2s_op_start_tx(struct device *dev)
{
    struct mhb_i2s_audio *i2s = device_get_private(dev);
    int result = 0;

    if (!i2s)
        return -ENODEV;

    sem_wait(&i2s->lock);

    if (i2s->state == MHB_I2S_AUDIO_START)
        goto out;

    i2s->state = MHB_I2S_AUDIO_START;

    result = _mhb_i2s_audio_apbe_on(i2s);
    if (result)
        lldbg("ERROR: failed start mhb i2s\n");
out:
    sem_post(&i2s->lock);
    return result;
}

static int mhb_i2s_op_stop_rx(struct device *dev)
{
    struct mhb_i2s_audio *i2s = device_get_private(dev);

    if (!i2s)
        return -ENODEV;

    sem_wait(&i2s->lock);

    i2s->state = MHB_I2S_AUDIO_OFF;
    i2s->i2s_status &= ~MHB_I2S_RX_ACTIVE;
    _mhb_i2s_audio_apbe_off(i2s);

    sem_post(&i2s->lock);

    return 0;
}

static int mhb_i2s_op_stop_tx(struct device *dev)
{
    struct mhb_i2s_audio *i2s = device_get_private(dev);

    if (!i2s)
        return -ENODEV;

    sem_wait(&i2s->lock);

    i2s->state = MHB_I2S_AUDIO_OFF;
    i2s->i2s_status &= ~MHB_I2S_TX_ACTIVE;
    _mhb_i2s_audio_apbe_off(i2s);

    sem_post(&i2s->lock);

    return 0;
}

static int _mhb_i2s_send_notification(struct mhb_i2s_audio *i2s,
    enum device_i2s_event event)
{

    if (!i2s->callback) {
        lldbg("ERROR: no callback.\n");
        return -EINVAL;
    }

    i2s->callback(i2s->i2s_dev, event);

    return 0;
}

static int _mhb_i2s_handle_msg(struct device *dev,
    struct mhb_hdr *hdr, uint8_t *payload, size_t payload_length)
{
    struct mhb_i2s_audio *i2s = &mhb_i2s;
    int state;

    if (!i2s) {
        return -ENODEV;
    }
    sem_wait(&i2s->lock);
    if (i2s->state & MHB_I2S_STATE_PENDING) {
        state = i2s->state & ~MHB_I2S_STATE_PENDING;
        /* we are in a state pending for the requests to be sent on slave connection,
         * check the current state and send config and start requests, on rsp failure
         * notify AP with error.
         */
        switch (state) {
            case MHB_I2S_AUDIO_CONFIG:
                if (hdr->result != MHB_RESULT_SUCCESS) {
                    i2s->state = MHB_I2S_AUDIO_OFF;
                    _mhb_i2s_send_notification(i2s, DEVICE_I2S_EVENT_UNSPECIFIED);
                    break;
                }
                if (hdr->type == MHB_TYPE_I2S_CONFIG_RSP)
                    i2s->state &= ~MHB_I2S_STATE_PENDING;
                else {
                    _mhb_i2s_send_notification(i2s, DEVICE_I2S_EVENT_UNSPECIFIED);
                    i2s->state = MHB_I2S_AUDIO_OFF;
                }
                break;
            case MHB_I2S_AUDIO_START_STREAM:
                if (hdr->result != MHB_RESULT_SUCCESS) {
                    i2s->state = MHB_I2S_AUDIO_OFF;
                    i2s->i2s_status = 0;
                    _mhb_i2s_send_notification(i2s, DEVICE_I2S_EVENT_UNSPECIFIED);
                    break;
                }
                if (hdr->type == MHB_TYPE_I2S_CONFIG_RSP) {
                    if (i2s->i2s_status & MHB_I2S_RX_ACTIVE) {
                        mhb_i2s_send_control_req(i2s, MHB_I2S_COMMAND_RX_START);
                        /* trigger audio codec to start receiving */
                        if (i2s->audio_dev)
                             device_audio_rx_dai_start(i2s->audio_dev);

                    }
                    if (i2s->i2s_status & MHB_I2S_TX_ACTIVE) {
                        mhb_i2s_send_control_req(i2s, MHB_I2S_COMMAND_TX_START);
                        /* trigger audio codec to start transmitting */
                        if (i2s->audio_dev)
                             device_audio_tx_dai_start(i2s->audio_dev);
                    }
                } else if (hdr->type == MHB_TYPE_I2S_CONTROL_RSP)
                    i2s->state &= ~MHB_I2S_STATE_PENDING;
                break;
            default:
               break;
        }
         sem_post(&i2s->lock);
         return 0;
    }
    /* not in pending state treat responses as synchronous messages,
     * on receiving response signal waiting thread
     */

    if (hdr->type != MHB_TYPE_I2S_STATUS_NOT) {
        memcpy(&i2s->last_rsp.hdr, hdr, sizeof(struct mhb_hdr));
        if (hdr->type == MHB_TYPE_I2S_STATUS_RSP) {
#ifdef MHB_I2S_LOG_TUNNEL_STATS
            mhb_print_i2s_status("APBA Info:", (struct i2s_tunnel_info_s *)&payload[0]);
            mhb_print_i2s_status("APBE Info:",
                      (struct i2s_tunnel_info_s *)&payload[sizeof(struct i2s_tunnel_info_s)]);
#endif
        }

    } else
        _mhb_i2s_send_notification(i2s, DEVICE_I2S_EVENT_UNSPECIFIED);

    _mhb_i2s_signal_response(i2s);
    sem_post(&i2s->lock);

    return 0;
}

static int mhb_i2s_op_register_cb(struct device *dev, device_i2s_notification_callback cb)
{
    struct mhb_i2s_audio *i2s = device_get_private(dev);

    if (!i2s)
        return -ENODEV;

    i2s->callback = cb;

    return 0;
}

static int mhb_i2s_op_unregister_cb(struct device *dev)
{
    struct mhb_i2s_audio *i2s = device_get_private(dev);

    if (!i2s)
        return -ENODEV;

    i2s->callback = NULL;

    return 0;
}

static int mhb_i2s_dev_open(struct device *dev)
{
    struct mhb_i2s_audio *i2s = device_get_private(dev);
    int result;
    struct device_aud_pcm_config aud_dev_pcm;
    struct device_aud_dai_config aud_dev_dai;

    if (!i2s) {
        result = -ENODEV;
        goto err;
    }

    if (i2s->mhb_dev) {
        lldbg("ERROR: already opened.\n");
        result = -EBUSY;
        goto err;
    }

    i2s->mhb_dev = device_open(DEVICE_TYPE_MHB, MHB_ADDR_I2S);
    if (!i2s->mhb_dev) {
        lldbg("ERROR: failed to open MHB device.\n");
        result = -ENOENT;
        goto err;
    }

    device_mhb_register_receiver(i2s->mhb_dev, MHB_ADDR_I2S,
                                 _mhb_i2s_handle_msg);


    i2s->slave_pwr_ctrl = device_open(DEVICE_TYPE_SLAVE_PWRCTRL_HW,
            MHB_ADDR_I2S);
    if (!i2s->slave_pwr_ctrl) {
        lldbg("ERROR: Failed to open slave pwr ctrl device\n");
        result = -ENODEV;
        goto err;
    }
    result = device_slave_pwrctrl_register_status_callback(i2s->slave_pwr_ctrl,
                             mhb_i2s_audio_slave_status_callback);
    if (result) {
        lldbg("ERROR: Failed to register slave pwr ctrl cb\n");
        goto err;
    }

    memcpy(&i2s->common_pcm_cfg, &tsb_i2s_pcm, sizeof(struct device_i2s_pcm));
    memcpy(&i2s->common_dai_cfg, &tsb_i2s_dai, sizeof(struct device_i2s_dai));
    i2s->audio_dev = device_open(DEVICE_TYPE_MUC_AUD_HW, DEVICE_AUDIO_MHB_ID);
    if (i2s->audio_dev) {
        /* get supported pcm config from audio device */
        aud_dev_dai.dai_type = DEVICE_AUDIO_DAI_I2S_TYPE;
        result = device_audio_get_config(i2s->audio_dev, &aud_dev_pcm,
                                          &aud_dev_dai);
        if (!result) {
            /* update common supported pcm and dai configuration */
            i2s->common_pcm_cfg.rate &= aud_dev_pcm.rate;
            i2s->common_pcm_cfg.format &= aud_dev_pcm.format;
            i2s->common_pcm_cfg.channels = MIN(aud_dev_pcm.channels, i2s->common_pcm_cfg.channels);
            i2s->common_dai_cfg.data_rx_edge &= aud_dev_dai.config.i2s_dai.data_rx_edge;
            i2s->common_dai_cfg.data_tx_edge &= aud_dev_dai.config.i2s_dai.data_tx_edge;
            i2s->common_dai_cfg.protocol &= aud_dev_dai.config.i2s_dai.protocol;
            i2s->common_dai_cfg.wclk_polarity &= aud_dev_dai.config.i2s_dai.wclk_polarity;
            i2s->common_dai_cfg.wclk_change_edge &= aud_dev_dai.config.i2s_dai.wclk_change_edge;
        }
    } else {
        lldbg("WARN: failed to open audio device.\n");
    }

    result = 0;

err:
    return result;
}

static int mhb_i2s_dev_probe(struct device *dev)
{
    struct mhb_i2s_audio *i2s = &mhb_i2s;

    device_set_private(dev, i2s);
    i2s->i2s_dev = dev;

    pthread_mutex_init(&i2s->mutex, NULL);
    sem_init(&i2s->lock, 0, 1);
    pthread_cond_init(&i2s->cond, NULL);

    return 0;
}

static void mhb_i2s_dev_close(struct device *dev) {
    struct mhb_i2s_audio *i2s = device_get_private(dev);

    if (!i2s) {
        return;
    }

    if (i2s->mhb_dev) {
        device_mhb_unregister_receiver(i2s->mhb_dev, MHB_ADDR_I2S,
                                       _mhb_i2s_handle_msg);
        device_close(i2s->mhb_dev);
        i2s->mhb_dev = NULL;
    }
    if (i2s->audio_dev) {
        device_close(i2s->audio_dev);
        i2s->audio_dev = NULL;
    }
    if(i2s->slave_pwr_ctrl) {
        device_slave_pwrctrl_unregister_status_callback(
                  i2s->slave_pwr_ctrl, mhb_i2s_audio_slave_status_callback);
        device_close(i2s->slave_pwr_ctrl);
        i2s->slave_pwr_ctrl = NULL;
        i2s->slave_state = MHB_PM_STATUS_PEER_NONE;
    }
    sem_destroy(&i2s->lock);
    pthread_mutex_destroy(&i2s->mutex);
    pthread_cond_destroy(&i2s->cond);
}

static void mhb_i2s_dev_remove(struct device *dev) {
    struct mhb_i2s_audio *i2s = device_get_private(dev);

    if (i2s) {
        device_set_private(dev, NULL);
        memset(i2s, 0, sizeof(*i2s));
    }
}

/* I2S PHY protocol driver and ops */
static struct device_i2s_type_ops mhb_i2s_type_ops = {
    .get_caps                     = mhb_i2s_op_get_capabilties,
    .set_config                   = mhb_i2s_op_set_config,
    .start_transmitter_port       = mhb_i2s_op_start_tx_stream,
    .start_receiver_port          = mhb_i2s_op_start_rx_stream,
    .stop_receiver_port           = mhb_i2s_op_stop_rx_stream,
    .stop_transmitter_port        = mhb_i2s_op_stop_tx_stream,
    .register_callback            = mhb_i2s_op_register_cb,
    .unregister_callback          = mhb_i2s_op_unregister_cb,
    .start_transmitter            = mhb_i2s_op_start_tx,
    .start_receiver               = mhb_i2s_op_start_rx,
    .stop_transmitter             = mhb_i2s_op_stop_tx,
    .stop_receiver                = mhb_i2s_op_stop_rx,
};

static struct device_driver_ops mhb_i2s_driver_ops = {
    .open       = mhb_i2s_dev_open,
    .close      = mhb_i2s_dev_close,
    .probe      = mhb_i2s_dev_probe,
    .remove     = mhb_i2s_dev_remove,
    .type_ops   = &mhb_i2s_type_ops,
};

struct device_driver mhb_i2s_driver = {
    .type   = DEVICE_TYPE_I2S_HW,
    .name   = "mhb_i2s_audio",
    .desc   = "MHB I2S Driver",
    .ops    = &mhb_i2s_driver_ops,
};
