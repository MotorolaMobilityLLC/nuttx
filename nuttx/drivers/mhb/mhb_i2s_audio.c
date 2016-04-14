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
#include <nuttx/mhb/device_mhb.h>
#include <nuttx/mhb/mhb_protocol.h>

#define MHB_I2S_OP_TIMEOUT_MS     800

#define MHB_I2S_RX_ACTIVE    BIT(0)
#define MHB_I2S_TX_ACTIVE    BIT(1)

struct mhb_response {
    struct mhb_hdr hdr;
    uint8_t *payload;
    size_t payload_length;
};

struct mhb_i2s_audio {
    struct device *i2s_dev;
    struct device *audio_dev;
    struct device *mhb_dev;
    struct device_i2s_pcm common_pcm_cfg;
    struct device_i2s_dai common_dai_cfg;
    uint8_t i2s_status;
    bool is_configured;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    struct mhb_response *last_rsp;
    device_i2s_notification_callback callback;
};


static struct mhb_i2s_audio mhb_i2s;

/*
 * I2S configuration supported by TSB, as we dont expect this to change
 * config is harcoded here, in future if we need we can query APBE to
 * to get supported configurations
 */
static const struct device_i2s_pcm tsb_i2s_pcm = {
    .rate = (DEVICE_I2S_PCM_RATE_16000 | DEVICE_I2S_PCM_RATE_96000 |
                            DEVICE_I2S_PCM_RATE_48000 |
                            DEVICE_I2S_PCM_RATE_192000),
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

    pthread_mutex_lock(&i2s->mutex);
    ret = pthread_cond_timedwait(&i2s->cond, &i2s->mutex, &expires);
    pthread_mutex_unlock(&i2s->mutex);

    if (ret) {
        /* timeout or other errors */
        lldbg("ERROR: wait error %d\n", ret);
        return -ETIME;
    }

    return 0;
}

static int mhb_i2s_send_config_req(struct mhb_i2s_audio *i2s, struct mhb_i2s_config *config)
{
    struct mhb_hdr hdr;

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_I2S;
    hdr.type = MHB_TYPE_I2S_CONFIG_REQ;

    return device_mhb_send(i2s->mhb_dev, &hdr, (uint8_t *)config,
                   sizeof(*config), 0);

}

static int mhb_i2s_send_control_req(struct mhb_i2s_audio *i2s,  uint8_t command)
{
    struct mhb_hdr hdr;
    struct mhb_i2s_control_req req;

    memset(&hdr, 0, sizeof(hdr));

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
        result = -ENODEV;
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

    result =  mhb_i2s_send_config_req(i2s, &i2s_cfg);
    if (result) {
        lldbg("ERROR: send config req failed: %d\n", result);
        goto err;
    }

    result = _mhb_i2s_wait_for_response(i2s);
    if (result) {
        lldbg("ERROR: send config req timeed out: %d\n", result);
        goto err;
    } else {
        rsp = i2s->last_rsp;
        if(rsp->hdr.type == MHB_TYPE_I2S_CONFIG_RSP &&
                    rsp->hdr.result == MHB_RESULT_SUCCESS)
            i2s->is_configured = true;
    }

    /* set pcm and dai config to audio codec */
    if (i2s->audio_dev) {
        struct device_aud_pcm_config pcm_cfg;
        struct device_aud_dai_config dai_cfg;

        memset(&dai_cfg, 0, sizeof(&dai_cfg));
        pcm_cfg.rate = i2s_pcm->rate;
        pcm_cfg.format = i2s_pcm->format;
        pcm_cfg.channels = i2s_pcm->channels;
        dai_cfg.dai_type = DEVICE_AUDIO_DAI_I2S_TYPE;
        dai_cfg.config.i2s_dai.wclk_polarity = i2s_dai->wclk_polarity;
        dai_cfg.config.i2s_dai.wclk_change_edge = i2s_dai->wclk_change_edge;
        dai_cfg.config.i2s_dai.data_rx_edge = i2s_dai->data_rx_edge;
        dai_cfg.config.i2s_dai.data_tx_edge = i2s_dai->data_tx_edge;

        result = device_audio_set_config(i2s->audio_dev, &pcm_cfg, &dai_cfg);
        if (result) {
            lldbg("ERROR: failed configuring audio codec\n");
            goto err;
        }
    }

    return 0;
err:
    i2s->is_configured = false;
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

static int mhb_i2s_op_start_receiver(struct device *dev)
{
    struct mhb_i2s_audio *i2s = device_get_private(dev);
    struct mhb_response *rsp;
    int result;

    if (!i2s)
        return -ENODEV;

    if (!i2s->is_configured)
        return -EINVAL;

    if (i2s->i2s_status & MHB_I2S_RX_ACTIVE)
        return 0;

    result = mhb_i2s_send_control_req(i2s, MHB_I2S_COMMAND_RX_START);
    if (result) {
        lldbg("ERROR: failed to send control req to start rx\n");
        return result;
    }
    result = _mhb_i2s_wait_for_response(i2s);
     if (result) {
        lldbg("ERROR: send control req timed out: %d\n", result);
        return result;
    }
    rsp = i2s->last_rsp;
    if (rsp->hdr.type == MHB_TYPE_I2S_CONTROL_RSP &&
                        rsp->hdr.result == MHB_RESULT_SUCCESS) {
        i2s->i2s_status |= MHB_I2S_RX_ACTIVE;
    }

    return 0;
}

static int mhb_i2s_op_start_transmitter(struct device *dev)
{
    struct mhb_i2s_audio *i2s = device_get_private(dev);
    struct mhb_response *rsp;
    int result;

    if (!i2s)
        return -ENODEV;

    if (!i2s->is_configured)
        return -EINVAL;

    if (i2s->i2s_status & MHB_I2S_TX_ACTIVE)
        return 0;

    result = mhb_i2s_send_control_req(i2s, MHB_I2S_COMMAND_TX_START);
    if (result) {
        lldbg("ERROR: failed to send control req to start rx\n");
        return result;
    }
    result = _mhb_i2s_wait_for_response(i2s);
     if (result) {
        lldbg("ERROR: send control req timed out: %d\n", result);
        return result;
    }
    rsp = i2s->last_rsp;
    if (rsp->hdr.type == MHB_TYPE_I2S_CONTROL_RSP &&
                        rsp->hdr.result == MHB_RESULT_SUCCESS) {
        i2s->i2s_status |= MHB_I2S_TX_ACTIVE;
    }

    return 0;
}

static int mhb_i2s_op_stop_transmitter(struct device *dev)
{
    struct mhb_i2s_audio *i2s = device_get_private(dev);
    struct mhb_response *rsp;
    int result;

    if (!i2s)
        return -ENODEV;

    if (!(i2s->i2s_status & MHB_I2S_TX_ACTIVE))
        return 0;

    result = mhb_i2s_send_control_req(i2s, MHB_I2S_COMMAND_TX_STOP);
    if (result) {
        lldbg("ERROR: failed to send control req to stop tx\n");
        return result;
    }
    result = _mhb_i2s_wait_for_response(i2s);
     if (result) {
        lldbg("ERROR: send control req timed out: %d\n", result);
        return result;
    }
    rsp = i2s->last_rsp;
    if (rsp->hdr.type == MHB_TYPE_I2S_CONTROL_RSP &&
                        rsp->hdr.result == MHB_RESULT_SUCCESS) {
        i2s->i2s_status &= ~MHB_I2S_TX_ACTIVE;
    }

    return 0;
}

static int mhb_i2s_op_stop_receiver(struct device *dev)
{
    struct mhb_i2s_audio *i2s = device_get_private(dev);
    struct mhb_response *rsp;
    int result;

    if (!i2s)
        return -ENODEV;

    if (!(i2s->i2s_status & MHB_I2S_RX_ACTIVE))
        return 0;

    result = mhb_i2s_send_control_req(i2s, MHB_I2S_COMMAND_RX_STOP);
    if (result) {
        lldbg("ERROR: failed to send control req to stop rx\n");
        return result;
    }
    result = _mhb_i2s_wait_for_response(i2s);
     if (result) {
        lldbg("ERROR: send control req timed out: %d\n", result);
        return result;
    }
    rsp = i2s->last_rsp;
    if (rsp->hdr.type == MHB_TYPE_I2S_CONTROL_RSP &&
                        rsp->hdr.result == MHB_RESULT_SUCCESS) {
        i2s->i2s_status &= ~MHB_I2S_RX_ACTIVE;
    }

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

    if (!i2s) {
        return -ENODEV;
    }

    llvdbg("type=%02x, result=%02x\n",
        hdr->type, hdr->result);

    if (hdr->type != MHB_TYPE_I2S_STATUS_NOT) {
        memcpy(&i2s->last_rsp->hdr, hdr, sizeof(*hdr));
        /* TODO: copy payload too, needed for status rsp */
    } else
        _mhb_i2s_send_notification(i2s, DEVICE_I2S_EVENT_UNSPECIFIED);

    _mhb_i2s_signal_response(i2s);

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

    // TODO: Request APBE on.

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
    .start_transmitter_port       = mhb_i2s_op_start_transmitter,
    .start_receiver_port          = mhb_i2s_op_start_receiver,
    .stop_receiver_port           = mhb_i2s_op_stop_receiver,
    .stop_transmitter_port        = mhb_i2s_op_stop_transmitter,
    .register_callback            = mhb_i2s_op_register_cb,
    .unregister_callback          = mhb_i2s_op_unregister_cb,
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
