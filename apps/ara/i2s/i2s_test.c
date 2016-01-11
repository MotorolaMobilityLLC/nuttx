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
 * @brief Low-level TSB I2S device driver test program
 */
/*
 * Just a quick hack pgm to test out i2s & lr_stereo
 * (48KHz sampling, 2 channels per sample, 16-bits per channel).
 * Audio format confirmed by an I2S analyzer.
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <signal.h>

#include <nuttx/device.h>
#include <nuttx/device_i2s.h>
#include <nuttx/ring_buf.h>
#include "i2s_test.h"
#include "gen_pcm.h"

#ifndef SIGKILL
#define SIGKILL     9
#endif

#ifndef SIGTERM
#define SIGTERM     15
#endif

#define DEFAULT_SAMPLE_RATE         48000
#define DEFAULT_AUDIO_FREQUENCY     120
#define MIN_AUDIO_FREQUENCY         1
#define MAX_AUDIO_FREQUENCY         100000
#define DEFAULT_AUDIO_VOLUME        8
#define MIN_AUDIO_VOLUME            0
#define MAX_AUDIO_VOLUME            19

sem_t i2s_test_done_sem;

struct i2s_test_sample {
    uint16_t    left;
    uint16_t    right;
};

/**
 * Hard coded test configuration parameters
 */
struct device_i2s_pcm i2s_test_pcm = {
    DEVICE_I2S_PCM_FMT_16,
    DEVICE_I2S_PCM_RATE_48000,
    2
};

static struct device_i2s_dai test_i2s_dai_m = {
    6144000,
    0,
    DEVICE_I2S_POLARITY_NORMAL,
    0,
    DEVICE_I2S_EDGE_RISING,
    DEVICE_I2S_EDGE_FALLING
};

static struct device_i2s_dai test_i2s_dai_s = {
    6144000,
    0,
    DEVICE_I2S_POLARITY_NORMAL,
    DEVICE_I2S_EDGE_RISING,
    DEVICE_I2S_EDGE_FALLING,
    DEVICE_I2S_EDGE_RISING
};

static void i2s_test_print_usage(char *argv[])
{
    printf("Usage: %s <-t|-r> <-i|-l> [-f frequency] [-v volume] [-c] "
           "<rb entries> <sample per rb entry>\n", argv[0]);
    printf("\t-t    transmit audio data\n");
    printf("\t-r    receive audio data\n");
    printf("\t-i    use I2S protocol\n");
    printf("\t-l    use LR Stereo protocol\n");
    printf("\t-f    generate output sine wave with \n");
    printf("\t\t frequency    decimal number between %d-%d (Hz)\n",
           MIN_AUDIO_FREQUENCY, MAX_AUDIO_FREQUENCY);
    printf("\t-v    generate output sine wave with \n");
    printf("\t\t volume    decimal number between %d-%d\n",
           MIN_AUDIO_VOLUME, MAX_AUDIO_VOLUME);
    printf("\t\t                  %d - mute\n", MIN_AUDIO_VOLUME);
    printf("\t\t                  %d - max volume\n", MAX_AUDIO_VOLUME);
    printf("\t-c    check receive data (only valid with -r)\n");
}

static int i2s_test_parse_cmdline(int argc, char *argv[],
                                  struct i2s_test_info *info)
{
    int tcnt, rcnt, icnt, lcnt, ccnt, vcnt, fcnt, errcnt;
    int option;
    int ret = 0;

    tcnt = rcnt = icnt = lcnt = ccnt = vcnt = fcnt = errcnt = 0;

    if (argc > 11) {
        fprintf(stderr, "Too many arguments: %d\n",argc);
        return -EINVAL;
    }

    info->aud_frequency = DEFAULT_AUDIO_FREQUENCY;
    info->aud_volume = DEFAULT_AUDIO_VOLUME;

    optind = -1;
    while ((option = getopt(argc, argv, "trilf:v:c")) != ERROR) {
        switch(option) {
        case 't':
            info->is_transmitter = 1;
            tcnt++;
            break;
        case 'r':
            info->is_receiver = 1;
            rcnt++;
            break;
        case 'i':
            info->is_i2s = 1;
            icnt++;
            break;
        case 'l':
            lcnt++;
            break;
        case 'f':
            info->is_gen_audio = 1;
            fcnt++;
            ret = sscanf(optarg, "%u", &info->aud_frequency);
            if (ret != 1) {
                info->aud_frequency = DEFAULT_AUDIO_FREQUENCY;
                fprintf(stderr,
                        "Invalid frequency (%s)! Using default %uHz (%u-%u).\n",
                        optarg,
                        info->aud_frequency,
                        MIN_AUDIO_FREQUENCY,
                        MAX_AUDIO_FREQUENCY);
            } else if ((info->aud_frequency < MIN_AUDIO_FREQUENCY) ||
                       (info->aud_frequency > MAX_AUDIO_FREQUENCY) ) {
                info->aud_frequency = DEFAULT_AUDIO_FREQUENCY;
                fprintf(stderr,
                        "Out of bounds frequency (%s)! Using default %uHz (%u-%u).\n",
                        optarg,
                        info->aud_frequency,
                        MIN_AUDIO_FREQUENCY,
                        MAX_AUDIO_FREQUENCY);
            }
            break;

        case 'v':
            info->is_gen_audio = 1;
            vcnt++;
            ret = sscanf(optarg, "%u", &info->aud_volume);
            if (ret != 1) {
                info->aud_volume = DEFAULT_AUDIO_VOLUME;
                fprintf(stderr,
                        "Invalid volume (%s)! Using default %u (%u-%u).\n",
                        optarg,
                        info->aud_volume,
                        MIN_AUDIO_VOLUME,
                        MAX_AUDIO_VOLUME);
            } else if ((info->aud_volume < MIN_AUDIO_VOLUME) ||
                       (info->aud_volume > MAX_AUDIO_VOLUME)) {
                info->aud_volume = DEFAULT_AUDIO_VOLUME;
                fprintf(stderr,
                        "Out of bounds volume (%s)! Using default %u (%u-%u).\n",
                        optarg,
                        info->aud_volume,
                        MIN_AUDIO_VOLUME,
                        MAX_AUDIO_VOLUME);
            }
            break;
        case 'c':
            info->check_rx_data = 1;
            ccnt++;
            break;
        default:
            errcnt++;
        }
    }

    if (info->is_gen_audio && !info->is_transmitter) {
        fprintf(stderr, "Must enable transmitter mode to generate audio\n");
        return -EINVAL;
    }

    if ((tcnt > 1) || (rcnt > 1) || ((tcnt != 1) && (rcnt != 1)) ||
        ((icnt + lcnt) != 1) || (ccnt && !rcnt) ||
        ((vcnt | fcnt) && !tcnt) ||
        errcnt){
        return -EINVAL;
    }

    if (optind + 1 >= argc) {
        fprintf(stderr,
                "Missing rb entries and/or sample per rb entry parameter\n");
        return -EINVAL;
    }

    info->rb_entries = strtoul(argv[optind], NULL, 0);
    info->samples_per_rb_entry = strtoul(argv[optind + 1], NULL, 0);

    return 0;
}

static void i2s_test_print_cmdline_summary(struct i2s_test_info *info)
{
    printf("\n");
    printf("-> %s %s data\n",
            info->is_transmitter ? "Transmitting" : "Receiving",
            info->is_i2s ? "I2S" : "LR Stereo");
    printf("   %lu ring buffer entries of %lu audio samples each\n",
            info->rb_entries, info->samples_per_rb_entry);
    if (info->is_gen_audio) {
        printf("Generate Audio Frequency: %lu Hz, Volume %lu\n",
                info->aud_frequency,
                info->aud_volume);
    }
}

static int i2s_test_rb_fill_and_pass(struct ring_buf *rb, void *arg)
{
    struct i2s_test_info *info = arg;
    struct i2s_test_sample *sample;
    unsigned int i;
    uint16_t right = 0;

    ring_buf_reset(rb);
    sample = ring_buf_get_tail(rb);

    for (i = 0; i < (ring_buf_space(rb) / sizeof(*sample)); i++) {
        sample[i].left = info->left;
        sample[i].right = right++;
    }

    ring_buf_put(rb, i * sizeof(*sample));
    ring_buf_pass(rb);

    info->left++;

    return 0;
}

static int i2s_test_rb_fill_sinewave_and_pass(struct ring_buf *rb)
{
    int ret_value = 0;
    uint32_t buff_size;

    ring_buf_reset(rb);
    buff_size = ring_buf_space(rb);


    ret_value = fill_output_buff((int16_t *)ring_buf_get_tail(rb),
                                 &buff_size,
                                 (sizeof(struct i2s_test_sample) /
                                  sizeof(((struct i2s_test_sample*)0)->left)));
    if (ret_value >= 0) {
        ring_buf_put(rb, ret_value);
        ring_buf_pass(rb);
        ret_value = 0;
    }

    return ret_value;
}

static void i2s_test_tx_callback(struct ring_buf *rb,
                                 enum device_i2s_event event, void *arg)
{
    struct i2s_test_info *info = arg;

    if (event != DEVICE_I2S_EVENT_TX_COMPLETE)
        info->stats.tx_err_cnt++;

    /* Don't use printf since its called from irq context */
    switch (event) {
    case DEVICE_I2S_EVENT_TX_COMPLETE:
        info->stats.tx_cnt++;

        if (ring_buf_is_producers(rb)) {
            if (info->is_gen_audio) {
                i2s_test_rb_fill_sinewave_and_pass(rb);
            } else {
                i2s_test_rb_fill_and_pass(rb, arg);
            }
        }

        return;
    case DEVICE_I2S_EVENT_UNDERRUN:
        lldbg("CB EVENT: DEVICE_I2S_EVENT_UNDERRUN\n");
        break;
    case DEVICE_I2S_EVENT_OVERRUN:
        lldbg("CB EVENT: DEVICE_I2S_EVENT_OVERRUN\n");
        break;
    case DEVICE_I2S_EVENT_CLOCKING:
        lldbg("CB EVENT: DEVICE_I2S_EVENT_CLOCKING\n");
        break;
    case DEVICE_I2S_EVENT_DATA_LEN:
        lldbg("CB EVENT: DEVICE_I2S_EVENT_DATA_LEN\n");
        break;
    case DEVICE_I2S_EVENT_UNSPECIFIED:
        lldbg("CB EVENT: DEVICE_I2S_EVENT_UNSPECIFIED\n");
        break;
    default:
        lldbg("CB EVENT: Unknown\n");
    }

    sem_post(&i2s_test_done_sem);
}

int i2s_test_start_transmitter(struct i2s_test_info *info,
                               struct device *dev)
{
    struct ring_buf *rb;
    int ret;

    /* 16-bit stereo */
    rb = ring_buf_alloc_ring(info->rb_entries, 0,
                             info->samples_per_rb_entry *
                             sizeof(struct i2s_test_sample),
                             0, i2s_test_rb_fill_and_pass, NULL, info);
    if (!rb) {
        fprintf(stderr, "ring_buf_alloc_ring failed\n");
        return -EIO;
    }

    ret = device_i2s_prepare_transmitter(dev, rb, i2s_test_tx_callback, info);
    if (ret) {
        fprintf(stderr, "prepare_tx failed: %d\n", ret);
        goto err_free_ring;
    }

    ret = device_i2s_start_transmitter(dev);
    if (ret) {
        fprintf(stderr, "start_tx failed: %d\n", ret);
        goto err_shutdown_transmitter;
    }

    return 0;

err_shutdown_transmitter:
    device_i2s_shutdown_transmitter(dev);
err_free_ring:
    ring_buf_free_ring(rb, NULL, NULL);

    return ret;
}

void i2s_test_stop_transmitter(struct device *dev)
{
    int ret;

    ret = device_i2s_stop_transmitter(dev);
    if (ret)
        fprintf(stderr, "stop_tx failed: %d\n", ret);

    ret = device_i2s_shutdown_transmitter(dev);
    if (ret)
        fprintf(stderr, "shutdown_tx failed: %d\n", ret);
}

static void i2s_test_rx_check_data(struct i2s_test_info *info,
                                   struct ring_buf *rb)
{
    struct i2s_test_sample *sample;
    unsigned int i;
    static uint8_t first_time = 1;

    sample = ring_buf_get_head(rb);

    for (i = 0; i < (ring_buf_len(rb) / sizeof(*sample)); i++) {
        if (first_time) {
            info->left = sample[i].left;
            info->right = sample[i].right;
            first_time = 0;
        }

        if ((sample[i].left != info->left) ||
            (sample[i].right != info->right)) {

            info->stats.rx_bad_data_cnt++;

            lldbg("Bad Data received: 0x%04x 0x%04x, expected: 0x%04x 0x%04x\n",
                  sample[i].left, sample[i].right, info->left, info->right);

            info->left = sample[i].left;
            info->right = sample[i].right;
        }

        if (++info->right >= info->samples_per_rb_entry) {
            info->left++;
            info->right = 0;
        }
    }
}

static void i2s_test_rx_callback(struct ring_buf *rb,
                                 enum device_i2s_event event, void *arg)
{
    struct i2s_test_info *info = arg;

    if (event != DEVICE_I2S_EVENT_RX_COMPLETE)
        info->stats.rx_err_cnt++;

    /* Don't use printf since its called from irq context */
    switch (event) {
    case DEVICE_I2S_EVENT_RX_COMPLETE:
        info->stats.rx_cnt++;

        if (info->check_rx_data)
            i2s_test_rx_check_data(info, rb);

        if (ring_buf_is_consumers(rb)) {
            ring_buf_reset(rb);
            ring_buf_pass(rb);
        }

        return;
    case DEVICE_I2S_EVENT_UNDERRUN:
        lldbg("CB EVENT: DEVICE_I2S_EVENT_UNDERRUN\n");
        break;
    case DEVICE_I2S_EVENT_OVERRUN:
        lldbg("CB EVENT: DEVICE_I2S_EVENT_OVERRUN\n");
        break;
    case DEVICE_I2S_EVENT_CLOCKING:
        lldbg("CB EVENT: DEVICE_I2S_EVENT_CLOCKING\n");
        break;
    case DEVICE_I2S_EVENT_DATA_LEN:
        lldbg("CB EVENT: DEVICE_I2S_EVENT_DATA_LEN\n");
        break;
    case DEVICE_I2S_EVENT_UNSPECIFIED:
        lldbg("CB EVENT: DEVICE_I2S_EVENT_UNSPECIFIED\n");
        break;
    default:
        lldbg("CB EVENT: Unknown\n");
    }

    sem_post(&i2s_test_done_sem);
}

static int i2s_test_start_receiver(struct i2s_test_info *info,
                                   struct device *dev)
{
    struct ring_buf *rb;
    int ret;

    /* 16-bit stereo */
    rb = ring_buf_alloc_ring(info->rb_entries, 0,
                             info->samples_per_rb_entry *
                                 sizeof(struct i2s_test_sample),
                             0, NULL, NULL, NULL);
    if (!rb) {
        fprintf(stderr, "ring_buf_alloc_ring failed\n");
        return -EIO;
    }

    ret = device_i2s_prepare_receiver(dev, rb, i2s_test_rx_callback, info);
    if (ret) {
        fprintf(stderr, "prepare_rx failed: %d\n", ret);
        goto err_free_ring;
    }

    ret = device_i2s_start_receiver(dev);
    if (ret) {
        fprintf(stderr, "start_rx failed: %d\n", ret);
        goto err_shutdown_receiver;
    }

    return 0;

err_shutdown_receiver:
    device_i2s_shutdown_receiver(dev);
err_free_ring:
    ring_buf_free_ring(rb, NULL, NULL);

    return ret;
}

static void i2s_test_stop_receiver(struct device *dev)
{
    int ret;

    ret = device_i2s_stop_receiver(dev);
    if (ret)
        fprintf(stderr, "stop_rx failed: %d\n", ret);

    ret = device_i2s_shutdown_receiver(dev);
    if (ret)
        fprintf(stderr, "shutdown_rx failed: %d\n", ret);
}

static int i2s_test_start_streaming_transmitter(struct i2s_test_info *info)
{
    struct device *dev;
    struct device_i2s_dai dai;
    int ret;

    if ((!info->is_transmitter) ||
        (info->is_receiver)){
        fprintf(stderr, "Start streaming call does not match device type\n");
        ret = -EINVAL;
    }

    dev = device_open(DEVICE_TYPE_I2S_HW, 0);
    if (!dev) {
        fprintf(stderr, "open failed\n");
        return -EIO;
    }

    /* Validate transmitter configuration */
    ret = device_i2s_get_caps(dev,
                              DEVICE_I2S_ROLE_MASTER,
                              &i2s_test_pcm,
                              &dai);

    /* Check for matching test configuration */
    if (ret) {

        fprintf(stderr,
                "I2S master does support hard coded pcm test configuration\n");
        goto err_dev_close;
    }

    if (!((dai.mclk_freq == test_i2s_dai_m.mclk_freq) &&
         (dai.wclk_polarity | test_i2s_dai_m.wclk_polarity) &&
         (dai.data_rx_edge | test_i2s_dai_m.data_rx_edge) &&
         (dai.data_tx_edge | test_i2s_dai_m.data_tx_edge))) {

        fprintf(stderr,
                "I2S master does support hard coded dai test configuration\n");
        goto err_dev_close;
    }

    /* Master is opposite of the slave setting */
    if (!(dai.wclk_change_edge | DEVICE_I2S_EDGE_FALLING)) {
        fprintf(stderr,
                "Transmitter test mode settings require wclk falling\n");
        goto err_dev_close;
    }
    test_i2s_dai_m.wclk_change_edge |= DEVICE_I2S_EDGE_FALLING;

    if (info->is_i2s) {
        if (!(dai.protocol | DEVICE_I2S_PROTOCOL_I2S)) {
            fprintf(stderr, "I2S master port does not support I2S protocol\n");
            goto err_dev_close;
        }
        test_i2s_dai_m.protocol |= DEVICE_I2S_PROTOCOL_I2S;
    } else {
        if (!(dai.protocol | DEVICE_I2S_PROTOCOL_LR_STEREO)) {
            fprintf(stderr, "I2S master port does not support LR protocol\n");
            goto err_dev_close;
        }
        test_i2s_dai_m.protocol |= DEVICE_I2S_PROTOCOL_LR_STEREO;
    }

    ret = device_i2s_set_config(dev,
                                DEVICE_I2S_ROLE_MASTER,
                                &i2s_test_pcm,
                                &test_i2s_dai_m);
    if (ret) {
        fprintf(stderr, "set configuration failed: %d\n", ret);
        goto err_dev_close;
    }

    ret = i2s_test_start_transmitter(info, dev);
    if (ret)
        goto err_dev_close;

    /*
     * Wait forever.  Can't just exit because callback is still being
     * called by driver to keep filling/draining the ring buffer.
     */
    while (sem_wait(&i2s_test_done_sem) && (errno == EINTR));

    i2s_test_stop_transmitter(dev);

err_dev_close:
    device_close(dev);

    return ret;
}

static int i2s_test_start_streaming_receiver(struct i2s_test_info *info)
{
    struct device *dev;
    struct device_i2s_dai dai;
    int ret;

    if ((info->is_transmitter) &&
        (!info->is_receiver)){
        fprintf(stderr, "Start streaming call does not match device type\n");
        ret = -EINVAL;
    }

    dev = device_open(DEVICE_TYPE_I2S_HW, 0);
    if (!dev) {
        fprintf(stderr, "open failed\n");
        return -EIO;
    }

    ret = device_i2s_get_caps(dev,
                              DEVICE_I2S_ROLE_SLAVE,
                              &i2s_test_pcm,
                              &dai);

    if (ret) {
        fprintf(stderr,
               "I2S slave configuration does not support test mode settings\n");
        goto err_dev_close;
    }

    /* Use whatever mclk frequency we are handed back */
    test_i2s_dai_s.mclk_freq = dai.mclk_freq;

    /* Verify match with hard coded settings */
    if (!((dai.wclk_polarity | test_i2s_dai_s.wclk_polarity) &&
         (dai.wclk_change_edge | test_i2s_dai_s.wclk_change_edge) &&
         (dai.data_rx_edge | test_i2s_dai_s.data_rx_edge) &&
         (dai.data_tx_edge | test_i2s_dai_s.data_tx_edge))) {

        fprintf(stderr,
                "I2S slave does support hard coded dai test configuration\n");
        goto err_dev_close;
    }

    if (info->is_i2s) {
        if (!(dai.protocol | DEVICE_I2S_PROTOCOL_I2S)) {
            fprintf(stderr, "I2S slave port does not support I2S protocol\n");
            goto err_dev_close;
        }
        test_i2s_dai_s.protocol |= DEVICE_I2S_PROTOCOL_I2S;
    } else {
        if (!(dai.protocol | DEVICE_I2S_PROTOCOL_LR_STEREO)) {
            fprintf(stderr, "I2S slave port does not support LR protocol\n");
            goto err_dev_close;
        }
        test_i2s_dai_s.protocol |= DEVICE_I2S_PROTOCOL_LR_STEREO;
    }

    ret = device_i2s_set_config(dev,
                               DEVICE_I2S_ROLE_SLAVE,
                               &i2s_test_pcm,
                               &test_i2s_dai_s);
    if (ret) {
        fprintf(stderr, "set configuration failed: %d\n", ret);
        goto err_dev_close;
    }

    ret = i2s_test_start_receiver(info, dev);
    if (ret)
        goto err_dev_close;

    /*
     * Wait forever.  Can't just exit because callback is still being
     * called by driver to keep filling/draining the ring buffer.
     */
    while (sem_wait(&i2s_test_done_sem) && (errno == EINTR));

    i2s_test_stop_receiver(dev);

err_dev_close:
    device_close(dev);

    return ret;
}

static void i2s_test_sig_handler(int sig)
{
    errno = 0;
    sem_post(&i2s_test_done_sem);
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int i2s_test_main(int argc, char *argv[])
#endif
{
    struct sigaction sig_act;
    struct i2s_test_info *info;
    int ret;

    info = zalloc(sizeof(*info));
    if (!info) {
        fprintf(stderr, "Can't alloc 'info'\n");
        return EXIT_FAILURE;
    }

    ret = i2s_test_parse_cmdline(argc, argv, info);
    if (ret) {
        i2s_test_print_usage(argv);
        ret = EXIT_FAILURE;
        goto err_free_info;
    }

    i2s_test_print_cmdline_summary(info);
    printf("   info struct address: 0x%08x\n", info);

    sem_init(&i2s_test_done_sem, 0, 0);

    sig_act.sa_handler = i2s_test_sig_handler;
    sig_act.sa_flags = 0;

    ret = sigaction(SIGKILL, &sig_act, NULL);
    if (ret != OK) {
        fprintf(stderr, "SIGKILL sigaction failed: %d - %s\n", errno,
                strerror(errno));
        ret = EXIT_FAILURE;
        goto err_sem_destroy;
    }

    ret = sigaction(SIGTERM, &sig_act, NULL);
    if (ret != OK) {
        fprintf(stderr, "SIGTERM sigaction failed: %d - %s\n", errno,
                strerror(errno));
        ret = EXIT_FAILURE;
        goto err_sem_destroy;
    }

    if (info->is_gen_audio) {
        gen_audio_init(info->aud_frequency, DEFAULT_SAMPLE_RATE, info->aud_volume);
    }

    if (info->is_transmitter) {

        ret = i2s_test_start_streaming_transmitter(info);

    } else {
        ret = i2s_test_start_streaming_receiver(info);
    }
    if (ret)
        ret = EXIT_FAILURE;

err_sem_destroy:
    sem_destroy(&i2s_test_done_sem);
err_free_info:
    free(info);

    return ret;
}
