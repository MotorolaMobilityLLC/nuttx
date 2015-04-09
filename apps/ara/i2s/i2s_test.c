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

#ifndef SIGKILL
#define SIGKILL     9
#endif

#ifndef SIGTERM
#define SIGTERM     15
#endif

struct i2s_test_stats {
    unsigned int    tx_cnt;
    unsigned int    tx_err_cnt;
    unsigned int    rx_cnt;
    unsigned int    rx_err_cnt;
    unsigned int    rx_bad_data_cnt;
};

struct i2s_test_info {
    uint8_t                 is_mclk_master;
    uint8_t                 is_bclk_master;
    uint8_t                 is_transmitter;
    uint8_t                 is_receiver;
    uint8_t                 is_i2s;
    uint8_t                 check_rx_data;
    unsigned long           rb_entries;
    unsigned long           samples_per_rb_entry;
    uint16_t                left;
    uint16_t                right;
    struct i2s_test_stats   stats;
};

static sem_t i2s_test_done_sem;

struct i2s_test_sample {
    uint16_t    left;
    uint16_t    right;
};

static void i2s_test_print_usage(char *argv[])
{
    printf("Usage: %s <-M|-m> <-B|-b> <-t|-r> <-i|-l> [-c] "
           "<rb entries> <sample per rb entry>\n", argv[0]);
    printf("\t-M    be MCLK master\n");
    printf("\t-m    be MCLK slave\n");
    printf("\t-B    be BCLK & LRCLK master\n");
    printf("\t-b    be BCLK & LRCLK slave\n");
    printf("\t-t    transmit audio data\n");
    printf("\t-r    receive audio data\n");
    printf("\t-i    use I2S protocol\n");
    printf("\t-l    use LR Stereo protocol\n");
    printf("\t-c    check receive data (only valid with -r)\n");
}

static int i2s_test_parse_cmdline(int argc, char *argv[],
                                  struct i2s_test_info *info)
{
    int Mcnt, mcnt, Bcnt, bcnt, tcnt, rcnt, icnt, lcnt, ccnt, errcnt;
    int option;

    Mcnt = mcnt = Bcnt = bcnt = tcnt = rcnt = icnt = lcnt = ccnt = errcnt = 0;

    if ((argc != 7) && (argc != 8))
        return -EINVAL;

    while ((option = getopt(argc, argv, "MmBbtrilc")) != ERROR) {
        switch(option) {
        case 'M':
            info->is_mclk_master = 1;
            Mcnt++;
            break;
        case 'm':
            mcnt++;
            break;
        case 'B':
            info->is_bclk_master = 1;
            Bcnt++;
            break;
        case 'b':
            bcnt++;
            break;
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
        case 'c':
            info->check_rx_data = 1;
            ccnt++;
            break;
        default:
            errcnt++;
        }
    }

    if (((Mcnt + mcnt) != 1) || ((Bcnt + bcnt) != 1) || (tcnt > 1) ||
        (rcnt > 1) || ((tcnt != 1) && (rcnt != 1)) || ((icnt + lcnt) != 1) ||
        (ccnt && !rcnt) || errcnt) {

        return -EINVAL;
    }

    info->rb_entries = strtoul(argv[optind], NULL, 0);
    info->samples_per_rb_entry = strtoul(argv[optind + 1], NULL, 0);

    return 0;
}

static void i2s_test_print_cmdline_summary(struct i2s_test_info *info)
{
    printf("\n");
    printf("-> %s %s data as MCLK %s and BCLK/LRCLK %s\n",
            info->is_transmitter ? "Transmitting" : "Receiving",
            info->is_i2s ? "I2S" : "LR Stereo",
            info->is_mclk_master ? "Master" : "Slave",
            info->is_bclk_master ? "Master" : "Slave");
    printf("   %u ring buffer entries of %u audio samples each\n",
            info->rb_entries, info->samples_per_rb_entry);
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

        if (ring_buf_is_producers(rb))
            i2s_test_rb_fill_and_pass(rb, arg);

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

static int i2s_test_start_transmitter(struct i2s_test_info *info,
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

static void i2s_test_stop_transmitter(struct device *dev)
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

static int i2s_test_start_streaming(struct i2s_test_info *info)
{
    struct device *dev;
    uint16_t config_count;
    const struct device_i2s_configuration *configs, *cfg;
    struct device_i2s_configuration config;
    unsigned int i;
    int ret;

    dev = device_open(DEVICE_TYPE_I2S_HW, 0);
    if (!dev) {
        fprintf(stderr, "open failed\n");
        return -EIO;
    }

    ret = device_i2s_get_supported_configurations(dev, &config_count, &configs);
    if (ret) {
        fprintf(stderr, "get_supported_configs failed; %d\n", ret);
        goto err_dev_close;
    }

    /* Pick 48KHz 16-bits/channel */
    for (i = 0, cfg = configs; i < config_count; i++, cfg++) {
        if ((cfg->sample_frequency == 48000) &&
            (cfg->num_channels == 2) &&
            (cfg->bytes_per_channel == 2) &&
            (cfg->byte_order & DEVICE_I2S_BYTE_ORDER_LE) &&
            (cfg->spatial_locations ==
                 (DEVICE_I2S_SPATIAL_LOCATION_FL |
                  DEVICE_I2S_SPATIAL_LOCATION_FR)) &&
            ((info->is_i2s && (cfg->ll_protocol & DEVICE_I2S_PROTOCOL_I2S)) ||
             (!info->is_i2s && (cfg->ll_protocol &
                                DEVICE_I2S_PROTOCOL_LR_STEREO))) &&
            ((info->is_mclk_master &&
              (cfg->ll_mclk_role & DEVICE_I2S_ROLE_MASTER)) ||
             (!info->is_mclk_master &&
              (cfg->ll_mclk_role & DEVICE_I2S_ROLE_SLAVE))) &&
            ((info->is_bclk_master &&
              (cfg->ll_bclk_role & DEVICE_I2S_ROLE_MASTER)) ||
             (!info->is_bclk_master &&
              (cfg->ll_bclk_role & DEVICE_I2S_ROLE_SLAVE))) &&
            (cfg->ll_wclk_polarity & DEVICE_I2S_POLARITY_NORMAL) &&
            (cfg->ll_wclk_change_edge & DEVICE_I2S_EDGE_FALLING) &&
            (cfg->ll_data_tx_edge & DEVICE_I2S_EDGE_FALLING) &&
            (cfg->ll_data_rx_edge & DEVICE_I2S_EDGE_RISING) &&
            ((info->is_i2s && (cfg->ll_data_offset == 1)) ||
             (!info->is_i2s && (cfg->ll_data_offset == 0)))) {

            break;
        }
    }

    if (i >= config_count) {
        fprintf(stderr, "no valid configuration\n");
        ret = -EINVAL;
        goto err_dev_close;
    }

    memcpy(&config, cfg, sizeof(config));

    config.byte_order = DEVICE_I2S_BYTE_ORDER_LE;

    if (info->is_i2s)
        config.ll_protocol = DEVICE_I2S_PROTOCOL_I2S;
    else
        config.ll_protocol = DEVICE_I2S_PROTOCOL_LR_STEREO;

    if (info->is_mclk_master)
        config.ll_mclk_role = DEVICE_I2S_ROLE_MASTER;
    else
        config.ll_mclk_role = DEVICE_I2S_ROLE_SLAVE;

    if (info->is_bclk_master) {
        config.ll_bclk_role = DEVICE_I2S_ROLE_MASTER;
        config.ll_wclk_role = DEVICE_I2S_ROLE_MASTER;
    } else {
        config.ll_bclk_role = DEVICE_I2S_ROLE_SLAVE;
        config.ll_wclk_role = DEVICE_I2S_ROLE_SLAVE;
    }

    if (info->is_transmitter)
        config.ll_wclk_change_edge = DEVICE_I2S_EDGE_RISING;
    else
        config.ll_wclk_change_edge = DEVICE_I2S_EDGE_FALLING;

    config.ll_wclk_polarity = DEVICE_I2S_POLARITY_NORMAL;
    config.ll_data_tx_edge = DEVICE_I2S_EDGE_FALLING;
    config.ll_data_rx_edge = DEVICE_I2S_EDGE_RISING;

    ret = device_i2s_set_configuration(dev, &config);
    if (ret) {
        fprintf(stderr, "set_configs failed: %d\n", ret);
        goto err_dev_close;
    }

    if (info->is_transmitter) {
        ret = i2s_test_start_transmitter(info, dev);
        if (ret)
            goto err_dev_close;
    }

    if (info->is_receiver) {
        if (info->is_transmitter) {
            printf("Transmitter started, press <enter> to start receiving:\n");
            fgetc(stdin);
            printf("Started...\n");
        }

        ret = i2s_test_start_receiver(info, dev);
        if (ret)
            goto err_dev_close;
    }

    if (ret)
        goto err_dev_close;

    /*
     * Wait forever.  Can't just exit because callback is still being
     * called by driver to keep filling/draining the ring buffer.
     */
    while (sem_wait(&i2s_test_done_sem) && (errno == EINTR));

    if (info->is_receiver)
        i2s_test_stop_receiver(dev);

    if (info->is_transmitter)
        i2s_test_stop_transmitter(dev);

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

    ret = i2s_test_start_streaming(info);
    if (ret)
        ret = EXIT_FAILURE;

err_sem_destroy:
    sem_destroy(&i2s_test_done_sem);
err_free_info:
    free(info);

    return ret;
}
