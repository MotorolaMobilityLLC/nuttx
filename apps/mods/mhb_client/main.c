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

#include <nuttx/config.h>

#include <errno.h>
#include <reglog.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <arch/byteorder.h>

#include <nuttx/device.h>
#include <nuttx/device_display.h>
#include <nuttx/device_lights.h>
#include <nuttx/device_slave_pwrctrl.h>
#include <nuttx/util.h>

#include <nuttx/i2s_tunnel/i2s_unipro.h>

#include <nuttx/mhb/device_mhb.h>
#include <nuttx/mhb/mhb_protocol.h>
#include <nuttx/mhb/mhb_utils.h>

#include <nuttx/unipro/unipro.h>

static struct device *g_backlight;
static struct device *g_display;
static struct mhb_unipro_stats g_unipro_stats;

typedef int (*COMMAND_FP)(int argc, char *argv[], struct device *dev);
struct command {
    const char *str;
    COMMAND_FP fp;
};

static struct device *g_dev;

static const struct command *mhb_find_cmd(const char *str, const struct command *cmds, size_t num_cmds)
{
    while (num_cmds--) {
        if (!strcmp(cmds->str, str))
            return cmds;
        cmds++;
    }
    return NULL;
}

static void mhb_print_cmds(const char *header,
                           const struct command *cmds,
                           size_t num_cmds,
                           const char *trailer)
{
    if (header != NULL) {
        printf("%s [ ", header);
    }
    while (num_cmds--) {
        printf(" %s %c", cmds->str, num_cmds > 0 ? '|' : ' ');
        cmds++;
    }
    printf("]");
    if (trailer != NULL) {
        printf(trailer);
    }
}

static int mhb_handle_sub_cmd(int argc,
                              char *argv[],
                              struct device *dev,
                              const char *str,
                              const struct command *cmds,
                              size_t num_cmds)
{
    static const struct command *cmd;
    int ret = OK;

    cmd = mhb_find_cmd(argv[0], cmds, num_cmds);
    if (cmd == NULL) {
        printf("Error: Unknown %s subcommand: %s\n", str, argv[0]);
        mhb_print_cmds(str, cmds, num_cmds, "\n");
        return -EINVAL;
    }
    ret = cmd->fp(argc-1, &argv[1], dev);
    printf("%s returned: %d\n", cmd->str, ret);
    return ret;
}

static int mhb_send_diag_reg_log_apbe_req_na(struct device *dev)
{
    struct mhb_hdr hdr;

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_DIAG;
    hdr.type = MHB_TYPE_DIAG_REG_LOG_REQ;
    return device_mhb_send(dev, &hdr, NULL, 0, 0);
}

static int mhb_send_diag_reg_log_apba_req_na(struct device *dev)
{
    struct mhb_hdr hdr;

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_DIAG|MHB_PEER_MASK;
    hdr.type = MHB_TYPE_DIAG_REG_LOG_REQ;
    return device_mhb_send(dev, &hdr, NULL, 0, 0);
}

static int mhb_handle_unipro(struct device *dev, struct mhb_hdr *hdr,
               uint8_t *payload, size_t payload_length)
{
    printf("addr=0x%02x, type=0x%02x, result=0x%02x\n",
        hdr->addr, hdr->type, hdr->result);

    const char *res = hdr->result == MHB_RESULT_SUCCESS ? "success" : "failure";

    switch (hdr->type) {
    case MHB_TYPE_UNIPRO_CONFIG_RSP:
        printf("config %s\n", res);
        return 0;
    case MHB_TYPE_UNIPRO_CONTROL_RSP:
        printf("control %s\n", res);
        return 0;
    case MHB_TYPE_UNIPRO_STATUS_RSP: {
        struct mhb_unipro_status_rsp *rsp =
            (struct mhb_unipro_status_rsp *)payload;
        printf("status=0x%08x\n", le32_to_cpu(rsp->status));
        return 0;
    }
    case MHB_TYPE_UNIPRO_READ_ATTR_RSP: {
        struct mhb_unipro_read_attr_rsp *rsp =
            (struct mhb_unipro_read_attr_rsp *)payload;
        printf("read %s, value=0x%08x\n", res, le32_to_cpu(rsp->value));
        return 0;
    }
    case MHB_TYPE_UNIPRO_WRITE_ATTR_RSP:
        printf("write %s\n", res);
        return 0;
    case MHB_TYPE_UNIPRO_STATS_RSP:
    {
        if (!payload || !payload_length) {
            return 0;
        }

        uint32_t *src = (uint32_t *)payload;
        uint32_t *dst = (uint32_t *)&g_unipro_stats;
        size_t count = sizeof(g_unipro_stats) / sizeof(uint32_t);
        size_t i;

        for (i = 0; i < count; i++) {
            dst[i] += le32_to_cpu(src[i]);
            printf("%08x\n", dst[i]);
            dst[i] = 0;
        }

        return 0;
    }
    case MHB_TYPE_UNIPRO_STATS_NOT:
        if (!payload || !payload_length) {
            return 0;
        }

        uint32_t *src = (uint32_t *)payload;
        uint32_t *dst = (uint32_t *)&g_unipro_stats;
        size_t count = sizeof(g_unipro_stats) / sizeof(uint32_t);
        size_t i;

        for (i = 0; i < count; i++) {
            dst[i] += le32_to_cpu(src[i]);
        }

        return 0;
    default:
        printf("ERROR: unknown Unipro event\n");
        return -EINVAL;
    }
}

static int mhb_handle_diag_print_reglog(struct reglog_value_s *entries, size_t n, const char *str)
{
    static unsigned int entry_num = 0;

    if (n != 0)
    {
        do
        {
            FROM_LE(entries->time);
            FROM_LE(entries->addr);
            FROM_LE(entries->val);
            printf("%s %04d - 0x%08x: 0x%08x 0x%08x\n", str, entry_num++, entries->time,
                   entries->addr, entries->val);
            entries++;
        } while (--n);
    }
    else
    {
        entry_num = 0;
    }
    return entry_num;
}

static int mhb_handle_diag(struct device *dev, struct mhb_hdr *hdr,
                          uint8_t *payload, size_t payload_length)
{
    int ret = OK;

    printf("addr=0x%02x, type=0x%02x, result=0x%02x\n",
        hdr->addr, hdr->type, hdr->result);

    const char *res = hdr->result == MHB_RESULT_SUCCESS ? "success" : "failure";

    switch (hdr->type) {
    case MHB_TYPE_DIAG_LOG_RSP:
    case MHB_TYPE_DIAG_LOG_NOT:
    case MHB_TYPE_DIAG_LAST_RSP:
        if (payload_length) {
            printf((char *)payload);
        }
        break;
    case MHB_TYPE_DIAG_MODE_RSP:
        printf("mode %s\n", res);
        break;
    case MHB_TYPE_DIAG_LOOP_RSP:
        break;
    case MHB_TYPE_DIAG_REG_LOG_RSP:
        if (hdr->addr & MHB_PEER_MASK) {
            if (mhb_handle_diag_print_reglog((struct reglog_value_s *)payload,
                                             payload_length/sizeof(struct reglog_value_s),
                                             "APBA") > 0) {
                /* Request more data. */
                ret = mhb_send_diag_reg_log_apba_req_na(dev);
            }
        } else {
            if (mhb_handle_diag_print_reglog((struct reglog_value_s *)payload,
                                             payload_length/sizeof(struct reglog_value_s),
                                             "APBE") > 0) {
                /* Request more data. */
                ret = mhb_send_diag_reg_log_apbe_req_na(dev);
            }
            else {
                /* Request APBA data. */
                ret = mhb_send_diag_reg_log_apba_req_na(dev);
            }
        }
        break;
    default:
        printf("ERROR: unknown client diag event: %d\n", hdr->type);
        ret = -EINVAL;
        break;
    }
    return ret;
}

void mhb_print_i2s_status(const char *header, struct i2s_tunnel_info_s *info)
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

    printf("%s:\n", header);
    printf("  Enabled:                      %u\n", info->enabled);
    printf("  Master:                       %u\n", info->is_master);
    printf("  Clock Rate:                   %u\n", info->bclk_rate);
    printf("  Bytes per sample:             %u\n", info->bytes_per_sample);
    printf("  Round trip 48MHz timer ticks: %u\n", info->roundtrip_timer_ticks);
    printf("  Timer offset ticks:           %d\n", info->timer_offset);
    printf("  Packets received:             %u\n", info->i2s_tunnel_rx_packets);
    printf("  Packet size:                  %u\n", info->i2s_tunnel_packet_size);
    printf("  Samples removed:              %u\n", info->i2s_tx_samples_dropped);
    printf("  Samples repeated:             %u\n", info->i2s_tx_samples_retransmitted);
    printf("  Packets dropped:              %u\n", info->i2s_tx_buffers_dropped);
}

static int mhb_handle_i2s_rx(struct device *dev, struct mhb_hdr *hdr,
               uint8_t *payload, size_t payload_length)
{
    printf("addr=0x%02x, type=0x%02x, result=0x%02x\n",
        hdr->addr, hdr->type, hdr->result);

    const char *res = hdr->result == MHB_RESULT_SUCCESS ? "success" : "failure";

    switch (hdr->type) {
    case MHB_TYPE_I2S_CONFIG_RSP:
        printf("I2S Config: %s.\n", res);
        break;
    case MHB_TYPE_I2S_CONTROL_RSP:
        printf("I2S Cmd: %s\n", res);
        break;
    case MHB_TYPE_I2S_STATUS_RSP:
        mhb_print_i2s_status("APBA Info:", (struct i2s_tunnel_info_s *)&payload[0]);
        mhb_print_i2s_status("APBE Info:",
                             (struct i2s_tunnel_info_s *)&payload[sizeof(struct i2s_tunnel_info_s)]);
        break;
    default:
        printf("ERROR: unknown i2s response\n");
        return -EINVAL;
    }
    return OK;
}

static int mhb_send_gear_req(int argc, char *argv[], struct device *dev)
{
    const char *gear;
    struct mhb_hdr hdr;
    struct mhb_unipro_control_req req;

    if (argc == 0) {
        goto usage;
    }

    gear = argv[0];
    if (!strcmp(gear, "pwm1")) {
        req.gear.tx = 1;
        req.gear.rx = 1;
        req.gear.pwrmode = (UNIPRO_SLOW_MODE << 4)|UNIPRO_SLOW_MODE;
        req.gear.series = UNIPRO_HS_SERIES_A;
    } else if (!strcmp(gear, "pwm2")) {
        req.gear.tx = 2;
        req.gear.rx = 2;
        req.gear.pwrmode = (UNIPRO_SLOW_MODE << 4)|UNIPRO_SLOW_MODE;
        req.gear.series = UNIPRO_HS_SERIES_A;
    } else if (!strcmp(gear, "pwm3")) {
        req.gear.tx = 3;
        req.gear.rx = 3;
        req.gear.pwrmode = (UNIPRO_SLOW_MODE << 4)|UNIPRO_SLOW_MODE;
        req.gear.series = UNIPRO_HS_SERIES_A;
    } else if (!strcmp(gear, "pwm4")) {
        req.gear.tx = 4;
        req.gear.rx = 4;
        req.gear.pwrmode = (UNIPRO_SLOW_MODE << 4)|UNIPRO_SLOW_MODE;
        req.gear.series = UNIPRO_HS_SERIES_A;
    } else if (!strcmp(gear, "hs1a")) {
        req.gear.tx = 1;
        req.gear.rx = 1;
        req.gear.pwrmode = (UNIPRO_FAST_MODE << 4)|UNIPRO_FAST_MODE;
        req.gear.series = UNIPRO_HS_SERIES_A;
    } else if (!strcmp(gear, "hs1b")) {
        req.gear.tx = 1;
        req.gear.rx = 1;
        req.gear.pwrmode = (UNIPRO_FAST_MODE << 4)|UNIPRO_FAST_MODE;
        req.gear.series = 2;
    } else if (!strcmp(gear, "hs2a")) {
        req.gear.tx = 2;
        req.gear.rx = 2;
        req.gear.pwrmode = (UNIPRO_FAST_MODE << 4)|UNIPRO_FAST_MODE;
        req.gear.series = UNIPRO_HS_SERIES_A;
    } else if (!strcmp(gear, "hs2b")) {
        req.gear.tx = 2;
        req.gear.rx = 2;
        req.gear.pwrmode = (UNIPRO_FAST_MODE << 4)|UNIPRO_FAST_MODE;
        req.gear.series = 2;
    } else if (!strcmp(gear, "hs3a")) {
        req.gear.tx = 3;
        req.gear.rx = 3;
        req.gear.pwrmode = (UNIPRO_FAST_MODE << 4)|UNIPRO_FAST_MODE;
        req.gear.series = UNIPRO_HS_SERIES_A;
    } else if (!strcmp(gear, "hs3b")) {
        req.gear.tx = 3;
        req.gear.rx = 3;
        req.gear.pwrmode = (UNIPRO_FAST_MODE << 4)|UNIPRO_FAST_MODE;
        req.gear.series = 2;
    } else if (!strcmp(gear, "cam")) {
        req.gear.tx = 2;
        req.gear.rx = 1;
        req.gear.pwrmode = (UNIPRO_SLOWAUTO_MODE << 4)|UNIPRO_FASTAUTO_MODE;
        req.gear.series = UNIPRO_HS_SERIES_A;
    } else if (!strcmp(gear, "disp")) {
        req.gear.tx = 1;
        req.gear.rx = 3;
        req.gear.pwrmode = (UNIPRO_FASTAUTO_MODE << 4)|UNIPRO_SLOWAUTO_MODE;
        req.gear.series = UNIPRO_HS_SERIES_A;
    } else {
        goto usage;
    }

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_UNIPRO;
    hdr.type = MHB_TYPE_UNIPRO_CONTROL_REQ;
    return device_mhb_send(dev, &hdr, (uint8_t *)&req, sizeof(req), 0);

usage:
    printf("ERROR: usage: "
        CONFIG_MODS_MHB_CLIENT_PROGNAME
        " gear [pwm1|pwm2|pwm3|pwm4|hs1a|hs1b|hs2a|hs2b|hs3a|hs3b|cam|disp]\n");
    return -EINVAL;
}

static int mhb_send_status_req(int argc, char *argv[], struct device *dev)
{
    struct mhb_hdr hdr;

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_UNIPRO;
    hdr.type = MHB_TYPE_UNIPRO_STATUS_REQ;
    return device_mhb_send(dev, &hdr, NULL, 0, 0);
}

static int mhb_send_read_attr_req(int argc, char *argv[], struct device *dev)
{
    struct mhb_hdr hdr;
    struct mhb_unipro_read_attr_req req;

    if (argc < 1) {
        goto usage;
    }

    memset(&req, 0, sizeof(req));

    req.attribute = strtoul(argv[0], NULL, 16);
    argc--; argv++;

    if (argc) req.selector = strtoul(argv[0], NULL, 10);
    argc--; argv++;

    if (argc) req.peer = strtoul(argv[0], NULL, 10);
    argc--; argv++;

    printf("attribute=0x%04x selector=%d peer=%d\n",
        req.attribute, req.selector, req.peer);

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_UNIPRO;
    hdr.type = MHB_TYPE_UNIPRO_READ_ATTR_REQ;
    return device_mhb_send(dev, &hdr, (uint8_t *)&req, sizeof(req), 0);

usage:
    printf("ERROR: usage: "
        CONFIG_MODS_MHB_CLIENT_PROGNAME
        " read <attr (hex)> [selector (dec)] [local|peer]\n");
    return -EINVAL;
}

static int mhb_send_write_attr_req(int argc, char *argv[], struct device *dev)
{
    struct mhb_hdr hdr;
    struct mhb_unipro_write_attr_req req;

    if (argc < 2) {
        goto usage;
    }

    memset(&req, 0, sizeof(req));

    req.attribute = strtoul(argv[0], NULL, 16);
    argc--; argv++;

    req.value = strtoul(argv[0], NULL, 16);
    argc--; argv++;

    if (argc) req.selector = strtoul(argv[0], NULL, 10);
    argc--; argv++;

    if (argv[0][0]) req.peer = strtoul(argv[0], NULL, 10);
    argc--; argv++;

    printf("attribute=0x%04x value=0x%08x, selector=%d peer=%d\n",
        req.attribute, req.value, req.selector, req.peer);

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_UNIPRO;
    hdr.type = MHB_TYPE_UNIPRO_WRITE_ATTR_REQ;
    return device_mhb_send(dev, &hdr, (uint8_t *)&req, sizeof(req), 0);

usage:
    printf("ERROR: usage: "
        CONFIG_MODS_MHB_CLIENT_PROGNAME
        " write <attr (hex)> <value (hex) [selector (dec)] [local|peer]\n");
    return -EINVAL;
}

static int mhb_send_stats_req(int argc, char *argv[], struct device *dev)
{
    struct mhb_hdr hdr;

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_UNIPRO;
    hdr.type = MHB_TYPE_UNIPRO_STATS_REQ;
    return device_mhb_send(dev, &hdr, NULL, 0, 0);
}

static int mhb_send_diag_log_req(int argc, char *argv[], struct device *dev)
{
    struct mhb_hdr hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_DIAG;
    hdr.type = MHB_TYPE_DIAG_LOG_REQ;

    return device_mhb_send(dev, &hdr, NULL, 0, 0);
}

static int mhb_send_diag_mode_req(int argc, char *argv[], struct device *dev)
{
    struct mhb_hdr hdr;
    struct mhb_diag_mode_req req;

    if (argc < 1) {
        goto usage;
    }

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_DIAG;
    hdr.type = MHB_TYPE_DIAG_MODE_REQ;

    req.mode = strtoul(argv[0], NULL, 10);
    argc--; argv++;

    printf("mode=%d (0x%08x)\n", req.mode, req.mode);

    req.mode = cpu_to_le32(req.mode);

    return device_mhb_send(dev, &hdr, (const uint8_t *)&req, sizeof(req), 0);

usage:
    printf("ERROR: usage: "
        CONFIG_MODS_MHB_CLIENT_PROGNAME " write <mode (dec)>\n");
    return -EINVAL;

}

static int mhb_send_i2s_enable(int argc, char *argv[], struct device *dev)
{
    struct mhb_hdr hdr;
    struct mhb_i2s_control_req i2s_cmd;
    (void)argc;
    (void)argv;

    hdr.addr = MHB_ADDR_I2S;
    hdr.type = MHB_TYPE_I2S_CONTROL_REQ;
    hdr.result = 0;
    i2s_cmd.command = MHB_I2S_COMMAND_ENABLE;
    return device_mhb_send(dev, &hdr, (const uint8_t *)&i2s_cmd, sizeof(i2s_cmd), 0);
}

static int mhb_send_i2s_disable(int argc, char *argv[], struct device *dev)
{
    struct mhb_hdr hdr;
    struct mhb_i2s_control_req i2s_cmd;
    (void)argc;
    (void)argv;

    hdr.addr = MHB_ADDR_I2S;
    hdr.type = MHB_TYPE_I2S_CONTROL_REQ;
    hdr.result = 0;
    i2s_cmd.command = MHB_I2S_COMMAND_DISABLE;
    return device_mhb_send(dev, &hdr, (const uint8_t *)&i2s_cmd, sizeof(i2s_cmd), 0);
}

static int mhb_send_i2s_config(int argc, char *argv[], struct device *dev)
{
    struct mhb_hdr hdr;
    struct mhb_i2s_config i2s_config;

    hdr.addr = MHB_ADDR_I2S;
    hdr.type = MHB_TYPE_I2S_CONFIG_REQ;
    hdr.result = 0;
    /* Set the defaults. */
    i2s_config.sample_rate  = 48000;
    i2s_config.protocol     = MHB_I2S_PROTOCOL_I2S;
    i2s_config.sample_size  = 16;
    i2s_config.num_channels = 2;
    i2s_config.tx_edge      = MHB_I2S_EDGE_FALLING;
    i2s_config.rx_edge      = MHB_I2S_EDGE_RISING;
    i2s_config.wclk_edge    = MHB_I2S_EDGE_RISING;
    i2s_config.clk_role     = MHB_I2S_ROLE_MASTER;
    switch (argc) {
        case 8:
            if (strcmp(argv[7], "falling") == 0)
                i2s_config.wclk_edge = MHB_I2S_EDGE_FALLING;
        case 7:
            if (strcmp(argv[6], "falling") == 0)
                i2s_config.rx_edge = MHB_I2S_EDGE_RISING;
        case 6:
            if (strcmp(argv[5], "rising") == 0)
                i2s_config.tx_edge = MHB_I2S_EDGE_FALLING;
        case 5:
            i2s_config.num_channels = atoi(argv[3]);
        case 4:
            i2s_config.sample_size = atoi(argv[3]);
        case 3:
            if (strcmp(argv[2], "mono") == 0)
                i2s_config.protocol = MHB_I2S_PROTOCOL_PCM;
            else if (strcmp(argv[2], "lr_stereo") == 0)
                i2s_config.protocol = MHB_I2S_PROTOCOL_LR_STEREO;

        case 2:
            if (strcmp(argv[1], "slave") == 0)
                 i2s_config.clk_role = MHB_I2S_ROLE_SLAVE;
        case 1:
            if (strcmp(argv[0], "help") == 0) {
                printf("config <rate> <direction (master|slave)> <protocol (i2s_stereo|lr_stereo|mono)> <size bits> "
                       "<channels (1|2)> tx edge (rising|fallinb)> <rx edge (rising|falling)> "
                       "<lr edge (rising|falling)>\n"
                       "Options can stop at any point if the detaults are acceptable for the rest of"
                       "the options.\n");
                return OK;
            }
            i2s_config.sample_rate = atoi(argv[0]);
    }
    return device_mhb_send(dev, &hdr, (const uint8_t *)&i2s_config, sizeof(i2s_config), 0);
}

static int mhb_send_i2s_arm(int argc, char *argv[], struct device *dev)
{
    struct mhb_hdr hdr;
    struct mhb_i2s_control_req i2s_cmd;
    (void)argc;
    (void)argv;

    hdr.addr = MHB_ADDR_I2S;
    hdr.type = MHB_TYPE_I2S_CONTROL_REQ;
    hdr.result = 0;
    i2s_cmd.command = MHB_I2S_COMMAND_ARM;
    return device_mhb_send(dev, &hdr, (const uint8_t *)&i2s_cmd, sizeof(i2s_cmd), 0);
}

static int mhb_send_i2s_disarm(int argc, char *argv[], struct device *dev)
{
    struct mhb_hdr hdr;
    struct mhb_i2s_control_req i2s_cmd;
    (void)argc;
    (void)argv;

    hdr.addr = MHB_ADDR_I2S;
    hdr.type = MHB_TYPE_I2S_CONTROL_REQ;
    hdr.result = 0;
    i2s_cmd.command = MHB_I2S_COMMAND_DISARM;
    return device_mhb_send(dev, &hdr, (const uint8_t *)&i2s_cmd, sizeof(i2s_cmd), 0);
}

static int mhb_send_i2s_trigger(int argc, char *argv[], struct device *dev)
{
    struct mhb_hdr hdr;
    struct mhb_i2s_control_req i2s_cmd;
    (void)argc;
    (void)argv;

    hdr.addr = MHB_ADDR_I2S;
    hdr.type = MHB_TYPE_I2S_CONTROL_REQ;
    hdr.result = 0;
    i2s_cmd.command = MHB_I2S_COMMAND_TRIGGER;
    return device_mhb_send(dev, &hdr, (const uint8_t *)&i2s_cmd, sizeof(i2s_cmd), 0);
}

static int mhb_send_i2s_info(int argc, char *argv[], struct device *dev)
{
    struct mhb_hdr hdr;
    (void)argc;
    (void)argv;

    hdr.addr = MHB_ADDR_I2S;
    hdr.type = MHB_TYPE_I2S_STATUS_REQ;
    hdr.result = 0;
    return device_mhb_send(dev, &hdr, NULL, 0, 0);
}

static int mhb_subcmd_i2s(int argc, char *argv[], struct device *dev)
{
    static const struct command i2s_cmd_tb[] = {
        { "enable",  mhb_send_i2s_enable  },
        { "disable", mhb_send_i2s_disable },
        { "config",  mhb_send_i2s_config  },
        { "arm",     mhb_send_i2s_arm     },
        { "disarm",  mhb_send_i2s_disarm  },
        { "trigger", mhb_send_i2s_trigger },
        { "info",    mhb_send_i2s_info    }
    };

    return mhb_handle_sub_cmd(argc, argv, dev, "i2s", i2s_cmd_tb, ARRAY_SIZE(i2s_cmd_tb));
}

static int mhb_send_pwr_on(int argc, char *argv[], struct device *dev)
{
    (void)argc;
    (void)argv;
    struct device *slave_pwr_ctrl;
    int ret = -ENODEV;

    slave_pwr_ctrl = device_open(DEVICE_TYPE_SLAVE_PWRCTRL_HW, MHB_ADDR_DIAG);
    if (slave_pwr_ctrl != NULL) {
        device_slave_pwrctrl_send_slave_state(slave_pwr_ctrl, 1);
        device_close(slave_pwr_ctrl);
        ret = OK;
    }
    return ret;
}

static int mhb_send_pwr_off(int argc, char *argv[], struct device *dev)
{
    (void)argc;
    (void)argv;
    struct device *slave_pwr_ctrl;
    int ret = -ENODEV;

    slave_pwr_ctrl = device_open(DEVICE_TYPE_SLAVE_PWRCTRL_HW, MHB_ADDR_DIAG);
    if (slave_pwr_ctrl != NULL) {
        device_slave_pwrctrl_send_slave_state(slave_pwr_ctrl, 0);
        device_close(slave_pwr_ctrl);
        ret = OK;
    }
    return ret;
}

static int mhb_subcmd_pwr(int argc, char *argv[], struct device *dev)
{
    static const struct command pwr_cmd_tb[] = {
        { "on",   mhb_send_pwr_on  },
        { "off",  mhb_send_pwr_off }
    };

    return mhb_handle_sub_cmd(argc, argv, dev, "pwr", pwr_cmd_tb, ARRAY_SIZE(pwr_cmd_tb));
}

static int mhb_send_diag_reg_log_apbe_req(int argc, char *argv[], struct device *dev)
{
    (void)argc;
    (void)argv;

    return mhb_send_diag_reg_log_apbe_req_na(dev);
}

static int mhb_send_diag_reg_log_fifo_req(int argc, char *argv[], struct device *dev)
{
    struct mhb_hdr hdr;
    struct mhb_diag_mode_req diag_req;

    (void)argc;
    (void)argv;

    hdr.addr = MHB_ADDR_DIAG;
    hdr.type = MHB_TYPE_DIAG_CONTROL_REQ;
    hdr.result = 0;
    diag_req.mode = MHB_DIAG_CONTROL_REGLOG_FIFO;
    return device_mhb_send(dev, &hdr, (const uint8_t *)&diag_req, sizeof(diag_req), 0);
}

static int mhb_send_diag_reg_log_stack_req(int argc, char *argv[], struct device *dev)
{
    struct mhb_hdr hdr;
    struct mhb_diag_mode_req diag_req;

    (void)argc;
    (void)argv;

    hdr.addr = MHB_ADDR_DIAG;
    hdr.type = MHB_TYPE_DIAG_CONTROL_REQ;
    hdr.result = 0;
    diag_req.mode = MHB_DIAG_CONTROL_REGLOG_STACK;
    return device_mhb_send(dev, &hdr, (const uint8_t *)&diag_req, sizeof(diag_req), 0);
}

static int mhb_subcmd_diag(int argc, char *argv[], struct device *dev)
{
    static const struct command pwr_cmd_tb[] = {
        { "reglog",       mhb_send_diag_reg_log_apbe_req  },
        { "reglog_fifo",  mhb_send_diag_reg_log_fifo_req  },
        { "reglog_stack", mhb_send_diag_reg_log_stack_req },
    };

    return mhb_handle_sub_cmd(argc, argv, dev, "diag", pwr_cmd_tb, ARRAY_SIZE(pwr_cmd_tb));
}

static int display_on(int argc, char *argv[], struct device *dev)
{
    if (g_display) {
        printf("ERROR: already open\n");
        return -EBUSY;
    }

    g_display = device_open(DEVICE_TYPE_DISPLAY_HW, 0);
    if (!g_display) {
        printf("ERROR: failed to open\n");
        return -EIO;
    }

    return device_display_set_state(g_display, DISPLAY_STATE_ON);
}

static int display_off(int argc, char *argv[], struct device *dev)
{
    if (!g_display) {
        printf("ERROR: not opened\n");
        return 0;
    }

    device_display_set_state(g_display, DISPLAY_STATE_OFF);

    device_close(g_display);
    g_display = NULL;

    return 0;
}

static int mhb_subcmd_display(int argc, char *argv[], struct device *dev)
{
    static const struct command display_cmd_tb[] = {
        { "on",   display_on  },
        { "off",  display_off }
    };

    return mhb_handle_sub_cmd(argc, argv, dev, "display", display_cmd_tb, ARRAY_SIZE(display_cmd_tb));
}

static int backlight_open(int argc, char *argv[], struct device *dev)
{
    if (g_backlight) {
        printf("ERROR: already open\n");
        return -EBUSY;
    }

    g_backlight = device_open(DEVICE_TYPE_LIGHTS_HW, 0);
    if (!g_backlight) {
        printf("ERROR: failed to open\n");
        return -EIO;
    }

    return 0;
}

static int backlight_set(int argc, char *argv[], struct device *dev)
{
    if (!g_backlight) {
        printf("ERROR: not opened\n");
        return -ENODEV;
    }

    int brightness = (argc > 0 ? atoi(argv[0]) : 255);
    vdbg("brightness=%d\n", brightness);

    return device_lights_set_brightness(g_backlight, 0, 0, brightness);
}

static int backlight_close(int argc, char *argv[], struct device *dev)
{
    if (!g_backlight) {
        printf("ERROR: not opened\n");
        return 0;
    }

    device_close(g_backlight);
    g_backlight = NULL;

    return 0;
}

static int mhb_subcmd_backlight(int argc, char *argv[], struct device *dev)
{
    static const struct command backlight_cmd_tb[] = {
        { "open", backlight_open },
        { "set", backlight_set },
        { "close", backlight_close }
    };

    return mhb_handle_sub_cmd(argc, argv, dev, "backlight", backlight_cmd_tb, ARRAY_SIZE(backlight_cmd_tb));
}

static int usbtun_on(int argc, char *argv[], struct device *dev)
{
    struct mhb_hdr hdr;
    struct mhb_hsic_control_req req;

    (void)argc;
    (void)argv;

    hdr.addr = MHB_ADDR_HSIC;
    hdr.type = MHB_TYPE_HSIC_CONTROL_REQ;
    hdr.result = 0;
    req.command = MHB_HSIC_COMMAND_START;
    return device_mhb_send(dev, &hdr, (const uint8_t *)&req, sizeof(req), 0);
}

static int usbtun_off(int argc, char *argv[], struct device *dev)
{
    struct mhb_hdr hdr;
    struct mhb_hsic_control_req req;

    (void)argc;
    (void)argv;

    hdr.addr = MHB_ADDR_HSIC;
    hdr.type = MHB_TYPE_HSIC_CONTROL_REQ;
    hdr.result = 0;
    req.command = MHB_HSIC_COMMAND_STOP;
    return device_mhb_send(dev, &hdr, (const uint8_t *)&req, sizeof(req), 0);
}

static int mhb_subcmd_hsic(int argc, char *argv[], struct device *dev)
{
    static const struct command hsic_cmd_tb[] = {
        { "on",   usbtun_on  },
        { "off",  usbtun_off }
    };

    return mhb_handle_sub_cmd(argc, argv, dev, "hsic", hsic_cmd_tb, ARRAY_SIZE(hsic_cmd_tb));
}

int mhb_client_main(int argc, char *argv[])
{
    const char *str;
    const static struct command COMMANDS[] = { \
        /* UniPro */
        { "gear",   mhb_send_gear_req },
        { "status", mhb_send_status_req },
        { "read",   mhb_send_read_attr_req },
        { "write",  mhb_send_write_attr_req },
        { "stats",  mhb_send_stats_req },
        /* Diag */
        { "log",    mhb_send_diag_log_req },
        { "mode",   mhb_send_diag_mode_req },
        { "diag",   mhb_subcmd_diag },
        /* I2S */
        { "i2s",    mhb_subcmd_i2s },
        /* Power control. */
        { "pwr",    mhb_subcmd_pwr },
        /* Display */
        { "display", mhb_subcmd_display },
        /* Backlight */
        { "backlight", mhb_subcmd_backlight },
        /* hsic */
        { "hsic",   mhb_subcmd_hsic },
    };


    if (argc == 1) {
        mhb_print_cmds("usage: " CONFIG_MODS_MHB_CLIENT_PROGNAME, COMMANDS, ARRAY_SIZE(COMMANDS), "\n");
        return -EAGAIN;
    }

    /* Open MHB device, if needed. */
    if (!g_dev) {
        struct device *dev = device_open(DEVICE_TYPE_MHB, MHB_ADDR_DIAG);
        if (!dev) {
            printf("ERROR: failed to open MHB device.\n");
            return -ENOENT;
        }

        device_mhb_register_receiver(dev, MHB_ADDR_UNIPRO, mhb_handle_unipro);
        device_mhb_register_receiver(dev, MHB_ADDR_DIAG, mhb_handle_diag);
        device_mhb_register_receiver(dev, MHB_ADDR_I2S, mhb_handle_i2s_rx);
        g_dev = dev;
    }

    str = argv[1];
    const struct command *command = mhb_find_cmd(str, COMMANDS, ARRAY_SIZE(COMMANDS));
    if (command) {
        printf(CONFIG_MODS_MHB_CLIENT_PROGNAME " %s:\n", str);
        return command->fp(argc - 2, &argv[2], g_dev);
    }

    printf("ERROR: invalid command: %s\n", str);
    return -EINVAL;
}
