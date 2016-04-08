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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <arch/byteorder.h>

#include <nuttx/device.h>
#include <nuttx/util.h>

#include <nuttx/mhb/device_mhb.h>
#include <nuttx/mhb/mhb_protocol.h>

#include <nuttx/unipro/unipro.h>

typedef int (*COMMAND_FP)(int argc, char *argv[], struct device *dev);
struct command {
    const char *str;
    COMMAND_FP fp;
};

static struct device *g_dev;

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
    default:
        printf("ERROR: unknown Unipro event\n");
        return -EINVAL;
    }
}

static int mhb_handle_diag(struct device *dev, struct mhb_hdr *hdr,
               uint8_t *payload, size_t payload_length)
{
    printf("addr=0x%02x, type=0x%02x, result=0x%02x\n",
        hdr->addr, hdr->type, hdr->result);

    const char *res = hdr->result == MHB_RESULT_SUCCESS ? "success" : "failure";

    switch (hdr->type) {
    case MHB_TYPE_DIAG_LOG_RSP:
    case MHB_TYPE_DIAG_LAST_RSP:
        printf((char *)payload);
        return 0;
    case MHB_TYPE_DIAG_MODE_RSP:
        printf("mode %s\n", res);
        return 0;
    case MHB_TYPE_DIAG_LOOP_RSP:
        return 0;
    default:
        printf("ERROR: unknown diag event\n");
        return -EINVAL;
    }
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

static int mhb_send_log_req(int argc, char *argv[], struct device *dev)
{
    struct mhb_hdr hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_DIAG;
    hdr.type = MHB_TYPE_DIAG_LOG_REQ;

    return device_mhb_send(dev, &hdr, NULL, 0, 0);
}

const static struct command COMMANDS[] = { \
    /* UniPro */
    { "gear", mhb_send_gear_req },
    { "status", mhb_send_status_req },
    { "read", mhb_send_read_attr_req },
    { "write", mhb_send_write_attr_req },
    /* Diag */
    { "log", mhb_send_log_req },
};

static void usage(char *argv0)
{
    size_t i;

    printf("usage: %s [", argv0);

    for (i=0; i < ARRAY_SIZE(COMMANDS); i++) {
        printf("%s|", COMMANDS[i].str);
    }

    printf("]\n");
}

int mhb_client_main(int argc, char *argv[])
{
    size_t i;
    const char *str;

    if (argc == 1) {
        usage(CONFIG_MODS_MHB_CLIENT_PROGNAME);
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
        g_dev = dev;
    }

    str = argv[1];
    for (i=0; i < ARRAY_SIZE(COMMANDS); i++) {
        const struct command *command = &COMMANDS[i];
        if (!strcmp(command->str, str)) {
            printf(CONFIG_MODS_MHB_CLIENT_PROGNAME " %s:\n", str);
            return command->fp(argc - 2, &argv[2], g_dev);
        }
    }

    printf("ERROR: invalid command: %s\n", str);
    return -EINVAL;
}
