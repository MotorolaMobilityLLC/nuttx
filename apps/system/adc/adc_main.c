/*
 * Copyright (c) 2017 Motorola Mobility, LLC.
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
#include <debug.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/ioctl.h>

#include <nuttx/analog/adc.h>
#include <nuttx/arch.h>
#include <nuttx/util.h>

typedef int (*COMMAND_FP)(int argc, const char *argv[]);

struct command {
    const char *str;
    const COMMAND_FP fp;
};

static const struct command *find_cmd(const char *str,
                                      const struct command *cmds,
                                      size_t num_cmds)
{
    while (num_cmds--) {
        if (!strcmp(cmds->str, str)) {
            return cmds;
        }
        cmds++;
    }

    return NULL;
}

static void print_cmds(const char *header,
                       const struct command *cmds,
                       size_t num_cmds,
                       const char *trailer)
{
    if (header) {
        printf(header);
    }

    while (num_cmds--) {
        printf(" %s %c", cmds->str, num_cmds > 0 ? '|' : ' ');
        cmds++;
    }

    if (trailer) {
        printf(trailer);
    }
}

static int execute_cmd(int argc,
                       const char *argv[],
                       const struct command *cmds,
                       size_t num_cmds)
{
    const struct command *cmd;
    int ret = OK;

    cmd = find_cmd(argv[0], cmds, num_cmds);
    if (!cmd) {
        printf("ERROR: Unknown command: %s\n", argv[0]);
        print_cmds("", cmds, num_cmds, "\n");
        return -EINVAL;
    }

    ret = cmd->fp(argc-1, argv+1);
    return ret;
}

static int _adc_get(const char *devpath, struct adc_msg_s *adc_msg, size_t num_msgs)
{
    int fd;
    size_t i;
    size_t retries = 0;
    int ret;

    fd = open(devpath, O_RDONLY|O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "ERROR: open failed: %d\n", errno);
        return -ENODEV;
    }

    /* Start the ADC conversion. */
    ret = ioctl(fd, ANIOC_TRIGGER, 0);
    if (ret < 0) {
        fprintf(stderr, "ERROR: ioctl failed: %d\n", errno);
        close(fd);
        return -EIO;
    }

    /* HACK: Give the ADC some time to receive all the values.
     * If we call read() too soon, it will mask the ADC interrupts and we will
     * lose interrupts. */
    up_udelay(50);

    /* The ADC device only allows a single adc_msg read at a time. */
    i = 0;
    while (i < num_msgs && retries < 3) {

        ret = read(fd, &adc_msg[i], sizeof(*adc_msg));
        vdbg("read: ret=%d, errno=%d\n", ret, errno);
        if (ret != sizeof(*adc_msg)) {
            if (errno == EAGAIN) {
                retries++;
                continue;
            }

            vdbg("read failed: ret=%d, errno=%d\n", ret, errno);
            ret = errno;
            break;
        } else {
            vdbg("chan=%d, data=%d\n", adc_msg[i].am_channel, adc_msg[i].am_data);
            i++;
            retries = 0;
            ret = OK;
        }
    }

    if (fd >= 0) {
        close(fd);
    }

    return i ? i : ret;
}


static int adc_get(int argc, const char *argv[])
{
    int interface;
    int count;
    int delay;
    char devpath[16];
    struct adc_msg_s adc_msgs[16];
    size_t c;
    size_t i;
    int ret = OK;

    interface = (argc > 0 ? atoi(argv[0]) : 0);
    count = (argc > 1 ? atoi(argv[1]) : 1);
    delay = (argc > 2 ? atoi(argv[2]) : 1000);

    snprintf(devpath, ARRAY_SIZE(devpath), "/dev/adc%d", interface);
    vdbg("get: interface=%d, devpath=%s\n", interface, devpath);

    for (c = 0; c < count; c++) {
        ret = _adc_get(devpath, adc_msgs, ARRAY_SIZE(adc_msgs));
        vdbg("ret=%d\n", ret);

        /* Success, print values. */
        if (ret > 0) {
            for (i = 0; i < ret; i++) {
                printf("channel=%d, value=%d (0x%08x)\n",
                  adc_msgs[i].am_channel, adc_msgs[i].am_data, adc_msgs[i].am_data);
            }
        }

        usleep(delay*1000);
    }

    return ret;
}

static const struct command COMMANDS[] = { \
    {"g", adc_get},
    {"get", adc_get},
};

int adc_main(int argc, const char *argv[])
{
    int ret;

    /* Eat the argument that got us here. */
    argc -= 1;
    argv += 1;

    if (argc == 0) {
        print_cmds("usage: " CONFIG_ADCTOOL_APP_PROGNAME,
                   COMMANDS, ARRAY_SIZE(COMMANDS), "\n");
        ret = -EAGAIN;
    } else {
        ret = execute_cmd(argc, argv, COMMANDS, ARRAY_SIZE(COMMANDS));
    }

    return ret;
}
