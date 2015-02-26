/*
 * Copyright (c) 2014 Google Inc.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef CONFIG_ARCH_BOARD_ARA_SVC
#include "up_switch.h"
#else
#include "svc.h"
#endif

#define DBG_COMP DBG_SVC
#include "up_debug.h"

#if CONFIG_ARCH_BOARD_ARA_SVC

/* ----------------------------------------------------------------------
 * Current code (for configs/ara/svc).
 */

#define INVALID -1
enum {
    HELP,
    INIT,
    EXIT,
    MAX_CMD,
};

struct command {
    const char shortc;
    const char *longc;
    const char *help;
};

static const struct command commands[] = {
    [HELP] = {'h', "help", "print this usage and exit"},
    [INIT] = {'i', "init", "initialize switch and SVC"},
    [EXIT] = {'e', "exit", "exit/de-initialize SVC"},
};

static void usage(int exit_status) {
    int i;
    printk("svc: usage:\n");
    for (i = 0; i < MAX_CMD; i++) {
        printk("    svc [%c|%s] : %s\n",
               commands[i].shortc, commands[i].longc, commands[i].help);
    }
    exit(exit_status);
}

static int ara_svc_main(int argc, char *argv[]) {
    /* Current main(), for configs/ara/svc (BDB1B, BDB2A, spiral 2
     * modules, etc.). */
    int i;
    int cmd = INVALID;
    const char *cmd_str;

    /* Parse arguments. */
    if (argc < 2) {
        usage(EXIT_FAILURE);
    }
    cmd_str = argv[1];
    for (i = 0; i < MAX_CMD; i++) {
        if (!strcmp(cmd_str, commands[i].longc)) {
            cmd = i;
            break;
        } else if (strlen(cmd_str) == 1 &&
                   cmd_str[0] == commands[i].shortc) {
            cmd = i;
            break;
        }
    }
    if (cmd == INVALID) {
        usage(EXIT_FAILURE);
    }

    /* Run the command. */
    switch (cmd) {
    case HELP:
        usage(EXIT_SUCCESS);
    case INIT:
        svc_init();
        break;
    case EXIT:
        svc_exit();
        break;
    default:
        usage(EXIT_FAILURE);
    }

    return 0;
}
#endif

/* ----------------------------------------------------------------------
 * Actual main(), with legacy fallback for older boards / configs.
 */

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int svc_main(int argc, char *argv[])
#endif
{
#ifdef CONFIG_ARCH_BOARD_ARA_SVC
    return ara_svc_main(argc, argv);
#else
    /* Legacy main(); for configs/{endo,bdb}/ */
    int state = 0;

    if (argc < 2)
        return ERROR;
    state = strtol(argv[1], NULL, 10);
    switch_control(state);
    return 0;
#endif
}
