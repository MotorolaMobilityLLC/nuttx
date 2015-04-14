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
#include <unistd.h>

#ifndef CONFIG_ARCH_BOARD_ARA_SVC
#include "up_switch.h"
#else
#include "svc.h"
#include "tsb_switch.h"
#include "ara_board.h"
#include "interface.h"
#endif

#define DBG_COMP DBG_SVC
#include "up_debug.h"

#if CONFIG_ARCH_BOARD_ARA_SVC

/* ----------------------------------------------------------------------
 * Current code (for configs/ara/svc).
 */

/* These are the largest possible values -- not necessarily the
 * largest supported values. */
#define PWM_GEAR_MAX 7
#define HS_GEAR_MAX 3

#define INVALID -1
enum {
    HELP,
    INIT,
    EXIT,
    LINKTEST,
    LINKSTATUS,
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
    [LINKTEST] = {'l', "linktest",
                  "test UniPro link power mode configuration"},
    [LINKSTATUS] = {'s', "linkstatus", "print UniPro link status bit mask"},
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

static void link_test_usage(int exit_status) {
    printk("svc %s: usage:\n", commands[LINKTEST].longc);
    printk("    -h: print this message and exit\n");
    printk("\n");
    printk("Options for testing a single port:\n");
    printk("    -p <port>   : Port to test, starting from 0.\n");
    printk("    -m <mode>   : UniPro power mode to set <port> to.\n"
           "                  One of \"hs\" or \"pwm\"; defaults to \"hs\".\n");
    printk("    -g <gear>   : pwm or hs gear. For pwm, <gear> is from 1-7.\n"
           "                  For hs, <gear> is from 1-3. Default is 1.\n");
    printk("    -l <lanes>  : Number of lanes for TX/RX. Default is 2.\n");
    printk("    -a          : Use \"auto\" <mode> variant. This alternates\n"
           "                  the link between BURST and SLEEP M-PHY states.\n"
           "                  If not given, the default is non-auto.\n");
    printk("    -s <series> : M-PHY high speed RATE series (\"A\" or \"B\")\n"
           "                  to use. If not given, the default is to\n"
           "                  leave this unchanged.\n");
    printk("\n");
    printk("Options for testing all builtin ports:\n");
    printk("    -t       : \"Torture test\": test all builtin ports, with\n"
           "               various mode, gear, and \"auto\" mode settings.\n"
           "               The output details the test parameters.\n"
           "               If set, other options are ignored.\n");
    exit(exit_status);
}

static int link_test_port_v(uint8_t port,
                            int hs,
                            unsigned int gear,
                            unsigned int nlanes,
                            unsigned int flags,
                            enum unipro_hs_series series,
                            int verbose) {
    struct tsb_switch *sw = ara_svc->sw;
    struct unipro_link_cfg cfg = {
        .upro_hs_ser = hs ? series : UNIPRO_HS_SERIES_UNCHANGED,
    };
    bool auto_variant = flags & UNIPRO_LINK_CFGF_AUTO;

    if (verbose) {
        printf("Port=%d, mode=%s, gear=%d, nlanes=%d, flags=0x%x, series=%s\n",
               port, hs ? "HS" : "PWM", gear, nlanes, flags,
               series == UNIPRO_HS_SERIES_A ? "A" :
               series == UNIPRO_HS_SERIES_B ? "B" :
               series == UNIPRO_HS_SERIES_UNCHANGED ? "<unchanged>" :
               "<UNKNOWN>");
    }

    if (hs) {
        /* TX configuration. */
        cfg.upro_tx_cfg.upro_mode   = (auto_variant ? UNIPRO_FASTAUTO_MODE :
                                       UNIPRO_FAST_MODE);
        cfg.upro_tx_cfg.upro_gear   = gear;
        cfg.upro_tx_cfg.upro_nlanes = nlanes;
        /* RX configuration. */
        cfg.upro_rx_cfg.upro_mode   = (auto_variant ? UNIPRO_FASTAUTO_MODE :
                                       UNIPRO_FAST_MODE);
        cfg.upro_rx_cfg.upro_gear   = gear;
        cfg.upro_rx_cfg.upro_nlanes = nlanes;
        /* Default user data. */
        cfg.upro_user.flags                           = UPRO_PWRF_FC0;
        cfg.upro_user.upro_pwr_fc0_protection_timeout = 0x1FFF;
        /* TX/RX termination. */
        cfg.flags = UPRO_LINKF_TX_TERMINATION | UPRO_LINKF_RX_TERMINATION;
    } else {
        /* TX configuration. */
        cfg.upro_tx_cfg.upro_mode   = (auto_variant ? UNIPRO_SLOWAUTO_MODE :
                                       UNIPRO_SLOW_MODE);
        cfg.upro_tx_cfg.upro_gear   = gear;
        cfg.upro_tx_cfg.upro_nlanes = nlanes;
        /* RX configuration. */
        cfg.upro_rx_cfg.upro_mode   = (auto_variant ? UNIPRO_SLOWAUTO_MODE :
                                       UNIPRO_SLOW_MODE);
        cfg.upro_rx_cfg.upro_gear   = gear;
        cfg.upro_rx_cfg.upro_nlanes = nlanes;
        /* Default user data */
        cfg.upro_user.flags                           = UPRO_PWRF_FC0;
        cfg.upro_user.upro_pwr_fc0_protection_timeout = 0x1FFF;
        /* TX termination only. */
        cfg.flags = UPRO_LINKF_TX_TERMINATION;
    }
    return switch_configure_link(sw, port, &cfg, NULL);
}

static int link_test_port(uint8_t port,
                          int hs,
                          unsigned int gear,
                          unsigned int nlanes,
                          unsigned int flags,
                          enum unipro_hs_series series) {
    return link_test_port_v(port, hs, gear, nlanes, flags, series, 0);
}

static int link_test_torture(unsigned int nlanes) {
    static int fail_hs[SWITCH_PORT_MAX][HS_GEAR_MAX][2][2];
    static int ok_hs[SWITCH_PORT_MAX][HS_GEAR_MAX][2][2];
    static int fail_pwm[SWITCH_PORT_MAX][PWM_GEAR_MAX][2];
    static int ok_pwm[SWITCH_PORT_MAX][PWM_GEAR_MAX][2];

    int rc = 0;
    const int hs_maxgear = 3;   /* max HS gear to _test_. */
    const int pwm_maxgear = 4;  /* max PWM gear to _test_. */
    const unsigned int trials_per_test = 500;
    struct interface *iface;
    int i;

    memset(fail_hs, 0, sizeof(fail_hs));
    memset(ok_hs, 0, sizeof(ok_hs));
    memset(fail_pwm, 0, sizeof(fail_pwm));
    memset(ok_pwm, 0, sizeof(ok_pwm));

    printk("========================================\n");
    printk("Starting link power mode test. Test parameters:\n");
    printk("\t%d trials per power mode.\n", trials_per_test);
    printk("\t%d lanes used for each trial.\n", nlanes);
    if (pwm_maxgear >= 1) {
        printk("\tPWM gears 1-%d tested.\n", pwm_maxgear);
    } else {
        printk("\tPWM gears not tested.\n", pwm_maxgear);
    }
    if (hs_maxgear >= 1) {
        printk("\tHS gears 1-%d tested.\n", hs_maxgear);
        printk("\tHS series A and B tested.\n");
    } else {
        printk("\tHS gears not tested.\n", hs_maxgear);
    }
    interface_foreach(iface, i) {
        int a, g, t, rc2 = 0;
        uint8_t port = (uint8_t)iface->switch_portid;

        if (!interface_is_builtin(iface)) {
            continue;
        }

        printk("Testing interface %s, port %u\n", iface->name, port);
        if (pwm_maxgear > 1) {
            printk("\tTesting PWM gears:");
        }
        for (g = 1; g <= pwm_maxgear; g++) {
            for (a = 0; a <= 1; a++) {
                unsigned int flags = a ? UNIPRO_LINK_CFGF_AUTO : 0;
                printk(" %sPWM-G%u...", a ? "Auto-" : "", g);
                for (t = 0; t < trials_per_test; t++) {
                    rc2 = link_test_port(port, 0, g, nlanes, flags,
                                         UNIPRO_HS_SERIES_UNCHANGED);
                    if (rc2) {
                        fail_pwm[port][g-1][a]++;
                        rc = -1;
                    } else {
                        ok_pwm[port][g-1][a]++;
                    }
                }
                printk("fail=%d/OK=%d",
                       fail_pwm[port][g-1][a], ok_pwm[port][g-1][a]);
            }
        }
        if (pwm_maxgear > 1) {
            printk("\n");
        }

        if (hs_maxgear > 1) {
            printk("\tTesting HS gears:");
        }
        for (g = 1; g <= hs_maxgear; g++) {
            for (a = 0; a <= 1; a++) {
                unsigned int flags = a ? UNIPRO_LINK_CFGF_AUTO : 0;
                printk(" %sHS-G%u-A...", a ? "Auto-" : "", g);
                for (t = 0; t < trials_per_test; t++) {
                    rc2 = link_test_port(port, 1, g, nlanes, flags,
                                         UNIPRO_HS_SERIES_A);
                    if (rc2) {
                        fail_hs[port][g-1][a][0]++;
                        rc = -1;
                    } else {
                        ok_hs[port][g-1][a][0]++;
                    }
                }
                printk("fail=%d/OK=%d",
                       fail_hs[port][g-1][a][0], ok_hs[port][g-1][a][0]);

                printk(" %sHS-G%u-B...", a ? "Auto-" : "", g);
                for (t = 0; t < trials_per_test; t++) {
                    rc2 = link_test_port(port, 1, g, nlanes, flags,
                                         UNIPRO_HS_SERIES_B);
                    if (rc2) {
                        fail_hs[port][g-1][a][1]++;
                        rc = -1;
                    } else {
                        ok_hs[port][g-1][a][1]++;
                    }
                }
                printk("fail=%d/OK=%d",
                       fail_hs[port][g-1][a][1], ok_hs[port][g-1][a][1]);

            }
        }
        if (hs_maxgear > 1) {
            printk("\n");
        }
    }

    printk("Finished power mode test. Results:\n");
    interface_foreach(iface, i) {
        unsigned int port = iface->switch_portid;

        if (!interface_is_builtin(iface)) {
            continue;
        }

        printk("-----------------------------------------------------------\n");
        printk("Interface %s, port %u\n", iface->name, port);
        printk("            Gear:        1       2       3       4       5       6       7\n");
        printk("                   ------- ------- ------- ------- ------- ------- -------\n");
        printk("      PWM fail/OK: %03d/%03d %03d/%03d %03d/%03d %03d/%03d %03d/%03d %03d/%03d %03d/%03d\n",
               fail_pwm[port][0][0], ok_pwm[port][0][0],
               fail_pwm[port][1][0], ok_pwm[port][1][0],
               fail_pwm[port][2][0], ok_pwm[port][2][0],
               fail_pwm[port][3][0], ok_pwm[port][3][0],
               fail_pwm[port][4][0], ok_pwm[port][4][0],
               fail_pwm[port][5][0], ok_pwm[port][5][0],
               fail_pwm[port][6][0], ok_pwm[port][6][0]);
        printk(" Auto PWM fail/OK: %03d/%03d %03d/%03d %03d/%03d %03d/%03d %03d/%03d %03d/%03d %03d/%03d\n",
               fail_pwm[port][0][1], ok_pwm[port][0][1],
               fail_pwm[port][1][1], ok_pwm[port][1][1],
               fail_pwm[port][2][1], ok_pwm[port][2][1],
               fail_pwm[port][3][1], ok_pwm[port][3][1],
               fail_pwm[port][4][1], ok_pwm[port][4][1],
               fail_pwm[port][5][1], ok_pwm[port][5][1],
               fail_pwm[port][6][1], ok_pwm[port][6][1]);
        printk("     HS-A fail/OK: %03d/%03d %03d/%03d %03d/%03d\n",
               fail_hs[port][0][0][0], ok_hs[port][0][0][0],
               fail_hs[port][1][0][0], ok_hs[port][1][0][0],
               fail_hs[port][2][0][0], ok_hs[port][2][0][0]);
        printk("Auto HS-A fail/OK: %03d/%03d %03d/%03d %03d/%03d\n",
               fail_hs[port][0][1][0], ok_hs[port][0][1][0],
               fail_hs[port][1][1][0], ok_hs[port][1][1][0],
               fail_hs[port][2][1][0], ok_hs[port][2][1][0]);
        printk("     HS-B fail/OK: %03d/%03d %03d/%03d %03d/%03d\n",
               fail_hs[port][0][0][1], ok_hs[port][0][0][1],
               fail_hs[port][1][0][1], ok_hs[port][1][0][1],
               fail_hs[port][2][0][1], ok_hs[port][2][0][1]);
        printk("Auto HS-B fail/OK: %03d/%03d %03d/%03d %03d/%03d\n",
               fail_hs[port][0][1][1], ok_hs[port][0][1][1],
               fail_hs[port][1][1][1], ok_hs[port][1][1][1],
               fail_hs[port][2][1][1], ok_hs[port][2][1][1]);
    }

    return rc;
}

static int link_test(int argc, char *argv[]) {
    char **args = argv + 1;
    int c;
    int rc = 0;

    /* Default settings for per-port test */
    int port = -1;
    int hs = 1;
    int gear = 1;
    int nlanes = 2;
    unsigned int auto_flags = 0;
    enum unipro_hs_series series = UNIPRO_HS_SERIES_UNCHANGED;
    /* Whether or not to torture test. */
    int torture = 0;

    const char opts[] = "hp:m:g:l:ats:";

    argc--;
    optind = -1; /* Force NuttX's getopt() to reinitialize. */
    while ((c = getopt(argc, args, opts)) != -1) {
        switch (c) {
        case 'h':
            link_test_usage(EXIT_SUCCESS);
            break;
        case 'p':
            port = strtol(optarg, NULL, 10);
            break;
        case 'm':
            if (!strcmp(optarg, "HS") || !strcmp(optarg, "hs")) {
                hs = 1;
            } else if (!strcmp(optarg, "PWM") || !strcmp(optarg, "pwm")) {
                hs = 0;
            } else {
                printk("Unknown mode %s, must be \"hs\" or \"pwm\"\n.",
                       optarg);
                link_test_usage(EXIT_FAILURE);
            }
            break;
        case 'g':
            gear = strtol(optarg, NULL, 10);
            break;
        case 'l':
            nlanes = strtol(optarg, NULL, 10);
            break;
        case 'a':
            auto_flags |= UNIPRO_LINK_CFGF_AUTO;
            break;
        case 't':
            torture = 1;
            break;
        case 's':
            if (!strcmp(optarg, "A") || !strcmp(optarg, "a")) {
                series = UNIPRO_HS_SERIES_A;
            } else if (!strcmp(optarg, "B") || !strcmp(optarg, "b")) {
                series = UNIPRO_HS_SERIES_B;
            } else {
                printk("Unknown rate series %s, must be \"A\" or \"B\".\n",
                       optarg);
                link_test_usage(EXIT_FAILURE);
            }
            break;
        case '?':
        default:
            printf("Unrecognized argument '%c'.\n", (char)c);
            link_test_usage(EXIT_FAILURE);
        }
    }

    if (port == -1 && !torture) {
        printk("Must specify one of -p or -t.\n");
        link_test_usage(EXIT_FAILURE);
    }
    if (nlanes <= 0) {
        printk("Number of lanes %d must be positive.\n", nlanes);
        link_test_usage(EXIT_FAILURE);
    }

    if (torture) {
        rc = link_test_torture((unsigned int)nlanes);
    } else {
        if (port < 0 || port > SWITCH_PORT_MAX) {
            printk("Invalid port %d, must be between %d and %d.\n",
                   port, 0, SWITCH_PORT_MAX - 1);
            link_test_usage(EXIT_FAILURE);
        }
        if (gear < 0 || (hs && gear > HS_GEAR_MAX) || gear > PWM_GEAR_MAX) {
            printk("Invalid gear %d.\n", gear);
            link_test_usage(EXIT_FAILURE);
        }
        if (nlanes < 0 || nlanes > PA_CONN_RX_DATA_LANES_NR ||
            nlanes > PA_CONN_RX_DATA_LANES_NR) {
            printk("Invalid number of lanes %d.\n", nlanes);
            link_test_usage(EXIT_FAILURE);
        }
        rc = link_test_port_v((uint8_t)port, hs, (unsigned int)gear,
                              (unsigned int)nlanes, auto_flags, series,
                              1);
    }

    return rc;
}

static int link_status(int argc, char *argv[]) {
    uint32_t link_status;
    struct tsb_switch *sw = ara_svc->sw;
    int rc;

    if (argc != 2) {
        printk("Ignoring unexpected arguments.\n");
    }

    rc = switch_internal_getattr(sw, SWSTA, &link_status);
    if (rc) {
        printk("Error: could not read link status: %d.\n", rc);
    } else {
        printk("Link status: 0x%x\n", link_status);
    }
    return rc;
}

static int ara_svc_main(int argc, char *argv[]) {
    /* Current main(), for configs/ara/svc (BDB1B, BDB2A, spiral 2
     * modules, etc.). */
    int rc = 0;
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
    case LINKTEST:
        rc = link_test(argc, argv);
        break;
    case LINKSTATUS:
        rc = link_status(argc, argv);
        break;
    default:
        usage(EXIT_FAILURE);
    }

    return rc;
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
