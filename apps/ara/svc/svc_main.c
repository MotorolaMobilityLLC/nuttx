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
#include "attr_names.h"
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
    DME_IO,
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
    [DME_IO]  = {'d', "dme", "get/set DME attributes"},
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

static void dme_io_usage(void) {
    printk("svc %s <r|w> [options]: usage:\n", commands[DME_IO].longc);
    printk("    Common options:\n");
    printk("        -h: print this message and exit\n");
    printk("\n");
    printk("    Options for reading an attribute or group of attributes:\n");
    printk("    svc %s r [-a <attrs>] [-s <sel>] [-p <port>] [-P]:\n",
           commands[DME_IO].longc);
    printk("\n");
    printk("        -a <attrs>: attribute (in hexadecimal) to read, or one of:\n");
    printk("                      \"L1\" (PHY layer),\n");
    printk("                      \"L1.5\" (PHY adapter layer),\n");
    printk("                      \"L2\" (link layer),\n");
    printk("                      \"L3\" (network layer),\n");
    printk("                      \"L4\" (transport layer),\n");
    printk("                      \"DME\" (DME),\n");
    printk("                      \"TSB\" (Toshiba-specific attributes),\n");
    printk("                      \"all\" (all of the above).\n");
    printk("                    If missing, default is \"all\".\n");
    printk("                    If <attrs> is \"L4\", -P is implied.\n");
    printk("        -s <sel>: attribute selector index (default is 0)\n");
    printk("        -p <port>: port to read attribute on (default is 0)\n");
    printk("        -P: if present, do a peer (instead of switch local) read\n");
    printk("\n");
    printk("    Options for writing an attribute:\n");
    printk("    svc %s w -a <attr> [-s <sel>] [-p <port>] [-P] <value>:\n",
           commands[DME_IO].longc);
    printk("\n");
    printk("        -a <attr>: attribute (in hexadecimal) to write.\n");
    printk("        -s <sel>: attribute selector index (default is 0)\n");
    printk("        -p <port>: port to read attribute on (default is 0)\n");
    printk("        -P: if present, do a peer (instead of switch local) write\n");
}

static int dme_io_dump(struct tsb_switch *sw, uint8_t port,
                       const char *attr_str, uint16_t attr,
                       uint16_t selector, int peer) {
    int rc;
    uint32_t val;

    if (peer) {
        rc = switch_dme_peer_get(sw, port, attr, selector, &val);
    } else {
        rc = switch_dme_get(sw, port, attr, selector, &val);
    }

    if (rc) {
        if (attr_str) {
            printk("Error: can't read attribute %s (0x%x): %d\n",
                   attr_str, attr, rc);
        } else {
            printk("Error: can't read attribute 0x%x: rc=%d\n", attr, rc);
        }
        return -EIO;
    }
    if (attr_str) {
        printk("Port=%d, peer=%s, sel=%d, %s (0x%x) = 0x%x (%u)\n",
               port,
               peer ? "yes" : "no",
               selector,
               attr_str,
               attr,
               val,
               val);
    } else {
        printk("Port=%d, peer=%s, sel=%d, 0x%x = 0x%x (%u)\n",
               port,
               peer ? "yes" : "no",
               selector,
               attr,
               val,
               val);
    }
    return 0;
}

static int dme_io_set(struct tsb_switch *sw, uint8_t port,
                      char *attr_str, uint16_t attr, uint32_t val,
                      uint16_t selector, int peer) {
    int rc;

    if (peer) {
        rc = switch_dme_peer_set(sw, port, attr, selector, val);
    } else {
        rc = switch_dme_set(sw, port, attr, selector, val);
    }

    if (rc) {
        if (attr_str) {
            printk("Error: can't set attribute %s (0x%x): rc=%d\n",
                   attr_str, attr, rc);
        } else {
            printk("Error: can't set attribute 0x%x: rc=%d\n",
                   attr, rc);
        }
        return -EIO;
    }
    if (attr_str) {
        printk("Port=%d, peer=%s, sel=%d, set %s (0x%x) = 0x%x (%u)\n",
               port,
               peer ? "yes" : "no",
               selector,
               attr_str,
               attr,
               val,
               val);
    } else {
        printk("Port=%d, peer=%s, sel=%d, set 0x%x = 0x%x (%u)\n",
               port,
               peer ? "yes" : "no",
               selector,
               attr,
               val,
               val);
    }
    return 0;
}

static int dme_io(int argc, char *argv[]) {
    enum {
        NONE, ALL, ONE, L1, L1_5, L2, L3, L4, L5, LD, TSB
    };
    int which_attrs = NONE;
    const char opts[] = "a:s:p:Ph"; /* TODO: add an -S option for switch number
                                     * when multiple switches are available
                                     * ... */
    struct tsb_switch *sw = ara_svc->sw; /* and get the right switch here. */
    uint16_t selector = 0;
    int peer = 0;
    char *end;
    uint8_t port = 0;
    uint16_t attr = 0xbeef;
    int attr_set = 0;
    uint32_t val = 0xdeadbeef;
    int val_set = 1;
    char **args;
    int read;
    int c;

    if (argc <= 2) {
        printk("BUG: invalid argument specification.\n");
        return -EINVAL;
    }

    /*
     * Decide whether this is a read or a write.
     */
    if (!strcmp(argv[2], "r") || !strcmp(argv[2], "R")) {
        read = 1;
    } else if (!strcmp(argv[2], "w") || !strcmp(argv[2], "W")) {
        read = 0;
    } else if (!strcmp(argv[2], "-h")) {
        dme_io_usage();
        return EXIT_SUCCESS;
    } else {
        printk("Must specify \"r\" or \"w\".\n\n");
        dme_io_usage();
        return EXIT_FAILURE;
    }

    /*
     * Parse the other command line options.
     */
    optind = -1; /* Force NuttX's getopt() to re-initialize. */
    argc -= 2;   /* skip over the "svc d" part */
    args = argv + 2;
    while ((c = getopt(argc, args, opts)) != -1) {
        switch (c) {
        case 'a':
            if (!strcmp(optarg, "all")) {
                which_attrs = ALL;
            } else if (!strcmp(optarg, "l1") || !strcmp(optarg, "L1")) {
                which_attrs = L1;
            } else if (!strcmp(optarg, "l1.5") || !strcmp(optarg, "L1.5")) {
                which_attrs = L1_5;
            } else if (!strcmp(optarg, "l2") || !strcmp(optarg, "L2")) {
                which_attrs = L2;
            } else if (!strcmp(optarg, "l3") || !strcmp(optarg, "L3")) {
                which_attrs = L3;
            } else if (!strcmp(optarg, "l4") || !strcmp(optarg, "L4")) {
                which_attrs = L4;
                peer = 1;
            } else if (!strcmp(optarg, "dme") || !strcmp(optarg, "DME")) {
                which_attrs = LD;
            } else if (!strcmp(optarg, "tsb") || !strcmp(optarg, "TSB")) {
                which_attrs = TSB;
            } else {
                end = NULL;
                attr = strtoul(optarg, &end, 16);
                if (*end) {
                    printk("-a %s invalid: must be one of: \"all\", \"L1\", "
                           "\"L2\", \"L3\", \"L4\", \"DME\", or a hexadecimal "
                           "attribute\n\n", optarg);
                    dme_io_usage();
                    return EXIT_FAILURE;
                }
                which_attrs = ONE;
                attr_set = 1;
            }
            break;
        case 's':
            end = NULL;
            selector = strtoul(optarg, &end, 10);
            if (*end) {
                printk("-s %s invalid: must specify a decimal selector "
                       "index\n\n", optarg);
                dme_io_usage();
                return EXIT_FAILURE;
            }
            break;
        case 'p':
            end = NULL;
            port = strtoul(optarg, &end, 10);
            if (*end) {
                printk("-p %s invalid: must specify a decimal port", optarg);
                dme_io_usage();
                return EXIT_FAILURE;
            }
            break;
        case 'P':
            peer = 1;
            break;
        case 'h':
            dme_io_usage();
            return EXIT_SUCCESS;
        default:
        case '?':
            printk("Unrecognized argument.\n");
            dme_io_usage();
            return EXIT_FAILURE;
        }
    }

    /*
     * Set some default values, validate options, and parse any arguments.
     */
    if (which_attrs == NONE) {
        if (read) {
            which_attrs = ALL;
        } else if (which_attrs != ONE) {
            printk("Must specify -a with attribute when writing.\n");
            return EXIT_FAILURE;
        }
    }
    if (which_attrs != ONE && !read) {
        printk("Only one attribute can be written at a time.\n\n");
        dme_io_usage();
        return EXIT_FAILURE;
    }
    if (which_attrs == L4 && !peer) {
        printk("No L4 attributes on switch ports (missing -P?).\n\n");
        dme_io_usage();
        return EXIT_FAILURE;
    }
    if (!read) {
        if (!attr_set) {
            printk("Must specify -a when writing.\n\n");
            dme_io_usage();
            return EXIT_FAILURE;
        }
        if (optind >= argc) {
            printk("Must specify value to write.\n\n");
            dme_io_usage();
            return EXIT_FAILURE;
        } else {
            end = NULL;
            val = strtoul(args[optind], &end, 10);
            if (*end) {
                printk("Invalid value to write: %s.\n\n", args[optind]);
                dme_io_usage();
                return EXIT_FAILURE;
            }
            val_set = 1;
        }
    }

    /*
     * Do the I/O.
     */
    if (read) {
        int rc;
        int i;
        const struct attr_name_group *attr_name_groups[7] = {0};
        int n_attr_name_groups;
        switch (which_attrs) {
        case ONE:
            ASSERT(attr_set);
            return dme_io_dump(sw, port, attr_get_name(attr), attr, selector,
                               peer);
        case ALL:
            attr_name_groups[0] = &unipro_l1_attr_group;
            attr_name_groups[1] = &unipro_l1_5_attr_group;
            attr_name_groups[2] = &unipro_l2_attr_group;
            attr_name_groups[3] = &unipro_l3_attr_group;
            if (peer) {
                attr_name_groups[4] = &unipro_l4_attr_group;
                attr_name_groups[5] = &unipro_dme_attr_group;
                attr_name_groups[6] = &unipro_tsb_attr_group;
                n_attr_name_groups = 7;
            } else {
                attr_name_groups[4] = &unipro_dme_attr_group;
                attr_name_groups[5] = &unipro_tsb_attr_group;
                n_attr_name_groups = 6;
            }
            break;
        case L1:
            attr_name_groups[0] = &unipro_l1_attr_group;
            n_attr_name_groups = 1;
            break;
        case L1_5:
            attr_name_groups[0] = &unipro_l1_5_attr_group;
            n_attr_name_groups = 1;
            break;
        case L2:
            attr_name_groups[0] = &unipro_l2_attr_group;
            n_attr_name_groups = 1;
            break;
        case L3:
            attr_name_groups[0] = &unipro_l3_attr_group;
            n_attr_name_groups = 1;
            break;
        case L4:
            attr_name_groups[0] = &unipro_l4_attr_group;
            n_attr_name_groups = 1;
            break;
        case LD:
            attr_name_groups[0] = &unipro_dme_attr_group;
            n_attr_name_groups = 1;
            break;
        case TSB:
            attr_name_groups[0] = &unipro_tsb_attr_group;
            n_attr_name_groups = 1;
            break;
        default:
        case NONE:
            printk("BUG: %s: can't happen.\n", __func__);
            return EXIT_FAILURE;
        }
        for (i = 0; i < n_attr_name_groups; i++) {
            const struct attr_name *ans = attr_name_groups[i]->attr_names;
            while (ans->name) {
                rc = (dme_io_dump(sw, port,
                                  ans->name, ans->attr,
                                  selector, peer) ||
                      rc);
                ans++;
            }
        }
        return rc;
    } else {
        ASSERT(attr_set);
        ASSERT(val_set);
        return dme_io_set(sw, port, attr_get_name(attr), attr, val, selector,
                          peer);
    }
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
    case DME_IO:
        rc = dme_io(argc, argv);
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
