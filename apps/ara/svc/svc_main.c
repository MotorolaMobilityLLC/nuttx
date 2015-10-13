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

#define DBG_COMP ARADBG_SVC
#include <ara_debug.h>

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <nuttx/util.h>
#include <nuttx/unipro/unipro.h>

#include "svc.h"
#include "tsb_switch.h"
#include "ara_board.h"
#include "interface.h"
#include "attr_names.h"

/* These are the largest possible values -- not necessarily the
 * largest supported values. */
#define PWM_GEAR_MAX 7
#define HS_GEAR_MAX 3

#define INVALID -1
enum {
    HELP,
    START,
    STOP,
    LINKTEST,
    LINKSTATUS,
    ROUTINGTABLE,
    DME_IO,
    TESTFEATURE,
    QOS,
    MAX_CMD,
};

struct command {
    const char shortc;
    const char *longc;
    const char *help;
};

static const struct command commands[] = {
    [HELP] = {'h', "help", "print this usage and exit"},
    [START] = {'i', "start", "start svcd"},
    [STOP] = {'e', "stop", "stop svcd"},
    [LINKTEST] = {'l', "linktest",
                  "test UniPro link power mode configuration"},
    [LINKSTATUS] = {'s', "linkstatus", "print UniPro link status bit mask"},
    [ROUTINGTABLE] = {'r', "routingtable", "dump Switch routing table"},
    [DME_IO]  = {'d', "dme", "get/set DME attributes"},
    [TESTFEATURE] = {'t', "testfeature", "UniPro test feature"},
    [QOS] = {'q', "qos", "Quality of Service control"},
};

static void usage(int exit_status) {
    int i;
    printf("svc: usage:\n");
    for (i = 0; i < MAX_CMD; i++) {
        printf("    svc [%c|%s] : %s\n",
               commands[i].shortc, commands[i].longc, commands[i].help);
    }
    exit(exit_status);
}

static void link_test_usage(int exit_status) {
    printf("svc %s: usage:\n", commands[LINKTEST].longc);
    printf("    -h: print this message and exit\n");
    printf("\n");
    printf("Options for testing a single port:\n");
    printf("    -p <port>   : Port to test, starting from 0.\n");
    printf("    -i <interface>: Interface to read attribute on (e.g. \"apb1\", etc.)\n");
    printf("                    If set, overrides -p.\n");
    printf("    -m <mode>   : UniPro power mode to set <port> to.\n"
           "                  One of \"hs\" or \"pwm\"; defaults to \"hs\".\n");
    printf("    -g <gear>   : pwm or hs gear. For pwm, <gear> is from 1-7.\n"
           "                  For hs, <gear> is from 1-3. Default is 1.\n");
    printf("    -l <lanes>  : Number of lanes for TX/RX. Default is 2.\n");
    printf("    -a          : Use \"auto\" <mode> variant. This alternates\n"
           "                  the link between BURST and SLEEP M-PHY states.\n"
           "                  If not given, the default is non-auto.\n");
    printf("    -s <series> : M-PHY high speed RATE series (\"A\" or \"B\")\n"
           "                  to use. If not given, the default is to\n"
           "                  leave this unchanged.\n");
    printf("\n");
    printf("Options for testing all builtin ports:\n");
    printf("    -t       : \"Torture test\": test all builtin ports, with\n"
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
    struct tsb_switch *sw = svc->sw;
    struct unipro_link_cfg cfg = {
        .upro_hs_ser = hs ? series : UNIPRO_HS_SERIES_UNCHANGED,
    };
    bool auto_variant = flags & UNIPRO_LINK_CFGF_AUTO;

    if (!sw) {
        return -ENODEV;
    }

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

    if (!interface_get_count()) {
        return -ENODEV;
    }

    memset(fail_hs, 0, sizeof(fail_hs));
    memset(ok_hs, 0, sizeof(ok_hs));
    memset(fail_pwm, 0, sizeof(fail_pwm));
    memset(ok_pwm, 0, sizeof(ok_pwm));

    printf("========================================\n");
    printf("Starting link power mode test. Test parameters:\n");
    printf("\t%d trials per power mode.\n", trials_per_test);
    printf("\t%d lanes used for each trial.\n", nlanes);
    if (pwm_maxgear >= 1) {
        printf("\tPWM gears 1-%d tested.\n", pwm_maxgear);
    } else {
        printf("\tPWM gears not tested.\n");
    }
    if (hs_maxgear >= 1) {
        printf("\tHS gears 1-%d tested.\n", hs_maxgear);
        printf("\tHS series A and B tested.\n");
    } else {
        printf("\tHS gears not tested.\n");
    }
    interface_foreach(iface, i) {
        int a, g, t, rc2 = 0;
        uint8_t port;

        /* The test is only for builtin bridges */
        if (!interface_is_builtin(iface)) {
            continue;
        }

        /* Check if there is a port on the interface */
        if (iface->switch_portid == INVALID_PORT) {
            continue;
        }

        port = (uint8_t) iface->switch_portid;

        printf("Testing interface %s, port %u\n", iface->name, port);
        if (pwm_maxgear > 1) {
            printf("\tTesting PWM gears:");
        }
        for (g = 1; g <= pwm_maxgear; g++) {
            for (a = 0; a <= 1; a++) {
                unsigned int flags = a ? UNIPRO_LINK_CFGF_AUTO : 0;
                printf(" %sPWM-G%u...", a ? "Auto-" : "", g);
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
                printf("fail=%d/OK=%d",
                       fail_pwm[port][g-1][a], ok_pwm[port][g-1][a]);
            }
        }
        if (pwm_maxgear > 1) {
            printf("\n");
        }

        if (hs_maxgear > 1) {
            printf("\tTesting HS gears:");
        }
        for (g = 1; g <= hs_maxgear; g++) {
            for (a = 0; a <= 1; a++) {
                unsigned int flags = a ? UNIPRO_LINK_CFGF_AUTO : 0;
                printf(" %sHS-G%u-A...", a ? "Auto-" : "", g);
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
                printf("fail=%d/OK=%d",
                       fail_hs[port][g-1][a][0], ok_hs[port][g-1][a][0]);

                printf(" %sHS-G%u-B...", a ? "Auto-" : "", g);
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
                printf("fail=%d/OK=%d",
                       fail_hs[port][g-1][a][1], ok_hs[port][g-1][a][1]);

            }
        }
        if (hs_maxgear > 1) {
            printf("\n");
        }
    }

    printf("Finished power mode test. Results:\n");
    interface_foreach(iface, i) {
        unsigned int port;

        /* The test is only for builtin bridges */
        if (!interface_is_builtin(iface)) {
            continue;
        }

        /* Check if there is a port on the interface */
        if (iface->switch_portid == INVALID_PORT) {
            continue;
        }

        port = iface->switch_portid;

        printf("-----------------------------------------------------------\n");
        printf("Interface %s, port %u\n", iface->name, port);
        printf("            Gear:        1       2       3       4       5       6       7\n");
        printf("                   ------- ------- ------- ------- ------- ------- -------\n");
        printf("      PWM fail/OK: %03d/%03d %03d/%03d %03d/%03d %03d/%03d %03d/%03d %03d/%03d %03d/%03d\n",
               fail_pwm[port][0][0], ok_pwm[port][0][0],
               fail_pwm[port][1][0], ok_pwm[port][1][0],
               fail_pwm[port][2][0], ok_pwm[port][2][0],
               fail_pwm[port][3][0], ok_pwm[port][3][0],
               fail_pwm[port][4][0], ok_pwm[port][4][0],
               fail_pwm[port][5][0], ok_pwm[port][5][0],
               fail_pwm[port][6][0], ok_pwm[port][6][0]);
        printf(" Auto PWM fail/OK: %03d/%03d %03d/%03d %03d/%03d %03d/%03d %03d/%03d %03d/%03d %03d/%03d\n",
               fail_pwm[port][0][1], ok_pwm[port][0][1],
               fail_pwm[port][1][1], ok_pwm[port][1][1],
               fail_pwm[port][2][1], ok_pwm[port][2][1],
               fail_pwm[port][3][1], ok_pwm[port][3][1],
               fail_pwm[port][4][1], ok_pwm[port][4][1],
               fail_pwm[port][5][1], ok_pwm[port][5][1],
               fail_pwm[port][6][1], ok_pwm[port][6][1]);
        printf("     HS-A fail/OK: %03d/%03d %03d/%03d %03d/%03d\n",
               fail_hs[port][0][0][0], ok_hs[port][0][0][0],
               fail_hs[port][1][0][0], ok_hs[port][1][0][0],
               fail_hs[port][2][0][0], ok_hs[port][2][0][0]);
        printf("Auto HS-A fail/OK: %03d/%03d %03d/%03d %03d/%03d\n",
               fail_hs[port][0][1][0], ok_hs[port][0][1][0],
               fail_hs[port][1][1][0], ok_hs[port][1][1][0],
               fail_hs[port][2][1][0], ok_hs[port][2][1][0]);
        printf("     HS-B fail/OK: %03d/%03d %03d/%03d %03d/%03d\n",
               fail_hs[port][0][0][1], ok_hs[port][0][0][1],
               fail_hs[port][1][0][1], ok_hs[port][1][0][1],
               fail_hs[port][2][0][1], ok_hs[port][2][0][1]);
        printf("Auto HS-B fail/OK: %03d/%03d %03d/%03d %03d/%03d\n",
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
    const char *iface_name = NULL;
    int port = -1;
    int hs = 1;
    int gear = 1;
    int nlanes = 2;
    unsigned int auto_flags = 0;
    enum unipro_hs_series series = UNIPRO_HS_SERIES_UNCHANGED;
    /* Whether or not to torture test. */
    int torture = 0;

    const char opts[] = "hi:p:m:g:l:ats:";

    argc--;
    optind = -1; /* Force NuttX's getopt() to reinitialize. */
    while ((c = getopt(argc, args, opts)) != -1) {
        switch (c) {
        case 'h':
            link_test_usage(EXIT_SUCCESS);
            break;
        case 'i':
            iface_name = optarg;
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
                printf("Unknown mode %s, must be \"hs\" or \"pwm\"\n.",
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
                printf("Unknown rate series %s, must be \"A\" or \"B\".\n",
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

    if (iface_name) {
        struct interface *iface = interface_get_by_name(iface_name);
        if (iface) {
            port = iface->switch_portid;
        } else {
            printf("Invalid interface %s\n", iface_name);
            link_test_usage(EXIT_FAILURE);
        }
    }

    if (port == -1 && !torture) {
        printf("Must specify one of -p or -t.\n");
        link_test_usage(EXIT_FAILURE);
    }

    if (port == INVALID_PORT) {
        printf("No port present on interface\n");
        link_test_usage(EXIT_FAILURE);
    }

    if (nlanes <= 0) {
        printf("Number of lanes %d must be positive.\n", nlanes);
        link_test_usage(EXIT_FAILURE);
    }

    if (torture) {
        rc = link_test_torture((unsigned int)nlanes);
    } else {
        if (port < 0 || port > SWITCH_PORT_MAX) {
            printf("Invalid port %d, must be between %d and %d.\n",
                   port, 0, SWITCH_PORT_MAX - 1);
            link_test_usage(EXIT_FAILURE);
        }
        if (gear < 0 || (hs && gear > HS_GEAR_MAX) || gear > PWM_GEAR_MAX) {
            printf("Invalid gear %d.\n", gear);
            link_test_usage(EXIT_FAILURE);
        }
        if (nlanes < 0 || nlanes > PA_CONN_RX_DATA_LANES_NR ||
            nlanes > PA_CONN_RX_DATA_LANES_NR) {
            printf("Invalid number of lanes %d.\n", nlanes);
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
    struct tsb_switch *sw = svc->sw;
    int rc;

    if (!sw) {
        return -ENODEV;
    }

    if (argc != 2) {
        printf("Ignoring unexpected arguments.\n");
    }

    rc = switch_internal_getattr(sw, SWSTA, &link_status);
    if (rc) {
        printf("Error: could not read link status: %d.\n", rc);
    } else {
        printf("Link status: 0x%x\n", link_status);
    }
    return rc;
}

static int dump_routing_table(int argc, char *argv[])
{
    struct tsb_switch *sw = svc->sw;
    int rc;

    if (!sw) {
        return -ENODEV;
    }

    rc = switch_dump_routing_table(sw);
    if (rc) {
        printf("Error: could not dump Switch routing table: %d.\n", rc);
    }

    return rc;
}

static void dme_io_usage(void) {
    printf("svc %s <r|w> [options]: usage:\n", commands[DME_IO].longc);
    printf("    Common options:\n");
    printf("        -h: print this message and exit\n");
    printf("\n");
    printf("    Options for reading an attribute or group of attributes:\n");
    printf("    svc %s r [-a <attrs>] [-s <sel>] [-i <interface>] [-p <port>] [-P]:\n",
           commands[DME_IO].longc);
    printf("\n");
    printf("        -a <attrs>: attribute (in hexadecimal) to read, or one of:\n");
    printf("                      \"L1\" (PHY layer),\n");
    printf("                      \"L1.5\" (PHY adapter layer),\n");
    printf("                      \"L2\" (link layer),\n");
    printf("                      \"L3\" (network layer),\n");
    printf("                      \"L4\" (transport layer),\n");
    printf("                      \"DME\" (DME),\n");
    printf("                      \"TSB\" (Toshiba-specific attributes),\n");
    printf("                      \"all\" (all of the above).\n");
    printf("                    If missing, default is \"all\".\n");
    printf("                    If <attrs> is \"L4\", -P is implied.\n");
    printf("        -s <sel>: attribute selector index (default is 0)\n");
    printf("        -i <interface>: Interface to read attribute on (e.g. \"apb1\", etc.)\n");
    printf("                        If set, overrides -p.\n");
    printf("        -p <port>: port to read attribute on (default is 0)\n");
    printf("        -P: if present, do a peer (instead of switch local) read\n");
    printf("\n");
    printf("    Options for writing an attribute:\n");
    printf("    svc %s w -a <attr> [-s <sel>] [-p <port>] [-P] <value>:\n",
           commands[DME_IO].longc);
    printf("\n");
    printf("        -a <attr>: attribute (in hexadecimal) to write.\n");
    printf("        -s <sel>: attribute selector index (default is 0)\n");
    printf("        -p <port>: port to read attribute on (default is 0)\n");
    printf("        -P: if present, do a peer (instead of switch local) write\n");
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
            printf("Error: can't read attribute %s (0x%x): %d\n",
                   attr_str, attr, rc);
        } else {
            printf("Error: can't read attribute 0x%x: rc=%d\n", attr, rc);
        }
        return -EIO;
    }
    if (attr_str) {
        printf("Port=%d, peer=%s, sel=%d, %s (0x%x) = 0x%x (%u)\n",
               port,
               peer ? "yes" : "no",
               selector,
               attr_str,
               attr,
               val,
               val);
    } else {
        printf("Port=%d, peer=%s, sel=%d, 0x%x = 0x%x (%u)\n",
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
                      const char *attr_str, uint16_t attr, uint32_t val,
                      uint16_t selector, int peer) {
    int rc;

    if (peer) {
        rc = switch_dme_peer_set(sw, port, attr, selector, val);
    } else {
        rc = switch_dme_set(sw, port, attr, selector, val);
    }

    if (rc) {
        if (attr_str) {
            printf("Error: can't set attribute %s (0x%x): rc=%d\n",
                   attr_str, attr, rc);
        } else {
            printf("Error: can't set attribute 0x%x: rc=%d\n",
                   attr, rc);
        }
        return -EIO;
    }
    if (attr_str) {
        printf("Port=%d, peer=%s, sel=%d, set %s (0x%x) = 0x%x (%u)\n",
               port,
               peer ? "yes" : "no",
               selector,
               attr_str,
               attr,
               val,
               val);
    } else {
        printf("Port=%d, peer=%s, sel=%d, set 0x%x = 0x%x (%u)\n",
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
    const char opts[] = "a:s:i:p:Ph";
    struct tsb_switch *sw = svc->sw;
    uint16_t selector = 0;
    int peer = 0;
    char *end;
    const char *iface_name = NULL;
    uint8_t port = 0;
    uint16_t attr = 0xbeef;
    int attr_set = 0;
    uint32_t val = 0xdeadbeef;
    int val_set = 1;
    char **args;
    int read;
    int c;

    if (!sw) {
        return -ENODEV;
    }

    if (argc <= 2) {
        printf("BUG: invalid argument specification.\n");
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
        printf("Must specify \"r\" or \"w\".\n\n");
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
                    printf("-a %s invalid: must be one of: \"all\", \"L1\", "
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
                printf("-s %s invalid: must specify a decimal selector "
                       "index\n\n", optarg);
                dme_io_usage();
                return EXIT_FAILURE;
            }
            break;
        case 'i':
            iface_name = optarg;
            break;
        case 'p':
            end = NULL;
            port = strtoul(optarg, &end, 10);
            if (*end) {
                printf("-p %s invalid: must specify a decimal port", optarg);
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
            printf("Unrecognized argument.\n");
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
            printf("Must specify -a with attribute when writing.\n");
            return EXIT_FAILURE;
        }
    }
    if (which_attrs != ONE && !read) {
        printf("Only one attribute can be written at a time.\n\n");
        dme_io_usage();
        return EXIT_FAILURE;
    }
    if (which_attrs == L4 && !peer) {
        printf("No L4 attributes on switch ports (missing -P?).\n\n");
        dme_io_usage();
        return EXIT_FAILURE;
    }
    if (!read) {
        if (!attr_set) {
            printf("Must specify -a when writing.\n\n");
            dme_io_usage();
            return EXIT_FAILURE;
        }
        if (optind >= argc) {
            printf("Must specify value to write.\n\n");
            dme_io_usage();
            return EXIT_FAILURE;
        } else {
            end = NULL;
            val = strtoul(args[optind], &end, 10);
            if (*end) {
                printf("Invalid value to write: %s.\n\n", args[optind]);
                dme_io_usage();
                return EXIT_FAILURE;
            }
            val_set = 1;
        }
    }
    /* Override the port if the user specified an interface by name. */
    if (iface_name) {
        struct interface *iface = interface_get_by_name(iface_name);
        if (!iface) {
            printf("Invalid interface: %s\n", iface_name);
            return EXIT_FAILURE;
        }
        port = iface->switch_portid;
    }

    /* Check if port is valid */
    if (port < 0 || port > SWITCH_PORT_MAX) {
        printf("Invalid port %d, must be between %d and %d.\n",
               port, 0, SWITCH_PORT_MAX - 1);
        return EXIT_FAILURE;
    }

    /*
     * Do the I/O.
     */
    if (read) {
        int rc = 0;
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
            printf("BUG: %s: can't happen.\n", __func__);
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

static void test_feature_usage(int exit_status) {
    printf("    svc %s <i|e> [-s <src_iface>] [-f <from_cport>] [-d <dst_iface>] [-t <to_cport>] [-m <size>]\n",
           commands[TESTFEATURE].longc);
    printf("\n");
    printf("    <i|e>: Initialize (start) or Exit (stop) the UniPro test-traffic feature\n");
    printf("    -h: print this message and exit\n");
    printf("    -s <src_iface>: Source UniPro interface for test traffic. Default is \"apb1\"\n");
    printf("    -f <from_cport>: Source CPort for test traffic on src_iface. Default is 0.\n");
    printf("    -d <dst_iface>: Dest UniPro interface for test traffic. Default is \"apb2\"\n");
    printf("    -t <to_cport>: Dest CPort for test traffic on dst_iface. Default is 0.\n");
    printf("    -m <size>: message size. Default is 272.\n");
    exit(exit_status);
}

static int test_feature(int argc, char* argv[]) {
    const char *longc = commands[TESTFEATURE].longc;
    const char opts[] = "hs:f:d:t:m:";
    const char *src_iface_name = "apb1";   /* -s */
    const char *dst_iface_name = "apb2";   /* -d */
    uint16_t src_cport = 0, dst_cport = 0; /* -f, -t */
    uint32_t msgsize = 272;                /* -m */
    bool init = false;
    struct tsb_switch *sw = svc->sw;
    struct interface *src_iface, *dst_iface;
    char** args;
    int rc, c;
    struct unipro_test_feature_cfg cfg;
    unsigned int src_devid, dst_devid;

    if (!sw) {
        return -ENODEV;
    }

    if (argc < 3 || strlen(argv[2]) > 1) {
        printf("svc %s: First argument must be 'i' or 'e'.\n", longc);
        test_feature_usage(EXIT_FAILURE);
    }

    switch (argv[2][0]) {
    case 'i':
        init = true;
        break;
    case 'e':
        init = false;
        break;
    default:
        printf("svc %s: first argument must be 'i' or 'e'.\n", longc);
        test_feature_usage(EXIT_FAILURE);
    }

    argc--;
    args = argv + 2;
    optind = -1;
    while ((c = getopt(argc, args, opts)) != -1) {
        switch(c) {
        case 'h':
            test_feature_usage(EXIT_SUCCESS);
            break;
        case 's':
            src_iface_name = optarg;
            break;
        case 'f':
            src_cport = strtol(optarg, NULL, 10);
            break;
        case 'd':
            dst_iface_name = optarg;
            break;
        case 't':
            dst_cport = strtol(optarg, NULL, 10);
            break;
        case 'm':
            msgsize = strtol(optarg, NULL, 10);
            break;
        case '?':
        default:
            printf("svc %s: unrecognized argument '%c'.\n", longc, (char)c);
            test_feature_usage(EXIT_FAILURE);
        }
    }

    src_iface = interface_get_by_name(src_iface_name);
    if (!src_iface) {
        printf("svc %s: nonexistent source interface %s.\n",
               longc, src_iface_name);
        test_feature_usage(EXIT_FAILURE);
    }

    /* Check if there is a port on the interface */
    if (src_iface->switch_portid == INVALID_PORT) {
        printf("svc %s: no port present on source interface %s.\n",
               longc, src_iface_name);
        test_feature_usage(EXIT_FAILURE);
    }

    dst_iface = interface_get_by_name(dst_iface_name);
    if (!dst_iface) {
        printf("svc %s: nonexistent destination interface %s.\n",
               longc, dst_iface_name);
        test_feature_usage(EXIT_FAILURE);
    }

    /* Check if there is a port on the interface */
    if (dst_iface->switch_portid == INVALID_PORT) {
        printf("svc %s: no port present on destination interface %s.\n",
               longc, dst_iface_name);
        test_feature_usage(EXIT_FAILURE);
    }

    /*
     * Assign device IDs for test interfaces. Device ID 0 is reserved
     * by the switch.
     */
    src_devid = src_iface->switch_portid + 1;
    rc = switch_if_dev_id_set(sw, src_iface->switch_portid, src_devid);
    if (rc) {
        printf("svc testfeature: Failed to assign device id: %u to interface %s: %d\n",
               src_devid, src_iface_name, rc);
        test_feature_usage(EXIT_FAILURE);
    }

    dst_devid = dst_iface->switch_portid + 1;
    rc = switch_if_dev_id_set(sw, dst_iface->switch_portid, dst_devid);
    if (rc) {
        printf("svc testfeature: Failed to assign device id: %u to interface %s: %d\n",
               dst_devid, dst_iface_name, rc);
        test_feature_usage(EXIT_FAILURE);
    }

    /*
     * Set up the Test Feature configuration struct.
     */
    cfg.tf_src = 0;
    cfg.tf_src_cportid = src_cport;
    cfg.tf_src_inc = 1;
    cfg.tf_src_size = msgsize;
    cfg.tf_src_count = 0;
    cfg.tf_src_gap_us = 0;
    cfg.tf_dst = 0;
    cfg.tf_dst_cportid = dst_cport;

    if (init) {
        /* Create a route and connection between the two endpoints. */
        rc = svc_connect_interfaces(src_iface, src_cport, dst_iface, dst_cport,
                                    CPORT_TC0,
                                    CPORT_FLAGS_CSD_N | CPORT_FLAGS_CSV_N);
        if (rc) {
            printf("%s(): couldn't connect [n=%s,c=%u]<->[n=%s,c=%u]: %i\n",
                   __func__, src_iface_name, src_cport, dst_iface_name,
                   dst_cport, rc);
            return rc;
        }
        /* Actually enable the test traffic. */
        rc = switch_enable_test_traffic(sw,
                                        src_iface->switch_portid,
                                        dst_iface->switch_portid,
                                        &cfg);
        if (rc) {
            printf("%s(): couldn't enable test traffic: %d\n", __func__, rc);
            return rc;
        }
    } else {
        rc = switch_disable_test_traffic(sw,
                                         src_iface->switch_portid,
                                         dst_iface->switch_portid,
                                         &cfg);
        if (rc) {
            printf("%s(): couldn't disable test traffic: %d\n",
                   __func__, rc);
            return rc;
        }
    }

    return 0;
}

static void qos_usage(int exit_status) {
    printf("    svc %s <sub-command> [-c <class>] [-p <port>]\n", commands[QOS].longc);
    printf("\n");
    printf("    Commands:\n");
    printf("    open -p <port>: open a port\n");
    printf("    close -p <port>: close a port\n");
    printf("    closed -p <port>: query if a port is open or closed\n");
    printf("    bwctrl -p <port> -c <class>: query if a port and traffic class have bandwidth-control enabled\n");
    printf("    bwctrl_enable -p <port> -c <class>: enable bandwidth-control on a port and class\n");
    printf("    bwctrl_disable -p <port> -c <class>: enable bandwidth-control on a port and class\n");
    printf("    signal -p <port> -c <class> -s <signal> -d <direction>: query a property of a tx or rx port\n");
    printf("    outtc: query which outgoing traffic classes are enabled\n");
    printf("    connected -p <port>: query if a switch port is connected\n");
    printf("    onreq -p <port> -c <class>: query if a switch port has requests for a traffic class\n");
    printf("    bwperiod -p <port> -c <class> [-v <cycles>]: query the bandwidth measurement period in microseconds\n");
    printf("    decsize -p <port> -c <class> [-v <bytes>]: query the decrement size\n");
    printf("    limit -p <port> -c <class> [-v <bytes>]: query the traffic limit per bandwidth measurement period\n");
    printf("    quantity -d <direction> [-p <port> -c <class>]: query the quantity of traffic passing through in bytes\n");
    printf("    reset -c <class>: reset the routing table for a traffic class\n");
    printf("    wdt -d <direction> [-v <cycles>]: query the watchdog timer in microseconds\n");
    printf("    subtc -p <port> [-c <class>]\n");
    printf("\n");
    printf("    Flags:\n");
    printf("    -c <class>: Traffic class (TC0, TC0HIGH, TC0BAND, TC1).  No default\n");
    printf("    -p <port>: Port on which to manipulate Quality of Service.  Default is 0.\n");
    printf("    -d <direction>: Transmit (tx) or receive (rx).  No default\n");
    printf("    -s <signal>: Asserted signal (accept, valid, busy, output, or tc).  No default\n");
    printf("    -v <value>: an arbitrary integer value.  No default\n");
    printf("\n");
    printf("QoS cycles run at a rate of 166 MHz.\n");
    exit(exit_status);
}

static int qos(int argc, char *argv[]) {
    const char *longc = commands[QOS].longc;
    struct tsb_switch *sw = svc->sw;
    const char opts[] = "p:c:s:d:v:";
    int rc, c;
    int64_t port = -1;
    int8_t tc = -1;
    char **args;
    bool b;
    int8_t signal = -1;
    int8_t direction = -1;
    uint8_t val;
    int64_t usr_val = -1;
    uint32_t dwval;

    if (!sw) {
        return -ENODEV;
    }

    if (argc < 3) {
        qos_usage(EXIT_FAILURE);
    }

    optind = -1;
    args = argv + 2;
    while ((c = getopt(argc - 1, args, opts)) != -1) {
        switch (c) {
        case 'p':
            if (!strchr("0123456789", optarg[0])) {
                printf("Invalid port %s, must be a decimal number in %d..%d.\n",
                       optarg, 0, SWITCH_PORT_MAX - 1);
                return -EINVAL;
            }
            port = strtol(optarg, NULL, 10);
            if (port > SWITCH_PORT_MAX) {
                printf("Invalid port %s, must be between %d and %d.\n", optarg,
                       0, SWITCH_PORT_MAX - 1);
                return -EINVAL;
            }
            break;
        case 'c':
            if (!strcmp(optarg, "TC0BAND")) {
                tc = SWITCH_TRAFFIC_CLASS_TC0BAND;
            } else if (!strcmp(optarg, "TC0HIGH")) {
                tc = SWITCH_TRAFFIC_CLASS_TC0HIGH;
            } else if (!strcmp(optarg, "TC0")) {
                tc = SWITCH_TRAFFIC_CLASS_TC0;
            } else if (!strcmp(optarg, "TC1")) {
                tc = SWITCH_TRAFFIC_CLASS_TC1;
            } else {
                printf("Invalid traffic class %s.\n", optarg);
                return -EINVAL;
            }
            break;
        case 's':
            if (!strcmp(optarg, "accept")) {
                signal = 0;
            } else if (!strcmp(optarg, "valid")) {
                signal = 1;
            } else if (!strcmp(optarg, "busy")) {
                signal = 2;
            } else if (!strcmp(optarg, "output")) {
                signal = 3;
            } else {
                printf("Invalid asserted-signal name: %s.\n", optarg);
                return -EINVAL;
            }
            break;
        case 'd':
            if (!strcmp(optarg, "tx")) {
                direction = 0;
            } else if (!strcmp(optarg, "rx")) {
                direction = 1;
            } else {
                printf("Traffic direction can only be tx or rx.\n");
                return -EINVAL;
            }
            break;
        case 'v':
            usr_val = strtol(optarg, NULL, 10);
            if (usr_val < 0) {
                printf("Value must be a positive integer.\n");
                return -EINVAL;
            }
            break;
        }
    }

    if (!strcmp(argv[2], "open")) {
        if (port < 0) {
            printf("Need a port number from %d to %d.\n", 0,
                   SWITCH_UNIPORT_MAX - 1);
            qos_usage(EXIT_FAILURE);
        }
        rc = switch_qos_open_port(sw, port);
        if (rc) {
            printf("svc %s %s -p %d: %d\n", longc, argv[2], (uint32_t)port, rc);
            return rc;
        }
    } else if (!strcmp(argv[2], "close")) {
        if (port < 0) {
            printf("Need a port number from %d to %d.\n", 0,
                   SWITCH_UNIPORT_MAX - 1);
            qos_usage(EXIT_FAILURE);
        }
        rc = switch_qos_close_port(sw, port);
        if (rc) {
            printf("svc %s %s -p %d: %d\n", longc, argv[2], (uint32_t)port, rc);
            return rc;
        }
    } else if (!strcmp(argv[2], "closed")) {
        if (port < 0) {
            printf("Need a port number from %d to %d.\n", 0,
                   SWITCH_UNIPORT_MAX - 1);
            qos_usage(EXIT_FAILURE);
        }
        rc = switch_qos_port_closed(sw, port, &b);
        if (rc) {
            return rc;
        }

        printf("svc %s: port %d %s\n", longc, (uint32_t)port,
               b ? "closed" : "open");
    } else if (!strcmp(argv[2], "bwctrl")) {
        if (port < 0) {
            printf("Need a port number from %d to %d.\n", 0,
                   SWITCH_UNIPORT_MAX - 1);
            qos_usage(EXIT_FAILURE);
        }
        if (tc < 0) {
            printf("Need a traffic class from: TC0, TC0BAND, TC0HIGH, TC1.\n");
            qos_usage(EXIT_FAILURE);
        }
        rc = switch_qos_bwctrl_enabled(sw, port, tc, &b);
        if (rc) {
            printf("svc %s %s -p %d -c %d: %d\n", longc, argv[2],
                   (uint32_t)port, tc, rc);
            return rc;
        }

        printf("svc %s: bandwidth control for port %d is %s\n", longc,
               (uint32_t)port, b ? "on" : "off");
    } else if (!strcmp(argv[2], "bwctrl_enable")) {
        if (port < 0) {
            printf("Need a port number from %d to %d.\n", 0,
                   SWITCH_UNIPORT_MAX - 1);
            qos_usage(EXIT_FAILURE);
        }
        if (tc < 0) {
            printf("Need a traffic class from: TC0, TC0BAND, TC0HIGH, TC1.\n");
            qos_usage(EXIT_FAILURE);
        }
        switch (tc) {
        case SWITCH_TRAFFIC_CLASS_TC0HIGH:
            rc = switch_qos_enable_bwctrl(sw, port, tc);
            if (rc) {
                printf("svc %s %s -p %d -c %d: %d\n", longc, argv[2],
                       (uint32_t)port, tc, rc);
                return rc;
            }
            break;
        case SWITCH_TRAFFIC_CLASS_TC0BAND:
            printf("svc %s %s -p %d -c %d: bandwidth control always enabled for TC0BAND\n");
            break;
        default:
            printf("svc %s %s -p %d -c %d: bandwidth control only possible for TC0BAND and TC0HIGH\n");
            break;
        }
    } else if (!strcmp(argv[2], "bwctrl_disable")) {
        if (port < 0) {
            printf("Need a port number from %d to %d.\n", 0,
                   SWITCH_UNIPORT_MAX - 1);
            qos_usage(EXIT_FAILURE);
        }
        if (tc < 0) {
            printf("Need a traffic class from: TC0, TC0BAND, TC0HIGH, TC1.\n");
            qos_usage(EXIT_FAILURE);
        }
        switch (tc) {
        case SWITCH_TRAFFIC_CLASS_TC0HIGH:
            rc = switch_qos_disable_bwctrl(sw, port, tc);
            if (rc) {
                printf("svc %s %s -p %d -c %d: %d\n", longc, argv[2],
                       (uint32_t)port, tc, rc);
                return rc;
            }
            break;
        case SWITCH_TRAFFIC_CLASS_TC0BAND:
            printf("svc %s %s -p %d -c %d: bandwidth control always enabled for TC0BAND\n",
                   longc, argv[2], (uint32_t)port, tc);
            break;
        default:
            printf("svc %s %s -p %d -c %d: bandwidth control only possible for TC0BAND and TC0HIGH\n",
                   longc, argv[2], (uint32_t)port, tc);
            break;
        }
    } else if (!strcmp(argv[2], "signal")) {
        if (port < 0) {
            printf("Need a port number from %d to %d.\n", 0,
                   SWITCH_UNIPORT_MAX - 1);
            qos_usage(EXIT_FAILURE);
        }
        if (signal < 0) {
            printf("Need a type of asserted signal.\n");
            qos_usage(EXIT_FAILURE);
        }
        switch (signal) {
        case 0:
            if (direction < 0) {
                printf("Need a direction: tx or rx.\n");
                qos_usage(EXIT_FAILURE);
            }
            if (!direction) {
                rc = switch_qos_transmit_accept_signal(sw, port, &b);
                if (rc) {
                    printf("svc %s %s -p %d -s %d -d tx: %d\n", longc, argv[2],
                           port, signal, rc);
                    return rc;
                }
            } else {
                if (tc < 0) {
                    printf("Need a traffic class from: TC0, TC0BAND, TC0HIGH, TC1.\n");
                    qos_usage(EXIT_FAILURE);
                }
                rc = switch_qos_source_accept_signal(sw, port, tc, &b);
                if (rc) {
                    printf("svc %s %s -p %d -s %d -d rx: %d\n", longc, argv[2],
                           port, signal, rc);
                    return rc;
                }
            }
            printf("svc %s %s -p %d -c %d: %s accept signal %s\n", longc,
                   argv[2], (uint32_t)port, tc, direction ? "rx" : "tx",
                   b ? "asserted" : "not asserted");
            break;
        case 1:
            if (direction < 0) {
                printf("Need a direction: tx or rx.\n");
                qos_usage(EXIT_FAILURE);
            }
            if (!direction) {
                rc = switch_qos_transmit_valid_signal(sw, port, &b);
                if (rc) {
                    printf("svc %s %s -p %d -s %d -d tx: %d\n", longc, argv[2],
                           port, signal, rc);
                    return rc;
                }
            } else {
                if (tc < 0) {
                    printf("Need a traffic class from: TC0, TC0BAND, TC0HIGH, TC1.\n");
                    qos_usage(EXIT_FAILURE);
                }
                rc = switch_qos_source_valid_signal(sw, port, tc, &b);
                if (rc) {
                    printf("svc %s %s -p %d -s %d -d rx: %d\n", longc, argv[2],
                           port, signal, rc);
                    return rc;
                }
            }
            printf("svc %s %s -p %d -c %d: %s valid signal %s\n", longc,
                   argv[2], (uint32_t)port, tc, direction ? "rx" : "tx",
                   b ? "asserted" : "not asserted");
            break;
        case 2:
            rc = switch_qos_gate_arb_busy(sw, &b);
            if (rc) {
                printf("svc %s %s -s %d: %d\n", longc, argv[2], signal, rc);
                return rc;
            }
            printf("svc %s %s: gate arbiter busy signal %s\n", longc, argv[2],
                   b ? "asserted" : "not asserted");
            break;
        case 3:
            rc = switch_qos_gate_arb_output(sw, port, &b);
            if (rc) {
                printf("svc %s %s -p %d -s %d: %d\n", longc, argv[2],
                       (uint32_t)port, signal, rc);
                return rc;
            }
            printf("svc %s %s -p %d: gate arbiter output signal %s\n", longc,
                   argv[2], (uint32_t)port, b ? "asserted" : "not asserted");
            break;
        }
    } else if (!strcmp(argv[2], "outtc")) {
        rc = switch_qos_output_traffic_class(sw, &val);
        if (rc) {
            printf("svc %s %s: %d\n", longc, argv[2], signal, rc);
            return rc;
        }
        printf("svc %s %s: TC0 (%b), TC0BAND (%b), TC0HIGH (%b), TC1 (%b)\n",
               longc, argv[2],
               val & (AR_STATUS_CONNECT_CLASS_TC0 >> 16),
               val & (AR_STATUS_CONNECT_CLASS_TC0BAND >> 16),
               val & (AR_STATUS_CONNECT_CLASS_TC0HIGH >> 16),
               val & (AR_STATUS_CONNECT_CLASS_TC1 >> 16));
    } else if (!strcmp(argv[2], "connected")) {
        if (port < 0) {
            printf("Need a port number from %d to %d.\n", 0,
                   SWITCH_UNIPORT_MAX - 1);
            qos_usage(EXIT_FAILURE);
        }
        rc = switch_qos_port_connected(sw, port, &b);
        if (rc) {
            printf("svc %s %s -p %d: %d\n", longc, argv[2], (uint32_t)port, rc);
            return rc;
        }
        printf("svc %s %s -p %d: %s\n", longc, argv[2], (uint32_t)port,
               b ? "connected" : "disconnected");
    } else if (!strcmp(argv[2], "onreq")) {
        if (port < 0) {
            printf("Need a port number from %d to %d.\n", 0,
                   SWITCH_UNIPORT_MAX - 1);
            qos_usage(EXIT_FAILURE);
        }
        if (tc < 0) {
            printf("Need a traffic class from: TC0, TC0BAND, TC0HIGH, TC1.\n");
            qos_usage(EXIT_FAILURE);
        }
        rc = switch_qos_request_status(sw, tc, port, &b);
        if (rc) {
            printf("svc %s %s -p %d -c %d: %d\n", longc, argv[2],
                   (uint32_t)port, tc, rc);
            return rc;
        }
        printf("svc %s %s -p %d -c %d: %s\n", longc, argv[2], (uint32_t)port,
               tc, b ? "on request" : "not on request");
    } else if (!strcmp(argv[2], "bwperiod")) {
        if (port < 0) {
            printf("Need a port number from %d to %d.\n", 0,
                   SWITCH_UNIPORT_MAX - 1);
            qos_usage(EXIT_FAILURE);
        }
        if (tc < 0) {
            printf("Need a traffic class from: TC0, TC0BAND, TC0HIGH, TC1.\n");
            qos_usage(EXIT_FAILURE);
        }
        if (usr_val < 0) {
            rc = switch_qos_get_bwperiod(sw, port, tc, &dwval);
            if (rc) {
                printf("svc %s %s -p %d -c %d: %d\n", longc, argv[2],
                       (uint32_t)port, tc, rc);
                return rc;
            }
            printf("svc %s %s -p %d -c %d: %d cycles = %d usecs\n", longc,
                   argv[2], (uint32_t)port, tc, dwval,
                   QOS_CYCLES_TO_USECS(dwval));
        } else {
            rc = switch_qos_set_bwperiod(sw, port, tc, usr_val);
            if (rc) {
                printf("svc %s %s -p %d -c %d -v %u: %d\n", longc, argv[2],
                       port, tc, (uint32_t)usr_val, rc);
                return rc;
            }
        }
    } else if (!strcmp(argv[2], "decsize")) {
        if (port < 0) {
            printf("Need a port number from %d to %d.\n", 0,
                   SWITCH_UNIPORT_MAX - 1);
            qos_usage(EXIT_FAILURE);
        }
        if (tc < 0) {
            printf("Need a traffic class from: TC0, TC0BAND, TC0HIGH, TC1.\n");
            qos_usage(EXIT_FAILURE);
        }
        if (usr_val < 0) {
            rc = switch_qos_get_decsize(sw, port, tc, &dwval);
            if (rc) {
                printf("svc %s %s -p %d -c %d: %d\n", longc, argv[2],
                       (uint32_t)port, tc, rc);
                return rc;
            }
            printf("svc %s %s -p %d -c %d: %u bytes\n", longc, argv[2],
                   (uint32_t)port, tc, dwval);
        } else {
            rc = switch_qos_set_decsize(sw, port, tc, (uint32_t)usr_val);
            if (rc) {
                printf("svc %s %s -p %d -c %d -v %u: %d\n", longc, argv[2],
                       port, tc, (uint32_t)usr_val, rc);
                return rc;
            }
        }
    } else if (!strcmp(argv[2], "limit")) {
        if (port < 0) {
            printf("Need a port number from %d to %d.\n", 0,
                   SWITCH_UNIPORT_MAX - 1);
            qos_usage(EXIT_FAILURE);
        }
        if (tc < 0) {
            printf("Need a traffic class from: TC0, TC0BAND, TC0HIGH, TC1.\n");
            qos_usage(EXIT_FAILURE);
        }
        if (usr_val < 0) {
            rc = switch_qos_get_limit(sw, port, tc, &dwval);
            if (rc) {
                printf("svc %s %s -p %d -c %d: %d\n", longc, argv[2],
                       (uint32_t)port, tc, rc);
                return rc;
            }
            printf("svc %s %s -p %d -c %d: 0x%x\n", longc, argv[2],
                   (uint32_t)port, tc, dwval);
        } else {
            rc = switch_qos_set_limit(sw, port, tc, (uint32_t)usr_val);
            if (rc) {
                printf("svc %s %s -p %d -c %d -v %u: %d\n", longc, argv[2],
                       port, tc, (uint32_t)usr_val, rc);
                return rc;
            }
        }
    } else if (!strcmp(argv[2], "quantity")) {
        if (direction < 0) {
            printf("Need a direction: tx or rx.\n");
            qos_usage(EXIT_FAILURE);
        }
        if (direction) {
            if (port < 0) {
                printf("Need a port number from %d to %d.\n", 0,
                       SWITCH_UNIPORT_MAX - 1);
                qos_usage(EXIT_FAILURE);
            }
            if (tc < 0) {
                printf("Need a traffic class from: TC0, TC0BAND, TC0HIGH, TC1.\n");
                qos_usage(EXIT_FAILURE);
            }
            rc = switch_qos_source_quantity(sw, port, tc, &dwval);
            if (rc) {
                printf("svc %s %s -p %d -c %d -d rx: %d\n", longc, argv[2],
                       (uint32_t)port, tc, rc);
                return rc;
            }
            printf("svc %s %s -p %d -c %d -d rx: 0x%x\n", longc, argv[2],
                   (uint32_t)port, dwval);
        } else {
            rc = switch_qos_transmit_quantity(sw, &dwval);
            if (rc) {
                printf("svc %s %s -d tx: %d\n", longc, argv[2], rc);
                return rc;
            }
            printf("svc %s %s -d tx: 0x%x\n", longc, argv[2], dwval);
        }
    } else if(!strcmp(argv[2], "reset")) {
        if (tc < 0) {
            printf("Need a traffic class from: TC0, TC1.\n");
            qos_usage(EXIT_FAILURE);
        }
        switch (tc) {
        case SWITCH_TRAFFIC_CLASS_TC0:
        case SWITCH_TRAFFIC_CLASS_TC1:
            rc = switch_qos_reset_routing_table(sw, tc);
            if (rc) {
                printf("svc %s %s -c %d: %d\n", longc, argv[2], tc, rc);
                return rc;
            }
            break;
        default:
            printf("svc %s %s -c %d: routing tables only exist for TC0 and TC1\n",
                   longc, argv[2], tc);
            break;
        }
    } else if(!strcmp(argv[2], "wdt")) {
        if (direction < 0) {
            printf("Need a direction: tx or rx.\n");
            qos_usage(EXIT_FAILURE);
        }
        if (usr_val < 0 && direction) {
            rc = switch_qos_get_in_wdt_count(sw, &dwval);
            if (rc) {
                printf("svc %s %s -d rx: %d\n", longc, argv[2], rc);
                return rc;
            }
            printf("svc %s %s -d rx: %d cycles = %d usecs\n", longc, argv[2],
                   dwval, QOS_CYCLES_TO_USECS(dwval));
        } else if (usr_val > 0 && direction) {
            rc = switch_qos_set_in_wdt_count(sw, usr_val);
            if (rc) {
                printf("svc %s %s -d rx -v 0x%x: %d\n", longc, argv[2], usr_val,
                       rc);
                return rc;
            }
        } else if (usr_val < 0 && !direction) {
            rc = switch_qos_get_out_wdt_count(sw, &dwval);
            if (rc) {
                printf("svc %s %s -d tx: %d\n", longc, argv[2], rc);
                return rc;
            }
            printf("svc %s %s -d tx: %d cycles = %d usecs\n", longc, argv[2],
                   dwval, QOS_CYCLES_TO_USECS(dwval));
        } else if (usr_val > 0 && !direction) {
            rc = switch_qos_set_out_wdt_count(sw, usr_val);
            if (rc) {
                printf("svc %s %s -d tx -v 0x%x: %d\n", longc, argv[2], usr_val,
                       rc);
                return rc;
            }
        }
    } else if (!strcmp(argv[2], "subtc")) {
        if (port < 0) {
            printf("Need a port number from %d to %d.\n", 0,
                   SWITCH_UNIPORT_MAX - 1);
            qos_usage(EXIT_FAILURE);
        }
        if (tc < 0) {
            rc = switch_qos_get_subtc(sw, port, (uint8_t*)&tc);
            if (rc) {
                printf("svc %s %s -p %d: %d\n", longc, argv[2], (int32_t)port,
                       rc);
                return rc;
            }
            switch (tc) {
            case SWITCH_TRAFFIC_CLASS_TC0HIGH:
                printf("svc %s %s -p %d: sub-traffic-class TC0HIGH requested\n",
                       longc, argv[2], (int32_t)port);
                break;
            case SWITCH_TRAFFIC_CLASS_TC0BAND:
                printf("svc %s %s -p %d: sub-traffic-class TC0BAND requested\n",
                   longc, argv[2], (int32_t)port);
                break;
            default:
                printf("svc %s %s -p %d: no sub-traffic-class requested for TC0\n",
                   longc, argv[2], (int32_t)port);
                break;
            }
        } else {
            rc = switch_qos_set_subtc(sw, port, tc);
            if (rc) {
                printf("svc %s %s -p %d -c %d: %d\n", longc, argv[2],
                       (int32_t)port, tc, rc);
                return rc;
            }
        }
    } else {
        qos_usage(EXIT_FAILURE);
    }

    return 0;
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int svc_main(int argc, char *argv[])
#endif
{
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
    case START:
        svcd_start();
        break;
    case STOP:
        svcd_stop();
        break;
    case LINKTEST:
        rc = link_test(argc, argv);
        break;
    case LINKSTATUS:
        rc = link_status(argc, argv);
        break;
    case ROUTINGTABLE:
        rc = dump_routing_table(argc, argv);
        break;
    case DME_IO:
        rc = dme_io(argc, argv);
        break;
    case TESTFEATURE:
        rc = test_feature(argc, argv);
        break;
    case QOS:
        rc = qos(argc, argv);
        break;
    default:
        usage(EXIT_FAILURE);
    }

    return rc;
}
