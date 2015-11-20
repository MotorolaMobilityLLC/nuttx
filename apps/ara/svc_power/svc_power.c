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

#define DBG_COMP ARADBG_POWER
#include <ara_debug.h>

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "up_power.h"
#include "interface.h"
#include "tsb_switch.h"

static const char *progname = PROGNAME;

/*
 * Helpers etc.
 */

enum {
    HELP,
    SET_POWER,
    WAKEOUT,
    DUMPSTATE,

    MAX_CMD,
};

struct command {
    const char shortc;
    const char *longc;
    const char *help;
    int (*command_func)(int argc, char *argv[]);
};

static int cmd_usage(int argc, char *argv[]);
static int cmd_set_power(int argc, char *argv[]);
static int cmd_wakeout(int argc, char *argv[]);
static int cmd_dumpstate(int argc, char *argv[]);

static const struct command commands[] = {
    [HELP] = {'h', "help", "print this usage and exit", cmd_usage},
    [SET_POWER] = {'p', "power", "get/set interface power", cmd_set_power},
    [WAKEOUT] = {'w', "wakeout", "pulse WAKEOUT", cmd_wakeout},
    [DUMPSTATE] = {'d', "dumpstate", "dump system power state", cmd_dumpstate},
};

static void print_interface_usage(void)
{
    int i;
    struct interface *iface;
    printf("\nLegal <interface> values on this board:\n");

    printf("  \"all\" -- all interfaces\n");
    interface_foreach(iface, i) {
        printf("  %s", iface->name);
        if (iface->switch_portid != INVALID_PORT) {
            printf("\t(switch port %d)\n", iface->switch_portid);
        } else {
            printf("\t(no switch port)\n");
        }
    }
}

__attribute__((noreturn))
static void usage(int exit_status)
{
    int i;
    printf("%s: usage:\n", progname);
    for (i = 0; i < MAX_CMD; i++) {
        printf("    %s [%c|%s]: %s\n",
               progname,
               commands[i].shortc, commands[i].longc, commands[i].help);
    }
    exit(exit_status);
}

/*
 * Generic "do something to an interface or all interfaces" helper.
 *
 * The interfaces on this board are searched for one named
 * "iface_name". (As a special case, "name" may be "all" to specify
 * all interfaces).
 *
 * The function iface_func is then called for each interface found in
 * this way, and is handed the given context each time. The first
 * nonzero value returned from iface_func is returned immediately.
 *
 * If iface_func returns 0 each time it is called, this function
 * returns 0.
 *
 * If no interface can be found, this function prints an error and
 * returns a negative errno.
 */
static int do_to_iface(char *iface_name,
                       int (*iface_func)(struct interface*, void*),
                       void *context)
{
    int i;
    int rc;
    struct interface *iface;
    if (!strcmp(iface_name, "all") || !strcmp(iface_name, "ALL")) {
        iface = NULL; /* NULL == "all interfaces" */
    } else {
        iface = interface_get_by_name(iface_name);
        if (!iface) {
            printf("Invalid interface: %s\n", iface_name);
            print_interface_usage();
            return -EINVAL;
        }
    }
    if (iface) {
        rc = iface_func(iface, context);
    } else {
        interface_foreach(iface, i) {
            rc = iface_func(iface, context);
            if (rc) {
                break;
            }
        }
    }
    return rc;
}

/*
 * Usage
 */

static int cmd_usage(int argc, char *argv[])
{
    usage(EXIT_SUCCESS);
}

/*
 * Interface power control
 */

static void set_power_usage(int exit_status)
{
    printf("%s %s <interface> <0|1>: usage:\n",
           progname, commands[SET_POWER].longc);
    printf("    <interface>: Interface to set power state of.\n");
    printf("    <0|1>: specify \"0\" to power off, \"1\" to power on.\n");
    printf("\n");
    printf("NOTE: This may interfere with the power subsystem's\n"
           "      refcounting. Use only if you know what you're doing.\n");
    print_interface_usage();
    exit(exit_status);
}

static int set_power_func(struct interface *iface, void *context)
{
    int enable = (int)context;
    return enable ? interface_pwr_enable(iface) : interface_pwr_disable(iface);
}

static int cmd_set_power(int argc, char *argv[])
{
    void *context;
    int enable;
    if (argc != 4) {
        set_power_usage(EXIT_FAILURE);
    }
    enable = strtol(argv[3], NULL, 10);
    context = (void*)enable;
    return do_to_iface(argv[2], set_power_func, context);
}

/*
 * Wake out
 */

static void wakeout_usage(int exit_status)
{
    printf("%s %s <interface>: usage:\n",
           progname, commands[WAKEOUT].longc);
    printf("   <interface>: Interface to send WAKEOUT to.\n");
    print_interface_usage();
    exit(exit_status);
}

static int wakeout_func(struct interface *iface, void *context)
{
    (void)context;
    return interface_generate_wakeout(iface, false);
}

static int cmd_wakeout(int argc, char *argv[])
{
    if (argc != 3) {
        wakeout_usage(EXIT_FAILURE);
    }
    return do_to_iface(argv[2], wakeout_func, NULL);
}

/*
 * Dump system state
 */

static void dumpstate_usage(int exit_status)
{
    printf("%s %s <interface>: dump power system state\n",
           progname, commands[DUMPSTATE].longc);
    print_interface_usage();
    exit(exit_status);
}

static int dumpstate_func(struct interface *iface, void *context)
{
    struct vreg *vreg;
    int i;

    (void)context;

    printf("Interface %s:\n", iface->name);

    if (iface->switch_portid == INVALID_PORT) {
        printf("\tswitch_portid=<none>\n");
    } else {
        printf("\tswitch_portid=%u\n", iface->switch_portid);
    }

    vreg = iface->vreg;

    printf("\tvreg: %s\n", vreg->name);
    if (!vreg->vregs) {
        printf("\t\t(no vregs)\n");
        return 0;
    }

    printf("\t\tnr_vregs=%u\n", vreg->nr_vregs);
    printf("\t\tpower_state=%d\n", vreg->power_state);
    printf("\t\tuse_count=%u\n", atomic_get(&vreg->use_count));
    for (i = 0; i < vreg->nr_vregs; i++) {
        printf("\t\tvregs[%d]: gpio %u, hold_time %u, active_high %u, def_val %u\n",
               i, vreg->vregs[i].gpio, vreg->vregs[i].hold_time,
               vreg->vregs[i].active_high, vreg->vregs[i].def_val);
    }

    /*
     * Do a little extra for the module ports, which are the currently
     * used type (e.g. DB3).
     */
    if (interface_is_module_port(iface)) {
        enum wd_debounce_state db_state = iface->detect_in.db_state;
        enum wd_debounce_state last_state = iface->detect_in.last_state;
        unsigned int detect_in_pol =
            iface->flags & ARA_IFACE_FLAG_DETECT_IN_ACTIVE_HIGH;

        printf("\tdetect in polarity: %s\n", detect_in_pol ? "high" : "low");

        printf("\twake/detect:\n");
        printf("\t\tgpio: %u\n", iface->detect_in.gpio);
        printf("\t\tdb_state: %s\n",
               db_state == WD_ST_INVALID ? "invalid" :
               db_state == WD_ST_INACTIVE_DEBOUNCE ? "inactive debounce" :
               db_state == WD_ST_ACTIVE_DEBOUNCE ? "active debounce" :
               db_state == WD_ST_INACTIVE_STABLE ? "inactive stable" :
               db_state == WD_ST_ACTIVE_STABLE ? "active stable" :
               "<internal error>");
        printf("\t\tlast_state: %s\n",
               last_state == WD_ST_INVALID ? "invalid" :
               last_state == WD_ST_INACTIVE_DEBOUNCE ? "inactive debounce" :
               last_state == WD_ST_ACTIVE_DEBOUNCE ? "active debounce" :
               last_state == WD_ST_INACTIVE_STABLE ? "inactive stable" :
               last_state == WD_ST_ACTIVE_STABLE ? "active stable" :
               "<internal error>");

        printf("\thotplug state: %s\n",
               iface->hp_state == HOTPLUG_ST_UNKNOWN ? "unknown" :
               iface->hp_state == HOTPLUG_ST_PLUGGED ? "plugged" :
               iface->hp_state == HOTPLUG_ST_UNPLUGGED ? "unplugged" :
               "<internal error>");
    }

    return 0;
}

static int cmd_dumpstate(int argc, char *argv[])
{
    if (argc != 3) {
        dumpstate_usage(EXIT_FAILURE);
    }
    return do_to_iface(argv[2], dumpstate_func, NULL);
}


/*
 * main()
 */

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int power_main(int argc, char *argv[])
#endif
{
    const char *cmd_str;
    int i;

    if (argc < 2) {
        usage(EXIT_FAILURE);
    }
    cmd_str = argv[1];
    for (i = 0; i < MAX_CMD; i++) {
        if (!strcmp(cmd_str, commands[i].longc) ||
            (strlen(cmd_str) == 1 && cmd_str[0] == commands[i].shortc)) {
            return commands[i].command_func(argc, argv);
        }
    }
    usage(EXIT_FAILURE);
}
