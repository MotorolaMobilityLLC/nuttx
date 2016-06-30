/*
 * Copyright (c) 2014, 2015 Google Inc.
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

/**
 * @file gpio.c
 * @brief Ara Toshiba bridge ASIC GPIO test program
 *
 * IMPORTANT: This app assumes that the corresponding GPIO chip is
 *            already registered.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include <nuttx/gpio.h>

/*
 * These commands mirror the Greybus GPIO protocol ops as of version
 * v0.1.
 *
 * The following Greybus GPIO operations have no counterparts here:
 *
 *     SET_DEBOUNCE
 *     IRQ_TYPE
 *     IRQ_ACK
 *     IRQ_MASK
 *     IRQ_UNMASK
 *     IRQ_EVENT
 */
#define INVALID -1
enum {
    HELP,
    LINE_COUNT,
    ACTIVATE,
    DEACTIVATE,
    GET_DIRECTION,
    DIRECTION_IN,
    DIRECTION_OUT,
    GET_VALUE,
    SET_VALUE,
    MAX_CMD,
};

struct command {
    const char shortc;
    const char *longc;
    const char *argspec;
    const char *help;
};

static const struct command commands[] = {
    [HELP]          = {'h', "help", NULL, "Print this message and exit"},
    [LINE_COUNT]    = {'l', "line-count",
                       NULL,
                       "Print number of GPIO lines\n"
                       "\t\t\t(NOT number minus 1, as Greybus returns)"},
    [ACTIVATE]      = {'a', "activate",
                       "<gpio> [...]",
                       "Assign GPIO lines for use by this app"},
    [DEACTIVATE]    = {'u', "deactivate",
                       "<gpio> [...]",
                       "Unassign GPIO lines for use by this app"},
    [GET_DIRECTION] = {'d', "get-direction",
                       "<gpio> [...]",
                       "Get direction (in or out) of GPIO lines"},
    [DIRECTION_IN]  = {'i', "direction-in",
                       "<gpio> [...]",
                       "Configure GPIO lines to input mode"},
    [DIRECTION_OUT] = {'o', "direction-out",
                       "<gpio> [...]",
                       "Configure GPIO lines to output mode"},
    [GET_VALUE]     = {'g', "get-value",
                       "<gpio> [...]",
                       "Get value (0 or 1) of GPIO lines"},
    [SET_VALUE]     = {'s', "set-value",
                       "<gpio> <value> [<gpio2> <value2> ...]",
                       "Set value (0 or 1) of GPIO lines, if configured "
                       "to output"},
};

static void print_usage(void)
{
    int i;
    printf("gpio: usage:\n");
    for (i = 0; i < MAX_CMD; i++) {
        const char *argspec = commands[i].argspec;
        const char *space = " ";
        if (!argspec) {
            space = "";
            argspec = "";
        }
        printf("    gpio [%c|%s]%s%s: %s\n",
               commands[i].shortc, commands[i].longc,
               space, argspec,
               commands[i].help);
    }
    printf("\n"
           "<gpio> values range from 0 to the line count minus one.\n");
}

static void usage(int exit_status)
{
    print_usage();
    exit(exit_status);
}

static void do_line_count(void)
{
    printf("GPIO line count: %u\n", gpio_line_count());
}

static void do_activate(uint8_t *which, size_t count)
{
    int i;
    for (i = 0; i < count; i++) {
        gpio_activate(which[i]);
    }
}

static void do_deactivate(uint8_t *which, size_t count)
{
    int i;
    for (i = 0; i < count; i++) {
        gpio_deactivate(which[i]);
    }
}

static void do_get_direction(uint8_t *which, size_t count)
{
    int i;
    int direction;
    for (i = 0; i < count; i++) {
        direction = gpio_get_direction(which[i]);
        printf("GPIO%d: %s\n", which[i],
               direction ? ((direction < 0) ? "UNKNOWN" : "IN") : "OUT");
    }
}

static void do_direction_in(uint8_t *which, size_t count)
{
    int i;
    for (i = 0; i < count; i++) {
        gpio_direction_in(which[i]);
    }
}

static void do_direction_out(uint8_t *which, size_t count)
{
    int i;
    for (i = 0; i < count; i++) {
        gpio_direction_out(which[i], 0);
    }
}

static void do_get_value(uint8_t *which, size_t count)
{
    int i;
    for (i = 0; i < count; i++) {
        printf("GPIO%d: %d\n", which[i], gpio_get_value(which[i]));
    }
}

static void do_set_value(uint8_t *which, uint8_t *values, size_t count)
{
    int i;
    for (i = 0; i < count; i++) {
        gpio_set_value(which[i], values[i]);
    }
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int gpio_main(int argc, char *argv[])
#endif
{
    int i;
    int cmd = INVALID;
    const char *cmd_str;
    const size_t max_gpios = 255;
    uint8_t *gpios = malloc(sizeof(uint8_t) * max_gpios);
    uint8_t *values = malloc(sizeof(uint8_t) * max_gpios);
    int rc = 0;

    if (!gpios || !values) {
        free(gpios);
        free(values);
        return EXIT_FAILURE;
    }

    /* Parse arguments. */
    if (argc < 2) {
        rc = EXIT_FAILURE;
        goto done;
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

    /* Parse command arguments and run command. */
    argc -= 2;
    void (*handler)(uint8_t*, size_t);
    switch (cmd) {
    /* These are special cases. */
    case HELP:
        print_usage();
        goto done;
    case LINE_COUNT:
        do_line_count();
        goto done;
    case SET_VALUE:
        if (argc == 0) {
            fprintf(stderr,
                    "You must specify at least one GPIO and value to set.\n");
            rc = EXIT_FAILURE;
            goto done;
        } else if (argc & 1) {
            fprintf(stderr, "You must specify one value per GPIO\n");
            rc = EXIT_FAILURE;
            goto done;
        } else {
            printf("Setting (gpio, value):");
            for (i = 0; i < argc / 2; i++) {
                gpios[i] = (uint8_t)atoi(argv[2 + 2 * i]);
                values[i] = !!(uint8_t)atoi(argv[3 + 2 * i]);
                printf(" (%u, %u)", gpios[i], values[i]);
            }
            printf("\n");
            do_set_value(gpios, values, (size_t)argc / 2);
        }
        goto done;
    default:
        rc = EXIT_FAILURE;
        goto done;
    /* The rest are all parsed in the same way. */
    case ACTIVATE:
        handler = do_activate;
        break;
    case DEACTIVATE:
        handler = do_deactivate;
        break;
    case GET_DIRECTION:
        handler = do_get_direction;
        break;
    case DIRECTION_IN:
        handler = do_direction_in;
        break;
    case DIRECTION_OUT:
        handler = do_direction_out;
        break;
    case GET_VALUE:
        handler = do_get_value;
        break;
    }

    if (argc == 0) {
        fprintf(stderr, "You must specify at least one GPIO.\n");
        rc = EXIT_FAILURE;
        goto done;
    }
    if (argc > max_gpios) {
        argc = max_gpios;
    }
    printf("GPIOs:");
    for (i = 0; i < argc; i++) {
        gpios[i] = (uint8_t)atoi(argv[2 + i]);
        printf(" %u", gpios[i]);
    }
    printf("\n");
    handler(gpios, (size_t)argc);
 done:
    free(gpios);
    free(values);
    if (rc) {
        usage(rc);
    }
    return 0;
}
