/*
 *   Copyright (C) 2015 Motorola Mobility, LLC.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/power/battery_state.h>
#include <nuttx/util.h>

// ARGV indices
#define OPT     1
#define ARG     2

/* Rely on the order of the batt_temp_e and batt_level_e enums */
static const char* t[] = {"normal", "reduced-chg", "no-chg", "cool-down", "unavailable"};
static const char* l[] = {"empty", "low", "normal", "full"};

static void print_usage(void)
{
    printf("\nUsage: battery_state [-t|l] [arguments]\n");
    printf("Where:\n");
    printf(" -t %s|%s|%s|%s : Set temperature.\n", t[0], t[1], t[2], t[3]);
    printf(" -l %s|%s|%s|%s : Set level.\n", l[0], l[1], l[2], l[3]);
}

static int do_temp(int argc, FAR char *argv[])
{
    int i;

    for (i = 0; i < ARRAY_SIZE(t); i++)
        if (!strcmp(argv[ARG], t[i]))
            return battery_state_set_temp(i);

    return -1;
}

static int do_level(int argc, FAR char *argv[])
{
    int i;

    for (i = 0; i < ARRAY_SIZE(l); i++)
        if (!strcmp(argv[ARG], l[i]))
            return battery_state_set_level(i);

    return -1;
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int battery_state_main(int argc, char *argv[])
#endif
{
    int ret;

    if (argc != 3) {
        ret = -1;
        goto done;
    }

    if (!strcmp(argv[OPT], "-t"))
        ret = do_temp(argc, argv);
    else if (!strcmp(argv[OPT], "-l"))
        ret = do_level(argc, argv);
    else
        ret = -1;
done:
    if (ret) {
        print_usage();
        exit(EXIT_FAILURE);
    } else {
        exit(EXIT_SUCCESS);
    }
}
