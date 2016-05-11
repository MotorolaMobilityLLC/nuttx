/*
 *   Copyright (C) 2016 Motorola Mobility, LLC.
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

#include <nuttx/power/bq25896.h>

// ARGV indices
#define OPT         1
#define REG         2
#define VAL         3
#define MASK        3
#define SET         4

static void print_usage(void)
{
    printf("\nUsage: bq25896 [-r|w|m] [arguments]\n");
    printf("Where:\n");
    printf(" -r : Read all device registers.\n");
    printf(" -r <reg address> : Read the register.\n");
    printf(" -w <reg address> <value>: Write to the register.\n");
    printf(" -m <reg address> <mask> <set> Mask write to the register.\n");
}

static void print_register(uint8_t reg)
{
    int val = bq25896_reg_read(reg);

    if (val < 0)
        printf("Err=%d reading 0x%02x\n", val, reg);
    else
        printf("reg[0x%02x]=0x%02x\n", reg, val);
}

static int do_read(int argc, FAR char *argv[])
{
    if (argc == 2) {
        uint8_t reg;
        for (reg = 0x00; reg <= 0x14; reg++) {
            print_register(reg);
        }
    } else if (argc == 3) {
        uint8_t reg = strtoul(argv[REG], NULL, 0);
        print_register(reg);
    } else {
        return -1;
    }

    return 0;
}

static int do_write(int argc, FAR char *argv[])
{
    if (argc == 4) {
        uint8_t reg = strtoul(argv[REG], NULL, 0);
        uint8_t val = strtoul(argv[VAL], NULL, 0);
        int ret = bq25896_reg_write(reg, val);
        if (ret) {
            printf("write: reg=0x%02x, val=0x%02x, err=%d\n", reg, val, ret);
        } else {
            print_register(reg);
        }
    } else {
        return -1;
    }

    return 0;
}

static int do_modify(int argc, FAR char *argv[])
{
    if (argc == 5) {
        int ret;
        uint8_t reg = strtoul(argv[REG], NULL, 0);
        uint8_t mask = strtoul(argv[MASK], NULL, 0);
        uint8_t set = strtoul(argv[SET], NULL, 0);
        ret = bq25896_reg_modify(reg, mask, set);
        if (ret) {
            printf("modify: reg=0x%02x, mask=0x%02x, set=0x%02x, err=%d\n", reg, mask, set, ret);
        } else {
            print_register(reg);
        }
    } else {
        return -1;
    }

    return 0;
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int bq25896_main(int argc, char *argv[])
#endif
{
    int ret = bq25896_driver_init(-1, -1);

    if(ret) {
        printf("bq25896 device is NOT available, err=%d\n", ret);
        exit(EXIT_FAILURE);
    }

    if (argc < 2) {
        ret = -1;
        goto done;
    }

    if (!strcmp(argv[OPT], "-r"))
        ret = do_read(argc, argv);
    else if (!strcmp(argv[OPT], "-w"))
        ret = do_write(argc, argv);
    else if (!strcmp(argv[OPT], "-m"))
        ret = do_modify(argc, argv);
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
