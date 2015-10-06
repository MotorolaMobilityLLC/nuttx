/**
 * Copyright (c) 2015 Google Inc.
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
 *
 * @author Mark Greer
 * @brief Dump Device Core device information
 */

#include <stdio.h>
#include <stdlib.h>

#include <nuttx/util.h>
#include <nuttx/device.h>
#include <nuttx/device_table.h>

#define GET_STR(s)  ((s) ? (s) : "<empty>")

static struct {
    enum device_state   state;
    char                *state_str;
} state_to_str[] = {
    {
        .state      = DEVICE_STATE_REMOVED,
        .state_str  = "REMOVED",
    },
    {
        .state      = DEVICE_STATE_PROBING,
        .state_str  = "PROBING",
    },
    {
        .state      = DEVICE_STATE_PROBED,
        .state_str  = "PROBED",
    },
    {
        .state      = DEVICE_STATE_OPENING,
        .state_str  = "OPENING",
    },
    {
        .state      = DEVICE_STATE_OPEN,
        .state_str  = "OPEN",
    },
    {
        .state      = DEVICE_STATE_CLOSING,
        .state_str  = "CLOSING",
    },
    {
        .state      = DEVICE_STATE_REMOVING,
        .state_str  = "REMOVING",
    },
};

static char *dev_state_to_str(enum device_state state)
{
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(state_to_str); i++) {
        if (state_to_str[i].state == state) {
            return state_to_str[i].state_str;
        }
    }

    return "UNKNOWN";
}

static struct {
    enum device_resource_type   restype;
    char                        *restype_str;
} restype_to_str[] = {
    {
        .restype        = DEVICE_RESOURCE_TYPE_REGS,
        .restype_str    = "REG",
    },
    {
        .restype        = DEVICE_RESOURCE_TYPE_IRQ,
        .restype_str    = "IRQ",
    },
    {
        .restype        = DEVICE_RESOURCE_TYPE_GPIO,
        .restype_str    = "GPIO",
    },
};

static char *dev_restype_to_str(enum device_resource_type restype)
{
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(restype_to_str); i++) {
        if (restype_to_str[i].restype == restype) {
            return restype_to_str[i].restype_str;
        }
    }

    return "UNKNOWN";
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int dev_info_main(int argc, char *argv[])
#endif
{
    struct device *dev;
    struct device_table_iter iter = DEVICE_TABLE_ITER_INITIALIZER;
    unsigned int i;
    char *type, *name, *desc;

    printf("\n");

    device_table_for_each_dev(dev, &iter) {
        type = GET_STR(device_get_type(dev));
        name = GET_STR(device_get_name(dev));
        desc = GET_STR(device_get_desc(dev));

        printf("%s %u:\n", type, device_get_id(dev));
        printf("  name:\t\t%s (%s)\n", name, desc);
        printf("  state:\t%s\n", dev_state_to_str(device_get_state(dev)));

        if (device_driver_is_attached(dev)) {
            name = GET_STR(device_driver_get_name(dev));
            desc = GET_STR(device_driver_get_desc(dev));

            printf("  driver:\t%s (%s)\n", name, desc);
        }

        printf("  resources:\n");

        for (i = 0; i < device_get_resource_count(dev); i++) {
            name = GET_STR(device_resource_get_name(dev, i));

            printf("    %s\t%s\t0x%08x\t0x%x\n", name,
                   dev_restype_to_str(device_resource_get_type(dev, i)),
                   device_resource_get_start(dev, i),
                   device_resource_get_count(dev, i));
        }

        printf("\n");
    }

    return EXIT_SUCCESS;
}
