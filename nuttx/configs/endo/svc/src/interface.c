/*
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
 */

/**
 * @brief: Functions and definitions for interface block management.
 * !! Thin and temporary layer between up_power.h and interface.h       !!
 * !! To be replaced with the full-featured interface abstraction layer !!
 */

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include "up_power.h"
#include "interface.h"

static struct interface interfaces[] = {
    { "Spring A", 0 },
    { "Spring B", 1 },
    { "Spring C", 2 },
    { "Spring D", 3 },
    { "Spring E", 4 },
    { "Spring F", 5 },
    { "Spring G", 6 },
    { "Spring H", 7 },
    { "Spring I", 8 },
    { "Spring J", 9 },
    { "Spring K", 10 },
    { "Spring L", 11 },
    { "Spring M", 12 },
    { "Spring N", 13 },
};


struct interface* interface_get(uint8_t index)
{
    if (index >= PWR_SPRING_NR)
        return NULL;

    return &interfaces[index];
}

int interface_generate_wakeout(struct interface *iface, bool assert)
{
    power_set_wakeout(1 << iface->nr, assert);

    return 0;
}
