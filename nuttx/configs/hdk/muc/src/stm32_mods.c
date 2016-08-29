/****************************************************************************
 *
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
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
 *
 ****************************************************************************/

#include <arch/board/mods.h>
#include <arch/irq.h>

#include <nuttx/arch.h>
#include <nuttx/config.h>
#include <nuttx/gpio.h>

#include <debug.h>
#include <errno.h>
#include <unistd.h>

/*
 * Handler for the pcard detection interrupt. When the pcard is attached or
 * detached, the HDK will reset so the proper firmware can be reloaded.
 *
 * You can remove this function when making your own mod.
 */
static int pcard_isr(int irq, void *context)
{
    dbg("reset\n");
    up_systemreset(); /* will not return */

    return OK;
}

void mods_init(void)
{
    gpio_direction_out(GPIO_MODS_INT, 0);
    /*
     * The code below is specific to the HDK for supporting pcards. You should
     * remove this code when making your own mod.
     */
    gpio_irqattach(GPIO_MODS_PCARD_DET_N, pcard_isr);
    set_gpio_triggering(GPIO_MODS_PCARD_DET_N, IRQ_TYPE_EDGE_BOTH);
}

void mods_host_int_set(bool value)
{
    gpio_set_value(GPIO_MODS_INT, value ? 1 : 0);
}

