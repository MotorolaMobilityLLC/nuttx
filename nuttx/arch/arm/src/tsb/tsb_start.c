/*
 * Copyright (c) 2014-2015 Google, Inc.
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
 * * may be used to endorse or promote products derived from this
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
#include <nuttx/init.h>
#include <arch/board/board.h>
#include <arch/tsb/isaa.h>
#include "up_arch.h"
#include "up_internal.h"
#include "ram_vectors.h"
#include "nvic.h"
#include "syslog.h"

#include "tsb_scm.h"
#include "tsb_lowputc.h"

#ifdef DEBUG_EARLY_BOOT
#define dbg(x) up_lowputc(x)
#else
#define dbg(x)
#endif

void tsb_start(void);
static void copy_data_section_to_ram(void);

extern uint32_t _stext_lma;
extern uint32_t _sdata_lma;

void __start(void) __attribute__((section(".bootstrap.loader")));

void __start(void)
{
    extern void bootstrap(void);
    bootstrap();
    copy_data_section_to_ram();

    tsb_start();
}

static void copy_data_section_to_ram(void)
{
    uint32_t *dst;
    const uint32_t *src;

    for (src = &_sdata_lma, dst = &_sdata; dst < &_edata;) {
        *dst++ = *src++;
    }
}

void tsb_start(void) {
    uint32_t *dst;
    __attribute__((unused)) int retval;

    /* Zero .bss */
    for (dst = &_sbss; dst < &_ebss;) {
        *dst++ = 0;
    }

    /* Relocate vector table (eg from bootrom) */
    extern uint32_t _vectors;
    putreg32((uint32_t)&_vectors, NVIC_VECTAB);

    /* Configure clocks */
    tsb_clk_init();

#ifdef CONFIG_UART_SERIAL_CONSOLE
    /*
     * If we're using the serial console, configure the UART so we can get
     * debug output as soon as possible.
     */
    tsb_lowsetup();
    dbg('A');
#endif

#ifdef CONFIG_TSB_PINSHARE_ETM
    retval = tsb_request_pinshare(TSB_PIN_ETM);
    if (retval) {
        lowsyslog("ETM: cannot get ownership of ETM pin.\n");
    } else {
        tsb_set_pinshare(TSB_PIN_ETM);

#ifdef CONFIG_TSB_TRACE_DRIVESTRENGTH_MIN
        tsb_set_drivestrength(TSB_TRACE_DRIVESTRENGTH, tsb_ds_min);
#elif CONFIG_TSB_TRACE_DRIVESTRENGTH_DEFAULT
        tsb_set_drivestrength(TSB_TRACE_DRIVESTRENGTH, tsb_ds_default);
#elif CONFIG_TSB_TRACE_DRIVESTRENGTH_MAX
        tsb_set_drivestrength(TSB_TRACE_DRIVESTRENGTH, tsb_ds_max);
#endif
    }

#endif

#ifdef CONFIG_16550_UART
    /* early init the 16550 driver */
    up_earlyserialinit();
#endif
    dbg('D');

#ifdef CONFIG_TSB_JTAG_ENABLE
    tsb_isaa_enable_jtag();
#endif

    tsb_boardinitialize();

    os_start();
}
