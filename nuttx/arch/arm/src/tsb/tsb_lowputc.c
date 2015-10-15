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
#include <arch/arm/semihosting.h>
#include <stdint.h>
#include <stdbool.h>
#include "chip.h"
#include "up_arch.h"

#include "tsb_scm.h"

#define UART_RBR_THR_DLL    (UART_BASE + 0x0)
#define UART_IER_DLH        (UART_BASE + 0x4)
#define UART_FCR_IIR        (UART_BASE + 0x8)
#define UART_LCR            (UART_BASE + 0xc)
#define UART_LSR            (UART_BASE + 0x14)

#define UART_115200_BPS     26
#define UART_DLL_115200     ((UART_115200_BPS >> 0) & 0xff)
#define UART_DLH_115200     ((UART_115200_BPS >> 8) & 0xff)
#define UART_LCR_DLAB  (0x1 << 7) // Divisor latch
#define UART_LCR_DLS_8 (0x3 << 0) // 8 bit

#define UART_FCR_IIR_IID0_FIFOE  (1 << 0) // FIFO Enable
#define UART_FCR_IIR_IID1_RFIFOR (1 << 1) // FIFO RX Reset
#define UART_FCR_IIR_IID1_XFIFOR (1 << 2) // FIFO RX Reset

#define UART_LSR_THRE (0x1 << 5)

static bool own_pinshare;

void tsb_lowsetup(void) {
    int retval;
    int i;

    /* enable UART RX/TX pins */
    retval = tsb_request_pinshare(TSB_PIN_UART_RXTX);
    if (retval) {
        return;
    }

    tsb_set_pinshare(TSB_PIN_UART_RXTX);
    own_pinshare = true;

    /* enable UART clocks */
    tsb_clk_enable(TSB_CLK_UARTP);
    tsb_clk_enable(TSB_CLK_UARTS);

    /* reset UART module */
    tsb_reset(TSB_RST_UARTP);
    tsb_reset(TSB_RST_UARTS);

    /*
     * The controller requires "several cycles" after reset to stabilize before
     * register writes will work. Try this a few times.
     *
     * And the LORD spake, saying, "First shalt thou take out the Holy Pin,
     * then shalt thou count to three, no more, no less. Three shall be the
     * number thou shalt count, and the number of the counting shall be three.
     * Four shalt thou not count, neither count thou two, excepting that thou
     * then proceed to three. Five is right out. Once the number three, being
     * the third number, be reached, then lobbest thou thy Holy Hand Grenade
     * of Antioch towards thy foe, who being naughty in My sight, shall snuff
     * it.
     *      -The Book of Armaments
     */
    for (i = 0; i < 3; i++) {
        putreg32(UART_LCR_DLAB | UART_LCR_DLS_8, UART_LCR);
        putreg32(UART_DLL_115200, UART_RBR_THR_DLL);
        putreg32(UART_DLH_115200, UART_IER_DLH);
        putreg32(UART_LCR_DLS_8, UART_LCR);
        putreg32(0x0, UART_IER_DLH);
        putreg32(UART_FCR_IIR_IID0_FIFOE | UART_FCR_IIR_IID1_RFIFOR |
                 UART_FCR_IIR_IID1_XFIFOR, UART_FCR_IIR);
    }
}

void up_lowputc(int c){
#if defined(CONFIG_ARM_SEMIHOSTING)
    semihosting_putc(c);
#elif defined(CONFIG_APB_USB_LOG)
    usb_putc(c);
#else
    if (!own_pinshare)
        return;

    while ((getreg32(UART_LSR) & UART_LSR_THRE) != UART_LSR_THRE)
        ;

    putreg32(c, UART_RBR_THR_DLL);
#endif
}


#ifdef CONFIG_16550_NO_SERIAL_CONSOLE

int up_putc(int ch) {
    if (ch == '\n') {
        up_lowputc('\r');
    }

    up_lowputc(ch);
    return ch;
}

#endif

#ifndef CONFIG_16550_UART

void up_serialinit(void)
{
}

#endif
