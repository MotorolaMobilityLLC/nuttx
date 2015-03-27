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

/*
 * Author: Benoit Cousson <bcousson@baylibre.com>
 *         Fabien Parent <fparent@baylibre.com>
 */

#ifndef __ARCH_ARM_SRC_TSB_TSB_SCM_H
#define __ARCH_ARM_SRC_TSB_TSB_SCM_H

#include "chip.h"


#define TSB_SCM_REG0  (0 << 24)
#define TSB_SCM_REG1  (1 << 24) /* I2S / Unipro / HSIC */
#define TSB_SCM_REG2  (2 << 24) /* CDSI */


/* Clock bit position for CLOCKGATING/CLOCKENABLE registers */

/* Register 0 */
#define TSB_CLK_WORKRAM         (TSB_SCM_REG0 | 8)
#define TSB_CLK_SDIOSYS         (TSB_SCM_REG0 | 9)
#define TSB_CLK_SDIOSD          (TSB_SCM_REG0 | 10)
#define TSB_CLK_BUFRAM          (TSB_SCM_REG0 | 11)
#define TSB_CLK_MBOX            (TSB_SCM_REG0 | 12)
#define TSB_CLK_GDMA            (TSB_SCM_REG0 | 13)
#define TSB_CLK_TMR             (TSB_SCM_REG0 | 14)
#define TSB_CLK_WDT             (TSB_SCM_REG0 | 15)
#define TSB_CLK_TIMER           (TSB_SCM_REG0 | 16)
#define TSB_CLK_PWMODP          (TSB_SCM_REG0 | 17)
#define TSB_CLK_PWMODS          (TSB_SCM_REG0 | 18)
#define TSB_CLK_I2CP            (TSB_SCM_REG0 | 19)
#define TSB_CLK_I2CS            (TSB_SCM_REG0 | 20)
#define TSB_CLK_UARTP           (TSB_SCM_REG0 | 21)
#define TSB_CLK_UARTS           (TSB_SCM_REG0 | 22)
#define TSB_CLK_SPIP            (TSB_SCM_REG0 | 23)
#define TSB_CLK_SPIS            (TSB_SCM_REG0 | 24)
#define TSB_CLK_GPIO            (TSB_SCM_REG0 | 25)

/* Register 1 */
#define TSB_CLK_I2SSYS          (TSB_SCM_REG1 | 0)
#define TSB_CLK_I2SBIT          (TSB_SCM_REG1 | 1)
#define TSB_CLK_UNIPROSYS       (TSB_SCM_REG1 | 2)
#define TSB_CLK_UNIPROREF       (TSB_SCM_REG1 | 3)
#define TSB_CLK_HSIC480         (TSB_SCM_REG1 | 4)
#define TSB_CLK_HSICBUS         (TSB_SCM_REG1 | 5)
#define TSB_CLK_HSICREF         (TSB_SCM_REG1 | 6)

/* Register 2 (APBridge only) */
#define TSB_CLK_CDSI0_TX_SYS    (TSB_SCM_REG2 | 0)
#define TSB_CLK_CDSI0_TX_APB    (TSB_SCM_REG2 | 1)
#define TSB_CLK_CDSI0_RX_SYS    (TSB_SCM_REG2 | 2)
#define TSB_CLK_CDSI0_RX_APB    (TSB_SCM_REG2 | 3)
#define TSB_CLK_CDSI0_REF       (TSB_SCM_REG2 | 4)
#define TSB_CLK_CDSI1_TX_SYS    (TSB_SCM_REG2 | 5)
#define TSB_CLK_CDSI1_TX_APB    (TSB_SCM_REG2 | 6)
#define TSB_CLK_CDSI1_RX_SYS    (TSB_SCM_REG2 | 7)
#define TSB_CLK_CDSI1_RX_APB    (TSB_SCM_REG2 | 8)
#define TSB_CLK_CDSI1_REF       (TSB_SCM_REG2 | 9)


/* Reset bit position SOFTRESET/SOFTRELEASE */

/* Register 0 */
#define TSB_RST_WORKRAM         (TSB_SCM_REG0 | 5)
#define TSB_RST_SDIOSYS         (TSB_SCM_REG0 | 6)
#define TSB_RST_SDIOSD          (TSB_SCM_REG0 | 7)
#define TSB_RST_BUFRAM          (TSB_SCM_REG0 | 8)
#define TSB_RST_MBOX            (TSB_SCM_REG0 | 9)
#define TSB_RST_GDMA            (TSB_SCM_REG0 | 10)
#define TSB_RST_TMR             (TSB_SCM_REG0 | 11)
#define TSB_RST_WDT             (TSB_SCM_REG0 | 12)
#define TSB_RST_TIMER           (TSB_SCM_REG0 | 13)
#define TSB_RST_PWMODP          (TSB_SCM_REG0 | 14)
#define TSB_RST_PWMODS          (TSB_SCM_REG0 | 15)
#define TSB_RST_I2CP            (TSB_SCM_REG0 | 16)
#define TSB_RST_I2CS            (TSB_SCM_REG0 | 17)
#define TSB_RST_UARTP           (TSB_SCM_REG0 | 18)
#define TSB_RST_UARTS           (TSB_SCM_REG0 | 19)
#define TSB_RST_SPIP            (TSB_SCM_REG0 | 20)
#define TSB_RST_SPIS            (TSB_SCM_REG0 | 21)
#define TSB_RST_GPIO            (TSB_SCM_REG0 | 22)

/* Register 1 */
#define TSB_RST_I2SSYS          (TSB_SCM_REG1 | 0)
#define TSB_RST_I2SBIT          (TSB_SCM_REG1 | 1)
#define TSB_RST_UNIPROSYS       (TSB_SCM_REG1 | 2) /* Assert > 1us */
#define TSB_RST_HSIC            (TSB_SCM_REG1 | 3)
#define TSB_RST_HSICPHY         (TSB_SCM_REG1 | 4)
#define TSB_RST_HSICPOR         (TSB_SCM_REG1 | 5)

/* Register 2 (APBridge only) */
#define TSB_RST_CDSI0_TX        (TSB_SCM_REG2 | 0)
#define TSB_RST_CDSI0_RX        (TSB_SCM_REG2 | 1)
#define TSB_RST_CDSI0_TX_AIO    (TSB_SCM_REG2 | 2)
#define TSB_RST_CDSI0_RX_AIO    (TSB_SCM_REG2 | 3)
#define TSB_RST_CDSI1_TX        (TSB_SCM_REG2 | 4)
#define TSB_RST_CDSI1_RX        (TSB_SCM_REG2 | 5)
#define TSB_RST_CDSI1_TX_AIO    (TSB_SCM_REG2 | 6)
#define TSB_RST_CDSI1_RX_AIO    (TSB_SCM_REG2 | 7)


/* PINSHARE bits */
#define TSB_PIN_UART_RXTX       (1 << 0)
#define TSB_PIN_UART_CTSRTS     (1 << 1)
#define TSB_PIN_SDIO            (1 << 2)
#define TSB_PIN_GPIO9           (1 << 3)
#define TSB_PIN_ETM             (1 << 4)
#define TSB_PIN_GPIO10          (1 << 5)
#define TSB_PIN_GPIO13          (1 << 6)
#define TSB_PIN_GPIO15          (1 << 7)
#define TSB_PIN_SDCARD          (1 << 8)
#define TSB_PIN_DBG             (1 << 9)


#define CLK_OFFSET(clk) (((clk >> 22) & 0xfc))
#define CLK_MASK(clk)   (1 << (clk & 0x1f))


#define INIT_PERIPHERAL_CLOCK_BITS \
    (CLK_MASK(TSB_CLK_SDIOSYS) | \
     CLK_MASK(TSB_CLK_SDIOSD)  | \
     CLK_MASK(TSB_CLK_MBOX)    | \
     CLK_MASK(TSB_CLK_GDMA)    | \
     CLK_MASK(TSB_CLK_TMR)     | \
     CLK_MASK(TSB_CLK_WDT)     | \
     CLK_MASK(TSB_CLK_TIMER)   | \
     CLK_MASK(TSB_CLK_PWMODP)  | \
     CLK_MASK(TSB_CLK_PWMODS)  | \
     CLK_MASK(TSB_CLK_I2CP)    | \
     CLK_MASK(TSB_CLK_I2CS)    | \
     CLK_MASK(TSB_CLK_UARTP)   | \
     CLK_MASK(TSB_CLK_UARTS)   | \
     CLK_MASK(TSB_CLK_SPIP)    | \
     CLK_MASK(TSB_CLK_SPIS)    | \
     CLK_MASK(TSB_CLK_GPIO))



void tsb_clk_init(void);
void tsb_clk_enable(uint32_t clk);
void tsb_clk_disable(uint32_t clk);
uint32_t tsb_clk_status(uint32_t clk);
void tsb_clk_dump(void);
void tsb_reset(uint32_t rst);
void tsb_set_pinshare(uint32_t pin);
void tsb_clr_pinshare(uint32_t pin);

#endif /* __ARCH_ARM_SRC_TSB_TSB_SCM_H */
