/*
 * Copyright (C) 2016 Motorola Mobility. All rights reserved.
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

#include <nuttx/util.h>
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
#define TSB_CLK_HSIC480         (TSB_SCM_REG1 | 3)
#define TSB_CLK_HSICBUS         (TSB_SCM_REG1 | 4)
#define TSB_CLK_HSICREF         (TSB_SCM_REG1 | 5)
#define TSB_CLK_BOOTROM         (TSB_SCM_REG1 | 6)
#define TSB_CLK_VIDENCRYPT      (TSB_SCM_REG1 | 7)
#define TSB_CLK_BUFCRYPT        (TSB_SCM_REG1 | 8)

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
#define TSB_RST_VIDCRYPTIF      (TSB_SCM_REG1 | 6)
#define TSB_RST_VIDCRYPTCH0     (TSB_SCM_REG1 | 7)
#define TSB_RST_VIDCRYPTCH1     (TSB_SCM_REG1 | 8)
#define TSB_RST_BUFCRYPT        (TSB_SCM_REG1 | 9)

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
#define TSB_PIN_UART_RXTX       BIT(0)
#define TSB_PIN_UART_CTSRTS     BIT(1)
#define TSB_PIN_SDIO            BIT(2)
#define TSB_PIN_GPIO9           BIT(3)
#define TSB_PIN_ETM             BIT(4)
#define TSB_PIN_GPIO10          BIT(5)
#define TSB_PIN_GPIO13          BIT(6)
#define TSB_PIN_GPIO15          BIT(7)
#define TSB_PIN_GPIO16          BIT(10)
#define TSB_PIN_GPIO18          BIT(11)
#define TSB_PIN_GPIO19          BIT(12)
#define TSB_PIN_GPIO20          BIT(13)
#define TSB_PIN_GPIO21          BIT(14)
#define TSB_PIN_GPIO22          BIT(15)

/* System Conf bits */
#define TSB_SYS_CONF_WDT_IRQ_EN BIT(1)

/* IO_DRIVE_STRENGTH: 2 Bits per Output */

enum tsb_drivestrength {
    tsb_ds_min     = 0,
    tsb_ds_default,
    tsb_ds_max,
    tsb_ds_invalid,
};

#define DRIVESTRENGTH_OFFSET(driver) (((driver >> 22) & 0xfc))
#define DRIVESTRENGTH_SHIFT(driver)  (driver & 0x1f)
#define DRIVESTRENGTH_MASK(driver)  (0x3 << DRIVESTRENGTH_SHIFT(driver))

#ifdef CONFIG_TSB_CHIP_REV_ES2
#define TSB_TRACE_DRIVESTRENGTH   (TSB_SCM_REG1 | 24)
#define TSB_SPI_DRIVESTRENGTH     (TSB_SCM_REG1 | 22)
#define TSB_PWM_DRIVESTRENGTH     (TSB_SCM_REG1 | 20)
#define TSB_I2S_DRIVESTRENGTH     (TSB_SCM_REG1 | 18)
#else
#define TSB_TRACE_DRIVESTRENGTH   (TSB_SCM_REG2 | 10)
#define TSB_SPI_DRIVESTRENGTH     (TSB_SCM_REG2 | 8)
#define TSB_PWM1_DRIVESTRENGTH    (TSB_SCM_REG2 | 6)
#define TSB_PWM0_DRIVESTRENGTH    (TSB_SCM_REG2 | 4)
#define TSB_I2S_DRIVESTRENGTH     (TSB_SCM_REG2 | 0)
#endif

/*
 * SCM PID Register values from efuse data (very early silicon reads 0)
 * Note: In ES1 documentation, this register was named MODULEID1.
 */

enum tsb_product_id {
    tsb_pid_unknown = 0,
    tsb_pid_apbridge = 1,
    tsb_pid_gpbridge = 2,
    tsb_pid_switch = 3
};

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
     CLK_MASK(TSB_CLK_GPIO))


uint32_t tsb_get_debug_reg(uint32_t reg);
void tsb_clk_init(void);
void tsb_clk_enable(uint32_t clk);
void tsb_clk_disable(uint32_t clk);
uint32_t tsb_clk_status(uint32_t clk);
void tsb_clk_dump(void);
void tsb_reset(uint32_t rst);
int tsb_set_pinshare(uint32_t pin);
int tsb_clr_pinshare(uint32_t pin);
uint32_t tsb_get_pinshare(void);
int tsb_set_system_conf(uint32_t bits);
uint32_t tsb_get_system_conf(void);
void tsb_set_drivestrength(uint32_t ds_id, enum tsb_drivestrength value);
enum tsb_drivestrength tsb_get_drivestrength(uint32_t ds_id);
uint32_t tsb_get_vendor_id(void);
enum tsb_product_id tsb_get_product_id(void);
int tsb_request_pinshare(uint32_t bits);
int tsb_release_pinshare(uint32_t bits);
void tsb_get_chip_id(uint32_t *id0, uint32_t *id1, uint32_t *id2);
void tsb_set_spi_clock(uint32_t freq_bits);

#endif /* __ARCH_ARM_SRC_TSB_TSB_SCM_H */
