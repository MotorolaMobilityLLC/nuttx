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

/*
 * - TODO MSM/SVC UART
 * - TODO "debug" UART
 * - TODO INA231s?
 * - TODO any non-IOEXP, non-INA231 I2C slaves (need to check for them)
 * - TODO UFS module port
 * - board_init, board_exit
 */

#define ARADBG_COMP ARADBG_SVC     /* DBG_COMP macro of the component */

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/util.h>
#include <nuttx/i2c.h>
#include <nuttx/gpio.h>

#include "nuttx/gpio/stm32_gpio_chip.h"
#include "nuttx/gpio/tca64xx.h"

#include <ara_debug.h>
#include "ara_board.h"
#include "interface.h"
#include "tsb_switch_driver_es2.h"
#include "stm32.h"

/*
 * I/O expander config. U4550 is IOEXP1, U4570 is IOEXP2. (These
 * defines match schematic net names.)
 */
#define IOEXP_I2C_BUS           1
#define IOEXP_U4550_I2C_ADDR    0x21
#define IOEXP_U4570_I2C_ADDR    0x20
#define IOEXP1_INT_N            STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN9)
#define IOEXP2_INT_N            STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN10)
#define SVC_RST_IOEXP1          STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN14)
#define SVC_RST_IOEXP2          STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN15)
#define SVC_RST_IOEXP1_GPIO     (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_PULLUP | \
                                 GPIO_PORTC | GPIO_PIN14)
#define SVC_RST_IOEXP2_GPIO     (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_PULLUP | \
                                 GPIO_PORTC | GPIO_PIN15)

/* 19.2 MHz system reference clocks */
#define REFCLK_REQ        STM32_GPIO_PIN(GPIO_PORTB | GPIO_PIN9)
#define REFCLK_BUFFERS_EN U4570_GPIO_PIN(2) /* Shared clock enable for modules */
#define REFCLK_SW_EN      U4570_GPIO_PIN(3) /* Switch */
#define REFCLK_1_EN       U4570_GPIO_PIN(4) /* Modules */
#define REFCLK_2_EN       U4570_GPIO_PIN(5)
#define REFCLK_3A_EN      U4570_GPIO_PIN(6)
#define REFCLK_3B_EN      U4570_GPIO_PIN(7)
#define REFCLK_4A_EN      U4570_GPIO_PIN(8)
#define REFCLK_4B_EN      U4570_GPIO_PIN(9)
#define REFCLK_5_EN       U4570_GPIO_PIN(10)

/* Ara Key switch */
#define ARA_KEY         (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTA | GPIO_PIN0)

/* Wake/detect pins */
#define WD_1_DET_IN       STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN1)
#define WD_2_DET_IN       STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN2)
#define WD_3A_DET_IN      STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN3)
#define WD_3B_DET_IN      STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN4)
#define WD_4A_DET_IN      STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN5)
#define WD_4B_DET_IN      STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN6)
#define WD_5_DET_IN       STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN7) /* LCD */
#define WD_8A_DET_IN      STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN11)
#define WD_8B_DET_IN      STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN12)

/* Module hotplug release request pins */
#define MOD_ACT_SW_1      STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN0)
#define MOD_ACT_SW_2      STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN2)
#define MOD_ACT_SW_3A     STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN3)
#define MOD_ACT_SW_3B     STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN4)
#define MOD_ACT_SW_4A     STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN6)
#define MOD_ACT_SW_4B     STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN7)
#define MOD_ACT_SW_5      STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN8)

/* Module release pins */
#define MOD_RELEASE_1      STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN0)
#define MOD_RELEASE_2      STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN1)
#define MOD_RELEASE_3A     STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN2)
#define MOD_RELEASE_3B     STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN4)
#define MOD_RELEASE_4A     STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN6)
#define MOD_RELEASE_4B     STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN7)
#define MOD_RELEASE_5      STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN7)

/* Switch control pins */
#define SW_STANDBY_N       STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN8)
#define SW_1P1_EN          STM32_GPIO_PIN(GPIO_PORTB | GPIO_PIN0)
#define SW_1P8_IO_EN       STM32_GPIO_PIN(GPIO_PORTB | GPIO_PIN1)
#define SVC_RST_SW         STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN11)
#define SVC_RST_SW_GPIO    (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORTD | GPIO_PIN11)
#define SW_UNIPRO_1P8_PWM  STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN2)
#define SW_IO_1P8_PWM      STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN4)
#define SW_1P8_UNIPRO_EN   STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN8)
#define SW_TO_SVC_INT      STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN9)
#define SW_TO_SVC_INT_GPIO (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | \
                            GPIO_PORTC | GPIO_PIN9)
#define SVC_SW_SPI_CS_GPIO (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_OUTPUT_SET \
                            GPIO_PORTB | GPIO_PIN12)

/* Hardware ID pins */
#define SVC_HWID_0         STM32_GPIO_PIN(GPIO_PORTB | GPIO_PIN5)
#define SVC_HWID_1         STM32_GPIO_PIN(GPIO_PORTB | GPIO_PIN6)
#define SVC_HWID_2         STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN12)
#define SVC_HWID_3         STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN13)

/* VSYS enable */
#define VSYS_EN1_N         U4550_GPIO_PIN(0)
#define VSYS_EN2_N         U4550_GPIO_PIN(2)
#define VSYS_EN3A_N        U4550_GPIO_PIN(4)
#define VSYS_EN3B_N        U4550_GPIO_PIN(6)
#define VSYS_EN4A_N        U4550_GPIO_PIN(8)
#define VSYS_EN4B_N        U4550_GPIO_PIN(10)
#define VSYS_EN5_N         U4550_GPIO_PIN(12)

/* VCHG enable */
#define VCHG_EN1_N         U4550_GPIO_PIN(1)
#define VCHG_EN2_N         U4550_GPIO_PIN(3)
#define VCHG_EN3A_N        U4550_GPIO_PIN(5)
#define VCHG_EN3B_N        U4550_GPIO_PIN(7)
#define VCHG_EN4A_N        U4550_GPIO_PIN(9)
#define VCHG_EN4B_N        U4550_GPIO_PIN(11)
#define VCHG_EN5_N         U4550_GPIO_PIN(13)

/*
 * APBridges regulator list and interface declarations
 *
 * TODO: clean up APB during bringup, UFS if specifically requested.
 *
 * Unlike on other boards and other interfaces on this board, there's
 * nothing to do regulator or clock-wise for the APBridges. The AP
 * controls their power and clock enable lines, and should be able to
 * ensure REFCLK_MAIN is supplied when necessary. So no vreg_data is
 * needed here.
 *
 * However, to be safe for pre-bringup, we:
 *
 * - turn on REFCLK_MAIN unconditionally by asserting REFCLK_REQ at
 *   init time to ensure APB clocks are enabled, and
 *
 * - declare these empty vreg_data arrays to avoid possible crashes
 *   due to having a null vreg_data in an ARA_IFACE_TYPE_MODULE_PORT
 *   interface crop up at an inconvenient time.
 *
 * This should get cleaned up (modules including the REFCLK_REQ in
 * their vreg_data, no separate refclk_main_vreg_data, and NULL APB
 * vreg_data) during bringup when we can test on real HW.
 */

static struct vreg_data apb1_vreg_data[] = {
};

static struct vreg_data apb2_vreg_data[] = {
};

DECLARE_MODULE_PORT_INTERFACE(apb1, apb1_vreg_data, 3,
                              WD_8A_DET_IN, ARA_IFACE_WD_ACTIVE_HIGH);
DECLARE_MODULE_PORT_INTERFACE(apb2, apb2_vreg_data, 1,
                              WD_8B_DET_IN, ARA_IFACE_WD_ACTIVE_HIGH);

/*
 * Modules voltage regulator list and interface declarations.
 *
 * As on DB3, the modules' load switches are controlled by the INA231
 * ALERT output, which should eventually driven via I2C commands to
 * e.g. disable power on overcurrent.
 *
 * Take a shortcut until the relevant INA231-related support is in:
 * directly control it via the VSYS_ENx GPIO pins.
 */

static struct vreg_data module_1_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(VSYS_EN1_N, HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_BUFFERS_EN),
    INIT_MODULE_CLK_DATA(REFCLK_1_EN),
};

static struct vreg_data module_2_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(VSYS_EN2_N, HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_BUFFERS_EN),
    INIT_MODULE_CLK_DATA(REFCLK_2_EN),
};

static struct vreg_data module_3A_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(VSYS_EN3A_N, HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_BUFFERS_EN),
    INIT_MODULE_CLK_DATA(REFCLK_3A_EN),
};

static struct vreg_data module_3B_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(VSYS_EN3B_N, HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_BUFFERS_EN),
    INIT_MODULE_CLK_DATA(REFCLK_3B_EN),
};

static struct vreg_data module_4A_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(VSYS_EN4A_N, HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_BUFFERS_EN),
    INIT_MODULE_CLK_DATA(REFCLK_4A_EN),
};

static struct vreg_data module_4B_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(VSYS_EN4B_N, HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_BUFFERS_EN),
    INIT_MODULE_CLK_DATA(REFCLK_4B_EN),
};

static struct vreg_data module_5_lcd_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(VSYS_EN5_N, HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_BUFFERS_EN),
    INIT_MODULE_CLK_DATA(REFCLK_5_EN),
};

DECLARE_MODULE_PORT_INTERFACE(module_1, module_1_vreg_data, 13,
                              WD_1_DET_IN, ARA_IFACE_WD_ACTIVE_LOW);
DECLARE_MODULE_PORT_INTERFACE(module_2, module_2_vreg_data, 11,
                              WD_2_DET_IN, ARA_IFACE_WD_ACTIVE_LOW);
DECLARE_MODULE_PORT_INTERFACE(module_3A, module_3A_vreg_data, 4,
                              WD_3A_DET_IN, ARA_IFACE_WD_ACTIVE_LOW);
DECLARE_MODULE_PORT_INTERFACE(module_3B, module_3B_vreg_data, 2,
                              WD_3B_DET_IN, ARA_IFACE_WD_ACTIVE_LOW);
DECLARE_MODULE_PORT_INTERFACE(module_4A, module_4A_vreg_data, 6,
                              WD_4A_DET_IN, ARA_IFACE_WD_ACTIVE_LOW);
DECLARE_MODULE_PORT_INTERFACE(module_4B, module_4B_vreg_data, 8,
                              WD_4B_DET_IN, ARA_IFACE_WD_ACTIVE_LOW);
DECLARE_MODULE_PORT_INTERFACE(module_5_lcd, module_5_lcd_vreg_data, 10,
                              WD_5_DET_IN, ARA_IFACE_WD_ACTIVE_LOW);

static struct interface *evt1_interfaces[] = {
    &apb1_interface,
    &apb2_interface,
    &module_1_interface,
    &module_2_interface,
    &module_3A_interface,
    &module_3B_interface,
    &module_4A_interface,
    &module_4B_interface,
    &module_5_lcd_interface,
};

/*
 * Switch vreg
 */

static struct vreg_data sw_vreg_data[] = { /* TODO */ };
DECLARE_VREG(sw, sw_vreg_data);

/*
 * I/O expanders
 */

static struct io_expander_info evt1_io_expanders[] = {
    {
        .part      = TCA6416_PART,
        .i2c_bus   = IOEXP_I2C_BUS,
        .i2c_addr  = IOEXP_U4550_I2C_ADDR,
        .reset     = SVC_RST_IOEXP1,
        /* TODO: U4550 IRQs are needed when the VSYS etc. pins are
         * inputs and the INA231 actually supplies VSYS. This is so
         * the ALERT pin can be used as an IRQ source on
         * overcurrent. */
        .irq       = TCA64XX_IO_UNUSED,
        .gpio_base = U4550_GPIO_CHIP_START,
    },
    {
        .part      = TCA6416_PART,
        .i2c_bus   = IOEXP_I2C_BUS,
        .i2c_addr  = IOEXP_U4570_I2C_ADDR
        .reset     = SVC_RST_IOEXP2,
        .irq       = TCA64XX_IO_UNUSED,
        .gpio_base = U4570_GPIO_CHIP_START,
    },
};

/*
 * Board info, init, and exit
 */

static struct ara_board_info evt1_board_info = {
    .interfaces = evt1_interfaces,
    .nr_interfaces = ARRAY_SIZE(evt1_interfaces),
    .nr_spring_interfaces = 0,

    .sw_data = {
        .vreg            = &sw_vreg,
        .gpio_reset      = SVC_RST_SW_GPIO,
        .gpio_irq        = SW_TO_SVC_INT_GPIO,
        .irq_rising_edge = false,
        .rev             = SWITCH_REV_ES2,
        .bus             = SW_SPI_PORT_2,
        .spi_cs          = SVC_SW_SPI_CS_GPIO,
    },

    .io_expanders = evt1_io_expanders,
    .nr_io_expanders = ARRAY_SIZE(evt1_io_expanders),
};

static struct vreg_data refclk_main_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_REQ);
}

DECLARE_VREG(refclk_main, refclk_main_vreg_data);

struct ara_board_info *board_init(void) {
    int rc;

    /* For now, just always enable REFCLK_MAIN. */
    rc = vreg_config(&refclk_main_vreg) ||
        vreg_get(&refclk_main_vreg);
    if (rc) {
        dbg_error("%s: can't start REFCLK_MAIN\n", __func__);
        return NULL;
    }

    return NULL; /* TODO -- rest of board_init() */
}

void board_exit(void) { dbg_error("%s: TODO, unimplemented\n", __func__); }
