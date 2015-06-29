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
 * @author: Perry Hung
 */

#define DBG_COMP DBG_SVC     /* DBG_COMP macro of the component */

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/util.h>
#include <nuttx/i2c.h>

#include "nuttx/gpio/stm32_gpio_chip.h"
#include "nuttx/gpio/tca64xx.h"

#include "up_debug.h"
#include "ara_board.h"
#include "interface.h"
#include "tsb_switch_driver_es1.h"
#include "stm32.h"
#include <up_adc.h>

#define SWITCH_I2C_BUS      (2)

#define SVC_LED_RED         (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_PORTA | \
                             GPIO_OUTPUT_SET | GPIO_PIN7)

/* U96 I/O Expander reset */
#define IO_RESET            (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_PULLUP | \
                             GPIO_PORTE | GPIO_PIN0)

/* U90 I/O Expander reset */
#define IO_RESET1           (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_PULLUP | \
                             GPIO_PORTE | GPIO_PIN1)

/* Main I/O Expander IRQ from U96 to SVC */
#define U96_IO_EXP_IRQ      STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN0)

/* I/O Expander IRQ from U90 cascaded to U96 */
#define U90_IO_EXP_IRQ      U96_GPIO_PIN(7)

/* I/O Expanders: I2C bus and addresses */
#define IOEXP_I2C_BUS       2
#define IOEXP_U90_I2C_ADDR  0x21
#define IOEXP_U96_I2C_ADDR  0x20
#define IOEXP_U135_I2C_ADDR 0x23

/*
 * How long to leave hold each regulator before the next.
 */
#define HOLD_TIME_1P1                   (50000) // 0-100ms before 1p2, 1p8
#define HOLD_TIME_1P8                   (0)
#define HOLD_TIME_1P2                   (0)

#define HOLD_TIME_SW_1P1                (50000) // 50ms for 1P1
#define HOLD_TIME_SW_1P8                (10000) // 10ms for 1P8
#define POWER_SWITCH_OFF_STAB_TIME_US   (10000) // 10ms switch off

#define WAKEOUT_APB1    (GPIO_FLOAT | GPIO_PORTE | GPIO_PIN8)
#define WAKEOUT_APB2    (GPIO_FLOAT | GPIO_PORTE | GPIO_PIN12)
#define WAKEOUT_APB3    (GPIO_FLOAT | GPIO_PORTA | GPIO_PIN4)
#define WAKEOUT_GPB1    (GPIO_FLOAT | GPIO_PORTH | GPIO_PIN11)
#define WAKEOUT_GPB2    (GPIO_FLOAT | GPIO_PORTH | GPIO_PIN12)
#define WAKEOUT_SPRING1 (GPIO_FLOAT | GPIO_PORTH | GPIO_PIN13)
#define WAKEOUT_SPRING2 (GPIO_FLOAT | GPIO_PORTE | GPIO_PIN10)
#define WAKEOUT_SPRING3 (GPIO_FLOAT | GPIO_PORTH | GPIO_PIN14)
#define WAKEOUT_SPRING4 (GPIO_FLOAT | GPIO_PORTC | GPIO_PIN15)
#define WAKEOUT_SPRING5 (GPIO_FLOAT | GPIO_PORTG | GPIO_PIN8)
#define WAKEOUT_SPRING6 (GPIO_FLOAT | GPIO_PORTB | GPIO_PIN15)
#define WAKEOUT_SPRING7 (GPIO_FLOAT | GPIO_PORTH | GPIO_PIN10)
#define WAKEOUT_SPRING8 (GPIO_FLOAT | GPIO_PORTH | GPIO_PIN15)

/* ADC Channels and GPIOs used for Spring Current Measurements */
#define SPRING_COUNT            8

#define SPRING1_ADC             ADC1
#define SPRING2_ADC             ADC1
#define SPRING3_ADC             ADC1
#define SPRING4_ADC             ADC3
#define SPRING5_ADC             ADC3
#define SPRING6_ADC             ADC3
#define SPRING7_ADC             ADC3
#define SPRING8_ADC             ADC1

#define SPRING1_SENSE_CHANNEL   14    /* PC4 is ADC1_IN14 */
#define SPRING2_SENSE_CHANNEL   10    /* PC0 is ADC1_IN10 */
#define SPRING3_SENSE_CHANNEL   15    /* PC5 is ADC1_IN15 */
#define SPRING4_SENSE_CHANNEL   7     /* PF9 is ADC3_IN7 */
#define SPRING5_SENSE_CHANNEL   6     /* PF8 is ADC3_IN6 */
#define SPRING6_SENSE_CHANNEL   14    /* PF4 is ADC3_IN14 */
#define SPRING7_SENSE_CHANNEL   15    /* PF5 is ADC3_IN15 */
#define SPRING8_SENSE_CHANNEL   11    /* PC1 is ADC1_IN11 */

#define SPRING1_SIGN_PIN (GPIO_INPUT | GPIO_PORTD | GPIO_PIN0)  /* PD0 */
#define SPRING2_SIGN_PIN (GPIO_INPUT | GPIO_PORTG | GPIO_PIN0)  /* PG0 */
#define SPRING3_SIGN_PIN (GPIO_INPUT | GPIO_PORTD | GPIO_PIN1)  /* PD1 */
#define SPRING4_SIGN_PIN (GPIO_INPUT | GPIO_PORTG | GPIO_PIN5)  /* PG5 */
#define SPRING5_SIGN_PIN (GPIO_INPUT | GPIO_PORTG | GPIO_PIN1)  /* PG1 */
#define SPRING6_SIGN_PIN (GPIO_INPUT | GPIO_PORTF | GPIO_PIN2)  /* PF2 */
#define SPRING7_SIGN_PIN (GPIO_INPUT | GPIO_PORTF | GPIO_PIN13) /* PF13 */
#define SPRING8_SIGN_PIN (GPIO_INPUT | GPIO_PORTA | GPIO_PIN8)  /* PA8 */

/*
 * Built-in bridge voltage regulator list
 */
static struct vreg_data apb1_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN4),
                               HOLD_TIME_1P1),
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN5),
                               HOLD_TIME_1P8),
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN6),
                               HOLD_TIME_1P2),
};

static struct vreg_data apb2_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN11),
                               HOLD_TIME_1P1),
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN10),
                               HOLD_TIME_1P8),
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN7),
                               HOLD_TIME_1P2),
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN6),
                               0),  // 2p8_tp
};

static struct vreg_data apb3_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN12),
                               HOLD_TIME_1P1),
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN15),
                               HOLD_TIME_1P8),
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN13),
                               HOLD_TIME_1P2),
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN7),
                               0),  // 2p8
};

static struct vreg_data gpb1_vreg_data[] =  {
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTB | GPIO_PIN5),
                               HOLD_TIME_1P1),
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN3),
                               HOLD_TIME_1P8),
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN2),
                               HOLD_TIME_1P2),
};

static struct vreg_data gpb2_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN7),
                               HOLD_TIME_1P1),
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN9),
                               HOLD_TIME_1P8),
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN8),
                               HOLD_TIME_1P2),
};

/*
 * Interfaces on this board
 */
DECLARE_INTERFACE(apb1, apb1_vreg_data, 0, WAKEOUT_APB1);
DECLARE_INTERFACE(apb2, apb2_vreg_data, 1, WAKEOUT_APB2);
DECLARE_INTERFACE(apb3, apb3_vreg_data, 2, WAKEOUT_APB3);
DECLARE_INTERFACE(gpb1, gpb1_vreg_data, 3, WAKEOUT_GPB1);
DECLARE_INTERFACE(gpb2, gpb2_vreg_data, 4, WAKEOUT_GPB2);

#define SPRING_INTERFACES_COUNT     8
DECLARE_SPRING_INTERFACE(1, STM32_GPIO_PIN(GPIO_PORTI | GPIO_PIN2), 9,
                         SPRING1_ADC, SPRING1_SENSE_CHANNEL, SPRING1_SIGN_PIN);
DECLARE_SPRING_INTERFACE(2, STM32_GPIO_PIN(GPIO_PORTF | GPIO_PIN14), 10,
                         SPRING2_ADC, SPRING2_SENSE_CHANNEL, SPRING2_SIGN_PIN);
DECLARE_SPRING_INTERFACE(3, STM32_GPIO_PIN(GPIO_PORTF | GPIO_PIN13), 11,
                         SPRING3_ADC, SPRING3_SENSE_CHANNEL, SPRING3_SIGN_PIN);
DECLARE_SPRING_INTERFACE(4, STM32_GPIO_PIN(GPIO_PORTF | GPIO_PIN11), 6,
                         SPRING4_ADC, SPRING4_SENSE_CHANNEL, SPRING4_SIGN_PIN);
DECLARE_SPRING_INTERFACE(5, STM32_GPIO_PIN(GPIO_PORTG | GPIO_PIN15), 7,
                         SPRING5_ADC, SPRING5_SENSE_CHANNEL, SPRING5_SIGN_PIN);
DECLARE_SPRING_INTERFACE(6, STM32_GPIO_PIN(GPIO_PORTG | GPIO_PIN12), 8,
                         SPRING6_ADC, SPRING6_SENSE_CHANNEL, SPRING6_SIGN_PIN);
DECLARE_SPRING_INTERFACE(7, STM32_GPIO_PIN(GPIO_PORTG | GPIO_PIN13), 5,
                         SPRING7_ADC, SPRING7_SENSE_CHANNEL, SPRING7_SIGN_PIN);
DECLARE_SPRING_INTERFACE(8, STM32_GPIO_PIN(GPIO_PORTG | GPIO_PIN15), 13,
                         SPRING8_ADC, SPRING8_SENSE_CHANNEL, SPRING8_SIGN_PIN);

/*
 * NB: always declare first the interfaces, then the spring interfaces.
 * Assumed by Spring Power Measurement Library (up_spring_pm.c).
 */
static struct interface *bdb1b_interfaces[] = {
    &apb1_interface,
    &apb2_interface,
    &apb3_interface,
    &gpb1_interface,
    &gpb2_interface,
    &bb1_interface,
    &bb2_interface,
    &bb3_interface,
    &bb4_interface,
    &bb5_interface,
    &bb6_interface,
    &bb7_interface,
    &bb8_interface,
};

/*
 * Switch power supplies.
 * Note: 1P8 is also used by the I/O Expanders.
 */
static struct vreg_data sw_vreg_data[] = {
    // Switch 1P1
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTH | GPIO_PIN9),
                               HOLD_TIME_SW_1P1),
    // Switch 1P8
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTH | GPIO_PIN6),
                               HOLD_TIME_SW_1P8),
};
DECLARE_VREG(sw, sw_vreg_data);

static struct io_expander_info bdb1b_io_expanders[] = {
        {
            .part       = TCA6416_PART,
            .i2c_bus    = IOEXP_I2C_BUS,
            .i2c_addr   = IOEXP_U96_I2C_ADDR,
            .reset      = IO_RESET,
            .irq        = U96_IO_EXP_IRQ,
            .gpio_base  = U96_GPIO_CHIP_START,
        },
        {
            .part       = TCA6416_PART,
            .i2c_bus    = IOEXP_I2C_BUS,
            .i2c_addr   = IOEXP_U90_I2C_ADDR,
            .reset      = IO_RESET1,
            .irq        = U90_IO_EXP_IRQ,
            .gpio_base  = U90_GPIO_CHIP_START,
        },
        {
            .part       = TCA6424_PART,
            .i2c_bus    = IOEXP_I2C_BUS,
            .i2c_addr   = IOEXP_U135_I2C_ADDR,
            .reset      = TCA64XX_IO_UNUSED,
            .irq        = TCA64XX_IO_UNUSED,
            .gpio_base  = U135_GPIO_CHIP_START,
        }
};

static struct ara_board_info bdb1b_board_info = {
    .interfaces = bdb1b_interfaces,
    .nr_interfaces = ARRAY_SIZE(bdb1b_interfaces),
    .nr_spring_interfaces = SPRING_INTERFACES_COUNT,

    .sw_data = {
        .vreg = &sw_vreg,
        .gpio_reset = (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR |
                       GPIO_PORTE | GPIO_PIN14),
        .gpio_irq   = (GPIO_PORTI | GPIO_PIN9),
        .rev        = SWITCH_REV_ES1,
        .bus        = SWITCH_I2C_BUS,
    },

    .io_expanders = bdb1b_io_expanders,
    .nr_io_expanders = ARRAY_SIZE(bdb1b_io_expanders),
};

struct ara_board_info *board_init(void) {
    int i;

    /* Pretty lights */
    stm32_configgpio(SVC_LED_RED);
    stm32_gpiowrite(SVC_LED_RED, true);

    /* Disable these for now */
    stm32_configgpio(IO_RESET);
    stm32_configgpio(IO_RESET1);
    stm32_gpiowrite(IO_RESET, false);
    stm32_gpiowrite(IO_RESET1, false);

    /*
     * Register the STM32 GPIOs to Gpio Chip
     *
     * This needs to happen before the I/O Expanders registration, which
     * uses some STM32 pins
     */
    stm32_gpio_init();

    /*
     * Configure the switch reset and power supply lines.
     * Hold all the lines low while we turn on the power rails.
     */
    vreg_config(&sw_vreg);
    stm32_configgpio(bdb1b_board_info.sw_data.gpio_reset);
    up_udelay(POWER_SWITCH_OFF_STAB_TIME_US);

    /*
     * Enable 1P1 and 1P8, used by the I/O Expanders.
     * This also enables the switch power supplies.
     */
    vreg_get(&sw_vreg);

    /* Register the TCA64xx I/O Expanders GPIOs to Gpio Chip */
    for (i = 0; i < bdb1b_board_info.nr_io_expanders; i++) {
        struct io_expander_info *io_exp = &bdb1b_board_info.io_expanders[i];

        io_exp->i2c_dev = up_i2cinitialize(io_exp->i2c_bus);
        if (!io_exp->i2c_dev) {
            dbg_error("%s(): Failed to get I/O Expander I2C bus %u\n",
                      __func__, io_exp->i2c_bus);
        } else {
            if (tca64xx_init(&io_exp->io_exp_driver_data,
                             io_exp->part,
                             io_exp->i2c_dev,
                             io_exp->i2c_addr,
                             io_exp->reset,
                             io_exp->irq,
                             io_exp->gpio_base) < 0) {
                dbg_error("%s(): Failed to register I/O Expander(0x%02x)\n",
                          __func__, io_exp->i2c_addr);
                up_i2cuninitialize(io_exp->i2c_dev);
            }
        }
    }

    return &bdb1b_board_info;
}

void board_exit(void) {
    int i;


    /*
     * First unregister the TCA64xx I/O Expanders and associated I2C bus(ses).
     * Done in reverse order from registration to account for IRQ chaining
     * between I/O Expander chips.
     */
    for (i = bdb1b_board_info.nr_io_expanders - 1; i >= 0; i--) {
        struct io_expander_info *io_exp = &bdb1b_board_info.io_expanders[i];

        if (io_exp->io_exp_driver_data)
            tca64xx_deinit(io_exp->io_exp_driver_data);

        if (io_exp->i2c_dev)
            up_i2cuninitialize(io_exp->i2c_dev);
    }

    /* Disable 1V1 and 1V8, used by the I/O Expanders */
    vreg_put(&sw_vreg);

    /* Lastly unregister the GPIO Chip driver */
    stm32_gpio_deinit();
}
