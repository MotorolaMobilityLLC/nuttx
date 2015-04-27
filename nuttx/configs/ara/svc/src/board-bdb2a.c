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
 * @author: Jean Pihet
 * @author: Perry Hung
 */

#define DBG_COMP DBG_SVC     /* DBG_COMP macro of the component */

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/util.h>

#include "nuttx/gpio/stm32_gpio_chip.h"

#include "up_debug.h"
#include "ara_board.h"
#include "interface.h"
#include "tsb_switch_driver_es2.h"
#include "stm32.h"

#define SVC_LED_GREEN       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_PORTB | \
                             GPIO_OUTPUT_SET | GPIO_PIN0)

#define IO_EXP_MODE         (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_PULLUP | \
                             GPIO_OUTPUT_CLEAR)
#define IO_RESET            (IO_EXP_MODE | GPIO_PORTE | GPIO_PIN0)
#define IO_RESET1           (IO_EXP_MODE | GPIO_PORTE | GPIO_PIN1)

#define VREG_DEFAULT_MODE   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_OUTPUT_CLEAR)
/* Spring pins are active low */
#define SPRING_VREG_DEFAULT_MODE    (GPIO_OUTPUT | GPIO_PUSHPULL | \
                                     GPIO_OUTPUT_SET)

/* SPI CS */
#define TSB_SW_CS           (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_OUTPUT_SET | \
                             GPIO_PORTA | GPIO_PIN4)

#define BOOTRET_MODE        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_OUTPUT_CLEAR)

/*
 * How long to leave hold each regulator before the next.
 */
#define HOLD_TIME_1P1   (50000)     // 0-100ms before 1p2, 1p8
#define HOLD_TIME_1P8   (0)
#define HOLD_TIME_1P2   (0)

#define WAKEOUT_APB1    (GPIO_FLOAT | GPIO_PORTE | GPIO_PIN8)
#define WAKEOUT_APB2    (GPIO_FLOAT | GPIO_PORTE | GPIO_PIN12)
#define WAKEOUT_APB3    (GPIO_FLOAT | GPIO_PORTF | GPIO_PIN6)
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
#define INIT_BOOTRET_DATA(g)        \
    {                               \
        .gpio = (BOOTRET_MODE | g), \
        .hold_time = 0,             \
        .active_high = 0,           \
    }

/*
 * Built-in bridge voltage regulator list
 */
static struct vreg_data apb1_vregs[] = {
    INIT_BOOTRET_DATA(GPIO_PORTF | GPIO_PIN12),
    INIT_VREG_DATA(GPIO_PORTD | GPIO_PIN4, HOLD_TIME_1P1),
    INIT_VREG_DATA(GPIO_PORTD | GPIO_PIN5, HOLD_TIME_1P8),
    INIT_VREG_DATA(GPIO_PORTD | GPIO_PIN6, HOLD_TIME_1P2),
};

static struct vreg_data apb2_vregs[] = {
    INIT_BOOTRET_DATA(GPIO_PORTG | GPIO_PIN3),
    INIT_VREG_DATA(GPIO_PORTD | GPIO_PIN11, HOLD_TIME_1P1),
    INIT_VREG_DATA(GPIO_PORTD | GPIO_PIN10, HOLD_TIME_1P8),
    INIT_VREG_DATA(GPIO_PORTE | GPIO_PIN7, HOLD_TIME_1P2),
    INIT_VREG_DATA(GPIO_PORTC | GPIO_PIN6, 0),              // 2p8_tp
};

static struct vreg_data apb3_vregs[] = {
    INIT_BOOTRET_DATA(GPIO_PORTG | GPIO_PIN4),
    INIT_VREG_DATA(GPIO_PORTD | GPIO_PIN12, HOLD_TIME_1P1),
    INIT_VREG_DATA(GPIO_PORTE | GPIO_PIN15, HOLD_TIME_1P8),
    INIT_VREG_DATA(GPIO_PORTE | GPIO_PIN13, HOLD_TIME_1P2),
    INIT_VREG_DATA(GPIO_PORTC | GPIO_PIN7, 0),              // 2p8
};

static struct vreg_data gpb1_vregs[] =  {
    INIT_BOOTRET_DATA(GPIO_PORTG | GPIO_PIN6),
    INIT_VREG_DATA(GPIO_PORTB | GPIO_PIN5, HOLD_TIME_1P1),
    INIT_VREG_DATA(GPIO_PORTC | GPIO_PIN3, HOLD_TIME_1P8),
    INIT_VREG_DATA(GPIO_PORTC | GPIO_PIN2, HOLD_TIME_1P2),
};

static struct vreg_data gpb2_vregs[] = {
    INIT_BOOTRET_DATA(GPIO_PORTG | GPIO_PIN7),
    INIT_VREG_DATA(GPIO_PORTD | GPIO_PIN7, HOLD_TIME_1P1),
    INIT_VREG_DATA(GPIO_PORTD | GPIO_PIN9, HOLD_TIME_1P8),
    INIT_VREG_DATA(GPIO_PORTD | GPIO_PIN8, HOLD_TIME_1P2),
};

/*
 * Interfaces on this board
 */
DECLARE_INTERFACE(apb1, apb1_vregs, 0, WAKEOUT_APB1);
DECLARE_INTERFACE(apb2, apb2_vregs, 1, WAKEOUT_APB2);
DECLARE_INTERFACE(apb3, apb3_vregs, 2, WAKEOUT_APB3);
DECLARE_INTERFACE(gpb1, gpb1_vregs, 3, WAKEOUT_GPB1);
DECLARE_INTERFACE(gpb2, gpb2_vregs, 4, WAKEOUT_GPB2);

DECLARE_SPRING_INTERFACE(1, (GPIO_PORTI | GPIO_PIN2), 9);
DECLARE_SPRING_INTERFACE(2, (GPIO_PORTF | GPIO_PIN14), 10);
DECLARE_SPRING_INTERFACE(3, (GPIO_PORTI | GPIO_PIN3), 11);
DECLARE_SPRING_INTERFACE(4, (GPIO_PORTF | GPIO_PIN11), 6);
DECLARE_SPRING_INTERFACE(5, (GPIO_PORTG | GPIO_PIN15), 7);
DECLARE_SPRING_INTERFACE(6, (GPIO_PORTG | GPIO_PIN12), 8);
DECLARE_SPRING_INTERFACE(7, (GPIO_PORTG | GPIO_PIN13), 5);
DECLARE_SPRING_INTERFACE(8, (GPIO_PORTF | GPIO_PIN15), 13);

/*
 * NB: always declare first the interfaces, then the spring interfaces.
 * Assumed by Spring Power Measurement Library (up_spring_pm.c).
 */
static struct interface *bdb2a_interfaces[] = {
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

static struct ara_board_info bdb2a_board_info = {
    .interfaces = bdb2a_interfaces,
    .nr_interfaces = ARRAY_SIZE(bdb2a_interfaces),

    .sw_1p1   = (VREG_DEFAULT_MODE | GPIO_PORTH | GPIO_PIN9),
    .sw_1p8   = (VREG_DEFAULT_MODE | GPIO_PORTH | GPIO_PIN6),
    .sw_reset = (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_PULLUP |
                 GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN14),
    .sw_irq   = (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTI | GPIO_PIN9),
    .svc_irq  = (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTA | GPIO_PIN0),
};

struct ara_board_info *board_init(struct tsb_switch *sw) {

    /* Pretty lights */
    stm32_configgpio(SVC_LED_GREEN);
    stm32_gpiowrite(SVC_LED_GREEN, true);

    /* Disable these for now */
    stm32_configgpio(IO_RESET);
    stm32_configgpio(IO_RESET1);
    stm32_gpiowrite(IO_RESET, false);
    stm32_gpiowrite(IO_RESET1, false);

    stm32_configgpio(TSB_SW_CS);
    stm32_gpiowrite(TSB_SW_CS, true);

    // Register the GPIO Chip driver
    stm32_gpio_init();

    // Initialize the Switch
    if (tsb_switch_es2_init(sw, SW_SPI_PORT)) {
        return NULL;
    }

    return &bdb2a_board_info;
}

void board_exit(struct tsb_switch *sw) {
    // Deinit the Switch
    tsb_switch_es2_exit(sw);

    // Unegister the GPIO Chip driver
    stm32_gpio_deinit();
}

/*
 * Required callbacks for NuttX SPI1
 */
void stm32_spi1select(struct spi_dev_s *dev, enum spi_dev_e devid, bool selected) {
    if (devid == SW_SPI_ID) {
        dbg_insane("%s(): CS %s\n", __func__,
                   selected ? "asserted" : "de-asserted");

        /*
         * SW-472: The STM32 SPI peripheral does not delay until the last
         * falling edge of SCK, instead dropping RXNE as soon as the rising
         * edge is clocked out.
         * Manually add a hacked delay in these cases...
         */
        if ((!selected) && (SWITCH_SPI_FREQUENCY < 8000000))
                up_udelay(2);

        /* Set the GPIO low to select and high to de-select */
        stm32_gpiowrite(TSB_SW_CS, !selected);
    }
}

uint8_t stm32_spi1status(struct spi_dev_s *dev, enum spi_dev_e devid) {
    return (devid == SW_SPI_ID) ? SPI_STATUS_PRESENT : 0;
}
