/**
 * Copyright (c) 2015 Google Inc.
 * Google Confidential/Restricted
 */

#define DBG_COMP DBG_SVC     /* DBG_COMP macro of the component */

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include "ara_board.h"
#include "interface.h"
#include "up_switch.h"
#include "stm32.h"

#define SVC_LED_RED         (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_PORTA | \
                             GPIO_OUTPUT_SET | GPIO_PIN7)

#define IO_EXP_MODE         (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_PULLUP | \
                             GPIO_OUTPUT_CLEAR)
#define IO_RESET            (IO_EXP_MODE | GPIO_PORTE | GPIO_PIN0)
#define IO_RESET1           (IO_EXP_MODE | GPIO_PORTE | GPIO_PIN1)

#define VREG_DEFAULT_MODE           (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_OUTPUT_CLEAR)
/* Spring pins are active low */
#define SPRING_VREG_DEFAULT_MODE    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_OUTPUT_SET)

/*
 * How long to leave hold each regulator before the next.
 */
#define HOLD_TIME_1P1   (50000)     // 0-100ms before 1p2, 1p8
#define HOLD_TIME_1P8   (0)
#define HOLD_TIME_1P2   (0)

/*
 * Built-in bridge voltage regulator list
 */
static struct vreg_data apb1_vregs[] = {
    INIT_VREG_DATA(GPIO_PORTD | GPIO_PIN4, HOLD_TIME_1P1),
    INIT_VREG_DATA(GPIO_PORTD | GPIO_PIN5, HOLD_TIME_1P8),
    INIT_VREG_DATA(GPIO_PORTD | GPIO_PIN6, HOLD_TIME_1P2),
};

static struct vreg_data apb2_vregs[] = {
    INIT_VREG_DATA(GPIO_PORTD | GPIO_PIN11, HOLD_TIME_1P1),
    INIT_VREG_DATA(GPIO_PORTD | GPIO_PIN10, HOLD_TIME_1P8),
    INIT_VREG_DATA(GPIO_PORTE | GPIO_PIN7, HOLD_TIME_1P2),
    INIT_VREG_DATA(GPIO_PORTC | GPIO_PIN6, 0),              // 2p8_tp
};

static struct vreg_data apb3_vregs[] = {
    INIT_VREG_DATA(GPIO_PORTD | GPIO_PIN12, HOLD_TIME_1P1),
    INIT_VREG_DATA(GPIO_PORTE | GPIO_PIN15, HOLD_TIME_1P8),
    INIT_VREG_DATA(GPIO_PORTE | GPIO_PIN13, HOLD_TIME_1P2),
    INIT_VREG_DATA(GPIO_PORTC | GPIO_PIN7, 0),              // 2p8
};

static struct vreg_data gpb1_vregs[] =  {
    INIT_VREG_DATA(GPIO_PORTB | GPIO_PIN5, HOLD_TIME_1P1),
    INIT_VREG_DATA(GPIO_PORTC | GPIO_PIN3, HOLD_TIME_1P8),
    INIT_VREG_DATA(GPIO_PORTC | GPIO_PIN2, HOLD_TIME_1P2),
};

static struct vreg_data gpb2_vregs[] = {
    INIT_VREG_DATA(GPIO_PORTD | GPIO_PIN7, HOLD_TIME_1P1),
    INIT_VREG_DATA(GPIO_PORTD | GPIO_PIN9, HOLD_TIME_1P8),
    INIT_VREG_DATA(GPIO_PORTD | GPIO_PIN8, HOLD_TIME_1P2),
};

/*
 * Interfaces on this board
 */
DECLARE_INTERFACE(apb1, apb1_vregs, 0);
DECLARE_INTERFACE(apb2, apb2_vregs, 1);
DECLARE_INTERFACE(apb3, apb3_vregs, 2);
DECLARE_INTERFACE(gpb1, gpb1_vregs, 3);
DECLARE_INTERFACE(gpb2, gpb2_vregs, 4);

DECLARE_SPRING_INTERFACE(1, (GPIO_PORTI | GPIO_PIN2), 9);
DECLARE_SPRING_INTERFACE(2, (GPIO_PORTF | GPIO_PIN14), 10);
DECLARE_SPRING_INTERFACE(3, (GPIO_PORTF | GPIO_PIN13), 11);
DECLARE_SPRING_INTERFACE(4, (GPIO_PORTF | GPIO_PIN11), 6);
DECLARE_SPRING_INTERFACE(5, (GPIO_PORTG | GPIO_PIN15), 7);
DECLARE_SPRING_INTERFACE(6, (GPIO_PORTG | GPIO_PIN12), 8);
DECLARE_SPRING_INTERFACE(7, (GPIO_PORTG | GPIO_PIN13), 5);
DECLARE_SPRING_INTERFACE(8, (GPIO_PORTG | GPIO_PIN15), 13);

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

static struct ara_board_info bdb1b_board_info = {
    .interfaces = bdb1b_interfaces,
    .nr_interfaces = NELEM(bdb1b_interfaces),

    .sw_i2c_bus = 2,
    .sw_1p1   = (VREG_DEFAULT_MODE | GPIO_PORTH | GPIO_PIN9),
    .sw_1p8   = (VREG_DEFAULT_MODE | GPIO_PORTH | GPIO_PIN6),
    .sw_reset = (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR |
                 GPIO_PORTE | GPIO_PIN14),
    .sw_irq   = (GPIO_PORTI | GPIO_PIN9),

    .svc_irq = (GPIO_PORTA | GPIO_PIN0),
};

struct ara_board_info *board_init(void) {
    /* Pretty lights */
    stm32_configgpio(SVC_LED_RED);
    stm32_gpiowrite(SVC_LED_RED, true);

    /* Disable these for now */
    stm32_configgpio(IO_RESET);
    stm32_configgpio(IO_RESET1);
    stm32_gpiowrite(IO_RESET, false);
    stm32_gpiowrite(IO_RESET1, false);

    return &bdb1b_board_info;
}

void board_exit(void) {
}
