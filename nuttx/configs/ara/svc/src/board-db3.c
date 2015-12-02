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
#include "pwr_mon.h"

/* U4550 I/O Expander reset */
#define SVC_RST_IOEXP1_PIN  STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN14)
#define SVC_RST_IOEXP1      (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_PULLUP | \
                             GPIO_PORTC | GPIO_PIN14)
/* U4570 I/O Expander reset */
#define SVC_RST_IOEXP2_PIN  STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN15)
#define SVC_RST_IOEXP2      (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_PULLUP | \
                             GPIO_PORTC | GPIO_PIN15)

/* I/O Expanders IRQ to SVC */
#define IO_EXP_IRQ          STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN10)

/* I/O Expander: I2C bus and addresses */
#define IOEXP_I2C_BUS           1
#define IOEXP_U4550_I2C_ADDR    0x21
#define IOEXP_U4570_I2C_ADDR    0x20

/* 19.2MHz system reference clocks */
#define REFCLK_REQ        STM32_GPIO_PIN(GPIO_PORTB | GPIO_PIN9)
#define REFCLK_APB1_EN    U4570_GPIO_PIN(0) // On-board APB1
#define REFCLK_APB2_EN    U4570_GPIO_PIN(1) // On-board APB2
#define REFCLK_BUFFERS_EN U4570_GPIO_PIN(2) // Shared clock enable for modules
#define REFCLK_SW_EN      U4570_GPIO_PIN(3) // Switch
#define REFCLK_1_EN       U4570_GPIO_PIN(4) // Modules
#define REFCLK_2_EN       U4570_GPIO_PIN(5)
#define REFCLK_3_EN       U4570_GPIO_PIN(6)
#define REFCLK_4A_EN      U4570_GPIO_PIN(7)
#define REFCLK_4B_EN      U4570_GPIO_PIN(8)
#define REFCLK_5_EN       U4570_GPIO_PIN(9)
#define REFCLK_6_EN       U4570_GPIO_PIN(10)

/* Ara Key switch */
#define ARA_KEY         (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTA | GPIO_PIN0)

/*
 * WAKE_DETECT lines directly connected to the SVC.
 * Configured by default as input floating without pull-up.
 */
#define WD_1_DET_IN     (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN1)
#define WD_2_DET_IN     (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN2)
#define WD_3_DET_IN     (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN3)
#define WD_4A_DET_IN    (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN4)
#define WD_4B_DET_IN    (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN5)
#define WD_5_DET_IN     (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN6)
#define WD_6_DET_IN     (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN7)
#define WD8A_DET_IN     (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN11)
#define WD8B_DET_IN     (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN12)

/*
 * MOD_ACT_SW lines connected to SVC.
 *
 * Configured by default as input pullup, as there is no external
 * pullup.
 */
#define MOD_ACT_SW_1    (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN0)
#define MOD_ACT_SW_2    (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN2)
#define MOD_ACT_SW_3    (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN3)
#define MOD_ACT_SW_4    (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN4)
#define MOD_ACT_SW_5    (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN6)
#define MOD_ACT_SW_6    (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN7)
#define MOD_ACT_SW_7    (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN8)

/*
 * On-board bridges clock control
 */

/* FIXME: put the bridge enables back in here once the handshaking
 * with MSM is OK. */

static struct vreg_data apb1_vreg_data[] = {
    /* INIT_MODULE_CLK_DATA(REFCLK_APB1_EN), */
};

static struct vreg_data apb2_vreg_data[] = {
    /* INIT_MODULE_CLK_DATA(REFCLK_APB2_EN), */
};

/*
 * Modules voltage regulator list
 *
 * Notes:
 *
 * 1. The modules loadswitches are controlled by the INA231 ALERT output,
 * to be driven via I2C commands. Take a shortcut for bring-up until the
 * INA231 support is in: directly control it via the VSYS_ENx GPIO pins.
 * (ToDo)
 * 2. Module port 6 (Display and TS) is controlled by the AP. Interface
 * control pins to the SVC are present but not used:
 * WD_6, REFCLK_6_EN, VSYS_EN6.
 */
static struct vreg_data module_1_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(U4550_GPIO_PIN(0), HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_1_EN),
};

static struct vreg_data module_2_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(U4550_GPIO_PIN(2), HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_2_EN),
};

static struct vreg_data module_3_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(U4550_GPIO_PIN(4), HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_3_EN),
};

static struct vreg_data module_4a_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(U4550_GPIO_PIN(6), HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_4A_EN),
};

static struct vreg_data module_4b_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(U4550_GPIO_PIN(6), HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_4B_EN),
};

static struct vreg_data module_5_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(U4550_GPIO_PIN(8), HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_5_EN),
};

/*
 * Interfaces on this board
 */
DECLARE_MODULE_PORT_INTERFACE(apb1, apb1_vreg_data, 3,
                              WD8A_DET_IN, ARA_IFACE_WD_ACTIVE_HIGH);
DECLARE_MODULE_PORT_INTERFACE(apb2, apb2_vreg_data, 1,
                              WD8B_DET_IN, ARA_IFACE_WD_ACTIVE_HIGH);
DECLARE_MODULE_PORT_INTERFACE(module_1, module_1_vreg_data, 13,
                              WD_1_DET_IN, ARA_IFACE_WD_ACTIVE_LOW);
DECLARE_MODULE_PORT_INTERFACE(module_2, module_2_vreg_data, 11,
                              WD_2_DET_IN, ARA_IFACE_WD_ACTIVE_LOW);
DECLARE_MODULE_PORT_INTERFACE(module_3, module_3_vreg_data, 4,
                              WD_3_DET_IN, ARA_IFACE_WD_ACTIVE_LOW);
DECLARE_MODULE_PORT_INTERFACE(module_4a, module_4a_vreg_data, 8,
                              WD_4A_DET_IN, ARA_IFACE_WD_ACTIVE_LOW);
DECLARE_MODULE_PORT_INTERFACE(module_4b, module_4b_vreg_data, 6,
                              WD_4B_DET_IN, ARA_IFACE_WD_ACTIVE_LOW);
DECLARE_MODULE_PORT_INTERFACE(module_5, module_5_vreg_data, 10,
                              WD_5_DET_IN, ARA_IFACE_WD_ACTIVE_LOW);

#define I2C_SEL1_A      BIT(0)
#define I2C_SEL1_B      BIT(1)
#define I2C_SEL1_INH    BIT(2)

enum {
    I2C_INA230_SEL1_A = STM32_GPIO_CHIP_BASE + 71, /* PE7 */
    I2C_INA230_SEL1_B,
    I2C_INA230_SEL1_INH,
};

/*
 * Global power monitoring I2C bus.
 */
const int pwrmon_i2c_bus = 1;

/*
 * Power rail groups definitions.
 *
 * If a bit is set in i2c_sel, then drive the GPIO low.
 */
const struct pwrmon_dev_ctx pwrmon_devs[] = {
    {
        .name = "SWitch",
        .i2c_sel = I2C_SEL1_A | I2C_SEL1_B | I2C_SEL1_INH,
        .rails = {
            DEFINE_PWR_RAIL("VSW_1P1_PLL",        0x42),
            DEFINE_PWR_RAIL("VSW_1P1_CORE",       0x41),
            DEFINE_PWR_RAIL("VSW_1P8_UNIPRO",     0x47),
            DEFINE_PWR_RAIL("VSW_1P8_IO",         0x48),
        },
        .num_rails = 4,
    },
    {
        .name = "APB1",
        .i2c_sel = I2C_SEL1_B | I2C_SEL1_INH,
        .rails = {
            DEFINE_PWR_RAIL("VAPB1_1P1_CORE",     0x41),
            DEFINE_PWR_RAIL("VAPB1_1P1_PLL1",     0x42),
            DEFINE_PWR_RAIL("VAPB1_1P2_CDSI_PLL", 0x4A),
            DEFINE_PWR_RAIL("VAPB1_1P2_CDSI",     0x4B),
            DEFINE_PWR_RAIL("VAPB1_1P2_HSIC",     0x46),
            DEFINE_PWR_RAIL("VAPB1_1P8_UNIPRO",   0x47),
            DEFINE_PWR_RAIL("VAPB1_1P8_IO",       0x48),
            DEFINE_PWR_RAIL("VAPB1_1P1_PLL2",     0x43),
        },
        .num_rails = 8,
    },
    {
        .name = "APB2",
        .i2c_sel = I2C_SEL1_A | I2C_SEL1_INH,
        .rails = {
            /*
             * NOTE:
             *
             * The APB2 1.2V CDSI and 1.1V PLL2 rails cannot be
             * measured on DB3.1, because they were accidentally on
             * the wrong I2C bus (they're on the bus for APB1 instead,
             * where they have an address conflict with the INA231s
             * associated with APB1's power rails).
             *
             * This was worked around by removing the following
             * commented-out power rails. If your board is different,
             * you can uncomment them.
             */
            DEFINE_PWR_RAIL("VAPB2_1P1_CORE",     0x41),
            DEFINE_PWR_RAIL("VAPB2_1P1_PLL1",     0x42),
            DEFINE_PWR_RAIL("VAPB2_1P2_CDSI_PLL", 0x4A),
            /* DEFINE_PWR_RAIL("VAPB2_1P2_CDSI",     0x4B), */
            DEFINE_PWR_RAIL("VAPB2_1P2_HSIC",     0x46),
            DEFINE_PWR_RAIL("VAPB2_1P8_UNIPRO",   0x47),
            DEFINE_PWR_RAIL("VAPB2_1P8_IO",       0x48),
            /* DEFINE_PWR_RAIL("VAPB2_1P1_PLL2",     0x43), */
        },
        .num_rails = 6,
    },
    {
        .name = "SVC",
        .i2c_sel = I2C_SEL1_INH,
        .rails = {
            DEFINE_PWR_RAIL("SVC_1P8_VDD",        0x42),
            DEFINE_PWR_RAIL("SVC_1P8_VBAT",       0x41),
            DEFINE_PWR_RAIL("SVC_1P8_VDDA",       0x47),
        },
        .num_rails = 3,
    },
};

const size_t pwrmon_num_devs = ARRAY_SIZE(pwrmon_devs);

void pwrmon_reset_i2c_sel(void)
{
    gpio_set_value(I2C_INA230_SEL1_INH, 1);
    gpio_set_value(I2C_INA230_SEL1_A, 1);
    gpio_set_value(I2C_INA230_SEL1_B, 1);
}

void pwrmon_init_i2c_sel(void)
{
    gpio_direction_out(I2C_INA230_SEL1_A, 1);
    gpio_direction_out(I2C_INA230_SEL1_B, 1);
    gpio_direction_out(I2C_INA230_SEL1_INH, 1);
}

int pwrmon_do_i2c_sel(uint8_t dev)
{
    if (dev >= ARRAY_SIZE(pwrmon_devs)) {
        return -EINVAL;
    }

    /* First inhibit all lines, to make sure there is no short/collision */
    gpio_set_value(I2C_INA230_SEL1_INH, 1);

    gpio_set_value(I2C_INA230_SEL1_A, pwrmon_devs[dev].i2c_sel & I2C_SEL1_A ? 0 : 1);
    gpio_set_value(I2C_INA230_SEL1_B, pwrmon_devs[dev].i2c_sel & I2C_SEL1_B ? 0 : 1);

    if (pwrmon_devs[dev].i2c_sel & I2C_SEL1_INH) {
        gpio_set_value(I2C_INA230_SEL1_INH, 0);
    }

    return 0;
}

/*
 * Important note: Always declare the spring interfaces last.
 * Assumed by Spring Power Measurement Library (up_spring_pm.c).
 */
static struct interface *db3_interfaces[] = {
    &apb1_interface,
    &apb2_interface,
    &module_1_interface,
    &module_2_interface,
    &module_3_interface,
    &module_4a_interface,
    &module_4b_interface,
    &module_5_interface,
};

/*
 * Switch clock and power supplies
 */
static struct vreg_data sw_vreg_data[] = {
    /* 19.2MHz clock */
    INIT_MODULE_CLK_DATA(REFCLK_SW_EN),
    /* Switch 1P1 */
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTB | GPIO_PIN0),
                               HOLD_TIME_SW_1P1),
    /* Switch 1P8 */
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTB | GPIO_PIN1),
                               HOLD_TIME_SW_1P8),
};
DECLARE_VREG(sw, sw_vreg_data);

static struct io_expander_info db3_io_expanders[] = {
        {
            .part       = TCA6416_PART,
            .i2c_bus    = IOEXP_I2C_BUS,
            .i2c_addr   = IOEXP_U4550_I2C_ADDR,
            .reset      = SVC_RST_IOEXP1_PIN,
            .irq        = IO_EXP_IRQ,
            .gpio_base  = U4550_GPIO_CHIP_START,
        },
        {
            .part       = TCA6416_PART,
            .i2c_bus    = IOEXP_I2C_BUS,
            .i2c_addr   = IOEXP_U4570_I2C_ADDR,
            .reset      = SVC_RST_IOEXP2_PIN,
            .irq        = IO_EXP_IRQ,
            .gpio_base  = U4570_GPIO_CHIP_START,
        },
};

static struct ara_board_info db3_board_info = {
    .interfaces = db3_interfaces,
    .nr_interfaces = ARRAY_SIZE(db3_interfaces),
    .nr_spring_interfaces = 0,

    .sw_data = {
        .vreg             = &sw_vreg,
        .gpio_reset       = (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR |
                             GPIO_PORTD | GPIO_PIN11),
        .gpio_irq         = (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | \
                             GPIO_PORTC | GPIO_PIN9),
        .irq_rising_edge  = true,
        .rev              = SWITCH_REV_ES2,
        .bus              = SW_SPI_PORT_2,
        .spi_cs           = (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_OUTPUT_SET | \
                             GPIO_PORTB | GPIO_PIN12)
    },

    .io_expanders   = db3_io_expanders,
    .nr_io_expanders = ARRAY_SIZE(db3_io_expanders),
};

struct ara_board_info *board_init(void) {
    int i;

    /* Disable the I/O Expanders for now */
    stm32_configgpio(SVC_RST_IOEXP1);
    stm32_gpiowrite(SVC_RST_IOEXP1, false);
    stm32_configgpio(SVC_RST_IOEXP2);
    stm32_gpiowrite(SVC_RST_IOEXP2, false);

    /*
     * Register the STM32 GPIOs to Gpio Chip
     *
     * This needs to happen before the I/O Expanders registration, which
     * uses some STM32 pins
     */
    stm32_gpio_init();

    /* Register the TCA64xx I/O Expanders GPIOs to Gpio Chip */
    for (i = 0; i < db3_board_info.nr_io_expanders; i++) {
        struct io_expander_info *io_exp = &db3_board_info.io_expanders[i];

        io_exp->i2c_dev = up_i2cinitialize(io_exp->i2c_bus);
        if (!io_exp->i2c_dev) {
            dbg_error("%s(): Failed to get I/O Expander I2C bus %u\n",
                      __func__, io_exp->i2c_bus);
            goto err_deinit_gpio;
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
                goto err_uninit_i2c;
            }
        }
    }

    /*
     * Turn on the global system clock and its buffered copy (which
     * goes to the modules and the switch).
     *
     * FIXME replace with real power management of clocks later.
     */
    gpio_activate(REFCLK_REQ);
    gpio_activate(REFCLK_BUFFERS_EN);
    gpio_direction_out(REFCLK_REQ, 1);
    gpio_direction_out(REFCLK_BUFFERS_EN, 1);

    /*
     * FIXME: tear this out once the handshaking with MSM is OK.
     */
    gpio_activate(REFCLK_APB1_EN);
    gpio_activate(REFCLK_APB2_EN);
    gpio_direction_out(REFCLK_APB1_EN, 1);
    gpio_direction_out(REFCLK_APB2_EN, 1);

    /*
     * Configure the switch power supply lines.
     * Hold all the lines low while we turn on the power rails.
     */
    vreg_config(&sw_vreg);
    stm32_configgpio(db3_board_info.sw_data.gpio_reset);
    up_udelay(POWER_SWITCH_OFF_STAB_TIME_US);

    /*
     * Configure the SVC WAKE_DETECT pins
     */
    stm32_configgpio(WD_1_DET_IN);
    stm32_configgpio(WD_2_DET_IN);
    stm32_configgpio(WD_3_DET_IN);
    stm32_configgpio(WD_4A_DET_IN);
    stm32_configgpio(WD_4B_DET_IN);
    stm32_configgpio(WD_5_DET_IN);
    stm32_configgpio(WD_6_DET_IN);
    stm32_configgpio(WD8A_DET_IN);
    stm32_configgpio(WD8B_DET_IN);

    /*
     * Configure the MOD_ACT_SW pins.
     */
    stm32_configgpio(MOD_ACT_SW_1);
    stm32_configgpio(MOD_ACT_SW_2);
    stm32_configgpio(MOD_ACT_SW_3);
    stm32_configgpio(MOD_ACT_SW_4);
    stm32_configgpio(MOD_ACT_SW_5);
    stm32_configgpio(MOD_ACT_SW_6);
    stm32_configgpio(MOD_ACT_SW_7);

    return &db3_board_info;

 err_uninit_i2c:
    /* Done in reverse order to account for possible IRQ chaining. */
    for (i = db3_board_info.nr_io_expanders - 1; i >= 0; i--) {
        struct io_expander_info *io_exp = &db3_board_info.io_expanders[i];
        if (io_exp->i2c_dev) {
            up_i2cuninitialize(io_exp->i2c_dev);
        }
    }
 err_deinit_gpio:
    stm32_gpio_deinit();
    /* Leave the I/O expanders in reset here. */
    return NULL;
}

void board_exit(void) {
    int i;
    /*
     * First unregister the TCA64xx I/O Expanders and associated I2C bus(ses).
     * Done in reverse order from registration to account for IRQ chaining
     * between I/O Expander chips.
     */
    for (i = db3_board_info.nr_io_expanders - 1; i >= 0; i--) {
        struct io_expander_info *io_exp = &db3_board_info.io_expanders[i];

        if (io_exp->io_exp_driver_data)
            tca64xx_deinit(io_exp->io_exp_driver_data);

        if (io_exp->i2c_dev)
            up_i2cuninitialize(io_exp->i2c_dev);
    }

    /* Lastly unregister the GPIO Chip driver */
    stm32_gpio_deinit();
}
