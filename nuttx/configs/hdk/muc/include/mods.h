/****************************************************************************
 *
 *   Copyright (C) 2015-2016 Motorola Mobility, LLC. All rights reserved.
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

#ifndef _STM32_MODS_H_
#define _STM32_MODS_H_

#include <stdbool.h>
#include <nuttx/gpio.h>
#include <nuttx/power/battery.h>

#define BOARD_REVISION           (CONFIG_ARCH_BOARDID_PID & 0x000000FF)

#define IS_BATT_PCARD   ((CONFIG_ARCH_BOARDID_PID & 0x0000FF00) == 0x00000800)

#define CALC_GPIO_NUM(port, pin)  ((16 * (port - 'A')) + pin)

#define GPIO_MODS_CHG_VINA_EN    CALC_GPIO_NUM('D',  5)
#define GPIO_MODS_CHG_VINB_EN    CALC_GPIO_NUM('D',  8)

#define GPIO_MODS_VBUS_PWR_EN    CALC_GPIO_NUM('A', 11)
#define GPIO_MODS_WAKE_N         CALC_GPIO_NUM('B',  0)
#define GPIO_MODS_SPI_CS_N       CALC_GPIO_NUM('B', 12)
#define GPIO_MODS_SPI_TACK       CALC_GPIO_NUM('B', 14)
#define GPIO_MODS_SPI_RACK       CALC_GPIO_NUM('B', 15)
#define GPIO_MODS_RFR            CALC_GPIO_NUM('C',  2)
#define GPIO_MODS_SL_BPLUS_AIN   CALC_GPIO_NUM('C',  5)
#define GPIO_MODS_INT            CALC_GPIO_NUM('C', 13)
#define GPIO_MODS_KEY_POWER_PMIC CALC_GPIO_NUM('D',  2)
#define GPIO_MODS_LED_DRV_1      CALC_GPIO_NUM('D',  7)
#define GPIO_MODS_PCARD_DET_N    CALC_GPIO_NUM('D',  9)
#define GPIO_MODS_LED_DRV_2      CALC_GPIO_NUM('E',  7)
#define GPIO_MODS_LED_DRV_3      CALC_GPIO_NUM('E',  8)
#define GPIO_MODS_FUSB302_INT_N  CALC_GPIO_NUM('G', 13)

#define GPIO_APBE_UART_RX        CALC_GPIO_NUM('B',  7)
#define GPIO_APBE_UART_TX        CALC_GPIO_NUM('B',  6)
#define GPIO_APBE_UART_CTS       CALC_GPIO_NUM('G', 11)
#define GPIO_APBE_UART_RTS       CALC_GPIO_NUM('B',  3)

#define GPIO_APBE_SPIBOOT_N      CALC_GPIO_NUM('A',  8)
#define GPIO_APBE_INT_N          CALC_GPIO_NUM('B',  8)
#define GPIO_APBE_RST_N          CALC_GPIO_NUM('H',  1)
#define GPIO_APBE_PWR_EN         CALC_GPIO_NUM('G', 14)
#define GPIO_APBE_WAKE           CALC_GPIO_NUM('B',  5)

#define GPIO_DISPLAY_PWR1_EN     CALC_GPIO_NUM('G', 10)
#define GPIO_DISPLAY_RST1_N      CALC_GPIO_NUM('C', 8)

#define GPIO_BACKLIGHT_RST_N     CALC_GPIO_NUM('C', 12)

/* Note: Touch uses GPIO_DISPLAY_PWR1_EN, PG10, as well. */
#define GPIO_TOUCH_INT_N         CALC_GPIO_NUM('C', 7)
#define GPIO_TOUCH_EN_N          CALC_GPIO_NUM('G', 12)
#define GPIO_TOUCH_RST_N         CALC_GPIO_NUM('D', 6)

#define GPIO_FACT_DISP_PWR1_EN   CALC_GPIO_NUM('A', 10)
#define GPIO_FACT_DISP_PWR2_EN   CALC_GPIO_NUM('C', 12)
#define GPIO_FACT_DISP_PWR3_EN   CALC_GPIO_NUM('A', 0)
#define GPIO_FACT_DISP_PWR4_EN   CALC_GPIO_NUM('A', 2)
#define GPIO_FACT_DISP_RST1_N    CALC_GPIO_NUM('C', 8)

#define GPIO_MODS_CHG_PG_N       (-1)

#define GPIO_CAM_DVDD_EN         CALC_GPIO_NUM('A', 3)
#define GPIO_CAM_AREG_EN         CALC_GPIO_NUM('A', 2)
#define GPIO_CAM_RST_N           CALC_GPIO_NUM('A', 5)

#define GPIO_MODS_SPI_SEL        CALC_GPIO_NUM('C', 6)

/* Select the SL_BPLUS_AIN pin for base attach */
#define GPIO_MODS_BASE_ATTACH    GPIO_MODS_SL_BPLUS_AIN

#define GPIO_MODS_DEMO_ENABLE    CALC_GPIO_NUM('G', 10)
#define GPIO_MODS_RST_LS         CALC_GPIO_NUM('H',  0)

#if IS_BATT_PCARD
# define GPIO_MODS_CC_ALERT      CALC_GPIO_NUM('G', 12)
# define GPIO_MODS_CHG_INT_N     CALC_GPIO_NUM('C',  7)
# define GPIO_MODS_CHG_EN        GPIO_MODS_DEMO_ENABLE
#else
# define GPIO_MODS_CC_ALERT      CALC_GPIO_NUM('B',  1)
# define GPIO_MODS_CHG_INT_N     CALC_GPIO_NUM('C',  4)
# define GPIO_MODS_CHG_EN        CALC_GPIO_NUM('D',  4)
#endif

#define GPIO_SPI1_CS_N           CALC_GPIO_NUM('A',  4)

/* Battery voltage comparator */
#if IS_BATT_PCARD
# define BATT_COMP               STM32_COMP1             /* Comparator */
# define BATT_COMP_INP           STM32_COMP_INP_PIN_2    /* Input plus */
#else
# define BATT_COMP               STM32_COMP2             /* Comparator */
# define BATT_COMP_INP           STM32_COMP_INP_PIN_1    /* Input plus */
#endif
#define BATT_COMP_INM            STM32_COMP_INM_VREF     /* Input minus */
#define BATT_COMP_HYST           STM32_COMP_HYST_HIGH    /* Hysteresis */
#define BATT_COMP_SPEED          STM32_COMP_SPEED_LOW    /* Speed */
#define BATT_COMP_INV            false                   /* Output not inverted */

static inline void mods_rfr_init(void)
{
    /* On this board, RFR is active high */
    gpio_direction_out(GPIO_MODS_RFR, 0);
}

static inline void mods_rfr_set(uint8_t value)
{
    /* On this board, RFR is active high */
    gpio_set_value(GPIO_MODS_RFR, value);
}

static inline uint8_t mods_rfr_get(void)
{
    /* On this board, RFR is active high */
    return gpio_get_value(GPIO_MODS_RFR);
}

void mods_host_int_set(bool value);

struct battery_dev_s *get_battery(void);

#endif /* _STM32_MODS_H_ */
