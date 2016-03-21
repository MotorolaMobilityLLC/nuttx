/************************************************************************************
 *
 *   Copyright (C) 2016 Motorola Mobility, LLC. All rights reserved.
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

#include <debug.h>
#include <errno.h>

#include <nuttx/config.h>
#include <nuttx/device.h>
#include <nuttx/device_battery.h>
#include <nuttx/device_battery_good.h>
#include <nuttx/device_display.h>
#include <nuttx/device_hid.h>
#include <nuttx/device_ptp.h>
#include <nuttx/device_ptp_chg.h>
#include <nuttx/device_raw.h>
#include <nuttx/device_sensors_ext.h>
#include <nuttx/device_slave_pwrctrl.h>
#include <nuttx/device_table.h>
#include <nuttx/device_uart.h>

#include <nuttx/power/battery_state.h>
#include <nuttx/util.h>

#include <nuttx/mhb/device_mhb.h>

#include <arch/board/board.h>
#include <arch/board/mods.h>

#include "up_arch.h"
#include "hdk.h"

#include <nuttx/gpio/stm32_gpio_chip.h>

struct board_gpio_cfg_s
{
  uint8_t  pin;
  uint32_t cfgset;
};

static const struct board_gpio_cfg_s board_gpio_cfgs[] =
{
  { GPIO_MODS_SL_BPLUS_EN,   (GPIO_PULLUP)           },
  { GPIO_MODS_CHG_PG_N,      (GPIO_INPUT|GPIO_FLOAT) },
  { GPIO_MODS_SPI_CS_N,      (GPIO_SPI2_NSS)         },
};

#ifdef CONFIG_DEVICE_CORE
typedef enum {
#ifdef CONFIG_GREYBUS_SENSORS_EXT_DUMMY_ACCEL
    DUMMY_ACCEL,
#endif
#ifdef CONFIG_GREYBUS_SENSORS_EXT_DUMMY_PRESSURE
    DUMMY_PRESSURE,
#endif
    SENSORS_TOTAL,
} sensor_type;

#ifdef CONFIG_STM32_UART_DEVICE
static struct device_resource stm32_uart_resources[] = {
    {
        .name   = "phy_id",
        .type   = DEVICE_RESOURCE_TYPE_REGS,
        .start  = 1, /* USART1 */
        .count  = 1,
    },
};
#endif

#ifdef CONFIG_MHB_UART
static struct device_resource mhb_resources[] = {
    {
        .name   = "uart_dev_id",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = 1, /* Maps to the DEVICE_TYPE_UART_HW instance. */
        .count  = 1,
    },
    {
        .name   = "peer_wake",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = 24, /* PB8 */
        .count  = 1,
    },
};
#endif

#ifdef CONFIG_MHB_DSI_DISPLAY
static struct device_resource dsi_display_resources[] = {
};
#endif

static struct device devices[] = {
#ifdef CONFIG_GREYBUS_SENSORS_EXT_DUMMY_PRESSURE
    {
        .type = DEVICE_TYPE_SENSORS_HW,
        .name = "sensors_ext_pressure",
        .desc = "Sensors Extension Protocol",
        .id   = DUMMY_PRESSURE,
    },
#endif
#ifdef CONFIG_GREYBUS_SENSORS_EXT_DUMMY_ACCEL
    {
        .type = DEVICE_TYPE_SENSORS_HW,
        .name = "sensors_ext_accel",
        .desc = "Sensors Extension Protocol",
        .id   = DUMMY_ACCEL,
    },
#endif
#ifdef CONFIG_MODS_RAW
    {
        .type = DEVICE_TYPE_RAW_HW,
        .name = "mods_raw",
        .desc = "Reference Raw Interface",
        .id   = 0,
    },
#endif
#ifdef CONFIG_GREYBUS_MODS_PTP_DEVICE_BATTERY
    {
        .type = DEVICE_TYPE_PTP_HW,
        .name = "battery_ptp",
        .desc = "Power transfer protocol for devices with a battery",
        .id   = 0,
    },
#endif
#ifdef CONFIG_GREYBUS_PTP_CHG_DEVICE_BQ24292
    {
        .type = DEVICE_TYPE_PTP_CHG_HW,
        .name = "bq2429_ptp_chg",
        .desc = "BQ24292 based charger driver for power transfer protocol",
        .id   = 0,
    },
#endif
#ifdef CONFIG_MAX17050_DEVICE
    {
        .type = DEVICE_TYPE_BATTERY_DEVICE,
        .name = "max17050_battery",
        .desc = "MAX17050 Battery Driver",
        .id   = 0,
    },
#endif

#ifdef CONFIG_MHB_APBE_CTRL_DEVICE
    /* For GB Mods Control Protocol */
    {
        .type = DEVICE_TYPE_SLAVE_PWRCTRL_HW,
        .name = "slave_pwrctrl",
        .desc = "slave power control",
        .id   = 0,
    },
#ifdef CONFIG_MHB_UART
    /* For APBE-Control */
    {
        .type = DEVICE_TYPE_MHB,
        .name = "mhb",
        .desc = "mhb",
        .id   = MHB_ADDR_PM,
        .resources = mhb_resources,
        .resource_count = ARRAY_SIZE(mhb_resources),
    },
#endif
#endif

#ifdef CONFIG_BATTERY_GOOD_DEVICE_COMP
    {
        .type = DEVICE_TYPE_BATTERY_GOOD_HW,
        .name = "comp_batt_good",
        .desc = "Battery good detection with voltage comparator",
        .id   = 0,
    },
#endif
#ifdef CONFIG_HDMI_DISPLAY
    {
        .type = DEVICE_TYPE_DISPLAY_HW,
        .name = "hdmi_display",
        .desc = "HDMI Display",
        .id   = 0,
    },
#endif
#ifdef CONFIG_STM32_UART_DEVICE
    {
        .type = DEVICE_TYPE_UART_HW,
        .name = "stm32_uart",
        .desc = "stm32 uart",
        .id   = 1, /* /dev/ttyS1 */
        .resources = stm32_uart_resources,
        .resource_count = ARRAY_SIZE(stm32_uart_resources),
    },
#endif

#ifdef CONFIG_MODS_MHB_CLIENT
    {
        .type = DEVICE_TYPE_MHB,
        .name = "mhb",
        .desc = "mhb",
        .id   = MHB_ADDR_DIAG,
        .resources = mhb_resources,
        .resource_count = ARRAY_SIZE(mhb_resources),
    },
#endif

#ifdef CONFIG_MHB_DSI_DISPLAY
    {
        .type = DEVICE_TYPE_DISPLAY_HW,
        .name = "mhb_dsi_display",
        .desc = "MHB DSI Display",
        .id   = 0, /* Must match device_open() in gb_mods_display_init() */
        .resources      = dsi_display_resources,
        .resource_count = ARRAY_SIZE(dsi_display_resources),
    },
# ifdef CONFIG_MHB_UART
    /* For DSI Display */
    {
        .type = DEVICE_TYPE_MHB,
        .name = "mhb",
        .desc = "mhb",
        .id   = MHB_ADDR_CDSI0,
        .resources = mhb_resources,
        .resource_count = ARRAY_SIZE(mhb_resources),
    },
# endif
# ifdef CONFIG_MHB_APBE_CTRL_DEVICE
    /* For DSI Display */
    {
        .type = DEVICE_TYPE_SLAVE_PWRCTRL_HW,
        .name = "slave_pwrctrl",
        .desc = "slave power control",
        .id   = MHB_ADDR_CDSI0,
    },
# endif
#endif

};

static struct device_table muc_device_table = {
    .device = devices,
    .device_count = ARRAY_SIZE(devices),
};
#endif

/* Map pin number to cfgset used by the STM32 GPIO framework */
int map_pin_nr_to_cfgset(uint8_t pin, uint32_t *cfgset)
{
  int i;

  if (pin > STM32_NGPIO)
    return -EINVAL;

  // 16 pins per port
  *cfgset = ((pin / 16) << GPIO_PORT_SHIFT) |
            ((pin % 16) << GPIO_PIN_SHIFT);

  for (i = 0; i < ARRAY_SIZE(board_gpio_cfgs); ++i)
    {
      if (board_gpio_cfgs[i].pin == pin)
        {
          // Additional pin configuration found
          *cfgset |= board_gpio_cfgs[i].cfgset;
        }
    }

  return 0;
}

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void stm32_boardinitialize(void)
{
  uint32_t regval;

  /* VDDIO2 is valid on this board */

  regval  = getreg32(STM32_PWR_CR2);
  regval |= PWR_CR2_IOSV;
  putreg32(regval, STM32_PWR_CR2);

#ifdef CONFIG_STM32_SPI_DMA
  /* set up DMA Channels for SPI */
  regval  = DMA1_CSELR_CHAN2_SPI1_RX;
  regval |= DMA1_CSELR_CHAN3_SPI1_TX;
  regval |= DMA1_CSELR_CHAN4_SPI2_RX;
  regval |= DMA1_CSELR_CHAN5_SPI2_TX;
  putreg32(regval, STM32_DMA1_CSELR);
#endif
}

/****************************************************************************
 * Name: board_initialize
 *
 * Description:
 *   If CONFIG_BOARD_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_initialize().  board_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_INITIALIZE
void board_initialize(void)
{
#if defined(CONFIG_GPIO_CHIP_STM32)
  stm32_gpio_init();
#endif

#ifdef CONFIG_MODS_DIET
  mods_init();
#endif

#if defined(CONFIG_BATTERY_STATE)
  battery_state_init();
#endif

#ifdef CONFIG_DEVICE_CORE
  device_table_register(&muc_device_table);

#ifdef CONFIG_MODS_RAW
  extern struct device_driver mods_raw_driver;
  device_register_driver(&mods_raw_driver);
#endif
#ifdef CONFIG_GREYBUS_MODS_PTP_DEVICE_BATTERY
  extern struct device_driver batt_ptp_driver;
  device_register_driver(&batt_ptp_driver);
#endif
#ifdef CONFIG_GREYBUS_PTP_CHG_DEVICE_BQ24292
  extern struct device_driver bq24292_ptp_chg_driver;
  device_register_driver(&bq24292_ptp_chg_driver);
#endif
#ifdef CONFIG_MAX17050_DEVICE
  extern struct device_driver batt_driver;
  device_register_driver(&batt_driver);
#endif
#ifdef CONFIG_MHB_APBE_CTRL_DEVICE
  extern struct device_driver apbe_pwrctrl_driver;
  device_register_driver(&apbe_pwrctrl_driver);
#endif
#ifdef CONFIG_BATTERY_GOOD_DEVICE_COMP
  extern struct device_driver comp_batt_good_driver;
  device_register_driver(&comp_batt_good_driver);
#endif
#ifdef CONFIG_GREYBUS_SENSORS_EXT_DUMMY_PRESSURE
  extern struct device_driver sensor_dummy_pressure_driver;
  device_register_driver(&sensor_dummy_pressure_driver);
#endif
#ifdef CONFIG_GREYBUS_SENSORS_EXT_DUMMY_ACCEL
  extern struct device_driver sensor_dummy_accel_driver;
  device_register_driver(&sensor_dummy_accel_driver);
#endif
#ifdef CONFIG_HDMI_DISPLAY
   extern struct device_driver hdmi_display_driver;
   device_register_driver(&hdmi_display_driver);
#endif
#ifdef CONFIG_STM32_UART_DEVICE
  extern struct device_driver stm32_uart_driver;
  device_register_driver(&stm32_uart_driver);
#endif
#if CONFIG_MHB_UART
   extern struct device_driver mhb_driver;
   device_register_driver(&mhb_driver);
#endif
#ifdef CONFIG_MHB_DSI_DISPLAY
   extern struct device_driver dsi_display_driver;
   device_register_driver(&dsi_display_driver);
#endif

#endif

}
#endif
