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
#include <nuttx/device_table.h>

{%- if config_battery is defined and config_battery %}
#include <nuttx/device_battery.h>
#include <nuttx/device_battery_good.h>
#include <nuttx/device_battery_level.h>
#include <nuttx/device_battery_temp.h>
#include <nuttx/device_battery_voltage.h>
{% endif %}
{%- if config_camera is defined and config_camera %}
#include <nuttx/device_cam_ext.h>
{% endif -%}
{%- if config_display is defined and config_display %}
#include <nuttx/device_display.h>
{% endif -%}
{%- if config_usb_charger is defined and config_usb_charger %}
#include <nuttx/device_ext_power.h>
{% endif -%}
{%- if config_display is defined and config_display %}
#include <nuttx/device_lights.h>
{% endif -%}
{%- if config_battery is defined and config_battery or
       config_usb_charger is defined and config_usb_charger %}
#include <nuttx/device_ptp.h>
#include <nuttx/device_ptp_chg.h>
{% endif -%}
{%- if config_raw is defined and config_raw %}
#include <nuttx/device_raw.h>
{% endif -%}
{%- if config_sensors is defined and config_sensors %}
#include <nuttx/device_sensors_ext.h>
{% endif -%}
{%- if config_display is defined and config_display %}
#include <nuttx/device_slave_pwrctrl.h>
#include <nuttx/device_uart.h>
{% endif -%}
{%- if config_speaker is defined and config_speaker %}
#include <nuttx/device_audio.h>
#include <nuttx/device_i2s.h>
{% endif -%}
{%- if config_camera is defined and config_camera %}
#include <nuttx/device_mhb_cam.h>
{% endif %}
{%- if config_usb_data is defined and config_usb_data %}
#include <nuttx/device_usb_ext.h>
{% endif -%}
{%- if (config_usb_data is defined and config_usb_data) or
       (config_usb_charger is defined and config_usb_charger) %}
#include <nuttx/fusb302.h>
{% endif -%}

{%- if config_battery is defined and config_battery %}
#include <nuttx/power/battery_state.h>
#include <nuttx/power/bq25896.h>
{% endif -%}
{%- if config_usb_charger is defined and config_usb_charger %}
#include <nuttx/power/ext_power.h>
{% endif %}

#include <nuttx/util.h>

{%- if config_display is defined and config_display %}
#include <nuttx/mhb/device_mhb.h>
{% endif %}
#include <arch/board/board.h>
#include <arch/board/mods.h>
{%- if config_display is defined and config_display %}
#include <arch/board/stm32_lptim1.h>
{% endif %}
#include "up_arch.h"
#include "{{ board }}.h"

#include <nuttx/gpio/stm32_gpio_chip.h>

struct board_gpio_cfg_s
{
  uint8_t  pin;
  uint32_t cfgset;
};

static const struct board_gpio_cfg_s board_gpio_cfgs[] =
{
{%- if config_battery is defined and config_battery %}
  { GPIO_MODS_CC_ALERT,      (GPIO_PULLUP)                },
{% endif %}
{%- if (config_usb_data is defined and config_usb_data) or
       (config_usb_charger is defined and config_usb_charger) %}
  { GPIO_MODS_FUSB302_INT_N, (GPIO_INPUT|GPIO_FLOAT)      },
{% endif %}
  { GPIO_MODS_CHG_PG_N,      (GPIO_INPUT|GPIO_FLOAT)      },
  { GPIO_MODS_SPI_CS_N,      (GPIO_SPI2_NSS)              },
  { GPIO_MODS_LED_DRV_1,     (GPIO_OPENDRAIN)             },
  { GPIO_MODS_LED_DRV_2,     (GPIO_OPENDRAIN)             },
  { GPIO_MODS_LED_DRV_3,     (GPIO_OPENDRAIN)             },
  { GPIO_MODS_PCARD_DET_N,   (GPIO_PULLUP)                },
};
{% if config_display is defined and config_display %}
static struct device_resource stm32_uart_resources[] = {
    {
        .name   = "phy_id",
        .type   = DEVICE_RESOURCE_TYPE_REGS,
        .start  = 1, /* USART1 */
        .count  = 1,
    },
};

static struct device_resource mhb_resources[] = {
    {
        .name   = "uart_dev_id",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = 1, /* Maps to the DEVICE_TYPE_UART_HW instance. */
        .count  = 1,
    },
    {
        .name   = "local_wake",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = GPIO_APBE_INT_N,
        .count  = 1,
    },
    {
        .name   = "peer_wake",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = GPIO_APBE_WAKE,
        .count  = 1,
    },
};

static struct device_resource dsi_display_resources[] = {
    {
        .name   = "pwr1_en",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = GPIO_DISPLAY_PWR1_EN,
        .count  = 1,
    },
    {
        .name   = "disp_rst1_n",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = GPIO_DISPLAY_RST1_N,
        .count  = 1,
    },
};

static struct device_resource lm27965_resources[] = {
    {
        .name   = "gpio_reset",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = GPIO_BACKLIGHT_RST_N,
        .count  = 1,
    },
};
{% endif %}

{%- if config_battery is defined and config_battery %}
static struct battery_dev_s *g_battery;

struct battery_dev_s *get_battery(void)
{
    return g_battery;
}

#define MAX17050_I2C_BUS          3
#define MAX17050_I2C_FREQ         400000

static struct device_resource bq25896_charger_resources[] = {
    {
       .name   = "int_n",
       .type   = DEVICE_RESOURCE_TYPE_GPIO,
       .start  = GPIO_MODS_CHG_INT_N,
       .count  = 1,
    },
    {
       .name   = "chg_en",
       .type   = DEVICE_RESOURCE_TYPE_GPIO,
       .start  = GPIO_MODS_CHG_EN,
       .count  = 1,
    },
};

static struct bq25896_reg bq25896_regs[] = {
    { BQ25896_REG02, BQ25896_REG02_ICO_MASK,        BQ25896_REG02_ICO_DIS        },
    { BQ25896_REG03, BQ25896_REG03_MIN_VBAT_MASK,   BQ25896_REG03_MIN_VBAT_2500MV},
    { BQ25896_REG05, BQ25896_REG05_IPRECHG_MASK,    BQ25896_REG05_IPRECHG_64MA   },
    { BQ25896_REG06, BQ25896_REG06_BATLOWV_MASK,    BQ25896_REG06_BATLOWV_2800MV },
    { BQ25896_REG07, BQ25896_REG07_WATCHDOG_MASK | BQ25896_REG07_TERM_MASK,
                     BQ25896_REG07_WATCHDOG_DIS  | BQ25896_REG07_TERM_DIS        },
};

struct bq25896_config bq25896_charger_init_data = {
    .reg        = bq25896_regs,
    .reg_num    = sizeof(bq25896_regs) / sizeof(struct bq25896_reg),
};

{% endif %}
{%- if config_usb_charger is defined and config_usb_charger %}
static struct device_resource switch_ptp_chg_resources[] = {
    {
       .name   = "base_path",
       .type   = DEVICE_RESOURCE_TYPE_GPIO,
       .start  = GPIO_MODS_CHG_VINA_EN,
       .count  = 1,
    },
    {
       .name   = "wrd_path",
       .type   = DEVICE_RESOURCE_TYPE_GPIO,
       .start  = GPIO_MODS_CHG_VINB_EN,
       .count  = 1,
    },
};

struct ptp_chg_init_data switch_ptp_chg_init_data = {
       .wls_active_low = false,
       .wrd_active_low = false,
       .base_active_low = false,
};
{% endif %}
{%- if config_usb_data is defined and config_usb_data %}
struct device_resource fusb302_usb_ext_resources[] = {
    {
       .name = "path",
       .type = DEVICE_RESOURCE_TYPE_REGS,
       .start = GB_USB_EXT_PATH_A,
       .count = 1,
    }
};
{% endif %}

{%- if config_speaker is defined and config_speaker %}
static struct device_resource tfa9890_audio_resources[] = {
    {
       .name   = "audio_en",
       .type   = DEVICE_RESOURCE_TYPE_GPIO,
       .start  = GPIO_MODS_DEMO_ENABLE,
       .count  = 1,
    },
    {
       .name   = "rst_ls",
       .type   = DEVICE_RESOURCE_TYPE_GPIO,
       .start  = GPIO_MODS_RST_LS,
       .count  = 1,
    },
};
{% endif %}

#ifdef CONFIG_DEVICE_CORE
static struct device devices[] = {
{%- if config_raw is defined and config_raw %}
    {
        .type = DEVICE_TYPE_RAW_HW,
        .name = "mods_raw",
        .desc = "Reference Raw Interface",
        .id   = 0,
    },
{% endif %}
{%- if config_battery is defined and config_battery %}
    {
        .type = DEVICE_TYPE_PTP_HW,
        .name = "mods_ptp",
        .desc = "Power transfer protocol for devices",
        .id   = 0,
    },
    {
        .type = DEVICE_TYPE_CHARGER_HW,
        .name = "bq25896_charger",
        .desc = "Charger driver for TI bq25896 IC",
        .id   = 0,
        .resources      = bq25896_charger_resources,
        .resource_count = ARRAY_SIZE(bq25896_charger_resources),
        .init_data      = &bq25896_charger_init_data,
    },
    {
        .type = DEVICE_TYPE_PTP_CHG_HW,
        .name = "switch_ptp_chg",
        .desc = "Charger driver with switches for power transfer protocol",
        .id   = 0,
{%- if config_usb_data is defined and config_usb_data %}
        .resources      = switch_ptp_chg_resources,
        .resource_count = ARRAY_SIZE(switch_ptp_chg_resources),
        .init_data = &switch_ptp_chg_init_data,
{% endif %}
    },
    {
        .type = DEVICE_TYPE_BATTERY_DEVICE,
        .name = "max17050_battery",
        .desc = "MAX17050 Battery Driver",
        .id   = 0,
    },
    {
        .type = DEVICE_TYPE_BATTERY_TEMP_HW,
        .name = "max17050_battery_temp",
        .desc = "Battery temperature monitoring with MAX17050",
        .id   = 0,
    },
    {
        .type = DEVICE_TYPE_BATTERY_LEVEL_HW,
        .name = "max17050_battery_level",
        .desc = "Battery level monitoring with MAX17050",
        .id   = 0,
    },
    {
        .type = DEVICE_TYPE_BATTERY_VOLTAGE_HW,
        .name = "max17050_battery_voltage",
        .desc = "Battery voltage monitoring with MAX17050",
        .id   = 0,
    },
    {
        .type = DEVICE_TYPE_BATTERY_GOOD_HW,
        .name = "comp_batt_good",
        .desc = "Battery good detection with voltage comparator",
        .id   = 0,
    },
{% endif %}
{%- if config_display is defined and config_display %}
    /* For GB Mods Control Protocol */
    {
        .type = DEVICE_TYPE_SLAVE_PWRCTRL_HW,
        .name = "slave_pwrctrl",
        .desc = "slave power control",
        .id   = 0,
    },
    /* For APBE-Control */
    {
        .type = DEVICE_TYPE_MHB,
        .name = "mhb",
        .desc = "mhb",
        .id   = MHB_ADDR_PM,
        .resources = mhb_resources,
        .resource_count = ARRAY_SIZE(mhb_resources),
    },
    {
        .type = DEVICE_TYPE_UART_HW,
        .name = "stm32_uart",
        .desc = "stm32 uart",
        .id   = 1, /* /dev/ttyS1 */
        .resources = stm32_uart_resources,
        .resource_count = ARRAY_SIZE(stm32_uart_resources),
    },
    {
        .type = DEVICE_TYPE_MHB,
        .name = "mhb",
        .desc = "mhb",
        .id   = MHB_ADDR_DIAG,
        .resources = mhb_resources,
        .resource_count = ARRAY_SIZE(mhb_resources),
    },
    /* Add a power device to allow the client to power the apbe on and off. */
    {
        .type = DEVICE_TYPE_SLAVE_PWRCTRL_HW,
        .name = "slave_pwrctrl",
        .desc = "slave power control",
        .id   = MHB_ADDR_DIAG,
    },
    {
        .type = DEVICE_TYPE_DISPLAY_HW,
        .name = "mhb_dsi_display",
        .desc = "MHB DSI Display",
        .id   = 0, /* Must match device_open() in gb_mods_display_init() */
        .resources      = dsi_display_resources,
        .resource_count = ARRAY_SIZE(dsi_display_resources),
    },
    /* For DSI Display */
    {
        .type = DEVICE_TYPE_MHB,
        .name = "mhb",
        .desc = "mhb",
        .id   = MHB_ADDR_CDSI0,
        .resources = mhb_resources,
        .resource_count = ARRAY_SIZE(mhb_resources),
    },
    /* For DSI Display */
    {
        .type = DEVICE_TYPE_SLAVE_PWRCTRL_HW,
        .name = "slave_pwrctrl",
        .desc = "slave power control",
        .id   = MHB_ADDR_CDSI0,
    },
    {
        .type = DEVICE_TYPE_LIGHTS_HW,
        .name = "lm27965_backlight",
        .desc = "LM27965 Backlight",
        .id   = 0,
        .resources = lm27965_resources,
        .resource_count = ARRAY_SIZE(lm27965_resources),
    },
{% endif %}
{%- if config_usb_charger is defined and config_usb_charger %}
    {
        .type = DEVICE_TYPE_PTP_CHG_HW,
        .name = "switch_ptp_chg",
        .desc = "Charger driver with switches for power transfer protocol",
        .id   = 0,
        .resources      = switch_ptp_chg_resources,
        .resource_count = ARRAY_SIZE(switch_ptp_chg_resources),
        .init_data = &switch_ptp_chg_init_data,
    },
    {
        .type = DEVICE_TYPE_EXT_POWER_HW,
        .name = "fusb302_ext_power",
        .desc = "FUSB 302 External Power",
        .id   = EXT_POWER_WIRED,
    },
{% endif -%}
{%- if (config_usb_charger is defined and config_usb_charger) or
       (config_usb_data is defined and config_usb_data) %}
    {
        .type = DEVICE_TYPE_USB_EXT_HW,
        .name = "fusb302_usb_ext",
        .desc = "USB-EXT Interface",
        .id   = 0,
        .resources = fusb302_usb_ext_resources,
        .resource_count = ARRAY_SIZE(fusb302_usb_ext_resources),
    },
{% endif -%}
{%- if config_speaker is defined and config_speaker %}
    {
        .type = DEVICE_TYPE_I2S_HW,
        .name   = "tfa9890_i2s_direct_driver",
        .desc   = "TFA9890 I2S Direct Driver",
        .id   = 1,
    },
    {
        .type = DEVICE_TYPE_MUC_AUD_HW,
        .name   = "audio_tfa9890_device_driver",
        .desc   = "Audio Tfa9890 Device Driver",
        .id   = 2,
        .resources = tfa9890_audio_resources,
        .resource_count = ARRAY_SIZE(tfa9890_audio_resources),
    },
{% endif -%}
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
  dbg("%d MHz\n", STM32_SYSCLK_FREQUENCY / 1000000);

  stm32_gpio_init();

{%- if config_display is defined and config_display %}
  stm32_lptim1_on();
{% endif %}
  stm32_spiinitialize();

  mods_init();

#ifndef CONFIG_BOARD_INITTHREAD
# error "Must enable INITTHREAD"
#endif

#ifdef CONFIG_DEVICE_CORE
  device_table_register(&muc_device_table);
#endif

{%- if (config_usb_data is defined and config_usb_data) or
       (config_usb_charger is defined and config_usb_charger) %}
       fusb302_register(GPIO_MODS_FUSB302_INT_N, GPIO_MODS_VBUS_PWR_EN);
{% endif %}
{%- if config_usb_data is defined and config_usb_data %}
  /**********************************************************************/
  /* USB DATA                                                           */
  /**********************************************************************/
  extern struct device_driver fusb302_usb_ext_driver;
  device_register_driver(&fusb302_usb_ext_driver);
{% endif %}
{%- if config_usb_charger is defined and config_usb_charger %}
  /**********************************************************************/
  /* USB CHARGING                                                       */
  /**********************************************************************/
  extern struct device_driver fusb302_ext_power_driver;
  device_register_driver(&fusb302_ext_power_driver);
{% endif %}
{%- if config_raw is defined and config_raw %}
  /**********************************************************************/
  /* RAW                                                                */
  /**********************************************************************/
  extern struct device_driver mods_raw_driver;
  device_register_driver(&mods_raw_driver);
{% endif %}
{%- if config_battery is defined and config_battery %}
  /**********************************************************************/
  /* BATTERY and CHARGING                                               */
  /**********************************************************************/
  extern struct device_driver mods_ptp_driver;
  device_register_driver(&mods_ptp_driver);

  extern struct device_driver bq25896_charger_driver;
  device_register_driver(&bq25896_charger_driver);

  extern struct device_driver switch_ptp_chg_driver;
  device_register_driver(&switch_ptp_chg_driver);

  extern struct device_driver batt_driver;
  device_register_driver(&batt_driver);

  extern struct device_driver max17050_battery_temp_driver;
  device_register_driver(&max17050_battery_temp_driver);

  extern struct device_driver max17050_battery_level_driver;
  device_register_driver(&max17050_battery_level_driver);

  extern struct device_driver comp_batt_good_driver;
  device_register_driver(&comp_batt_good_driver);

   struct i2c_dev_s *i2c = up_i2cinitialize(MAX17050_I2C_BUS);
   if (i2c) {
      g_battery = max17050_initialize(i2c, MAX17050_I2C_FREQ,
                  GPIO_MODS_CC_ALERT);
      if (!g_battery) {
         up_i2cuninitialize(i2c);
      }
   }

   /* Must be initialized after MAX17050 core driver has been initialized */
   battery_state_init();

{% endif %}
{%- if config_display is defined and config_display %}
  /**********************************************************************/
  /* DISPLAY and BACKLIGHT                                              */
  /**********************************************************************/

  extern struct device_driver apbe_pwrctrl_driver;
  device_register_driver(&apbe_pwrctrl_driver);

  extern struct device_driver stm32_uart_driver;
  device_register_driver(&stm32_uart_driver);

  extern struct device_driver mhb_driver;
  device_register_driver(&mhb_driver);

  extern struct device_driver dsi_display_driver;
  device_register_driver(&dsi_display_driver);

  extern struct device_driver lm27965_backlight_driver;
  device_register_driver(&lm27965_backlight_driver);
{% endif %}

{%- if config_speaker is defined and config_speaker %}
  extern struct device_driver tfa9890_i2s_direct_driver;
  device_register_driver(&tfa9890_i2s_direct_driver);
  extern struct device_driver tfa9890_audio_dev_driver;
  device_register_driver(&tfa9890_audio_dev_driver);
{% endif %}

{%- if (not config_battery is defined or not config_battery) and
    (not config_usb_charger is defined or not config_usb_charger) %}
  /* Set the power paths to be able to use USB VBUS as the system power and
   * to prevent applying voltage to VBUS pin on the Mod connector. Also,
   * prevent accepting power from the PCARD VBUS connection.
   */
  gpio_set_value(GPIO_MODS_CHG_VINA_EN, 1);
  gpio_set_value(GPIO_MODS_CHG_VINB_EN, 0);
{% endif %}
}
#endif
