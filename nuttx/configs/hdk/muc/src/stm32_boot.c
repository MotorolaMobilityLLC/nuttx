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
#include <nuttx/device_battery_level.h>
#include <nuttx/device_battery_temp.h>
#include <nuttx/device_battery_voltage.h>
#include <nuttx/device_cam_ext.h>
#include <nuttx/device_display.h>
#include <nuttx/device_ext_power.h>
#include <nuttx/device_hid.h>
#include <nuttx/device_lights.h>
#include <nuttx/device_ptp.h>
#include <nuttx/device_ptp_chg.h>
#include <nuttx/device_raw.h>
#include <nuttx/device_sensors_ext.h>
#include <nuttx/device_slave_pwrctrl.h>
#include <nuttx/device_table.h>
#include <nuttx/device_uart.h>
#include <nuttx/device_audio.h>
#include <nuttx/device_i2s.h>
#include <nuttx/device_mhb_cam.h>
#include <nuttx/device_usbtun.h>
#include <nuttx/device_usb_ext.h>
#include <nuttx/fusb302.h>

#include <nuttx/power/battery_state.h>
#include <nuttx/power/bq25896.h>
#include <nuttx/power/ext_power.h>
#include <nuttx/util.h>
#include <nuttx/usb.h>

#include <nuttx/mhb/device_mhb.h>

#include <arch/board/board.h>
#include <arch/board/mods.h>
#include <arch/board/stm32_lptim1.h>

#include "up_arch.h"
#include "hdk.h"
#include "stm32_adc_init.h"

#include <nuttx/gpio/stm32_gpio_chip.h>

struct board_gpio_cfg_s
{
  uint8_t  pin;
  uint32_t cfgset;
};

static const struct board_gpio_cfg_s board_gpio_cfgs[] =
{
  { GPIO_MODS_CC_ALERT,      (GPIO_PULLUP)                },
  { GPIO_MODS_FUSB302_INT_N, (GPIO_INPUT|GPIO_FLOAT)      },
  { GPIO_MODS_SPI_CS_N,      (GPIO_SPI2_NSS)              },
#ifdef CONFIG_CHARGING_MODS_DONGLE
  { GPIO_MODS_SPI_SCK,       (GPIO_OUTPUT)                },/* dongle detect */
#endif
  { GPIO_MODS_LED_DRV_1,     (GPIO_OPENDRAIN)             },
  { GPIO_MODS_LED_DRV_2,     (GPIO_OPENDRAIN)             },
  { GPIO_MODS_LED_DRV_3,     (GPIO_OPENDRAIN)             },
  { GPIO_MODS_PCARD_DET_N,   (GPIO_PULLUP)                },
};

#ifdef CONFIG_BATTERY_MAX17050
static struct battery_dev_s *g_battery;

struct battery_dev_s *get_battery(void)
{
    return g_battery;
}

# if IS_BATT_PCARD
#  define MAX17050_I2C_BUS          2
# else
#  define MAX17050_I2C_BUS          3
# endif
# define MAX17050_I2C_FREQ         400000
#endif


#ifdef CONFIG_DEVICE_CORE
typedef enum {
#ifdef CONFIG_GREYBUS_SENSORS_EXT_DUMMY_ACCEL
    DUMMY_ACCEL,
#endif
#ifdef CONFIG_GREYBUS_SENSORS_EXT_DUMMY_PRESSURE
    DUMMY_PRESSURE,
#endif
#ifdef CONFIG_GREYBUS_SENSORS_EXT_LSM9DS1_ACCEL
    LSM9DS1_ACCEL,
#endif
    SENSORS_TOTAL,
} sensor_type;

#ifdef CONFIG_GREYBUS_SENSORS_EXT_LSM9DS1_ACCEL
static struct device_resource lsm9ds1_resources[] = {
    {
        .name   = "i2c_bus",
        .type   = DEVICE_RESOURCE_TYPE_REGS,
        .start  = 2,  /* I2C2 */
        .count  = 1,
    },
};
#endif

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
#ifdef GPIO_APBE_INT_N
    {
        .name   = "local_wake",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = GPIO_APBE_INT_N,
        .count  = 1,
    },
#endif
#ifdef GPIO_APBE_WAKE
    {
        .name   = "peer_wake",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = GPIO_APBE_WAKE,
        .count  = 1,
    },
#endif
};
#endif

#ifdef CONFIG_MHB_DSI_DISPLAY
# if CONFIG_DISPLAY_NT35355_360P
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
# elif CONFIG_DISPLAY_SMD_470_720P || CONFIG_DISPLAY_TDI_546_1080P
static struct device_resource dsi_display_resources[] = {
    {
        .name   = "pwr1_en",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = GPIO_FACT_DISP_PWR1_EN,
        .count  = 1,
    },
    {
        .name   = "pwr2_en",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = GPIO_FACT_DISP_PWR2_EN,
        .count  = 1,
    },
    {
        .name   = "pwr3_en",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = GPIO_FACT_DISP_PWR3_EN,
        .count  = 1,
    },
    {
        .name   = "pwr4_en",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = GPIO_FACT_DISP_PWR4_EN,
        .count  = 1,
    },
    {
        .name   = "disp_rst1_n",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = GPIO_FACT_DISP_RST1_N,
        .count  = 1,
    },
};
# endif
#endif

#ifdef CONFIG_BACKLIGHT_LM27965
static struct device_resource lm27965_resources[] = {
    {
        .name   = "gpio_reset",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = GPIO_BACKLIGHT_RST_N,
        .count  = 1,
    },
};
#endif

#ifdef CONFIG_CHARGER_DEVICE_BQ25896
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
#endif

#if defined (CONFIG_GREYBUS_MODS_PTP_CHG_DEVICE_SWITCH) && defined (CONFIG_GREYBUS_PTP_EXT_SUPPORTED)
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
#endif
#ifdef CONFIG_FUSB302_USB_EXT
struct device_resource fusb302_usb_ext_resources[] = {
    {
       .name = "path",
       .type = DEVICE_RESOURCE_TYPE_REGS,
       .start = GB_USB_EXT_PATH_A,
       .count = 1,
    }
};
#endif

#if defined(CONFIG_CAMERA_IMX220) || defined(CONFIG_CAMERA_IMX230)
static struct device_resource cam_resources[] = {
    {
        .name   = "rst_n",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = GPIO_CAM_RST_N,
        .count  = 1,
    },
    {
        .name    = "dvdd_en",
        .type    = DEVICE_RESOURCE_TYPE_GPIO,
        .start    = GPIO_CAM_DVDD_EN,
        .count    = 1,
    },
    {
        .name    = "areg_en",
        .type    = DEVICE_RESOURCE_TYPE_GPIO,
        .start    = GPIO_CAM_AREG_EN,
        .count    = 1,
    },
    {
        .name    = "spi_sel",
        .type    = DEVICE_RESOURCE_TYPE_GPIO,
        .start    = GPIO_MODS_SPI_SEL,
        .count    = 1,
    },
};
#endif

#if defined(CONFIG_CAMERA_OV5647_PI) || defined(CONFIG_CAMERA_IMX219_PI)
static struct device_resource cam_resources[] = {
    {
        .name   = "rst_n",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = GPIO_PI_CAM_GPIO0,
        .count  = 1,
    },
    {
        .name    = "led_en",
        .type    = DEVICE_RESOURCE_TYPE_GPIO,
        .start    = GPIO_PI_CAM_GPIO1,
        .count    = 1,
    },
    {
        .name    = "spi_sel",
        .type    = DEVICE_RESOURCE_TYPE_GPIO,
        .start    = GPIO_MODS_SPI_SEL,
        .count    = 1,
    },
};
#endif

#if defined (CONFIG_MODS_AUDIO_TFA9890) || defined (CONFIG_MODS_MHB_AUDIO_TFA9890)
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
#endif

#ifdef CONFIG_CHARGING_MODS_DONGLE
const static uint32_t dongle_gpio_cfgset = GPIO_INPUT | GPIO_PULLUP | GPIO_MODS_SPI_SCK;

static struct device_resource dongle_resources[] = {
    {
       .name   = "capability",
       .type   = DEVICE_RESOURCE_TYPE_GPIO,
       .start  = GPIO_MODS_SPI_SCK,
       .count  = 1,
    },
    {
       .name = "gpio-cfgset",
       .type = DEVICE_RESOURCE_TYPE_REGS,
       .start = (uint32_t)&dongle_gpio_cfgset,
       .count = 1,
    }
};
#endif

#ifdef CONFIG_MODS_RAW_HSIC
static struct device_resource hsic_resources[] = {
    {
       .name   = "hub_rst_n",
       .type   = DEVICE_RESOURCE_TYPE_GPIO,
       .start  = GPIO_USBHUB_RST_N,
       .count  = 1,
    },
    {
       .name   = "susp_int",
       .type   = DEVICE_RESOURCE_TYPE_GPIO,
       .start  = GPIO_SUSP_IRQ_INT_N,
       .count  = 1,
    },
};
static struct device_resource hsic_usb_resources[] = {
    {
       .name   = "c_int_n",
       .type   = DEVICE_RESOURCE_TYPE_GPIO,
       .start  = GPIO_USBC_INT_N,
       .count  = 1,
    },
    {
       .name   = "vbus_ena",
       .type   = DEVICE_RESOURCE_TYPE_GPIO,
       .start  = GPIO_USBC_VBUS_ENA,
       .count  = 1,
    },
};
#endif

static struct device devices[] = {
#ifdef CONFIG_GREYBUS_SENSORS_EXT_DUMMY_PRESSURE
    {
        .type = DEVICE_TYPE_SENSORS_HW,
        .name = "sensors_ext_dummy_pressure",
        .desc = "Sensors Extension Protocol",
        .id   = DUMMY_PRESSURE,
    },
#endif
#ifdef CONFIG_MODS_USB2
    {
        .type = DEVICE_TYPE_USB_EXT_HW,
        .name = "usb_ext_usb2",
        .desc = "USB-EXT USB2 Interface",
        .id   = 0,
    },
#endif
#ifdef CONFIG_GREYBUS_SENSORS_EXT_DUMMY_ACCEL
    {
        .type = DEVICE_TYPE_SENSORS_HW,
        .name = "sensors_ext_dummy_accel",
        .desc = "Sensors Extension Protocol",
        .id   = DUMMY_ACCEL,
    },
#endif
#ifdef CONFIG_GREYBUS_SENSORS_EXT_LSM9DS1_ACCEL
    {
        .type = DEVICE_TYPE_SENSORS_HW,
        .name = "sensors_ext_accel",
        .desc = "Sensors Extension Protocol",
        .id   = LSM9DS1_ACCEL,
        .resources = lsm9ds1_resources,
        .resource_count = ARRAY_SIZE(lsm9ds1_resources),
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
#ifdef CONFIG_MODS_RAW_STUB
    {
        .type = DEVICE_TYPE_RAW_HW,
        .name = "mods_raw_stub",
        .desc = "Raw Stub with Thread/Queue",
        .id   = 0,
    },
#endif
#ifdef CONFIG_MODS_RAW_BLINKY
    {
        .type = DEVICE_TYPE_RAW_HW,
        .name = "mods_raw_blinky",
        .desc = "Blinky LED Raw Interface",
        .id   = 0,
    },
#endif
#ifdef CONFIG_MODS_RAW_TERMAPP
    {
        .type = DEVICE_TYPE_RAW_HW,
        .name = "mods_raw_termapp",
        .desc = "Termapp Raw Interface",
        .id   = 0,
    },
#endif
#ifdef CONFIG_MODS_RAW_TEMPERATURE
    {
        .type = DEVICE_TYPE_RAW_HW,
        .name = "mods_raw_temperature",
        .desc = "Temperature sensor Raw Interface",
        .id   = 0,
    },
#endif
#ifdef CONFIG_GREYBUS_MODS_PTP_DEVICE
    {
        .type = DEVICE_TYPE_PTP_HW,
        .name = "mods_ptp",
        .desc = "Power transfer protocol for devices",
        .id   = 0,
    },
#endif
#ifdef CONFIG_CHARGER_DEVICE_BQ25896
    {
        .type = DEVICE_TYPE_CHARGER_HW,
        .name = "bq25896_charger",
        .desc = "Charger driver for TI bq25896 IC",
        .id   = 0,
        .resources      = bq25896_charger_resources,
        .resource_count = ARRAY_SIZE(bq25896_charger_resources),
        .init_data      = &bq25896_charger_init_data,
    },
#endif
#ifdef CONFIG_GREYBUS_MODS_PTP_CHG_DEVICE_SWITCH
    {
        .type = DEVICE_TYPE_PTP_CHG_HW,
        .name = "switch_ptp_chg",
        .desc = "Charger driver with switches for power transfer protocol",
        .id   = 0,
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
        .resources      = switch_ptp_chg_resources,
        .resource_count = ARRAY_SIZE(switch_ptp_chg_resources),
        .init_data = &switch_ptp_chg_init_data,
#endif
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
#ifdef CONFIG_BATTERY_TEMP_DEVICE_MAX17050
    {
        .type = DEVICE_TYPE_BATTERY_TEMP_HW,
        .name = "max17050_battery_temp",
        .desc = "Battery temperature monitoring with MAX17050",
        .id   = 0,
    },
#endif
#ifdef CONFIG_BATTERY_LEVEL_DEVICE_MAX17050
    {
        .type = DEVICE_TYPE_BATTERY_LEVEL_HW,
        .name = "max17050_battery_level",
        .desc = "Battery level monitoring with MAX17050",
        .id   = 0,
    },
#endif
#ifdef CONFIG_BATTERY_VOLTAGE_DEVICE_MAX17050
    {
        .type = DEVICE_TYPE_BATTERY_VOLTAGE_HW,
        .name = "max17050_battery_voltage",
        .desc = "Battery voltage monitoring with MAX17050",
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
#ifdef CONFIG_MODS_RAW_FACTORY
        .id   = DISPLAY_TYPE_DP,
#else
        .id   = 0,
#endif
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
    /* Add a power device to allow the client to power the apbe on and off. */
    {
        .type = DEVICE_TYPE_SLAVE_PWRCTRL_HW,
        .name = "slave_pwrctrl",
        .desc = "slave power control",
        .id   = MHB_ADDR_DIAG,
    },
#endif

#ifdef CONFIG_MHB_DSI_DISPLAY
    {
        .type = DEVICE_TYPE_DISPLAY_HW,
        .name = "mhb_dsi_display",
        .desc = "MHB DSI Display",
#ifdef  CONFIG_MODS_RAW_FACTORY
        .id   = DISPLAY_TYPE_DSI,
#else
        .id   = 0, /* Must match device_open() in gb_mods_display_init() */
#endif
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

#ifdef CONFIG_BACKLIGHT_DCS
    {
        .type = DEVICE_TYPE_LIGHTS_HW,
        .name = "dcs_backlight",
        .desc = "DCS Backlight",
        .id   = 0,
    },
#endif
#ifdef CONFIG_BACKLIGHT_ISL98611
    {
        .type = DEVICE_TYPE_LIGHTS_HW,
        .name = "isl98611_backlight",
        .desc = "ISL98611 Backlight",
        .id   = 0,
    },
#endif
#ifdef CONFIG_BACKLIGHT_LM27965
    {
        .type = DEVICE_TYPE_LIGHTS_HW,
        .name = "lm27965_backlight",
        .desc = "LM27965 Backlight",
        .id   = 0,
        .resources = lm27965_resources,
        .resource_count = ARRAY_SIZE(lm27965_resources),
    },
#endif

#ifdef CONFIG_FUSB302_EXT_POWER
    {
        .type = DEVICE_TYPE_EXT_POWER_HW,
        .name = "fusb302_ext_power",
        .desc = "FUSB 302 External Power",
        .id   = EXT_POWER_WIRED,
    },
#endif
#ifdef CONFIG_FUSB302_USB_EXT
    {
        .type = DEVICE_TYPE_USB_EXT_HW,
        .name = "fusb302_usb_ext",
        .desc = "USB-EXT Interface",
        .id   = 0,
        .resources = fusb302_usb_ext_resources,
        .resource_count = ARRAY_SIZE(fusb302_usb_ext_resources),
    },
#endif
#ifdef CONFIG_CHARGING_MODS_DONGLE
    {
        .type = DEVICE_TYPE_EXT_POWER_HW,
        .name = "dongle_ext_power",
        .desc = "Charging Dongle",
        .id   = EXT_POWER_DONGLE,
        .resources = dongle_resources,
        .resource_count = ARRAY_SIZE(dongle_resources),
    },
#endif
#ifdef CONFIG_MHB_CAMERA
#ifdef CONFIG_MHB_UART
    /* For CSI Camera */
    {
        .type = DEVICE_TYPE_MHB,
        .name = "mhb",
        .desc = "mhb",
        .id   = MHB_ADDR_CDSI1,
        .resources = mhb_resources,
        .resource_count = ARRAY_SIZE(mhb_resources),
    },
#endif
#ifdef CONFIG_MHB_APBE_CTRL_DEVICE
    /* For CSI Camera */
    {
        .type = DEVICE_TYPE_SLAVE_PWRCTRL_HW,
        .name = "slave_pwrctrl",
        .desc = "slave power control",
        .id   = MHB_ADDR_CDSI1,
    },
#endif
    {
        .type = DEVICE_TYPE_CAMERA_EXT_HW,
        .name = "Motorola",
        .desc = "Motorola MHB Camera",
        .id   = 0,
    },
#endif /* CONFIG_MHB_CAMERA */
#if defined(CONFIG_CAMERA_IMX220) || defined(CONFIG_CAMERA_IMX230)
    {
        .type = DEVICE_TYPE_MHB_CAMERA_HW,
        .name = "Sony",
        .desc = "MHB Camera",
        .resources = cam_resources,
        .resource_count = ARRAY_SIZE(cam_resources),
        .id   = MHB_CAM_DRIVER_ID,
    },
#endif
#if defined(CONFIG_CAMERA_OV5647_PI)
    {
        .type = DEVICE_TYPE_MHB_CAMERA_HW,
        .name = "OV5647_PI",
        .desc = "Raspbery Pi Camera",
        .resources = cam_resources,
        .resource_count = ARRAY_SIZE(cam_resources),
        .id   = MHB_CAM_DRIVER_ID,
    },
#endif
#if defined(CONFIG_CAMERA_IMX219_PI)
    {
        .type = DEVICE_TYPE_MHB_CAMERA_HW,
        .name = "IMX219_PI",
        .desc = "Raspbery Pi Camera",
        .resources = cam_resources,
        .resource_count = ARRAY_SIZE(cam_resources),
        .id   = MHB_CAM_DRIVER_ID,
    },
#endif
#ifdef CONFIG_MODS_AUDIO_TFA9890
    {
        .type = DEVICE_TYPE_I2S_HW,
        .name   = "tfa9890_i2s_direct_driver",
        .desc   = "TFA9890 I2S Direct Driver",
        .id   = 1,
    },
#endif
#ifdef CONFIG_MODS_MHB_AUDIO_TFA9890
    {
        .type = DEVICE_TYPE_SLAVE_PWRCTRL_HW,
        .name = "slave_pwrctrl",
        .desc = "slave power control",
        .id   = MHB_ADDR_I2S,
    },
    /* For I2S Tunnel audio */
    {
        .type = DEVICE_TYPE_MHB,
        .name = "mhb",
        .desc = "mhb",
        .id   = MHB_ADDR_I2S,
    },
    {
        .type = DEVICE_TYPE_I2S_HW,
        .name = "mhb_i2s_audio",
        .desc = "MHB I2S Driver",
        .id   = 1,
    },
    {
        .type = DEVICE_TYPE_MUC_AUD_HW,
        .name   = "audio_tfa9890_device_driver",
        .desc   = "Audio Tfa9890 Device Driver",
        .id   = DEVICE_AUDIO_MHB_ID,
    },
#endif
#if defined (CONFIG_MODS_AUDIO_TFA9890) || defined (CONFIG_MODS_MHB_AUDIO_TFA9890)
    {
        .type = DEVICE_TYPE_MUC_AUD_HW,
        .name   = "audio_tfa9890_device_driver",
        .desc   = "Audio Tfa9890 Device Driver",
        .id   = 2,
        .resources = tfa9890_audio_resources,
        .resource_count = ARRAY_SIZE(tfa9890_audio_resources),
    },
#endif
#ifdef CONFIG_MODS_RAW_FACTORY
    {
        .type = DEVICE_TYPE_DISPLAY_HW,
        .name = "display_mux",
        .desc = "Display Mux Driver",
        .id   = 0,
    },
    {
        .type = DEVICE_TYPE_RAW_HW,
        .name = "mods_raw_factory",
        .desc = "Factory Raw Interface",
        .id   = 0,
    },
#endif

#ifdef CONFIG_MODS_RAW_HSIC
#ifdef CONFIG_MHB_UART
    {
        .type = DEVICE_TYPE_MHB,
        .name = "mhb",
        .desc = "mhb",
        .id   = MHB_ADDR_HSIC,
        .resources = mhb_resources,
        .resource_count = ARRAY_SIZE(mhb_resources),
    },
#endif
#ifdef CONFIG_MHB_APBE_CTRL_DEVICE
    {
        .type = DEVICE_TYPE_SLAVE_PWRCTRL_HW,
        .name = "slave_pwrctrl",
        .desc = "slave power control",
        .id   = MHB_ADDR_HSIC,
    },
#endif
    {
        .type = DEVICE_TYPE_RAW_HW,
        .name = "raw_hsic",
        .desc = "Raw HSIC Interface",
        .id   = 0,
    },
    {
        .type = DEVICE_TYPE_HSIC_DEVICE,
        .name = "usb3813",
        .desc = "USB3813 HSIC Hub",
        .id   = 0,
        .resources = hsic_resources,
        .resource_count = ARRAY_SIZE(hsic_resources),
    },
    {
        .type = DEVICE_TYPE_USB_EXT_HW,
        .name = "usb_ext",
        .desc = "USB-EXT Interface",
        .resources = hsic_usb_resources,
        .resource_count = ARRAY_SIZE(hsic_usb_resources),
        .id   = 0,
    },
    {
        .type = DEVICE_TYPE_USBTUN_HW,
        .name = "mhb_usbtun",
        .desc = "MHB USB Tunneling Driver",
        .id   = 0,
    },
#endif /* MODS_RAW_HSIC */
#ifdef CONFIG_MODS_HID_EXAMPLE
    {
        .type = DEVICE_TYPE_HID_HW,
        .name = "hid_game",
        .desc = "HID Game Controller",
    },
#endif
#ifdef CONFIG_MODS_RAW_FLIR
    {
        .type = DEVICE_TYPE_RAW_HW,
        .name = "flir_raw",
        .desc = "Raw FLIR Control Interface",
        .id   = 0,
    },
#endif /* MODS_RAW_FLIR */
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

#if defined(CONFIG_GPIO_CHIP_STM32)
  stm32_gpio_init();
#endif

#ifdef CONFIG_STM32_LPTIM1
  stm32_lptim1_on();
#endif

#ifdef CONFIG_STM32_SPI
  stm32_spiinitialize();
#endif

#if CONFIG_STM32_ADC_INIT
  stm32_adc_initialize();
#endif

#ifdef CONFIG_MODS_DIET
  mods_init();
#endif

#ifndef CONFIG_BOARD_INITTHREAD
# error "Must enable INITTHREAD"
#endif

#ifdef CONFIG_DEVICE_CORE
  device_table_register(&muc_device_table);

#ifdef CONFIG_FUSB302
  fusb302_register(GPIO_MODS_FUSB302_INT_N, GPIO_MODS_VBUS_PWR_EN);
#endif

#ifdef CONFIG_FUSB302_USB_EXT
  extern struct device_driver fusb302_usb_ext_driver;
  device_register_driver(&fusb302_usb_ext_driver);
#endif
#ifdef CONFIG_FUSB302_EXT_POWER
  extern struct device_driver fusb302_ext_power_driver;
  device_register_driver(&fusb302_ext_power_driver);
#endif
#ifdef CONFIG_MODS_RAW
  extern struct device_driver mods_raw_driver;
  device_register_driver(&mods_raw_driver);
#endif
#ifdef CONFIG_MODS_USB2
  extern struct device_driver usb2_driver;
  device_register_driver(&usb2_driver);
#endif
#ifdef CONFIG_MODS_RAW_STUB
  extern struct device_driver raw_stub_driver;
  device_register_driver(&raw_stub_driver);
#endif
#ifdef CONFIG_MODS_RAW_BLINKY
  extern struct device_driver mods_raw_blinky_driver;
  device_register_driver(&mods_raw_blinky_driver);
#endif
#ifdef CONFIG_MODS_RAW_TERMAPP
  extern struct device_driver mods_raw_termapp_driver;
  device_register_driver(&mods_raw_termapp_driver);
#endif
#ifdef CONFIG_MODS_RAW_TEMPERATURE
  extern struct device_driver mods_raw_temperature_driver;
  device_register_driver(&mods_raw_temperature_driver);
#endif
#ifdef CONFIG_GREYBUS_MODS_PTP_DEVICE
  extern struct device_driver mods_ptp_driver;
  device_register_driver(&mods_ptp_driver);
#endif
#ifdef CONFIG_CHARGER_DEVICE_BQ25896
  extern struct device_driver bq25896_charger_driver;
  device_register_driver(&bq25896_charger_driver);
#endif
#ifdef CONFIG_CHARGING_MODS_DONGLE
  extern struct device_driver dongle_ext_power_driver;
  device_register_driver(&dongle_ext_power_driver);
#endif
#ifdef CONFIG_GREYBUS_MODS_PTP_CHG_DEVICE_SWITCH
  extern struct device_driver switch_ptp_chg_driver;
  device_register_driver(&switch_ptp_chg_driver);
#endif
#ifdef CONFIG_MAX17050_DEVICE
  extern struct device_driver batt_driver;
  device_register_driver(&batt_driver);
#endif
#ifdef CONFIG_BATTERY_TEMP_DEVICE_MAX17050
  extern struct device_driver max17050_battery_temp_driver;
  device_register_driver(&max17050_battery_temp_driver);
#endif
#ifdef CONFIG_BATTERY_LEVEL_DEVICE_MAX17050
  extern struct device_driver max17050_battery_level_driver;
  device_register_driver(&max17050_battery_level_driver);
#endif
#ifdef CONFIG_BATTERY_VOLTAGE_DEVICE_MAX17050
  extern struct device_driver max17050_battery_voltage_driver;
  device_register_driver(&max17050_battery_voltage_driver);
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
#ifdef CONFIG_GREYBUS_SENSORS_EXT_LSM9DS1_ACCEL
  extern struct device_driver sensor_lsm9sd1_accel_driver;
  device_register_driver(&sensor_lsm9sd1_accel_driver);
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
#ifdef CONFIG_BACKLIGHT_DCS
   extern struct device_driver dcs_backlight_driver;
   device_register_driver(&dcs_backlight_driver);
#endif
#ifdef CONFIG_BACKLIGHT_ISL98611
   extern struct device_driver isl98611_backlight_driver;
   device_register_driver(&isl98611_backlight_driver);
#endif
#ifdef CONFIG_BACKLIGHT_LM27965
   extern struct device_driver lm27965_backlight_driver;
   device_register_driver(&lm27965_backlight_driver);
#endif
#if defined(CONFIG_MHB_CAMERA)
   extern struct device_driver cam_ext_mhb_driver;
   device_register_driver(&cam_ext_mhb_driver);
#endif
#if defined(CONFIG_CAMERA_IMX220)
    extern struct device_driver imx220_mhb_camera_driver;
    device_register_driver(&imx220_mhb_camera_driver);
#endif
#if defined(CONFIG_CAMERA_IMX230)
    extern struct device_driver imx230_mhb_camera_driver;
    device_register_driver(&imx230_mhb_camera_driver);
#endif
#if defined(CONFIG_CAMERA_OV5647_PI)
    extern struct device_driver ov5647_pi_mhb_camera_driver;
    device_register_driver(&ov5647_pi_mhb_camera_driver);
#endif
#if defined(CONFIG_CAMERA_IMX219_PI)
    extern struct device_driver imx219_pi_mhb_camera_driver;
    device_register_driver(&imx219_pi_mhb_camera_driver);
#endif
#ifdef CONFIG_MODS_AUDIO_TFA9890
  extern struct device_driver tfa9890_i2s_direct_driver;
  device_register_driver(&tfa9890_i2s_direct_driver);
  extern struct device_driver tfa9890_audio_dev_driver;
  device_register_driver(&tfa9890_audio_dev_driver);
#endif
#ifdef CONFIG_MODS_MHB_AUDIO_TFA9890
  extern struct device_driver tfa9890_audio_dev_driver;
  device_register_driver(&tfa9890_audio_dev_driver);
  extern struct device_driver mhb_i2s_driver;
  device_register_driver(&mhb_i2s_driver);
#endif
#ifdef CONFIG_MODS_RAW_FACTORY
   extern struct device_driver display_mux_driver;
   device_register_driver(&display_mux_driver);
   extern struct device_driver mods_raw_factory_driver;
   device_register_driver(&mods_raw_factory_driver);
#endif
#ifdef CONFIG_MODS_RAW_HSIC
   extern struct device_driver usb3813_driver;
   device_register_driver(&usb3813_driver);
   extern struct device_driver raw_hsic_usb_ext_driver;
   device_register_driver(&raw_hsic_usb_ext_driver);
   extern struct device_driver mhb_usbtun_driver;
   device_register_driver(&mhb_usbtun_driver);
   extern struct device_driver mods_raw_hsic_driver;
   device_register_driver(&mods_raw_hsic_driver);
#endif
#ifdef CONFIG_MODS_HID_EXAMPLE
   extern struct device_driver hid_game_driver;
   device_register_driver(&hid_game_driver);
#endif
#ifdef CONFIG_MODS_RAW_FLIR
   extern struct device_driver mods_flir_raw_driver;
   device_register_driver(&mods_flir_raw_driver);
#endif
#endif

#ifdef CONFIG_BATTERY_MAX17050
   struct i2c_dev_s *i2c = up_i2cinitialize(MAX17050_I2C_BUS);
   if (i2c) {
      g_battery = max17050_initialize(i2c, MAX17050_I2C_FREQ,
                  GPIO_MODS_CC_ALERT);
      if (!g_battery) {
         up_i2cuninitialize(i2c);
      }
   }
# if defined(CONFIG_BATTERY_STATE)
   /* Must be initialized after MAX17050 core driver has been initialized */
   battery_state_init();
# endif
#endif

#if !defined(CONFIG_GREYBUS_PTP_EXT_SUPPORTED)
   /* Set the power paths to be able to use USB VBUS as the system power and
    * to prevent applying voltage to VBUS pin on the Mod connector. Also,
    * prevent accepting power from the PCARD VBUS connection.
    */
   gpio_set_value(GPIO_MODS_CHG_VINA_EN, 1);
   gpio_set_value(GPIO_MODS_CHG_VINB_EN, 0);
#endif
}
#endif
