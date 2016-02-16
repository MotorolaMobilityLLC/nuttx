/*
 * Copyright (c) 2015 Motorola Mobility, LLC.
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

#include <errno.h>
#include <debug.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <arch/chip/cdsi.h>
#include <arch/chip/cdsi_config.h>
#include <arch/chip/cdsi_dbg.h>
#include <arch/chip/cdsi_display.h>
#include <arch/chip/cdsi_reg_def.h>
#include <arch/chip/gpio.h>

#include <nuttx/greybus/debug.h>
#include <nuttx/arch.h>
#include <nuttx/gpio.h>
#include <nuttx/i2c.h>
#include <nuttx/device.h>
#include <nuttx/device_display.h>

#include <apps/ice/cdsi.h>

//#define CONFIG_LYNX_DISPLAY_TE (1)
//#define CONFIG_LYNX_DISPLAY_PANEL_ID (1)
//#define CONFIG_LYNX_DISPLAY_POWER_MODE (1)

#define GPIO_APBE_DISP_PWR1_EN (3) /* backlight EN-P */
#define GPIO_APBE_DISP_PWR2_EN (4) /* backlight EN-N */
#define GPIO_APBE_DISP_PWR3_EN (5) /* I2C switch */

#define GPIO_APBE_DISP_RST_N   (7) /* backlight reset */
#define GPIO_APBE_DISP_RST2_N  (9) /* touch reset */

#define GPIO_APBE_DISP_TE      (8) /* input from panel */

#define LYNX_DISPLAY_POWER_DELAY_US (100000)

#define LYNX_BACKLIGHT_I2C_BUS (0)
#define LYNX_BACKLIGHT_I2C_ADDRESS (0x29)
#define LYNX_BACKLIGHT_RESET_DELAY_US (5000)

#define LYNX_SUPPLIER_ID (0x6001)
#define LYNX_ID0 (0x05)
#define LYNX_ID1 (0x03)
#define LYNX_ID2 (0x01)

/* ISL98611 Backlight controller */
#define ISL98611_FAULT_STATUS (0x01)
#define ISL98611_ENABLE (0x02)
#define ISL98611_VP_VN_HEADROOM (0x03)
#define ISL98611_VP_VOLTAGE (0x04)
#define ISL98611_VN_VOLTAGE (0x05)
#define ISL98611_VBST_CTRL (0x06)
/* 0x07 - 0x0f reserved */
#define ISL98611_BRIGHTNESS_CTRL_MSB (0x10)
#define ISL98611_BRIGHTNESS_CTRL_LSB (0x11)
#define ISL98611_LED_CURRENT (0x12)
#define ISL98611_DIMMING_CTRL (0x13)
#define ISL98611_LED_PWM_CTRL (0x14)
/* 0x15 reserved */
#define ISL98611_VLED_FREQ (0x16)
#define ISL98611_LED_CONFIG (0x17)

/* FIXME: Kinzie panel EID 720x2560 */
const static uint8_t LYNX_EDID_CONFIG[] = {
    0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00,
    0x22, 0xf0, 0x4b, 0x28, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x15, 0x01, 0x03, 0x80, 0x34, 0x20, 0x78,
    0xee, 0x9e, 0xc5, 0xa6, 0x56, 0x4b, 0x9a, 0x25,
    0x13, 0x50, 0x54, 0x00, 0x00, 0x00, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x86, 0x35,
    0xd0, 0xa0, 0x20, 0x00, 0x23, 0xa0, 0x30, 0x20,
    0x36, 0x00, 0x06, 0x44, 0x21, 0x00, 0x00, 0x1a,
    0x00, 0x00, 0x00, 0xfd, 0x00, 0x32, 0x3f, 0x18,
    0x4c, 0x11, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20,
    0x20, 0x20, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x4c,
    0x41, 0x32, 0x34, 0x30, 0x35, 0x0a, 0x20, 0x20,
    0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xff,
    0x00, 0x43, 0x4e, 0x34, 0x31, 0x30, 0x31, 0x30,
    0x30, 0x4c, 0x54, 0x0a, 0x20, 0x20, 0x00, 0x75,
};

const static struct cdsi_config LYNX_DISPLAY_CONFIG = {
    /* Common */
    .mode = TSB_CDSI_MODE_DSI,
    .tx_num_lanes = 4,
    .rx_num_lanes = 4,
    .tx_mbits_per_lane = 816*1000*1000,
    .rx_mbits_per_lane = 816*1000*1000,
    /* RX only */
    .hs_rx_timeout = 0xffffffff,
    /* TX only */
    .pll_frs = 0,
    .pll_prd = 1,
    .pll_fbd = 84,
    .framerate = 60,
    .width = 1080,
    .height = 1920,
    .bpp = 24,
    .bta_enabled = 1,
    .continuous_clock = 0,
    .blank_packet_enabled = 0,
    .video_mode = 0,
    .color_bar_enabled = 0,
    /* CSI only */
    /* DSI only */
    .vss_control_payload = 0x0000,
    /* Video Mode only */
    /* Command Mode only */
};

const static struct dsi_cmd LYNX_DISPLAY_COMMANDS[] = {
    {CTYPE_LP_SHORT, DT_DCS_WRITE0, .u = { .sp = { 0x0011 } } }, /* exit_sleep_mode */
#if CONFIG_LYNX_DISPLAY_TE
    {CTYPE_LP_SHORT, DT_DCS_WRITE1, .u = { .sp = { 0x0035 } } }, /* set_tear_on */
#else
    {CTYPE_LP_SHORT, DT_DCS_WRITE1, .u = { .sp = { 0x0034 } } }, /* set_tear_off */
#endif
    {CTYPE_LP_SHORT, DT_DCS_WRITE0, .u = { .sp = { 0x0029 } } }, /* set_display_on */
};

static struct lynx_display
{
    struct device *display_device;
    struct cdsi_dev *cdsi_dev;
    display_notification_cb callback;
    uint8_t state;
    uint8_t brightness;
#if CONFIG_LYNX_DISPLAY_POWER_MODE
    uint8_t power_mode;
#endif
#if CONFIG_LYNX_DISPLAY_TE
    uint32_t te_count;
#endif
} g_display;

#if CONFIG_LYNX_DISPLAY_PANEL_ID
static int lynx_display_read_panel_id(void)
{
    uint16_t supplier_id = 0;
    uint8_t id0 = 0;
    uint8_t id1 = 0;
    uint8_t id2 = 0;

    dsi_read_panel_info(g_display.cdsi_dev, &supplier_id, &id0, &id1, &id2);
    if (supplier_id != LYNX_SUPPLIER_ID)
      {
        dbg("WARNING: Supplier ID does not match: expected=0x%04x, received=0x%04x\n", LYNX_SUPPLIER_ID, supplier_id);
      }

    if (id0 != LYNX_ID0)
      {
        dbg("WARNING: ID0 does not match: expected=0x%02x, received=0x%02x\n", LYNX_ID0, id0);
      }

    if (id1 != LYNX_ID1)
      {
        dbg("WARNING: ID1 does not match: expected=0x%02x, received=0x%02x\n", LYNX_ID1, id1);
      }

    if (id2 != LYNX_ID2)
      {
        dbg("WARNING: ID2 does not match: expected=0x%02x, received=0x%02x\n", LYNX_ID2, id2);
      }

    return 0;
}
#endif

#if CONFIG_LYNX_DISPLAY_POWER_MODE
static void lynx_display_read_power_mode(void)
{
    dsi_read_power_mode(g_display.cdsi_dev, &g_display.power_mode);
    dbg("power mode=0x%02x\n", g_display.power_mode);
}
#endif

static void lynx_dsi_init(struct cdsi_dev *dev)
{
    cdsi_initialize_tx(dev, &LYNX_DISPLAY_CONFIG);
    cdsi_tx_stop(dev);

#if CONFIG_LYNX_DISPLAY_PANEL_ID
    lynx_display_read_panel_id();
#endif

#if CONFIG_LYNX_DISPLAY_POWER_MODE
    lynx_display_read_power_mode();
#endif

    dsi_write_cmds(dev, LYNX_DISPLAY_COMMANDS, ARRAY_SIZE(LYNX_DISPLAY_COMMANDS));
    cdsi_tx_start(dev);
}

const static struct display_panel lynx_panel = {
    .cdsi_panel_init = lynx_dsi_init,
};

static int lynx_display_notification(enum display_notification_event event)
{
    if (g_display.callback)
      {
        g_display.callback(g_display.display_device, event);
      }

    return 0;
}

static int lynx_display_get_config_size(struct device *dev, uint32_t *size)
{
    *size = sizeof(LYNX_EDID_CONFIG);
    return 0;
}

static int lynx_display_get_config(struct device *dev, uint8_t *display_type,
    uint8_t *config_type, uint32_t *size, uint8_t **config)
{
    *display_type = DISPLAY_TYPE_DSI;
    *config_type = DISPLAY_CONFIG_TYPE_EDID_1P3;
    *size = sizeof(LYNX_EDID_CONFIG);
    *config = (uint8_t *)LYNX_EDID_CONFIG;

    return 0;
}

static int lynx_display_set_config(struct device *dev, uint8_t index)
{
    if (index != 0)
      {
        dbg("ERROR: Set invalid configuration: index=%d\n", index);
        return -EINVAL;
      }

    return 0;
}

static int lynx_display_get_state(struct device *dev, uint8_t *state)
{
    *state = g_display.state;
    return 0;
}

static void lynx_display_power_on(void)
{
    gpio_direction_out(GPIO_APBE_DISP_PWR1_EN, 1);
    gpio_direction_out(GPIO_APBE_DISP_PWR2_EN, 1);
    gpio_direction_out(GPIO_APBE_DISP_PWR3_EN, 1);
    usleep(LYNX_DISPLAY_POWER_DELAY_US);
    gpio_direction_out(GPIO_APBE_DISP_RST_N, 1);
    gpio_direction_out(GPIO_APBE_DISP_RST2_N, 1);
}

static void lynx_display_power_off(void)
{
    gpio_direction_out(GPIO_APBE_DISP_PWR1_EN, 0);
    gpio_direction_out(GPIO_APBE_DISP_PWR2_EN, 0);
    gpio_direction_out(GPIO_APBE_DISP_PWR3_EN, 0);
    gpio_direction_out(GPIO_APBE_DISP_RST_N, 0);
    gpio_direction_out(GPIO_APBE_DISP_RST2_N, 0);
}

static int lynx_display_set_state_on(void)
{
    int result;

    lynx_display_power_on();

    g_display.cdsi_dev = dsi_initialize((struct display_panel *)&lynx_panel, TSB_CDSI0, TSB_CDSI_TX);
    if (!g_display.cdsi_dev)
      {
        dbg("ERROR: Failed to initialize DSI\n");
        result = -EIO;
        goto pwroff;
      }

    result = cdsi_apba_display_rx_start(&LYNX_DISPLAY_CONFIG);
    if (result)
      {
        dbg("ERROR: Failed to start APBA DSI RX\n");
        goto uninit;
      }

    return result;

uninit:
    dsi_uninitialize(g_display.cdsi_dev);
    g_display.cdsi_dev = NULL;
pwroff:
    lynx_display_power_off();

    return result;
}

static int lynx_display_set_state_off(void)
{
    int result;

    result = cdsi_apba_display_rx_stop();
    if (result)
      {
        dbg("ERROR: Failed to stop APBA DSI RX\n");
      }

    cdsi_tx_stop(g_display.cdsi_dev);

    dsi_uninitialize(g_display.cdsi_dev);
    g_display.cdsi_dev = NULL;

    lynx_display_power_off();

    return 0;
}

static int lynx_display_set_state(struct device *dev, uint8_t state)
{
    int result;

    switch (state)
      {
        case DISPLAY_STATE_OFF:
          {
            result = lynx_display_set_state_off();
            if (!result)
              {
                g_display.state = state;
              }
            break;
          }
        case DISPLAY_STATE_ON:
          {
            result = lynx_display_set_state_on();
            if (!result)
              {
                g_display.state = state;
              }
            break;
          }
        default:
          {
            dbg("ERROR: Invalid display state\n");
            result = -EINVAL;
            break;
          }
    }

    return result;
}

static int lynx_display_i2c_write(struct i2c_dev_s *dev, uint16_t addr,
        uint8_t regaddr, uint8_t value)
{
    int result;
    struct i2c_msg_s msg;
    uint8_t wbuf[] = { regaddr, value };

    msg.addr   = addr;
    msg.flags  = 0;
    msg.buffer = wbuf;
    msg.length = sizeof(wbuf)/sizeof(wbuf[0]);

    result = I2C_TRANSFER(dev, &msg, 1);
    return result;
}

static int lynx_display_set_backlight_brightness(struct device *dev, uint8_t brightness)
{
    dbg("brightness=%d\n", brightness);

    const int bus = LYNX_BACKLIGHT_I2C_BUS;
    struct i2c_dev_s *i2c_dev = up_i2cinitialize(bus);
    if (!i2c_dev) {
        dbg("ERROR: Failed to get bus %d\n", bus);
        return -EIO;
    }

    lynx_display_i2c_write(i2c_dev, LYNX_BACKLIGHT_I2C_ADDRESS, ISL98611_ENABLE, 0x3f);
    usleep(LYNX_BACKLIGHT_RESET_DELAY_US);

    lynx_display_i2c_write(i2c_dev, LYNX_BACKLIGHT_I2C_ADDRESS, ISL98611_DIMMING_CTRL, 0x00);
    lynx_display_i2c_write(i2c_dev, LYNX_BACKLIGHT_I2C_ADDRESS, ISL98611_LED_PWM_CTRL, 0x7d);
    lynx_display_i2c_write(i2c_dev, LYNX_BACKLIGHT_I2C_ADDRESS, ISL98611_VP_VOLTAGE, 0x14);
    lynx_display_i2c_write(i2c_dev, LYNX_BACKLIGHT_I2C_ADDRESS, ISL98611_VN_VOLTAGE, 0x14);
    lynx_display_i2c_write(i2c_dev, LYNX_BACKLIGHT_I2C_ADDRESS, ISL98611_LED_CURRENT, 0xbf);
    lynx_display_i2c_write(i2c_dev, LYNX_BACKLIGHT_I2C_ADDRESS, 0x15, 0x86);

    lynx_display_i2c_write(i2c_dev, LYNX_BACKLIGHT_I2C_ADDRESS, ISL98611_BRIGHTNESS_CTRL_LSB, 0x00);
    lynx_display_i2c_write(i2c_dev, LYNX_BACKLIGHT_I2C_ADDRESS, ISL98611_BRIGHTNESS_CTRL_MSB, brightness);

    (void)up_i2cuninitialize(i2c_dev);

    g_display.brightness = brightness;
    return 0;
}

static int lynx_display_register_callback(struct device *dev,
        display_notification_cb callback)
{
    if (!callback)
      {
        return -EINVAL;
      }

    g_display.callback = callback;
    return 0;
}

static int lynx_display_unregister_callback(struct device *dev)
{
    g_display.callback = NULL;
    return 0;
}

#if CONFIG_LYNX_DISPLAY_TE
static int lynx_display_te_irq(int irq, FAR void *context)
{
#if 0
    g_display.te_count++;
    if ((g_display.te_count % 60) == 0)
      {
        up_lowputc('#');
      }
#endif

    gpio_clear_interrupt(irq);
    return 0;
}
#endif

static int lynx_display_config_gpios(void)
{
    gpio_direction_out(GPIO_APBE_DISP_PWR1_EN, 0);
    gpio_direction_out(GPIO_APBE_DISP_PWR2_EN, 0);
    gpio_direction_out(GPIO_APBE_DISP_PWR3_EN, 0);

    gpio_direction_out(GPIO_APBE_DISP_RST_N, 0);
    gpio_direction_out(GPIO_APBE_DISP_RST2_N, 0);

#if CONFIG_LYNX_DISPLAY_TE
    gpio_direction_in(GPIO_APBE_DISP_TE);
    set_gpio_triggering(GPIO_APBE_DISP_TE, IRQ_TYPE_EDGE_RISING);
    gpio_irqattach(GPIO_APBE_DISP_TE, lynx_display_te_irq);
    gpio_clear_interrupt(GPIO_APBE_DISP_TE);
    gpio_unmask_irq(GPIO_APBE_DISP_TE);
#endif

    return 0;
}

static int lynx_display_probe(struct device *dev)
{
    int result = lynx_display_config_gpios();
    if (result)
      {
        dbg("ERROR: Failed to configure GPIOS\n");
        return -EINVAL;
      }

    memset(&g_display, 0, sizeof(g_display));
    g_display.display_device = dev;
    g_display.state = DISPLAY_STATE_OFF;

    return 0;
}

static struct device_display_type_ops lynx_display_ops = {
    .get_config_size = lynx_display_get_config_size,
    .get_config = lynx_display_get_config,
    .set_config = lynx_display_set_config,
    .get_state = lynx_display_get_state,
    .set_state = lynx_display_set_state,
    .register_callback = lynx_display_register_callback,
    .unregister_callback = lynx_display_unregister_callback,
};

const static struct device_driver_ops lynx_display_driver_ops = {
    .probe = lynx_display_probe,
    .type_ops = &lynx_display_ops,
};

const struct device_driver lynx_display_driver = {
    .type = DEVICE_TYPE_DISPLAY_HW,
    .name = "lynx_display",
    .desc = "Lynx Display Driver",
    .ops = (struct device_driver_ops *)&lynx_display_driver_ops,
};

#if CONFIG_NSH_CONSOLE
void lynx_display_main(int argc, char **argv)
{
    const char *op = argv[1];
    printf("command: %s\n", op);

    if (!strcmp(op, "off")) {
        lynx_display_power_off();
    } else if (!strcmp(op, "on")) {
        lynx_display_power_on();
    } else if (!strcmp(op, "start")) {
        lynx_display_set_state_on();
    } else if (!strcmp(op, "stop")) {
        lynx_display_set_state_off();
    } else if (!strcmp(op, "level")) {
        uint8_t brightness = argc > 2 ? strtoul(argv[2], NULL, 10) : 255;
        lynx_display_set_backlight_brightness(g_display.display_device, brightness);
    }
}
#endif
