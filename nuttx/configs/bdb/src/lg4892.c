/**
 * Copyright (c) 2014 Google, Inc.
 * Google Confidential/Restricted
 * @author Patrick Titiano
 * @brief LG4892 LCD Panel Driver
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <debug.h>

#include <arch/board/lg4892.h>
#include <nuttx/gpio/tca6408.h>

#define TCA6408_U72             0x20
#define TCA6408_U72_RST_GPIO    0x04

#define APB2_I2C0               0

#define APB2_LCD_VDD_EN         0x00
#define APB2_LCD_VDDI_EN        0x01
#define APB2_LCD_ENP            0x02
#define APB2_DSI_LCD_TE0        0x03
#define APB2_LCD_RST            0x04
#define APB2_TP_INT_N           0x05
#define APB2_TP_RST             0x06

/* usleep() resolution is 1us. Delays rounded to higher 1us */
/* Power-up Timings */
#define LG4892_T_VDD_TO_VDDIO               1 /* datasheet: 0 */
#define LG4892_T_VDDIO_TO_DDVDH             2000 /* datasheet: 1ms */
#define LG4892_T_DDVDH_TO_DDVDN             2000 /* datasheet: 1ms */
#define LG4892_T_DDVDN_TO_RESET             12000 /* datasheet: 10ms */
#define LG4892_T_RESET_TO_1ST_COMMAND       12000 /* datasheet: 10ms */
#define LG4892_T_SLEEP_OUT_TO_ON_COMMAND    12000 /* datasheet: 10ms */
#define LG4892_T_TOUCH_CALIBRATION          100000 /* datasheet: 90ms */
/* Power-down Timings */
#define LG4892_T_RESET_TO_DDVDN             2000 /* datasheet: > 1ms */
#define LG4892_T_DDVDN_TO_DDVDH             2000 /* datasheet: > 1ms */
#define LG4892_T_DDVDH_TO_VDDIO             2000 /* datasheet: > 1ms */
#define LG4892_T_VDDIO_TO_VDD               2000 /* datasheet: > 1ms */

/* disable the verbose debug output */
#undef lldbg
#define lldbg(x...)

/* Initialize GPIO expander that handles LCD panel dedicated IOs. */
int lg4892_gpio_init(void)
{
    lldbg("reset U72 (controlling LCD power supply).\n");
    tca6408_reset(TCA6408_U72_RST_GPIO, 0);

    lldbg("configure default outputs as 0.\n");
    tca6408_set_default_outputs(APB2_I2C0, TCA6408_U72, 0);

    lldbg("configure pins as output.\n");
    tca6408_set_direction_out(APB2_I2C0, TCA6408_U72, APB2_LCD_VDD_EN);
    tca6408_set_direction_out(APB2_I2C0, TCA6408_U72, APB2_LCD_VDDI_EN);
    tca6408_set_direction_out(APB2_I2C0, TCA6408_U72, APB2_LCD_ENP);
    tca6408_set_direction_out(APB2_I2C0, TCA6408_U72, APB2_LCD_RST);

    return 0;
}

/* Turn LCD Panel ON/OFF. Assume prior GPIO expander initialization. */
int lg4892_enable(bool enable)
{
    if (enable) {
        lldbg("enable LCD_VDD_EN\n");
        tca6408_set(APB2_I2C0, TCA6408_U72, APB2_LCD_VDD_EN, 1);
        usleep(LG4892_T_VDD_TO_VDDIO);

        lldbg("enable LCD_VDDI_EN\n");
        tca6408_set(APB2_I2C0, TCA6408_U72, APB2_LCD_VDDI_EN, 1);
        usleep(LG4892_T_VDDIO_TO_DDVDH + LG4892_T_DDVDH_TO_DDVDN);

        lldbg("enable LCD_ENP\n");
        tca6408_set(APB2_I2C0, TCA6408_U72, APB2_LCD_ENP, 1);
        usleep(LG4892_T_DDVDN_TO_RESET);

        lldbg("release LCD_RST\n");
        tca6408_set(APB2_I2C0, TCA6408_U72, APB2_LCD_RST, 1);
        usleep(LG4892_T_RESET_TO_1ST_COMMAND);
        lldbg("LCD Panel powered ON and out of reset.\n");
    } else {
        lldbg("assert LCD_RST\n");
        tca6408_set(APB2_I2C0, TCA6408_U72, APB2_LCD_RST, 0);
        usleep(LG4892_T_RESET_TO_DDVDN);

        lldbg("disable LCD_ENP\n");
        tca6408_set(APB2_I2C0, TCA6408_U72, APB2_LCD_ENP, 0);
        usleep(LG4892_T_DDVDN_TO_DDVDH + LG4892_T_DDVDH_TO_VDDIO);

        lldbg("disable LCD_VDDI_EN\n");
        tca6408_set(APB2_I2C0, TCA6408_U72, APB2_LCD_VDDI_EN, 0);
        usleep(LG4892_T_VDDIO_TO_VDD);

        lldbg("disable LCD_VDD_EN\n");
        tca6408_set(APB2_I2C0, TCA6408_U72, APB2_LCD_VDD_EN, 0);
        lldbg("LCD Panel powered OFF.\n");
    }

    return 0;
}
