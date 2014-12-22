/**
 * Copyright (c) 2014 Google, Inc.
 * Google Confidential/Restricted
 * @author Patrick Titiano
 * @brief TCA6408 GPIO Expander Driver Test Application
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/gpio/tca6408.h>

#define TCA6408_U72             0x20
#define TCA6408_U72_RST_GPIO    0x04

#define APB2_LCD_VDD_EN         0x00
#define APB2_LCD_VDDI_EN        0x01
#define APB2_LCD_ENP            0x02
#define APB2_DSI_LCD_TE0        0x03
#define APB2_LCD_RST            0x04
#define APB2_TP_INT_N           0x05
#define APB2_TP_RST             0x06
#define APB2_ETP172             0x07

#define APB2_I2C0               0

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int tca_main(int argc, char *argv[])
#endif
{
    int ret = 0, pol1, pol2;
    int level5, prev_level5, level5b;
    int level7, prev_level7;

    tca6408_reset(TCA6408_U72_RST_GPIO, 0);
    printf("TCA6408 U72 got reset.\n\n");

    /* Test output functions */
    ret = tca6408_set_direction_out(APB2_I2C0, TCA6408_U72, APB2_LCD_VDD_EN);
    ret += tca6408_set_direction_out(APB2_I2C0, TCA6408_U72, APB2_LCD_VDDI_EN);
    ret += tca6408_set_direction_out(APB2_I2C0, TCA6408_U72, APB2_LCD_ENP);
    ret += tca6408_set_direction_out(APB2_I2C0, TCA6408_U72, APB2_LCD_RST);
    if (ret != 0) {
        printf("Failed to configure U72 outputs! (%d)\n", ret);
        return ret;
    }
    printf("TCA6408 U72 [0,1,2,4] pins set as output.\n");

    ret = tca6408_set(APB2_I2C0, TCA6408_U72, APB2_LCD_VDD_EN, 0);
    ret += tca6408_set(APB2_I2C0, TCA6408_U72, APB2_LCD_VDDI_EN, 1);
    ret += tca6408_set(APB2_I2C0, TCA6408_U72, APB2_LCD_ENP, 1);
    ret += tca6408_set(APB2_I2C0, TCA6408_U72, APB2_LCD_RST, 0);
    if (ret != 0) {
        printf("Failed to set U72 [0,1,2,4] pins! (%d)\n", ret);
        return ret;
    }
    printf("U72 [0,1,2,4] output pins set.\n");
    printf("U72 pin #0 (APB2_LCD_VDD_EN, ETP133) should now be 0.\n");
    printf("U72 pin #1 (APB2_LCD_VDDI_EN, ETP137) should now be 1.\n");
    printf("U72 pin #2 (APB2_LCD_ENP, ETP186) should now be 1.\n");
    printf("U72 pin #4 (APB2_LCD_RST, ETP57) should now be 0.\n");
    printf("Please check with multimeter/scope.\n\n");

    /* Test input functions */
    ret = tca6408_set_direction_in(APB2_I2C0, TCA6408_U72, APB2_TP_INT_N);
    if (ret != 0) {
        printf("Failed to configure U72 input #5! (%d)\n", ret);
        return ret;
    }
    printf("TCA6408 U72 input #5 (APB2_TP_INT_N, ETP35) set as input.\n");
    ret = tca6408_set_direction_in(APB2_I2C0, TCA6408_U72, 7);
    if (ret != 0) {
        printf("Failed to configure U72 input #7! (%d)\n", ret);
        return ret;
    }
    printf("TCA6408 U72 input #7 (ETP172) set as input.\n");

    /* Test input polarity inversion */
    pol1 = tca6408_get_polarity_inverted(APB2_I2C0, TCA6408_U72, APB2_TP_INT_N);
    printf("Current U72 input #5 (APB2_TP_INT_N, ETP35) polarity: %d\n", pol1);
    level5 = tca6408_get(APB2_I2C0, TCA6408_U72, APB2_TP_INT_N);
    printf("U72 input #5 (ETP35) = %d\n", level5);
    ret = tca6408_set_polarity_inverted(APB2_I2C0, TCA6408_U72, APB2_TP_INT_N,
                                        !pol1);
    pol2 = tca6408_get_polarity_inverted(APB2_I2C0, TCA6408_U72, APB2_TP_INT_N);
    printf("New U72 input #5 (APB2_TP_INT_N, ETP35) polarity: %d\n", pol2);
    if (pol2 == pol1) {
        printf("Failed to invert polarity!\n");
        return ret;
    }
    printf("U72 input #5 polarity successfully inverted.\n");
    level5b = tca6408_get(APB2_I2C0, TCA6408_U72, APB2_TP_INT_N);
    printf("U72 input #5 (ETP35) = %d\n", level5b);
    if (level5b == level5) {
        printf("U72 input #5 (ETP35) should have toggled!\n");
        return ret;
    }
    printf("U72 input #5 (ETP35) toggled as expected.\n\n");
    tca6408_set_polarity_inverted(APB2_I2C0, TCA6408_U72, APB2_TP_INT_N, 0);

    printf("This test app should now reflect input level changes.\n");
    printf("Please set input level via ETP35 or ETP172...\n");
    prev_level5 = 2;
    prev_level7 = 2;
    while (1) {
        level5 = tca6408_get(APB2_I2C0, TCA6408_U72, APB2_TP_INT_N);
        level7 = tca6408_get(APB2_I2C0, TCA6408_U72, APB2_ETP172);
        if (level5 >= 0) {
            if (level5 != prev_level5) {
                printf("U72 input #5 (ETP35) = %d\n", level5);
                prev_level5 = level5;
            }
        } else {
            printf("Failed to get U72 input #5 level! (%d)\n", level5);
        }
        if (level7 >= 0) {
            if (level7 != prev_level7) {
                printf("U72 input #7 (ETP172) = %d\n", level7);
                prev_level7 = level7;
            }
        } else {
            printf("Failed to get U72 input #7 level! (%d)\n", level7);
        }
        usleep(200 * 1000); /* 200ms */
    }

    return 0;
}
