/*
 * Copyright (c) 2016 Motorola Mobility, LLC.
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

#ifndef __INCLUDE_NUTTX_POWER_BQ25896_H
#define __INCLUDE_NUTTX_POWER_BQ25896_H

#include <stdint.h>

/**
 * Device registers.
 */
#define BQ25896_REG00                       (0x00)
#define BQ25896_REG00_INLIM_SHIFT           (0)      /* Bits 0-5: Input Current Limit */
#define BQ25896_REG00_INLIM_MASK            (63 << BQ25896_REG00_INLIM_SHIFT)
#define BQ25896_REG00_ILIM_SHIFT            (6)      /* Bit 6: Enable ILIM Pin */
#define BQ25896_REG00_ILIM_MASK             (1 << BQ25896_REG00_ILIM_SHIFT)
#define BQ25896_REG00_ILIM_DIS              (0 << BQ25896_REG00_ILIM_SHIFT)
#define BQ25896_REG00_ILIM_EN               (1 << BQ25896_REG00_ILIM_SHIFT)

#define BQ25896_REG02                       (0x02)
#define BQ25896_REG02_ICO_SHIFT             (4)      /* Bit 4: Input Current Optimizer */
#define BQ25896_REG02_ICO_MASK              (1 << BQ25896_REG02_ICO_SHIFT)
#  define BQ25896_REG02_ICO_DIS             (0 << BQ25896_REG02_ICO_SHIFT)
#  define BQ25896_REG02_ICO_EN              (1 << BQ25896_REG02_ICO_SHIFT)

#define BQ25896_REG03                       (0x03)
#define BQ25896_REG03_MIN_VBAT_SHIFT        (0)      /* Bits 0: Min Battery voltage to exit boost mode */
#define BQ25896_REG03_MIN_VBAT_MASK         (1 << BQ25896_REG03_MIN_VBAT_SHIFT)
#  define BQ25896_REG03_MIN_VBAT_2900MV     (0 << BQ25896_REG03_MIN_VBAT_SHIFT)
#  define BQ25896_REG03_MIN_VBAT_2500MV     (1 << BQ25896_REG03_MIN_VBAT_SHIFT)
#define BQ25896_REG03_CONFIG_SHIFT          (4)      /* Bits 4-5: Config */
#define BQ25896_REG03_CONFIG_MASK           (3 << BQ25896_REG03_CONFIG_SHIFT)
#  define BQ25896_REG03_CONFIG_OFF          (0 << BQ25896_REG03_CONFIG_SHIFT)
#  define BQ25896_REG03_CONFIG_CHG          (1 << BQ25896_REG03_CONFIG_SHIFT)
#  define BQ25896_REG03_CONFIG_OTG          (2 << BQ25896_REG03_CONFIG_SHIFT)

#define BQ25896_REG04                       (0x04)
#define BQ25896_REG04_ICHG_SHIFT            (0)      /* Bits 0-6: Fast Charge Current Limit */
#define BQ25896_REG04_ICHG_MASK             (127 << BQ25896_REG04_ICHG_SHIFT)

#define BQ25896_REG05                       (0x05)
#define BQ25896_REG05_IPRECHG_SHIFT         (4)     /* Bits 4-7: Precharge current limit */
#define BQ25896_REG05_IPRECHG_MASK          (15 << BQ25896_REG05_IPRECHG_SHIFT)
#  define BQ25896_REG05_IPRECHG_64MA        (0  << BQ25896_REG05_IPRECHG_SHIFT)
#  define BQ25896_REG05_IPRECHG_128MA       (1  << BQ25896_REG05_IPRECHG_SHIFT)
#  define BQ25896_REG05_IPRECHG_192MA       (2  << BQ25896_REG05_IPRECHG_SHIFT)
#  define BQ25896_REG05_IPRECHG_256MA       (3  << BQ25896_REG05_IPRECHG_SHIFT)
#  define BQ25896_REG05_IPRECHG_320MA       (4  << BQ25896_REG05_IPRECHG_SHIFT)
#  define BQ25896_REG05_IPRECHG_384MA       (5  << BQ25896_REG05_IPRECHG_SHIFT)
#  define BQ25896_REG05_IPRECHG_448MA       (6  << BQ25896_REG05_IPRECHG_SHIFT)
#  define BQ25896_REG05_IPRECHG_512MA       (7  << BQ25896_REG05_IPRECHG_SHIFT)
#  define BQ25896_REG05_IPRECHG_576MA       (8  << BQ25896_REG05_IPRECHG_SHIFT)
#  define BQ25896_REG05_IPRECHG_640MA       (9  << BQ25896_REG05_IPRECHG_SHIFT)
#  define BQ25896_REG05_IPRECHG_704MA       (10 << BQ25896_REG05_IPRECHG_SHIFT)
#  define BQ25896_REG05_IPRECHG_768MA       (11 << BQ25896_REG05_IPRECHG_SHIFT)
#  define BQ25896_REG05_IPRECHG_832MA       (12 << BQ25896_REG05_IPRECHG_SHIFT)
#  define BQ25896_REG05_IPRECHG_896MA       (13 << BQ25896_REG05_IPRECHG_SHIFT)
#  define BQ25896_REG05_IPRECHG_960MA       (14 << BQ25896_REG05_IPRECHG_SHIFT)
#  define BQ25896_REG05_IPRECHG_1024MA      (15 << BQ25896_REG05_IPRECHG_SHIFT)

#define BQ25896_REG06                       (0x06)
#define BQ25896_REG06_BATLOWV_SHIFT         (1)      /* Bit 1: Battery Precharge to Fast Charge Threshold */
#define BQ25896_REG06_BATLOWV_MASK          (1 << BQ25896_REG06_BATLOWV_SHIFT)
#define BQ25896_REG06_BATLOWV_2800MV        (0 << BQ25896_REG06_BATLOWV_SHIFT)
#define BQ25896_REG06_BATLOWV_3000MV        (1 << BQ25896_REG06_BATLOWV_SHIFT)
#define BQ25896_REG06_VREG_SHIFT            (2)      /* Bits 2-7: Charge Voltage Limit */
#define BQ25896_REG06_VREG_MASK             (63 << BQ25896_REG06_VREG_SHIFT)

#define BQ25896_REG07                       (0x07)
#define BQ25896_REG07_WATCHDOG_SHIFT        (4)      /* Bits 4-5: I2C Watchdog Timer */
#define BQ25896_REG07_WATCHDOG_MASK         (3 << BQ25896_REG07_WATCHDOG_SHIFT)
#  define BQ25896_REG07_WATCHDOG_DIS        (0 << BQ25896_REG07_WATCHDOG_SHIFT)
#  define BQ25896_REG07_WATCHDOG_40S        (1 << BQ25896_REG07_WATCHDOG_SHIFT)
#  define BQ25896_REG07_WATCHDOG_80S        (2 << BQ25896_REG07_WATCHDOG_SHIFT)
#  define BQ25896_REG07_WATCHDOG_160S       (3 << BQ25896_REG07_WATCHDOG_SHIFT)
#define BQ25896_REG07_TERM_SHIFT            (7)      /* Bit 7: Charging Termination */
#define BQ25896_REG07_TERM_MASK             (1 << BQ25896_REG07_TERM_SHIFT)
#define BQ25896_REG07_TERM_DIS              (0 << BQ25896_REG07_TERM_SHIFT)
#define BQ25896_REG07_TERM_EN               (1 << BQ25896_REG07_TERM_SHIFT)

#define BQ25896_REG09                       (0x09)
#define BQ25896_REG09_BATFET_SHIFT          (5)      /* Bit 5: Force BATFET off to enable ship mode */
#define BQ25896_REG09_BATFET_MASK           (1 << BQ25896_REG09_BATFET_SHIFT)
#define BQ25896_REG09_BATFET_ON             (0 << BQ25896_REG09_BATFET_SHIFT)
#define BQ25896_REG09_BATFET_OFF            (1 << BQ25896_REG09_BATFET_SHIFT)

#define BQ25896_REG0A                       (0x0A)
#define BQ25896_REG0A_BOOST_LIM_SHIFT       (0)      /* Bits 0-2: Boost Mode Current Limit */
#define BQ25896_REG0A_BOOST_LIM_MASK        (7 << BQ25896_REG0A_BOOST_LIM_SHIFT)
#  define BQ25896_REG0A_BOOST_LIM_500MA     (0 << BQ25896_REG0A_BOOST_LIM_SHIFT)
#  define BQ25896_REG0A_BOOST_LIM_750MA     (1 << BQ25896_REG0A_BOOST_LIM_SHIFT)
#  define BQ25896_REG0A_BOOST_LIM_1200MA    (2 << BQ25896_REG0A_BOOST_LIM_SHIFT)
#  define BQ25896_REG0A_BOOST_LIM_1400MA    (3 << BQ25896_REG0A_BOOST_LIM_SHIFT)
#  define BQ25896_REG0A_BOOST_LIM_1650MA    (4 << BQ25896_REG0A_BOOST_LIM_SHIFT)
#  define BQ25896_REG0A_BOOST_LIM_1875MA    (5 << BQ25896_REG0A_BOOST_LIM_SHIFT)
#  define BQ25896_REG0A_BOOST_LIM_2150MA    (6 << BQ25896_REG0A_BOOST_LIM_SHIFT)

#define BQ25896_REG0B                       (0x0B)
#define BQ25896_REG0B_PG                    (1 << 2) /* Bit 2: Power Good */

#define BQ25896_REG0C                       (0x0C)
#define BQ25896_REG0C_BOOST_FAULT           (1 << 6) /* Bit 6: Boost Mode Fault */

#define BQ25896_REG0D                       (0x0D)
#define BQ25896_REG0D_VINDPM_SHIFT          (0)      /* Bits 0-6: Absolute input voltage limit */
#define BQ25896_REG0D_VINDPM_MASK           (127 << BQ25896_REG0D_VINDPM_SHIFT)
#define BQ25896_REG0D_FORCE_VINDPM_SHIFT    (7)      /* Bit 7: Input voltage limit settng method */
#define BQ25896_REG0D_FORCE_VINDPM_MASK     (1 << BQ25896_REG0D_FORCE_VINDPM_SHIFT)
#  define BQ25896_REG0D_VINDPM_RELATIVE     (0 << BQ25896_REG0D_FORCE_VINDPM_SHIFT)
#  define BQ25896_REG0D_VINDPM_ABSOLUTE     (1 << BQ25896_REG0D_FORCE_VINDPM_SHIFT)

/**
 * @brief Initialize driver for register access and interrupt processing.
 *
 * @param int_n INT pin gpio. Use negative number if pin is not connected.
 * @param pg_n PG pin gpio. Use negative number if pin is not connected.
 * @return 0 on success, negative errno on error.
 */
int bq25896_driver_init(int16_t int_n, int16_t pg_n);

/**
 * @brief Read device register.
 *
 * @param reg Address.
 * @return Register value on success, negative errno on error.
 */
int bq25896_reg_read(uint8_t reg);

/**
 * @brief Write new value to device register.
 *
 * @param reg Address.
 * @param val Value.
 * @return 0 on success, negative errno on error.
 */
int bq25896_reg_write(uint8_t reg, uint8_t val);

/**
 * @brief Perform masked write to device register.
 *
 * @param reg Address.
 * @param mask Bit mask.
 * @param set Value.
 * @return 0 on success, negative errno on error.
 */
int bq25896_reg_modify(uint8_t reg, uint8_t mask, uint8_t set);

/**
 * Register value with a mask.
 */
struct bq25896_reg {
    /**Address */
    uint8_t reg;
    /** Bit mask */
    uint8_t mask;
    /** Value. */
    uint8_t set;
};

/**
 * Array with registers.
 */
struct bq25896_config {
    /** Array. */
    struct bq25896_reg *reg;
    /** Size of the array. */
    int reg_num;
};

/**
 * @brief Modify a set of device registers.
 *
 * @param cfg Registers and their values.
 * @return 0 on success, negative errno on error.
 */
int bq25896_configure(const struct bq25896_config *cfg);

/**
 * Notification events.
 */
enum bq25896_event {
    /** Good power input source. */
    POWER_GOOD,
    /** VBUS over-current or over-voltage in boost mode. */
    BOOST_FAULT,
    /** No good power input source. */
    NO_POWER_GOOD,
};

/**
 * @brief Event notification callback.
 *
 * @param event Notification event.
 * @param arg value provided during callback registration.
 */
typedef void (*bq25896_callback)(enum bq25896_event event, void *arg);

/**
 * @brief Register for event notifications.
 *
 * @param cb function to run when events happen.
 * @param arg value that will be returned in callback.
 * @return 0 on success, negative errno on error.
 */
int bq25896_register_callback(bq25896_callback cb, void *arg);

#endif // __INCLUDE_NUTTX_POWER_BQ25896_H
