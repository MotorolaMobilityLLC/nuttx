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

/*
 * @file    configs/ara/svc/src/tca6424.c
 * @brief   TCA6424 GPIO Expander Driver
 * @author  Patrick Titiano
 */

#define DBG_COMP DBG_POWER

#include <nuttx/config.h>
#include <errno.h>
#include <unistd.h>
#include <tca6424.h>
#include <stdio.h>
#include <up_debug.h>


#define TCA6424_INPUT0_REG          0x00
#define TCA6424_INPUT1_REG          0x01
#define TCA6424_INPUT2_REG          0x02
#define TCA6424_OUTPUT0_REG         0x04
#define TCA6424_OUTPUT1_REG         0x05
#define TCA6424_OUTPUT2_REG         0x06
#define TCA6424_POLARITY0_REG       0x08
#define TCA6424_POLARITY1_REG       0x09
#define TCA6424_POLARITY2_REG       0x0A
#define TCA6424_CONFIG0_REG         0x0C
#define TCA6424_CONFIG1_REG         0x0C
#define TCA6424_CONFIG2_REG         0x0C

#define TCA6424_MAX_ADDR_SHIFT      2

/**
 * @brief           Return the register address shift for a given IO.
 *
 *                  TCA6424 has 3 8-bit registers to handle 24 bit I/Os.
 *                  Use this function to retrieve the correct register address
 *                  shift for a given pin.
 * @return          register address shift ([0-2])
 * @param[in]       which: pin ([0-23])
 */
static uint8_t tca6424_get_addr_shift(uint8_t which)
{
    uint8_t shift = which / 8;

    if (shift > TCA6424_MAX_ADDR_SHIFT){
        dbg_error("%s(): out of range! (%hhu)\n", __func__);
    } else {
        dbg_verbose("%s(): which=%hhu shift=%hhu\n", __func__, which, shift);
    }

    return shift;
}

/**
 * @brief           Read content from a given register of a given
 *                  device on a given I2C bus
 * @return          0 on success, standard error codes otherwise
 * @param[in]       dev: I2C bus device
 * @param[in]       addr: I2C device address
 * @param[in]       reg: I2C device register address
 * @param[out]      val: content read from I2C device register
 */
static int tca6424_i2c_get(struct i2c_dev_s *dev, uint8_t addr, uint8_t reg,
                           uint8_t *val)
{
    int ret;
    struct i2c_msg_s msg[] = {
        {
            .addr = addr,
            .flags = 0,
            .buffer = &reg,
            .length = 1,
        }, {
            .addr = addr,
            .flags = I2C_M_READ,
            .buffer = val,
            .length = 1,
        },
    };

    ret = I2C_TRANSFER(dev, msg, 2);
    if (!ret) {
        dbg_verbose("%s(): addr=0x%02hhX, reg=0x%02hhX: read 0x%02hhX\n",
                    __func__, addr, reg, *val);
    } else {
        dbg_error("%s(): addr=0x%02hhX, reg=0x%02hhX: failed!\n",
                  __func__, addr, reg);
    }

    return ret;
}

/**
 * @brief           Write data into a given register of a given
 *                  device on a given I2C bus
 * @return          0 on success, standard error codes otherwise
 * @param[in]       dev: I2C bus device
 * @param[in]       addr: I2C device address
 * @param[in]       reg: I2C device register address
 * @param[in]       val: data to be written into I2C device register
 */
static int tca6424_i2c_set(struct i2c_dev_s *dev, uint8_t addr, uint8_t reg,
                           uint8_t val)
{
    int ret;
    uint8_t cmd[2] = {reg, val};
    struct i2c_msg_s msg[] = {
        {
            .addr = addr,
            .flags = 0,
            .buffer = cmd,
            .length = 2,
        },
    };

    dbg_verbose("%s(): addr=0x%02hhX: reg=0x%02hhX, val=0x%02hhX\n",
                __func__, addr, reg, val);
    ret = I2C_TRANSFER(dev, msg, 1);
    if (ret) {
        dbg_error("%s(): failed to write register! (%d)\n", __func__, ret);
    }

    return ret;
}

/**
 * @brief           Initialize TCA6424 device.
 *
 *                  To be called first and only once until deinitialization.
 * @return          Initialized TCA6424 device structure
 * @param[in]       dev: I2C bus device
 * @param[in]       addr: I2C address of the TCA6424 chip
 */
tca6424_device *tca6424_init(struct i2c_dev_s *i2c_dev, uint8_t addr)
{
    tca6424_device *tca6424_dev;

    if ((i2c_dev == NULL) || (addr >= 0x7F)) {
        dbg_error("%s(): invalid argument!\n", __func__);
        return NULL;
    }

    tca6424_dev = (void *) malloc(sizeof(tca6424_device));
    if (!tca6424_dev) {
        dbg_error("%s(): failed to alloc device struct!\n", __func__);
        return NULL;
    }

    tca6424_dev->i2c_dev = i2c_dev;
    tca6424_dev->addr = addr;

    dbg_verbose("%s(): tca6424 device successfully created.\n", __func__);
    return tca6424_dev;
}

/**
 * @brief           Deinitialize TCA6424 device.
 * @param[in]       dev: TCA6424 device
 */
void tca6424_deinit(tca6424_device *dev)
{
    if (!dev) {
        dbg_error("%s(): invalid argument!\n", __func__);
        return;
    }

    free(dev);
}

/**
 * @brief           Configure selected I/O pin as an input
 * @return          0 on success, standard error codes otherwise
 * @param[in]       dev: TCA6424 device
 * @param[in]       which: pin ([0-23])
 */
int tca6424_set_direction_in(tca6424_device *dev, uint8_t which)
{
    uint8_t reg;
    int ret;
    uint8_t shift;

    if (!dev) {
        dbg_error("%s(): invalid argument!\n", __func__);
        return -EINVAL;
    }

    dbg_verbose("%s(): which=%hhu\n", __func__, which);
    /* Configure pin as input
     *
     * The Configuration Register (register 3) configures the direction of
     * the I/O pins. If a bit in this register is set to 1,
     * the corresponding port pin is enabled as an input with a
     * high-impedance output driver. If a bit in this register is
     * cleared to 0, the corresponding port pin is enabled as an output.
     */
    shift = tca6424_get_addr_shift(which);
    if (shift > TCA6424_MAX_ADDR_SHIFT) {
        return -EINVAL;
    }
    ret = tca6424_i2c_get(dev->i2c_dev, dev->addr,
                          TCA6424_CONFIG0_REG + shift, &reg);
    if (ret)
        return -EIO;
    dbg_verbose("%s(): current cfg=0x%02X\n", __func__, reg);
    which = which - (8 * shift);
    reg |= (1 << which);
    dbg_verbose("%s(): new cfg=0x%02X\n", __func__, reg);
    ret = tca6424_i2c_set(dev->i2c_dev, dev->addr,
                          TCA6424_CONFIG0_REG + shift, reg);
    if (ret) {
         return -EIO;
    }

    return 0;
}

/**
 * @brief           Configure selected I/O pin as an output
 * @return          0 on success, standard error codes otherwise
 * @param[in]       dev: TCA6424 device
 * @param[in]       which: pin ([0-23])
 */
int tca6424_set_direction_out(tca6424_device *dev, uint8_t which)
{
    uint8_t reg;
    int ret;
    uint8_t shift;

    if (!dev) {
        dbg_error("%s(): invalid argument!\n", __func__);
        return -EINVAL;
    }

    dbg_verbose("%s(): which=%hhu\n", __func__, which);
    /* Configure pin as output
     *
     * The Configuration Register configures the direction of
     * the I/O pins. If a bit in this register is set to 1,
     * the corresponding port pin is enabled as an input with a
     * high-impedance output driver. If a bit in this register is
     * cleared to 0, the corresponding port pin is enabled as an output.
     */
    shift = tca6424_get_addr_shift(which);
    if (shift > TCA6424_MAX_ADDR_SHIFT) {
        return -EINVAL;
    }
    ret = tca6424_i2c_get(dev->i2c_dev, dev->addr,
                          TCA6424_CONFIG0_REG + shift, &reg);
    if (ret)
        return -EIO;
    dbg_verbose("%s(): current cfg=0x%02X\n", __func__, reg);
    which = which - (8 * shift);
    reg &= ~(1 << which);
    dbg_verbose("%s(): new cfg=0x%02X\n", __func__, reg);
    ret = tca6424_i2c_set(dev->i2c_dev, dev->addr,
                          TCA6424_CONFIG0_REG + shift, reg);
    if (ret)
        return -EIO;

    return 0;
}


/**
 * @brief           Return the direction of selected I/O pin.
 * @return          < 0: standard error code
 *                  0: output
 *                  1: input
 * @param[in]       dev: TCA6424 device
 * @param[in]       which: pin ([0-23])
 */
int tca6424_get_direction(tca6424_device *dev, uint8_t which)
{
    uint8_t direction;
    int ret;
    uint8_t shift;

    if (!dev) {
        dbg_error("%s(): invalid argument!\n", __func__);
        return -EINVAL;
    }

    dbg_verbose("%s(): which=%hhu", __func__, which);
    /*
     * The Configuration Register (register 3) configures the direction of
     * the I/O pins. If a bit in this register is set to 1,
     * the corresponding port pin is enabled as an input with a
     * high-impedance output driver. If a bit in this register is
     * cleared to 0, the corresponding port pin is enabled as an output.
     */
    shift = tca6424_get_addr_shift(which);
    if (shift > TCA6424_MAX_ADDR_SHIFT) {
        return -EINVAL;
    }
    ret = tca6424_i2c_get(dev->i2c_dev, dev->addr,
                          TCA6424_CONFIG0_REG + shift, &direction);
    if (ret)
        return -EIO;
    which = which - (8 * shift);
    direction = (direction & (1 << which)) >> which;
    return direction;
}


/**
 * @brief           Enable/disable I/O polarity inversion.
 * @return          0 on success, standard error codes otherwise
 * @param[in]       dev: TCA6424 device
 * @param[in]       which: pin ([0-23])
 * @param[in]       inverted: 0: disabled (polarity not inverted)
 *                            1: enabled (polarity is inverted)
 */
int tca6424_set_polarity_inverted(tca6424_device *dev,
                                  uint8_t which, uint8_t inverted)
{
    uint8_t polarity;
    int ret;
    uint8_t shift;

    if (!dev) {
        dbg_error("%s(): invalid argument!\n", __func__);
        return -EINVAL;
    }

    dbg_verbose("%s(): which=%hhu inverted=%hhu\n", __func__, which, inverted);
    /* Configure pin polarity inversion
     *
     * The Polarity Inversion Register (register 2) allows
     * polarity inversion of pins defined as inputs by the Configuration
     * Register. If a bit in this register is set (written with 1),
     * the corresponding port pin's polarity is inverted. If a bit in
     * this register is cleared (written with a 0), the corresponding
     * port pin's original polarity is retained.
     */
    shift = tca6424_get_addr_shift(which);
    if (shift > TCA6424_MAX_ADDR_SHIFT) {
        return -EINVAL;
    }
    ret = tca6424_i2c_get(dev->i2c_dev, dev->addr,
                          TCA6424_POLARITY0_REG + shift, &polarity);
    if (ret != 0)
        return -EIO;
    dbg_verbose("%s(): current polarity reg=0x%02hhX\n", __func__, polarity);
    which = which - (8 * shift);
    if (inverted) {
        polarity |= (1 << which);
    } else {
        polarity &= ~(1 << which);
    }
    dbg_verbose("%s(): new polarity reg=0x%02hhX\n", __func__, polarity);
    ret = tca6424_i2c_set(dev->i2c_dev, dev->addr,
                          TCA6424_POLARITY0_REG + shift, polarity);
    if (ret)
        return -EIO;

    return 0;
}


/**
 * @brief           Return polarity inversion status (enabled/disabled).
 * @return          < 0: standard error code
 *                  0: disabled (not inverted)
 *                  1: enabled (inverted)
 * @param[in]       dev: TCA6424 device
 * @param[in]       which: pin ([0-23])
 */
int tca6424_get_polarity_inverted(tca6424_device *dev, uint8_t which)
{
    uint8_t polarity;
    int ret;
    uint8_t shift;

    if (!dev) {
        dbg_error("%s(): invalid argument!\n", __func__);
        return -EINVAL;
    }

    dbg_verbose("%s(): which=%hhu\n", __func__, which);
    /*
     * The Configuration Register (register 3) configures the direction of
     * the I/O pins. If a bit in this register is set to 1,
     * the corresponding port pin is enabled as an input with a
     * high-impedance output driver. If a bit in this register is
     * cleared to 0, the corresponding port pin is enabled as an output.
     */
    shift = tca6424_get_addr_shift(which);
    if (shift > TCA6424_MAX_ADDR_SHIFT) {
        return -EINVAL;
    }
    ret = tca6424_i2c_get(dev->i2c_dev, dev->addr,
                          TCA6424_POLARITY0_REG + shift, &polarity);
    if (ret)
        return -EIO;
    dbg_verbose("%s(): polarity reg=0x%02hhX\n", __func__, polarity);
    which = which - (8 * shift);
    polarity = (polarity & (1 << which)) >> which;
    dbg_verbose("%s(): polarity=0x%hhu\n", __func__, polarity);

    return polarity;
}


/**
 * @brief           Set output pin value.
 * @return          0 on success, standard error codes otherwise
 * @param[in]       dev: TCA6424 device
 * @param[in]       which: pin ([0-23])
 * @param[in]       val: output pin value (0 (low) or 1 (high))
 */
int tca6424_set(tca6424_device *dev, uint8_t which, uint8_t val)
{
    uint8_t reg;
    int ret;
    uint8_t shift;

    if (!dev) {
        dbg_error("%s(): invalid argument!\n", __func__);
        return -EINVAL;
    }

    dbg_verbose("%s(): which=%hhu, val=%hhu\n", __func__, which, val);
    /* Set output pins default value (before configuring it as output
     *
     * The Output Port Register (register 1) shows the outgoing logic
     * levels of the pins defined as outputs by the Configuration Register.
     */
    shift = tca6424_get_addr_shift(which);
    if (shift > TCA6424_MAX_ADDR_SHIFT) {
        return -EINVAL;
    }
    ret = tca6424_i2c_get(dev->i2c_dev, dev->addr,
                          TCA6424_OUTPUT0_REG + shift, &reg);
    if (ret)
        return -EIO;
    dbg_verbose("%s(): current reg=0x%02hhX\n", __func__, reg);
    which = which - (8 * shift);
    if (val) {
        reg |= (1 << which);
    } else {
        reg &= ~(1 << which);
    }
    dbg_verbose("%s(): new reg=0x%02hhX\n", __func__, reg);
    ret = tca6424_i2c_set(dev->i2c_dev, dev->addr,
                          TCA6424_OUTPUT0_REG + shift, reg);
    if (ret)
        return -EIO;

    return 0;
}


/**
 * @brief           Return input pin value.
 * @return          Input pin value (0 (low) or 1 (high))
 * @param[in]       dev: TCA6424 device
 * @param[in]       which: pin ([0-23])
 */
int tca6424_get(tca6424_device *dev, uint8_t which)
{
    uint8_t in;
    int ret;
    uint8_t shift;

    if (!dev) {
        dbg_error("%s(): invalid argument!\n", __func__);
        return -EINVAL;
    }

    dbg_verbose("%s(): which=%hhu\n", __func__, which);
    /*
     * The Input Port Register (register 0) reflects the incoming logic
     * levels of the pins, regardless of whether the pin is defined as an
     * input or an output by the Configuration Register. They act only on
     * read operation.
     */
    shift = tca6424_get_addr_shift(which);
    if (shift > TCA6424_MAX_ADDR_SHIFT) {
        return -EINVAL;
    }
    ret = tca6424_i2c_get(dev->i2c_dev, dev->addr,
                          TCA6424_INPUT0_REG + shift, &in);
    if (ret)
        return -EIO;
    dbg_verbose("%s(): input reg=0x%02hhX\n", in);
    which = which - (8 * shift);
    in &= (1 << which);
    in = !!in;
    dbg_verbose("%s(): input=%hhu\n", __func__, in);

    return in;
}
