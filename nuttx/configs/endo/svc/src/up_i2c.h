/*
 * Copyright (c) 2014 Google Inc.
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

/****************************************************************************
 * configs/endo/svc/src/up_i2c.h
 * Endo/SVC I2C support, used for communication with the switch and the
 *  IO Expanders
 *
 ****************************************************************************/
#ifndef __CONFIGS_ENDO_INCLUDE_UP_I2C_H
#define __CONFIGS_ENDO_INCLUDE_UP_I2C_H

/* STM32 bus to the switch and IO expanders */
#define I2C_SW_BUS                  2

/*
 * I2C2 bus addresses on Endo
 */
#define I2C_ADDR_IOEXP_U601         0x20    /* IO Expander U601 */
#define I2C_ADDR_IOEXP_U602         0x21    /* IO Expander U602 */
#define I2C_ADDR_SWITCH             0x44    /* Unipro switch */
/* I2C flags */
#define I2C_NOFLAGS                 0x00    /* No specific flags */

extern inline int i2c_init_comm(uint8_t bus);
extern inline void i2c_reset(void);
extern inline void i2c_select_device(uint8_t addr);
extern inline int i2c_switch_send_msg(uint8_t *buf, unsigned int size);
extern inline int i2c_switch_receive_msg(uint8_t *buf, unsigned int size);
extern int i2c_ioexp_write(uint8_t *msg, int size, uint8_t addr);
extern int i2c_ioexp_read(uint8_t *msg, int size, uint8_t addr);
#endif // __CONFIGS_ENDO_INCLUDE_UP_I2C_H
