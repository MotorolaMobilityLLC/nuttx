/****************************************************************************
 * configs/endo/svc/src/up_i2c.h
 * Endo/SVC I2C support, used for communication with the switch and the
 *  IO Expanders
 *
 * Copyright (C) 2014 Google, Inc.
 * Google Confidential/Restricted
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
