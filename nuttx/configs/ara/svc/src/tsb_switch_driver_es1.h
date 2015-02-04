/**
 * Copyright (c) 2015 Google, Inc.
 * Google Confidential/Restricted
 * @author: Perry Hung
 */

#ifndef  _TSB_SWITCH_DRIVER_ES1_H_
#define  _TSB_SWITCH_DRIVER_ES1_H_

struct tsb_switch_driver *tsb_switch_es1_init(unsigned int i2c_bus);
void tsb_switch_es1_exit(void);

#endif


