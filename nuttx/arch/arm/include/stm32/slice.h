/****************************************************************************
 *
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *
 ****************************************************************************/

#ifndef _STM32_SLICE_H_
#define _STM32_SLICE_H_

#include <nuttx/irq.h>

int slice_init(xcpt_t irqhandler);
int slice_uninit(void);
void slice_host_int_set(bool value);
bool slice_base_det_read(void);
void slice_vbus_en_sw(bool value);

#endif /* _STM32_SLICE_H_ */

