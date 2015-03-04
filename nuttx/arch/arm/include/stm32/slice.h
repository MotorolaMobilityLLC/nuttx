/****************************************************************************
 *
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *
 ****************************************************************************/

#ifndef _STM32_SLICE_H_
#define _STM32_SLICE_H_

void slice_init(void);
void slice_uninit(void);
void slice_host_int_set(bool value);

#endif /* _STM32_SLICE_H_ */

