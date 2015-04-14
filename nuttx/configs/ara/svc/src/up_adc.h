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

/**
 * @file    configs/ara/svc/src/up_adc.h
 * @brief   SVC STM32 integrated ADC (Analog to Digital Converter) support.
 * @author  Patrick Titiano
 */

#ifndef __UP_ADC_H__
#define __UP_ADC_H__

#include <stdint.h>

/* ADC instances */
#define ADC1                            1
#define ADC2                            2
#define ADC3                            3
#define ADC_COUNT                       3
#define ADC_NCHANNEL                    16
#define ADC_DEFAULT_AVG_SAMPLE_COUNT    128 /* 128 averaging samples */
#define ADC_MAX_AVG_SAMPLE_COUNT        128 /* 128 averaging samples */
#define ADC_PRECISION                   ((uint32_t) 12) /* bits */

int adc_init(uint8_t adc, uint8_t channel, uint8_t count);
int adc_deinit(uint8_t adc, uint8_t channel);
int32_t adc_get_sampling_time(uint8_t adc);
int adc_get_data(uint8_t adc, uint8_t channel, uint32_t *data);

#endif
