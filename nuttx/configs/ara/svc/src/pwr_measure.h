/**
 * Copyright (c) 2015 Google, Inc.
 * Google Confidential/Restricted.
 *
 * @brief Structure defining a Power Measure
 */

#ifndef __PWR_MEASURE_H__
#define __PWR_MEASURE_H__

#include <stdint.h>

typedef struct {
    int32_t uV; /* uV */
    int32_t uA; /* uA */
    int32_t uW; /* uW */
} pwr_measure;

#endif
