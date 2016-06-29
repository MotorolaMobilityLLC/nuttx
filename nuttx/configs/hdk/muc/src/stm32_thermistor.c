/**
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

#include <stdint.h>

typedef struct {
    int16_t rt; /* Real temperature in Celsius */
    int16_t ft; /* FuelGauge temperature in Celsius */
} lut_entry_t;

/* fuelgauge to real LUT */
static const lut_entry_t f2r[] = {
    /* Real           FuelGauge */
    { -35,            -22, },
    { -34,            -21, },
    { -31,            -20, },
    { -28,            -19, },
    { -25,            -18, },
    { -23,            -17, },
    { -21,            -16, },
    { -18,            -15, },
    { -16,            -14, },
    { -15,            -13, },
    { -13,            -12, },
    { -11,            -11, },
    { -10,            -10, },
    {  -8,             -9, },
    {  -7,             -8, },
    {  -6,             -7, },
    {  -4,             -6, },
    {  -3,             -5, },
    {  -2,             -4, },
    {  -1,             -3, },
    {   0,             -2, },
    {   1,             -1, },
    {   2,              0, },
    {   3,              1, },
    {   4,              2, },
    {   5,              3, },
    {   6,              4, },
    {   7,              5, },
    {   8,              6, },
    {   9,              7, },
    {  10,              8, },
    {  11,              9, },
    {  12,             10, },
    {  13,             11, },
    {  14,             12, },
    {  15,             13, },
    {  16,             14, },
    {  16,             15, },
    {  17,             16, },
    {  18,             17, },
    {  19,             18, },
    {  20,             19, },
    {  21,             20, },
    {  22,             21, },
    {  23,             22, },
    {  24,             23, },
    {  25,             24, },
    {  26,             25, },
    {  26,             26, },
    {  27,             27, },
    {  28,             28, },
    {  29,             29, },
    {  30,             30, },
    {  31,             31, },
    {  32,             32, },
    {  33,             33, },
    {  34,             34, },
    {  35,             35, },
    {  36,             36, },
    {  37,             37, },
    {  39,             38, },
    {  40,             39, },
    {  41,             40, },
    {  42,             41, },
    {  43,             42, },
    {  44,             43, },
    {  45,             44, },
    {  47,             45, },
    {  48,             46, },
    {  49,             47, },
    {  50,             48, },
    {  52,             49, },
    {  53,             50, },
    {  55,             51, },
    {  56,             52, },
    {  58,             53, },
    {  60,             54, },
    {  62,             55, },
    {  63,             56, },
    {  65,             57, },
    {  68,             58, },
    {  70,             59, },
    {  72,             60, },
    {  74,             61, },
    {  77,             62, },
    {  79,             63, },
    {  80,             64, },
};
/* real to fuelgauge LUT */
static const lut_entry_t r2f[] = {
    /* Real     FuelGauge */
    { -35,            -21, },
    { -34,            -21, },
    { -33,            -21, },
    { -32,            -21, },
    { -31,            -20, },
    { -30,            -20, },
    { -29,            -20, },
    { -28,            -19, },
    { -27,            -19, },
    { -26,            -19, },
    { -25,            -18, },
    { -24,            -18, },
    { -23,            -17, },
    { -22,            -17, },
    { -21,            -16, },
    { -20,            -16, },
    { -19,            -15, },
    { -18,            -15, },
    { -17,            -14, },
    { -16,            -14, },
    { -15,            -13, },
    { -14,            -13, },
    { -13,            -12, },
    { -12,            -11, },
    { -11,            -11, },
    { -10,            -10, },
    {  -9,             -9, },
    {  -8,             -9, },
    {  -7,             -8, },
    {  -6,             -7, },
    {  -5,             -6, },
    {  -4,             -5, },
    {  -3,             -5, },
    {  -2,             -4, },
    {  -1,             -3, },
    {   0,             -2, },
    {   1,             -1, },
    {   2,              0, },
    {   3,              1, },
    {   4,              2, },
    {   5,              3, },
    {   6,              4, },
    {   7,              5, },
    {   8,              6, },
    {   9,              7, },
    {  10,              8, },
    {  11,              9, },
    {  12,             10, },
    {  13,             11, },
    {  14,             12, },
    {  15,             13, },
    {  16,             14, },
    {  17,             15, },
    {  18,             17, },
    {  19,             18, },
    {  20,             19, },
    {  21,             20, },
    {  22,             21, },
    {  23,             22, },
    {  24,             23, },
    {  25,             24, },
    {  26,             25, },
    {  27,             26, },
    {  28,             27, },
    {  29,             29, },
    {  30,             30, },
    {  31,             31, },
    {  32,             32, },
    {  33,             33, },
    {  34,             34, },
    {  35,             35, },
    {  36,             36, },
    {  37,             37, },
    {  38,             38, },
    {  39,             38, },
    {  40,             39, },
    {  41,             40, },
    {  42,             41, },
    {  43,             42, },
    {  44,             43, },
    {  45,             44, },
    {  46,             45, },
    {  47,             45, },
    {  48,             46, },
    {  49,             47, },
    {  50,             48, },
    {  51,             48, },
    {  52,             49, },
    {  53,             50, },
    {  54,             50, },
    {  55,             51, },
    {  56,             52, },
    {  57,             52, },
    {  58,             53, },
    {  59,             54, },
    {  60,             54, },
    {  61,             55, },
    {  62,             55, },
    {  63,             56, },
    {  64,             56, },
    {  65,             57, },
    {  66,             57, },
    {  67,             58, },
    {  68,             58, },
    {  69,             59, },
    {  70,             59, },
    {  71,             60, },
    {  72,             60, },
    {  73,             60, },
    {  74,             61, },
    {  75,             61, },
    {  76,             61, },
    {  77,             62, },
    {  78,             62, },
    {  79,             63, },
    {  80,             62, },
};

#define     F2R_TP_NUM      (sizeof(f2r)/sizeof(lut_entry_t))
#define     F2R_TP_MIN      (0)
#define     F2R_TP_MAX      (F2R_TP_NUM-1)

#define     R2F_TP_NUM      (sizeof(r2f)/sizeof(lut_entry_t))
#define     R2F_TP_MIN      (0)
#define     R2F_TP_MAX      (R2F_TP_NUM-1)

/**
 * @brief Convert real-temperature to fuelgauge-temperature.
 *
 * This function is called by max17050 driver on /nuttx/drivers/power/max17050.c
 * need to include header file /nuttx/drivers/power/max17050_thermistor.h
 * and config CONFIG_BATTERY_MAX17050_THERMISTOR_ACCURATE_TEMP.
 *
 * @param real-temperature integer value (x10 Celsius).
 * @return fuelgauge-temperature integer value (x10 Celsius).
 */
int max17050_thermistor_real_to_fuelgauge(int real_temp)
{
    int fuelgauge_temp, i, dt;

    i = real_temp / 10 - r2f[R2F_TP_MIN].rt;

    if (real_temp < 0) {
        i -= 1;
    }

    if (i < R2F_TP_MIN) {
        fuelgauge_temp =  r2f[R2F_TP_MIN].ft * 10;
    } else if (i >= R2F_TP_MAX) {
        fuelgauge_temp = r2f[R2F_TP_MAX].ft * 10;
    } else {
        dt = real_temp - r2f[i].rt * 10;
        fuelgauge_temp = r2f[i].ft * 10 + (r2f[i+1].ft - r2f[i].ft) * dt;
    }

    return fuelgauge_temp; /* return x10 Celsius */
}

/**
 * @brief Convert fuelgauge-temperature to real-temperature.
 *
 * This function is called by max17050 driver on /nuttx/drivers/power/max17050.c
 * need to include header file /nuttx/drivers/power/max17050_thermistor.h
 * and config CONFIG_BATTERY_MAX17050_THERMISTOR_ACCURATE_TEMP.
 *
 * @param fuelgauge-temperature integer value (x10 Celsius).
 * @return real-temperature integer value (x10 Celsius).
 */
int max17050_thermistor_fuelgauge_to_real(int fuelgauge_temp)
{
    int real_temp, i, dt;

    i = fuelgauge_temp / 10 - f2r[F2R_TP_MIN].ft;

    if (fuelgauge_temp < 0) {
        i -= 1;
    }

    if (i < F2R_TP_MIN) {
        real_temp =  f2r[F2R_TP_MIN].rt * 10;
    } else if (i >= F2R_TP_MAX) {
        real_temp = f2r[F2R_TP_MAX].rt * 10;
    } else {
        dt = fuelgauge_temp - f2r[i].ft * 10;
        real_temp = f2r[i].rt * 10 + (f2r[i+1].rt - f2r[i].rt) * dt;
    }

    return real_temp; /* return x10 Celsius */
}
