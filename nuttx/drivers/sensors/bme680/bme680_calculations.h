/*
 ****************************************************************************
 * Copyright (C) 2015 Bosch Sensortec GmbH
 *
 * File : bme680_calculations.h
 *
* Date : 2016/06/10
*
* Revision: 2.0.0
 *
 * Usage: Sensor Driver for BME680 sensor
 *
 ****************************************************************************
 * \Section Disclaimer
 *
 * License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
 **************************************************************************/
/*! \file bme680_calculations.h
 \brief BME680 Sensor Driver calculation Header File */

/*************************************************************************/
#ifndef __BME680_CALCULATIONS_H__
#define __BME680_CALCULATIONS_H__

#ifdef __cplusplus
extern "C"
{
#endif

/***************************************************************************
 Header files
 ****************************************************************************/
#include "bme680.h"

/***************************************************************************
 Macros, Enums, Constants
 ****************************************************************************/

/***************************************************************************
 Module globals, typedefs
 ****************************************************************************/

/***************************************************************************
 Function definitions
 ****************************************************************************/
/* bme680_calculations.h */
#ifdef	FIXED_POINT_COMPENSATION
/**************************************************************/
/**\name	FUNCTION FOR INTEGER OUTPUT GAS*/
/**************************************************************/
/*!
 * @brief This function is used to convert uncompensated gas data to
 * compensated gas data using compensation formula(integer version)
 */
s32 bme680_calculate_gas_int32(u16 gas_adc_u16, u8 gas_range_u8,
	struct bme680_t *bme680);
/**************************************************************/
/**\name	FUNCTION FOR INTEGER OUTPUT TEMPERATURE*/
/**************************************************************/
/*!
 * @brief This function is used to convert the uncompensated
 * temperature data to compensated temperature data using
 * compensation formula(integer version)
 *
 * @note Returns the value in 0.01 degree Centigrade
 * Output value of "5123" equals 51.23 DegC.
 */
s32 bme680_compensate_temperature_int32(u32 v_uncomp_temperature_u32,
	struct bme680_t *bme680);
/**************************************************************/
/**\name	FUNCTION FOR INTEGER OUTPUT HUMIDITY*/
/**************************************************************/
/*!
 * @brief This function is used to convert the uncompensated
 * humidity data to compensated humidity data using
 * compensation formula(integer version)
 *
 * @note Returns the value in %rH as unsigned 32bit integer
 * in Q22.10 format(22 integer 10 fractional bits).
 * @note An output value of 42313 represents 42313 / 1024 = 41.321 %rH
 */
s32 bme680_compensate_humidity_int32(u32 v_uncomp_humidity_u32,
	struct bme680_t *bme680);
/**************************************************************/
/**\name	FUNCTION FOR INTEGER OUTPUT PRESSURE*/
/**************************************************************/
/*!
 * @brief This function is used to convert the uncompensated
 * pressure data to compensated pressure data data using
 * compensation formula(integer version)
 *
 * @note Returns the value in Pascal(Pa)
 * Output value of "96386" equals 96386 Pa = 963.86 hPa = 963.86 millibar
 */
s32 bme680_compensate_pressure_int32(u32 v_uncomp_pressure_u32,
	struct bme680_t *bme680);
/**************************************************************/
/**\name	FUNCTION FOR INTEGER TEMPERATURE-RESISTANCE*/
/**************************************************************/
/*!
 * @brief This function is used to convert temperature to resistance
 * using the integer compensation formula
 */
u8 bme680_convert_temperature_to_resistance_int32(u16 heater_temp_u16,
	s16 ambient_temp_s16, struct bme680_t *bme680);
/**************************************************************/
/**\name	FUNCTION TO CONVERT INT32_H to U16_H BIT OPUTPUT*/
/**************************************************************/
/*!
 * @brief Reads actual humidity from uncompensated humidity
 *
 * @note Returns the value in %rH as unsigned 16bit integer
 * @note An output value of 42313 represents 42313/512 = 82.643 %rH
 */
u16 bme680_compensate_H_int32_sixteen_bit_output(u32 v_uncomp_humidity_u32,
	struct bme680_t *bme680);
/**************************************************************/
/**\name	FUNCTION TO CONVERT INT32_T to S16_T BIT OPUTPUT*/
/**************************************************************/
/*!
 * @brief Reads actual temperature from uncompensated temperature
 *
 * @note Returns the value with 500LSB/DegC centred around 24 DegC
 * output value of "5123" equals(5123/500)+24 = 34.246DegC
 */
s16 bme680_compensate_T_int32_sixteen_bit_output(u32 v_uncomp_temperature_u32,
	struct bme680_t *bme680);
/**************************************************************/
/**\name	FUNCTION TO CONVERT INT32_P to U24_P BIT OPUTPUT*/
/**************************************************************/
/*!
 * @brief Reads actual pressure from uncompensated pressure in Pa.
 *
 * @note Output value of "12337434" represents
 * 12337434 / 128 = 96386.2 Pa = 963.862 hPa
 */
u32 bme680_compensate_P_int32_twentyfour_bit_output(u32 v_uncomp_pressure_u32,
	struct bme680_t *bme680);

#else
/**************************************************************/
/**\name	FUNCTION FOR FLOAT OUTPUT GAS */
/**************************************************************/
/*!
 * @brief This function is used to convert uncompensated gas data to
 * compensated gas data using compensation formula
 */
double bme680_compensate_gas_double(u16 gas_adc_u16, u8 gas_range_u8,
	struct bme680_t *bme680);

/**************************************************************/
/**\name	FUNCTION FOR FLOAT OUTPUT HUMIDITY */
/**************************************************************/

/*!
 * @brief This function is used to convert the uncompensated
 * humidity data to compensated humidity data data using
 * compensation formula
 *
 * @note returns the value in relative humidity (%rH)
 * @note Output value of "42.12" equals 42.12 %rH
 */
double bme680_compensate_humidity_double(u16 uncom_humidity_u16,
	double comp_temperature, struct bme680_t *bme680);

/**************************************************************/
/**\name	FUNCTION FOR FLOAT OUTPUT PRESSURE*/
/**************************************************************/
/*!
 * @brief This function is used to convert the uncompensated
 * pressure data to compensated data using compensation formula
 *
 * @note Returns pressure in Pa as double.
 * @note Output value of "96386.2" equals 96386.2 Pa = 963.862 hPa.
 */
double bme680_compensate_pressure_double(u32 uncom_pressure_u32,
	struct bme680_t *bme680);

/**************************************************************/
/**\name	FUNCTION FOR FLOAT OUTPUT TEMPERATURE*/
/**************************************************************/

/*!
 * @brief This function used to convert temperature data
 * to uncompensated temperature data using compensation formula
 *
 * @note returns the value in Degree centigrade
 * @note Output value of "51.23" equals 51.23 DegC.
 */
double bme680_compensate_temperature_double(u32 uncom_temperature_u32,
	struct bme680_t *bme680);

/**************************************************************/
/**\name	FUNCTION FOR TEMPERATURE TO RESISTANCE */
/**************************************************************/

/*!
 * @brief This function is used to convert temperature to resistance
 * using the compensation formula
 */
double bme680_convert_temperature_to_resistance_double(u16 heater_temp_u16,
	s16 ambient_temp_s16, struct bme680_t *bme680);
#endif
/* bme680_calculations.h */

#ifdef __cplusplus
}
#endif

#endif
