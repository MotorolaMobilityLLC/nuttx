/*
*
****************************************************************************
* Copyright (C) 2015 Bosch Sensortec GmbH
*
* File : bme680.h
*
* Date : 2016/08/17
*
* Revision: 2.0.1
*
* Usage: Sensor Driver for BME680 sensor
*
****************************************************************************
*
* Section Disclaimer
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
/*! \file bme680.h
    \brief BME680 Sensor Driver Support Header File */

#ifndef __BME680_H__
#define __BME680_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* BME680 Release version 2.0.0
BME680 Release Version format major_version.minor_version.point_version
Example: 2.0.0 */
#define BME680_API_REL_MAJOR_VERSION (2)
#define BME680_API_REL_MINOR_VERSION (0)
#define BME680_API_REL_POINT_VERSION (1)

/***************************************************************************
			Header files
****************************************************************************/
#include "sensor_api_common_types.h"


/* sensor_api_common_types.h */

/************************************************************************
			Macros, Enums, Constants
*************************************************************************/
#define BME680_PRESSURE				(0U)
#define BME680_TEMPERATURE			(1U)
#define BME680_HUMIDITY				(2U)
#define BME680_GAS					(3U)
#define BME680_ALL					(4U)

#define BME680_STATUS_DATA_LEN			(2U)
#define BME680_TEMPERATURE_DATA_LEN		(3U)
#define BME680_PRESSURE_DATA_LEN		(3U)
#define BME680_GAS_DATA_LEN				(2U)
#define BME680_HUMIDITY_DATA_LEN		(2U)

#define BME680_PRESENT_DATA_FIELD				(1U)
#define BME680_PRESENT_AND_PREVIOUS_DATA_FIELD	(2U)
#define BME680_ALL_DATA_FIELD					(3U)

#define BME680_MAX_FIELD_INDEX		(3U)
#define BME680_FIELD_INDEX0			(0U)
#define BME680_FIELD_INDEX1			(1U)
#define BME680_FIELD_INDEX2			(2U)

/***************************************************************/
/**\name	BUS READ AND WRITE FUNCTION POINTERS        */
/***************************************************************/


/**< function pointer to the SPI or I2C burst read function */
typedef s8 (*sensor_burst_read)(u8 slave_addr, u8 reg_addr, u8 *data_u8,
	u32 length_u32);

typedef  s8 (*sensor_write)(u8 dev_addr, u8 reg_addr, u8 *reg_data_ptr,
	u8 data_len);
/**< function pointer for Write operation in either I2C or SPI*/
typedef  s8 (*sensor_read)(u8 dev_addr, u8 reg_addr, u8 *reg_data_ptr,
	u8 data_len);
/**< function pointer for Read operation in either I2C or SPI*/

#define BME680_MAX_NO_OF_SENSOR			(2)
/**< This macro used for maximum number of sensor*/

#define BME680_MDELAY_DATA_TYPE			u32
/**< This macro used for delay*/

#define BME680_CHIP_ID				(0x61)
/**< BME680 chip identifier */

/*#define BME680_SPECIFIC_FIELD_DATA_READ_ENABLED*/
/**< This macro is used to prevent the compilation
of single function calls when not used */

/*
 * Use below macro for fixed Point Calculation
 * else Floating Point calculation will be used
*/

#define FIXED_POINT_COMPENSATION

/* temperature to Resistance  formulae #defines */

/*
 *	Use any of the below constants according to
 *	the heater version of the sensor used
*/

#define HEATER_C1_ENABLE

/* Sensor Specific constants */
#define BME680_SLEEP_MODE				(0x00)
#define BME680_FORCED_MODE				(0x01)
#define BME680_PARALLEL_MODE			(0x02)
#define BME680_SEQUENTIAL_MODE			(0x03)
#define BME680_GAS_PROFILE_TEMPERATURE_MIN	(200)
#define BME680_GAS_PROFILE_TEMPERATURE_MAX	(400)
#define BME680_GAS_RANGE_RL_LENGTH		(16)
#define BME680_SIGN_BIT_MASK			(0x08)

#ifdef FIXED_POINT_COMPENSATION
/**< Multiply by 1000, In order to convert
float value into fixed point  */
#define BME680_MAX_HUMIDITY_VALUE		(102400)
#define BME680_MIN_HUMIDITY_VALUE		(0)
#else
#define BME680_MAX_HUMIDITY_VALUE		(double)(100.0)
#define BME680_MIN_HUMIDITY_VALUE		(double)(0.0)
#endif

/* BME680 I2C addresses */
#define BME680_I2C_ADDR_PRIMARY			(0x76)
#define BME680_I2C_ADDR_SECONDARY		(0x77)

/* Maximum no of gas profiles to be used */
#define BME680_MAX_PROFILES				(10)


/**************************************************************/
/**\name Interface selection macro */
/*************************************************************/

#define BME680_SPI_INTERFACE			(1)
#define BME680_I2C_INTERFACE			(2)



/* bme680_internal.h */

/***************************************************************/
/**\name	COMMON USED CONSTANTS      */
/***************************************************************/
/* Constants */
#define BME680_NULL_PTR				((void *)0)
#define BME680_RETURN_FUNCTION_TYPE		s8
#define BME680_INIT_VALUE			((u8)0)


/* Section 3.5: Function macros */
#define BME680_SET_REG(reg, data, mask, shift)\
	((reg & mask) | ((data << shift) & ~mask))
#define BME680_GET_REG(reg, mask, shift)\
	((reg & ~mask) >> shift)
#define DIFF(a, b)	((a > b)?(a - b):(b - a))


/*************************************************************
			Module globals, typedefs
**************************************************************/

/*!
 * @brief This structure holds all
 * calibration parameters
 */
struct  bme680_calibration_param_t {

	s8  par_T3;/**<calibration T3 data*/
	s8  par_P3;/**<calibration P3 data*/
	s8  par_P6;/**<calibration P6 data*/
	s8  par_P7;/**<calibration P7 data*/
	u8  par_P10;/**<calibration P10 data*/
	s8  par_H3;/**<calibration H3 data*/
	s8  par_H4;/**<calibration H4 data*/
	s8  par_H5;/**<calibration H5 data*/
	u8  par_H6;/**<calibration H6 data*/
	s8  par_H7;/**<calibration H7 data*/
	s8  par_GH1;/**<calibration GH1 data*/
	u8  res_heat_range;/**<resistance calculation*/
	s8  res_heat_val; /**<correction factor*/
	s8  range_switching_error;/**<range switching error*/
	s16 par_GH2;/**<calibration GH2 data*/
	u16 par_T1;/**<calibration T1 data*/
	s16 par_T2;/**<calibration T2 data*/
	u16 par_P1;/**<calibration P1 data*/
	s16 par_P2;/**<calibration P2 data*/
	s16 par_P4;/**<calibration P4 data*/
	s16 par_P5;/**<calibration P5 data*/
	s16 par_P8;/**<calibration P8 data*/
	s16 par_P9;/**<calibration P9 data*/
	u16 par_H1;/**<calibration H1 data*/
	u16 par_H2;/**<calibration H2 data*/
	s32 t_fine;/**<calibration T_FINE data*/
	s8  par_GH3;/**<calibration GH3 data*/
};
/*!
*	@brief bme680 structure
*	This structure holds all relevant
*	information about bme680
*/
struct  bme680_t {
	struct bme680_calibration_param_t cal_param;
	/**<This structure holds all the calibration parameters */
	u8 latest_field_index;
	/**<stores the field index of latest data */
	u8 recent_field_index;
	/**<stores the field index of recent data */
	u8 old_field_index;
	/**<stores the field index of old data */

	/**< The structure stores the calibration values*/
	u8 chip_id;
	/**< used to save the bme680's chip id*/
	u8 dev_addr;
	/**< used to store the I2C address*/
	u8 last_set_mode;
	/**< used to store the last set power mode*/
	u8 interface;
	/**< used to store the communication protocol*/
	sensor_write bme680_bus_write;
	/**< function pointer to the SPI or I2C write function */
	sensor_read bme680_bus_read;
	/**< function pointer to the SPI or I2C read function */
	sensor_burst_read bme680_burst_read;
	/**< function pointer to the SPI or I2C burst read function */
	void (*delay_msec)(BME680_MDELAY_DATA_TYPE);
	/**< function pointer to a delay in milliseconds function */
};

/*!
 * @brief This structure holds heater configuration
 * parameters
 */
struct bme680_heater_conf {

	u8 heatr_idacv[BME680_MAX_PROFILES];
	/**< used to store the idac parameter */
	u16 heatr_dur_shared;
	/**< variable to store heater duration for parallel mode */
	u16 heater_temp[BME680_MAX_PROFILES];
	/**< variable to store heater resistance */
	u16 heatr_dur[BME680_MAX_PROFILES];
	/**< variable to store heater duration for force and sequential mode*/
	u8 profile_cnt;
	/**< variable to store profile count for user reference */
};


/*!
 * @brief Enumeration for function return codes
 *
 */
enum  bme680_return_type {
	BME680_COMM_RES_OK,
	BME680_COMM_RES_ERROR = -1,
	BME680_ERROR_NULL_PTR = -2,
	BME680_CHIP_ID_ERROR	= -4,
	BME680_PROFILE_CNT_ERROR = -5
};
/*!
 *	@brief This enum holds different ODR values
 *	for Gas data
 */
enum  bme680_odr {
	BME680_ODR_0_59MS,
	BME680_ODR_62_5MS,
	BME680_ODR_125MS,
	BME680_ODR_250MS,
	BME680_ODR_500MS,
	BME680_ODR_1000MS,
	BME680_ODR_10MS,
	BME680_ODR_20MS,
	BME680_ODR_NONE
};
/*!
 *	@brief This enum holds gas control
 *	parameters
 */
enum  bme680_run_gas {
	BME680_RUN_GAS_DISABLE,
	BME680_RUN_GAS_ENABLE
};
/*!
 *	@brief This enum holds heater control
 *	parameters
 */
enum bme680_heatr_ctrl {
	BME680_HEATR_CTRL_ENABLE,
	BME680_HEATR_CTRL_DISABLE
};
/*!
 *	@brief This enum holds osrs setting
 *	of TPH data
 */
enum bme680_osrs_x {
	BME680_OSRS_NONE,
	BME680_OSRS_1X,
	BME680_OSRS_2X,
	BME680_OSRS_4X,
	BME680_OSRS_8X,
	BME680_OSRS_16X
};

/*!
 *	@brief This enum holds filter
 *	coefficient settings
 */
enum bme680_filter {
	BME680_FILTER_COEFF_0,
	BME680_FILTER_COEFF_1,
	BME680_FILTER_COEFF_3,
	BME680_FILTER_COEFF_7,
	BME680_FILTER_COEFF_15,
	BME680_FILTER_COEFF_31,
	BME680_FILTER_COEFF_63,
	BME680_FILTER_COEFF_127

};
/*!
 *	@brief This enum holds spi 3wire
 *	interrupt control setting parameters
 */
enum  bme680_spi_3w_intr {
	BME680_SPI_3W_INTR_DISABLE,
	BME680_SPI_3W_INTR_ENABLE
};
/*!
 *	@brief This enum holds spi_3w_interface
 *	enabling parameters
 */

enum bme680_spi_3w {
	BME680_SPI_3W_DISABLE,
	BME680_SPI_3W_ENABLE
};
/*!
 *	@brief This enum holds nb conversion
 *	parameters
 */
enum bme680_nb_conv {
	BME680_NB_CONV_SKIPPED,
	BME680_NB_CONV_1,
	BME680_NB_CONV_2,
	BME680_NB_CONV_3,
	BME680_NB_CONV_4,
	BME680_NB_CONV_5,
	BME680_NB_CONV_6,
	BME680_NB_CONV_7,
	BME680_NB_CONV_8,
	BME680_NB_CONV_9,
	BME680_NB_CONV_10,
};
/*!
 *	@brief This structure holds sensor configuration
 *	parameters
 */
struct  bme680_sens_conf {
	s8 nb_conv;
	/**< variable to store nb conversion */
	enum bme680_heatr_ctrl heatr_ctrl;
	/**< instance of heater control */
	enum bme680_odr odr;
	/**< instance of ODR */
	enum bme680_run_gas run_gas;
	/**< instance of gas enable */
	enum bme680_osrs_x osrs_hum;
	/**< instance of osrs for humidity */
	enum bme680_osrs_x osrs_temp;
	/**< instance of osrs for temperature */
	enum bme680_osrs_x osrs_pres;
	/**< instance of osrs for pressure */
	enum bme680_filter filter;
	/**< instance of filter */
	enum bme680_spi_3w_intr intr;
	/**< instance of 3_wire_spi for interrupt mode */
	enum bme680_spi_3w spi_3w;
	/**< instance of 3 wire spi interface */
};
/*!
 *	@brief This structure holds sensor status
 *	parameters
 */
struct bme680_status {
	u8 new_data;
	/**<New data flag */
	u8 gas_meas_stat;
	/**<Gas measuring status */
	u8 tphg_meas_stat;
	/**<TPHG status */
	u8 gas_meas_index;
	/**<Gas measuring index */
	u8 meas_index;
	/**<Measurement index */
	u8 gas_valid;
	/**<gas data valid check */
	u8 heatr_stab;
	/**<stability check */
};

/*!
 *	@brief This structure holds the compensated data
 *	for either fixed or floating point compensation
 */

struct bme680_comp_field_data {

	#ifdef FIXED_POINT_COMPENSATION

	s32 comp_pressure;
	/**< the value of field2 compensated pressure*/
	s32 comp_temperature1;
	/**< the value of field2 compensated temperature1*/
	s32 comp_humidity;
	/**< the value of field2 compensated humidity*/
	s32 comp_gas;
	/**< the value of field2 compensated gas*/

	#else

	double comp_pressure;
	/**< the value of field2 compensated pressure*/
	double comp_temperature1;
	/**< the value of field2 compensated temperature1*/
	double comp_humidity;
	/**< the value of field2 compensated humidity*/
	double comp_gas;
	/**< the value of field2 compensated gas*/
	#endif
};
/*!
 *	@brief This structure holds the uncompensated data
 *	for either fixed or floating point compensation
 */

struct  bme680_uncomp_field_data {
	/**< data field status*/
	struct bme680_status status;

	u8 gas_range;
	/**<Range index of the back-end of the ADC */
	#ifdef FIXED_POINT_COMPENSATION

	u32 pres_adcv;
	/**< the value of field2 uncompensated pressure*/
	u32 temp_adcv;
	/**< the value of field2 uncompensated temperature1*/
	u16 hum_adcv;
	/**< the value of field2 uncompensated humidity*/
	u16 gas_res_adcv;
	/**< the value of field2 uncompensated gas*/

	#else

	double pres_adcv;
	/**< the value of pressure*/
	double temp_adcv;
	/**< adc value of temperature*/
	double hum_adcv;
	/**< adc value of humidity*/
	double gas_res_adcv;
	/**< adc value of gas resistance*/
	#endif
};

/*********************************************************
	Function declarations
**********************************************************/

/*********************************************************/
/**\name	 FUNCTION FOR BME680 INITIALIZE  */
/*********************************************************/
/*!
 * @brief This function is used to read the
 * the chip id and calibration data of the BME680 sensor
 * chip id is read in the register 0xD0/0x50(I2C/SPI) from bit 0 to 7
*/
enum bme680_return_type bme680_init(struct bme680_t *bme680);

/**************************************************/
/**\name	 FUNCTION FOR COMMON WRITE AND READ  */
/*************************************************/
/*!
 * @brief This function is used to write the data to
 * the given register
 */
enum bme680_return_type bme680_write_reg(u8 addr_u8, u8 *data_u8, u8 len_u8,
	struct bme680_t *bme680);

/*!
 * @brief This function is used to reads the data from
 * the given register
 */
enum bme680_return_type bme680_read_reg(u8 addr_u8, u8 *data_u8, u8 len_u8,
	struct bme680_t *bme680);


/**************************************************/
/**\name	 FUNCTION FOR NEW DATA STATUS
			OF FIELD0, FIELD1 AND FIELD2*/
/**************************************************/
/*!
 * @brief This function is used to read the new data0
 * @note Field-0(new_data_0),Field-1(new_data_1) and Field-2(new_data_2)
*/
enum bme680_return_type bme680_get_new_data(u8 *new_data_u8,
	u8 field_u8, struct bme680_t *bme680);

/*!
 * @brief This function is used to read the uncompensated
 * sensor data from Field-0, Field-1, Field-2 and page-1
*/
enum bme680_return_type bme680_get_uncomp_data(
	struct bme680_uncomp_field_data *uncomp_data, u8 field_count,
	u8 sensor_type, struct bme680_t *bme680);

/*!
 * @brief This function is used to get the
 * Operational Mode from the sensor in the
 * register 0x74 bit 0 and 1
*/
enum bme680_return_type bme680_get_power_mode(u8 *power_mode_u8,
	struct bme680_t *bme680);

/*!
 * @brief This function is used to set the
 * Operational Mode of the sensor in the
 * register 0x74 bit 0 and 1
*/
enum bme680_return_type bme680_set_power_mode(u8 power_mode_u8,
	struct bme680_t *bme680);

/*!
 * @brief This function is used to set the sensor configuration
*/
enum bme680_return_type bme680_set_sensor_config(
	struct bme680_sens_conf *sens_conf, struct bme680_t *bme680);

/*!
 * @brief This function is used for setting gas heater configuration
 * of the sensor from register 50 to 6E address
*/
enum bme680_return_type bme680_set_gas_heater_config(
	struct bme680_heater_conf *heatr_conf, u8 power_mode_u8,
	struct bme680_t *bme680);

/*!
 * @brief This function is used to get the sensor configuration
*/
enum bme680_return_type bme680_get_sensor_config(
	struct bme680_sens_conf *sens_conf, struct bme680_t *bme680);


/*!
 * @brief This function is used to read the sensor heater
 * configuration from register 50 to 6E address
*/
enum bme680_return_type bme680_get_gas_heater_config(
	struct bme680_heater_conf *heatr_conf, struct bme680_t *bme680);

/*!
 * @brief This function is used to compensate the TPHG raw
 * values of the sensor in order to convert to meaningful values
 *
 * @note: if "BME680_SPECIFIC_FIELD_DATA_READ_ENABLED" is not defined in
 * bme680.h then for any sensor_type function will perform
 * read operation for BME680_ALL.
 * @note : pressure and humidity depends on temperature.
*/
enum bme680_return_type bme680_compensate_data(
	struct bme680_uncomp_field_data uncomp_data[],
	struct bme680_comp_field_data comp_data[], u8 field_count,
	u8 sensor_type, struct bme680_t *bme680);

/*!
 * @brief This function is used to Align uncompensated data
 * from function bme680_get_uncomp_data()
*/
void bme680_align_uncomp_data(u8 *a_data_u8, u8 field_count, u8 sensor_type,
	struct bme680_uncomp_field_data *uncomp_data,
	struct bme680_t *bme680);

/*!
 * @brief This function is used to read the status according to filed index.
*/
enum bme680_return_type bme680_read_status_fields(
	struct bme680_uncomp_field_data *uncomp_data,
	u8 *a_data_u8, u8 *new_data,
	struct bme680_t *bme680);
/* bme680_calculations.h */

#ifdef __cplusplus
}
#endif

#endif
