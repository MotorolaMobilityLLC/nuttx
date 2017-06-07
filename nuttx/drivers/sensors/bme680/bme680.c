/** \mainpage
****************************************************************************
* Copyright (C) 2015 Bosch Sensortec GmbH
*
* File : bme680.c
*
* Date : 2016/06/10
*
* Revision: 2.0.0
*
* Usage: Sensor Driver for BME680 sensor
*
****************************************************************************
* Section Disclaimer
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
/*! \file bme680.c
    \brief BME680 Sensor Driver Support source File */

/***************************************************************************
			Header files
****************************************************************************/
#include "bme680.h"
#include "bme680_calculations.h"
#include "bme680_internal.h"

/***************************************************************************
			Macros, Enums, Constants
****************************************************************************/


/***************************************************************************
			File globals, typedefs
****************************************************************************/


/* Static function declarations */
static enum bme680_return_type bme680_get_calib_param(struct bme680_t *bme680);

static void bme680_scale_to_multiplication_factor(u16 *duration_u16);

#ifndef	__linux__
static void bme680_buffer_restruct_burst_write(u8 arr[], u8 reg_addr,
	u8 data_size, u8 arr_size);
#endif

static u8 bme680_find_largest_index(u8 *meas_index);

static enum bme680_return_type bme680_set_memory_page(u8 memory_page_u8,
	struct bme680_t *bme680);

static void bme680_align_sensor_type_uncomp_data(u8 *a_data_u8, u8 index,
	u8 offset, u8 sensor_type,
	struct bme680_uncomp_field_data *uncomp_data);

static void bme680_packing_calib_param(u8 *a_data_u8, struct bme680_t *bme680);

static void bme680_copy_ordered_sensor_field_data(
	struct bme680_uncomp_field_data *sensor_data,
	u8 latest, u8 recent, u8 old, u8 sensor_type,
	struct bme680_uncomp_field_data *temp_sensor_data);

static void bme680_get_latest_recent_old_field_index(
	struct bme680_uncomp_field_data *sensor_data,
	struct bme680_t *bme680);

#ifdef BME680_SPECIFIC_FIELD_DATA_READ_ENABLED
static enum bme680_return_type bme680_get_field_specific_uncomp_data(
	u8 field_index, u8 sensor_type, u8 *a_data_u8, struct bme680_t *bme680);

static enum bme680_return_type bme680_Temp_field_specific_uncomp_read(
	u8 field_index,	u8 *a_data_u8, struct bme680_t *bme680);

static enum bme680_return_type bme680_Pressure_field_specific_uncomp_read(
	u8 field_index,	u8 *a_data_u8, struct bme680_t *bme680);

static enum bme680_return_type bme680_Humidity_field_specific_uncomp_read(
	u8 field_index,	u8 *a_data_u8, struct bme680_t *bme680);

static enum bme680_return_type bme680_Gas_field_specific_uncomp_read(
	u8 field_index, u8 *a_data_u8, struct bme680_t *bme680);

#endif


/***************************************************************************
				Function definitions
****************************************************************************/


/*!
 *	@brief This function is used to read the
 *  the chip id and calibration data of the BME680 sensor
 *	chip id is read in the register 0xD0/0x50(I2C/SPI) from bit 0 to 7
 *
 *	@param bme680 structure pointer.
 *
 *	@note Structure with the below data to be filled
 *  before passing to this function.
 *	@note Device address( applicable only for I2C, bypassed for SPI)
 *  @note Bus interface functions.
 *	@note Delay function
 *  @note With the above data properly set, Chip id will be read from the sensor
 *
 *	@note While changing the parameter of the bme680_t
 *	@note consider the following point:
 *	Changing the reference value of the parameter
 *	will change the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *	(Better case don't change the reference value of the parameter)
 *
 *
 *
 *
 *	@return Either the results of bus communication status or
 *	chip_id fail status
 *	@retval 0 -> Success both bus communication & chip_id
 *	@retval any negative value -> Bus communication failed
 *	-4 -> chip_id corrupted and bus communication ok
 *
 *
*/
enum bme680_return_type bme680_init(struct bme680_t *bme680)
{
	/* used to return the communication result*/
	enum bme680_return_type com_status = BME680_COMM_RES_ERROR;
	u8 data_u8 = BME680_INIT_VALUE;
	/* assign the pointer*/
	if (BME680_SPI_INTERFACE == bme680->interface) {
		/*SPI address 0x45*/
		/* read the chip id*/
		com_status = (enum bme680_return_type)bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_PAGE0_SPI_ID_REG,
						&data_u8,
						BME680_GEN_READ_DATA_LENGTH);
	} else if (BME680_I2C_INTERFACE == bme680->interface) {
		/* read the chip id*/
		com_status = (enum bme680_return_type)bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_PAGE0_I2C_ID_REG,
						&data_u8,
						BME680_GEN_READ_DATA_LENGTH);
	}
	bme680->chip_id = data_u8;

	if (BME680_COMM_RES_OK == com_status) {
		if (BME680_CHIP_ID == bme680->chip_id) {
			/* read the calibration values*/
			com_status = bme680_get_calib_param(bme680);
		} else {
			com_status = BME680_CHIP_ID_ERROR;
		}
	}
	return com_status;
}
/*!
 *	@brief This function is used to retrieve the calibration
 *  data from the image registers of the sensor.
 *
 *	@note Registers 8Ah  to A1h for calibration data 1 to 24
 *        from bit 0 to 7
 *  @note Registers E1h to F0h for calibration data 25 to 40
 *        from bit 0 to 7
 *	@param bme680 structure pointer.
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval any negative value -> Error
 *
 *
*/
static enum bme680_return_type bme680_get_calib_param(struct bme680_t *bme680)
{
	/* used to return the communication result*/
	enum bme680_return_type com_status = BME680_COMM_RES_ERROR;
	/* array of data holding the calibration values*/
	u8 v_data_u8 = BME680_INIT_VALUE;
	u8 a_data_u8[BME680_CALIB_PARAM_SIZE];
	u8 index = BME680_INIT_VALUE;


	for (; index < BME680_CALIB_PARAM_SIZE; index++)
		a_data_u8[index] = BME680_INIT_VALUE;

	/* check the bme680 structure pointer as NULL*/
	if (BME680_NULL_PTR == bme680) {
		com_status = BME680_ERROR_NULL_PTR;
		} else {
		if (BME680_SPI_INTERFACE == bme680->interface) {
			/* memory page switch the SPI address*/
			com_status = bme680_set_memory_page(
					BME680_PAGE0_INTERFACE_SPI, bme680);

		if (BME680_COMM_RES_OK == com_status) {

			/* read the pressure and temperature
			calibration data*/
			com_status = (enum bme680_return_type)
				bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_CALIB_SPI_ADDR_1,
						a_data_u8,
						BME680_CALIB_DATA_LENGTH_GAS);
			/* read the humidity and gas
			calibration data*/
			com_status = (enum bme680_return_type)
					bme680->bme680_bus_read(
					bme680->dev_addr,
					BME680_CALIB_SPI_ADDR_2,
					(a_data_u8 +
					BME680_CALIB_DATA_LENGTH_GAS),
					BME680_CALIB_DATA_LENGTH);
			}
			} else if (BME680_I2C_INTERFACE == bme680->interface) {
				/* read the pressure and temperature
				calibration data*/
				com_status = (enum bme680_return_type)
						bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_CALIB_I2C_ADDR_1,
						a_data_u8,
						BME680_CALIB_DATA_LENGTH_GAS);
				/* read the humidity and gas
				calibration data*/
				com_status = (enum bme680_return_type)
					     bme680->bme680_bus_read(
					     bme680->dev_addr,
					     BME680_CALIB_I2C_ADDR_2,
					    (a_data_u8 +
					    BME680_CALIB_DATA_LENGTH_GAS),
					    BME680_CALIB_DATA_LENGTH);

			} else {
				com_status = BME680_COMM_RES_ERROR;
			}

	if (BME680_COMM_RES_OK == com_status) {
		/*read TPGH calibration*/
		bme680_packing_calib_param(a_data_u8, bme680);

	if (BME680_SPI_INTERFACE == bme680->interface) {
		/* memory page switch the SPI address*/
		com_status = bme680_set_memory_page(BME680_PAGE1_INTERFACE_SPI,
									bme680);
	}

		com_status = (enum bme680_return_type)bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_ADDR_RES_HEAT_RANGE,
						&v_data_u8,
						BME680_GEN_READ_DATA_LENGTH);

		bme680->cal_param.res_heat_range = BME680_GET_REG(v_data_u8,
						BME680_MASK_RES_HEAT_RANGE,
						BME680_SHIFT_RES_HEAT_RANGE);

		com_status = (enum bme680_return_type)bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_ADDR_RES_HEAT_VAL,
						&v_data_u8,
						BME680_GEN_READ_DATA_LENGTH);

		bme680->cal_param.res_heat_val = v_data_u8;

		com_status = (enum bme680_return_type)bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_ADDR_RANGE_SWITCHING_ERR,
						&v_data_u8,
						BME680_GEN_READ_DATA_LENGTH);

		bme680->cal_param.range_switching_error = BME680_GET_REG(
						(s8)v_data_u8,
						(s8)BME680_MASK_RANGE_ERR,
						BME680_SHIFT_RANGE_ERR);
		}

	}
	return com_status;
}

/*!
 * @brief This function is used to write the data to
 *	the given register
 *
 *
 *	@param addr_u8 -> Address of the register
 *	@param data_u8 -> The data to write to the register
 *	@param len_u8 -> No of bytes to write
 *	@param bme680 structure pointer.
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval any negative value -> Error
 *
 *
 */
enum bme680_return_type bme680_write_reg(u8 addr_u8, u8 *data_u8, u8 len_u8,
	struct bme680_t *bme680)
{
	/* used to return the communication result*/
	enum bme680_return_type com_status = BME680_COMM_RES_ERROR;
	/* check the bme680 is NULL pointer */
	if (BME680_NULL_PTR == bme680) {
		com_status = BME680_ERROR_NULL_PTR;
		} else {
		com_status = (enum bme680_return_type)bme680->bme680_bus_write(
							bme680->dev_addr,
							addr_u8,
							data_u8,
							len_u8);
	}
	return com_status;
}
/*!
 * @brief This function is used to reads the data from
 *	the given register
 *
 *
 *	@param addr_u8 -> Address of the register
 *	@param data_u8 -> Pointer to store the
 *  received data from the register
 *	@param len_u8 -> No of bytes to read
 *	@param bme680 structure pointer.
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval any negative value -> Error
 *
 *
 */
enum bme680_return_type bme680_read_reg(u8 addr_u8, u8 *data_u8, u8 len_u8,
	struct bme680_t *bme680)
{
	/* used to return the communication result*/
	enum bme680_return_type com_status = BME680_COMM_RES_ERROR;
	/* check the bme680 is NULL pointer */
	if (BME680_NULL_PTR == bme680) {
		com_status = BME680_ERROR_NULL_PTR;
		} else {
			com_status = (enum bme680_return_type)
				bme680->bme680_bus_read(bme680->dev_addr,
								addr_u8,
								data_u8,
								len_u8);
		}
	return com_status;
}

/*!
 *	@brief This function is used to read the new data0
 *	@note Field-0(new_data_0),
 *	Field-1(new_data_1) and Field-2(new_data_2)
 *	@note Page-1
 *
 *
 *	@param new_data_u8: The value of new data
 *	@param field_u8: The value of field selection for new data
 *   field    |  value
 * -----------|-------------
 *     0      | BME680_FIELD_ZERO
 *     1      | BME680_FIELD_ONE
 *     2      | BME680_FIELD_TWO
 *
 *         field       |   Register
 *  -------------------|------------
 *   BME680_FIELD_ZERO |  0x1D bit 7
 *   BME680_FIELD_ONE  |  0x2E bit 7
 *   BME680_FIELD_TWO  |  0x3F bit 7
 *
 *
 *	@param bme680 structure pointer.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval any negative value -> Error
 *
 *
*/
enum bme680_return_type bme680_get_new_data(u8 *new_data_u8, u8 field_u8,
	struct bme680_t *bme680)
{

	/* used to return the communication result*/
	enum bme680_return_type com_status = BME680_COMM_RES_ERROR;
	/* check the bme680 is NULL pointer */
	if (BME680_NULL_PTR == bme680) {
		com_status = BME680_ERROR_NULL_PTR;
		} else {
			if (BME680_SPI_INTERFACE == bme680->interface) {
				/* memory page switch the SPI address*/
				com_status = bme680_set_memory_page(
						BME680_PAGE1_INTERFACE_SPI,
						bme680);

			}

	if (BME680_I2C_INTERFACE == bme680->interface)
		com_status = BME680_COMM_RES_OK;

	if (BME680_COMM_RES_OK == com_status) {

		switch (field_u8) {
		case BME680_FIELD_ZERO:
			/* read field0 new data zero*/
			com_status = (enum bme680_return_type)
				bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_ADDR_FIELD_0,
						new_data_u8,
						BME680_GEN_READ_DATA_LENGTH);

		break;
		case BME680_FIELD_ONE:
			/* read field1 new data one*/
			com_status = (enum bme680_return_type)
				bme680->bme680_bus_read(
						bme680->dev_addr,
						(BME680_ADDR_FIELD_0 +
						BME680_FIELD_ONE_OFFSET),
						new_data_u8,
						BME680_GEN_READ_DATA_LENGTH);

		break;
		case BME680_FIELD_TWO:
			/* read field2 new data two*/
			com_status =
			(enum bme680_return_type)
				bme680->bme680_bus_read(
						bme680->dev_addr,
						(BME680_ADDR_FIELD_0 +
						BME680_FIELD_TWO_OFFSET),
						new_data_u8,
						BME680_GEN_READ_DATA_LENGTH);

		break;
		default:
			com_status = BME680_COMM_RES_ERROR;
		break;
		}
		if (BME680_COMM_RES_OK == com_status)
			*new_data_u8 = BME680_GET_REG(*new_data_u8,
							BME680_MASK_NEW_DATA,
							BME680_SHIFT_NEW_DATA);

		}
	}
	return com_status;
}

/*!
 *	@brief This function is used to read the uncompensated
 *	sensor data from Field-0, Field-1, Field-2 and page-1
 *
 *	@param uncomp_data:
 *	Pointer to store the value of uncompensated sensor
 *	data of pressure, temperature, humidity and gas
 *
 *	@param field_count : total no of field data which needs
 *	to be read from the sensor
 *
 *	@note:
 *	field_count = 1 : only latest field data out of 3 fields
 *	field_count = 2 : latest and recent field data out of 3 field
 *	field_count = 3 : All 3 latest, recent and old field data
 *
 *	@param sensor_type : Type of sensor
 *	e.g; BME680_PRESSURE,BME680_TEMPERATURE,BME680_HUMIDITY
 *	BME680_GAS,BME680_ALL
 *
 *	@note: if "BME680_SPECIFIC_FIELD_DATA_READ_ENABLED" is not defined in
 *	bme680.h then for any sensor_type function will perform
 *	read operation for BME680_ALL.
 *
 *	@param bme680 structure pointer.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval any negative value -> Error
 *
 *	@note error code is returned when data readout is attempted
 *	in sleep mode or when field_count is not in the below range
 *	it must be 1<= field_count <= 3
 *
*/
enum bme680_return_type bme680_get_uncomp_data(
	struct bme680_uncomp_field_data *uncomp_data, u8 field_count,
	u8 sensor_type, struct bme680_t *bme680)
{
	/* used to return the communication result*/

	enum bme680_return_type com_status = BME680_COMM_RES_ERROR;
	u8 index = BME680_INIT_VALUE;
	u8 a_data_u8[BME680_LEN_ALL_FIELD_SIZE];
	struct bme680_uncomp_field_data temp_sensor_data[BME680_THREE];

	#ifdef BME680_SPECIFIC_FIELD_DATA_READ_ENABLED

	/*Array to store the new_data status of all 3 fields*/
	u8 new_data[BME680_THREE] = {BME680_INIT_VALUE,	BME680_INIT_VALUE,
					BME680_INIT_VALUE};
	#endif
	/*clear the the latest, recent and old field index*/
	bme680->latest_field_index = BME680_INIT_VALUE;
	bme680->recent_field_index = BME680_INIT_VALUE;
	bme680->old_field_index = BME680_INIT_VALUE;

	if ((field_count < BME680_PRESENT_DATA_FIELD
		|| field_count > BME680_ALL_DATA_FIELD)
		|| (BME680_SLEEP_MODE == bme680->last_set_mode)) {
		com_status = BME680_COMM_RES_ERROR;
	} else {
		com_status = BME680_COMM_RES_OK;
	}
	if (BME680_COMM_RES_OK == com_status) {
		#ifndef BME680_SPECIFIC_FIELD_DATA_READ_ENABLED
		sensor_type = BME680_ALL;
		field_count = BME680_ALL_DATA_FIELD;
		#endif

	if (BME680_FORCED_MODE == bme680->last_set_mode) {

		com_status = (enum bme680_return_type)
				bme680->bme680_bus_read(
					bme680->dev_addr,
					BME680_ADDR_FIELD_0, a_data_u8,
					BME680_SINGLE_FIELD_LENGTH);
		field_count = BME680_PRESENT_DATA_FIELD;
	} else {
	#ifdef BME680_SPECIFIC_FIELD_DATA_READ_ENABLED

		/*read status field of all 3 filed and extract the new_data
		flag status.*/
		com_status = bme680_read_status_fields(uncomp_data, a_data_u8,
							new_data, bme680);

		/*get the latest, recent and old field index*/
		bme680_get_latest_recent_old_field_index(uncomp_data, bme680);


		/*By default read latest field data */
		if (BME680_TRUE == new_data[bme680->latest_field_index]) {
				com_status =
				bme680_get_field_specific_uncomp_data(
						bme680->latest_field_index,
						sensor_type,
						a_data_u8,
						bme680);

		}
		if (BME680_PRESENT_AND_PREVIOUS_DATA_FIELD == field_count) {
			/* read recent field data */
			if (BME680_TRUE ==
			new_data[bme680->recent_field_index]) {
				com_status =
				bme680_get_field_specific_uncomp_data(
						bme680->recent_field_index,
						sensor_type,
						a_data_u8,
						bme680);
			}

		} else if (BME680_ALL_DATA_FIELD == field_count) {

			/* read recent field data */
			if (BME680_TRUE ==
			new_data[bme680->recent_field_index]) {
				com_status =
				bme680_get_field_specific_uncomp_data(
						bme680->recent_field_index,
						sensor_type,
						a_data_u8,
						bme680);
			}

			/* read old field data */
			if (BME680_TRUE ==
			new_data[bme680->old_field_index]) {
				com_status =
				bme680_get_field_specific_uncomp_data(
						bme680->old_field_index,
						sensor_type,
						a_data_u8,
						bme680);

			}
		}
	#else
		if (BME680_ALL == sensor_type) {
			/*read uncompensated sensor data of field 0,1,2*/
			com_status = (enum bme680_return_type)
					bme680->bme680_bus_read(
					bme680->dev_addr,
					BME680_ADDR_FIELD_0, a_data_u8,
					BME680_LEN_ALL_FIELD_SIZE);
		}
		(uncomp_data + 0)->status.meas_index =	a_data_u8[1];
		(uncomp_data + 1)->status.meas_index =	a_data_u8[18];
		(uncomp_data + 2)->status.meas_index =	a_data_u8[35];

		/*get the latest, recent and old field index*/
		bme680_get_latest_recent_old_field_index(uncomp_data, bme680);
	#endif
	}

	if (BME680_COMM_RES_OK == com_status) {

			bme680_align_uncomp_data(a_data_u8,
						field_count,
						sensor_type,
						uncomp_data,
						bme680);
		if (BME680_FORCED_MODE != bme680->last_set_mode) {

			for (index = BME680_INIT_VALUE; index <
				BME680_ALL_DATA_FIELD; index++)
				temp_sensor_data[index] =
					*(uncomp_data + index);


			bme680_copy_ordered_sensor_field_data(
				uncomp_data, bme680->latest_field_index,
				bme680->recent_field_index,
				bme680->old_field_index, sensor_type,
				temp_sensor_data);
			}
		}
	}
	return com_status;
}
#ifdef BME680_SPECIFIC_FIELD_DATA_READ_ENABLED
/*!
 *	@brief This function is used to read the uncompensated
 *	data according to sensor type
 *	@note Field-0, Field-1 and Field-2
 *	@note Page-1
 *
 *
 *	@param field_index : index of the field which needs
 *	to be read from the sensor
 *
 *	@param sensor_type : Type of sensor
 *	e.g; BME680_PRESSURE,BME680_TEMPERATURE,BME680_HUMIDITY
 *	BME680_GAS,BME680_ALL
 *
 *	@param a_data_u8 : pointer to store read data.
 *	@param bme680 structure pointer.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval any negative value -> Error
 *
 *
*/
enum bme680_return_type bme680_get_field_specific_uncomp_data(
	u8 field_index, u8 sensor_type, u8 *a_data_u8, struct bme680_t *bme680)
{

	enum bme680_return_type com_status = BME680_COMM_RES_ERROR;


	if (BME680_PRESSURE == sensor_type ||
		BME680_TEMPERATURE == sensor_type ||
		BME680_HUMIDITY == sensor_type) {

		com_status = bme680_Temp_field_specific_uncomp_read(
								field_index,
								a_data_u8,
								bme680);

		switch (sensor_type) {
		case BME680_PRESSURE:
			com_status = bme680_Pressure_field_specific_uncomp_read(
								field_index,
								a_data_u8,
								bme680);

		break;
		case BME680_HUMIDITY:
			com_status = bme680_Humidity_field_specific_uncomp_read(
								field_index,
								a_data_u8,
								bme680);

		break;
		}
	} else if (BME680_GAS == sensor_type) {

		com_status = bme680_Gas_field_specific_uncomp_read(field_index,
								a_data_u8,
								bme680);

	} else if (BME680_ALL == sensor_type) {
		/*read uncompensated sensor data of field 0,1,2*/
		com_status = (enum bme680_return_type)bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_ADDR_FIELD_0,
						a_data_u8,
						BME680_LEN_ALL_FIELD_SIZE);

	}
	return com_status;
}


/*!
 *	@brief This function is used to read the uncompensated
 *	Temperature for specific Field type.
 *	Field-0, Field-1 and Field-2
 *
 *	@param field_index : index of the field which needs
 *	to be read from the sensor
 *
 *	@param a_data_u8 : pointer to store read data.
 *	@param bme680 structure pointer.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval any negative value -> Error
 *
 *
*/
enum bme680_return_type bme680_Temp_field_specific_uncomp_read(
u8 field_index, u8 *a_data_u8, struct bme680_t *bme680)
{
	enum bme680_return_type com_status = BME680_COMM_RES_ERROR;
	/* local buffer length is 5 and it's the maximum */
	u8 temp_data_u8[BME680_THREE];
	u8 count = BME680_INIT_VALUE;

	for (count = BME680_INIT_VALUE; count < BME680_THREE; count++)
		temp_data_u8[count] = BME680_INIT_VALUE;

		/*read uncompensated Temperature of field 0*/
	if (BME680_FIELD_INDEX0 == field_index) {
		/*read the 3 byte of T1 data form 0x22*/
		com_status = (enum bme680_return_type)
						bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_ADDR_FIELD_0_TEMP1,
						temp_data_u8,
						BME680_TEMPERATURE_DATA_LEN);
		/*Assign data to the reserved index
		5,6 & 7 of the input buffer*/
		for (count = BME680_INIT_VALUE;
		count < BME680_TEMPERATURE_DATA_LEN; count++)
			a_data_u8[5 + count] = temp_data_u8[count];

		/*read the 3 byte of T2 data form 0x27*/
		com_status = (enum bme680_return_type)bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_ADDR_FIELD_0_TEMP2,
						temp_data_u8,
						BME680_TEMPERATURE_DATA_LEN);
		/*Assign data to the reserved index
		10,11 & 12 of the input buffer*/
		for (count = BME680_INIT_VALUE;
		count < BME680_TEMPERATURE_DATA_LEN; count++)
			a_data_u8[10 + count] = temp_data_u8[count];

			/*read uncompensated Temperature of field 1*/
		}	else if (BME680_FIELD_INDEX1 == field_index) {

			/*read the 3 byte of T1 data form 0x33*/
			com_status = (enum bme680_return_type)
						bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_ADDR_FIELD_1_TEMP1,
						temp_data_u8,
						BME680_TEMPERATURE_DATA_LEN);
			/*Assign data to the reserved index
			22,23 & 24 of the input buffer*/
			for (count = BME680_INIT_VALUE;
			count < BME680_TEMPERATURE_DATA_LEN; count++)
				a_data_u8[22 + count] = temp_data_u8[count];

			/*read the 3 byte of T2 data form 0x38*/
			com_status = (enum bme680_return_type)
						bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_ADDR_FIELD_1_TEMP2,
						temp_data_u8,
						BME680_TEMPERATURE_DATA_LEN);
			/*Assign data to the reserved index
			27,28 & 29 of the input buffer*/
			for (count = BME680_INIT_VALUE;
				count < BME680_TEMPERATURE_DATA_LEN; count++)
				a_data_u8[27 + count] = temp_data_u8[count];

		/*read uncompensated Temperature of field 2*/
	} else if (BME680_FIELD_INDEX2 == field_index) {

			/*read the 3 byte of T1 data form 0x44*/
			com_status = (enum bme680_return_type)
						bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_ADDR_FIELD_2_TEMP1,
						temp_data_u8,
						BME680_TEMPERATURE_DATA_LEN);
			/*Assign data to the reserved index
			39,40 & 41 of the input buffer*/
			for (count = BME680_INIT_VALUE;
				count < BME680_TEMPERATURE_DATA_LEN; count++)
				a_data_u8[39 + count] = temp_data_u8[count];

			/*read the 3 byte of T2 data form 0x49*/
			com_status = (enum bme680_return_type)
						bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_ADDR_FIELD_2_TEMP2,
						temp_data_u8,
						BME680_TEMPERATURE_DATA_LEN);
			/*Assign data to the reserved index
			44,45 & 46 of the input buffer*/
			for (count = BME680_INIT_VALUE;
				count < BME680_TEMPERATURE_DATA_LEN; count++)
				a_data_u8[44 + count] = temp_data_u8[count];

	}
	return com_status;
}
/*!
 *	@brief This function is used to read the uncompensated
 *	Pressure for specific Field type.
 *	Field-0, Field-1 and Field-2
 *
 *	@param field_index : index of the field which needs
 *	to be read from the sensor
 *
 *
 *	@param a_data_u8 : pointer to store read data.
 *	@param bme680 structure pointer.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval any negative value -> Error
 *
 *
*/
enum bme680_return_type bme680_Pressure_field_specific_uncomp_read(
u8 field_index,	u8 *a_data_u8, struct bme680_t *bme680)
{
	enum bme680_return_type com_status = BME680_COMM_RES_ERROR;
	/* local buffer length is 5 and it's the maximum */
	u8 temp_data_u8[BME680_THREE];
	u8 count = BME680_INIT_VALUE;

	for (count = BME680_INIT_VALUE; count < BME680_THREE; count++)
		temp_data_u8[count] = BME680_INIT_VALUE;

	/*read uncompensated Pressure of field 0*/
	if (BME680_FIELD_INDEX0 == field_index) {
		/*read the 3 byte of P data form 0x1F*/
		com_status = (enum bme680_return_type)
				bme680->bme680_bus_read(
				bme680->dev_addr,
				BME680_ADDR_FIELD_0_PRESS,
				temp_data_u8,
				BME680_PRESSURE_DATA_LEN);
		/*Assign data to the reserved index
		2,3 & 4 of the input buffer*/
		for (count = BME680_INIT_VALUE;
			count < BME680_PRESSURE_DATA_LEN; count++)
				a_data_u8[2 + count] = temp_data_u8[count];

		/*read uncompensated Pressure of field 1*/
	} else if (BME680_FIELD_INDEX1 == field_index) {

		/*read the 3 byte of P data
			form 0x30*/
			com_status = (enum bme680_return_type)
					bme680->bme680_bus_read(
					bme680->dev_addr,
					BME680_ADDR_FIELD_1_PRESS,
					temp_data_u8,
					BME680_PRESSURE_DATA_LEN);
			/*Assign data to the
			reserved index
			19,20 & 21 of the input buffer*/
			for (count = BME680_INIT_VALUE;
				count <	BME680_PRESSURE_DATA_LEN; count++)
				a_data_u8[19 + count] =	temp_data_u8[count];

		/*read uncompensated Pressure of field 2*/
	} else if (BME680_FIELD_INDEX2 == field_index) {

			/*read the 3 byte of P data
			form 0x41*/
			com_status = (enum bme680_return_type)
						bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_ADDR_FIELD_2_PRESS,
						temp_data_u8,
						BME680_PRESSURE_DATA_LEN);
			/*Assign data to the reserved
			index 36,37 & 38 of the input
			buffer*/
			for (count = BME680_INIT_VALUE;
			count <	BME680_PRESSURE_DATA_LEN; count++)
				a_data_u8[36 + count] =	temp_data_u8[count];

	}
	return com_status;
}
/*!
 *	@brief This function is used to read the uncompensated
 *	Humidity for specific Field type.
 *	Field-0, Field-1 and Field-2
 *
 *	@param field_index : index of the field which needs
 *	to be read from the sensor
 *
 *
 *	@param a_data_u8 : pointer to store read data.
 *	@param bme680 structure pointer.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval any negative value -> Error
 *
 *
*/
enum bme680_return_type bme680_Humidity_field_specific_uncomp_read(
u8 field_index,	u8 *a_data_u8, struct bme680_t *bme680)
{
	enum bme680_return_type com_status = BME680_COMM_RES_ERROR;
	/* local buffer length is 5 and it's the maximum */
	u8 temp_data_u8[BME680_TWO];
	u8 count = BME680_INIT_VALUE;

	for (count = BME680_INIT_VALUE; count < BME680_TWO; count++)
		temp_data_u8[count] = BME680_INIT_VALUE;
	/*read uncompensated Humidity of field 0*/
	if (BME680_FIELD_INDEX0 == field_index) {
		/*read the 2 byte of H data form 0x25*/
		com_status = (enum bme680_return_type)
						bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_ADDR_FIELD_0_HUM,
						temp_data_u8,
						BME680_HUMIDITY_DATA_LEN);
		/*Assign data to the reserved index
		8 & 9 of the input buffer*/
		for (count = BME680_INIT_VALUE;
			count < BME680_HUMIDITY_DATA_LEN; count++)
				a_data_u8[8 + count] = temp_data_u8[count];

		/*read uncompensated Humidity of field 1*/
	} else if (BME680_FIELD_INDEX1 == field_index) {

			/*read the 2 byte of H data form 0x36*/
			com_status = (enum bme680_return_type)
						bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_ADDR_FIELD_1_HUM,
						temp_data_u8,
						BME680_HUMIDITY_DATA_LEN);
			/*Assign data to the reserved index
			25 & 26 of the input buffer*/
			for (count = BME680_INIT_VALUE;
				count < BME680_HUMIDITY_DATA_LEN; count++)
				a_data_u8[25 + count] = temp_data_u8[count];

		/*read uncompensated Humidity of field 2*/
	} else if (BME680_FIELD_INDEX2 == field_index) {

			/*read the 2 byte of H data form 0x47*/
			com_status = (enum bme680_return_type)
						bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_ADDR_FIELD_2_HUM,
						temp_data_u8,
						BME680_HUMIDITY_DATA_LEN);
			/*Assign data to the reserved index
			42 & 43 of the input buffer*/
			for (count = BME680_INIT_VALUE;
				count < BME680_HUMIDITY_DATA_LEN; count++)
				a_data_u8[42 + count] = temp_data_u8[count];

	}
	return com_status;
}
/*!
 *	@brief This function is used to read the uncompensated
 *	Gas for specific Field type.
 *	Field-0, Field-1 and Field-2
 *
 *	@param field_index : index of the field which needs
 *	to be read from the sensor
 *
 *
 *	@param a_data_u8 : pointer to store read data.
 *	@param bme680 structure pointer.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval any negative value -> Error
 *
 *
*/
enum bme680_return_type bme680_Gas_field_specific_uncomp_read(
	u8 field_index, u8 *a_data_u8, struct bme680_t *bme680)
{
	enum bme680_return_type com_status = BME680_COMM_RES_ERROR;
	/* local buffer length is 5 and it's the maximum */
	u8 temp_data_u8[BME680_TWO];
	u8 count = BME680_INIT_VALUE;

	for (count = BME680_INIT_VALUE; count < BME680_TWO; count++)
		temp_data_u8[count] = BME680_INIT_VALUE;

	/*read uncompensated Gas of field 0*/
	if (BME680_FIELD_INDEX0 == field_index) {
		/*Default field_0 required*/
		/*read the 2 byte of G data form 0x2A*/
		com_status = (enum bme680_return_type)bme680->bme680_bus_read(
							bme680->dev_addr,
							BME680_ADDR_FIELD_0_GAS,
							temp_data_u8,
							BME680_GAS_DATA_LEN);
		/*Assign data to the reserved index
			13,14 of the input buffer*/
		for (count = BME680_INIT_VALUE;
			count < BME680_GAS_DATA_LEN; count++)
			a_data_u8[13 + count] = temp_data_u8[count];

		/*read uncompensated Gas of field 1*/
	} else if (BME680_FIELD_INDEX1 == field_index) {

			/*read the 2 byte of G data form 0x3B*/
			com_status = (enum bme680_return_type)
						bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_ADDR_FIELD_1_GAS,
						temp_data_u8,
						BME680_GAS_DATA_LEN);
			/*Assign data to the reserved index
			 30,31 of the input buffer*/
			for (count = BME680_INIT_VALUE;
				count < BME680_GAS_DATA_LEN; count++)
				a_data_u8[30 + count] = temp_data_u8[count];

			/*read uncompensated Gas of field 2*/
	} else if (BME680_FIELD_INDEX2 == field_index) {

			/*read the 2 byte of G data form 0x4C*/
			com_status = (enum bme680_return_type)
						bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_ADDR_FIELD_2_GAS,
						temp_data_u8,
						BME680_GAS_DATA_LEN);
			/*Assign data to the reserved index
			47,48 of the input buffer*/
			for (count = BME680_INIT_VALUE;
				count < BME680_GAS_DATA_LEN; count++)
				a_data_u8[47 + count] =	temp_data_u8[count];

	}
	return com_status;
}

#endif




/*!
 *	@brief This function is used to get the
 *	Operational Mode from the sensor in the
 *	register 0x74 bit 0 and 1
 *
 *	@param power_mode_u8 : Pointer to store the received value
 *	of power mode
 *  value     |    mode
 * -----------|------------------
 *	0x00  |	BME680_SLEEP_MODE
 *	0x01  |	BME680_FORCED_MODE
 *	0x02  |	BME680_PARALLEL_MODE
 *	0x03  |	BME680_SEQUENTIAL_MODE
 *
 *	@param bme680 structure pointer.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval any negative value -> Error
 *
 *
*/
enum bme680_return_type bme680_get_power_mode(u8 *power_mode_u8,
	struct bme680_t *bme680)
{
	u8 data_u8 = BME680_INIT_VALUE;
	/* used to return the communication result*/
	enum bme680_return_type com_status = BME680_COMM_RES_ERROR;
	/* check the bme680 is NULL pointer */
	if (BME680_NULL_PTR == bme680) {
		com_status = BME680_ERROR_NULL_PTR;
		} else {
			/* read power mode*/
			com_status = (enum bme680_return_type)
				bme680->bme680_bus_read(bme680->dev_addr,
						BME680_ADDR_OP_MODE,
						&data_u8,
						BME680_GEN_READ_DATA_LENGTH);

			if (BME680_COMM_RES_OK == com_status) {
				*power_mode_u8 = BME680_GET_REG(data_u8,
							BME680_MASK_OP_MODE,
							BME680_SHIFT_OP_MODE);
				/* updating power mode in global structure*/
				if (bme680->last_set_mode != BME680_FORCED_MODE)
					bme680->last_set_mode =	*power_mode_u8;

		}
	}
	return com_status;
}
/*!
 *	@brief This function is used to set the
 *	Operational Mode of the sensor in the
 *	register 0x74 bit 0 and 1
 *
 *	@param power_mode_u8 : The value of power mode
 *  value       |    mode
 * -------------|------------------
 *	0x00    |	BME680_SLEEP_MODE
 *	0x01	|	BME680_FORCED_MODE
 *      0x02	|       BME680_PARALLEL_MODE
 *	0x03    |	BME680_SEQUENTIAL_MODE
 *
 *	@param bme680 structure pointer.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval any negative value -> Error
 *
 *
*/
enum bme680_return_type bme680_set_power_mode(u8 power_mode_u8,
	struct bme680_t *bme680)
{
	u8 data_u8 = BME680_INIT_VALUE;
	/* used to return the communication result*/
	enum bme680_return_type com_status = BME680_COMM_RES_ERROR;
	/* check the bme680 is NULL pointer */
	if (BME680_NULL_PTR == bme680) {
		com_status = BME680_ERROR_NULL_PTR;
		} else {
		/* write power mode*/
		com_status = (enum bme680_return_type)
			bme680->bme680_bus_read(bme680->dev_addr,
						BME680_ADDR_OP_MODE,
						&data_u8,
						BME680_GEN_READ_DATA_LENGTH);
		if (BME680_COMM_RES_OK == com_status) {
			data_u8 = BME680_SET_REG(data_u8, power_mode_u8,
				BME680_MASK_OP_MODE, BME680_SHIFT_OP_MODE);
			com_status = (enum bme680_return_type)
				bme680->bme680_bus_write(bme680->dev_addr,
						BME680_ADDR_OP_MODE,
						&data_u8,
						BME680_GEN_WRITE_DATA_LENGTH);
		}
		/* updating power mode in global structure*/
		if (BME680_COMM_RES_OK == com_status)
			bme680->last_set_mode = power_mode_u8;
	}
	return com_status;
}
/*!
 *	@brief This function is used to set the sensor configuration
 *
 *  @param sens_conf : structure pointer which points to
 *	bme680_sens_conf structure passed by the user.
 *
 *	@param bme680 structure pointer.
 *
 *  @note reference input values from user are below
 *
 *	heatr_ctrl = BME680_HEATR_CTRL_ENABLE;
 *	odr = BME680_ODR_20MS;
 *	run_gas = BME680_RUN_GAS_ENABLE;
 *	nb_conv = 0x01;
 *	osrs_hum = BME680_OSRS_1X;
 *	osrs_pres = BME680_OSRS_1X;
 *	osrs_temp = BME680_OSRS_1X;
 *	filter = BME680_FILTER_COEFF_1;
 *	spi_3w = BME680_SPI_3W_DISABLE;
 *	intr = BME680_SPI_3W_INTR_DISABLE
 *
 *	@note nb_conv parameter is specific to power mode
 *	refer data sheet for detailed info.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval any negative value -> Error
*/
enum bme680_return_type bme680_set_sensor_config(
	struct bme680_sens_conf *sens_conf, struct bme680_t *bme680)
{
	/* used to return the communication result*/
	enum bme680_return_type com_status = BME680_COMM_RES_ERROR;
	u8 data_u8[(BME680_SENS_CONF_LEN*2)-1];
	u8 index;
	/* check the bme680 is NULL pointer */
	if (BME680_NULL_PTR == bme680) {
		com_status = BME680_ERROR_NULL_PTR;
		} else {
		if (BME680_SPI_INTERFACE == bme680->interface) {
			/* memory page switch the SPI address*/
			com_status = bme680_set_memory_page(
			   BME680_PAGE1_INTERFACE_SPI, bme680);
		}
	if (BME680_I2C_INTERFACE == bme680->interface)
		com_status = BME680_COMM_RES_OK;

	if (BME680_COMM_RES_OK == com_status) {

		for (index = 0; index < (BME680_SENS_CONF_LEN * 2) - 2;
			index++)
			data_u8[index] = BME680_INIT_VALUE;
		com_status = (enum bme680_return_type)bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_ADDR_SENSOR_CONFIG,
						data_u8,
						(BME680_SENS_CONF_LEN));

		if (BME680_COMM_RES_OK == com_status) {
			data_u8[BME680_INDEX_CTRL_GAS_0] =
				(sens_conf->heatr_ctrl & 0x01)
				<< BME680_SHIFT_HEATR_CTRL;

			data_u8[BME680_INDEX_CTRL_GAS_1] =
				((sens_conf->odr & 0x08) << BME680_SHIFT_ODR_3)
				| ((sens_conf->run_gas & 0x01) <<
				BME680_SHIFT_RUN_GAS) |
				(sens_conf->nb_conv & 0x0F);

			data_u8[BME680_INDEX_CTRL_HUM] =
				((sens_conf->intr & 0x01) <<
				BME680_SHIFT_SPI_3W_INT) |
				(sens_conf->osrs_hum & 0x07);

			data_u8[BME680_INDEX_CTRL_MEAS] =
				((sens_conf->osrs_pres & 0x07) <<
				BME680_SHIFT_OSRS_PRES) |
				((sens_conf->osrs_temp & 0x07) <<
				BME680_SHIFT_OSRS_TEMP) |
				(data_u8[BME680_INDEX_CTRL_MEAS] & 0x03);

			data_u8[BME680_INDEX_CONFIG] =
				(((sens_conf->odr) & 0x07) <<
				BME680_SHIFT_ODR_2_0) |
				((sens_conf->filter & 0x07) <<
				BME680_SHIFT_FILTER) |
				(sens_conf->spi_3w & 0x01);

#ifndef	__linux__
			bme680_buffer_restruct_burst_write(data_u8,
						0x70,
						BME680_SENS_CONF_LEN,
						(BME680_SENS_CONF_LEN * 2)-1);

			com_status = (enum bme680_return_type)
				bme680->bme680_bus_write(bme680->dev_addr,
						BME680_ADDR_SENSOR_CONFIG,
						data_u8,
						(BME680_SENS_CONF_LEN * 2)-1);
#else
			com_status = (enum bme680_return_type)
				bme680->bme680_bus_write(bme680->dev_addr,
						BME680_ADDR_SENSOR_CONFIG,
						data_u8,
						BME680_SENS_CONF_LEN);
#endif
		}
	}
	}
	return com_status;
}
/*!
 *	@brief This function is used to get the sensor configuration
 *
 *  @param sens_conf : structure pointer which points to
 *  bme680_sens_conf structure passed by the user to store the
 *	received configuration
 *
 *	@param bme680 structure pointer.
 *
 *  @note reference output values from the sensor are below
 *
 *	heatr_ctrl = BME680_HEATR_CTRL_ENABLE;
 *	odr = BME680_ODR_20MS;
 *	run_gas = BME680_RUN_GAS_ENABLE;
 *	nb_conv = 0x01;
 *	osrs_hum = BME680_OSRS_1X;
 *	osrs_pres = BME680_OSRS_1X;
 *	osrs_temp = BME680_OSRS_1X;
 *	filter = BME680_FILTER_COEFF_1;
 *	spi_3w = BME680_SPI_3W_DISABLE;
 *	intr = BME680_SPI_3W_INTR_DISABLE
 *
 *  @note actual settings may differ as configured by user earlier using
 *	bme680_set_sensor_config function.
 *	@note nb_conv parameter is specific to power mode
 *	refer data sheet for detailed info.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval any negative value -> Error

*/
enum bme680_return_type bme680_get_sensor_config(
	struct bme680_sens_conf *sens_conf, struct bme680_t *bme680)
{
	/* used to return the communication result*/
	enum bme680_return_type com_status = BME680_COMM_RES_ERROR;
	u8 data_u8[BME680_SENS_CONF_LEN];
	u8 index;
	/* check the bme680 is NULL pointer */
	if (BME680_NULL_PTR == bme680) {
		com_status = BME680_ERROR_NULL_PTR;
		} else {
		if (BME680_SPI_INTERFACE == bme680->interface) {
			/* memory page switch the SPI address*/
			com_status = bme680_set_memory_page(
			    BME680_PAGE1_INTERFACE_SPI, bme680);

		}

		else if (BME680_I2C_INTERFACE == bme680->interface)
			com_status = BME680_COMM_RES_OK;

	if (BME680_COMM_RES_OK == com_status) {

		for (index = 0; index < BME680_SENS_CONF_LEN ; index++)
			data_u8[index] = BME680_INIT_VALUE;

		com_status = (enum bme680_return_type)
						bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_ADDR_SENSOR_CONFIG,
						data_u8,
						BME680_SENS_CONF_LEN);
		if (BME680_COMM_RES_OK == com_status) {
			sens_conf->heatr_ctrl = (enum bme680_heatr_ctrl)
				BME680_GET_REG(data_u8[BME680_INDEX_CTRL_GAS_0],
						BME680_MASK_HEATR_CTRL,
						BME680_SHIFT_HEATR_CTRL);
			sens_conf->run_gas = (enum bme680_run_gas)
				BME680_GET_REG(data_u8[BME680_INDEX_CTRL_GAS_1],
							BME680_MASK_RUN_GAS,
							BME680_SHIFT_RUN_GAS);
			sens_conf->nb_conv = BME680_GET_REG(
					data_u8[BME680_INDEX_CTRL_GAS_1],
					BME680_MASK_PROF_INDEX,
					BME680_SHIFT_PROF_INDEX);
			sens_conf->odr = (enum bme680_odr)((BME680_GET_REG(
					data_u8[BME680_INDEX_CTRL_GAS_1],
					BME680_MASK_ODR_3,
					BME680_SHIFT_ODR_3)) | (BME680_GET_REG(
					data_u8[BME680_INDEX_CONFIG],
					BME680_MASK_ODR_2_0,
					BME680_SHIFT_ODR_2_0)));
			sens_conf->osrs_hum = (enum bme680_osrs_x)
				BME680_GET_REG(data_u8[BME680_INDEX_CTRL_HUM],
						BME680_MASK_OSRS_HUM,
						BME680_SHIFT_OSRS_HUM);
			sens_conf->intr = (enum bme680_spi_3w_intr)
				BME680_GET_REG(data_u8[BME680_INDEX_CTRL_HUM],
						BME680_MASK_SPI_3W_INT,
						BME680_SHIFT_SPI_3W_INT);
			sens_conf->osrs_pres = (enum bme680_osrs_x)
				BME680_GET_REG(data_u8[BME680_INDEX_CTRL_MEAS],
							BME680_MASK_OSRS_PRES,
							BME680_SHIFT_OSRS_PRES);

			sens_conf->osrs_temp = (enum bme680_osrs_x)
				BME680_GET_REG(data_u8[BME680_INDEX_CTRL_MEAS],
							BME680_MASK_OSRS_TEMP,
							BME680_SHIFT_OSRS_TEMP);
			sens_conf->filter = (enum bme680_filter)BME680_GET_REG(
						data_u8[BME680_INDEX_CONFIG],
						BME680_MASK_FILTER,
						BME680_SHIFT_FILTER);
			sens_conf->spi_3w = (enum bme680_spi_3w)BME680_GET_REG(
						data_u8[BME680_INDEX_CONFIG],
						BME680_MASK_SPI_3W_EN,
						BME680_SHIFT_SPI_3W_EN);
			}
		}
	}
	return com_status;
}


/*!
 *	@brief This function is used for setting gas heater configuration
 *	of the sensor from register 50 to 6E address
 *
 *  @param heatr_conf : structure pointer of Heater configuration
 *	structure
 *
 *	@param bme680 structure pointer.
 *
 *	@note reference input values from user are below
 *
 *  heatr_idacv	- initial heater current for
 *	target heater temperature(optional)
 *	heater_temp	- target temperature (200 to 400 deg cls)
 *	heatr_dur	- heater duration ( 1 to 4032 ms)
 *	heatr_dur_shared - wait time for parallel mode
 *	profile_cnt - user configurable profiles(1 to 10)
 *
 *  @note heatr_dur and heatr_duration share behaviour varies
 *  between parallel and other modes.
 *  kindly refer data-sheet
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval any negative value -> Error
 *
*/
enum bme680_return_type bme680_set_gas_heater_config(
	struct bme680_heater_conf *heatr_conf, u8 power_mode_u8,
	struct bme680_t *bme680)
{

		/* used to return the communication result*/
	enum bme680_return_type com_status = BME680_COMM_RES_ERROR;
	u8 data_u8[(BME680_SENS_HEATR_CONF_LEN << 1)-1] = {0};
	u8 index;
	u8 power_mode = 0;
	/* check the bme680 is NULL pointer */
	if (BME680_NULL_PTR == bme680) {
		com_status = BME680_ERROR_NULL_PTR;
		} else {
		if (BME680_SPI_INTERFACE == bme680->interface) {
			/* memory page switch the SPI address*/
			com_status = bme680_set_memory_page(
				BME680_PAGE1_INTERFACE_SPI, bme680);
		} else if (BME680_I2C_INTERFACE == bme680->interface)
			com_status = BME680_COMM_RES_OK;

	if (BME680_COMM_RES_OK == com_status) {

		for (index = 0; index < heatr_conf->profile_cnt;
		index++) {
			data_u8[index] = heatr_conf->heatr_idacv[index];
		#ifdef FIXED_POINT_COMPENSATION
			data_u8[index + 10] =
				bme680_convert_temperature_to_resistance_int32(
					heatr_conf->heater_temp[index],
					25, bme680);
		#else
			data_u8[index + 10] =
				bme680_convert_temperature_to_resistance_double(
					heatr_conf->heater_temp[index],
					25, bme680);
		#endif
			if (power_mode != BME680_PARALLEL_MODE)
				bme680_scale_to_multiplication_factor(
					&heatr_conf->heatr_dur[index]);
			data_u8[index + 20] = heatr_conf->heatr_dur[index];

		}
		if (BME680_PARALLEL_MODE == power_mode) {
			heatr_conf->heatr_dur_shared  =
				(heatr_conf->heatr_dur_shared * 1000) /
				BME680_GAS_WAIT_STEP_SIZE;
			bme680_scale_to_multiplication_factor(
				&heatr_conf->heatr_dur_shared);
			data_u8[30] = heatr_conf->heatr_dur_shared;

		}
#ifndef	__linux__
		bme680_buffer_restruct_burst_write(data_u8,
					BME680_ADDR_SENS_CONF_START,
					BME680_SENS_HEATR_CONF_LEN,
					(BME680_SENS_HEATR_CONF_LEN << 1)-1);

		com_status = (enum bme680_return_type)bme680->bme680_bus_write(
					bme680->dev_addr,
					BME680_ADDR_SENS_CONF_START,
					data_u8,
					(BME680_SENS_HEATR_CONF_LEN << 1)-1);
#else
		com_status = (enum bme680_return_type)bme680->bme680_bus_write(
					bme680->dev_addr,
					BME680_ADDR_SENS_CONF_START,
					data_u8,
					BME680_SENS_HEATR_CONF_LEN);
#endif
		}
	}
		return com_status;
}

/*!
 *	@brief This function is used to convert the gas duration
 *	according to multiplication factor of the sensor
 *
 *	@note
 *	gas_wait_X(5:0) define 64 timer values
 *	gas_wait_X(7:6) define a multiplication factor
 *
 *	gas_wait_X (7:6) |	multiplication factor
 * ------------------|------------------------------
 *		00			 |	1
 *		01			 |	4
 *		10			 |	16
 *		11			 |	64
 *
 *	@param duration_u16 : pointer to store the
 *	the converted gas duration
 *
 *	@return none
*/

static void bme680_scale_to_multiplication_factor(u16 *duration_u16)
{

	u8 factor = 0;

	while ((*duration_u16) > BME680_GAS_WAIT_MAX_TIMER_VALUE) {
		(*duration_u16) = (*duration_u16) >> 2;
		factor += 1;
	}
	(*duration_u16) = (*duration_u16) + (factor * 64);

}

/*!
 *	@brief This function is used to rearrange the buffer
 *	according to burst write configuration of BME680 sensor
 *
 *	@param arr : interface buffer for SPI or I2C
 *	@param reg_addr : register address to write
 *	@param data_size : no of bytes of data to write
 *	@param arr_size : size of the array
 *
 *	@return none
 *
*/
#ifndef	__linux__
static void bme680_buffer_restruct_burst_write(u8 arr[], u8 reg_addr,
	u8 data_size, u8 arr_size)
{
	s8 index, sub_index  = 0;

	 for (index = 0 ; index < (data_size - 1); index++) {
		arr[arr_size - 1 - sub_index] = arr[data_size - index - 1];
		arr[arr_size - 2 - sub_index] = reg_addr + data_size - 1;
		--reg_addr;
		sub_index += 2;
	}

}
#endif
/*!
 *	@brief This function is used to read the sensor heater
 *	configuration from register 50 to 6E address
 *
 *	@param heatr_conf : structure pointer of Heater
 *	configuration structure
 *
 *	@param bme680 structure pointer.
 *
 *	@note reference output values from the sensor are below
 *
 *  heatr_idacv	- initial heater current for
 *	target heater temperature(optional)
 *	heater_temp	- target temperature (200 to 400 deg cls)
 *	heatr_dur	- heater duration ( 1 to 4032 ms)
 *	heatr_dur_shared - wait time for parallel mode
 *
 *  @note heatr_dur and heatr_duration share behaviour varies
 *  between parallel and other modes.
 *  kindly refer data-sheet
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval any negative value -> Error
 *
 *	@note if error code is -5 = BME680_PROFILE_CNT_ERROR
 *	means profile count is not set by user.
 *	it must be set before calling this function.
 *
*/
enum bme680_return_type bme680_get_gas_heater_config(
	struct bme680_heater_conf *heatr_conf, struct bme680_t *bme680)
{
	/* used to return the communication result*/
	enum bme680_return_type com_status = BME680_COMM_RES_ERROR;
	u8 data_u8[BME680_SENS_HEATR_CONF_LEN];
	u8 index;
	/* check the bme680 is NULL pointer */
	if (BME680_NULL_PTR == bme680) {
		com_status = BME680_ERROR_NULL_PTR;
		} else {
		if (BME680_SPI_INTERFACE == bme680->interface) {
			/* memory page switch the SPI address*/
			com_status = bme680_set_memory_page(
				BME680_PAGE1_INTERFACE_SPI, bme680);
		} else if (BME680_I2C_INTERFACE == bme680->interface)
			com_status = BME680_COMM_RES_OK;

	if (BME680_COMM_RES_OK == com_status) {

		for (index = 0; index < BME680_SENS_HEATR_CONF_LEN; index++)
				data_u8[index] = BME680_INIT_VALUE;

			com_status = (enum bme680_return_type)
				bme680->bme680_bus_read(bme680->dev_addr,
						0x50,
						data_u8,
						BME680_SENS_HEATR_CONF_LEN);
		if (BME680_COMM_RES_OK == com_status) {
			/* 0<= pc <=10 has been modified in order to
			*	avoid warning, "pointless comparison of
			*	unsigned int with zero"
			*/
			if ((heatr_conf->profile_cnt > 0 &&
			heatr_conf->profile_cnt <= BME680_PROFILE_MAX) ||
			(heatr_conf->profile_cnt == 0)) {
				for (index = 0; index <	BME680_PROFILE_MAX;
					index++) {
					heatr_conf->heatr_idacv[index] =
						data_u8[index];
					heatr_conf->heater_temp[index] =
						data_u8[index + 10];
					heatr_conf->heatr_dur[index] =
						data_u8[index + 20];
				}
			} else {
					com_status = BME680_PROFILE_CNT_ERROR;
			}

			heatr_conf->heatr_dur_shared = data_u8[30];
		}
	}
	}
	return com_status;


}

/*!
 *	@brief This function is used to compensate the TPHG raw
 *	values of the sensor in order to convert to meaningful values
 *
 *	@param uncomp_data : Pointer to array of structure which
 *	contains the uncompensated TPHG data
 *	@param comp_data : Pointer to array of structure which
 *	stores the compensated TPHG data
 *	@param field_count : total no of field data which needs to be
 *  compensated.
 *
 *field_count   |       Expected values
 * -------------|-----------------------
 *	 1	|	ONLY_ONE FIELD
 *	 2	|	ONLY_TWO_FIELD
 *	 3	|	THREE_FIELD
 *
 *	@param sensor_type : Type of sensor
 *
 *	sensor_type	|	Expected values
 * ---------------------|-------------------
 *	BME680_ALL	|	TPGH data
 *	BME680_PRESSURE	|	Pressure data
 *	BME680_TEMPERATURE|	Temp data
 *	BME680_HUMIDITY	|	Humidity data
 *	BME680_GAS	|	Gas data
 *
 *	@note: if "BME680_SPECIFIC_FIELD_DATA_READ_ENABLED" is not defined in
 *	bme680.h then for any sensor_type function will perform
 *	read operation for BME680_ALL.
 *	@note : pressure and humidity depends on temperature.
 *
 *	@param bme680 : structure pointer.
 *
 *  @note Undefined behaviour for values other than mentioned above
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval any negative value -> Error
 *
 *	@note error code is returned when data readout is attempted
 *	in sleep mode or when field_count is not in the below range
 *	it must be 1<= field_count <= 3
 *
*/
enum bme680_return_type bme680_compensate_data(
	struct bme680_uncomp_field_data *uncomp_data,
	struct bme680_comp_field_data *comp_data,
	u8 field_count, u8 sensor_type, struct bme680_t *bme680)
{

		/* used to return the communication result*/
	enum bme680_return_type com_status = BME680_COMM_RES_ERROR;
	u8 index;
	u8 max_count = 3;

	if ((field_count < 1 || field_count > 3)
		|| (BME680_SLEEP_MODE == bme680->last_set_mode)) {
		com_status = BME680_COMM_RES_ERROR;
	} else {

	if (BME680_FORCED_MODE == bme680->last_set_mode)
		field_count = BME680_PRESENT_DATA_FIELD;

#ifndef BME680_SPECIFIC_FIELD_DATA_READ_ENABLED
	sensor_type = BME680_ALL;
	field_count = BME680_ALL_DATA_FIELD;
#endif

for (index = 0; ((index < field_count) &&
	(index < max_count)); index++) {

#ifdef FIXED_POINT_COMPENSATION
	switch (sensor_type) {

	case BME680_TEMPERATURE:
		(comp_data + index)->comp_temperature1 =
			bme680_compensate_temperature_int32(
						(uncomp_data+index)->temp_adcv,
						bme680);
	break;
	case	BME680_PRESSURE:
		(comp_data + index)->comp_temperature1 =
			bme680_compensate_temperature_int32(
						(uncomp_data+index)->temp_adcv,
						bme680);
		(comp_data + index)->comp_pressure =
			bme680_compensate_pressure_int32(
						(uncomp_data+index)->pres_adcv,
						bme680);
		break;
	case	BME680_HUMIDITY:
		(comp_data + index)->comp_temperature1 =
			bme680_compensate_temperature_int32(
						(uncomp_data+index)->temp_adcv,
						bme680);
		(comp_data + index)->comp_humidity =
			bme680_compensate_humidity_int32(
						(uncomp_data+index)->hum_adcv,
						bme680);
		break;
	case BME680_GAS:
		(comp_data + index)->comp_gas = bme680_calculate_gas_int32(
				(uncomp_data+index)->gas_res_adcv,
				(uncomp_data + index)->gas_range, bme680);
		break;
	case BME680_ALL:
		(comp_data + index)->comp_temperature1 =
			bme680_compensate_temperature_int32(
						(uncomp_data+index)->temp_adcv,
						bme680);

		(comp_data + index)->comp_pressure =
			bme680_compensate_pressure_int32(
						(uncomp_data+index)->pres_adcv,
						bme680);

		(comp_data + index)->comp_humidity =
			bme680_compensate_humidity_int32(
						(uncomp_data+index)->hum_adcv,
						bme680);

		(comp_data + index)->comp_gas = bme680_calculate_gas_int32(
					(uncomp_data+index)->gas_res_adcv,
					(uncomp_data + index)->gas_range,
					bme680);
	break;
}

#else
	switch (sensor_type) {

	case BME680_TEMPERATURE:
		(comp_data + index)->comp_temperature1 =
			bme680_compensate_temperature_double(
						(uncomp_data+index)->temp_adcv,
						bme680);
	break;
	case	BME680_PRESSURE:
		(comp_data + index)->comp_temperature1 =
			bme680_compensate_temperature_double(
						(uncomp_data+index)->temp_adcv,
						bme680);
		(comp_data + index)->comp_pressure =
			bme680_compensate_pressure_double(
						(uncomp_data+index)->pres_adcv,
						bme680);
	break;
	case	BME680_HUMIDITY:
		(comp_data + index)->comp_temperature1 =
			bme680_compensate_temperature_double(
						(uncomp_data+index)->temp_adcv,
						bme680);
		(comp_data + index)->comp_humidity =
			bme680_compensate_humidity_double(
					(uncomp_data+index)->hum_adcv,
					(comp_data + index)->comp_temperature1,
		bme680);
	break;
	case BME680_GAS:
		(comp_data + index)->comp_gas =
			bme680_compensate_gas_double(
					(uncomp_data+index)->gas_res_adcv,
					(uncomp_data + index)->gas_range,
					bme680);
	break;
	case BME680_ALL:
		(comp_data + index)->comp_temperature1 =
			bme680_compensate_temperature_double(
						(uncomp_data+index)->temp_adcv,
						bme680);
		(comp_data + index)->comp_pressure =
			bme680_compensate_pressure_double(
						(uncomp_data+index)->pres_adcv,
						bme680);
		(comp_data + index)->comp_humidity =
			bme680_compensate_humidity_double(
					(uncomp_data+index)->hum_adcv,
					(comp_data + index)->comp_temperature1,
					bme680);
		(comp_data + index)->comp_gas = bme680_compensate_gas_double(
					(uncomp_data+index)->gas_res_adcv,
					(uncomp_data + index)->gas_range,
					bme680);
	break;
	}


#endif
	}
		com_status = BME680_COMM_RES_OK;
	}
		return com_status;

}
/*!
 *	@brief This function is used to write memory page
 *	from the register 0x73 bit 4
 *
 *
 *	@param memory_page_u8:
 *	The value of memory page
 *	value  | Description
 * --------|--------------
 *   0     | BME680_PAGE0_INTERFACE_SPI
 *   1     | BME680_PAGE1_INTERFACE_SPI
 *
 *
 *	@param bme680 structure pointer.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval any negative value -> Error
 *
 *
*/
static enum bme680_return_type bme680_set_memory_page(u8 memory_page_u8,
	struct bme680_t *bme680)
{
	u8 data_u8 = BME680_INIT_VALUE;
	/* used to return the communication result*/
	enum bme680_return_type com_status = BME680_COMM_RES_ERROR;
	/* check the bme680 is NULL pointer */
	if (BME680_NULL_PTR == bme680) {
		com_status = BME680_ERROR_NULL_PTR;
		} else {
		/* write memory page*/
		com_status = (enum bme680_return_type)bme680->bme680_bus_read(
						bme680->dev_addr,
						BME680_ADDR_SPI_MEM_PAGE,
						&data_u8,
						BME680_GEN_READ_DATA_LENGTH);
		data_u8 = BME680_SET_REG(data_u8, memory_page_u8,
						BME680_MASK_MEM_PAGE,
						BME680_SHIFT_SPI_MEM_PAGE);
		if (BME680_COMM_RES_OK == com_status)
			com_status = (enum bme680_return_type)
				bme680->bme680_bus_write(
						bme680->dev_addr,
						BME680_ADDR_SPI_MEM_PAGE,
						&data_u8,
						BME680_GEN_WRITE_DATA_LENGTH);
	}
	return com_status;
}

/*!
 *	@brief This function is used to Align uncompensated data
 *	from function bme680_get_uncomp_data()
 *
 *	@param a_data_u8 : pointer to buffer
 *	@param field_count : total no of field data which needs
 *	to be compensated
 *	@param sensor_type : Type of sensor
 *
 *	sensor_type	|	Expected values
 * ---------------------|-------------------
 *	BME680_ALL	|	TPGH data
 *	BME680_PRESSURE	|	Pressure data
 *	BME680_TEMPERATURE|	Temp data
 *	BME680_HUMIDITY	|	Humidity data
 *	BME680_GAS	|	Gas data
 *
 *	@note : pressure and humidity depends on temperature.
 *
 *	@param uncomp_data : Pointer to array of structure which
 *	contains the uncompensated TPHG data
 *	@param bme680 structure pointer.
 *	@return - None
 *
 *
*/
void bme680_align_uncomp_data(u8 *a_data_u8, u8 field_count, u8 sensor_type,
	struct bme680_uncomp_field_data *uncomp_data,	struct bme680_t *bme680)
{
	u8 offset = BME680_INIT_VALUE;
	s8 index = BME680_INIT_VALUE;

	if (BME680_FORCED_MODE != bme680->last_set_mode)
		field_count = BME680_ALL_DATA_FIELD;

	for (index = 0; index < field_count; index++) {

		offset = (index * BME680_FIELD_SIZE);

		/*  field_index status */
		(uncomp_data + index)->status.new_data = BME680_GET_REG(
				a_data_u8[FIELD_0_MEAS_STATUS_0 + offset],
				BME680_MASK_NEW_DATA,
				BME680_SHIFT_NEW_DATA);
		(uncomp_data + index)->status.gas_meas_stat = BME680_GET_REG(
				a_data_u8[FIELD_0_MEAS_STATUS_0 + offset],
				BME680_MASK_GAS_MEAS_STAT,
				BME680_SHIFT_GAS_MEAS_STAT);
		(uncomp_data + index)->status.tphg_meas_stat = BME680_GET_REG(
				a_data_u8[FIELD_0_MEAS_STATUS_0 + offset],
				BME680_MASK_TPHG_MEAS_STAT,
				BME680_SHIFT_TPHG_MEAS_STAT);
		(uncomp_data + index)->status.gas_meas_index = BME680_GET_REG(
				a_data_u8[FIELD_0_MEAS_STATUS_0 + offset],
				BME680_MASK_GAS_MEAS_INDEX,
				BME680_SHIFT_GAS_MEAS_INDEX);
		(uncomp_data + index)->status.meas_index =
				a_data_u8[FIELD_0_MEAS_STATUS_1 + offset];
		(uncomp_data + index)->gas_range = BME680_GET_REG(
					a_data_u8[FIELD_0_GAS_RL_LSB + offset],
					BME680_MASK_GAS_RANGE,
					BME680_SHIFT_GAS_RANGE);
		(uncomp_data + index)->status.gas_valid = BME680_GET_REG(
					a_data_u8[FIELD_0_GAS_RL_LSB + offset],
					BME680_MASK_GAS_VALID,
					BME680_SHIFT_GAS_VALID);
		(uncomp_data + index)->status.heatr_stab = BME680_GET_REG(
					a_data_u8[FIELD_0_GAS_RL_LSB + offset],
					BME680_MASK_HEATR_STAB,
					BME680_SHIFT_HEATR_STAB);

		/* uncompensated field zero
		pressure data*/
		bme680_align_sensor_type_uncomp_data(a_data_u8, index, offset,
								sensor_type,
								uncomp_data);

	}

}

/*!
 *	@brief This function is used to get the index of latest, recent and old
 *	field data according to the sub_meas_index parameter.
 *
 *	@param sensor_data : structure pointer of uncompensated array
 *	of 3 structure
 *
 *	@param bme680 structure pointer.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval Any Negative -> Error
 *
 *
*/
void bme680_get_latest_recent_old_field_index(
		struct bme680_uncomp_field_data *sensor_data,
		struct bme680_t *bme680)
{
	/* Array holding the field0, field1 and field2
	temperature, pressure, humidity and gas data*/
	u8 latest = BME680_INIT_VALUE;
	u8 recent = BME680_INIT_VALUE;
	u8 old = BME680_INIT_VALUE;
	u8 index = BME680_INIT_VALUE;
	u8 large_index = BME680_INIT_VALUE;
	u8 max_index = 2;
	u8 meas_index[3];

	for (index = 0; index < 3; index++)
		meas_index[index] = (sensor_data + index)->status.meas_index;

	large_index = bme680_find_largest_index(meas_index);
	latest = large_index;
		if (large_index == max_index) {
			recent = max_index - 1;
			old = max_index - 2;
		} else if (large_index == (max_index - 1)) {
				recent = max_index - 2;
				old = max_index;
		} else {
			recent = max_index;
			old = max_index - 1;
		}

	bme680->latest_field_index = latest;
	bme680->recent_field_index = recent;
	bme680->old_field_index = old;

}
/*!
 *	@brief This function is used to read the status of all 3 fields
 *
 *	@param uncomp_data : Pointer to array of uncompensated data structure.
 *	@param a_data_u8: pointer to store the read status data.
 *	@param new_data: pointer to store the new_data value of given field
 *	@param bme680 structure pointer.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval Any Negative -> Error
 *
 *
*/
enum bme680_return_type bme680_read_status_fields(
	struct bme680_uncomp_field_data *uncomp_data,
	u8 *a_data_u8, u8 *new_data,
	struct bme680_t *bme680)
{
	/* used to return the communication result*/
	enum bme680_return_type com_status = BME680_COMM_RES_ERROR;

	u8 count = BME680_INIT_VALUE;
	/* local buffer length is 5 and it's the maximum */
	u8 temp_data_u8[2] = {BME680_INIT_VALUE, BME680_INIT_VALUE};


		/*read the 2 byte of status form 0x1D - field_0*/
		com_status = (enum bme680_return_type)
			bme680->bme680_bus_read(bme680->dev_addr,
				BME680_ADDR_FIELD_0_STATUS,
				temp_data_u8,
				BME680_STATUS_DATA_LEN);
		/* Assign data to the reserved
			index of the input buffer */
		for (count = BME680_INIT_VALUE;
			count < BME680_STATUS_DATA_LEN; count++)
			a_data_u8[0 + count] = temp_data_u8[count];

			(uncomp_data + 0)->status.meas_index =	a_data_u8[1];

		if (BME680_COMM_RES_OK == com_status)
			new_data[0] = BME680_GET_REG(a_data_u8[0],
							BME680_MASK_NEW_DATA,
							BME680_SHIFT_NEW_DATA);


		/*read the 2 byte of status form 0x2E  - field_1*/
		com_status = (enum bme680_return_type)
			bme680->bme680_bus_read(bme680->dev_addr,
			BME680_ADDR_FIELD_1_STATUS,
			temp_data_u8,
			BME680_STATUS_DATA_LEN);
			/*Assign data to the reserved index
			17 and 18 of the input buffer*/
			for (count = BME680_INIT_VALUE;
			count < BME680_STATUS_DATA_LEN; count++)
				a_data_u8[17 + count] = temp_data_u8[count];

			(uncomp_data + 1)->status.meas_index =	a_data_u8[18];

		if (BME680_COMM_RES_OK == com_status)
				new_data[1] = BME680_GET_REG(a_data_u8[17],
							BME680_MASK_NEW_DATA,
							BME680_SHIFT_NEW_DATA);

			/*read the 2 byte of status form 0x3F  - field_2*/
			com_status = (enum bme680_return_type)
				bme680->bme680_bus_read(bme680->dev_addr,
						BME680_ADDR_FIELD_2_STATUS,
						temp_data_u8,
						BME680_STATUS_DATA_LEN);
			/*Assign data to the reserved index
			34 and 35 of the input buffer*/
			for (count = BME680_INIT_VALUE;
			count < BME680_STATUS_DATA_LEN; count++)
				a_data_u8[34 + count] = temp_data_u8[count];

			(uncomp_data + 2)->status.meas_index =	a_data_u8[35];
			if (BME680_COMM_RES_OK == com_status)
				new_data[2] = BME680_GET_REG(a_data_u8[34],
							BME680_MASK_NEW_DATA,
							BME680_SHIFT_NEW_DATA);
			return com_status;
}

/*!
 *	@brief This function is used to copy the ordered
 *	uncompensated data
 *
 *	@param sensor_data : structure pointer of uncompensated array
 *	of 3 structure
 *	@param latest : total no of field
 *	@param recent : total no of field
 *	@param old : total no of field
 *
 *	@param sensor_type : type of the sensor
 *	@param temp_sensor_data: structure pointer.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval Any Negative -> Error
 *
 *
*/
void bme680_copy_ordered_sensor_field_data(
	struct bme680_uncomp_field_data *sensor_data,
	u8 latest, u8 recent, u8 old, u8 sensor_type,
	struct bme680_uncomp_field_data *temp_sensor_data)
{

	u8 index = BME680_INIT_VALUE;
#ifndef BME680_SPECIFIC_FIELD_DATA_READ_ENABLED
	 sensor_type = BME680_ALL;
#endif

#ifdef BME680_SPECIFIC_FIELD_DATA_READ_ENABLED
	/* copy status of all field */
	for (index = BME680_INIT_VALUE; index < BME680_MAX_FIELD_INDEX;
		index++) {
		if (index == BME680_FIELD_INDEX0)
			sensor_data[index].status =
			temp_sensor_data[latest].status;
		else if (index == BME680_FIELD_INDEX1)
			sensor_data[index].status =
			temp_sensor_data[recent].status;
		else
			sensor_data[index].status =
			temp_sensor_data[old].status;
	}
	if (BME680_PRESSURE == sensor_type || BME680_TEMPERATURE == sensor_type
		|| BME680_HUMIDITY == sensor_type) {

		/* copy temperature data
			by default for Pressure and Humidity
		*/
		for (index = BME680_INIT_VALUE;	index < BME680_MAX_FIELD_INDEX;
			index++) {
			if (index == BME680_FIELD_INDEX0)
				sensor_data[index].temp_adcv =
					temp_sensor_data[latest].temp_adcv;
			else if (index == BME680_FIELD_INDEX1)
				sensor_data[index].temp_adcv =
					temp_sensor_data[recent].temp_adcv;
			else {
				sensor_data[index].temp_adcv =
					temp_sensor_data[old].temp_adcv;
			}
		}
		switch (sensor_type) {
		case BME680_PRESSURE:
		/* copying only pressure data */
		for (index = BME680_INIT_VALUE; index < BME680_MAX_FIELD_INDEX;
		index++) {
			if (index == BME680_FIELD_INDEX0)
				sensor_data[index].pres_adcv =
					temp_sensor_data[latest].pres_adcv;
			else if (index == BME680_FIELD_INDEX1)
				sensor_data[index].pres_adcv =
					temp_sensor_data[recent].pres_adcv;
			else	{
				sensor_data[index].pres_adcv =
					temp_sensor_data[old].pres_adcv;
			}
		}
		break;
		case BME680_HUMIDITY:
		/* copying only humidity data */
		for (index = BME680_INIT_VALUE; index < BME680_MAX_FIELD_INDEX;
		index++) {
			if (index == BME680_FIELD_INDEX0)
				sensor_data[index].hum_adcv =
					temp_sensor_data[latest].hum_adcv;
			else if (index == BME680_FIELD_INDEX1)
				sensor_data[index].hum_adcv =
					temp_sensor_data[recent].hum_adcv;
			else	{
				sensor_data[index].hum_adcv =
					temp_sensor_data[old].hum_adcv;
			}
			}
		break;
		}
	} else if (BME680_GAS == sensor_type) {
			/* copying only gas data */
		for (index = BME680_INIT_VALUE; index < BME680_MAX_FIELD_INDEX;
		index++) {
			if (index == BME680_FIELD_INDEX0)
				sensor_data[index].gas_res_adcv =
					temp_sensor_data[latest].gas_res_adcv;
			else if (index == BME680_FIELD_INDEX1)
				sensor_data[index].gas_res_adcv =
					temp_sensor_data[recent].gas_res_adcv;
			else	{
				sensor_data[index].gas_res_adcv =
					temp_sensor_data[old].gas_res_adcv;
			}
			}
	} else if (BME680_ALL == sensor_type) {
		/* copying T,P,G,& H data */
		for (index = BME680_INIT_VALUE; index < BME680_MAX_FIELD_INDEX;
		index++) {
			if (index == BME680_FIELD_INDEX0)
				*(sensor_data + index) =
					*(temp_sensor_data + latest);
			else if (index == BME680_FIELD_INDEX1)
				*(sensor_data + index) =
					*(temp_sensor_data + recent);
			else
				*(sensor_data + index) =
					*(temp_sensor_data + old);
				}
		}
	#else
	if (BME680_ALL == sensor_type) {
					/* copying T,P,G,& H data */
		for (index = BME680_INIT_VALUE; index < BME680_MAX_FIELD_INDEX;
		index++) {
			if (index == BME680_FIELD_INDEX0)
				*(sensor_data + index) =
					*(temp_sensor_data + latest);
			else if (index == BME680_FIELD_INDEX1)
				*(sensor_data + index) =
					*(temp_sensor_data + recent);
			else
				*(sensor_data + index) =
					*(temp_sensor_data + old);
				}
		}
#endif
}
/*!
 *	@brief This utility function is to return the largest number
 *	index of the input array passed to the function.
 *
 *	@param meas_index: pointer to the integer array
 *
 *	@return index of largest element of the array
 *
 *
*/
static u8 bme680_find_largest_index(u8 *meas_index)
{

	u8 index = BME680_INIT_VALUE;
	u8 temp_index = BME680_INIT_VALUE;

	if (*(meas_index + index) > *(meas_index + (index + 2))) {
		if (*(meas_index + index) > *(meas_index + (index + 1)))
			temp_index = index;
		else
			temp_index = index + 1;
	} else {
		temp_index = index + 2;
	}

	return temp_index;
}
/*!
 *	@brief This function is used to uncompensated data
 *	for the specified sensor type and called from the
 *	function bme680_align_uncomp_data()
 *
 *	@param a_data_u8 : pointer to buffer
 *	@param index : index value
 *	@param offset : offset value
 *	@param uncomp_data : Pointer to array of structure which
 *	contains the uncompensated TPHG data
 *	@param sensor_type : type of sensor which needs
 *	to be compensated.
 *
 *	sensor_type	|	Expected values
 * ---------------------|-------------------
 *	BME680_ALL	|	TPGH data
 *	BME680_PRESSURE	|	Pressure data
 *	BME680_TEMPERATURE|	Temp data
 *	BME680_HUMIDITY	|	Humidity data
 *	BME680_GAS	|	Gas data
 *
 *	@note : pressure and humidity depends on temperature.
 *
 *	@param bme680 structure pointer.
 *
 *	@return - None
 *
 *
*/

static void bme680_align_sensor_type_uncomp_data(u8 *a_data_u8, u8 index,
	u8 offset, u8 sensor_type,
	struct bme680_uncomp_field_data *uncomp_data)
{

	switch (sensor_type) {
	case BME680_PRESSURE:
		/* uncompensated field zero
		temperature data*/
		(uncomp_data + index)->temp_adcv = (u32)(((((u32)a_data_u8[
			BME680_DATA_FRAME_TEMPERATURE1_MSB_DATA + offset]))
			<< 12) | ((((u32)a_data_u8[
			BME680_DATA_FRAME_TEMPERATURE1_LSB_DATA + offset]))
			<< 4) | ((u32)a_data_u8[
			BME680_DATA_FRAME_TEMPERATURE1_XLSB_DATA + offset]
			>> 4));
		/* uncompensated field zero
		pressure data*/
		(uncomp_data + index)->pres_adcv = (u32)(((((u32)a_data_u8[
			BME680_DATA_FRAME_PRESSURE_MSB_DATA + offset])) << 12) |
			((((u32)a_data_u8[BME680_DATA_FRAME_PRESSURE_LSB_DATA
			+ offset])) << 4) | ((u32)a_data_u8[
			BME680_DATA_FRAME_PRESSURE_XLSB_DATA + offset] >> 4));
	break;
	case BME680_TEMPERATURE:
		/* uncompensated field zero
		temperature data*/
		(uncomp_data + index)->temp_adcv = (u32)(((((u32)a_data_u8[
			BME680_DATA_FRAME_TEMPERATURE1_MSB_DATA	+ offset]))
			<< 12) | ((((u32)a_data_u8[
			BME680_DATA_FRAME_TEMPERATURE1_LSB_DATA + offset]))
			<< 4) | ((u32)a_data_u8[
			BME680_DATA_FRAME_TEMPERATURE1_XLSB_DATA + offset]
			>> 4));
	break;
	case BME680_HUMIDITY:
		/* uncompensated field zero
		temperature data*/
		(uncomp_data + index)->temp_adcv = (u32)(((((u32)a_data_u8[
			BME680_DATA_FRAME_TEMPERATURE1_MSB_DATA	+ offset]))
			<< 12) | ((((u32)a_data_u8[
			BME680_DATA_FRAME_TEMPERATURE1_LSB_DATA + offset]))
			<< 4) | ((u32)a_data_u8[
			BME680_DATA_FRAME_TEMPERATURE1_XLSB_DATA + offset]
			>> 4));
		/* uncompensated field zero
		humidity data*/
		(uncomp_data + index)->hum_adcv = (u16)(((((u16)a_data_u8[
			BME680_DATA_FRAME_HUMIDITY_MSB_DATA + offset])) << 8)|
			((a_data_u8[BME680_DATA_FRAME_HUMIDITY_LSB_DATA +
			offset])));
	break;
	case BME680_GAS:
		/* Gas values are updated
		only if gas valid is set */
		/* uncompensated field zero Gas data*/
		if (BME680_TRUE == (uncomp_data + index)->status.gas_valid) {
			(uncomp_data + index)->gas_res_adcv =
				(u16)(((((u16)a_data_u8[
				BME680_DATA_FRAME_GAS_MSB_DATA + offset])) << 2)
				| ((((u16)a_data_u8[
				BME680_DATA_FRAME_GAS_LSB_DATA + offset])
				& BME680_GAS_BIT_MASK) >> 6));
		}
	break;
	case BME680_ALL:
		/* uncompensated field zero
		pressure data*/
		(uncomp_data + index)->pres_adcv = (u32)
			(((((u32)a_data_u8[BME680_DATA_FRAME_PRESSURE_MSB_DATA +
			offset])) << 12) | ((((u32)a_data_u8[
			BME680_DATA_FRAME_PRESSURE_LSB_DATA + offset])) << 4) |
			((u32)a_data_u8[BME680_DATA_FRAME_PRESSURE_XLSB_DATA +
			offset] >> 4));
		/* uncompensated field zero
		temperature data*/
		(uncomp_data + index)->temp_adcv = (u32)(((((u32)a_data_u8[
			BME680_DATA_FRAME_TEMPERATURE1_MSB_DATA + offset]))
			<< 12) | ((((u32)a_data_u8[
			BME680_DATA_FRAME_TEMPERATURE1_LSB_DATA + offset]))
			<< 4) | ((u32)a_data_u8[
			BME680_DATA_FRAME_TEMPERATURE1_XLSB_DATA + offset]
			>> 4));
		/* uncompensated field zero
		humidity data*/
		(uncomp_data + index)->hum_adcv = (u16)(((((u16)a_data_u8[
			BME680_DATA_FRAME_HUMIDITY_MSB_DATA + offset])) << 8)|
			((a_data_u8[BME680_DATA_FRAME_HUMIDITY_LSB_DATA +
			offset])));
		/* Gas values are updated
		only if gas valid is set */
		/* uncompensated field zero Gas data*/
		if (BME680_TRUE == (uncomp_data + index)->status.gas_valid) {
			(uncomp_data + index)->gas_res_adcv =
				(u16)(((((u16)a_data_u8[
				BME680_DATA_FRAME_GAS_MSB_DATA
				+ offset])) << 2) | ((((u16)a_data_u8[
				BME680_DATA_FRAME_GAS_LSB_DATA + offset]) &
				BME680_GAS_BIT_MASK) >> 6));
		}
	break;
	}

}


static void bme680_packing_calib_param(u8 *a_data_u8, struct bme680_t *bme680)
{

	/* read temperature calibration*/
	bme680->cal_param.par_T1 = (u16)((((u16)(a_data_u8[DIG_T1_MSB_REG]))
		<< 8) | a_data_u8[DIG_T1_LSB_REG]);
	bme680->cal_param.par_T2 = (s16)(((((u16)a_data_u8[DIG_T2_MSB_REG]))
		<< 8) | a_data_u8[DIG_T2_LSB_REG]);
	bme680->cal_param.par_T3 = (s8)(a_data_u8[DIG_T3_REG]);

	/* read pressure calibration*/
	bme680->cal_param.par_P1 = (u16)((((u16)(a_data_u8[DIG_P1_MSB_REG])) <<
		8) | a_data_u8[DIG_P1_LSB_REG]);
	bme680->cal_param.par_P2 = (s16)(((((u16)a_data_u8[DIG_P2_MSB_REG]))
		<< 8) | a_data_u8[DIG_P2_LSB_REG]);
	bme680->cal_param.par_P3 = (s8)a_data_u8[DIG_P3_REG];
	bme680->cal_param.par_P4 = (s16)(((((u16)a_data_u8[DIG_P4_MSB_REG]))
		<< 8) | a_data_u8[DIG_P4_LSB_REG]);
	bme680->cal_param.par_P5 = (s16)(((((u16)a_data_u8[DIG_P5_MSB_REG]))
		<< 8) | a_data_u8[DIG_P5_LSB_REG]);
	bme680->cal_param.par_P6 = (s8)(a_data_u8[DIG_P6_REG]);
	bme680->cal_param.par_P7 = (s8)(a_data_u8[DIG_P7_REG]);
	bme680->cal_param.par_P8 = (s16)(((((u16)a_data_u8[DIG_P8_MSB_REG]))
		<< 8) | a_data_u8[DIG_P8_LSB_REG]);
	bme680->cal_param.par_P9 = (s16)(((((u16)a_data_u8[DIG_P9_MSB_REG]))
		<< 8) | a_data_u8[DIG_P9_LSB_REG]);
	bme680->cal_param.par_P10 = (u8)(a_data_u8[DIG_P10_REG]);

	/* read humidity calibration*/
	bme680->cal_param.par_H1 = (u16)(((((u16)a_data_u8[DIG_H1_MSB_REG]))
		<< 4) | (a_data_u8[DIG_H1_LSB_REG] & BME680_BIT_MASK_H1_DATA));
	bme680->cal_param.par_H2 = (u16)(((((u16)a_data_u8[DIG_H2_MSB_REG]))
		<< 4) | ((a_data_u8[DIG_H2_LSB_REG]) >> 4));
	bme680->cal_param.par_H3 = (s8)a_data_u8[DIG_H3_REG];
	bme680->cal_param.par_H4 = (s8) a_data_u8[DIG_H4_REG];
	bme680->cal_param.par_H5 = (s8) a_data_u8[DIG_H5_REG];
	bme680->cal_param.par_H6 = (u8)a_data_u8[DIG_H6_REG];
	bme680->cal_param.par_H7 = (s8)a_data_u8[DIG_H7_REG];

	/* read gas calibration*/
	bme680->cal_param.par_GH1 = (s8)a_data_u8[DIG_GH1_REG];
	bme680->cal_param.par_GH2 = (s16)(((((u16)a_data_u8[DIG_GH2_MSB_REG]))
		<<8) | a_data_u8[DIG_GH2_LSB_REG]);
	bme680->cal_param.par_GH3 = (s8)a_data_u8[DIG_GH3_REG];

}

