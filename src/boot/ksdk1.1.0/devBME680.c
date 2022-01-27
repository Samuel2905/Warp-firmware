/*
	Authored 2016-2018. Phillip Stanley-Marbell. Additional contributors,
	2018-onwards, see git log.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdlib.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"


extern volatile WarpI2CDeviceState	deviceBME680State;
extern volatile uint8_t			deviceBME680CalibrationValues[];
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;


void
initBME680(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceBME680State.i2cAddress			= i2cAddress;
	deviceBME680State.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	return;
}

WarpStatus
writeSensorRegisterBME680(uint8_t deviceRegister, uint8_t payload)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	status;

	if (deviceRegister > 0xFF)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
	{
		.address = deviceBME680State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;


	warpScaleSupplyVoltage(deviceBME680State.operatingVoltageMillivolts);
	warpEnableI2Cpins();
	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							1,
							gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
readSensorRegisterBME680(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);

	/*
	 *	We only check to see if it is past the config registers.
	 *
	 *	TODO: We should eventually numerate all the valid register addresses
	 *	(configuration, control, and calibration) here.
	 */
	if (deviceRegister > kWarpSensorConfigurationRegisterBME680CalibrationRegion2End)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
	{
		.address = deviceBME680State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	cmdBuf[0] = deviceRegister;

	warpScaleSupplyVoltage(deviceBME680State.operatingVoltageMillivolts);
	warpEnableI2Cpins();
	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceBME680State.i2cBuffer,
							1,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}


WarpStatus
configureSensorBME680(uint8_t payloadCtrl_Hum, uint8_t payloadCtrl_Meas, uint8_t payloadGas_0)
{
	uint8_t		reg, index = 0;
	// uint8_t i = 0;
	uint8_t		T1_msb, T1_lsb, T2_msb, T2_lsb, T3_all;
	int16_t	T1_all, T2_all;
	WarpStatus	status1, status2, status3, status4 = 0;
	WarpStatus  status_x;

	warpScaleSupplyVoltage(deviceBME680State.operatingVoltageMillivolts);
	status1 = writeSensorRegisterBME680(kWarpSensorConfigurationRegisterBME680Ctrl_Hum,
							payloadCtrl_Hum);

	status2 = writeSensorRegisterBME680(kWarpSensorConfigurationRegisterBME680Ctrl_Meas,
							payloadCtrl_Meas);

	status3 = writeSensorRegisterBME680(kWarpSensorConfigurationRegisterBME680Ctrl_Gas_0,
							payloadGas_0);

	/*
	 *	Read the calibration registers
	 */
	for (	reg = kWarpSensorConfigurationRegisterBME680CalibrationRegion1Start;
		reg < kWarpSensorConfigurationRegisterBME680CalibrationRegion1End;
		reg++)
	{
		status4 |= readSensorRegisterBME680(reg, 1 /* numberOfBytes */);
		deviceBME680CalibrationValues[index++] = deviceBME680State.i2cBuffer[0];
	}

	for (	reg = kWarpSensorConfigurationRegisterBME680CalibrationRegion2Start;
		reg < kWarpSensorConfigurationRegisterBME680CalibrationRegion2End;
		reg++)
	{
		status4 |= readSensorRegisterBME680(reg, 1 /* numberOfBytes */);
		deviceBME680CalibrationValues[index++] = deviceBME680State.i2cBuffer[0];
	}
	/*
	for (i = 0; i < index; i++) {
		warpPrint("Calib Data %u: %x\n", i, deviceBME680CalibrationValues[i] );
	}
	*/
	status_x = readSensorRegisterBME680(0x8A, 1 /* numberOfBytes */);
	T2_lsb = deviceBME680State.i2cBuffer[0];
	status_x = readSensorRegisterBME680(0x8B, 1 /* numberOfBytes */);
	T2_msb = deviceBME680State.i2cBuffer[0];
	status_x = readSensorRegisterBME680(0x8C, 1 /* numberOfBytes */);
	T3_all = deviceBME680State.i2cBuffer[0];
	status_x = readSensorRegisterBME680(0xE9, 1 /* numberOfBytes */);
	T1_lsb = deviceBME680State.i2cBuffer[0];
	status_x = readSensorRegisterBME680(0xEA, 1 /* numberOfBytes */);
	T1_msb = deviceBME680State.i2cBuffer[0];

	if (status_x != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}

	T1_all = ((T1_msb & 0xFF) << 8) | (T1_lsb);
	T2_all = ((T2_msb & 0xFF) << 8) | (T2_lsb);

	warpPrint("T1: %d\n", T1_all);
	warpPrint("T2: %d\n", T2_all);
	warpPrint("T3: %u\n", T3_all);

	return (status1 | status2 | status3 | status4);
}


uint32_t
printSensorDataBME680(bool hexModeFlag)
{
	// Function modified to return the raw temperature
	// And not print anything

	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	uint16_t	readSensorRegisterValueXLSB;
	uint32_t	unsignedRawAdcValue;
	uint32_t	raw_temperature = 0;
	WarpStatus	triggerStatus, i2cReadStatusMSB, i2cReadStatusLSB, i2cReadStatusXLSB;


	warpScaleSupplyVoltage(deviceBME680State.operatingVoltageMillivolts);

	/*
	 *	First, trigger a measurement
	 */
	triggerStatus = writeSensorRegisterBME680(kWarpSensorConfigurationRegisterBME680Ctrl_Meas,
							0b00100101);

	i2cReadStatusMSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680press_msb, 1);
	readSensorRegisterValueMSB = deviceBME680State.i2cBuffer[0];
	i2cReadStatusLSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680press_lsb, 1);
	readSensorRegisterValueLSB = deviceBME680State.i2cBuffer[0];
	i2cReadStatusXLSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680press_xlsb, 1);
	readSensorRegisterValueXLSB = deviceBME680State.i2cBuffer[0];
	unsignedRawAdcValue =
			((readSensorRegisterValueMSB & 0xFF)  << 12) |
			((readSensorRegisterValueLSB & 0xFF)  << 4)  |
			((readSensorRegisterValueXLSB & 0xF0) >> 4);

	if ((triggerStatus != kWarpStatusOK) || (i2cReadStatusMSB != kWarpStatusOK) || (i2cReadStatusLSB != kWarpStatusOK) || (i2cReadStatusXLSB != kWarpStatusOK))
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB, readSensorRegisterValueXLSB);
		}
		else
		{
			// warpPrint(" %u,", unsignedRawAdcValue);
		}
	}

	i2cReadStatusMSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680temp_msb, 1);
	readSensorRegisterValueMSB = deviceBME680State.i2cBuffer[0];
	i2cReadStatusLSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680temp_lsb, 1);
	readSensorRegisterValueLSB = deviceBME680State.i2cBuffer[0];
	i2cReadStatusXLSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680temp_xlsb, 1);
	readSensorRegisterValueXLSB = deviceBME680State.i2cBuffer[0];
	unsignedRawAdcValue =
			((readSensorRegisterValueMSB & 0xFF)  << 12) |
			((readSensorRegisterValueLSB & 0xFF)  << 4)  |
			((readSensorRegisterValueXLSB & 0xF0) >> 4);
	if ((triggerStatus != kWarpStatusOK) || (i2cReadStatusMSB != kWarpStatusOK) || (i2cReadStatusLSB != kWarpStatusOK) || (i2cReadStatusXLSB != kWarpStatusOK))
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB, readSensorRegisterValueXLSB);
		}
		else
		{
			// warpPrint(" %u,", unsignedRawAdcValue);
			raw_temperature = unsignedRawAdcValue;
		}
	}

	i2cReadStatusMSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680hum_msb, 1);
	readSensorRegisterValueMSB = deviceBME680State.i2cBuffer[0];
	i2cReadStatusLSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680hum_lsb, 1);
	readSensorRegisterValueLSB = deviceBME680State.i2cBuffer[0];
	unsignedRawAdcValue = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);
	if ((triggerStatus != kWarpStatusOK) || (i2cReadStatusMSB != kWarpStatusOK) || (i2cReadStatusLSB != kWarpStatusOK))
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			// warpPrint(" %u,", unsignedRawAdcValue);
		}
	}
	return raw_temperature;
}

int16_t
calc_temperature(uint32_t temp_adc)
{
	int64_t var1;
	int64_t var2;
	int64_t var3;
	int32_t T_fine;
	int16_t calc_temp;
	int16_t	T1_all, T2_all;
	uint8_t	T3_all;

	// Calibration values
	// Calculated in configure sensor, but are now hard coded for simplicity
	// As they don't (assuming the same sensor is used)

	T1_all = 25883;
	T2_all = 26584;
	T3_all = 3;

	// Calculation taken from below repository, although it doesn't seem to give the correct answer
	// https://github.com/BoschSensortec/BME680_driver/blob/757e1f155c13bcdd34403579bd246b27f2963bf4/bme680.c#L867

	var1 = ((int32_t) temp_adc >> 3) - ((int32_t) T1_all << 1);
	var2 = (var1 * (int32_t) T2_all) >> 11;
	var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
	var3 = ((var3) * ((int32_t) T3_all << 4)) >> 14;
	T_fine = (int32_t) (var2 + var3);
	calc_temp = (int16_t) (((T_fine * 5) + 128) >> 8);
	return calc_temp;
}
