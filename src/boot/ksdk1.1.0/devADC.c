// Page 57 of kinetis SDK Manual for ADC16 peripheral driver
// One time trigger mode?
// Configure/ Calibrate and then initialise
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
#include "fsl_adc16_driver.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

adc16_user_config_t MyAdcUserConfig;
adc16_chn_config_t MyChnConfig;
int32_t MyAdcValue;
int32_t MyAdcDec;
float ADCfloat;
int32_t MyAdcVol;
uint32_t i;

uint32_t instance = 0;
uint32_t chnGroup = 0;
uint8_t chn = 8;        //PTB1 is ADC0_SE8
uint8_t reps = 10;
float ADC_Val = 0;
float height = 0;
uint16_t height_mm = 0;

void initADC(void) {
  // Initialize the ADC converter. //
  ADC16_DRV_StructInitUserConfigDefault(&MyAdcUserConfig);
  ADC16_DRV_Init(instance, &MyAdcUserConfig);

  // Configuration for ADC channel. //
  MyChnConfig.chnNum = chn;
  MyChnConfig.diffEnable= false;
  MyChnConfig.intEnable = false;
}

uint16_t readADC(void) {
  // Trigger the conversion with indicated channel’s configuration.
  ADC16_DRV_ConfigConvChn(instance, chnGroup, &MyChnConfig);

  // Wait for the conversion to be done.
  ADC16_DRV_WaitConvDone(instance, chnGroup);

  // Fetch the conversion value and format it.
  MyAdcValue = ADC16_DRV_GetConvValueRAW(instance, chnGroup);
  //warpPrint("ADC Hex Value: 0x%X\n", MyAdcValue);
  MyAdcDec = ADC16_DRV_ConvRAWData(MyAdcValue, false, kAdcResolutionBitOfSingleEndAs12);
  //warpPrint("ADC dec Value: %ld\n", MyAdcDec);
  return MyAdcDec;
}

uint16_t Voltage_ADC(uint16_t raw) {
  // Convert raw ADC value into voltage
  ADCfloat = raw * 2970 / 4095;
  MyAdcVol = (int)ADCfloat;
  //warpPrint("ADC Voltage: %ld mV\n", MyAdcVol);
  return MyAdcVol;
}

uint16_t level(void) {
  // returns the water level
  // averages several measurements to account for ripples on pond
  // uses experimental calibration values
  // voltage is invesely proportional to water level
  ADC_Val = 0;
	for (uint8_t n = 0; n<reps; n+=1)
	{
		ADC_Val += readADC();
    OSA_TimeDelay(1000);
	}
	ADC_Val /= reps;
  height = (-1393720 / ADC_Val) + 716;
  height_mm = (int)height;
	return height_mm;
}
