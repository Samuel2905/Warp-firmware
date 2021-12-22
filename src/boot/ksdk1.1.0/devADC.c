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
int32_t MyAdcVol;
uint32_t i;

uint32_t instance = 0;
uint32_t chnGroup = 0;
uint8_t chn = 8;        //PTB1 is ADC0_SE8

void initADC(void) {
  // PORT_HAL_SetMuxMode(PORTB_BASE, 1u, kPortMuxAlt0);

  // Initialize the ADC converter. //
  ADC16_DRV_StructInitUserConfigDefault(&MyAdcUserConfig);
  ADC16_DRV_Init(instance, &MyAdcUserConfig);

  // Configuration for ADC channel. //
  MyChnConfig.chnNum = chn;
  MyChnConfig.diffEnable= false;
  MyChnConfig.intEnable = false;
  warpPrint("ADC initialised");
}

void readADC(void) {
  // Trigger the conversion with indicated channel’s configuration. //
  ADC16_DRV_ConfigConvChn(instance, chnGroup, &MyChnConfig);

  // Wait for the conversion to be done. //
  ADC16_DRV_WaitConvDone(instance, chnGroup);

  // Fetch the conversion value and format it. //
  MyAdcValue = ADC16_DRV_GetConvValueRAW(instance, chnGroup);
  warpPrint("ADC Hex Value: 0x%X\n", MyAdcValue);
  MyAdcDec = ADC16_DRV_ConvRAWData(MyAdcValue, false, kAdcResolutionBitOfSingleEndAs12)
  warpPrint("ADC dec Value: %ld\n", MyAdcDec);
  MyAdcVol = myAdcDec * 2970 / 4095
  warpPrint("ADC Voltage: %ld mV\n", MyAdcDec);
}
