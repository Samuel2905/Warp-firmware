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



void initADC() {
  ADC16_DRV_Init( uint32_t instance, adc16_user_config_t *userConfigPtr);
}
