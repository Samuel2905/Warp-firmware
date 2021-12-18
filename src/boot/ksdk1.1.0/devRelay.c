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

enum
{
	kRelayPin1	= GPIO_MAKE_PIN(HW_GPIOB, 1),
  kRelayPin2	= GPIO_MAKE_PIN(HW_GPIOB, 2),
  kRelayPin5	= GPIO_MAKE_PIN(HW_GPIOB, 5),
  kRelayPin6	= GPIO_MAKE_PIN(HW_GPIOB, 6),
  kRelayPin7	= GPIO_MAKE_PIN(HW_GPIOB, 7),
  kRelayPin11	= GPIO_MAKE_PIN(HW_GPIOB, 11),
};

void TurnOnRelay(void) {
  GPIO_DRV_SetPinOutput(kRelayPin1);
  GPIO_DRV_SetPinOutput(kRelayPin2);
  GPIO_DRV_SetPinOutput(kRelayPin5);
  GPIO_DRV_SetPinOutput(kRelayPin6);
  GPIO_DRV_SetPinOutput(kRelayPin7);
  GPIO_DRV_SetPinOutput(kRelayPin11);
  warpPrint("Relay On\n");
  OSA_TimeDelay(1000);
}

void TurnOffRelay(void) {
  GPIO_DRV_ClearPinOutput(kRelayPin1);
  warpPrint("Relay Off\n");
}
