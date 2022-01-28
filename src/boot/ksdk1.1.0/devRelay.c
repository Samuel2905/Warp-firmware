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
	kRelayPin	= GPIO_MAKE_PIN(HW_GPIOB, 6), // PTB2 doesn't work
	kRelayLEDPin	= GPIO_MAKE_PIN(HW_GPIOB, 7),
	kLEDErrors= GPIO_MAKE_PIN(HW_GPIOB, 0),
	kLEDHighWater= GPIO_MAKE_PIN(HW_GPIOB, 5),
};

void initOutputPins(void) {
	PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);
}

void TurnOnRelay(void) {
  GPIO_DRV_SetPinOutput(kRelayPin);
	GPIO_DRV_SetPinOutput(kRelayLEDPin);
  // warpPrint("Relay On\n");
}

void TurnOffRelay(void) {
  GPIO_DRV_ClearPinOutput(kRelayPin);
	GPIO_DRV_ClearPinOutput(kRelayLEDPin);
  // warpPrint("Relay Off\n");
}

void ErrorsOn(void) {
  GPIO_DRV_SetPinOutput(kLEDErrors);
}

void HighWaterOn(void) {
  GPIO_DRV_SetPinOutput(kLEDHighWater);
}

void ErrorsOff(void) {
  GPIO_DRV_ClearPinOutput(kLEDErrors);
}

void HighWaterOff(void) {
  GPIO_DRV_ClearPinOutput(kLEDHighWater);
}
