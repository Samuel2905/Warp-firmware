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
	kRelayPin	= GPIO_MAKE_PIN(HW_GPIOB, 1),
};

void initRelay(void) {
	PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortMuxAsGpio);
}
void TurnOnRelay(void) {
  GPIO_DRV_SetPinOutput(kRelayPin);
  warpPrint("Relay On\n");
}

void TurnOffRelay(void) {
  GPIO_DRV_ClearPinOutput(kRelayPin);
  warpPrint("Relay Off\n");
}
