/*
	Authored 2016-2018. Phillip Stanley-Marbell.

	Additional contributions, 2018 onwards: See git blame.

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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

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
#include "fsl_lpuart_driver.h"
#include "fsl_adc16_driver.h"
#include "glaux.h"
#include "warp.h"
#include "errstrs.h"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"


#define							kWarpConstantStringI2cFailure		"\rI2C failed, reg 0x%02x, code %d\n"
#define							kWarpConstantStringErrorInvalidVoltage	"\rInvalid supply voltage [%d] mV!"
#define							kWarpConstantStringErrorSanity		"\rSanity check failed!"



#include "devRelay.h"
#include "devADC.h"

#if (WARP_BUILD_ENABLE_DEVBME680)
	#include "devBME680.h"
	volatile WarpI2CDeviceState			deviceBME680State;
	volatile uint8_t				deviceBME680CalibrationValues[kWarpSizesBME680CalibrationValuesCount];
#endif

#if (WARP_BUILD_ENABLE_DEVRV8803C7)
	#include "devRV8803C7.h"
	volatile WarpI2CDeviceState			deviceRV8803C7State;
#endif


volatile i2c_master_state_t				i2cMasterState;
volatile spi_master_state_t				spiMasterState;
volatile spi_master_user_config_t			spiUserConfig;
volatile lpuart_user_config_t				lpuartUserConfig;
volatile lpuart_state_t					lpuartState;


volatile bool						gWarpBooted				= false;
volatile uint32_t					gWarpI2cBaudRateKbps			= kWarpDefaultI2cBaudRateKbps;
volatile uint32_t					gWarpUartBaudRateBps			= kWarpDefaultUartBaudRateBps;
volatile uint32_t					gWarpSpiBaudRateKbps			= kWarpDefaultSpiBaudRateKbps;
volatile uint32_t					gWarpSleeptimeSeconds			= kWarpDefaultSleeptimeSeconds;
volatile WarpModeMask					gWarpMode				= kWarpModeDisableAdcOnSleep;
volatile uint32_t					gWarpI2cTimeoutMilliseconds		= kWarpDefaultI2cTimeoutMilliseconds;
volatile uint32_t					gWarpSpiTimeoutMicroseconds		= kWarpDefaultSpiTimeoutMicroseconds;
volatile uint32_t					gWarpUartTimeoutMilliseconds		= kWarpDefaultUartTimeoutMilliseconds;
volatile uint32_t					gWarpMenuPrintDelayMilliseconds		= kWarpDefaultMenuPrintDelayMilliseconds;
volatile uint32_t					gWarpSupplySettlingDelayMilliseconds	= kWarpDefaultSupplySettlingDelayMilliseconds;
volatile uint16_t					gWarpCurrentSupplyVoltage		= kWarpDefaultSupplyVoltageMillivolts;
char							gWarpPrintBuffer[kWarpDefaultPrintBufferSizeBytes];

/*
 *	Since only one SPI transaction is ongoing at a time in our implementation
 */
uint8_t							gWarpSpiCommonSourceBuffer[kWarpMemoryCommonSpiBufferBytes];
uint8_t							gWarpSpiCommonSinkBuffer[kWarpMemoryCommonSpiBufferBytes];

static void						sleepUntilReset(void);
static void						lowPowerPinStates(void);
static void						disableTPS62740(void);
static void						enableTPS62740(uint16_t voltageMillivolts);
static void						setTPS62740CommonControlLines(uint16_t voltageMillivolts);
static void						dumpProcessorState(void);
static void						repeatRegisterReadForDeviceAndAddress(WarpSensorDevice warpSensorDevice, uint8_t baseAddress,
								bool autoIncrement, int chunkReadsPerAddress, bool chatty,
								int spinDelay, int repetitionsPerAddress, uint16_t sssupplyMillivolts,
								uint16_t adaptiveSssupplyMaxMillivolts, uint8_t referenceByte);
static int						char2int(int character);
static void						activateAllLowPowerSensorModes(bool verbose);
static void						powerupAllSensors(void);
static uint8_t						readHexByte(void);
static int						read4digits(void);
static void						printAllSensors(bool printHeadersAndCalibration, bool hexModeFlag, int menuDelayBetweenEachRun, bool loopForever);

/*
 *	TODO: change the following to take byte arrays
 */
WarpStatus						writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte);
WarpStatus						writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength);


void							warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState);



/*
 *	Derived from KSDK power_manager_demo.c BEGIN>>>
 */
clock_manager_error_code_t clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData);

/*
 *	static clock callback table.
 */
clock_manager_callback_user_config_t		clockManagerCallbackUserlevelStructure =
									{
										.callback	= clockManagerCallbackRoutine,
										.callbackType	= kClockManagerCallbackBeforeAfter,
										.callbackData	= NULL
									};

static clock_manager_callback_user_config_t *	clockCallbackTable[] =
									{
										&clockManagerCallbackUserlevelStructure
									};

clock_manager_error_code_t
clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData)
{
	clock_manager_error_code_t result = kClockManagerSuccess;

	switch (notify->notifyType)
	{
		case kClockManagerNotifyBefore:
			break;
		case kClockManagerNotifyRecover:
		case kClockManagerNotifyAfter:
			break;
		default:
			result = kClockManagerError;
		break;
	}

	return result;
}


/*
 *	Override the RTC IRQ handler
 */
void
RTC_IRQHandler(void)
{
	if (RTC_DRV_IsAlarmPending(0))
	{
		RTC_DRV_SetAlarmIntCmd(0, false);
	}
}

/*
 *	Override the RTC Second IRQ handler
 */
void
RTC_Seconds_IRQHandler(void)
{
	gWarpSleeptimeSeconds++;
}

/*
 *	LLW_IRQHandler override. Since FRDM_KL03Z48M is not defined,
 *	according to power_manager_demo.c, what we need is LLW_IRQHandler.
 *	However, elsewhere in the power_manager_demo.c, the code assumes
 *	FRDM_KL03Z48M _is_ defined (e.g., we need to use LLWU_IRQn, not
 *	LLW_IRQn). Looking through the code base, we see in
 *
 *		ksdk1.1.0/platform/startup/MKL03Z4/gcc/startup_MKL03Z4.S
 *
 *	that the startup initialization assembly requires LLWU_IRQHandler,
 *	not LLW_IRQHandler. See power_manager_demo.c, circa line 216, if
 *	you want to find out more about this dicsussion.
 */
void
LLWU_IRQHandler(void)
{
	/*
	 *	BOARD_* defines are defined in warp.h
	 */
	LLWU_HAL_ClearExternalPinWakeupFlag(LLWU_BASE, (llwu_wakeup_pin_t)BOARD_SW_LLWU_EXT_PIN);
}

/*
 *	IRQ handler for the interrupt from RTC, which we wire up
 *	to PTA0/IRQ0/LLWU_P7 in Glaux. BOARD_SW_LLWU_IRQ_HANDLER
 *	is a synonym for PORTA_IRQHandler.
 */
void
BOARD_SW_LLWU_IRQ_HANDLER(void)
{
	/*
	 *	BOARD_* defines are defined in warp.h
	 */
	PORT_HAL_ClearPortIntFlag(BOARD_SW_LLWU_BASE);
}

/*
 *	Power manager user callback
 */
power_manager_error_code_t
callback0(power_manager_notify_struct_t *  notify, power_manager_callback_data_t *  dataPtr)
{
	WarpPowerManagerCallbackStructure *		callbackUserData = (WarpPowerManagerCallbackStructure *) dataPtr;
	power_manager_error_code_t			status = kPowerManagerError;

	switch (notify->notifyType)
	{
		case kPowerManagerNotifyBefore:
			status = kPowerManagerSuccess;
			break;
		case kPowerManagerNotifyAfter:
			status = kPowerManagerSuccess;
			break;
		default:
			callbackUserData->errorCount++;
			break;
	}

	return status;
}
/*
 *	Derived from KSDK power_manager_demo.c <<END
 */



void
sleepUntilReset(void)
{
	while (1)
	{
		warpLowPowerSecondsSleep(1, false /* forceAllPinsIntoLowPowerState */);
		warpLowPowerSecondsSleep(60, true /* forceAllPinsIntoLowPowerState */);
	}
}


void
enableLPUARTpins(void)
{
	/*
	 *	Enable UART CLOCK
	 */
	CLOCK_SYS_EnableLpuartClock(0);

	/*
	 *	Set UART pin association. See, e.g., page 99 in
	 *
	 *		https://www.nxp.com/docs/en/reference-manual/KL03P24M48SF0RM.pdf
	 *
	 *	Setup:
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX for UART TX
	 *		PTB4/kWarpPinI2C0_SCL_UART_RX for UART RX

//TODO: we don't use hw flow control so don't need RTS/CTS
 *		PTA6/kWarpPinSPI_MISO_UART_RTS for UART RTS
 *		PTA7/kWarpPinSPI_MOSI_UART_CTS for UART CTS
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt3);

//TODO: we don't use hw flow control so don't need RTS/CTS
//	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
//	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);
//	GPIO_DRV_SetPinOutput(kWarpPinSPI_MISO_UART_RTS);
//	GPIO_DRV_SetPinOutput(kWarpPinSPI_MOSI_UART_CTS);

	/*
	 *	Initialize LPUART0. See KSDK13APIRM.pdf section 40.4.3, page 1353
	 */
	lpuartUserConfig.baudRate = gWarpUartBaudRateBps;
	lpuartUserConfig.parityMode = kLpuartParityDisabled;
	lpuartUserConfig.stopBitCount = kLpuartOneStopBit;
	lpuartUserConfig.bitCountPerChar = kLpuart8BitsPerChar;
	lpuartUserConfig.clockSource = kClockLpuartSrcMcgIrClk;

	LPUART_DRV_Init(0,(lpuart_state_t *)&lpuartState,(lpuart_user_config_t *)&lpuartUserConfig);
}


void
disableLPUARTpins(void)
{
	/*
	 *	LPUART deinit
	 */
	LPUART_DRV_Deinit(0);

	/*
	 *	Set UART pin association. See, e.g., page 99 in
	 *
	 *		https://www.nxp.com/docs/en/reference-manual/KL03P24M48SF0RM.pdf
	 *
	 *	Setup:
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX for UART TX
	 *		PTB4/kWarpPinI2C0_SCL_UART_RX for UART RX

//TODO: we don't use the HW flow control and that messes with the SPI any way
 *		PTA6/kWarpPinSPI_MISO_UART_RTS for UART RTS
 *		PTA7/kWarpPinSPI_MOSI_UART_CTS for UART CTS
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);

//TODO: we don't use flow-control
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO_UART_RTS);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI_UART_CTS);

	/*
	 *	Disable LPUART CLOCK
	*/
	CLOCK_SYS_DisableLpuartClock(0);
}



WarpStatus
sendBytesToUART(uint8_t *  bytes, size_t nbytes)
{
	lpuart_status_t	status;

	status = LPUART_DRV_SendDataBlocking(0, bytes, nbytes, gWarpUartTimeoutMilliseconds);
	if (status != 0)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}



void
warpEnableSPIpins(void)
{
	CLOCK_SYS_EnableSpiClock(0);

	/*	kWarpPinSPI_MISO_UART_RTS_UART_RTS --> PTA6 (ALT3)	*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAlt3);

	/*	kWarpPinSPI_MOSI_UART_CTS --> PTA7 (ALT3)	*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAlt3);

	#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		/*	kWarpPinSPI_SCK	--> PTA9	(ALT3)		*/
		PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAlt3);
	#else
		/*	kWarpPinSPI_SCK	--> PTB0	(ALT3)		*/
		PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAlt3);
	#endif

	/*
	 *	Initialize SPI master. See KSDK13APIRM.pdf Section 70.4
	 */
	uint32_t			calculatedBaudRate;
	spiUserConfig.polarity		= kSpiClockPolarity_ActiveHigh;
	spiUserConfig.phase		= kSpiClockPhase_FirstEdge;
	spiUserConfig.direction		= kSpiMsbFirst;
	spiUserConfig.bitsPerSec	= gWarpSpiBaudRateKbps * 1000;
	SPI_DRV_MasterInit(0 /* SPI master instance */, (spi_master_state_t *)&spiMasterState);
	SPI_DRV_MasterConfigureBus(0 /* SPI master instance */, (spi_master_user_config_t *)&spiUserConfig, &calculatedBaudRate);
}



void
warpDisableSPIpins(void)
{
	SPI_DRV_MasterDeinit(0);

	/*	kWarpPinSPI_MISO_UART_RTS	--> PTA6	(GPI)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);

	/*	kWarpPinSPI_MOSI_UART_CTS	--> PTA7	(GPIO)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

	#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		/*	kWarpPinSPI_SCK	--> PTA9	(GPIO)			*/
		PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);
	#else
		/*	kWarpPinSPI_SCK	--> PTB0	(GPIO)			*/
		PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);
	#endif

//TODO: we don't use HW flow control so can remove these since we don't use the RTS/CTS
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI_UART_CTS);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO_UART_RTS);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK);

	CLOCK_SYS_DisableSpiClock(0);
}



void
warpDeasserAllSPIchipSelects(void)
{
	/*
	 *	By default, assusme pins are currently disabled (e.g., by a recent lowPowerPinStates())
	 *
	 *	Drive all chip selects high to disable them. Individual drivers call this routine before
	 *	appropriately asserting their respective chip selects.
	 *
	 *	Setup:
	 *		PTA12/kWarpPinISL23415_SPI_nCS	for GPIO
	 *		PTA9/kWarpPinAT45DB_SPI_nCS	for GPIO
	 *		PTA8/kWarpPinADXL362_SPI_nCS	for GPIO
	 *		PTB1/kWarpPinFPGA_nCS		for GPIO
	 *
	 *		On Glaux
	 		PTB2/kGlauxPinFlash_SPI_nCS for GPIO
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortMuxAsGpio);
}



void
debugPrintSPIsinkBuffer(void)
{
	for (int i = 0; i < kWarpMemoryCommonSpiBufferBytes; i++)
	{
		warpPrint("\tgWarpSpiCommonSinkBuffer[%d] = [0x%02X]\n", i, gWarpSpiCommonSinkBuffer[i]);
	}
	warpPrint("\n");
}



void
warpEnableI2Cpins(void)
{
	CLOCK_SYS_EnableI2cClock(0);

	/*
	 *	Setup:
	 *
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX	-->	(ALT2 == I2C)
	 *		PTB4/kWarpPinI2C0_SDA_UART_RX	-->	(ALT2 == I2C)
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt2);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt2);

	I2C_DRV_MasterInit(0 /* I2C instance */, (i2c_master_state_t *)&i2cMasterState);
}



void
warpDisableI2Cpins(void)
{
	I2C_DRV_MasterDeinit(0 /* I2C instance */);

	/*
	 *	Setup:
	 *
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX	-->	disabled
	 *		PTB4/kWarpPinI2C0_SDA_UART_RX	-->	disabled
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);

	CLOCK_SYS_DisableI2cClock(0);
}

	void
	lowPowerPinStates(void)
	{
		/*
		 *	Following Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf),
		 *	we configure all pins as output and set them to a known state. We choose
		 *	to set them all to '0' since it happens that the devices we want to keep
		 *	deactivated (SI4705) also need '0'.
		 */

		/*
		 *			PORT A
		 */
		/*
		 *	For now, don't touch the PTA0/1/2 SWD pins. Revisit in the future.
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
		PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
		PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);

		/*
		 *	PTA3 and PTA4 are the EXTAL0/XTAL0. They are also connected to the clock output
		 *	of the RV8803 (and PTA4 is a sacrificial pin for PTA3), so do not want to drive them.
		 *	We however have to configure PTA3 to Alt0 (kPortPinDisabled) to get the EXTAL0
		 *	functionality.
		 *
		 *	NOTE:	kPortPinDisabled is the equivalent of `Alt0`
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 3, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 4, kPortPinDisabled);

		/*
		 *	Disable PTA5
		 *
		 *	NOTE: Enabling this significantly increases current draw
		 *	(from ~180uA to ~4mA) and we don't need the RTC on revC.
		 *
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortPinDisabled);

		/*
		 *	Section 2.6 of Kinetis Energy Savings – Tips and Tricks says
		 *
		 *		"Unused pins should be configured in the disabled state, mux(0),
		 *		to prevent unwanted leakage (potentially caused by floating inputs)."
		 *
		 *	However, other documents advice to place pin as GPIO and drive low or high.
		 *	For now, leave disabled. Filed issue #54 low-power pin states to investigate.
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortPinDisabled);

		/*
		 *	NOTE: The KL03 has no PTA10 or PTA11
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortPinDisabled);


		/*
		 *			PORT B
		 */
		PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortPinDisabled);
	}


void
disableTPS62740(void)
{
	#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT)
		GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_REGCTRL);
	#endif
}

void
enableTPS62740(uint16_t voltageMillivolts)
{
	#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT)
		/*
		 *	By default, assusme pins are currently disabled (e.g., by a recent lowPowerPinStates())
		 *
		 *	Setup:
		 *		PTB5/kWarpPinTPS62740_REGCTRL for GPIO
		 *		PTB6/kWarpPinTPS62740_VSEL4 for GPIO
		 *		PTB7/kWarpPinTPS62740_VSEL3 for GPIO
		 *		PTB10/kWarpPinTPS62740_VSEL2 for GPIO
		 *		PTB11/kWarpPinTPS62740_VSEL1 for GPIO
		 */
		PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortMuxAsGpio);
		PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortMuxAsGpio);
		PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortMuxAsGpio);
		PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortMuxAsGpio);
		PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortMuxAsGpio);

		setTPS62740CommonControlLines(voltageMillivolts);
		GPIO_DRV_SetPinOutput(kWarpPinTPS62740_REGCTRL);
	#endif
}

void
setTPS62740CommonControlLines(uint16_t voltageMillivolts)
{
	#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT)
		switch(voltageMillivolts)
		{
			case 1800:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 1900:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2000:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2100:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2200:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2300:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2400:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2500:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2600:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2700:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2800:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2900:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 3000:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 3100:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 3200:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 3300:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			/*
			 *	Should never happen, due to previous check in warpScaleSupplyVoltage()
			 */
			default:
			{
				warpPrint(RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_YELLOW RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorSanity RTT_CTRL_RESET "\n");
			}
		}

		/*
		 *	Vload ramp time of the TPS62740 is 800us max (datasheet, Table 8.5 / page 6)
		 */
		OSA_TimeDelay(gWarpSupplySettlingDelayMilliseconds);
	#endif
}



void
warpScaleSupplyVoltage(uint16_t voltageMillivolts)
{
	if (voltageMillivolts == gWarpCurrentSupplyVoltage)
	{
		return;
	}

	#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT)
		if (voltageMillivolts >= 1800 && voltageMillivolts <= 3300)
		{
			enableTPS62740(voltageMillivolts);
			gWarpCurrentSupplyVoltage = voltageMillivolts;
		}
		else
		{
			warpPrint(RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_RED RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorInvalidVoltage RTT_CTRL_RESET "\n", voltageMillivolts);
		}
	#endif
}



void
warpDisableSupplyVoltage(void)
{
	#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT)
		disableTPS62740();

		/*
		 *	Vload ramp time of the TPS62740 is 800us max (datasheet, Table 8.5 / page 6)
		 */
		OSA_TimeDelay(gWarpSupplySettlingDelayMilliseconds);
	#endif
}


void
warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState)
{
	WarpStatus	status = kWarpStatusOK;

	/*
	 *	Set all pins into low-power states. We don't just disable all pins,
	 *	as the various devices hanging off will be left in higher power draw
	 *	state. And manuals say set pins to output to reduce power.
	 */
	if (forceAllPinsIntoLowPowerState)
	{
		lowPowerPinStates();
	}

	warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* Sleep Seconds */);
	if ((status != kWarpStatusOK) && (status != kWarpStatusPowerTransitionErrorVlpr2Vlpr))
	{
		warpPrint("warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* sleep seconds : irrelevant here */)() failed...\n");
	}

	status = warpSetLowPowerMode(kWarpPowerModeVLPS, sleepSeconds);
	if (status != kWarpStatusOK)
	{
		warpPrint("warpSetLowPowerMode(kWarpPowerModeVLPS, 0 /* sleep seconds : irrelevant here */)() failed...\n");
	}
}

void
dumpProcessorState(void)
{
	uint32_t	cpuClockFrequency;

	CLOCK_SYS_GetFreq(kCoreClock, &cpuClockFrequency);
	warpPrint("\r\n\n\tCPU @ %u KHz\n", (cpuClockFrequency / 1000));
	warpPrint("\r\tCPU power mode: %u\n", POWER_SYS_GetCurrentMode());
	warpPrint("\r\tCPU clock manager configuration: %u\n", CLOCK_SYS_GetCurrentConfiguration());
	warpPrint("\r\tRTC clock: %d\n", CLOCK_SYS_GetRtcGateCmd(0));
	warpPrint("\r\tSPI clock: %d\n", CLOCK_SYS_GetSpiGateCmd(0));
	warpPrint("\r\tI2C clock: %d\n", CLOCK_SYS_GetI2cGateCmd(0));
	warpPrint("\r\tLPUART clock: %d\n", CLOCK_SYS_GetLpuartGateCmd(0));
	warpPrint("\r\tPORT A clock: %d\n", CLOCK_SYS_GetPortGateCmd(0));
	warpPrint("\r\tPORT B clock: %d\n", CLOCK_SYS_GetPortGateCmd(1));
	warpPrint("\r\tFTF clock: %d\n", CLOCK_SYS_GetFtfGateCmd(0));
	warpPrint("\r\tADC clock: %d\n", CLOCK_SYS_GetAdcGateCmd(0));
	warpPrint("\r\tCMP clock: %d\n", CLOCK_SYS_GetCmpGateCmd(0));
	warpPrint("\r\tVREF clock: %d\n", CLOCK_SYS_GetVrefGateCmd(0));
	warpPrint("\r\tTPM clock: %d\n", CLOCK_SYS_GetTpmGateCmd(0));
}


void
printBootSplash(uint16_t gWarpCurrentSupplyVoltage, uint8_t menuRegisterAddress, WarpPowerManagerCallbackStructure *  powerManagerCallbackStructure)
{
	/*
	 *	We break up the prints with small delays to allow us to use small RTT print
	 *	buffers without overrunning them when at max CPU speed.
	 */
	warpPrint("\r\n\n\n\n[ *\t\t\t\tWarp (HW revision C) / Glaux (HW revision B)\t\t\t* ]\n");
	warpPrint("\r[  \t\t\t\t      Cambridge / Physcomplab   \t\t\t\t  ]\n\n");
	warpPrint("\r\tSupply=%dmV,\tDefault Target Read Register=0x%02x\n",
			gWarpCurrentSupplyVoltage, menuRegisterAddress);
	warpPrint("\r\tI2C=%dkb/s,\tSPI=%dkb/s,\tUART=%db/s,\tI2C Pull-Up=%d\n\n",
			gWarpI2cBaudRateKbps, gWarpSpiBaudRateKbps, gWarpUartBaudRateBps);
	warpPrint("\r\tSIM->SCGC6=0x%02x\t\tRTC->SR=0x%02x\t\tRTC->TSR=0x%02x\n", SIM->SCGC6, RTC->SR, RTC->TSR);
	warpPrint("\r\tMCG_C1=0x%02x\t\t\tMCG_C2=0x%02x\t\tMCG_S=0x%02x\n", MCG_C1, MCG_C2, MCG_S);
	warpPrint("\r\tMCG_SC=0x%02x\t\t\tMCG_MC=0x%02x\t\tOSC_CR=0x%02x\n", MCG_SC, MCG_MC, OSC_CR);
	warpPrint("\r\tSMC_PMPROT=0x%02x\t\t\tSMC_PMCTRL=0x%02x\t\tSCB->SCR=0x%02x\n", SMC_PMPROT, SMC_PMCTRL, SCB->SCR);
	warpPrint("\r\tPMC_REGSC=0x%02x\t\t\tSIM_SCGC4=0x%02x\tRTC->TPR=0x%02x\n\n", PMC_REGSC, SIM_SCGC4, RTC->TPR);
	warpPrint("\r\t%ds in RTC Handler to-date,\t%d Pmgr Errors\n", gWarpSleeptimeSeconds, powerManagerCallbackStructure->errorCount);
}

void
blinkLED(int pin)
{
	GPIO_DRV_SetPinOutput(pin);
	OSA_TimeDelay(200);
	GPIO_DRV_ClearPinOutput(pin);
	OSA_TimeDelay(200);

	return;
}

void
warpPrint(const char *fmt, ...)
{
	int	fmtlen;
	va_list	arg;

	/*
	 *	We use an ifdef rather than a C if to allow us to compile-out
	 *	all references to SEGGER_RTT_*printf if we don't want them.
	 *
	 *	NOTE: SEGGER_RTT_vprintf takes a va_list* rather than a va_list
	 *	like usual vprintf. We modify the SEGGER_RTT_vprintf so that it
	 *	also takes our print buffer which we will eventually send over
	 *	BLE. Using SEGGER_RTT_vprintf() versus the libc vsnprintf saves
	 *	2kB flash and removes the use of malloc so we can keep heap
	 *	allocation to zero.
	 */
	#if (WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF)
		/*
		 *	We can't use SEGGER_RTT_vprintf to format into a buffer
		 *	since SEGGER_RTT_vprintf formats directly into the special
		 *	RTT memory region to be picked up by the RTT / SWD mechanism...
		 */
		va_start(arg, fmt);
		fmtlen = SEGGER_RTT_vprintf(0, fmt, &arg, gWarpPrintBuffer, kWarpDefaultPrintBufferSizeBytes);
		va_end(arg);

		if (fmtlen < 0)
		{
			SEGGER_RTT_WriteString(0, gWarpEfmt);

			#if (WARP_BUILD_ENABLE_DEVBGX)
				if (gWarpBooted)
				{
					WarpStatus	status;

					enableLPUARTpins();
					initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);
					status = sendBytesToUART((uint8_t *)gWarpEfmt, strlen(gWarpEfmt)+1);
					if (status != kWarpStatusOK)
					{
						SEGGER_RTT_WriteString(0, gWarpEuartSendChars);
					}
					disableLPUARTpins();

					/*
					 *	We don't want to deInit() the BGX since that would drop
					 *	any remote terminal connected to it.
					 */
					//deinitBGX();
				}
			#endif

			return;
		}

		/*
		 *	If WARP_BUILD_ENABLE_DEVBGX, also send the fmt to the UART / BLE.
		 */
		#if (WARP_BUILD_ENABLE_DEVBGX)
			if (gWarpBooted)
			{
				WarpStatus	status;

				enableLPUARTpins();
				initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);

				status = sendBytesToUART((uint8_t *)gWarpPrintBuffer, max(fmtlen, kWarpDefaultPrintBufferSizeBytes));
				if (status != kWarpStatusOK)
				{
					SEGGER_RTT_WriteString(0, gWarpEuartSendChars);
				}
				disableLPUARTpins();

				/*
				 *	We don't want to deInit() the BGX since that would drop
				 *	any remote terminal connected to it.
				 */
				//deinitBGX();
			}
		#endif
	#else
		/*
		 *	If we are not compiling in the SEGGER_RTT_printf,
		 *	we just send the format string of warpPrint()
		 */
		SEGGER_RTT_WriteString(0, fmt);

		/*
		 *	If WARP_BUILD_ENABLE_DEVBGX, also send the fmt to the UART / BLE.
		 */
		#if (WARP_BUILD_ENABLE_DEVBGX)
			if (gWarpBooted)
			{
				WarpStatus	status;

				enableLPUARTpins();
				initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);
				status = sendBytesToUART(fmt, strlen(fmt));
				if (status != kWarpStatusOK)
				{
					SEGGER_RTT_WriteString(0, gWarpEuartSendChars);
				}
				disableLPUARTpins();

				/*
				 *	We don't want to deInit() the BGX since that would drop
				 *	any remote terminal connected to it.
				 */
				//deinitBGX();
			}
		#endif
	#endif

	return;
}

int
warpWaitKey(void)
{
	/*
	 *	SEGGER'S implementation assumes the result of result of
	 *	SEGGER_RTT_GetKey() is an int, so we play along.
	 */
	int		rttKey, bleChar = kWarpMiscMarkerForAbsentByte;

	/*
	 *	Set the UART buffer to 0xFF and then wait until either the
	 *	UART RX buffer changes or the RTT icoming key changes.
	 *
	 *	The check below on rttKey is exactly what SEGGER_RTT_WaitKey()
	 *	does in SEGGER_RTT.c.
	 */
	#if (WARP_BUILD_ENABLE_DEVBGX)
		deviceBGXState.uartRXBuffer[0] = kWarpMiscMarkerForAbsentByte;
		enableLPUARTpins();
		initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);
	#endif

	do
	{
		rttKey	= SEGGER_RTT_GetKey();

		#if (WARP_BUILD_ENABLE_DEVBGX)
			bleChar	= deviceBGXState.uartRXBuffer[0];
		#endif

		/*
		 *	NOTE: We ignore all chars on BLE except '0'-'9', 'a'-'z'/'A'-Z'
		 */
		if (!(bleChar > 'a' && bleChar < 'z') && !(bleChar > 'A' && bleChar < 'Z') && !(bleChar > '0' && bleChar < '9'))
		{
			bleChar = kWarpMiscMarkerForAbsentByte;
		}
	} while ((rttKey < 0) && (bleChar == kWarpMiscMarkerForAbsentByte));

	#if (WARP_BUILD_ENABLE_DEVBGX)
		if (bleChar != kWarpMiscMarkerForAbsentByte)
		{
			/*
			 *	Send a copy of incoming BLE chars to RTT
			 */
			SEGGER_RTT_PutChar(0, bleChar);
			disableLPUARTpins();

			/*
			 *	We don't want to deInit() the BGX since that would drop
			 *	any remote terminal connected to it.
			 */
			//deinitBGX();

			return (int)bleChar;
		}

		/*
		 *	Send a copy of incoming RTT chars to BLE
		 */
		WarpStatus status = sendBytesToUART((uint8_t *)&rttKey, 1);
		if (status != kWarpStatusOK)
		{
			SEGGER_RTT_WriteString(0, gWarpEuartSendChars);
		}

		disableLPUARTpins();

		/*
		 *	We don't want to deInit() the BGX since that would drop
		 *	any remote terminal connected to it.
		 */
		//deinitBGX();
	#endif

	return rttKey;
}

int
main(void)
{
	WarpStatus				status;
	uint8_t					key;
	WarpSensorDevice			menuTargetSensor		= kWarpSensorBMX055accel;
	volatile WarpI2CDeviceState *		menuI2cDevice			= NULL;
	uint8_t					menuRegisterAddress		= 0x00;
	rtc_datetime_t				warpBootDate;
	power_manager_user_config_t		warpPowerModeWaitConfig;
	power_manager_user_config_t		warpPowerModeStopConfig;
	power_manager_user_config_t		warpPowerModeVlpwConfig;
	power_manager_user_config_t		warpPowerModeVlpsConfig;
	power_manager_user_config_t		warpPowerModeVlls0Config;
	power_manager_user_config_t		warpPowerModeVlls1Config;
	power_manager_user_config_t		warpPowerModeVlls3Config;
	power_manager_user_config_t		warpPowerModeRunConfig;

	/*
	 *	We use this as a template later below and change the .mode fields for the different other modes.
	 */
	const power_manager_user_config_t	warpPowerModeVlprConfig = {
							.mode			= kPowerManagerVlpr,
							.sleepOnExitValue	= false,
							.sleepOnExitOption	= false
						};

	power_manager_user_config_t const *	powerConfigs[] = {
							/*
							 *	NOTE: POWER_SYS_SetMode() depends on this order
							 *
							 *	See KSDK13APIRM.pdf Section 55.5.3
							 */
							&warpPowerModeWaitConfig,
							&warpPowerModeStopConfig,
							&warpPowerModeVlprConfig,
							&warpPowerModeVlpwConfig,
							&warpPowerModeVlpsConfig,
							&warpPowerModeVlls0Config,
							&warpPowerModeVlls1Config,
							&warpPowerModeVlls3Config,
							&warpPowerModeRunConfig,
						};

	WarpPowerManagerCallbackStructure		powerManagerCallbackStructure;

	/*
	 *	Callback configuration structure for power manager
	 */
	const power_manager_callback_user_config_t callbackCfg0 = {
							callback0,
							kPowerManagerCallbackBeforeAfter,
							(power_manager_callback_data_t *) &powerManagerCallbackStructure};

	/*
	 *	Pointers to power manager callbacks.
	 */
	power_manager_callback_user_config_t const *	callbacks[] = {
								&callbackCfg0
						};

	/*
	 *	Enable clock for I/O PORT A and PORT B
	 */
	CLOCK_SYS_EnablePortClock(0);
	CLOCK_SYS_EnablePortClock(1);

	/*
	 *	Set board crystal value (Warp revB and earlier).
	 */
	g_xtal0ClkFreq = 32768U;

	/*
	 *	Initialize KSDK Operating System Abstraction layer (OSA) layer.
	 */
	OSA_Init();

	/*
	 *	Setup SEGGER RTT to output as much as fits in buffers.
	 *
	 *	Using SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL can lead to deadlock, since
	 *	we might have SWD disabled at time of blockage.
	 */
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);

	/*
	 *	When booting to CSV stream, we wait to be up and running as soon as possible after
	 *	a reset (e.g., a reset due to waking from VLLS0)
	 */
	if (!WARP_BUILD_BOOT_TO_CSVSTREAM)
	{
		warpPrint("\n\n\n\rBooting Warp, in 3... ");
		OSA_TimeDelay(1000);
		warpPrint("2... ");
		OSA_TimeDelay(1000);
		warpPrint("1...\n\n\n\r");
		OSA_TimeDelay(1000);
	}

	/*
	 *	Configure Clock Manager to default, and set callback for Clock Manager mode transition.
	 *
	 *	See "Clocks and Low Power modes with KSDK and Processor Expert" document (Low_Power_KSDK_PEx.pdf)
	 */
	CLOCK_SYS_Init(	g_defaultClockConfigurations,
			CLOCK_CONFIG_NUM, /* The default value of this is defined in fsl_clock_MKL03Z4.h as 2 */
			&clockCallbackTable,
			ARRAY_SIZE(clockCallbackTable)
			);
	CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);

	/*
	 *	Initialize RTC Driver (not needed on Glaux, but we enable it anyway for now
	 *	as that lets us use the current sleep routines). NOTE: We also don't seem to
	 *	be able to go to VLPR mode unless we enable the RTC.
	 */
	RTC_DRV_Init(0);

	/*
	 *	Set initial date to 1st January 2016 00:00, and set date via RTC driver
	 */
	warpBootDate.year	= 2016U;
	warpBootDate.month	= 1U;
	warpBootDate.day	= 1U;
	warpBootDate.hour	= 0U;
	warpBootDate.minute	= 0U;
	warpBootDate.second	= 0U;
	RTC_DRV_SetDatetime(0, &warpBootDate);

	/*
	 *	Setup Power Manager Driver
	 */
	memset(&powerManagerCallbackStructure, 0, sizeof(WarpPowerManagerCallbackStructure));

	warpPowerModeVlpwConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpwConfig.mode = kPowerManagerVlpw;

	warpPowerModeVlpsConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpsConfig.mode = kPowerManagerVlps;

	warpPowerModeWaitConfig = warpPowerModeVlprConfig;
	warpPowerModeWaitConfig.mode = kPowerManagerWait;

	warpPowerModeStopConfig = warpPowerModeVlprConfig;
	warpPowerModeStopConfig.mode = kPowerManagerStop;

	warpPowerModeVlls0Config = warpPowerModeVlprConfig;
	warpPowerModeVlls0Config.mode = kPowerManagerVlls0;

	warpPowerModeVlls1Config = warpPowerModeVlprConfig;
	warpPowerModeVlls1Config.mode = kPowerManagerVlls1;

	warpPowerModeVlls3Config = warpPowerModeVlprConfig;
	warpPowerModeVlls3Config.mode = kPowerManagerVlls3;

	warpPowerModeRunConfig.mode = kPowerManagerRun;

	POWER_SYS_Init(	&powerConfigs,
			sizeof(powerConfigs)/sizeof(power_manager_user_config_t *),
			&callbacks,
			sizeof(callbacks)/sizeof(power_manager_callback_user_config_t *)
			);

	/*
	 *	Switch CPU to Very Low Power Run (VLPR) mode
	 */
	if (WARP_BUILD_BOOT_TO_VLPR)
	{
		warpPrint("About to switch CPU to VLPR mode... ");
		status = warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* Sleep Seconds */);
		if ((status != kWarpStatusOK) && (status != kWarpStatusPowerTransitionErrorVlpr2Vlpr))
		{
			warpPrint("warpSetLowPowerMode(kWarpPowerModeVLPR() failed...\n");
		}
		warpPrint("done.\n\r");
	}

	/*
	 *	Initialize the GPIO pins with the appropriate pull-up, etc.,
	 *	defined in the inputPins and outputPins arrays (gpio_pins.c).
	 *
	 *	See also Section 30.3.3 GPIO Initialization of KSDK13APIRM.pdf
	 */
	warpPrint("About to GPIO_DRV_Init()... ");
	GPIO_DRV_Init(inputPins  /* input pins */, outputPins  /* output pins */);
	warpPrint("done.\n");

	/*
	 *	Make sure the SWD pins, PTA0/1/2 SWD pins in their ALT3 state (i.e., as SWD).
	 *
	 *	See GitHub issue https://github.com/physical-computation/Warp-firmware/issues/54
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);

	/*
	 *	Note that it is lowPowerPinStates() that sets the pin mux mode,
	 *	so until we call it pins are in their default state.
	 */
	warpPrint("About to lowPowerPinStates()... ");
	lowPowerPinStates();
	warpPrint("done.\n");

	// Variable for main program loop
	uint16_t ADCDec, ADCVolt;
	uint16_t Water_level;
	uint8_t bcd_dates[7];
	uint8_t conv_dates[7];
	bool pump = false;
	bool pumped = false;

	// Set the level below which the pump turns on (mm)
	uint16_t theshold = 200;

	/*

	 *	Initialize all the sensors
	 */
	 // Initialise and turn off relay
	 initRelay();
	 TurnOnRelay();
	 OSA_TimeDelay(2000);
	 TurnOffRelay();

	 // Initialise ADC
	 initADC();

	 ADCDec = readADC();
	 warpPrint("ADC Dec: %d \n", ADCDec);
	 ADCVolt = Voltage_ADC(ADCDec);
	 warpPrint("ADC Voltage: %d mV\n", ADCVolt);
	 Water_level = level();
	 warpPrint("Water Level: %d mm\n", Water_level);

	 // Initialise BME680 (temperature sensor)
	 initBME680(	0x77	/* i2cAddress */,		kWarpDefaultSupplyVoltageMillivoltsBME680	);
	 status = configureSensorBME680(	0b00000001,	/*	payloadCtrl_Hum: Humidity oversampling (OSRS) to 1x				*/
	 				0b00100100,	/*	payloadCtrl_Meas: Temperature oversample 1x, pressure overdsample 1x, mode 00	*/
	 				0b00001000	/*	payloadGas_0: Turn off heater							*/
	 				);
	 if (status != kWarpStatusOK)
	 {
					warpPrint("configureSensorBME680() failed...\n");
	 }
	 printSensorDataBME680(false);
	 warpPrint("\n");

	 // Initialise RV8803C7 external RTC and check it's working
	#if (WARP_BUILD_ENABLE_DEVRV8803C7)
		initRV8803C7(	0x32	/* i2cAddress */,					kWarpDefaultSupplyVoltageMillivoltsRV8803C7	);
		status = setRTCCountdownRV8803C7(0 /* countdown */, kWarpRV8803ExtTD_1HZ /* frequency */, false /* interupt_enable */);
		if (status != kWarpStatusOK)
		{
			warpPrint("setRTCCountdownRV8803C7() failed...\n");
		}
		else
		{
			warpPrint("setRTCCountdownRV8803C7() succeeded.\n");
		}

		/*
		 *	Set the CLKOUT frequency to 1Hz, to reduce CV^2 power on the CLKOUT pin.
		 *	See RV-8803-C7_App-Manual.pdf section 3.6 (register is 0Dh)
		 */
		uint8_t	extReg;
		status = readRTCRegisterRV8803C7(kWarpRV8803RegExt, &extReg);
		if (status != kWarpStatusOK)
		{
			warpPrint("readRTCRegisterRV8803C7() failed...\n");
		}
		else
		{
			warpPrint("readRTCRegisterRV8803C7() succeeded.\n");
		}

		/*
		 *	Set bits 3:2 (FD) to 10 (1Hz CLKOUT)
		 */
		extReg &= 0b11110011;
		extReg |= 0b00001000;
		status = writeRTCRegisterRV8803C7(kWarpRV8803RegExt, extReg);
		if (status != kWarpStatusOK)
		{
			warpPrint("writeRTCRegisterRV8803C7() failed...\n");
		}
		else
		{
			warpPrint("writeRTCRegisterRV8803C7() succeeded.\n");
		}
	#endif
	/*
	// Set RTC to this time
	rtc_datetime_t				warpRTC;
	warpRTC.year	= 2022U;
	warpRTC.month	= 1U;
	warpRTC.day	= 26U;
	warpRTC.hour	= 02U;
	warpRTC.minute	= 32U;
	warpRTC.second	= 00U;
	setRTCTimeRV8803C7(&warpRTC);
	*/

	/*
	 *	At this point, we consider the system "booted" and, e.g., warpPrint()s
	 *	will also be sent to the BLE if that is compiled in.
	 */
	gWarpBooted = true;
	warpPrint("Boot done.\n");

	while (1)
	{
		// main loop code here
		Water_level = level();
		warpPrint("Water Level: %d mm\n", Water_level);

		status = readRTCRegistersRV8803C7(kWarpRV8803RegSec, 7, &bcd_dates);
		if (status != kWarpStatusOK)
		{
			warpPrint("readRTCRegistersRV8803C7(kWarpRV8803RegSec, &date) failed\n");
		}
		else
		{
			for (uint8_t n = 0; n<7; n+=1)
			{
				if (n != 3) // don't convert weekday
				{
					conv_dates[n] = bcd2bin(bcd_dates[n]);
				}
			}
			warpPrint("%d:%d:%d %d/%d/%d\n", conv_dates[2], conv_dates[1], conv_dates[0], conv_dates[4], conv_dates[5], conv_dates[6]);
		}
		OSA_TimeDelay(1000);
	}
	return 0;
}

void
printAllSensors(bool printHeadersAndCalibration, bool hexModeFlag, int menuDelayBetweenEachRun, bool loopForever)
{
	/*
	 *	A 32-bit counter gives us > 2 years of before it wraps, even if sampling at 60fps
	 */
	uint32_t	readingCount = 0;
	uint32_t	numberOfConfigErrors = 0;


	#if (WARP_BUILD_ENABLE_DEVAMG8834)
	numberOfConfigErrors += configureSensorAMG8834(	0x3F,/* Initial reset */
					0x01,/* Frame rate 1 FPS */
					);
	#endif
	#if (WARP_BUILD_ENABLE_DEVMMA8451Q)
	numberOfConfigErrors += configureSensorMMA8451Q(0x00,/* Payload: Disable FIFO */
					0x01/* Normal read 8bit, 800Hz, normal, active mode */
					);
	#endif
	#if (WARP_BUILD_ENABLE_DEVMAG3110)
	numberOfConfigErrors += configureSensorMAG3110(	0x00,/*	Payload: DR 000, OS 00, 80Hz, ADC 1280, Full 16bit, standby mode to set up register*/
					0xA0,/*	Payload: AUTO_MRST_EN enable, RAW value without offset */
					);
	#endif
	#if (WARP_BUILD_ENABLE_DEVL3GD20H)
	numberOfConfigErrors += configureSensorL3GD20H(	0b11111111,/* ODR 800Hz, Cut-off 100Hz, see table 21, normal mode, x,y,z enable */
					0b00100000,
					0b00000000/* normal mode, disable FIFO, disable high pass filter */
					);
	#endif
	#if (WARP_BUILD_ENABLE_DEVBME680)
	numberOfConfigErrors += configureSensorBME680(	0b00000001,	/*	payloadCtrl_Hum: Humidity oversampling (OSRS) to 1x				*/
							0b00100100,	/*	payloadCtrl_Meas: Temperature oversample 1x, pressure overdsample 1x, mode 00	*/
							0b00001000	/*	payloadGas_0: Turn off heater							*/
					);

	if (printHeadersAndCalibration)
	{
		warpPrint("\r\n\nBME680 Calibration Data: ");
		for (uint8_t i = 0; i < kWarpSizesBME680CalibrationValuesCount; i++)
		{
			warpPrint("0x%02x", deviceBME680CalibrationValues[i]);
			if (i < kWarpSizesBME680CalibrationValuesCount - 1)
			{
				warpPrint(", ");
			}
			else
			{
				warpPrint("\n\n");
			}
		}
	}
	#endif

	#if (WARP_BUILD_ENABLE_DEVHDC1000)
	numberOfConfigErrors += writeSensorRegisterHDC1000(kWarpSensorConfigurationRegisterHDC1000Configuration,/* Configuration register	*/
					(0b1010000<<8),
					);
	#endif

	#if (WARP_BUILD_ENABLE_DEVCCS811)
	uint8_t		payloadCCS811[1];
	payloadCCS811[0] = 0b01000000;/* Constant power, measurement every 250ms */
	numberOfConfigErrors += configureSensorCCS811(payloadCCS811,
					);
	#endif
	#if (WARP_BUILD_ENABLE_DEVBMX055)
	numberOfConfigErrors += configureSensorBMX055accel(0b00000011,/* Payload:+-2g range */
					0b10000000,/* Payload:unfiltered data, shadowing enabled */
					);
	numberOfConfigErrors += configureSensorBMX055mag(0b00000001,/* Payload:from suspend mode to sleep mode*/
					0b00000001,/* Default 10Hz data rate, forced mode*/
					);
	numberOfConfigErrors += configureSensorBMX055gyro(0b00000100,/* +- 125degrees/s */
					0b00000000,/* ODR 2000 Hz, unfiltered */
					0b00000000,/* normal mode */
					0b10000000,/* unfiltered data, shadowing enabled */
					);
	#endif
	#if (WARP_BUILD_ENABLE_DEVINA219)
	numberOfConfigErrors += configureSensorINA219(0x399F /* Configuration register*/,
					0x1000 /*Calibration Register*/
					);
	#endif

	if (printHeadersAndCalibration)
	{
		warpPrint("Measurement number, RTC->TSR, RTC->TPR,\t\t");

		#if (WARP_BUILD_ENABLE_DEVADXL362)
			warpPrint(" ADXL362 x, ADXL362 y, ADXL362 z,");
		#endif

		#if (WARP_BUILD_ENABLE_DEVAMG8834)
		for (uint8_t i = 0; i < 64; i++)
		{
			warpPrint(" AMG8834 %d,", i);
		}
		warpPrint(" AMG8834 Temp,");
		#endif

		#if (WARP_BUILD_ENABLE_DEVMMA8451Q)
			warpPrint(" MMA8451 x, MMA8451 y, MMA8451 z,");
		#endif

		#if (WARP_BUILD_ENABLE_DEVMAG3110)
			warpPrint(" MAG3110 x, MAG3110 y, MAG3110 z, MAG3110 Temp,");
		#endif

		#if (WARP_BUILD_ENABLE_DEVL3GD20H)
			warpPrint(" L3GD20H x, L3GD20H y, L3GD20H z, L3GD20H Temp,");
		#endif

		#if (WARP_BUILD_ENABLE_DEVBME680)
			warpPrint(" BME680 Press, BME680 Temp, BME680 Hum,");
		#endif

		#if (WARP_BUILD_ENABLE_DEVBMX055)
			warpPrint(" BMX055acc x, BMX055acc y, BMX055acc z, BMX055acc Temp,");
			warpPrint(" BMX055mag x, BMX055mag y, BMX055mag z, BMX055mag RHALL,");
			warpPrint(" BMX055gyro x, BMX055gyro y, BMX055gyro z,");
		#endif

		#if (WARP_BUILD_ENABLE_DEVCCS811)
			warpPrint(" CCS811 ECO2, CCS811 TVOC, CCS811 RAW ADC value,");
		#endif

		#if (WARP_BUILD_ENABLE_DEVHDC1000)
			warpPrint(" HDC1000 Temp, HDC1000 Hum,");
		#endif

		#if (WARP_BUILD_ENABLE_DEVINA219)
			warpPrint(" INA219 Current,");
		#endif

		warpPrint(" RTC->TSR, RTC->TPR, # Config Errors");
		warpPrint("\n\n");
	}

	do
	{
		warpPrint("%12u, %12d, %6d,\t\t", readingCount, RTC->TSR, RTC->TPR);

		#if (WARP_BUILD_ENABLE_DEVADXL362)
			printSensorDataADXL362(hexModeFlag);
		#endif

		#if (WARP_BUILD_ENABLE_DEVAMG8834)
			printSensorDataAMG8834(hexModeFlag);
		#endif

		#if (WARP_BUILD_ENABLE_DEVMMA8451Q)
			printSensorDataMMA8451Q(hexModeFlag);
		#endif

		#if (WARP_BUILD_ENABLE_DEVMAG3110)
			printSensorDataMAG3110(hexModeFlag);
		#endif

		#if (WARP_BUILD_ENABLE_DEVL3GD20H)
			printSensorDataL3GD20H(hexModeFlag);
		#endif

		#if (WARP_BUILD_ENABLE_DEVBME680)
			printSensorDataBME680(hexModeFlag);
		#endif

		#if (WARP_BUILD_ENABLE_DEVBMX055)
			printSensorDataBMX055accel(hexModeFlag);
			printSensorDataBMX055mag(hexModeFlag);
			printSensorDataBMX055gyro(hexModeFlag);
		#endif

		#if (WARP_BUILD_ENABLE_DEVCCS811)
			printSensorDataCCS811(hexModeFlag);
		#endif

		#if (WARP_BUILD_ENABLE_DEVHDC1000)
			printSensorDataHDC1000(hexModeFlag);
		#endif

		#if (WARP_BUILD_ENABLE_DEVINA219)
			printSensorDataINA219(hexModeFlag, 0x04 /* Current*/);
		#endif

		warpPrint(" %12d, %6d, %2u\n", RTC->TSR, RTC->TPR, numberOfConfigErrors);

		if (menuDelayBetweenEachRun > 0)
		{
			OSA_TimeDelay(menuDelayBetweenEachRun);
		}

		readingCount++;
	} while (loopForever);
}


void
loopForSensor(	const char *  tagString,
		WarpStatus  (* readSensorRegisterFunction)(uint8_t deviceRegister, int numberOfBytes),
		volatile WarpI2CDeviceState *  i2cDeviceState,
		volatile WarpSPIDeviceState *  spiDeviceState,
		uint8_t  baseAddress,
		uint8_t  minAddress,
		uint8_t  maxAddress,
		int  repetitionsPerAddress,
		int  chunkReadsPerAddress,
		int  spinDelay,
		bool  autoIncrement,
		uint16_t  sssupplyMillivolts,
		uint8_t  referenceByte,
		uint16_t adaptiveSssupplyMaxMillivolts,
		bool  chatty
		)
{
	WarpStatus		status;
	uint8_t			address = min(minAddress, baseAddress);
	int			readCount = repetitionsPerAddress + 1;
	int			nSuccesses = 0;
	int			nFailures = 0;
	int			nCorrects = 0;
	int			nBadCommands = 0;
	uint16_t		actualSssupplyMillivolts = sssupplyMillivolts;


	if (	(!spiDeviceState && !i2cDeviceState) ||
		(spiDeviceState && i2cDeviceState) )
	{
		warpPrint(RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_YELLOW RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorSanity RTT_CTRL_RESET "\n");
	}

	warpScaleSupplyVoltage(actualSssupplyMillivolts);
	warpPrint(tagString);

	/*
	 *	Keep on repeating until we are above the maxAddress, or just once if not autoIncrement-ing
	 *	This is checked for at the tail end of the loop.
	 */
	while (true)
	{
		for (int i = 0; i < readCount; i++) for (int j = 0; j < chunkReadsPerAddress; j++)
		{
			status = readSensorRegisterFunction(address+j, 1 /* numberOfBytes */);
			if (status == kWarpStatusOK)
			{
				nSuccesses++;
				if (actualSssupplyMillivolts > sssupplyMillivolts)
				{
					actualSssupplyMillivolts -= 100;
					warpScaleSupplyVoltage(actualSssupplyMillivolts);
				}

				if (spiDeviceState)
				{
					if (referenceByte == spiDeviceState->spiSinkBuffer[2])
					{
						nCorrects++;
					}

					if (chatty)
					{
						warpPrint("\r\t0x%02x --> [0x%02x 0x%02x 0x%02x]\n",
							address+j,
							spiDeviceState->spiSinkBuffer[0],
							spiDeviceState->spiSinkBuffer[1],
							spiDeviceState->spiSinkBuffer[2]);
					}
				}
				else
				{
					if (referenceByte == i2cDeviceState->i2cBuffer[0])
					{
						nCorrects++;
					}

					if (chatty)
					{
						warpPrint("\r\t0x%02x --> 0x%02x\n",
							address+j,
							i2cDeviceState->i2cBuffer[0]);
					}
				}
			}
			else if (status == kWarpStatusDeviceCommunicationFailed)
			{
				warpPrint("\r\t0x%02x --> ----\n",
					address+j);

				nFailures++;
				if (actualSssupplyMillivolts < adaptiveSssupplyMaxMillivolts)
				{
					actualSssupplyMillivolts += 100;
					warpScaleSupplyVoltage(actualSssupplyMillivolts);
				}
			}
			else if (status == kWarpStatusBadDeviceCommand)
			{
				nBadCommands++;
			}

			if (spinDelay > 0)
			{
				OSA_TimeDelay(spinDelay);
			}
		}

		if (autoIncrement)
		{
			address++;
		}

		if (address > maxAddress || !autoIncrement)
		{
			/*
			 *	We either iterated over all possible addresses, or were asked to do only
			 *	one address anyway (i.e. don't increment), so we're done.
			 */
			break;
		}
	}

	/*
	 *	We intersperse RTT_printfs with forced delays to allow us to use small
	 *	print buffers even in RUN mode.
	 */
	warpPrint("\r\n\t%d/%d success rate.\n", nSuccesses, (nSuccesses + nFailures));
	OSA_TimeDelay(50);
	warpPrint("\r\t%d/%d successes matched ref. value of 0x%02x.\n", nCorrects, nSuccesses, referenceByte);
	OSA_TimeDelay(50);
	warpPrint("\r\t%d bad commands.\n\n", nBadCommands);
	OSA_TimeDelay(50);


	return;
}



void
repeatRegisterReadForDeviceAndAddress(WarpSensorDevice warpSensorDevice, uint8_t baseAddress, bool autoIncrement, int chunkReadsPerAddress, bool chatty, int spinDelay, int repetitionsPerAddress, uint16_t sssupplyMillivolts, uint16_t adaptiveSssupplyMaxMillivolts, uint8_t referenceByte)
{
	switch (warpSensorDevice)
	{
		case kWarpSensorADXL362:
		{
			/*
			 *	ADXL362: VDD 1.6--3.5
			 */
			#if (WARP_BUILD_ENABLE_DEVADXL362)
				loopForSensor(	"\r\nADXL362:\n\r",		/*	tagString			*/
						&readSensorRegisterADXL362,	/*	readSensorRegisterFunction	*/
						NULL,				/*	i2cDeviceState			*/
						&deviceADXL362State,		/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x2E,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tADXL362 Read Aborted. Device Disabled :(");
			#endif

			break;
		}

		case kWarpSensorMMA8451Q:
		{
			/*
			 *	MMA8451Q: VDD 1.95--3.6
			 */
			#if (WARP_BUILD_ENABLE_DEVMMA8451Q)
				loopForSensor(	"\r\nMMA8451Q:\n\r",		/*	tagString			*/
						&readSensorRegisterMMA8451Q,	/*	readSensorRegisterFunction	*/
						&deviceMMA8451QState,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x31,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tMMA8451Q Read Aborted. Device Disabled :(");
			#endif

			break;
		}

		case kWarpSensorBME680:
		{
			/*
			 *	BME680: VDD 1.7--3.6
			 */
			#if (WARP_BUILD_ENABLE_DEVBME680)
				loopForSensor(	"\r\nBME680:\n\r",		/*	tagString			*/
						&readSensorRegisterBME680,	/*	readSensorRegisterFunction	*/
						&deviceBME680State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x1D,				/*	minAddress			*/
						0x75,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\nBME680 Read Aborted. Device Disabled :(");
			#endif

			break;
		}

		case kWarpSensorBMX055accel:
		{
			/*
			 *	BMX055accel: VDD 2.4V -- 3.6V
			 */
			#if (WARP_BUILD_ENABLE_DEVBMX055)
				loopForSensor(	"\r\nBMX055accel:\n\r",		/*	tagString			*/
						&readSensorRegisterBMX055accel,	/*	readSensorRegisterFunction	*/
						&deviceBMX055accelState,	/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x39,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tBMX055accel Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorBMX055gyro:
		{
			/*
			 *	BMX055gyro: VDD 2.4V -- 3.6V
			 */
			#if (WARP_BUILD_ENABLE_DEVBMX055)
				loopForSensor(	"\r\nBMX055gyro:\n\r",		/*	tagString			*/
						&readSensorRegisterBMX055gyro,	/*	readSensorRegisterFunction	*/
						&deviceBMX055gyroState,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x39,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tBMX055gyro Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorBMX055mag:
		{
			/*
			 *	BMX055mag: VDD 2.4V -- 3.6V
			 */
			#if WARP_BUILD_ENABLE_DEVBMX055
				loopForSensor(	"\r\nBMX055mag:\n\r",		/*	tagString			*/
						&readSensorRegisterBMX055mag,	/*	readSensorRegisterFunction	*/
						&deviceBMX055magState,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x40,				/*	minAddress			*/
						0x52,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\t BMX055mag Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorMAG3110:
		{
			/*
			 *	MAG3110: VDD 1.95 -- 3.6
			 */
			#if (WARP_BUILD_ENABLE_DEVMAG3110)
				loopForSensor(	"\r\nMAG3110:\n\r",		/*	tagString			*/
						&readSensorRegisterMAG3110,	/*	readSensorRegisterFunction	*/
						&deviceMAG3110State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x11,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tMAG3110 Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorL3GD20H:
		{
			/*
			 *	L3GD20H: VDD 2.2V -- 3.6V
			 */
			#if (WARP_BUILD_ENABLE_DEVL3GD20H)
				loopForSensor(	"\r\nL3GD20H:\n\r",		/*	tagString			*/
						&readSensorRegisterL3GD20H,	/*	readSensorRegisterFunction	*/
						&deviceL3GD20HState,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x0F,				/*	minAddress			*/
						0x39,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tL3GD20H Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorLPS25H:
		{
			/*
			 *	LPS25H: VDD 1.7V -- 3.6V
			 */
			#if (WARP_BUILD_ENABLE_DEVLPS25H)
				loopForSensor(	"\r\nLPS25H:\n\r",		/*	tagString			*/
						&readSensorRegisterLPS25H,	/*	readSensorRegisterFunction	*/
						&deviceLPS25HState,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x08,				/*	minAddress			*/
						0x24,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tLPS25H Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorTCS34725:
		{
			/*
			 *	TCS34725: VDD 2.7V -- 3.3V
			 */
			#if WARP_BUILD_ENABLE_DEVTCS34725
				loopForSensor(	"\r\nTCS34725:\n\r",		/*	tagString			*/
						&readSensorRegisterTCS34725,	/*	readSensorRegisterFunction	*/
						&deviceTCS34725State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x1D,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tTCS34725 Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorSI4705:
		{
			/*
			 *	SI4705: VDD 2.7V -- 5.5V
			 */
			#if (WARP_BUILD_ENABLE_DEVSI4705)
				loopForSensor(	"\r\nSI4705:\n\r",		/*	tagString			*/
						&readSensorRegisterSI4705,	/*	readSensorRegisterFunction	*/
						&deviceSI4705State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x09,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tSI4705 Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorHDC1000:
		{
			/*
			 *	HDC1000: VDD 3V--5V
			 */
			#if (WARP_BUILD_ENABLE_DEVHDC1000)
				loopForSensor(	"\r\nHDC1000:\n\r",		/*	tagString			*/
						&readSensorRegisterHDC1000,	/*	readSensorRegisterFunction	*/
						&deviceHDC1000State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x1F,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tHDC1000 Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorSI7021:
		{
			/*
			 *	SI7021: VDD 1.9V -- 3.6V
			 */
			#if (WARP_BUILD_ENABLE_DEVSI7021)
				loopForSensor(	"\r\nSI7021:\n\r",		/*	tagString			*/
						&readSensorRegisterSI7021,	/*	readSensorRegisterFunction	*/
						&deviceSI7021State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x09,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tSI7021 Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorCCS811:
		{
			/*
			 *	CCS811: VDD 1.8V -- 3.6V
			 */
			#if (WARP_BUILD_ENABLE_DEVCCS811)
				loopForSensor(	"\r\nCCS811:\n\r",		/*	tagString			*/
						&readSensorRegisterCCS811,	/*	readSensorRegisterFunction	*/
						&deviceCCS811State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0xFF,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tCCS811 Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorAMG8834:
		{
			/*
			 *	AMG8834: VDD 3.3V -- 3.3V
			 */
			#if WARP_BUILD_ENABLE_DEVAMG8834
				loopForSensor(	"\r\nAMG8834:\n\r",		/*	tagString			*/
						&readSensorRegisterAMG8834,	/*	readSensorRegisterFunction	*/
						&deviceAMG8834State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0xFF,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tAMG8834 Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorAS7262:
		{
			/*
			 *	AS7262: VDD 2.7--3.6
			 */
			#if (WARP_BUILD_ENABLE_DEVAS7262)
				loopForSensor(	"\r\nAS7262:\n\r",		/*	tagString			*/
						&readSensorRegisterAS7262,	/*	readSensorRegisterFunction	*/
						&deviceAS7262State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x2B,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tAS7262 Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorAS7263:
		{
			/*
			 *	AS7263: VDD 2.7--3.6
			 */
			#if WARP_BUILD_ENABLE_DEVAS7263
				loopForSensor(	"\r\nAS7263:\n\r",		/*	tagString			*/
						&readSensorRegisterAS7263,	/*	readSensorRegisterFunction	*/
						&deviceAS7263State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x2B,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tAS7263 Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorINA219:
		{
			/*
			 *	INA219
			 */
			#if (WARP_BUILD_ENABLE_DEVINA219)
				loopForSensor(	"\r\nINA219:\n\r",		/*	tagString			*/
						&readSensorRegisterINA219,	/*	readSensorRegisterFunction	*/
						&deviceINA219State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x05,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tINA219 Read Aborted. Device Disabled :(");
			#endif

			break;
		}

		default:
		{
			warpPrint("\r\tInvalid warpSensorDevice [%d] passed to repeatRegisterReadForDeviceAndAddress.\n", warpSensorDevice);
		}
	}

	if (warpSensorDevice != kWarpSensorADXL362)
	{
		warpDisableI2Cpins();
	}
}



int
char2int(int character)
{
	if (character >= '0' && character <= '9')
	{
		return character - '0';
	}

	if (character >= 'a' && character <= 'f')
	{
		return character - 'a' + 10;
	}

	if (character >= 'A' && character <= 'F')
	{
		return character - 'A' + 10;
	}

	return 0;
}



uint8_t
readHexByte(void)
{
	uint8_t		topNybble, bottomNybble;

	topNybble = warpWaitKey();
	bottomNybble = warpWaitKey();

	return (char2int(topNybble) << 4) + char2int(bottomNybble);
}



int
read4digits(void)
{
	uint8_t		digit1, digit2, digit3, digit4;

	digit1 = warpWaitKey();
	digit2 = warpWaitKey();
	digit3 = warpWaitKey();
	digit4 = warpWaitKey();

	return (digit1 - '0')*1000 + (digit2 - '0')*100 + (digit3 - '0')*10 + (digit4 - '0');
}



WarpStatus
writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte)
{
	i2c_status_t	status;
	uint8_t		commandBuffer[1];
	uint8_t		payloadBuffer[1];
	i2c_device_t	i2cSlaveConfig =
			{
				.address = i2cAddress,
				.baudRate_kbps = gWarpI2cBaudRateKbps
			};

	commandBuffer[0] = commandByte;
	payloadBuffer[0] = payloadByte;

	status = I2C_DRV_MasterSendDataBlocking(
						0	/* instance */,
						&i2cSlaveConfig,
						commandBuffer,
						(sendCommandByte ? 1 : 0),
						payloadBuffer,
						(sendPayloadByte ? 1 : 0),
						gWarpI2cTimeoutMilliseconds);

	return (status == kStatus_I2C_Success ? kWarpStatusOK : kWarpStatusDeviceCommunicationFailed);
}



WarpStatus
writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength)
{
	uint8_t		inBuffer[payloadLength];
	spi_status_t	status;

	warpEnableSPIpins();
	status = SPI_DRV_MasterTransferBlocking(0					/* master instance */,
						NULL					/* spi_master_user_config_t */,
						payloadBytes,
						inBuffer,
						payloadLength				/* transfer size */,
						gWarpSpiTimeoutMicroseconds		/* timeout in microseconds (unlike I2C which is ms) */);
	warpDisableSPIpins();

	return (status == kStatus_SPI_Success ? kWarpStatusOK : kWarpStatusCommsError);
}



void
powerupAllSensors(void)
{
	/*
	 *	BMX055mag
	 *
	 *	Write '1' to power control bit of register 0x4B. See page 134.
	 */
	#if (WARP_BUILD_ENABLE_DEVBMX055)
		WarpStatus	status = writeByteToI2cDeviceRegister(	deviceBMX055magState.i2cAddress		/*	i2cAddress		*/,
							true					/*	sendCommandByte		*/,
							0x4B					/*	commandByte		*/,
							true					/*	sendPayloadByte		*/,
							(1 << 0)				/*	payloadByte		*/);
		if (status != kWarpStatusOK)
		{
			warpPrint("\r\tPowerup command failed, code=%d, for BMX055mag @ 0x%02x.\n", status, deviceBMX055magState.i2cAddress);
		}
	#else
		warpPrint("\r\tPowerup command failed. BMX055 disabled \n");
	#endif
}



void
activateAllLowPowerSensorModes(bool verbose)
{


	/*
	 *	BMX055accel: At POR, device is in Normal mode. Move it to Deep Suspend mode.
	 *
	 *	Write '1' to deep suspend bit of register 0x11, and write '0' to suspend bit of register 0x11. See page 23.
	 */
	#if WARP_BUILD_ENABLE_DEVBMX055
		WarpStatus	status = writeByteToI2cDeviceRegister(	deviceBMX055accelState.i2cAddress	/*	i2cAddress		*/,
							true					/*	sendCommandByte		*/,
							0x11					/*	commandByte		*/,
							true					/*	sendPayloadByte		*/,
							(1 << 5)				/*	payloadByte		*/);
		if ((status != kWarpStatusOK) && verbose)
		{
			warpPrint("\r\tPowerdown command failed, code=%d, for BMX055accel @ 0x%02x.\n", status, deviceBMX055accelState.i2cAddress);
		}
	#else
		warpPrint("\r\tPowerdown command abandoned. BMX055 disabled\n");
	#endif

	/*
	 *	BMX055gyro: At POR, device is in Normal mode. Move it to Deep Suspend mode.
	 *
	 *	Write '1' to deep suspend bit of register 0x11. See page 81.
	 */
	#if (WARP_BUILD_ENABLE_DEVBMX055)
		status = writeByteToI2cDeviceRegister(	deviceBMX055gyroState.i2cAddress	/*	i2cAddress		*/,
							true					/*	sendCommandByte		*/,
							0x11					/*	commandByte		*/,
							true					/*	sendPayloadByte		*/,
							(1 << 5)				/*	payloadByte		*/);
		if ((status != kWarpStatusOK) && verbose)
		{
			warpPrint("\r\tPowerdown command failed, code=%d, for BMX055gyro @ 0x%02x.\n", status, deviceBMX055gyroState.i2cAddress);
		}
	#else
		warpPrint("\r\tPowerdown command abandoned. BMX055 disabled\n");
	#endif

	#if (WARP_BUILD_ENABLE_DEVL3GD20H)
		status = writeByteToI2cDeviceRegister(	deviceL3GD20HState.i2cAddress	/*	i2cAddress		*/,
							true				/*	sendCommandByte		*/,
							0x20				/*	commandByte		*/,
							true				/*	sendPayloadByte		*/,
							0x00				/*	payloadByte		*/);
		if ((status != kWarpStatusOK) && verbose)
		{
			warpPrint("\r\tPowerdown command failed, code=%d, for L3GD20H @ 0x%02x.\n", status, deviceL3GD20HState.i2cAddress);
		}
	#else
		warpPrint("\r\tPowerdown command abandoned. L3GD20H disabled\n");
	#endif

	#if (WARP_BUILD_ENABLE_DEVTCS34725)
		status = writeByteToI2cDeviceRegister(	deviceTCS34725State.i2cAddress	/*	i2cAddress		*/,
							true				/*	sendCommandByte		*/,
							0x00				/*	commandByte		*/,
							true				/*	sendPayloadByte		*/,
							0x00				/*	payloadByte		*/);
		if ((status != kWarpStatusOK) && verbose)
		{
			warpPrint("\r\tPowerdown command failed, code=%d, for TCS34725 @ 0x%02x.\n", status, deviceTCS34725State.i2cAddress);
		}
	#else
		warpPrint("\r\tPowerdown command abandoned. TCS34725 disabled\n");
	#endif

}
