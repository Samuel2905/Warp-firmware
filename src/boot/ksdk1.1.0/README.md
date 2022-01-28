## Lake Water Level sensor and pump
Samuel Clarke - sc2101 - Pembroke College

This project uses an analogue water level sensor to turn a pump on and off to keep a lake at a constant, pre-set level.
The voltage from the level sensor is read by the microcontrollers ADC, which uses a digital output to turn a relay on or off, controlling the pump
The project also contains an external RTC (RV8803C7) to get an accurate time reference, as well as a temperature sensor (BME680)
3 LEDs are also used as outputs

The system is based on the Warp firmware, with the setup and main program code boot.c
It uses the dev8803C7 and devBME680 driver files from Warp (with some modifications), as well as new driver files devADC and devRelay
The file structure is the same as Warp, so the aforementioned files are contained in `src\boot\ksdk1.1.0` and the program is compiled using the top level Makefile  

## Source File Descriptions

##### `boot.c`
Contains setup code and functions from the Warp firmware
Lines 1372-1471 is the setup code specific to this program
Lines 1473-1557 contains the code to take and print measurements and control the outputs, this is looped over
Also contains the `wait_time()` function to calculate the time to wait until taking next measurement

##### `devRV8803C7.c`
Contains an additional function `bcd2bin()` to convert bcd values from the RTC back to decimal
Also contains `RegistersToBin()` which reads the date and time registers and creates a `rtc_datetime_t1` structure
Line 332 contains a modification to `setRTCTimeRV8803C7()` as the clock wasn't being restarted correctly

##### `devBME680.c`
`configureSensorBME680()` was modified to include reading the calibration values stored in registers on the sensor
These values are then used in `calc_temperature()` to convert the `unsignedRawAdcValue` for temperature into centigrade

##### `devRelay.c`
This driver file sets the required pins to outputs for the relay and 3 LEDs
It also contains functions to turn the LEDs on and off, which are simple to call from `boot.c`

##### `devADC.c`
This sets up the ADC that is on the microcontroller into one-time trigger mode
`readADC()` takes a measurement and returns the raw value (from 0 to 4095)
`Voltage_ADC` converts the raw value to a voltage
`level()` takes several time averaged measurements and then calculates and return the water level in mm
