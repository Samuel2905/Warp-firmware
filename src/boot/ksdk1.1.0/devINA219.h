void		initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);
WarpStatus	readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload, uint16_t menuI2cPullupValue);
WarpStatus  configureSensorINA219(uint16_t payloadConfig_Reg, uint16_t payloadCalibration, uint16_t menuI2cPullupValue);
void		printSensorDataINA219(bool hexModeFlag);
