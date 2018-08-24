#include "MPU9250.h"

uint8_t MPU9250::readRegister(uint8_t deviceAddress, uint8_t reg) {
	_Wire.beginTransmission(deviceAddress);
	_Wire.write(reg);
	_Wire.endTransmission(false);

	uint8_t data;
	_Wire.requestFrom(deviceAddress, (uint8_t) 1);
	data = _Wire.read();
	return data;
}

void MPU9250::readRegisters( uint8_t deviceAddress,uint8_t firstRegister, uint8_t numRegisters, uint8_t * store){
		_Wire.beginTransmission(deviceAddress);   
		_Wire.write(firstRegister);            
		_Wire.endTransmission(false);  
		uint8_t i = 0;
		_Wire.requestFrom(deviceAddress, numRegisters);  // Read bytes from slave register address 
		while (_Wire.available()) {
			store[i++] = _Wire.read();
		}         // Put read results in the Rx buffer
}

void MPU9250::writeRegister( uint8_t deviceAddress, uint8_t reg, uint8_t whatToWrite)
{
	_Wire.beginTransmission(deviceAddress);
	_Wire.write(reg);
	_Wire.write(whatToWrite);
	_Wire.endTransmission();
}

void MPU9250::readAcc( float * storage)
{
	int16_t rawData[3];
	readRawAcc(&rawData[0]);
	storage[0] = rawData[0] * _accRes*_accErrorScale[0] - _accOffset[0];
	storage[1] = rawData[1] * _accRes*_accErrorScale[1] - _accOffset[1];
	storage[2] = rawData[2] * _accRes*_accErrorScale[2] - _accOffset[2];
}

void MPU9250::readRawAcc(int16_t * storage)
{
	uint8_t rawData[6];
	readRegisters(_activeMPUAddress ,ACCEL_XOUT_H, 6, &rawData[0]);
	storage[0] = ((int16_t)rawData[0] << 8) | rawData[1]; 
	storage[1] = ((int16_t)rawData[2] << 8) | rawData[3];
	storage[2] = ((int16_t)rawData[4] << 8) | rawData[5];
}

void MPU9250::readGyro(float * storage)
{
	int16_t rawData[3];
	readRawGyro(&rawData[0]);
	storage[0] = rawData[0] * _gyroRes - _gyroOffset[0];
	storage[1] = rawData[1] * _gyroRes - _gyroOffset[1];
	storage[2] = rawData[2] * _gyroRes - _gyroOffset[2];
}

void MPU9250::readRawGyro(int16_t * storage)
{
	uint8_t rawData[6];
	readRegisters(_activeMPUAddress,GYRO_XOUT_H, 6, &rawData[0]);
	storage[0] = ((int16_t)rawData[0] << 8) | rawData[1];
	storage[1] = ((int16_t)rawData[2] << 8) | rawData[3];
	storage[2] = ((int16_t)rawData[4] << 8) | rawData[5];
}

void MPU9250::readMag(float * storage)
{
	int16_t rawData[3];
	readRawMag(&rawData[0]);
	storage[0] = (float)rawData[0] * _magRes*_magSensAdjust[0]*_magErrorScale[0] - _magOffset[0];
	storage[1] = (float)rawData[1] * _magRes*_magSensAdjust[1]*_magErrorScale[1] - _magOffset[1];
	storage[2] = (float)rawData[2] * _magRes*_magSensAdjust[2]*_magErrorScale[2] - _magOffset[2];
}

void MPU9250::readRawMag(int16_t * storage)
{
	uint8_t rawData[7];
	readRegisters(_ADDRESS_MAG, MAG_XOUT_L, 7, &rawData[0]);
	storage[0] = ((int16_t)rawData[1] << 8) | rawData[0];
	storage[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	storage[2] = ((int16_t)rawData[5] << 8) | rawData[4];
}

void MPU9250::readAccGyro(float * storage)
{
	/*uint8_t rawData[14];
	readRegisters(_activeMPUAddress, ACCEL_XOUT_H, 14, &rawData[0]);
	storage[0] = ((int16_t)rawData[0] << 8) | rawData[1];
	storage[1] = ((int16_t)rawData[2] << 8) | rawData[3];
	storage[2] = ((int16_t)rawData[4] << 8) | rawData[5];
	storage[4] = ((int16_t)rawData[8] << 8) | rawData[9];
	storage[5] = ((int16_t)rawData[10] << 8) | rawData[11];
	storage[6] = ((int16_t)rawData[12] << 8) | rawData[13];*/

	readAcc(&storage[0]);
	readGyro(&storage[3]);


}

bool MPU9250::initMPU()
{
	//check if MPU can be found at given address and responds correctly
	if (readRegister(_activeMPUAddress, WHO_AM_I_MPU9250) != 0x71) {
		Serial.println("MPU NOT FOUND");
		return false;
	}
	//Serial.println("Found MPU");
	writeRegister(_activeMPUAddress, PWR_MGMT_1, 0x81);//(reset all registers)/wake up device
	delay(100);
	//Serial.println("Reset");
	writeRegister(_activeMPUAddress, PWR_MGMT_1, 0x01);//set a clock source, to be safe 
	delay(10);
	//Serial.println("ClockSource");
	writeRegister(_activeMPUAddress, PWR_MGMT_2, 0x00);//enable all axis, to be safe
	delay(10);
	//Serial.println("Enabled");
	writeRegister(_activeMPUAddress, INT_PIN_CFG, 2);//enable bypass to make magnometer accessable
	delay(10);
	//Serial.println("Enabling Bypass");
	/*if (readRegister(_activeMPUAddress, INT_PIN_CFG) == 2) {
		Serial.println("Bypass enabled");

	}	else{
		Serial.println("Bypass not enabled");

	}*/
		//Gyroscope Lowpass filter 
	setLowPassMode(_lowPassModeAcc, _lowPassModeGyro);
	setScaleAndResolution(_accScale, _gyroScale, _magBit);

	initMag();

	return true;
}

void MPU9250::changeMPUAddress()
{
	if (_activeMPUAddress == _ADDRESS_AD0) _activeMPUAddress = _ADDRESS_AD1;

	if (_activeMPUAddress == _ADDRESS_AD1) _activeMPUAddress = _ADDRESS_AD0;
}

void MPU9250::setLowPassMode(uint8_t accMode, uint8_t gyroMode)
{	//accMode:depending on the number set in the header set the lowpass for the acc
	//gyroMode: depending on the number set in the header set the lowpass for the gyro
	//TODO: use enum, and set up methods to interact with the mode from the main

	//hereafter the register entries to set the lowpass for the acc are done
	switch (accMode){
	case 0://no lowpass
		writeRegister(_activeMPUAddress, ACCEL_CONFIG2, 0x08);
		_accSampleDelayInMicro =(size_t) 750;
		break;
	case 1://bandwidth 480Hz
		writeRegister(_activeMPUAddress, ACCEL_CONFIG2, 0x00);
		_accSampleDelayInMicro = (size_t)1940;
		break;
	case 2://bandwdith 184Hz
		writeRegister(_activeMPUAddress, ACCEL_CONFIG2, 0x01);
		_accSampleDelayInMicro = (size_t)5800;
		break;
	case 3://bandwdith 92Hz
		writeRegister(_activeMPUAddress, ACCEL_CONFIG2, 0x02);
		_accSampleDelayInMicro = (size_t)7800;
		break;
	case 4://bandwidth 41Hz
		writeRegister(_activeMPUAddress, ACCEL_CONFIG2, 0x03);
		_accSampleDelayInMicro = (size_t)11800;
		break;
	case 5: //bandwidth 20Hz
		writeRegister(_activeMPUAddress, ACCEL_CONFIG2, 0x04);
		_accSampleDelayInMicro = (size_t)19800;
		break;
	case 6: //bandwidth 10Hz
		writeRegister(_activeMPUAddress, ACCEL_CONFIG2, 0x05);
		_accSampleDelayInMicro = (size_t)35700;
		break;
	case 7: //bandwidth 5Hz
		writeRegister(_activeMPUAddress, ACCEL_CONFIG2, 0x06);
		_accSampleDelayInMicro = (size_t)66960;
		break;
	case 8:
		writeRegister(_activeMPUAddress, ACCEL_CONFIG2, 0x07);
		_accSampleDelayInMicro = (size_t)1940;
		break;
	default:
		Serial.print("Invalid AccMode");
	}
	uint8_t temp;
	//hereafter the register entries to set the lowpass for the gyro are done.
	//To not change the values that are  already present in the register we store the
	//byte and conduct the approriate bitmath to get the desired result
	switch (gyroMode) {
	case 0://no lowPass bandwidth 8800Hz(11 or 01 for fchoiceb and whatever for dlpf_cfg
		temp = readRegister(_activeMPUAddress, GYRO_CONFIG);
		writeRegister(_activeMPUAddress, GYRO_CONFIG, 0x03 | temp);
		break;
	case 1://bandwidth 3600Hz (10 for fchoiceb and whatever for dlpf_cfg
		temp = readRegister(_activeMPUAddress, GYRO_CONFIG);
		temp = B11111100 & temp;
		writeRegister(_activeMPUAddress, GYRO_CONFIG, 0x02 | temp);
		break;
	case 2://bandwidth 250Hz (00 for fchoiceb and 000 for dlpf_cfg
		temp = readRegister(_activeMPUAddress, GYRO_CONFIG);
		writeRegister(_activeMPUAddress, GYRO_CONFIG, B11111100 & temp);
		temp = readRegister(_activeMPUAddress, CONFIG);
		writeRegister(_activeMPUAddress, CONFIG, B11111000 & temp);
		break;
	case 3://bandwidth 184Hz (00 for fchoiceb 001 for dlpf_cfg
		temp = readRegister(_activeMPUAddress, GYRO_CONFIG);
		writeRegister(_activeMPUAddress, GYRO_CONFIG, B11111100 & temp);
		temp = readRegister(_activeMPUAddress, CONFIG);
		temp = B11111000 & temp;
		writeRegister(_activeMPUAddress, CONFIG, B00000001 | temp);
		break;
	case 4://bandwidth 92Hz(00 for fchoiceb 010 for dlpf_cfg
		temp = readRegister(_activeMPUAddress, GYRO_CONFIG);
		writeRegister(_activeMPUAddress, GYRO_CONFIG, B11111100 & temp);
		temp = readRegister(_activeMPUAddress, CONFIG);
		temp = B11111000 & temp;
		writeRegister(_activeMPUAddress, CONFIG, B00000010 | temp);
		break;
	case 5://bandwidth 41Hz (00 for fchoiceb and 011 for dlpf_cfg
		temp = readRegister(_activeMPUAddress, GYRO_CONFIG);
		writeRegister(_activeMPUAddress, GYRO_CONFIG, B11111100 & temp);
		temp = readRegister(_activeMPUAddress, CONFIG);
		temp = B11111000 & temp;
		writeRegister(_activeMPUAddress, CONFIG, B00000011 | temp);
		break;
	case 6://bandwidth 20Hz (00 for fchoiceb and 100 for dlpf_cfg
		temp = readRegister(_activeMPUAddress, GYRO_CONFIG);
		writeRegister(_activeMPUAddress, GYRO_CONFIG, B11111100 & temp);
		temp = readRegister(_activeMPUAddress, CONFIG);
		temp = B11111000 & temp;
		writeRegister(_activeMPUAddress, CONFIG, B00000100 | temp);
		break;
	case 7://bandwidth 10Hz (00 for fchoiceb and 101 for dlpf_cfg
		temp = readRegister(_activeMPUAddress, GYRO_CONFIG);
		writeRegister(_activeMPUAddress, GYRO_CONFIG, B11111100 & temp);
		temp = readRegister(_activeMPUAddress, CONFIG);
		temp = B11111000 & temp;
		writeRegister(_activeMPUAddress, CONFIG, B00000101 | temp);
		break;
	case 8://bandwidth 5Hz (00 for fchoiceb and 110 for dlpf_cfg
		temp = readRegister(_activeMPUAddress, GYRO_CONFIG);
		writeRegister(_activeMPUAddress, GYRO_CONFIG, B11111100 & temp);
		temp = readRegister(_activeMPUAddress, CONFIG);
		temp = B11111000 & temp;
		writeRegister(_activeMPUAddress, CONFIG, B00000110 | temp);
		break;
	case 9://bandwidth 3600Hz (00 for fchoiceb and 111 for dlpf_cfg
		temp = readRegister(_activeMPUAddress, GYRO_CONFIG);
		writeRegister(_activeMPUAddress, GYRO_CONFIG, B11111100 & temp);
		temp = readRegister(_activeMPUAddress, CONFIG);
		temp = B11111000 & temp;
		writeRegister(_activeMPUAddress, CONFIG, B00000111 | temp);
		break;
	default:
		Serial.print("Invalid GyroMode");
	}

}

void MPU9250::setScaleAndResolution(uint8_t accScale, uint16_t gyroScale, uint8_t magBit)
{
	volatile uint8_t temp,temp2;
	switch (accScale){
	case 2:
		_accScale = 2;
		temp = 0x00;
		break;
	case 4:
		_accScale = 4;
		temp = 0x08;
		break;
	case 6: 
		_accScale = 6;
		temp = 0x10;
		break;
	case 8:
		_accScale = 8;
		temp = 0x18;
		break;
	default:
		Serial.println("Invalid AccScale");
	}
	_accRes = _accScale / 32768.f;
	writeRegister(_activeMPUAddress, 28, temp);

	temp = readRegister(_activeMPUAddress, 27);
	writeRegister(_activeMPUAddress, 27, temp &B11100111);
	switch (gyroScale) {
	case 250:
		_gyroScale = 250;
		temp2 = B00000000;
		break;
	case 500:
		_gyroScale = 500;
		temp2 = B00001000;
		break;
	case 1000:
		_gyroScale = 1000;
		temp2 = B00010000;
		break;
	case 2000:
		_gyroScale = 2000;
		temp2 = B00011000;
		break;
	default:
		Serial.println("Invalid GyroScale");
	}
	writeRegister(_activeMPUAddress,27, temp | temp2);
	_gyroRes = _gyroScale / 32768.f;

	switch (magBit) {
	case 14:
		_magRes = 4912.F / 8190.f;
	case 16:
		_magRes = 4912.F / 32760.0f;

	}

}

uint8_t MPU9250::getAddress()
{
	uint8_t test = _activeMPUAddress;//forgot to change during refactor, now ill leave it for the laughs
	return test;
}

void MPU9250::initMag()
{
	uint8_t rawData[3];  // x/y/z gyro calibration data stored here
	writeRegister(_ADDRESS_MAG, MAG_CNTL1, 0x00); // Power down magnetometer  
	delay(10);
	writeRegister(_ADDRESS_MAG, MAG_CNTL1, 0x0F); // Enter Fuse ROM access mode
	delay(10);
	readRegisters(_ADDRESS_MAG, MAG_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
	_magSensAdjust[0] = (float)(rawData[0] - 128) / 256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
	_magSensAdjust[1] = (float)(rawData[1] - 128) / 256. + 1.;
	_magSensAdjust[2] = (float)(rawData[2] - 128) / 256. + 1.;
	writeRegister(_ADDRESS_MAG, MAG_CNTL1, 0x00); // Power down magnetometer  
	delay(10);
	//Continous Read  100Hz and 16bit /14bit
	if (_magBit == 16) {
		writeRegister(_ADDRESS_MAG, MAG_CNTL1, B10110); // Set magnetometer data resolution and sample ODR
	}
	else {
		writeRegister(_ADDRESS_MAG, MAG_CNTL1, B00110); // Set magnetometer data resolution and sample ODR
	}
	
	delay(10);
}

void MPU9250::calibrateGyro()
{
	//if the sensor isnt moving the gyro should measure zero angular velocity if not there is an offset
	//hence we measure the gyro output and average it if its (there should be) non zero mean, thats our offset
	float offset[3] = { 0,0,0 };

	float temp[3];
	for (int i = 0; i < 500; ++i) {

	readGyro(&temp[0]);
	offset[0] += temp[0] / 500;
	offset[1] += temp[1] / 500;
	offset[2] += temp[2] / 500;
	delayMicroseconds(_gyroSampleDelayInMicro);
	}

	_gyroOffset[0] = offset[0];
	_gyroOffset[1] = offset[1];
	_gyroOffset[2] = offset[2];
	/*Serial.println("Gyroscope Offset");
	Serial.print(offset[0]);
	Serial.print(" ");
	Serial.print(offset[1]);
	Serial.print(" ");
	Serial.println(offset[2]);
*/
	
}

void MPU9250::calibrateMag()
{	//this is a slightly modified version from the one from
	//@krisWiner
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = { 0, 0, 0 }, mag_scale[3] = { 0, 0, 0 };
	int16_t mag_max[3] = { -32767, -32767, -32767 }, mag_min[3] = { 32767, 32767, 32767 }, mag_temp[3] = { 0, 0, 0 };

	
	delay(5000);
	Serial.println("START");
	delay(500);
	
	if (_magBit == 14) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
	if (_magBit == 16) sample_count = 2000;  // at 100 Hz ODR, new mag data is available every 10 ms
	
	for (ii = 0; ii < sample_count; ii++) {
		readRawMag(&mag_temp[0]);  // Read the mag data   
		for (int jj = 0; jj < 3; jj++) {
			if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}
		if (_magBit == 14) delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
		if (_magBit == 16) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
	}


	// Get hard iron correction
	mag_bias[0] = (mag_max[0] + mag_min[0]) / 2;  // get average x mag bias in counts
	mag_bias[1] = (mag_max[1] + mag_min[1]) / 2;  // get average y mag bias in counts
	mag_bias[2] = (mag_max[2] + mag_min[2]) / 2;  // get average z mag bias in counts

	_magOffset[0] = (float)mag_bias[0] * _magRes*_magSensAdjust[0];  // save mag biases in G for main program
	_magOffset[1] = (float)mag_bias[1] * _magRes*_magSensAdjust[1];
	_magOffset[2] = (float)mag_bias[2] * _magRes*_magSensAdjust[2];

	// Get soft iron correction estimate
	mag_scale[0] = (mag_max[0] - mag_min[0]) / 2;  // get average x axis max chord length in counts
	mag_scale[1] = (mag_max[1] - mag_min[1]) / 2;  // get average y axis max chord length in counts
	mag_scale[2] = (mag_max[2] - mag_min[2]) / 2;  // get average z axis max chord length in counts

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	_magErrorScale[0] = avg_rad / ((float)mag_scale[0]);
	_magErrorScale[1] = avg_rad / ((float)mag_scale[1]);
	_magErrorScale[2] = avg_rad / ((float)mag_scale[2]);

	//Serial.println("Mag Calibration done!");
	/*for (int i = 0; i < 3; ++i) {
		Serial.print(_magErrorScale[i], DEC);
		Serial.print("   ");
	}
	for (int i = 0; i < 2; ++i) {
		Serial.print(_magOffset[i], DEC);
		Serial.print("   ");
	}
	Serial.println(_magOffset[2], DEC);*/
}

void MPU9250::printMagCalibration()
{
	Serial.println("ErrorScale and Offset of the magnometer");
	for (uint8_t i = 0; i < 3; i++) {
		Serial.print(_magErrorScale[i]);
		Serial.print(" ");
	}
	for (uint8_t i = 0; i < 3; i++) {
		Serial.print(_magOffset[i]);
		Serial.print(" ");
	}
	Serial.print("\n");

}

void MPU9250::setMagCalibration(float * errorScale, float * offset)
{
	for (uint8_t i = 0; i < 3; i++) {
		_magErrorScale[i] = errorScale[i];
		_magOffset[i] = offset[i];
	}
}

void MPU9250::printGyroOffset()
{
	Serial.println("The offset fo the gyroscope");
	for (uint8_t i = 0; i < 3; i++) {
		Serial.print(_gyroOffset[i]);
		Serial.print(" ");
	}
	Serial.print("\n");
}

void MPU9250::setGyroOffset(float * gyroOffset)
{
	for (uint8_t i = 0; i < 3; i++) {
		_gyroOffset[i] = gyroOffset[i];
	}
}

uint32_t MPU9250::getGyroSampleDelayInMicro()
{
	return _gyroSampleDelayInMicro;
}

uint32_t MPU9250::getAccSampleDelayInMicro()
{
	return _accSampleDelayInMicro;
}