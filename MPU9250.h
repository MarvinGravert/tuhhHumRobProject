// MPU9250.h

#ifndef _MPU9250_h
#define _MPU9250_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
//Accelerometer/Gyroscope Sensor Registry
#define SELF_TEST_X_GYRO 0x00                  
#define SELF_TEST_Y_GYRO 0x01                                                                          
#define SELF_TEST_Z_GYRO 0x02

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E   
#define WOM_THR          0x1F   

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B  //ACC
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43  //GYRP
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L 0x7E
//Magnometer aka AK8963 Registry 
#define MAG_WHO_AM_I  0x00 // should return 0x48
#define MAG_INFO      0x01
#define MAG_ST1       0x02  // data ready status bit 0
#define MAG_XOUT_L	 0x03  //lower byte x direction
#define MAG_XOUT_H	 0x04  //higher byte x direction
#define MAG_YOUT_L	 0x05 
#define MAG_YOUT_H	 0x06
#define MAG_ZOUT_L	 0x07
#define MAG_ZOUT_H	 0x08
#define MAG_ST2       0x09  // Status 2  this has to be read so that the mag knows it has ot update the measurement entries
#define MAG_CNTL1      0x0A  //Control 1 
#define MAG_CNTL2	0x0B//Control 2  soft reset
#define MAG_ASTC      0x0C  // Self test control
#define MAG_I2CDIS    0x0F  // I2C disable
#define MAG_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define MAG_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define MAG_ASAZ	  0x12 // Fuse ROM z-axis sensitivity adjustment value

class MPU9250
{
 
 public:
	 uint8_t readRegister(uint8_t, uint8_t);//w
	 void readRegisters( uint8_t, uint8_t, uint8_t, uint8_t*);//w

	 void writeRegister(uint8_t, uint8_t, uint8_t);//w

	 void readAcc( float*);
	 void readRawAcc(int16_t*);
	 void readGyro(float*);
	 void readRawGyro(int16_t*);//w
	 void readMag(float*);
	 void readRawMag(int16_t*);

	 void readAccGyro(float*);

	 bool initMPU();

	 void changeMPUAddress();


	 void setLowPassMode(uint8_t, uint8_t);

	 void setScaleAndResolution(uint8_t, uint16_t, uint8_t);

	 uint8_t getAddress();
	
	 void i2cScanner();

	 void initMag();

	 void calibrateGyro();

	 void calibrateMag();

	 float _magOffset[3];
	 float _magErrorScale[3];
	 
 private:
	 

	 const uint8_t _ADDRESS_AD0 = 0x68;
	 const uint8_t _ADDRESS_AD1 = 0x69;
	 const uint8_t _ADDRESS_MAG = 0x0C;//AK8963 =MAG

	 uint8_t _activeMPUAddress=_ADDRESS_AD0;
	 uint8_t _accScale = 2;
	 uint16_t _gyroScale = 250;
	 //uint16_t _magScale = 4800;
	 uint8_t _magBit = 16;

	 float _accRes, _gyroRes, _magRes;
	 const float _gyroOffset[3] = { -1.63333f,2.56453f,-0.014816f };
	 //const float _gyroOffset[3] = { 0.f,0.f,0.f };
	 //const float _accOffset[3] = { -0.040248f,-0.013765f,0.029220f };
	 const float _accOffset[3] = { 0.0f,0.0f,0.0f };
	 //const float _accErrorScale[3] = { 0.997761f,0.996936f,0.983371f };//roughly from Matlab
	 const float _accErrorScale[3] = { 1.0f,1.0f,1.0f };//roughly from Matlab

	 const float _magOff[3] = { -1.64464f,25.1956f,-36.55816f };
	 //const float _magOff[3] = { 0.0f,0.0f,0.0f };
	 const float _magError[3] = { 1.07888f,0.97809f,0.95174f };
	 //const float _magError[3] = { 0.0f,0.0f,0.0f };
	 float _magSensAdjust[3];
	





	 uint8_t _lowPassModeAcc = 4;
	/*
	 No LowPass / Mode 0 Bandwidth = 1.13kHz timedelay = 0.75ms intSample = 4kHz
	 Mode 1: bw = 460 ;td = 1.94 intSample = 1kHz
	 Mode 2 : bw = 184 ;td = 5.80 intSample = 1kHz
	 Mode 3 : bw = 92; td = 7.80 intSample = 1kHz
	 Mode 4 : bw = 41 ;td = 11.80 intSample = 1kHz
	 Mode 5 : bw = 20 ;td = 19.80 intSample = 1kHz
	 Mode 6 : bw = 10 ;td = 35.70 intSample = 1kHz
	 Mode 7 : bw = 5; td = 66.96 intSample = 1kHz
	 Mode 8 : bw = 460 td = 1.94 intSample = 1kHz %same as mode 1
	 */

	 uint8_t _lowPassModeGyro=5;//prob consider enum
	 /* 
	 No LowPass / Mode 0:Bandwidth = 8800Hz  timedelay = 0.064ms intSample 32mkHz
	 Mode 1 : bw = 3600 td = 0.11ms intSample 32kHz
	 Mode 2 : bw = 250 td = 0.97ms intSample 8
	 Mode 3 : bw = 184 td = 2.9ms intSample 1
	 Mode 4 : bw = 92 td = 3.9ms intSample 1
	 Mode 5 : bw = 41 td = 5.9ms intSample 1
	 Mode 6 : bw = 20 td = 9.9ms intSample 1
	 Mode 7 : bw = 10 td = 17.85ms intSample 1
	 Mode 8 : bw = 5 td = 33.48ms  intSample 1
	 Mode 9 : bw = 3600 td = 0.17ms intSample 8 % dunno why this exist its worse than mode 3
	  */

	 


};


#endif

