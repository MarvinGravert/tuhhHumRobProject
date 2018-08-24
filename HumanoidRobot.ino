// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       HumanoidRobot.ino
    Created:	12.08.2018 18:26:52
    Author:     LaptopGravert\MarvinGravert
*/

//#include "i2c_t3.h"
#include <i2c_t3.h>
#include "MadgwickAHRS.h"
#include "MPU9250.h"
//#include <Wire.h>


// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
											  // There is a tradeoff in the beta parameter between accuracy and response speed.
											  // In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
											  // However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
											  // Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
											  // By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
											  // I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
											  // the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
											  // In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value


uint32_t Now = 0;        // used to calculate integration interval

float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };    // vector to hold quaternion

float pitch, yaw, roll, pitch2, roll2, yaw2,roll3,pitch3,yaw3;
float deltaT = 0.0f; // integration interval for both filter schemes
uint32_t lastUpdate = 0; // used to calculate integration interval

MPU9250 mySensor(Wire);//PIN  SCL 19 SDA 18
MPU9250 mySensor2(Wire1);//PIN SCL 37 SDA 38
MPU9250 mySensor3(Wire2);//PIN SCL 3 SDA 4

MadgwickAHRS fusion1;
MadgwickAHRS fusion2;
MadgwickAHRS fusion3;

float accVal[3],accVal2[3],accVal3[3];
float gyroVal[3],gyroVal2[3],gyroVal3[3];
float magVal[3],magVal2[3],magVal3[3];


uint32_t currentTime, lastTimeAcc,lastTimeGyro,lastTimeMag,outputTimer,lastTimeSampledSens1 ,lastTimeSampledSens2 ,lastTimeSampledSens3 ;
size_t gyro1SampleDelay = 0,gyro2SampleDelay = 0,gyro3SampleDelay = 0;

uint32_t startTime = 0, accEndTime = 0, gyroEndTime = 0, magEndTime = 0;

uint8_t loopCounter = 0;
typedef union
{
	float number;
	uint8_t bytes[4];
} FLOATUNION_t;

bool fused = false;


//void setup2()
//{
//
//	Serial.begin(230400);
//	Wire.begin();
//	while (!Serial);
//	Wire.setClock(400000);
//
//	mySensor.initMPU();
//
//	mySensor.readAcc(&accVal[0]);
//	mySensor.readGyro(&gyroVal[0]);
//	mySensor.readMag(&magVal[0]);
//
//
//	lastTimeAcc = micros();
//	ax = accVal[0];
//	ay = accVal[1];
//	az = accVal[2];
//	gx = gyroVal[0];
//	gy = gyroVal[1];
//	gz = gyroVal[2];
//	mx = magVal[0];
//	my = magVal[1];
//	mz = magVal[2];
//}
//void loop2()
//{
//	loopCounter++;
//	currentTime = micros();
//
//	if ((currentTime - lastTimeAcc) > 6000) {
//		mySensor.readAcc(&accVal[0]);
//		mySensor.readGyro(&gyroVal[0]);
//		ax = accVal[0];
//		ay = accVal[1];
//		az = accVal[2];
//		gx = gyroVal[0];
//		gy = gyroVal[1];
//		gz = gyroVal[2];
//		lastTimeAcc = currentTime;
//	}
//
//
//	if ((currentTime - lastTimeMag)> 10000) {
//		mySensor.readMag(&magVal[0]);
//		mx = magVal[0];
//		my = magVal[1];
//		mz = magVal[2];
//		lastTimeMag = currentTime;
//	}
//
//
//	Now = micros();
//	/*while ((Now - lastUpdate) < 80000) {
//	Now = micros();
//	}*/
//	deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
//	lastUpdate = Now;
//
//	//startTime = micros();
//	MadgwickQuaternionUpdate(ax, ay, az, gx*PI / 180.0f, gy*PI / 180.0f, gz*PI / 180.0f, my, mx, mz);
//
//
//	////OUTPUT
//	//if (Serial.available() > 0) {
//	//	yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
//	//	pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
//	//	roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
//	//	pitch *= 180.0f / PI;
//	//	yaw *= 180.0f / PI;
//	//	//yaw -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
//	//	roll *= 180.0f / PI;
//	//
//	//	Serial.print(roll);
//	//	Serial.print(" ");
//	//	Serial.print(pitch);
//	//	Serial.print(" ");
//	//	Serial.println(yaw);
//	//	clearSerialBuffer();
//	//}
//	if (loopCounter > 3) {
//		yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
//		pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
//		roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
//		pitch *= 180.0f / PI;
//		yaw *= 180.0f / PI;
//		roll *= 180.0f / PI;
//
//		/*send2Simulink(roll);
//		send2Simulink(pitch);
//		send2Simulink(yaw);*/
//		//Serial.print('\n');
//
//		Serial.print(roll);
//		Serial.print(" ");
//		Serial.print(pitch);
//		Serial.print(" ");
//		Serial.println(yaw);
//
//		//clearSerialBuffer();
//		loopCounter = 0;
//	}
//
//
//
//}
void setup()
{
	Serial.begin(230400);
	bool doCalibration = false;
	while (!Serial);
	Wire.begin();//PINS 19/18  SCL:19 SDA 18
	Wire1.begin();//PINS 37/38  SLC 37 SDA 38
	//Wire2.begin();//PINS 3/4  SCL 3 SDA 4
	Wire.setClock(400000L);
	Wire1.setClock(400000L);
	//Wire2.setClock(400000L);
	Serial.println(mySensor.getAddress());
	delay(2000);

	mySensor.initMPU();
	mySensor2.initMPU();
	//mySensor3.initMPU();

	
	//either calibrate gyroscope and magnometer or use values from previous calibration
	if (doCalibration) {
		Serial.print("First the gyroscopes of all Sensors will be calibrated dont move the sensors");
		delay(2000);
		mySensor.calibrateGyro();
		mySensor2.calibrateGyro();
		//mySensor3.calibrateGyro();
		Serial.println("Gyroscope calibration done. Printing values");
		Serial.println("Offset of gyroscope of Sensor on Pin 18/19");
		mySensor.printGyroOffset();
		Serial.println("Offset of gyroscope of Sensor on Pin 37/38");
		mySensor2.printGyroOffset();
		/*Serial.println("Offset of gyroscope of Sensor on Pin 3/4");
		mySensor3.printGyroOffset();*/
		delay(8000);
		Serial.println("Now the magnometer will be calibrated, starting with the sensor on Port 18/19");
		Serial.println("Get ready to move the sensor in elliptical curves. Start in roughly 5 seconds");
		mySensor.calibrateMag();
		Serial.println("First is done");
		delay(1000);
		Serial.println("Now the magnometer of the sensor on Port 37/38 will be calibrated");
		Serial.println("Get ready. Start in roughly 5seconds");
		mySensor2.calibrateMag();
		Serial.println("Second is done");
		delay(1000);
		/*Serial.println("Now the magnometer of the sensor on Port 3/4 will be calibrated");
		Serial.println("Get ready. Start in roughly 5seconds");
		mySensor3.calibrateMag();
		Serial.println("Third is done");*/
		Serial.println("Now the values will be printed");
		Serial.println("Sensor on Pin 18/19:");
		mySensor.printMagCalibration();
		Serial.println("Sensor on Pin 37/38");
		mySensor2.printMagCalibration();
	/*	Serial.println("Sensor on Pin 3/4");
		mySensor3.printMagCalibration();*/
		Serial.println("Thats all calibration. Procedding as normal in 20seconds");
		delay(20000);

	}
	else {
		//in the following the values for the offsets and scale of the sensors derived from previous calibration are entered  
		//Sensor 1 
		float mag1ErrorScale[] = { 1.04f,1.04f,0.96f };
		float mag1Offset[] = { 17.81f,-13.4f,-26.77f };
		float gyro1Offset[] = { 3.71f,1.11,-0.06 };

		//Sensor 2
		float mag2ErrorScale[] = { 1.09f,0.96f,0.96f };
		float mag2Offset[] = { 55.02f,20.32f,-72.29f };
		float gyro2Offset[] = { 1.52f,0.48f, -0.77f };

		//Sensor 3
		float mag3ErrorScale[] = { 1.09f,0.96f,0.96f };
		float mag3Offset[] = { 55.02f,20.32f,-72.29f };
		float gyro3Offset[] = { 1.52f,0.48f, -0.77f };

		//Set the offset and errorscales
		mySensor.setMagCalibration(&mag1ErrorScale[0], &mag1Offset[0]);
		mySensor.printMagCalibration();
		mySensor2.setMagCalibration(&mag2ErrorScale[0], &mag2Offset[0]);
		mySensor2.printMagCalibration();
		//mySensor3.setMagCalibration(&mag3ErrorScale[0], &mag3Offset[0]);
		//mySensor3.printMagCalibration();
		Serial.println("30seconds to compare, for debuggig purposes");
		delay(30000);
		
		mySensor.setGyroOffset(&gyro1Offset[0]);
		mySensor2.setGyroOffset(&gyro2Offset[0]);
		//mySensor3.setGyroOffset(&gyro3Offset[0]);
	}
	delay(500);

	mySensor.readAcc(&accVal[0]);
	mySensor.readGyro(&gyroVal[0]);
	mySensor.readMag(&magVal[0]);

	mySensor2.readAcc(&accVal2[0]);
	mySensor2.readGyro(&gyroVal[0]);
	mySensor2.readMag(&magVal2[0]);

	/*mySensor3.readAcc(&accVal3[0]);
	mySensor3.readGyro(&gyroVal3[0]);
	mySensor3.readMag(&magVal3[0]);*/


	//different times due to possibly different lowpass filter
	gyro1SampleDelay = mySensor.getGyroSampleDelayInMicro();
	gyro2SampleDelay = mySensor2.getGyroSampleDelayInMicro();
	//gyro3SampleDelay = mySensor3.getGyroSampleDelayInMicro();

	currentTime = micros();
	lastTimeSampledSens1 = currentTime;
	lastTimeSampledSens2 = currentTime;
	lastTimeSampledSens3 = currentTime;
	lastTimeMag = currentTime;
	outputTimer = currentTime;
}
void loop()
{	
	currentTime = micros();
	//Sensor 1 Pin 19/18
	if ((currentTime - lastTimeSampledSens1) > gyro1SampleDelay) {
		mySensor.readGyro(&gyroVal[0]);
		mySensor.readAcc(&accVal[0]);
		lastTimeSampledSens1 = currentTime;
	}
	//Sensor 2 Pin 37/38
	if ((currentTime - lastTimeSampledSens2) > gyro2SampleDelay) {
		mySensor2.readGyro(&gyroVal2[0]);
		mySensor2.readAcc(&accVal2[0]);
		lastTimeSampledSens2 = currentTime;
	}
	//Sensor 3 Pin 3/4
	/*if ((currentTime - lastTimeSampledSens3) > gyro3SampleDelay) {
		mySensor3.readGyro(&gyroVal3[0]);
		mySensor3.readAcc(&accVal3[0]);
		lastTimeSampledSens3 = currentTime;
	}*/

	if ((currentTime-lastTimeMag)> 10000 ) {//mag updates with 100Hz=>0.01s aka 10000 us
		mySensor.readMag(&magVal[0]);
		mySensor2.readMag(&magVal2[0]);
		//mySensor3.readMag(&magVal3[0]);
		lastTimeMag = currentTime;
	}

	currentTime = micros();
	
	deltaT = ((currentTime - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
	lastUpdate = currentTime;
	
	//try deltat=gyroSampledelay/try different calculation/try different sqrt/try different beta
	fusion1.update(gyroVal[0], gyroVal[1], gyroVal[2], accVal[0], accVal[1], accVal[2], magVal[0], magVal[1], magVal[2], deltaT);
	fusion2.update(gyroVal2[0], gyroVal2[1], gyroVal2[2], accVal2[0], accVal2[1], accVal2[2], magVal2[0], magVal2[1], magVal2[2], deltaT);
	//fusion3.update(gyroVal3[0], gyroVal3[1], gyroVal3[2], accVal3[0], accVal3[1], accVal3[2], magVal3[0], magVal3[1], magVal3[2], deltat);
	
	if (lastUpdate-outputTimer > 100000L){
		/*yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
		pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
		roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]); 
		pitch *= 180.0f / PI;
		yaw *= 180.0f / PI;
		roll *= 180.0f / PI;*/	
		roll = fusion1.getRoll();
		pitch = fusion1.getPitch();
		yaw = fusion1.getYaw();
		roll2 = fusion2.getRoll();
		pitch2 = fusion2.getPitch();
		yaw2 = fusion2.getYaw();
		/*roll3 = fusion3.getRoll();
		pitch3 = fusion3.getPitch();
		yaw3 = fusion3.getYaw();*/

		//For MATLAB
		//set a minus to allign the KOS
		/*send2Simulink(roll);
		send2Simulink(pitch);
		send2Simulink(-yaw);
		send2Simulink(roll2);
		send2Simulink(pitch2);
		send2Simulink(-yaw2);*/
		/*send2Simulink(roll3);
		send2Simulink(pitch3);
		send2Simulink(-yaw3);*/
		Serial.print('\n');

		//FOR SERIAL MONITOR

		Serial.print(roll);
		Serial.print(" ");
		Serial.print(pitch);
		Serial.print(" ");
		Serial.print(yaw);
		Serial.print(" ");
		Serial.print(roll2);
		Serial.print(" ");
		Serial.print(pitch2);
		Serial.print(" ");
		Serial.print(yaw2);
		Serial.print('\n');

	/*	int16_t test[3];
		int16_t test2[3];
		mySensor.readRawGyro(&test[0]);
		mySensor2.readRawGyro(&test2[0]);
		Serial.print(test[0]);
		Serial.print(" ");
		Serial.print(test[1]);
		Serial.print(" ");
		Serial.print(test[2]);
		Serial.print(" ");
		Serial.print(test2[0]);
		Serial.print(" ");
		Serial.print(test2[1]);
		Serial.print(" ");
		Serial.print(test2[2]);
		Serial.print('\n');*/

		/*Serial.print(gyroVal[0]);
		Serial.print(" ");
		Serial.print(gyroVal[1]);
		Serial.print(" ");
		Serial.print(gyroVal[2]);
		Serial.print(" ");
		Serial.print(gyroVal2[0]);
		Serial.print(" ");
		Serial.print(gyroVal2[1]);
		Serial.print(" ");
		Serial.print(gyroVal2[2]);
		Serial.print('\n'); */
		//clearSerialBuffer();
		outputTimer = lastUpdate;
	}
}	
//void setup0()
//{
//	Serial.begin(230400);
//	Wire.begin();
//	Wire1.begin();
//	while (!Serial);
//	Wire.setClock(400000);
//	Wire1.setClock(400000);
//
//	mySensor.initMPU();
//	mySensor2.initMPU();
//
//	mySensor._magErrorScale[0] = 1.04f;
//	mySensor._magErrorScale[1] = 1.04f;
//	mySensor._magErrorScale[2] = 0.96f;
//
//	mySensor._magOffset[0] = 17.81f;
//	mySensor._magOffset[0] = -13.4f;
//	mySensor._magOffset[0] = -26.77f;
//
//	mySensor2._magErrorScale[0] = 1.09f;
//	mySensor2._magErrorScale[1] = 0.96f;
//	mySensor2._magErrorScale[2] = 0.96f;
//
//	mySensor2._magOffset[0] = 55.02f;
//	mySensor2._magOffset[0] = 20.32;
//	mySensor2._magOffset[0] = -72.29f;
//
//
//	mySensor._gyroOff[0] = 3.71f;
//	mySensor._gyroOff[1] = 1.11;
//	mySensor._gyroOff[2] = -0.06f;
//
//	mySensor._gyroOff[0] = 1.52f;
//	mySensor._gyroOff[1] = 0.48;
//	mySensor._gyroOff[2] = -0.77f;
//
//
//
//
//	mySensor.readAcc(&accVal[0]);
//	mySensor.readGyro(&gyroVal[0]);
//	mySensor.readMag(&magVal[0]);
//
//	mySensor2.readAcc(&accVal2[0]);
//	mySensor2.readGyro(&gyroVal[0]);
//	mySensor2.readMag(&magVal2[0]);
//
//	currentTime = micros();
//	lastTimeSampled = currentTime;
//	outputTimer = currentTime;
//}
//void loop0()
//{
//	currentTime = micros();
//
//	if ((currentTime - lastTimeSampled) > 10000) {
//		mySensor.readAcc(&accVal[0]);
//		mySensor2.readAcc(&accVal2[0]);
//		mySensor.readGyro(&gyroVal[0]);
//		mySensor2.readGyro(&gyroVal2[0]);
//
//		mySensor.readMag(&magVal[0]);
//		mySensor2.readMag(&magVal2[0]);
//		
//		lastTimeSampled = currentTime;
//		fused = false;
//	}
//
//	/*if ((currentTime - lastTimeMag)> 10000) {
//		mySensor.readMag(&magVal[0]);
//		mySensor2.readMag(&magVal2[0]);
//		lastTimeMag = currentTime;
//	}
//*/
//
//	//Now = micros();
//	///*while ((Now - lastUpdate) < 80000) {
//	//Now = micros();
//	//}*/
//	//deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
//	//lastUpdate = Now;
//	deltat = lastTimeSampled / 1000000.0f;
//	if (!fused) {
//		fusion1.update(gyroVal[0], gyroVal[1], gyroVal[2], accVal[0], accVal[1], accVal[2], magVal[0], magVal[1], magVal[2], deltat);
//		fusion2.update(gyroVal2[0], gyroVal2[1], gyroVal2[2], accVal2[0], accVal2[1], accVal2[2], magVal2[0], magVal2[1], magVal2[2], deltat);
//		fused = true;
//	}
//	
//
//	////OUTPUT
//	//if (Serial.available() > 0) {
//	//	yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
//	//	pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
//	//	roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
//	//	pitch *= 180.0f / PI;
//	//	yaw *= 180.0f / PI;
//	//	//yaw -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
//	//	roll *= 180.0f / PI;
//	//
//	//	Serial.print(roll);
//	//	Serial.print(" ");
//	//	Serial.print(pitch);
//	//	Serial.print(" ");
//	//	Serial.println(yaw);
//	//	clearSerialBuffer();
//	//}
//	if (currentTime - outputTimer > 100000L) {
//		/*yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
//		pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
//		roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
//		pitch *= 180.0f / PI;
//		yaw *= 180.0f / PI;
//		roll *= 180.0f / PI;*/
//
//
//
//		roll = fusion1.getRoll();
//		pitch = fusion1.getPitch();
//		yaw = fusion1.getYaw();
//		roll2 = fusion2.getRoll();
//		pitch2 = fusion2.getPitch();
//		yaw2 = fusion2.getYaw();
//
//		//For MATLAB
//		//set a minus to allign the KOS
//		send2Simulink(roll);
//		send2Simulink(pitch);
//		send2Simulink(-yaw);
//		send2Simulink(roll2);
//		send2Simulink(pitch2);
//		send2Simulink(-yaw2);
//		Serial.print('\n');
//
//		//FOR SERIAL MONITOR
//
//		//Serial.print(roll);
//		//Serial.print(" ");
//		//Serial.print(pitch);
//		//Serial.print(" ");
//		//Serial.print(yaw);
//		//Serial.print(" ");
//		//Serial.print(roll2);
//		//Serial.print(" ");
//		//Serial.print(pitch2);
//		//Serial.print(" ");
//		//Serial.print(yaw2);
//		//Serial.print('\n');
//
//		/*	int16_t test[3];
//		int16_t test2[3];
//		mySensor.readRawGyro(&test[0]);
//		mySensor2.readRawGyro(&test2[0]);
//		Serial.print(test[0]);
//		Serial.print(" ");
//		Serial.print(test[1]);
//		Serial.print(" ");
//		Serial.print(test[2]);
//		Serial.print(" ");
//		Serial.print(test2[0]);
//		Serial.print(" ");
//		Serial.print(test2[1]);
//		Serial.print(" ");
//		Serial.print(test2[2]);
//		Serial.print('\n');*/
//
//		/*Serial.print(gyroVal[0]);
//		Serial.print(" ");
//		Serial.print(gyroVal[1]);
//		Serial.print(" ");
//		Serial.print(gyroVal[2]);
//		Serial.print(" ");
//		Serial.print(gyroVal2[0]);
//		Serial.print(" ");
//		Serial.print(gyroVal2[1]);
//		Serial.print(" ");
//		Serial.print(gyroVal2[2]);
//		Serial.print('\n'); */
//		//clearSerialBuffer();
//		loopCounter = 0;
//		outputTimer = currentTime;
//	}
//
//
//
//
//}

void i2cScanner() {
	byte error, address;
	int nDevices;

	Serial.println("Scanning...");

	nDevices = 0;
	for (address = 1; address < 127; address++)
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire1.beginTransmission(address);
		error = Wire.endTransmission();

		if (error == 0)
		{
			Serial.print("I2C device found at address 0x");
			if (address<16)
				Serial.print("0");
			Serial.print(address, HEX);
			Serial.println("  !");

			nDevices++;
		}
		else if (error == 4)
		{
			Serial.print("Unknown error at address 0x");
			if (address<16)
				Serial.print("0");
			Serial.println(address, HEX);
		}
	}
	if (nDevices == 0)
		Serial.println("No I2C devices found\n");
	else
		Serial.println("done\n");

	delay(5000);           // wait 5 seconds for next scan
}

void send2Simulink(float tt) {
	FLOATUNION_t fa;
	fa.number = tt;
	for (int i = 0; i<4; i++)
	{
		Serial.write(fa.bytes[i]);
	}
}
// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrtf(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrtf(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrtf(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f / norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * deltaT;
	q2 += qDot2 * deltaT;
	q3 += qDot3 * deltaT;
	q4 += qDot4 * deltaT;
	norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f / norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
}