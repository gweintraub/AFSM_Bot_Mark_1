#include <Wire.h>
#include <SPI.h>
#include <SFE_LSM9DS0.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_MotorShield.h>
//#include "utility/Adafruit_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

/* Assign a unique base ID for this sensor */   
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);

#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_SCLK, LSM9DS0_MISO, LSM9DS0_MOSI, LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);

float xVal, yVal, zVal;

int set = 0;
const float kp = 9;
const float kd = 7.75;
int error = 0;
int oldError = 0;
float sum = 0.0;
float pAction = 0.00;
float dAction = 0.00;
float Action = 0.00;
float time = 0;
float oldTime = 0;
int index=0;
int nAvg=4;
int de[4]= {0,0,0,0};
float deAvg=0.00;

void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

void displaySensorDetails(void)
{
  sensor_t accel, mag, gyro, temp;
  
  lsm.getSensor(&accel, &mag, &gyro, &temp);
  
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(accel.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(accel.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(accel.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(accel.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(accel.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(accel.resolution); Serial.println(F(" m/s^2"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(mag.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(mag.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(mag.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(mag.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(mag.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(mag.resolution); Serial.println(F(" uT"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(gyro.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(gyro.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(gyro.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(gyro.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(gyro.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(gyro.resolution); Serial.println(F(" rad/s"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(temp.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(temp.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(temp.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(temp.max_value); Serial.println(F(" C"));
  Serial.print  (F("Min Value:    ")); Serial.print(temp.min_value); Serial.println(F(" C"));
  Serial.print  (F("Resolution:   ")); Serial.print(temp.resolution); Serial.println(F(" C"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  
  delay(500);
}

void setup()
{
	Serial.begin(115200);

	if(!lsm.begin())
	{
	  /* There was a problem detecting the LSM9DS0 ... check your connections */
	  Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring!"));
	  while(1);
	}
	Serial.println(F("Found LSM9DS0 9DOF"));
	
	/* Display some basic information on this sensor */
	displaySensorDetails();
	
	/* Setup the sensor gain and integration time */
	configureSensor();
	
	/* We're ready to go! */
	Serial.println("");
}

void loop()
{
	time = millis();
	/* Get a new sensor event */ 
	sensors_event_t accel, mag, gyro, temp;

	lsm.getEvent(&accel, &mag, &gyro, &temp); 
	xVal = gyro.gyro.x;
	yVal = gyro.gyro.y;
	zVal = gyro.gyro.z;

	// print out gyroscopic data
	Serial.print("Gyro  X: "); Serial.print(gyro.gyro.x); Serial.print(" ");
	Serial.print("  \tY: "); Serial.print(gyro.gyro.y);       Serial.print(" ");
	Serial.print("  \tZ: "); Serial.print(gyro.gyro.z);     Serial.println("  \tdps");

	error = zVal - set;
	sum = sum - de[index];
	de[index] = (error-oldError)/((time-oldTime)/1000);  //moving average for dError
	sum = sum + de[index];
	deAvg = sum/nAvg;
	index++;
	if (index >= nAvg)
	{
	  index = 0;
	}

	pAction = kp*error;
	dAction = kd*deAvg;
	Action = (pAction + dAction);
	Action = constrain(Action, -255, 255);

	if(Action > 0){
	    myMotor1->setSpeed(Action);
	    myMotor2->setSpeed(255-Action);
	    myMotor1->run(BACKWARD);
	    myMotor2->run(FORWARD);
	}
	else if (Action < 0){
		myMotor1->setSpeed(255 - Action);
		myMotor2->setSpeed(0 - Action);
		myMotor1->run(FORWARD);
		myMotor2->run(BACKWARD);
	}
	else {
		myMotor1->run(RELEASE);
		myMotor2->run(RELEASE);
	}

	Serial.print("Z = ");
	Serial.print(zVal, DEC);
	Serial.print("  Action: ");
	Serial.println(Action);

	delay(20);
}
