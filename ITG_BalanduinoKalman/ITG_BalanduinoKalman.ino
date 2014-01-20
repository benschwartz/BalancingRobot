/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.
 
 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").
 
 Contact information
 -------------------
 
 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

#define dirA 12
#define dirB 13
#define speedA 3
#define speedB 11
#define brakeA 9
#define brakeB 8
#define currA A0
#define currB A1

#define KP 75
#define KD 35 
#define KI 00
 
int16_t angle[2]; // pitch & roll
 
// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();
float rawSixDof[6];
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

Kalman kalman2X;
Kalman kalman2Y;

/* IMU Data */
int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;

double accXangle, accYangle; // Angle calculate using the accelerometer
double temp; // Temperature
double gyroXangle, gyroYangle; // Angle calculate using the gyro
double compAngleX, compAngleY; // Calculate the angle using a complementary filter
double kalAngleX, kalAngleY; // Calculate the angle using a Kalman filter

/* IMU Data 2*/
int16_t acc2X, acc2Y, acc2Z;
int16_t tempRaw2;
int16_t gyro2X, gyro2Y, gyro2Z;

double acc2Xangle, acc2Yangle; // Angle calculate using the accelerometer
double temp2; // Temperature
double gyro2Xangle, gyro2Yangle; // Angle calculate using the gyro
double compAngle2X, compAngle2Y; // Calculate the angle using a complementary filter
double kalAngle2X, kalAngle2Y; // Calculate the angle using a Kalman filter

//PID controls
double dAngle;
double intAngle;
double lastAngle;
double Igx;
double v_e;
boolean fell_over;

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

void setup() {  
  Serial.begin(115200);
  pinMode(dirA, OUTPUT);
  pinMode(dirB, OUTPUT);
  pinMode(speedA, OUTPUT);
  pinMode(speedB, OUTPUT);
  pinMode(brakeA, OUTPUT);
  pinMode(brakeB, OUTPUT);
  digitalWrite(brakeA,LOW);
  digitalWrite(brakeB,LOW);
  fell_over = false;
  
  Wire1.begin();
  Wire.begin();
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while(i2cWrite(0x19,i2cData,4,false)); // Write to all four registers at once
  while(i2cWrite(0x6B,0x01,true)); // PLL with X axis gyroscope reference and disable sleep mode 
  
  //Sensor Init
  sixDOF.init();                        //begin the IMU
  delay(5);

  //PID controls
  dAngle = 0;
  intAngle = 0;
  lastAngle = 0;
  
  while(i2cRead(0x75,i2cData,1));
  if(i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while(1);
  }
  
  delay(100); // Wait for sensor to stabilize
  
  /* Set kalman and gyro starting angle */
  sixDOF.getRawValues(rawSixDof);
  while(i2cRead(0x3B,i2cData,6));
  acc2X = rawSixDof[0];//((i2cData[0] << 8) | i2cData[1]);
  acc2Y = rawSixDof[1];//((i2cData[2] << 8) | i2cData[3]);
  acc2Z = rawSixDof[2];//((i2cData[4] << 8) | i2cData[5]);
  
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  acc2Yangle = (atan2(acc2X,acc2Z)+PI)*RAD_TO_DEG;
  acc2Xangle = (atan2(acc2Y,acc2Z)+PI)*RAD_TO_DEG;
  
  kalmanX.setAngle(accXangle); // Set starting angle
  kalmanY.setAngle(accYangle);
  gyroXangle = accXangle;
  gyroYangle = accYangle;
  compAngleX = accXangle;
  compAngleY = accYangle;
  
  kalman2X.setAngle(acc2Xangle); // Set starting angle
  kalman2Y.setAngle(acc2Yangle);
  gyro2Xangle = acc2Xangle;
  gyro2Yangle = acc2Yangle;
  compAngle2X = acc2Xangle;
  compAngle2Y = acc2Yangle;
  
  timer = micros();
  Serial.println("done setup");
}

void loop() {
  /* Update all the values */  
  sixDOF.getRawValues(rawSixDof);
  while(i2cRead(0x3B,i2cData,14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = ((i2cData[6] << 8) | i2cData[7]);  
  gyroX = ((i2cData[8] << 8) | i2cData[9]);
  gyroY = ((i2cData[10] << 8) | i2cData[11]);
  gyroZ = ((i2cData[12] << 8) | i2cData[13]);
  
  acc2X = rawSixDof[0];//((i2cData[0] << 8) | i2cData[1]);
  acc2Y = rawSixDof[1];//((i2cData[2] << 8) | i2cData[3]);
  acc2Z = rawSixDof[2];//((i2cData[4] << 8) | i2cData[5]);
//  tempRaw = ((i2cData[6] << 8) | i2cData[7]);  
  gyro2X = rawSixDof[3];//((i2cData[8] << 8) | i2cData[9]);
  gyro2Y = rawSixDof[4];//((i2cData[10] << 8) | i2cData[11]);
  gyro2Z = rawSixDof[5];//((i2cData[12] << 8) | i2cData[13]);
  
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
  acc2Xangle = (atan2(acc2Y,acc2Z)+PI)*RAD_TO_DEG;
  acc2Yangle = (atan2(acc2X,acc2Z)+PI)*RAD_TO_DEG;
  
  double gyroXrate = (double)gyroX/131.0;
  double gyroYrate = -((double)gyroY/131.0);
  gyroXangle += gyroXrate*((double)(micros()-timer)/1000000); // Calculate gyro angle without any filter  
  gyroYangle += gyroYrate*((double)(micros()-timer)/1000000);
  //gyroXangle += kalmanX.getRate()*((double)(micros()-timer)/1000000); // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate()*((double)(micros()-timer)/1000000);
  
  double gyro2Xrate = (double)gyro2X/131.0;
  double gyro2Yrate = -((double)gyro2Y/131.0);
  gyro2Xangle += gyro2Xrate*((double)(micros()-timer)/1000000); // Calculate gyro angle without any filter  
  gyro2Yangle += gyro2Yrate*((double)(micros()-timer)/1000000);
  
  compAngleX = (0.93*(compAngleX+(gyroXrate*(double)(micros()-timer)/1000000)))+(0.07*accXangle); // Calculate the angle using a Complimentary filter
  compAngleY = (0.93*(compAngleY+(gyroYrate*(double)(micros()-timer)/1000000)))+(0.07*accYangle);
  
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000); // Calculate the angle using a Kalman filter
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/1000000);

//2
  compAngle2X = (0.93*(compAngle2X+(gyro2Xrate*(double)(micros()-timer)/1000000)))+(0.07*acc2Xangle); // Calculate the angle using a Complimentary filter
  compAngle2Y = (0.93*(compAngle2Y+(gyro2Yrate*(double)(micros()-timer)/1000000)))+(0.07*acc2Yangle);
  
  kalAngle2X = kalman2X.getAngle(acc2Xangle, gyro2Xrate, (double)(micros()-timer)/1000000); // Calculate the angle using a Kalman filter
  kalAngle2Y = kalman2Y.getAngle(acc2Yangle, gyro2Yrate, (double)(micros()-timer)/1000000);  
  
  //temp = ((double)tempRaw + 12412.0) / 340.0;
  
  //fallen over?
  if(abs(kalAngleX -180) > 45){
    fell_over = true;
  }
  
  //PID on kalAngleX
  Igx = kalAngleX - 178;
  dAngle = kalAngleX - lastAngle;
  lastAngle = kalAngleX;
  intAngle += Igx*(double)(micros() - timer)/1000000;
  v_e = KP*Igx/**abs(Igx)*/ + KD*dAngle + KI*intAngle;
  if(v_e > 0){
    digitalWrite(dirA, LOW);
    digitalWrite(dirB, HIGH); 
  }
  else{
    digitalWrite(dirA, HIGH);
    digitalWrite(dirB, LOW);    
  }
  if(fell_over){
    analogWrite(speedA, 0);
    analogWrite(speedB, 0);
  }
  else {
    analogWrite(speedA, min(abs(v_e), 255));
    analogWrite(speedB, min(abs(v_e), 255));
  }
  
  timer = micros();
  
  /* Print Data */
  /*
  Serial.print(accX);Serial.print(",");
  Serial.print(accY);Serial.print(",");
  Serial.print(accZ);Serial.print(",");
  
  Serial.print(gyroX);Serial.print(",");
  Serial.print(gyroY); Serial.print(",");
  Serial.print(gyroZ);Serial.println("");
  */
  Serial.print(accXangle);Serial.print(",");
  Serial.print(gyroXangle);Serial.print(",");
  Serial.print(compAngleX);Serial.print(",");
  Serial.print(kalAngleX);Serial.print(",");
  
  Serial.print(acc2Xangle);Serial.print(",");
  Serial.print(gyro2Xangle);Serial.print(",");
  Serial.print(compAngle2X);Serial.print(",");
  Serial.print(kalAngle2X);Serial.println("");
  
//  Serial.print("\t");
  /*
  Serial.print(accYangle);Serial.print(",");
  Serial.print(gyroYangle);Serial.print(",");
  Serial.print(compAngleY); Serial.print(",");
  Serial.print(kalAngleY);Serial.println("\t");
  */
  //Serial.print(temp);Serial.print("\t");
   
  //Serial.print("\r\n");
  //delay(1);
}
