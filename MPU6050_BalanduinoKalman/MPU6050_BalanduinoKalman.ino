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

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;

double accXangle, accYangle; // Angle calculate using the accelerometer
double temp; // Temperature
double gyroXangle, gyroYangle; // Angle calculate using the gyro
double compAngleX, compAngleY; // Calculate the angle using a complementary filter
double kalAngleX, kalAngleY; // Calculate the angle using a Kalman filter

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
  
  Wire.begin();
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while(i2cWrite(0x19,i2cData,4,false)); // Write to all four registers at once
  while(i2cWrite(0x6B,0x01,true)); // PLL with X axis gyroscope reference and disable sleep mode 

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
  while(i2cRead(0x3B,i2cData,6));
  Serial.println("Hello 3");
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  
  kalmanX.setAngle(accXangle); // Set starting angle
  kalmanY.setAngle(accYangle);
  gyroXangle = accXangle;
  gyroYangle = accYangle;
  compAngleX = accXangle;
  compAngleY = accYangle;
  
  timer = micros();
  Serial.println("done setup");
}

void loop() {
  /* Update all the values */  
  while(i2cRead(0x3B,i2cData,14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = ((i2cData[6] << 8) | i2cData[7]);  
  gyroX = ((i2cData[8] << 8) | i2cData[9]);
  gyroY = ((i2cData[10] << 8) | i2cData[11]);
  gyroZ = ((i2cData[12] << 8) | i2cData[13]);
  
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
  
  double gyroXrate = (double)gyroX/131.0;
  double gyroYrate = -((double)gyroY/131.0);
  gyroXangle += gyroXrate*((double)(micros()-timer)/1000000); // Calculate gyro angle without any filter  
  gyroYangle += gyroYrate*((double)(micros()-timer)/1000000);
  //gyroXangle += kalmanX.getRate()*((double)(micros()-timer)/1000000); // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate()*((double)(micros()-timer)/1000000);
  
  compAngleX = (0.93*(compAngleX+(gyroXrate*(double)(micros()-timer)/1000000)))+(0.07*accXangle); // Calculate the angle using a Complimentary filter
  compAngleY = (0.93*(compAngleY+(gyroYrate*(double)(micros()-timer)/1000000)))+(0.07*accYangle);
  
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000); // Calculate the angle using a Kalman filter
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/1000000);
  
  
  temp = ((double)tempRaw + 12412.0) / 340.0;
  
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
  Serial.print(kalAngleX);Serial.println("");
  
//  Serial.print("\t");
  /*
  Serial.print(accYangle);Serial.print(",");
  Serial.print(gyroYangle);Serial.print(",");
  Serial.print(compAngleY); Serial.print(",");
  Serial.print(kalAngleY);Serial.println("\t");
  */
  //Serial.print(temp);Serial.print("\t");
   
  //Serial.print("\r\n");
  delay(1);
}
