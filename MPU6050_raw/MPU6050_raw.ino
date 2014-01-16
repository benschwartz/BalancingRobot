// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro(0x68);
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t ax0, ay0,az0;
int16_t gx0, gy0,gz0;
int16_t Iax, Iay, Iaz;
int16_t Igx, Igy, Igz;
long lastMillis;

#define dirA 12
#define dirB 13
#define speedA 3
#define speedB 11
#define brakeA 9
#define brakeB 8
#define currA A0
#define currB A1

#define KP 0.3
#define KD 20
#define KI 3


void setup() {
    Iax = 0;
    Iay = 0;
    Iaz = 0;
    Igx = 0;
    Igy = 0;
    Igz = 0;
    pinMode(dirA, OUTPUT);
    pinMode(dirB, OUTPUT);
    pinMode(speedA, OUTPUT);
    pinMode(speedB, OUTPUT);
    pinMode(brakeA, OUTPUT);
    pinMode(brakeB, OUTPUT);
    digitalWrite(brakeA,LOW);
    digitalWrite(brakeB,LOW);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(19200);
    
    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    //accelgyro.setRate(38400);
    // verify connection
    Serial.println("Testing device connections...");
    //while(!accelgyro.testConnection()){
    //      accelgyro.initialize();
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    //}
    // use the code below to change accel/gyro offset values
    /*
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax0=ax;
    ay0=ay;
    az0=az;
    gx0=gx;
    gy0=gy;
    gz0=gz;
    // configure Arduino LED for
    lastMillis = millis();
}

void loop() {
    // read raw accel/gyro measurements from device
    long currMillis = millis();
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax-=ax0;
    ay-=ay0;
    az-=az0;
    gx-=gx0;
    gy-=gy0;
    gz-=gz0;
    Iax += ax*(currMillis-lastMillis) / 1000;
    Iay += ay*(currMillis-lastMillis) / 1000;
    Iaz += az*(currMillis-lastMillis) / 1000;
    Igx += gx*(currMillis-lastMillis) / 1000;
    Igy += gy*(currMillis-lastMillis) / 1000;
    Igz += gz*(currMillis-lastMillis) / 1000;
    if(Igx > 0){
      digitalWrite(dirA, LOW);
      digitalWrite(dirB, HIGH); 
    }
    else{
      digitalWrite(dirA, HIGH);
      digitalWrite(dirB, LOW);    
    }
    analogWrite(speedA, min(abs(KP*(double)Igx), 255));
    analogWrite(speedB, min(abs(KP*(double)Igx), 255));
    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);


        // display tab-separated accel/gyro x/y/z values
//        Serial.print("a/g:\t");
        /*Serial.print(ax); Serial.print(",");
        Serial.print(ay); Serial.print(",");
        Serial.print(az); Serial.print(",");*/
        Serial.print(gx); Serial.print(",");
/*        Serial.print(gy); Serial.print(",");
        Serial.print(gz); Serial.print(","); 
        /*Serial.print(Iax); Serial.print(",");
        Serial.print(Iay); Serial.print(",");
        Serial.print(Iaz); Serial.print(","); */
        Serial.println(Igx); //Serial.print(",");
/*        Serial.print(Igy); Serial.print(",");
        Serial.println(Igz);*/

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif
    lastMillis = currMillis;
    // blink LED to indicate activity
}
