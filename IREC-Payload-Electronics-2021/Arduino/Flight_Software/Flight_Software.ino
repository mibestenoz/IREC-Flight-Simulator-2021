//load relevant libraries
#include <Wire.h> 
#include<SD.h> 
#include<SPI.h>
#include <Adafruit_Sensor.h> 
#include <Adafruit_BNO055.h>
#include "Adafruit_BMP3XX.h"

/*
   IMU Connections I2C
   ===================
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground
  
   Altimeter Connections I2C
   =========================
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect Vin to 3.3-5V DC
   Connect GROUND to common ground

   SD Connnections SPI
   ===================
   Connect CLK to digital 13
   Connect DO to digital 12
   Connect DI to digital 11
   Connect CS to digital 10
   Connect 5V to 5V DC
   Connect GROUND to common ground

   GPS Connections 
   ===============
   Connect TX
   Connect RX
   Connect Vin to 3.3-5V DC
   Connect GROUND to common ground
 */
#include <utility/imumaths.h>
#include <math.h>

//BMP
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

//sample rate
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define BMP388_SAMPLERATE_DELAY_MS (100)

//creating sensor objects
Adafruit_BNO055 IMU = Adafruit_BNO055(55, 0x28);
Adafruit_BMP3XX bmp;


//selecting pins for SD
int chipSelect = 4; 
File mySensorData; 

void setup() {

//
//turning everything on
//
Serial.begin(115200); 

IMU.begin();

bmp.begin_I2C();
  
pinMode(10, OUTPUT);
SD.begin(chipSelect);

delay(1000);


}

void loop() {

//
//calibration
//
uint8_t system, gyro, accel, mg = 0;
IMU.getCalibration(&system, &gyro, &accel, &mg);
if ((accel < 3) && (gyro < 3) && (mg < 3) && (system < 3)) {
  Serial.print(accel); Serial.print(gyro); 
  Serial.print(mg); Serial.println(system);
}

//
//open file on sd
//
mySensorData = SD.open("Flight_Data.txt",FILE_WRITE);

//
//raw data from imu
//
imu::Vector<3> acc = IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
imu::Vector<3> gyr = IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
imu::Vector<3> mag = IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

//
// writing data to sensor
//
if(mySensorData) {
mySensorData.print(acc.x()); mySensorData.print(", "); 
mySensorData.print(acc.y()); mySensorData.print(", "); 
mySensorData.print(acc.z()); mySensorData.print(", "); 
mySensorData.print(gyr.x()); mySensorData.print(", "); 
mySensorData.print(gyr.y()); mySensorData.print(", "); 
mySensorData.print(gyr.z()); mySensorData.print(", ");
mySensorData.print(mag.x()); mySensorData.print(", "); 
mySensorData.print(mag.y()); mySensorData.print(", "); 
mySensorData.print(mag.z()); mySensorData.print(", ");
mySensorData.print(bmp.temperature); mySensorData.print(", "); 
mySensorData.print(bmp.pressure); mySensorData.print(", ");
mySensorData.print(bmp.readAltitude(SEALEVELPRESSURE_HPA)); mySensorData.print(", ");
mySensorData.close();
}

}
