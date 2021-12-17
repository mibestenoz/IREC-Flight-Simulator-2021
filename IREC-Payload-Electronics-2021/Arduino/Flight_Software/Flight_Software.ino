/* TROPOGATOR ROCKET FLIGHT COMPUTER
 * Datalogging program
 * Written by Wiler Sanchez
 */

/*
 * CHANGELOG
 * 12-06-2021 Created program, adapted from LabRatMatt's code from Youtube
 */
//---------load relevant libraries---------
#include <Wire.h>             //Wire library for I2C interface
#include<SD.h>                //SD card library
#include<SPI.h>               //SPI connection library
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

   SD Card Connnections SPI
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

//BMP380
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

//sample rate
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define BMP388_SAMPLERATE_DELAY_MS (100)

//relevent pins
#define chipSelect 4
#define YELLOW_LED_PIN 9

//creating sensor objects
Adafruit_BNO055 IMU = Adafruit_BNO055(55, 0x28);
Adafruit_BMP3XX bmp;

//declare file
File mySensorData;

//declare general variables
bool cal = FALSE; 
//int MODE = 0;           //initialized mode to zero
//int t = 0;              //create timestamp value
//int samplerate = 10;    //specified sampling rate

void setup() {
//
//turning everything on
//
//slight delay before turning on serial monitor
delay(5000);
Serial.begin(115200); 

pinMode(10, OUTPUT);
pinMode(YELLOW_LED_PIN, OUTPUT);
SD.begin(chipSelect);

//
// welcome message
//
Serial.println("-------------------------------------------");
Serial.println("--------TROPOGATOR FLIGHT COMPUTER---------");
Serial.println("-------------------------------------------");
delay(500);

//
// initializing components
//
Serial.println("Commencing initialization sequence...");
delay(500);

//initialize BNO055
if(IMU.begin()){
  Serial.println("BNO055 initialized");
  delay(2000);
}

//initialize BMP388
//if(bmp.begin_I2C()){
//  Serial.println("BMP388 initialized");
//}

//initialize SD Card
if(SD.begin(chipSelect)){
  Serial.println("SD card initialized");
  delay(2000);
  if(!SD.exists("data.txt")){
//clear SD data
  if(SD.exists("data.txt")){
    if(SD.remove("data.txt") == true){
      Serial.println("Previous data cleared");
    }
  }
Serial.println("No data detected in SD card");
delay(2000);
}
}
Serial.println("Initialization sequence completed!");
delay(2000);

//
//calibration
//
uint8_t system, gyro, accel, mg = 0;
IMU.getCalibration(&system, &gyro, &accel, &mg);
Serial.println("Commencing calibration sequence...");
delay(2000);

while(!cal) {
  Serial.print("Gyroscope: "); Serial.println(gyro);
  delay(500);
  if (gyro == 3){
    Serial.println("Gyroscope calibration completed");
    delay(2000);
    cal = TRUE;
  } 
}
cal = FALSE;

while(!cal) {
  Serial.print("Magnetometer: "); Serial.println(mg);
  delay(500);
  if (mg == 3){
    Serial.println("Magnetometer calibration completed");
    delay(2000);
    cal = TRUE;
  } 
}
cal = FALSE;

while(!cal) {
  Serial.print("Accelerometer: "); Serial.println(accel);
  delay(500);
  if (accel == 3){
    Serial.println("Accelerometer calibration completed");
    delay(2000);
    cal = TRUE;
  } 
}
cal = FALSE;

if (system == 3) {
Serial.println("Calibration sequence completed!");
Serial.println("Computer is flight ready");
}

//
//open file on sd
//
mySensorData = SD.open("Flight_Data.txt",FILE_WRITE);
Serial.println("SD Card file 'Flight_Data.txt' opened");


digitalWrite(9, HIGH);
}

void loop() {

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
mySensorData.close();
}

}
