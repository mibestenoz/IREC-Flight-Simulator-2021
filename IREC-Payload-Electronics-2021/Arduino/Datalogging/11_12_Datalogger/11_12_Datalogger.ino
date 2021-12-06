#include<SD.h> //load the SD library
#include<SPI.h> //load the SPI library
#include <Wire.h> //inputs the wire library for I2C communication
#include <Adafruit_Sensor.h> //Import BNO055 library
#include <Adafruit_BNO055.h> 
#include <utility/imumaths.h> 
#include <math.h> //Import BNO055 library
Adafruit_BNO055 mySensor; //creating sensor object for IMU
Adafruit_BNO055 IMU = Adafruit_BNO055();

float a_x;
float a_y;
float a_z;

float g_x;
float g_y;
float g_z;

unsigned long myTime;

int chipSelect = 4; //CS pin for the SD card reader
File mySensorData; //Data object that will be overwritten with sensor data

void setup() {
  Serial.begin(115200);
  mySensor.begin();

  pinMode(10, OUTPUT); //reserver pin 10 for output due to sd library
  SD.begin(chipSelect);
}

void loop() {

imu::Vector<3> acc = IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
imu::Vector<3> gyr = IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

a_x = acc.x();
a_y = acc.y();
a_z = acc.z();

g_x = gyr.x();
g_y = gyr.y();
g_z = gyr.z();

myTime = millis();
mySensorData = SD.open("test3.txt",FILE_WRITE); //Open SensorData.txt on SD Card

if(mySensorData) { //only writes data if data file opened successfully
delay(10); //delay between readings
mySensorData.print(myTime); //write time value 
mySensorData.print(",");
mySensorData.print(a_x); //write ax value to SD card
mySensorData.print(",");
mySensorData.print(a_y); //write ay value to SD card
mySensorData.print(",");
mySensorData.print(a_z); //write az value to SD card
mySensorData.print(",");
mySensorData.print(g_x); //write gx value to SD card
mySensorData.print(",");
mySensorData.print(g_y); //write gy value to SD card
mySensorData.print(",");
mySensorData.println(g_z); //write gz value to SD card
mySensorData.close(); 
}
}
