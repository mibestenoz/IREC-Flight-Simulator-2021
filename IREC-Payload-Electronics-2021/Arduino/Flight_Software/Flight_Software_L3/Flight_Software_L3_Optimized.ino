#include <Wire.h> 
#include<SD.h> 
#include<SPI.h>
#include <Adafruit_Sensor.h> 
#include <Adafruit_BNO055.h>
#include "Adafruit_BMP3XX.h"
#include "Adafruit_BME680.h"
#include "HX710B.h"
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SCK_PIN 3
#define SDI_PIN 4
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define BMP388_SAMPLERATE_DELAY_MS (100)
#define BME680_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 IMU = Adafruit_BNO055(55, 0x28);
Adafruit_BMP3XX bmp;
Adafruit_BME680 bme;
int chipSelect = 4; 
File mySensorData; 
HX710B air_press(SCK_PIN,SDI_PIN);
void setup() {
Serial.begin(115200); 
IMU.begin();
bmp.begin_I2C();
bme.begin();
pinMode(10, OUTPUT);
SD.begin(chipSelect);
delay(1000);
}
void loop() {
uint32_t data_raw = 0;
uint8_t system, gyro, accel, mg = 0;
IMU.getCalibration(&system, &gyro, &accel, &mg);
if ((accel < 3) && (gyro < 3) && (mg < 3) && (system < 3)) {
  Serial.print(accel); Serial.print(gyro); 
  Serial.print(mg); Serial.println(system);
}
mySensorData = SD.open("Flight_Data.txt",FILE_WRITE);
imu::Vector<3> acc = IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
imu::Vector<3> gyr = IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
imu::Vector<3> mag = IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
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
mySensorData.print(bme.temperature); mySensorData.print(", ");
mySensorData.print(bme.pressure); mySensorData.print(", ");
mySensorData.print(bme.humidity); mySensorData.print(", ");
mySensorData.print(bme.gas_resistance / 1000.0); mySensorData.print(", ");
mySensorData.print(bme.readAltitude(SEALEVELPRESSURE_HPA)); mySensorData.print(", ");
mySensorData.println((unsigned long) data_raw);
mySensorData.close();
}
}
