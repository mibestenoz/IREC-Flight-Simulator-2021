//load relevant libraries
#include <Wire.h> 
#include<SD.h> 
#include<SPI.h>
#include <Adafruit_Sensor.h> 
#include <Adafruit_BNO055.h>
#include "Adafruit_BMP3XX.h"
#include "Adafruit_BME680.h"
#include "HX710B.h"
#include "HX711-multi.h"
#include "HX711.h"


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

   BME680 Connections SPI/I2C
   ==========================

   HX710B Connections I2C
   ======================
   
 */
//BME680
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

//HX710B
#define CLK A3
#define DOUT1 A4
#define DOUT2 A5

//sample rate
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define BMP388_SAMPLERATE_DELAY_MS (100)
#define BME680_SAMPLERATE_DELAY_MS (100)

//creating sensor objects
Adafruit_BNO055 IMU = Adafruit_BNO055(55, 0x28);
Adafruit_BMP3XX bmp;
Adafruit_BME680 bme;


//selecting pins for SD
int chipSelect = 4; 
File mySensorData; 

//HX710B
byte DOUTS[2] = {DOUT1, DOUT2};
#define CHANNEL_COUNT sizeof(DOUTS)/sizeof(byte)
long int results[CHANNEL_COUNT];
HX711MULTI scales(CHANNEL_COUNT, DOUTS, CLK);

void setup() {

//
//turning everything on
//
Serial.begin(115200); 

// BNO055
IMU.begin();

// BMP388
bmp.begin_I2C();

// BME680
bme.begin();

// SD Card
pinMode(10, OUTPUT);
SD.begin(chipSelect);

delay(1000);

}

void sendRawData() {
  scales.read(results);
  for (int i=0; i<scales.get_count(); ++i) {;
    Serial.print( -results[i]);  
    Serial.print( (i!=scales.get_count()-1)?"\t":"\n");
  }  
  delay(1000);
}

void loop() {

//
// initialization
//
uint32_t data_raw = 0;

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
mySensorData.print(bmp.temperature); mySensorData.print(", "); 
mySensorData.print(bmp.pressure); mySensorData.print(", ");
mySensorData.print(bmp.readAltitude(SEALEVELPRESSURE_HPA)); mySensorData.print(", ");
mySensorData.print(bme.temperature); mySensorData.print(", ");
mySensorData.print(bme.pressure); mySensorData.print(", ");
mySensorData.print(bme.humidity); mySensorData.print(", ");
mySensorData.print(bme.gas_resistance / 1000.0); mySensorData.print(", ");
mySensorData.print(bme.readAltitude(SEALEVELPRESSURE_HPA)); mySensorData.print(", ");

mySensorData.print(-results[i]); 
mySensorData.println((i!=scales.get_count()-1)?", ":"");

mySensorData.close();
}

}
