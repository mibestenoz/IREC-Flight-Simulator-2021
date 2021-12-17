#include<SD.h> 
#include<SPI.h>
#include <Wire.h>
#include "HX710B.h"
#define SCK_PIN 3 
#define SDI_PIN 4

Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);
Adafruit_BME680 bme; // I2C

HX710B air_press(SCK_PIN,SDI_PIN);

unsigned long myTime;

int chipSelect = 4; //CS pin for the SD card reader
File mySensorData; //Data object that will be overwritten with sensor data

void setup() {
  Serial.begin(115200);
  
  if (!air_press.init())
  {
    Serial.println(F("HX710 not found!"));
  }
  if (!SD.begin())
  {
    Serial.println(F("SD Card not found!"));
  }
  if (!bme.performReading())
  {
    Serial.println("BME680 not found!");
  }
  pinMode(10, OUTPUT); //reserver pin 10 for output due to sd library
  SD.begin(chipSelect);
  air_press.init();

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void loop() {
myTime = millis();
uint32_t data_raw = 0;
mySensorData = SD.open("data.txt",FILE_WRITE); //Open SensorData.txt on SD Card
if(mySensorData) { //only writes data if data file opened successfully
  delay(10); //delay between readings
  mySensorData.print(myTime); //write time value 
  mySensorData.print(",");
  mySensorData.println((unsigned long) data_raw); //write gz value to SD card
  mySensorData.close(); 
}
}
