#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

//HX710b
#include "HX711-multi.h"

//BNO055
#include <Adafruit_BNO055.h>

//microSD
#include <SD.h>

//BME680
#include "Adafruit_BME680.h"

//
//initialization
//

//HX710b
#define CLK A5 // sensor clock (on same line)
#define DOUT1 A6 //data output first pressure sensor
#define DOUT2 A7 //data output second pressure sensor
#define PD_SCK
byte DOUTS[2] = {DOUT1, DOUT2};
#define CHANNEL_COUNT sizeof(DOUTS)/sizeof(byte)
long int results[CHANNEL_COUNT];
HX711MULTI hx(CHANNEL_COUNT, DOUTS, CLK);
#define TARE_TIMEOUT_SECONDS 4

//BNO055
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

//microSD
const int chipSelect = 53;
Sd2Card card;
SdVolume volume;
SdFile root;
File mySensorData; 

//BME680
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

//
//constants
//
bool cal = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("--------SENSOR CHECK--------");
  delay(2000);
  //HX710B
  if (hx.is_ready()){
    Serial.println("HX710Bs working");
    pinMode(10, OUTPUT);
    SD.begin(chipSelect);
    delay(1000);  
  }
  //BNO055
  if (bno.begin()) {
    Serial.println("BNO055 working");
    delay(1000);
  }
  //microSD
  if (card.init(SPI_HALF_SPEED, chipSelect)) {
    Serial.println("MicroSD working");
    
    delay(1000);
  }
  //BME680
  if (bme.begin()) {
    Serial.println("BME680 working");
    delay(1000);
  }

  //
  //calibration
  //
  Serial.println("--------CALIBRATION--------");
  //BNO055
  uint8_t system, gyro, accel, mg = 0;
  bno.getCalibration(&system, &gyro, &accel, &mg);
  while (!cal) {
  Serial.print("Gyroscope: "); Serial.println(gyro);
  delay(500);
  if (gyro == 3){
    Serial.println("Gyroscope calibration completed");
    delay(2000);
    cal = 1;
    } 
  }
  cal = 0;

if (system == 1) {
Serial.println("Calibration sequence completed!");
Serial.println("Computer is flight ready");
}

//
//open file on sd
//
mySensorData = SD.open("Flight_Data.txt",FILE_WRITE);
Serial.println("SD Card file 'Flight_Data.txt' opened");
}

//part of pressure sensor example code
void tare() {
  bool tareSuccessful = false;

  unsigned long tareStartTime = millis();
  while (!tareSuccessful && millis()<(tareStartTime+TARE_TIMEOUT_SECONDS*1000)) {
    tareSuccessful = hx.tare(20,10000);  //reject 'tare' if still ringing
  }
}

void sendRawData() {
  hx.read(results);
  for (int i=0; i<hx.get_count(); ++i) {;
    Serial.print( -results[i]);  
    Serial.print( (i!=hx.get_count()-1)?", ":"");
  }  
  delay(1000);
}

void loop() {
//
//raw data from imu
//
imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

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
mySensorData.print(bme.temperature); mySensorData.print(", ");
mySensorData.print(bme.pressure); mySensorData.print(", ");
mySensorData.print(bme.humidity); mySensorData.print(", ");
mySensorData.print(bme.gas_resistance / 1000.0); mySensorData.print(", ");
mySensorData.print(bme.readAltitude(SEALEVELPRESSURE_HPA)); mySensorData.print(", ");
Serial.print( -results[1]);
mySensorData.close();
  }
}
