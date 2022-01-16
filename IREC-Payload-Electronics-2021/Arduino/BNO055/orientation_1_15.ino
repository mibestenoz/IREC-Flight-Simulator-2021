#include<SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "Adafruit_BMP3XX.h"
#include "Adafruit_BME680.h"
#include <utility/imumaths.h>
#include <math.h>

//BMP388
#define BMP_SCK 9
#define BMP_MISO 8
#define BMP_MOSI 7
#define BMP_CS 6
#define SEALEVELPRESSURE_HPA (1013.25)

//BME680
#define BME_SCK 5
#define BME_MISO 4
#define BME_MOSI 3
#define BME_CS 2
#define SEALEVELPRESSURE_HPA (1013.25)

//creating sensor object
Adafruit_BNO055 IMU = Adafruit_BNO055();
Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);
Adafruit_BMP3XX bmp;

//declare general variables
bool cal = FALSE; 

float thetaM; //measured theta value
float phiM; //measured phi value
float psiM; //measured psi value

float thetaFold = 0; //filtered theta old value, set to zero initially so that the program doesn't crash
float thetaFnew; //filtered theta new value
float phiFold = 0; //filtered phi old value
float phiFnew; //filtered phi new value
float psiFold = 0; //filtered psi old value
float psiFnew; //filtered psi new value

float thetaG = 0; //measured theta value from gyroscope
float phiG = 0; //measured phi value from gyroscope
float psiG = 0; //measured psi value from gyroscope

float theta; //system theta value
float phi; //system phi value
float psi; //system psi value

float thetarad; //system theta value converted to radians
float phirad; //system phi value converted to radians

float Xm; //x-value magnetometer
float Ym; //y-value magnetometer
float psim; //psi value constructed from magnetometer

float dt; //change of time, present milliseconds - old milliseconds
unsigned long millisOld; //must be reduced


//defining the sample rate delay on the sensor to return data every
//100 milliseconds
#define BNO055_SAMPLERATE_DELAY_MS (10)
#define BMP388_SAMPLERATE_DELAY_MS (10)
#define BME680_SAMPLERATE_DELAY_MS (10)

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200); //we want a higher baud to avoid clutter in the serial monitor
IMU.begin(); //turns the IMU on
delay(1000);

//checking the temperature of the sensor, this has an effect on the
//measurement values, also a good way of checking communication to the IMU
int8_t temp = IMU.getTemp();
Serial.println(temp);

//use the external crystal on the board to conduct the measurement, this 
//will provide more accurate readings
IMU.setExtCrystalUse(true);

millisOld = millis(); //to acquire current millis value

pinMode(13, OUTPUT);

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 m

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  
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
  else {
  Serial.println("BNO055 not initialized");
  delay(2000);
}

//initialize BMP388
if(bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)){
  Serial.println("BMP388 initialized");
  delay(2000); 
}
  else {
  Serial.println("BMP388 not initialized");
  delay(2000);
}

//initialize BME680
if(bme.begin()) {
  Serial.println("BME680 initialized");
  delay(2000);
}
  else { 
  Serial.println("BME680 not initialized");
  delay(2000);
}

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

//while(!cal) {
//  Serial.print("Magnetometer: "); Serial.println(mg);
//  delay(500);
//  if (mg == 3){
//    Serial.println("Magnetometer calibration completed");
//    delay(2000);
//    cal = TRUE;
//  } 
//}
//cal = FALSE;

//while(!cal) {
//  Serial.print("Accelerometer: "); Serial.println(accel);
//  delay(500);
//  if (accel == 3){
//    Serial.println("Accelerometer calibration completed");
//    delay(2000);
//    cal = TRUE;
//  } 
//}
//cal = FALSE;

if (system == 3) {
Serial.println("Calibration sequence completed!");
Serial.println("Computer is flight ready");
}
else {
Serial.println("WARNING: BNO055 not calibrated!");
}
}

void loop() {
bme.performReading();
bmp.performReading();
imu::Vector<3> acc = IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
imu::Vector<3> gyr = IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
imu::Vector<3> mag = IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

//tilt calculations
thetaM = -atan2(acc.x()/9.8,acc.z()/9.8)/2/3.141592654*360; //pitch
phiM = -atan2(acc.y()/9.8,acc.z()/9.8)/2/3.141592654*360; //roll
psiM = atan2(acc.y()/9.8,acc.x()/9.8)/2/3.141592654*360; //yaw

//rudimentary implementation of a low-pass filter
thetaFnew = .95*thetaFold + .05*thetaM;
phiFnew = .95*phiFold + .05*phiM;
psiFnew = .95*psiFold + .05*psiM;

//gyroscope calculation to return orientation angle
dt =(millis()-millisOld)/1000.; //decimal is placed to make sure the value is a float and doesn't create crazy math
millisOld = millis(); //resetting the millis value every loop

//raw gyro orientation calculation values
thetaG = thetaG + gyr.y()*dt;
phiG = phiG - gyr.x()*dt;
psiG = psiG + gyr.z()*dt;

//conversion to radians
thetarad = theta/360*(2*3.141592654);
phirad = phi/360*(2*3.141592654);

//magnetometer psi calculation
Xm = mag.x()*cos(thetarad) - mag.y()*sin(phirad)*sin(thetarad)+mag.z()*cos(phirad)*sin(thetarad); //adding projected values in from 3d to 2d x-y plane
Ym = mag.y()*cos(phirad) + mag.z()*sin(phirad);
psim = atan2(Ym,Xm)/(2*3.141592654)*360;

//complementary filter between gyroscope and accelerometer
theta = (thetaG)*.95 + (thetaM)*.05;
phi = (phiG)*.95 + (phiM)*.05;
psi = (psiG)*.95 + (psiM)*.05;

Serial.println("-------------------BNO055-------------------");
Serial.print("a_x: "); Serial.print(acc.x()); Serial.print(" m/s^2 ");
Serial.print("a_y: "); Serial.print(acc.y()); Serial.print(" m/s^2 ");
Serial.print("a_z: "); Serial.print(acc.z()); Serial.println(" m/s^2");
Serial.print("g_x: "); Serial.print(gyr.x()); Serial.print(" rad/s ");
Serial.print("g_y: "); Serial.print(gyr.y()); Serial.print(" rad/s ");
Serial.print("g_z: "); Serial.print(gyr.z()); Serial.println(" rad/s");
Serial.print("m_x: "); Serial.print(mag.x()); Serial.print(" uT ");
Serial.print("m_y: "); Serial.print(mag.y()); Serial.print(" uT ");
Serial.print("m_z: "); Serial.print(mag.z()); Serial.println(" uT");
Serial.print("Pitch: "); Serial.print(theta); Serial.print(" DEG ");
Serial.print("Roll: "); Serial.print(phi); Serial.print(" DEG ");
Serial.print("Yaw: "); Serial.print(psim); Serial.println(" DEG");
//Serial.print("Pitch(COTS): "); Serial.print(euler.x()); Serial.print(" DEG ");
//Serial.print("Roll(COTS): "); Serial.print(euler.y()); Serial.print(" DEG ");
//Serial.print("Yaw(COTS): "); Serial.print(psim); Serial.println(" DEG");

Serial.println("-------------------BME680-------------------");
Serial.print("Temperature = "); Serial.print(bme.temperature); Serial.println(" *C");
Serial.print("Pressure = "); Serial.print(bme.pressure); Serial.println(" Pa");
Serial.print("Humidity = "); Serial.print(bme.humidity); Serial.println(" %");
Serial.print("Approx. Altitude = "); Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA)); Serial.println(" m");

Serial.println("-------------------BMP388-------------------");
Serial.print("Temperature = "); Serial.print(bmp.temperature); Serial.println(" *C");
Serial.print("Pressure = "); Serial.print(bmp.pressure); Serial.println(" Pa");
Serial.print("Humidity = "); Serial.print(bme.humidity); Serial.println(" %");
Serial.print("Approx. Altitude = "); Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA)); Serial.println(" m\n");
delay(1000);
}
