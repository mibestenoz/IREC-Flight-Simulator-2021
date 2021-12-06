//by denoting a '#' before include, we are loading relevant libraries that
//be used for the following code
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

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
#define BNO055_SAMPLERATE_DELAY_MS (100)

//creating sensor object
Adafruit_BNO055 IMU = Adafruit_BNO055();

void setup() {
  
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
}

void loop() {
  
//returning a vector of three components from one of the sensors
uint8_t system, gyro, accel, mg = 0; //efficient way of storing data from sensors
IMU.getCalibration(&system, &gyro, &accel, &mg); //calibration function
//NOTES ON CALIBRATION
//gyro is the easiest to calibrate because you just have to let it sit still
//magnetometer should be swong around a couple times for calibration
//accelerometer must be shown gravitational vector, takes a little longer than the other ones, 6 static positions

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
psi = atan2(Ym,Xm)/(2*3.141592654)*360;

//complementary filter between gyroscope and accelerometer
theta = (thetaG)*.95 + (thetaM)*.05;
phi = (phiG)*.95 + (phiM)*.05;

//printing calibration values
Serial.print(accel); Serial.print(",");
Serial.print(gyro); Serial.print(",");
Serial.print(mg); Serial.print(",");
Serial.print(system); Serial.print(",");

//printing vectored integrated angular velocity based on gyroscope
//Serial.print(theta); Serial.print(","); 
//Serial.print(phi); Serial.print(",");
//Serial.print(psi); Serial.print(",");

//printing raw gyro angular velocity
//Serial.print(gyr.x()); Serial.print(","); 
//Serial.print(gyr.y()); Serial.print(",");
Serial.println(gyr.z());
//printing magnetometer psi value
//Serial.println(psim);

thetaFold = thetaFnew;
phiFold = phiFnew;
psiFold = psiFnew;

delay(BNO055_SAMPLERATE_DELAY_MS);

}
