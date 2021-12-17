#include "HX711.h"

#define calibration_factor -7050.0 //This value is obtained using the SparkFun_HX711_Calibration sketch

#define CLK  A3

HX711 scale1;
HX711 scale2;

void setup() {
  Serial.begin(9600);
  Serial.println("HX711 scale demo");

  scale1.begin(A5, CLK);
  scale2.begin(A4, CLK);
  scale1.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale2.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale1.tare(); //Assuming there is no weight on the scale at start up, reset the scale to 0
  scale2.tare(); //Assuming there is no weight on the scale at start up, reset the scale to 0
  Serial.println("Readings:");
}

void loop() {
  Serial.print("Reading 1: ");
  Serial.print(scale1.get_units(), 1); //scale.get_units() returns a float
  Serial.println(" lbs"); //You can change this to kg but you'll need to refactor the calibration_factor
  Serial.print("Reading 2: ");
  Serial.print(scale2.get_units(), 1); //scale.get_units() returns a float
  Serial.println(" lbs"); //You can change this to kg but you'll need to refactor the calibration_factor
  delay(1000);
}
