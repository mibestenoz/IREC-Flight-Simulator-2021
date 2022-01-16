/*
Pressure Measurements with the
MPS20N0040D Breakout Board
with the HX710B/HX711 ADC
5V Supply Voltage
 */
#include <HX711.h>

const byte OUT_pin = 6; // OUT data pin
const byte SCK_pin = 5; // clock data pin
int avg_size = 10; // #pts to average over

HX711 scale;

void setup() {
  Serial.begin(115200); // start the serial port
  scale.begin(OUT_pin, SCK_pin); // start comm with the HX710B
}

void loop() {
  float avg_val = 0.0; // variable for averaging
  for (int ii=0;ii<avg_size;ii++){
    avg_val += scale.read(); // add multiple ADC readings
    delay(50); // delay between readings
  }
  avg_val /= avg_size;
  Serial.println(avg_val,0); // print out the average
}
