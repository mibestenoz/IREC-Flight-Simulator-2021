int x = 0;
int y = 0;
int z = 0;

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
x = x + 1;
y = y + 2;
z = z + 4;

Serial.print(x); 
Serial.print(",");
Serial.print(y);
Serial.print(",");
Serial.println(z);
delay(100);
}
