
int MTR0 = 9;
int DIR0 = 13;
int Pot = 0; //Potentiometer
int rpmMaximum = 0;
const int hallSensorPinBlue = 2;  //Hall sensor is connected to these pins.
const int hallSensorPinRed = 3;
const int hallSensorPinGreen = 4;
const unsigned long sampleTime = 1000; 

int countBlue = 0;
int countGreen = 0;
int countRed = 0;

unsigned long time;
void setup() {
    Serial.begin(9600);
    pinMode(MTR0, INPUT);
    pinMode(DIR0, INPUT);

    pinMode(hallSensorPinBlue, INPUT);
    pinMode( hallSensorPinRed, INPUT);
    pinMode(hallSensorPinGreen, INPUT);

    pinMode(Pot, INPUT);
}

void loop() {
    delay (100); //100 ms
    int rpm = getRPM();
    if (rpm > rpmMaximum) rpmMaximum = rpm;
}

//Come back and comment later
int getRPM() {
    int countBlue = 0;
    int countRed = 0;
    int countGreen = 0;
    boolean countFlagBlue = LOW;
    boolean countFlagRed = LOW;
    boolean countFlagGreen = LOW;
    unsigned long currentTime = 0;
    unsigned long startTime = millis();
    while (currentTime <= sampleTime){
        if (digitalRead(hallSensorPinBlue)==HIGH){
            countFlagBlue = HIGH;
        }
        if (digitalRead(hallSensorPinBlue)==LOW&&countFlagBlue==HIGH){
            countBlue++;
            countFlagBlue = LOW;
        }
        if (digitalRead(hallSensorPinRed)==HIGH){
            countFlagRed = HIGH;
        }
        if (digitalRead(hallSensorPinRed)==LOW&&countFlagRed==HIGH){
            countRed++;
            countFlagRed = LOW;
        }
        if (digitalRead(hallSensorPinGreen)==HIGH){
            countFlagGreen = HIGH;
        }
        if (digitalRead(hallSensorPinGreen)==LOW&&countFlagGreen==HIGH){
            countGreen++;
            countFlagGreen = LOW;
        }
        currentTime = millis()-startTime;
    }
    int countRPMBlue = int (60000/float(sampleTime)*countBlue);
    int countRPMRed = int (60000/float(sampleTime)*countRed);
    int countRPMGreen = int (60000/float(sampleTime)*countGreen);
    int countRPM = (countRPMBlue + countRPMGreen + countRPMRed)/3;
    Serial.print("countRPMBlue: "); Serial.println(countRPMBlue);
    Serial.print("countRPMGreen: "); Serial.println(countRPMGreen);
    Serial.print("countRPMRed: "); Serial.println(countRPMRed);
    Serial.print("countRPM: "); Serial.println(countRPM);
    return countRPM;
}
