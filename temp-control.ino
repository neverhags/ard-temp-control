#include <AutoPID.h>

#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define TARGET 25 // °C

/* Conversion */
const int maxTempCalib1 = 510;
const int minTempCalib1 = 670;

/* Time vars */
unsigned long actTime = 0;
unsigned long readTempTimeout1 = 0;
unsigned long showPloterTimeout1 = 0;

/* Temp vars */
unsigned long iterations1 = 0;
float poolValues1 = 0.0;
float temp1 = 0;

int scanTime1 = 1; // Seconds
const int millisMultipler = 1000;
const int offsetStart1 = 80; // [0-254]

/* PID */
double Setpoint1, Input1 = 0, Output1 = 0;
/* Define the aggressive and conservative Tuning Parameters */
double aggKp1=4, aggKi1=0.2, aggKd1=1;
double consKp1=4, consKi1=0.2, consKd1=1;

/* Sensor Pins */
int positive1 = A0;
int negative1 = A2;
int signalPin = A1;
int pinOutput1 = 3;

/* Specify the links and initial tuning parameters */
AutoPID PID1(&Input1, &Setpoint1, &Output1, OUTPUT_MIN, OUTPUT_MAX, consKp1, consKi1, consKd1);

void setup() {
  Serial.begin(115200); 
  analogReference(INTERNAL);
  pinMode(positive1, OUTPUT);
  pinMode(negative1, OUTPUT);
  digitalWrite(positive1, HIGH);
  digitalWrite(negative1, LOW);

  pinMode(signalPin, INPUT);

  PID1.setBangBang(15);
  PID1.setTimeStep(1000);

  Setpoint1 = 100 - TARGET; // max temp - target temp
}

void loop() {
  actTime = millis();
  if (actTime >= readTempTimeout1) {
    temp1 = (poolValues1/iterations1);
    reset();
    readTempTimeout1 = actTime + (millisMultipler * scanTime1);
    ploter1();
  } else {
    scanTemp1();
    pid1();
  }
}

void reset() {
  poolValues1 = 0;
  iterations1 = 0;
}

void scanTemp1() {
  poolValues1 += map(analogRead(1), maxTempCalib1, minTempCalib1, 1000,0) / 10;
  iterations1++;
}

void pid1() {
  if (temp1 != 0) {
    Input1 = 100 - temp1;
    double gap = abs(Setpoint1 - Input1);
    PID1.run(); 
    Serial.println(temp1);
    Serial.println(Output1);
    if (Output1 > offsetStart1) {
      analogWrite(pinOutput1, Output1);
    }
  }
}

void ploter1() {
  if (actTime >= showPloterTimeout1) {
    Serial.println (temp1, 1);
    Serial.println (Output1, 1);
  }
}
