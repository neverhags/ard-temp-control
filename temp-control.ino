#include <PID_v1.h>

/* Conversion */
const int maxTempCalib = 510;
const int minTempCalib = 670;

/* Time vars */
unsigned long actTime = 0;
unsigned long readTempTimeout = 0;

/* Temp vars */
unsigned long iterations = 0;
float poolValues = 0.0;

int scanTime = 1; // Seconds
const int millisMultipler = 1000;
double Setpoint, Input, Output;
float t1=0;


void setup() {
  Serial.begin(9600); 
  analogReference(INTERNAL);
  pinMode(A0, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A1, INPUT);
  digitalWrite(A0, HIGH);
  digitalWrite(A2, LOW);
}

void loop() {
  actTime = millis();
  if (actTime >= readTempTimeout) {
    // para ver en el monitor
    Serial.println ((poolValues/iterations),1);
    reset();
    readTempTimeout = actTime + (millisMultipler * scanTime);
  } else {
    poolValues += map(analogRead(1), maxTempCalib, minTempCalib, 1000,0) / 10;
    iterations++;
  }
}

void reset() {
  poolValues = 0;
  iterations = 0;
}
