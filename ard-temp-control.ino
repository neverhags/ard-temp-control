#include <AutoPID.h>
#include <SPI.h>
#include <MFRC522.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DS18B20.h>

#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define TARGET 30 // °C
#define SS_PIN 10
#define RST_PIN 9
#define FAN_OFFSET 25
#define SCAN_TIME 15 // Seconds
#define SCAN_TIME_OFFSET 5
#define DOOR_TIMEOUT 5
#define BANG 5
#define SET_OFFSET_AS_MIN true
#define MILLIS_MULTIPLIER 1000

/* Sensor Pins */
int positive1 = A0; 
int negative1 = A2; // utility
int signalPin = A1; // utility
int pinOutput1 = 3;

/* Rele pins */
int rele1 = 2;
int rele2 = 4;

/* Door Sensor */
int door1 = 5;

/* Time vars */
unsigned long actTime = 0;
unsigned long readTempTimeout1 = 0;
unsigned long openDoorTimeout = 0;

/* Temp vars */
unsigned long iterations1 = 0;
float poolValues1 = 0.0;
float temp1 = 0;

String inputString = "";
bool stringComplete = false;
const int maxTemp = 125;
bool isEEPROMEmpty = false;

/* PID */
double Setpoint1, Input1 = 0, Output1 = 0;
/* Define the Tuning Parameters */
double consKp1=4, consKi1=0.2, consKd1=1;

/* Dark Mode */
bool dark = true;

/* Specify the links and initial tuning parameters */
AutoPID PID1(&Input1, &Setpoint1, &Output1, OUTPUT_MIN, OUTPUT_MAX, consKp1, consKi1, consKd1);

MFRC522 rfid(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key; 

OneWire oneWire(signalPin);
DS18B20 sensor(&oneWire);

byte* nuidPICC;

void setup() {
  Serial.begin(115200); 
  Serial.println(__FILE__);
   
  analogReference(INTERNAL);
  pinMode(positive1, OUTPUT);
  pinMode(negative1, OUTPUT);
  pinMode(rele1, OUTPUT);
  pinMode(rele2, OUTPUT);
  pinMode(door1, INPUT);
  digitalWrite(rele1, HIGH);
  digitalWrite(rele2, HIGH);
  digitalWrite(positive1, HIGH);
  digitalWrite(negative1, LOW);
 
  pinMode(signalPin, INPUT);

  PID1.setBangBang(BANG);
  PID1.setTimeStep(MILLIS_MULTIPLIER * (SCAN_TIME + SCAN_TIME_OFFSET));

  SPI.begin();
  sensor.begin();
  rfid.PCD_Init();

  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }

  Setpoint1 = maxTemp - TARGET;
  inputString.reserve(255);
}

void loop() {
  actTime = millis();
  ploter1();
  openDoor();
  scanTemp1();
  pid1();
  readCard();
  scanDoor();
  scanSerial();
} 

void reset() {
  poolValues1 = 0;
  iterations1 = 0;
}

void scanTemp1() {
  sensor.requestTemperatures();
  while (!sensor.isConversionComplete());  // wait until sensor is ready
  poolValues1 += sensor.getTempC();
  iterations1++;
}

void scanDoor() {
  if(digitalRead(door1) && !dark) {
    releOn(rele2);
  } else {
    releOff(rele2);
  }
}

void scanSerial() {
  if (stringComplete) {
    commands(inputString);
    inputString = "";
    stringComplete = false;
  }
}

void commands(String command) {
  if (inputString.equals("loff\r\n")) {
    dark = true;
  }
  if (inputString.equals("lon\r\n")) {
    dark = false;
  }
  if (inputString.equals("open\r\n")) {
    setDoorTimeout();
  }
}

void pid1() {
  if (temp1 != 0) {
    Input1 = maxTemp - temp1;
    double gap = abs(Setpoint1 - Input1);
    PID1.run(); 
    if (Output1 > FAN_OFFSET) {
      analogWrite(pinOutput1, Output1);
    } else {
      if (SET_OFFSET_AS_MIN) {
        analogWrite(pinOutput1, FAN_OFFSET);
      } else {
        analogWrite(pinOutput1, 0);
      }
    }
  }
}

void ploter1() {
  if (actTime >= readTempTimeout1) {
    temp1 = (poolValues1/iterations1);
    float output = Output1;
    if (output < FAN_OFFSET) {
      output = FAN_OFFSET;
    }
    reset();
    if(temp1) {
      Serial.print ("T");
      Serial.print (temp1, 2);
      Serial.print (",B");
      Serial.println (output, 2);
      Serial.print (temp1, 2);
      Serial.print (",");
      Serial.println (output / 10, 2);
    }
    
    readTempTimeout1 = actTime + (MILLIS_MULTIPLIER * SCAN_TIME);
  }
}

void readCard() {
  // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
  if ( ! rfid.PICC_IsNewCardPresent())
    return;

  // Verify if the NUID has been readed
  if ( ! rfid.PICC_ReadCardSerial())
    return;

  MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);

  // Check is the PICC of Classic MIFARE type
  if (piccType != MFRC522::PICC_TYPE_MIFARE_MINI &&  
    piccType != MFRC522::PICC_TYPE_MIFARE_1K &&
    piccType != MFRC522::PICC_TYPE_MIFARE_4K) {
    return;
  }
  
  if (compareData(rfid.uid.uidByte, rfid.uid.size)) {
    setDoorTimeout();
  }

  // Halt PICC
  rfid.PICC_HaltA();

  // Stop encryption on PCD
  rfid.PCD_StopCrypto1();
}

void setDoorTimeout() {
  openDoorTimeout = actTime + (DOOR_TIMEOUT * MILLIS_MULTIPLIER);
}

void openDoor() {
  if (actTime >= openDoorTimeout) {
    releOff(rele1);
  } else {
    releOn(rele1);
  }
}

void printHex(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

void printDec(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], DEC);
  }
}

void releOn(int rele) {
  digitalWrite(rele, LOW);
}

void releOff(int rele) {
  digitalWrite(rele, HIGH);
}

void saveEEPROM(uint8_t* data) {
  for (byte i = 0; i < 4; i++) {
    EEPROM.put(i, data[i]);
  }
}

bool compareData(uint8_t* inputData, byte bufferSize) {
  isEEPROMEmpty = false;
  byte emptyCounter = 0; 
  for (uint8_t i = 0; i < 6; i++) { 
    if (EEPROM.read(i) != inputData[i]) {
      if (EEPROM.read(i) == 0) {
        emptyCounter++;
      } else {
        return false;
      }
    }
  }
  if(emptyCounter == 4) {
    saveEEPROM(inputData);
  }
  return true;
}


void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
