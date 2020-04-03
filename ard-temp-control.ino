#include <AutoPID.h>
#include <SPI.h>
#include <MFRC522.h>
#include <EEPROM.h>

#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define TARGET 26 // °C
#define SS_PIN 10
#define RST_PIN 9
#define FAN_OFFSET 20
#define SCAN_TIME 4 // Seconds
#define DOOR_TIMEOUT 5
#define BANG 5
#define SET_OFFSET_AS_MIN true

/* Sensor Pins */
int positive1 = A0;
int negative1 = A2;
int signalPin = A1;
int pinOutput1 = 3;

/* Rele pins */
int rele1 = 2;
int rele2 = 4;

/* Conversion */
const int maxTempCalib1 = 510;
const int minTempCalib1 = 670;

/* Time vars */
unsigned long actTime = 0;
unsigned long readTempTimeout1 = 0;
unsigned long showPloterTimeout1 = 0;
unsigned long openDoorTimeout = 0;

/* Temp vars */
unsigned long iterations1 = 0;
float poolValues1 = 0.0;
float temp1 = 0;

String inputString = "";
bool stringComplete = false;
const int millisMultipler = 1000;
const int maxTemp = 100;
bool isEEPROMEmpty = false;

/* PID */
double Setpoint1, Input1 = 0, Output1 = 0;
/* Define the Tuning Parameters */
double consKp1=4, consKi1=0.2, consKd1=1;


/* Specify the links and initial tuning parameters */
AutoPID PID1(&Input1, &Setpoint1, &Output1, OUTPUT_MIN, OUTPUT_MAX, consKp1, consKi1, consKd1);

MFRC522 rfid(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key; 

byte* nuidPICC;

void setup() {
  Serial.begin(115200); 
  analogReference(INTERNAL);
  pinMode(positive1, OUTPUT);
  pinMode(negative1, OUTPUT);
  pinMode(rele1, OUTPUT);
  pinMode(rele2, OUTPUT);
  digitalWrite(rele1, HIGH);
  digitalWrite(rele2, HIGH);
  digitalWrite(positive1, HIGH);
  digitalWrite(negative1, LOW);
 
  pinMode(signalPin, INPUT);

  PID1.setBangBang(BANG);
  PID1.setTimeStep(5000);

  SPI.begin();
  rfid.PCD_Init();

  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }

  Setpoint1 = maxTemp - TARGET;
}

void loop() {
  actTime = millis();
  if (actTime >= readTempTimeout1) {
    temp1 = (poolValues1/iterations1);
    reset();
    ploter1(); 
    readTempTimeout1 = actTime + (millisMultipler * SCAN_TIME);
  } else {
    openDoor();
    scanTemp1();
    pid1();
    readCard();
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
  if (actTime >= showPloterTimeout1) {
    Serial.print ("T");
    Serial.print (temp1, 2);
    Serial.print (",B");
    Serial.println (Output1, 2);
    Serial.print (temp1, 2);
    Serial.print (",");
    Serial.println (Output1 / 10, 2);
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
  Serial.println(rfid.PICC_GetTypeName(piccType));

  // Check is the PICC of Classic MIFARE type
  if (piccType != MFRC522::PICC_TYPE_MIFARE_MINI &&  
    piccType != MFRC522::PICC_TYPE_MIFARE_1K &&
    piccType != MFRC522::PICC_TYPE_MIFARE_4K) {
    return;
  }
  
  if (compareData(rfid.uid.uidByte, rfid.uid.size)) {
    openDoorTimeout = actTime + (DOOR_TIMEOUT * millisMultipler);
  }

  
  if (rfid.uid.uidByte[0] != nuidPICC[0] || 
    rfid.uid.uidByte[1] != nuidPICC[1] || 
    rfid.uid.uidByte[2] != nuidPICC[2] || 
    rfid.uid.uidByte[3] != nuidPICC[3] ) {

    // Store NUID into nuidPICC array
    for (byte i = 0; i < 4; i++) {
      nuidPICC[i] = rfid.uid.uidByte[i];
    }
   
    printHex(rfid.uid.uidByte, rfid.uid.size);
    Serial.println();
    Serial.print(F("In dec: "));
    printDec(rfid.uid.uidByte, rfid.uid.size);
    Serial.println();
  }
  else Serial.println(F("Card read previously."));

  // Halt PICC
  rfid.PICC_HaltA();

  // Stop encryption on PCD
  rfid.PCD_StopCrypto1();
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
        Serial.print("emptyCounter: ");
        Serial.println(emptyCounter);
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
