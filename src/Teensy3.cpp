/*
MCU: Teensy 4.1
PCB: Teensy 3 v2
4/9/2026 : Wheel RPM - Jason 
*/

#include <TimeLib.h>
#include <sdios.h>
#include <SD.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_ADXL345_U.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// TEMP CONST
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
double currAmbientTempF = 0;
double currObjectTempF = 0;
unsigned long tempTimer = 0;

// SCREEN CONST
#define OLED_W 128
#define OLED_H 64
#define OLED_ADDR 0x3C
//#define MASTER_ADDR 0x12

Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);

// WHEEL RPM LEFT CONST
const int wheelRpmPinL = 20;
const int TEETHL = 14;
unsigned long timerSinceLastToothL = 0;
bool lastStateL = 0;
double wheelRPML = 0;
const int wheelTimeOutL = 500; // ms

// WHEEL RPM RIGHT CONST
const int wheelRpmPinR = 21;
const int TEETHR = 14;
unsigned long timerSinceLastToothR = 0;
bool lastStateR = 0;
double wheelRPMR = 0;
const int wheelTimeOutR = 500; // ms

// STEERING CONST
const int steeringPin = 27;

// ACCEL CONST
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
sensors_event_t accelEvent;

// SD CONST
const int chipSelect = BUILTIN_SDCARD;
File dataFile;
const int runNumberAddress = 0;
char fileName[20];
int runNumber;
unsigned long flushTimer = 0;

// TEENSY2TEENSY COMMS (DEACTIVATED)
/*
volatile int16_t rxAmbC = INT16_MIN;
volatile int16_t rxObjC = INT16_MIN;
volatile bool newPacket = false;
char msg[48] = "Obj: --   Amb: --";
int16_t textX = 0;
int16_t textY = 0;
Function below is Teensy-Teensy
void onReceiveMaster(int len) {
  if (len < 4) {
    while (Wire1.available()){
     Wire1.read();
    }
    return;
  }
  uint8_t b0 = Wire1.read();
  uint8_t b1 = Wire1.read();
  uint8_t b2 = Wire1.read();
  uint8_t b3 = Wire1.read();
  while (Wire1.available()){
    Wire1.read();
  }
  rxObjC = (int16_t)((b0 << 8) | b1);
  rxAmbC = (int16_t)((b2 << 8) | b3);
  newPacket = true;
}
*/

void setup() {
// INITIALIZE
  Serial.begin(9600);
  Wire.begin();

// STEERING INIT
  pinMode(steeringPin, INPUT);

// ERPM INIT
  //pinMode(rpmPin, INPUT);

// LED INIT
  pinMode(LED_BUILTIN, OUTPUT);

// WRPM INIT
  pinMode(wheelRpmPinL, INPUT);
  pinMode(wheelRpmPinR, INPUT);

// SCREEN INIT
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  // Uncomment if Teensy-Teensy is needed
  // Wire1.begin(MASTER_ADDR);
  // Wire1.onReceive(onReceiveMaster);

  display.display();

// TEMP ERROR
  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
  } else {
    Serial.println("Adafruit MLX90614 Initialized");
  }

// ACCEL ERROR
  if (!accel.begin()) {
    Serial.println("No ADXL345 sensor detected.");
  } else {
    accel.setRange(ADXL345_RANGE_16_G);
    Serial.println("Adafruit ADXL345 Initialized");
  }

// SCREEN ERROR
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)){
    Serial.println("SSD1306 init failed");
    while (1);
  }

// SD ERROR
  if (!SD.begin(chipSelect)) {
    Serial.println("Error: SD card initialization failed!");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);
      digitalWrite(LED_BUILTIN, LOW);
      delay(250);
    }
  }

// RUN # FROM EEPROM
  runNumber = EEPROM.read(runNumberAddress);
  runNumber++;
  //Good luck on the car, dont cry too much Luke.
  EEPROM.write(runNumberAddress, runNumber);

  sprintf(fileName, "SEPT28_%d.csv", runNumber);
  dataFile = SD.open(fileName, FILE_WRITE);

// FILE ERROR
  if (!dataFile) {
    Serial.println("Error: Could not open the file for writing.");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);
      digitalWrite(LED_BUILTIN, LOW);
      delay(250);
    }
  }


// FILE HEADER
  dataFile.print("hours:minutes:seconds,");
  dataFile.print("millis,");              // total milliseconds
  dataFile.print("currObjectTempF,");     // object temperature in Fahrenheit
  dataFile.print("currAmbientTempF,");
  dataFile.print("angle,");
  dataFile.print("X g,");
  dataFile.print("Y g,");
  dataFile.print("Z g,");
  dataFile.print("FL RPM,");
  dataFile.println("FR RPM,");
  //dataFile.print("Eng RPM");
  dataFile.flush();
  //You best keep being your goofy ahh self Mckay
  Serial.print("Logging to file: ");
  Serial.println(fileName);

//
  flushTimer = millis();
  tempTimer = millis();
// Light will flash Amber for 1 sec to indicate processes went through.
  Serial.println("Booting!");
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
// TIME
  unsigned long now = millis();
  unsigned int seconds = (now / 1000) % 60;
  unsigned int minutes = (now / 60000) % 60;
  unsigned int hours   = (now / 3600000) % 60;

  String timeStr = String(hours) + ":" + String(minutes) + ":" + String(seconds);

// STEERING
  int sensorValue = analogRead(steeringPin);
  float angle = sensorValue / 4.413 - 98.0;  // your calibration

// TEMP SCREEN (1hz)
  if (now - tempTimer >= 1000) {
    tempTimer = now;
    currObjectTempF = mlx.readObjectTempF();
    currAmbientTempF = mlx.readAmbientTempF();

    display.clearDisplay();
    display.drawRect(0, 0, OLED_W, OLED_H, SSD1306_WHITE);
    // Change setCursor values to determine where u want text to appear on the screen
    display.setCursor(10, 15);
    display.print("Obj: ");
    display.print(currObjectTempF, 1);
    display.print("F");
    display.setCursor(10, 35);
    display.print("Amb: ");
    display.print(currAmbientTempF, 1);
    display.print("F");
    display.display();
  }

// ACCEL
  accel.getEvent(&accelEvent);
  float x_g = accelEvent.acceleration.x / 9.8;
  float y_g = accelEvent.acceleration.y / 9.8;
  float z_g = accelEvent.acceleration.z / 9.8;

// LEFT WHEEL RPM
  bool currentStateL = digitalRead(wheelRpmPinL);
  if (currentStateL != lastStateL && currentStateL == HIGH) {
    unsigned long currentTimeL = millis();
    if (timerSinceLastToothL != 0) {
      double timeBetweenTeethL = currentTimeL - timerSinceLastToothL;
      if (timeBetweenTeethL > 0 && (timeBetweenTeethL < wheelTimeOutL)) {
        // 60,000 ms/min
        wheelRPML = 60000 / ((double)timeBetweenTeethL * TEETHL);
      }
      else{
        wheelRPML = 0;
      }
    }
    timerSinceLastToothL = currentTimeL;
  }
  lastStateL = currentStateL;


// RIGHT WHEEL RPM
  bool currentStateR = digitalRead(wheelRpmPinR);
  if (currentStateR != lastStateR && currentStateR == HIGH) {
    unsigned long currentTimeR = millis();
    if (timerSinceLastToothR != 0) {
      double timeBetweenTeethR = currentTimeR - timerSinceLastToothR;
      if (timeBetweenTeethR > 0 && (timeBetweenTeethR < wheelTimeOutR)) {
        // 60,000 ms/min
        wheelRPMR = 60000 / ((double)timeBetweenTeethR * TEETHR);
      }
      else{
        wheelRPMR = 0;
      }
    }
    timerSinceLastToothR = currentTimeR;
  }
  lastStateR = currentStateR;

// FILE WRITE
  if (dataFile) {
  // TIME
    dataFile.print(timeStr);
    dataFile.print(",");
    dataFile.print(now);
    dataFile.print(",");

  // TEMP
    dataFile.print(currObjectTempF);
    dataFile.print(",");
    dataFile.print(currAmbientTempF);
    dataFile.print(",");

  // STEERING ANGLE
    dataFile.print(angle);
    dataFile.print(",");

  // ACCEL
    dataFile.print(x_g);
    dataFile.print(",");
    dataFile.print(y_g);
    dataFile.print(",");
    dataFile.println(z_g);
    dataFile.print(",");

  // WHEEL RPM
    dataFile.print(wheelRPML);
    dataFile.print(",");
    dataFile.println(wheelRPMR);

// FLUSH TIMER
    if (now - flushTimer >= 4000) {
      flushTimer = now;
      dataFile.flush();
    }
  }

// SERIAL DEBUG
  Serial.print(timeStr);
  Serial.print(", ");
  Serial.print(now);
  Serial.print(", ");
  Serial.print(currObjectTempF);
  Serial.print("F, ");
  Serial.print(currAmbientTempF);
  Serial.print("F, ");
  Serial.print("angle=");
  Serial.println(angle);
  //Serial.print(", RPM=");
  //Serial.println(engineRPM);
}
