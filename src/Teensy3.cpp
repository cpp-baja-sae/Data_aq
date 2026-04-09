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

// TEMP
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
double currAmbientTempF = 0;
double currObjectTempF = 0;
unsigned long tempTimer = 0;

// SCREEN
#define OLED_W 128
#define OLED_H 64
#define OLED_ADDR 0x3C
//#define MASTER_ADDR 0x12

Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);

//volatile int16_t rxObjC = INT16_MIN;
//volatile int16_t rxAmbC = INT16_MIN;
//volatile bool newPacket = false;

// char msg[48] = "Obj: --   Amb: --";
//int16_t textX = 0;
//int16_t textY = 0;
// Function below is Teensy-Teensy
//void onReceiveMaster(int len) {
//  if (len < 4) {
//    while (Wire1.available()){
//     Wire1.read();
//    }
//    return;
//  }
//  uint8_t b0 = Wire1.read();
//  uint8_t b1 = Wire1.read();
//  uint8_t b2 = Wire1.read();
//  uint8_t b3 = Wire1.read();
//  while (Wire1.available()){
//    Wire1.read();
//  }
//  rxObjC = (int16_t)((b0 << 8) | b1);
//  rxAmbC = (int16_t)((b2 << 8) | b3);
//  newPacket = true;
//}

// STEERING
const int steeringPin = 40;

// ACCEL
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
sensors_event_t accelEvent;


// FILE
const int chipSelect = BUILTIN_SDCARD;
File dataFile;
const int runNumberAddress = 0;
char fileName[20];
int runNumber;
unsigned long flushTimer = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(steeringPin, INPUT);
  //pinMode(rpmPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // TEMP
  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
  } else {
    Serial.println("Adafruit MLX90614 Initialized");
  }

  // ACCEL
  if (!accel.begin()) {
    Serial.println("No ADXL345 sensor detected.");
  } else {
    accel.setRange(ADXL345_RANGE_16_G);
    Serial.println("Adafruit ADXL345 Initialized");
  }

  // SCREEN
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)){
    Serial.println("SSD1306 init failed");
    while (1);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  // Uncomment if Teensy-Teensy is needed
  // Wire1.begin(MASTER_ADDR);
  // Wire1.onReceive(onReceiveMaster);

  display.display();

  // SD
  if (!SD.begin(chipSelect)) {
    Serial.println("Error: SD card initialization failed!");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);
      digitalWrite(LED_BUILTIN, LOW);
      delay(250);
    }
  }

  // Run number from EEPROM
  runNumber = EEPROM.read(runNumberAddress);
  runNumber++;
  //Good luck on the car, dont cry too much Luke.
  EEPROM.write(runNumberAddress, runNumber);

  sprintf(fileName, "SEPT28_%d.csv", runNumber);
  dataFile = SD.open(fileName, FILE_WRITE);

  if (!dataFile) {
    Serial.println("Error: Could not open the file for writing.");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);
      digitalWrite(LED_BUILTIN, LOW);
      delay(250);
    }
  }


  // Write header
  dataFile.print("hours:minutes:seconds,");
  dataFile.print("millis,");              // total milliseconds
  dataFile.print("currObjectTempF,");     // object temperature in Fahrenheit
  dataFile.print("currAmbientTempF,");
  dataFile.print("angle,");
  dataFile.print("X g,");
  dataFile.print("Y g,");
  dataFile.print("Z g");
  //dataFile.print("Eng RPM");
  dataFile.println();
  dataFile.flush();
  //You best keep being your goofy ahh self Mckay
  Serial.print("Logging to file: ");
  Serial.println(fileName);

  flushTimer = millis();
  tempTimer = millis();
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

  // TEMP (update once per second)
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

  if (dataFile) {
    //TIME
    dataFile.print(timeStr);
    dataFile.print(",");
    dataFile.print(now);
    dataFile.print(",");

    //TEMP
    dataFile.print(currObjectTempF);
    dataFile.print(",");
    dataFile.print(currAmbientTempF);
    dataFile.print(",");

    //STEERING ANGLE
    dataFile.print(angle);
    dataFile.print(",");

    //ACCEL
    dataFile.print(x_g);
    dataFile.print(",");
    dataFile.print(y_g);
    dataFile.print(",");
    dataFile.println(z_g);
    //dataFile.print(",");

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
