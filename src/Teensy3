#include <TimeLib.h>
#include <sdios.h>
#include <SD.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_ADXL345_U.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>

// TIME
unsigned long rpmTimer = 0;

// TEMP
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
double currAmbientTempF = 0;
double currObjectTempF = 0;
unsigned long tempTimer = 0;

// STEERING
const int steeringPin = 40;

// ACCEL
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
sensors_event_t accelEvent;

// ENGINE RPM
const int rpmPin = 33;
unsigned long timeOfSpark = 0;
unsigned long prevTimeOfSpark = 0;
unsigned long timeBetweenSparks = 0;
double engineRPM = 0;
bool prevState = LOW;

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
    dataFile.print(z_g);
    //dataFile.print(",");

    //ENGINERPM
    //dataFile.print(engineRPM);
    dataFile.println();

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
