/*
MCU: Teensy 4.1
PCB: Teensy 3 v2
4/9/2026 : Wheel RPM - Jason 

Pin List:
1 : LED
  : Temp
  : Screen
  : WheelRPM
  : Gyro
  : 
*/

#include <TimeLib.h>
#include <SD.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_LSM6DSOX.h>
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
// #define SLAVE_ADDR 0x12

Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);
/* Wheel RPM Commented Out For Now
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
*/
// STEERING CONST
const int steeringPin = 27;

// ACCEL/GYRO CONST
Adafruit_LSM6DSOX gyro;

// SD CONST
const int chipSelect = BUILTIN_SDCARD;
File dataFile;
const int runNumberAddress = 0;
char fileName[20];
int runNumber;
unsigned long flushTimer = 0;

volatile uint32_t pendingTime = 0;
volatile bool timeUpdatePending = false;

// TIME CONST (RTC HERE)
// TIME CONST
#define TIME_HEADER  "T"   // Header tag for serial time sync message
unsigned long lastTimeSync = 0;

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 

  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     if( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
       pctime = 0L; // return 0 to indicate that the time is not valid
     }
  }
  return pctime;
}

time_t getTeensyTime() {
    return Teensy3Clock.get();
}

int runLoop = 0;
// TEENSY-TEENSY
/*void onReceiveMaster(int len) {
  if (len < 4) {
    while (Wire1.available()){
     Wire1.read();
    }
    return;
  }
  uint32_t t = 0;
  t |= (uint32_t)Wire1.read() << 24;
  t |= (uint32_t)Wire1.read() << 16;
  t |= (uint32_t)Wire1.read() << 8;
  t |= (uint32_t)Wire1.read();

  pendingTime = t;
  timeUpdatePending = true;
  //rxObjC = (int16_t)((b0 << 8) | b1);
  //rxAmbC = (int16_t)((b2 << 8) | b3);
  //newPacket = true;
}
*/
void setup() {
// INITIALIZE
  Serial.begin(115200);
  Wire.begin();
  //Wire1.begin(SLAVE_ADDR); TEENSY - TEENSY
  analogReadResolution(10);

// STEERING INIT
  pinMode(steeringPin, INPUT);

// LED INIT
  pinMode(LED_BUILTIN, OUTPUT);
/*
// WRPM INIT
  pinMode(wheelRpmPinL, INPUT);
  pinMode(wheelRpmPinR, INPUT);
*/
// SCREEN ERROR
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)){
    Serial.println("SSD1306 init failed");
    while (1);
  }
  
// SCREEN INIT
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.display();
// Teensy Teensy (SLAVE)
   //Wire1.onReceive(onReceiveMaster);
// TEMP ERROR
  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
  } else {
    Serial.println("Adafruit MLX90614 Initialized");
  }

// ACCEL/GYRO ERROR - Note: The object 'gyro' is declaration of our ACCEL/GYRO sensor don't get confused
  if (!gyro.begin_I2C()) {
    Serial.println("No Gyro sensor detected.");
  } else {
    gyro.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    gyro.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
    gyro.setAccelDataRate(LSM6DS_RATE_208_HZ);
    gyro.setGyroDataRate(LSM6DS_RATE_208_HZ);
    Serial.println("Adafruit LSM6DSOX Initialized");
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
// RTC INIT
  setSyncProvider(getTeensyTime); // Sets Time.lib to use RTC
  // If not set, sync time
  if (timeStatus()!= timeSet) {
    if (Serial.available()) {
      time_t t = processSyncMessage();
      if (t != 0) {
        Teensy3Clock.set(t); // set the RTC
        setTime(t);
      }
    }
  }

// RUN # FROM EEPROM
  runNumber = EEPROM.read(runNumberAddress);
  runNumber++;
  //Good luck on the car, dont cry too much Luke.
  EEPROM.write(runNumberAddress, runNumber);

  snprintf(fileName, sizeOf(fileName),  "%02d_%02d.csv", month(), day());

  dataFile = SD.open(fileName, FILE_WRITE);

// FILE ERROR
  if (dataFile) {
    dataFile.print("hours:minutes:seconds,");
    dataFile.print("millis,");              // total milliseconds
    dataFile.print("currObjectTempF,");     // object temperature in Fahrenheit
    dataFile.print("currAmbientTempF,");
    dataFile.print("angle,");
    dataFile.print("X g,");
    dataFile.print("Y g,");
    dataFile.print("Z g,");
    dataFile.print("X rad/s,");
    dataFile.print("Y rad/s,");
    dataFile.print("Z rad/s,");
    dataFile.print("FL RPM,");
    dataFile.println("FR RPM");
    dataFile.flush();
  //You best keep being your goofy ahh self Mckay
    Serial.print("Logging to file: ");
    Serial.println(fileName);
  }
  else{
    Serial.println("Error: Could not open the file for writing.");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);
      digitalWrite(LED_BUILTIN, LOW);
      delay(250);
    }
  }

//
  flushTimer = millis();
  tempTimer = millis();
}

void loop() {

// RUN INDICATOR
  digitalWrite(LED_BUILTIN, HIGH);
  runLoop++;
// TIME STAMP
  unsigned long board_timer = millis();
// THE ABOVE IS SPECIFICALLY FOR SD WRITING.
  char timeStr[20];
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", hour(), minute(), second());

// STEERING
  int sensorValue = analogRead(steeringPin);
  float angle = sensorValue / 4.413 - 98.0;  // your calibration

// TEMP SCREEN (1hz)
  if (board_timer - tempTimer >= 1000) {
    tempTimer = board_timer;
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

    // Date and Time + File Name in Bot Right (GPT'D, needs to be tested for positioning)
    char currentDateTime[25];
    snprintf(currentDateTime, sizeOf(currentDateTime),  "%02d/%02d %02d:%02d:%02d", 
    month(), day(), hour(), minute(), second());

    int dtWidth = strlen(dateTimeStr) * 6;
    display.setCursor(OLED_W - dtWidth - 2, OLED_H - 16);
    display.print(dateTimeStr);

    int fileWidth = strlen(fileName) * 6;
    display.setCursor(OLED_W - fileWidth - 2, OLED_H - 8);
    display.print(fileName);
    
    display.display();
  }

// ACCEL/GYRO

  sensors_event_t accelEvent;
  sensors_event_t gyroEvent;
  sensors_event_t tempEvent;
  
  gyro.getEvent(&accelEvent, &gyroEvent, &tempEvent);
  float x_g = accelEvent.acceleration.x / 9.8;
  float y_g = accelEvent.acceleration.y / 9.8;
  float z_g = accelEvent.acceleration.z / 9.8;
  float x_rads = gyroEvent.gyro.x;
  float y_rads = gyroEvent.gyro.y;
  float z_rads = gyroEvent.gyro.z;
/*
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

  if (millis() - timerSinceLastToothL > wheelTimeOutL) {
    wheelRPML = 0;
  }


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

  if (millis() - timerSinceLastToothR > wheelTimeOutR) {
    wheelRPMR = 0;
  }
  */
// FILE WRITE
  if (dataFile) {
  // TIME
    dataFile.print(timeStr);
    dataFile.print(",");
    dataFile.print(board_timer);
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
    dataFile.print(z_g);
    dataFile.print(",");
    dataFile.print(x_rads);
    dataFile.print(",");
    dataFile.print(y_rads);
    dataFile.print(",");
    dataFile.println(z_rads);
    
/*
  // WHEEL RPM
    dataFile.print(wheelRPML);
    dataFile.print(",");
    dataFile.println(wheelRPMR); */

// FLUSH TIMER
    if (board_timer - flushTimer >= 1000) {
      flushTimer = board_timer;
      dataFile.flush();

      //SERIAL DEBUG (1 SEC)

      Serial.print("hours:minutes:seconds,");
      Serial.print("millis,");              // total milliseconds
      Serial.print("currObjectTempF,");     // object temperature in Fahrenheit
      Serial.print("currAmbientTempF,");
      Serial.print("angle,");
      Serial.print("X g,");
      Serial.print("Y g,");
      Serial.print("Z g,");
      Serial.print("X rad/s,");
      Serial.print("Y rad/s,");
      Serial.print("Z rad/s,");
 /*   Serial.print("FL RPM,"); 
      Serial.print("FR RPM,"); */
      Serial.println("runLoops");
      
      Serial.print(timeStr);
      Serial.print(",");
      Serial.print(board_timer);
      Serial.print(",");
      Serial.print(currObjectTempF);
      Serial.print(",");
      Serial.print(currAmbientTempF);
      Serial.print(",");
      Serial.print(angle);
      Serial.print(",");
      Serial.print(x_g);
      Serial.print(",");
      Serial.print(y_g);
      Serial.print(",");
      Serial.print(z_g);
      Serial.print(",");
      Serial.print(x_rads);
      Serial.print(",");
      Serial.print(y_rads);
      Serial.print(",");
      Serial.print(z_rads);
     /* Serial.print(",");
      Serial.print(wheelRPML);
      Serial.print(",");
      Serial.print(wheelRPMR); */
      Serial.print(",");
      Serial.println(runLoop);
    }
    runLoop = 0;
  }
 
}
