/*
MCU: Teensy 4.1
PCB: Teensy 2 v2
4/9/2026 - Kareem + Jason

Pin List:
1 : LED
21: ERPM
23: Rear WPM
26: Front Brake Pres
27: Rear Brake Pres
I2C: Accel
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
#include <cmath>
#include <TimeLib.h>

// #define MASTER_ADDR 0x12

// SD CONST
const int chipSelect = BUILTIN_SDCARD;
File dataFile;
char fileName[20];
const int runNumberAddress = 0;

// LED CONST
const int LED_PIN = LED_BUILTIN;

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
// TEENSY-TEENSY
/*void sendTime() {
  uint32_t t = now(); // get current unix timestamp (4 bytes)
  
  Wire1.beginTransmission(MASTER_ADDR);
  Wire1.write((t >> 24) & 0xFF); // byte 3
  Wire1.write((t >> 16) & 0xFF); // byte 2
  Wire1.write((t >> 8)  & 0xFF); // byte 1
  Wire1.write((t)       & 0xFF); // byte 0
  Wire1.endTransmission();
}
*/
unsigned long lastFlush = 0;
unsigned int runLoop = 0;

// THROTTLE CONST (DEACTIVATED)
const int CS_PIN = 10;

// BRAKES CONST
const int brakeRearPin  = 27; //Changed from 40 to match PCB 
const int brakeFrontPin = 26; //Changed from 41 to match PCB
  // Our Brake Sensors read 0-2000 PSI from .5V -> 4.5V.
const double V_Logic = 3.3;
const int ADC_Res = 1023;

const double V_Min = 0.357; // Based off our resistor divider (0.5 * 11/15)
const double V_Max = 3.214; // Based off our resistor divider (4.5 * 11/15)
const double PSI_Min = 0.0; // Data sheet of brake pressure sensor
const double PSI_Max = 2000.0; // Data sheet of brake pressure sensor

double convertToPSI(int read){

  double V = read * V_Logic / ADC_Res;
  double PSI = ((V-V_Min) / (V_Max - V_Min)) * (PSI_Max - PSI_Min) + PSI_Min;

  return PSI; 
}

// ACCELEROMETER CONST
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

// ENGINE RPM CONST
const int engineRpmPin = 21;  // Digital input pin that receives the RPM pulse signal

const float PULSES_PER_REVOLUTION = 0.5f;  

  // Timestamp (in microseconds) of the most recent detected rising edge (pulse).
volatile uint32_t lastPulseTime_us = 0;
volatile uint32_t lastValidPulseTime_us = 0;

  // Time difference (in microseconds) between the last two valid pulses.
  // This is the pulse period we use to compute RPM.
volatile uint32_t lastPulsePeriod_us = 0;
// If we dont get a period within 100 ms then the engine has to be off, read 0.
const uint32_t TIMEOUT_us = 100000;

  // If pulses per revolution = 0.5, then
  //   RPM = (60e6 / period_us) / 0.5 = 120e6 / period_us
  // At 50000us, 2400 RPM at 33333us, 3600 RPM. Smaller period = Higher RPM
const uint32_t MAX_PERIOD_us = 50000;
const uint32_t MIN_PERIOD_us = 33333;

float engineRPM = 0.0f;

void onRpmPulseRise() {
  uint32_t now_us = micros();

  // If this isn't the very first pulse, we can compute the time between pulses.
  if (lastPulseTime_us != 0) {
    uint32_t period_us = now_us - lastPulseTime_us;

    // From our graph we know our engine rpm can only be between 2400 to 3600, so any other values are worthless
    if (period_us >= MIN_PERIOD_us && period_us <= MAX_PERIOD_us) {
      lastPulsePeriod_us = period_us;
      lastValidPulseTime_us = now_us;
    }
  }
  // Always update the time of the most recent pulse
  lastPulseTime_us = now_us;
}
/* Wheel RPM Disabled for Now
// WHEEL RPM CONST
const int wheelRpmPin = 23;
const int TEETH = 14;
unsigned long timerSinceLastTooth = 0;
bool lastState = 0;
double wheelRPM = 0;
const int wheelTimeOut = 500; // ms
*/


void setup() {
// INITIALIZATION
  Serial.begin(115200);
  analogReadResolution(10);

// LED INIT
  pinMode(LED_PIN, OUTPUT);

// THROTTLE INIT (DEACTIVATED)
  SPI.begin();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

// ERPM INIT
  // Use INPUT if your sensor output is a clean push-pull digital signal.
  // Use INPUT_PULLUP only if your sensor output is open-collector/open-drain.
  pinMode(engineRpmPin, INPUT);
  // Attach interrupt on rising edge of the pulse signal.
  attachInterrupt(digitalPinToInterrupt(engineRpmPin), onRpmPulseRise, RISING);
/*
// Wheel RPM INIT
  pinMode(wheelRpmPin, INPUT);
*/
// BRAKES INIT
  pinMode(brakeRearPin, INPUT);
  pinMode(brakeFrontPin, INPUT);

// SD ERROR
  if (!SD.begin(chipSelect)) {
    Serial.println("Error: SD card initialization failed!");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(250);
      digitalWrite(LED_PIN, LOW);
      delay(250);
    }
  }

// RTC INIT
  setSyncProvider(getTeensyTime); // Sets Time.lib to use RTC
  //Wire1.begin();
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

// RTC ERROR
  if (timeStatus()!= timeSet) {
    Serial.println("Unable to sync with the RTC");
    while(1);
  } else {
    Serial.println("RTC has set the system time");
  }

// ACCEL ERROR
  if (!accel.begin()) {
    Serial.println("Error: ADXL345 not detected.");
    while(1);
  }
  accel.setDataRate(ADXL345_DATARATE_200_HZ);

// RUN # FROM EEPROM
  int runNumber = EEPROM.read(runNumberAddress);
  runNumber++;
  EEPROM.write(runNumberAddress, runNumber);

// SD INIT
  snprintf(fileName, sizeof(fileName),  "%02d_%02d.csv", month(), day());
  dataFile = SD.open(fileName, FILE_WRITE);

// FILE HEADER
  if (dataFile) {
  // TIME
    dataFile.print("hours:minutes:seconds,");
    dataFile.print("millis,");  // total milliseconds

  // THROTTLE (DEACTIVATED)
    // dataFile.print("Throttle Angle,");

  // BRAKE PRESSURE
    dataFile.print("PSI_Front,"); //Completely unfiltered data taken at 5ms marks
    dataFile.print("PSI_Rear,"); // Accounting for noise

  // ACCELEROMETER
    dataFile.print("Accel X g,");
    dataFile.print("Accel Y g,");
    dataFile.print("Accel Z g,");
    dataFile.println("Eng RPM,");
/*
  // WHEEL RPM
    dataFile.println("Rear RPM");
*/
// SD ERROR
    Serial.print("Logging to file: ");
    Serial.println(fileName);
    digitalWrite(LED_PIN, LOW);
  } else {
    Serial.println("Error: Could not open the file for writing.");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(250);
  //Love you guys, gonna miss you :(
      digitalWrite(LED_PIN, LOW);
      delay(250);
    }
  }
}
void loop() {
  
// RUN INDICATOR 
  digitalWrite(LED_BUILTIN, HIGH);
  runLoop++;
  
// FILE ERROR
  if (!dataFile) {
    Serial.println("Error: dataFile invalid.");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(250);
      digitalWrite(LED_PIN, LOW);
      delay(250);
    }
  }


// TIME STAMP
  unsigned long board_timer = millis();
  // THE ABOVE IS SPECIFICALLY FOR SD WRITING.
  char timeStr[20];
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", hour(), minute(), second());

// THROTTLE (DISABLED L31)
  // digitalWrite(CS_PIN, LOW);
  // SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
  // uint16_t result = SPI.transfer16(0x0000);
  // SPI.endTransaction();
  // digitalWrite(CS_PIN, HIGH);
  // uint16_t angleRaw = (result >> 6) & 0x03FF;  // 10-bit angle (0–1023)
  // float angleDeg = (angleRaw / 1023.0) * 360.0;
  // dataFile.print(angleDeg);
  // dataFile.print(",");


// BRAKE PRESSURE
  int rearRaw = analogRead(brakeRearPin);
  int frontRaw = analogRead(brakeFrontPin);

  double rearPSI_raw = convertToPSI(rearRaw);
  double frontPSI_raw = convertToPSI(frontRaw);
 
// ACCELEROMETER
  sensors_event_t event;
  accel.getEvent(&event);

  float x_g = (event.acceleration.x / 9.8);
  float y_g = (event.acceleration.y / 9.8);
  float z_g = (event.acceleration.z / 9.8);

// ENGINE RPM
   // Make local copies of ISR-updated variables safely.
  uint32_t periodCopy_us;
  uint32_t lastValidTimeCopy_us;

  // We briefly disable interrupts so we don't read half-updated values.
  noInterrupts();
  periodCopy_us = lastPulsePeriod_us;
  lastValidTimeCopy_us = lastValidPulseTime_us;
  interrupts();

  uint32_t now_us = micros();

  // If we have a valid period, compute RPM.
  if (lastValidTimeCopy_us == 0 || (now_us - lastValidTimeCopy_us) > TIMEOUT_us){
    engineRPM = 0.0f;
  }
  else if (periodCopy_us > 0) {
    // Frequency (pulses per second) = 1,000,000 / period_us
    // Pulses per minute = 60 * 1,000,000 / period_us = 60,000,000 / period_us
    // Revolutions per minute (RPM) = (pulses per minute) / (pulses per revolution)
    engineRPM = (60.0e6f / (float)periodCopy_us) / PULSES_PER_REVOLUTION;
  }

/*
// WHEEL RPM
  bool currentState = digitalRead(wheelRpmPin);
  if (currentState != lastState && currentState == HIGH) {
    unsigned long currentTime = millis();
    if (timerSinceLastTooth != 0) {
      double timeBetweenTeeth = currentTime - timerSinceLastTooth;
      if (timeBetweenTeeth > 0 && (timeBetweenTeeth < wheelTimeOut)) {
        // 60,000 ms/min
        wheelRPM = 60000 / ((double)timeBetweenTeeth * TEETH);
      }
      else{
        wheelRPM = 0;
      }
    }
    timerSinceLastTooth = currentTime;
  }
  lastState = currentState;

  if (millis() - timerSinceLastTooth > wheelTimeOut) {
    wheelRPM = 0;
  }

  dataFile.println(wheelRPM);
  */
// WRITE (100HZ)
  if (board_timer - write_timer >= 10){
    write_timer = board_timer;
    
    dataFile.print(timeStr);
    dataFile.print(",");
    dataFile.print(board_timer);
    dataFile.print(",");

    dataFile.print(frontPSI_raw);
    dataFile.print(",");
    dataFile.print(rearPSI_raw);
    dataFile.print(",");

    dataFile.print(x_g);
    dataFile.print(",");
    dataFile.print(y_g);
    dataFile.print(",");
    dataFile.print(z_g);
    dataFile.print(",");

    dataFile.print(engineRPM);
    dataFile.println(",");
  }
// FLUSH (1Hz)
  if (board_timer - lastFlush >= 1000) {
    dataFile.flush();
    lastFlush = board_timer;

// SERIAL DEBUG (1Hz)
    Serial.println("time,board_timer,rearPSI,frontPSI,accel_x,accel_y,accel_z,engineRPM,wheelRPM,runLoop");

    Serial.print(timeStr);       
    Serial.print(",");
    Serial.print(board_timer);   
    Serial.print(",");
    Serial.print(rearPSI_raw);   
    Serial.print(",");
    Serial.print(frontPSI_raw);  
    Serial.print(",");
    Serial.print(x_g, 3); 
    Serial.print(",");
    Serial.print(y_g / 9.8, 3); 
    Serial.print(",");
    Serial.print(z_g / 9.8, 3); 
    Serial.print(",");
    Serial.print(engineRPM);     
    Serial.print(",");
  /*  Serial.print(wheelRPM);      
    Serial.print(","); */
    Serial.println(runLoop);

// RUN LOOP RESET
  runLoop = 0;

  }

}
