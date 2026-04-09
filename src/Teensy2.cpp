#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
#include <cmath>
#include <TimeLib.h>

// SD CARD
const int chipSelect = BUILTIN_SDCARD;
File dataFile;
char fileName[20];
const int runNumberAddress = 0;

// LED
const int LED_PIN = LED_BUILTIN;

// TIME
unsigned long lastFlush = 0;

// THROTTLE (Commented Out)
const int CS_PIN = 10;

// BRAKES
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

// ACCELEROMETER
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

// ENGINE RPM
const int rpmPin = 23;  // Digital input pin that receives the RPM pulse signal

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

// WHEEL RPM (currently unused, kept for future)
// const int rpmPin = 4;
// const int TEETH = 14;
// const int microPerTooth = 60000000 / TEETH;
// unsigned long timerForTooth = 0;
// unsigned long timerForLastTooth = 0;
// unsigned long timer3 = 0;
// unsigned long timerSinceLastTooth = 0;
// int state = 0;
// int lastState = 0;
// double timeBetweenTeeth;
// double revMin;
// double lastRevMin = 0;
// const int stuffingCutOff = 1000;


void setup() {

// LED ON
  pinMode(LED_PIN, OUTPUT);

// THROTTLE SPI (currently not used)
  SPI.begin();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

// Engine RPM
  // Use INPUT if your sensor output is a clean push-pull digital signal.
  // Use INPUT_PULLUP only if your sensor output is open-collector/open-drain.
  pinMode(rpmPin, INPUT);
  // Attach interrupt on rising edge of the pulse signal.
  attachInterrupt(digitalPinToInterrupt(rpmPin), onRpmPulseRise, RISING);

// BRAKES
  pinMode(brakeRearPin, INPUT);
  pinMode(brakeFrontPin, INPUT);

  Serial.begin(9600);
  analogReadResolution(10);

// SD
  if (!SD.begin(chipSelect)) {
    Serial.println("Error: SD card initialization failed!");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(250);
      digitalWrite(LED_PIN, LOW);
      delay(250);
    }
  }

//RTC - When this is first uploaded to the new PCB, AFTER THE FIRST UPLOAD COMMENT THE TWO LINES BELOW THIS.
  setTime(14, 30, 0, 7, 4, 2026);
  Teensy3Clock.set(now());
  setSyncProvider(Teensy3Clock.get);

// ACCEL
  if (!accel.begin()) {
    Serial.println("Error: ADXL345 not detected.");
  }
  accel.setDataRate(ADXL345_DATARATE_200_HZ);

  // Run number from EEPROM
  int runNumber = EEPROM.read(runNumberAddress);
  runNumber++;
  EEPROM.write(runNumberAddress, runNumber);

// SD CARD
  sprintf(fileName, "Teensy2_%d.csv", runNumber);

  dataFile = SD.open(fileName, FILE_WRITE);

  if (dataFile) {
    dataFile.print("hours:minutes:seconds,");
    dataFile.print("millis,");  // total milliseconds


    // THROTTLE (currently disabled)
    // dataFile.print("Throttle Angle,");


    // Brake pressure (currently ADC counts)
    dataFile.print("PSI_Front,"); //Completely unfiltered data taken at 5ms marks
    dataFile.print("PSI_Rear,"); // Accounting for noise

    // ACCELEROMETER
    dataFile.print("Accel X g,");
    dataFile.print("Accel Y g,");
    dataFile.print("Accel Z g,");
    dataFile.print("Eng RPM");


    // WHEEL RPM (disabled)
    // dataFile.print(",Rear RPM");


    dataFile.println();


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

// SD CARD FAIL INDICATOR
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
  unsigned long timer = millis();
  // THE ABOVE IS SPECIFICALLY FOR SD WRITING.
  char timeStr[20];
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", hour(), minute(), second());

  dataFile.print(timeStr);
  dataFile.print(",");
  dataFile.print(timer);
  dataFile.print(",");

// THROTTLE (DISABLED L22)
  // digitalWrite(CS_PIN, LOW);
  // SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
  // uint16_t result = SPI.transfer16(0x0000);
  // SPI.endTransaction();
  // digitalWrite(CS_PIN, HIGH);
  // uint16_t angleRaw = (result >> 6) & 0x03FF;  // 10-bit angle (0–1023)
  // float angleDeg = (angleRaw / 1023.0) * 360.0;
  // dataFile.print(angleDeg);
  // dataFile.print(",");


// BRAKE PRESSURE (ADC counts)
  int rearRaw = analogRead(brakeRearPin);
  int frontRaw = analogRead(brakeFrontPin);

  double rearPSI_raw = convertToPSI(rearRaw);
  double frontPSI_raw = convertToPSI(frontRaw);
 
  dataFile.print(frontPSI_raw);
  dataFile.print(",");
  dataFile.print(rearPSI_raw);
  dataFile.print(",");

// ACCELEROMETER
  sensors_event_t event;
  accel.getEvent(&event);

  dataFile.print(event.acceleration.x / 9.8);
  dataFile.print(",");
  dataFile.print(event.acceleration.y / 9.8);
  dataFile.print(",");
  dataFile.print(event.acceleration.z / 9.8);
  dataFile.print(",");

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

  dataFile.println(engineRPM);
  
  // Flush every 1000 ms
  if (timer - lastFlush >= 1000) {
    dataFile.flush();
    lastFlush = timer;
  }

}
