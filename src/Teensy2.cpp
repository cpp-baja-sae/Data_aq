#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
#include <cmath>
// #include <Adafruit_MLX90614.h>  // not used yet - note from kareem, i dont think our temp sensor is Adafruit i believe its smt else.

const int chipSelect = BUILTIN_SDCARD;
File dataFile;
char fileName[20];
const int runNumberAddress = 0;


const int LED_PIN = LED_BUILTIN;


// TIME
unsigned long lastFlush = 0;


// THROTTLE (commented out for now)
const int CS_PIN = 10;


// BRAKES
const int brakeRearPin  = 21; //Changed from 40 to match PCB 
const int brakeFrontPin = 22; //Changed from 41 to match PCB
// Our Brake Sensors read 0-2000 PSI from .5V -> 4.5V.
const int PSImax = 2000;
const int PSImin = 0;
//Maximum and Minimum voltages our pin is reading from the voltage divider.
double Vmax = 3.214; // Current Voltage Divider values. Update when V2 comes to 3.3V
double Vmin = .357; // Current Voltage Divider Values

double readtoPSI(int readVoltage){
  double V = readVoltage * 3.3 / 1023;
  float psi = ((V-Vmin) * (PSImax - PSImin) / (Vmax - Vmin)) + PSImin;
  //Mapping .357V -> 3.214V into 0-2000 PSI
  return psi;
}

// ACCELEROMETER
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

// ENGINE RPM
const int rpmPin = 23;  // Digital input pin that receives the RPM pulse signal

const float PULSES_PER_REVOLUTION = 0.5f;  

  // Timestamp (in microseconds) of the most recent detected rising edge (pulse).
volatile uint32_t lastPulseTime_us = 0;

  // Time difference (in microseconds) between the last two valid pulses.
  // This is the pulse period we use to compute RPM.
volatile uint32_t lastPulsePeriod_us = 0;

  // -----------------------------
  // Noise filtering / sanity limits
  // -----------------------------

  // If pulses per revolution = 0.5, then
  //   RPM = (60e6 / period_us) / 0.5 = 120e6 / period_us
  // If period_us = 40,000 us => RPM ~ 4000, which is above our expected range.
  // So anything faster than 40,000 us is noise/ringing.
const uint32_t MIN_VALID_PERIOD_us = 15000;

  // Reject pulses that are extremely slow (could be a stopped engine / bad wiring).
  // 300000 us = 0.3 s between pulses
const uint32_t MAX_VALID_PERIOD_us = 300000;

  // If we haven’t seen a pulse for this long, force RPM to 0.
const uint32_t NO_PULSE_TIMEOUT_us = 300000;

  // -----------------------------
  // Output (computed) RPM
  // -----------------------------
float engineRPM = 0.0f;

  /*
  Interrupt Service Routine (ISR)
  Runs immediately when a rising edge is detected on rpmPin.

  What it does:
  1) Reads current time in microseconds.
  2) Computes period since previous pulse.
  3) If that period looks valid (not noise), stores it for the main loop to use.
*/
void onRpmPulseRise() {
  uint32_t now_us = micros();

  // If this isn't the very first pulse, we can compute the time between pulses.
  if (lastPulseTime_us != 0) {
    uint32_t period_us = now_us - lastPulseTime_us;

    // Basic noise filtering:
    // - Too short  => likely electrical noise / ringing
    // - Too long   => likely not a real running engine pulse (or first pulse after stop)
    if (period_us >= MIN_VALID_PERIOD_us && period_us <= MAX_VALID_PERIOD_us) {
      lastPulsePeriod_us = period_us;
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


  // ACCEL
  if (!accel.begin()) {
    Serial.println("Error: ADXL345 not detected.");
  }


  // Run number from EEPROM
  int runNumber = EEPROM.read(runNumberAddress);
  runNumber++;
  EEPROM.write(runNumberAddress, runNumber);


  sprintf(fileName, "Teensy2_%d.csv", runNumber);


  dataFile = SD.open(fileName, FILE_WRITE);


  if (dataFile) {
    dataFile.print("hours:minutes:seconds,");
    dataFile.print("millis,");  // total milliseconds


    // THROTTLE (currently disabled)
    // dataFile.print("Throttle Angle,");


    // Brake pressure (currently ADC counts)
    dataFile.print("Rear PSI,");
    dataFile.print("RB_Raw,");
    dataFile.print("Front PSI,");
    dataFile.print("FB_Raw,");


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
  if (!dataFile) {
    Serial.println("Error: dataFile invalid.");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(250);
      digitalWrite(LED_PIN, LOW);
      delay(250);
    }
  }


  // TIME
  unsigned long timer = millis(); // This is super unreliable, I (Kareem) Highly recommend switching to an RTC Clock w/ Backup 3.2V Ion Cell (unsure if against rules)
  unsigned int seconds = (timer / 1000) % 60;
  unsigned int minutes = (timer / 60000) % 60;
  unsigned int hours   = (timer / 3600000) % 60;
  String timeStr = String(hours) + ":" + String(minutes) + ":" + String(seconds);


  dataFile.print(timeStr);
  dataFile.print(",");
  dataFile.print(timer);
  dataFile.print(",");  // <-- separator before first brake value
  Serial.println("Time: " + timer);

  // THROTTLE (currently disabled)
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
  int rearADC  = analogRead(brakeRearPin);
  int frontADC = analogRead(brakeFrontPin);
  double PSI_Rear = readtoPSI(rearADC);
  double PSI_Front = readtoPSI(frontADC);

  Serial.print("PSI_R = ");
  Serial.print(PSI_Rear);
  Serial.print(" PSI_VoltageR: ");
  Serial.print(rearADC);
  Serial.print(" PSI_F = ");
  Serial.print(PSI_Front);
  Serial.print(" PSI_VoltageF: ");
  Serial.println(frontADC);
 

  dataFile.print(PSI_Rear);
  dataFile.print(",");
  dataFile.print(rearADC);
  dataFile.print(",");
  dataFile.print(PSI_Front);
  dataFile.print(",");
  dataFile.print(frontADC);
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
  
  Serial.print("X: ");
  Serial.print(event.acceleration.x/9.8);
  Serial.print(" Y: ");
  Serial.print(event.acceleration.y/9.8);
  Serial.print(" Z: ");
  Serial.println(event.acceleration.z/9.8);

// WHEEL RPM block is currently commented out


// ENGINE RPM
   // Make local copies of ISR-updated variables safely.
  uint32_t periodCopy_us;
  uint32_t lastTimeCopy_us;

  // We briefly disable interrupts so we don't read half-updated values.
  noInterrupts();
  periodCopy_us   = lastPulsePeriod_us;
  lastTimeCopy_us = lastPulseTime_us;
  interrupts();

  uint32_t now_us = micros();

  // If we haven't seen a pulse for a while, treat the engine as stopped.
  if (lastTimeCopy_us == 0 || (now_us - lastTimeCopy_us) > NO_PULSE_TIMEOUT_us) {
    engineRPM = 0.0f;
  }
  // If we have a valid period, compute RPM.
  else if (periodCopy_us > 0) {
    // Frequency (pulses per second) = 1,000,000 / period_us
    // Pulses per minute = 60 * 1,000,000 / period_us = 60,000,000 / period_us
    // Revolutions per minute (RPM) = (pulses per minute) / (pulses per revolution)
    engineRPM = (60.0e6f / (float)periodCopy_us) / PULSES_PER_REVOLUTION;
  }

  dataFile.print(engineRPM);
  dataFile.println(",");
  
  Serial.print("RPM: ");
  Serial.println(engineRPM);

  // Flush every 1000 ms
  if (timer - lastFlush >= 1000) {
    dataFile.flush();
    lastFlush = timer;
  }
delay(2);
}
