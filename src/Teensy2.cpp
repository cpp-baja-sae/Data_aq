#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
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
const int breakRearPin  = 22; //Changed from 40 to match PCB 
const int breakFrontPin = 21; //Changed from 41 to match PCB
// Our Brake Sensors read 0-2000 PSI from .5V -> 4.5V.
const int PSImax = 2000;
const int PSImin = 0;
//Maximum and Minimum voltages our pin is reading from the voltage divider.
const int Vmax = 3.214; // Current Voltage Divider values. Update when V2 comes to 3.3V
const int Vmin = .357; // Current Voltage Divider Values

double readtoPSI(int readVoltage){
  double V = read * 3.3 / 1023;
  float psi = ((V-Vmin) * (PSImax - PSImin) / (Vmax - Vmin)) + PSImin;
  //Mapping .357V -> 3.214V into 0-2000 PSI
  return psi;
}

// ACCELEROMETER
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

// ENGINE RPM
const int rpmPin = 23; // Updated from previous, was 33. Incorrect based on PCBs. 
unsigned long timeOfSpark = 0;
unsigned long prevTimeOfSpark = 0; // Could change these, idk if we need to be storing this much data - calculations estimate for roughly 30 ms timeBetweenSparks
unsigned long timeBetweenSparks = 0;
double engineRPM = 0;
bool prevState = LOW;

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
  pinMode(rpmPin, INPUT);
  digitalWrite(CS_PIN, HIGH);


  // BRAKES
  pinMode(breakRearPin, INPUT);
  pinMode(breakFrontPin, INPUT);


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
    dataFile.print("Rear ADC,");
    dataFile.print("Front ADC,");


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
  int rearADC  = analogRead(breakRearPin);
  int frontADC = analogRead(breakFrontPin);


  dataFile.print(rearADC);
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


  // WHEEL RPM block is currently commented out


  // ENGINE RPM
  bool state = digitalRead(rpmPin);
    if (state != prevState && state == HIGH) {
      unsigned long currentSparkTime = millis();
      if (prevTimeOfSpark != 0) {
        timeBetweenSparks = currentSparkTime - prevTimeOfSpark;
        if (timeBetweenSparks > 0) {
          // 120000 = 60,000 ms/min * 2 revs/spark (example: .5 pulses per rev)
          engineRPM = 120000.0 / (double)timeBetweenSparks;
        }
      }

      prevTimeOfSpark = currentSparkTime;
    }
    prevState = state;
 
  dataFile.print(engineRPM);
  dataFile.println();


  // Flush every 1000 ms
  if (timer - lastFlush >= 1000) {
    dataFile.flush();
    lastFlush = timer;
  }
delay(2);
}
