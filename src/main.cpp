#include "main.h"
#include "../.pio/libdeps/teensy41/TinyGPSPlus/src/TinyGPS++.h"
#include "ADS8588H.h"
#include "mux.h"
#include <Arduino.h>
#include <FlexIO_t4.h>
#include <FlexSerial.h>
#include <SoftwareSerial.h>
#include <WProgram.h>

global_t global_values = {0};
elapsedMillis heart_beat_timer;
elapsedMicros ADC_Timer;
elapsedMillis GPS_Timer;
TinyGPSPlus gps;

int main() {
  dumb_scheduler timer;
  setup_pins();
  Serial.begin(9600);
  Serial8.begin(9600);
  int state = 0;
  ADS8588H_init();
  while (1) {
    // ADC task
    if (ADC_Timer >= Timer_interval_2khz) {
      digitalWrite(32, HIGH);
      ADC_Timer = ADC_Timer - Timer_interval_2khz;
      ADC_SERVICE_ROUTINE();
      digitalWrite(32, LOW);
    }
    // Heart beat task
    if (heart_beat_timer >= heart_beat_compare_time_ms) {
      heart_beat_timer = heart_beat_timer - heart_beat_compare_time_ms;
      timer.Heart_Beat(timer);
    }
    // GPS

    while (Serial8.available()) {
      if (state == 0) {
        digitalWrite(33, HIGH);
        state = 1;
      }

      if (gps.encode(Serial8.read())) {
        Serial.println(gps.time.second());
        Serial.print("LNG=");
        Serial.println(gps.location.lng(), 6);
        Serial.print("LNG=");
        Serial.println(gps.location.lat(), 6);
      }
    }
    if (state == 1) {
      digitalWrite(33, LOW);
      state = 0;
    }
  }
  return 0;
}
void setup_pins() {
  pinMode(ADC_CS_PIN, OUTPUT);
  pinMode(ADC_SCLK_PIN, OUTPUT);
  pinMode(ADC_RESET, OUTPUT);
  pinMode(ADC_CONV_AB, OUTPUT);
  pinMode(SG_ADC_1_DOUTA, INPUT_PULLDOWN);
  pinMode(SG_ADC_2_DOUTA, INPUT_PULLDOWN);
  pinMode(A0_TC_1_4, INPUT);
  pinMode(A1_TC_1_4, OUTPUT);
  pinMode(A0_TC_5_8, OUTPUT);
  pinMode(A1_TC_5_8, OUTPUT);
  pinMode(CORE_LED0_PIN, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(35, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(34, INPUT);
  pinMode(32, OUTPUT);
  pinMode(7, INPUT_PULLUP);
}