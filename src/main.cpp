#include "main.h"
#include "ADS8588H.h"
#include "mux.h"
#include <Arduino.h>
#include <WProgram.h>

global_t global_values = {0};
elapsedMillis heart_beat_timer;
elapsedMicros ADC_Timer;

int main() {
  dumb_scheduler timer;
  setup_pins();
  Serial.begin(115200);
  ADS8588H_init();
  while (1) {
    // ADC task
    if (ADC_Timer >= Timer_interval_2khz) {
      ADC_Timer = ADC_Timer - Timer_interval_2khz;
      ADC_SERVICE_ROUTINE();
    }
    // Heart beat task
    if (heart_beat_timer >= heart_beat_compare_time_ms) {
      heart_beat_timer = heart_beat_timer - heart_beat_compare_time_ms;
      timer.Heart_Beat(timer);
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
  pinMode(A0_TC_1_4, OUTPUT);
  pinMode(A1_TC_1_4, OUTPUT);
  pinMode(A0_TC_5_8, OUTPUT);
  pinMode(A1_TC_5_8, OUTPUT);
  pinMode(CORE_LED0_PIN, OUTPUT);
  pinMode(4, OUTPUT);
}