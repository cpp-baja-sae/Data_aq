#include "main.h"
#include "ADS8588H.h"
#include "mux.h"
#include <Arduino.h>
#include <WProgram.h>

global_t global_values = {0};
elapsedMillis heart_beat_timer;
elapsedMicros ADC_Timer;
elapsedMicros Digital_Timer;
elapsedMicros LEDTESTING;
IntervalTimer digital;
dumb_scheduler timer;
Digital_Muxed_Data_t Digital = {0};
int period = 10;
unsigned long time_now = 0;

void digital_work() {
  digitalWrite(32, HIGH);
  digital_mux(&Digital);
  digitalWrite(32, LOW);
}
void digital_setup() {
  digital.begin(digital_work, 20);
  digital.priority(32);
}

int main() {

  setup_pins();
  Serial.begin(115200);
  ADS8588H_init();
  digital_setup();
  int yo = 0;
  while (1) {
    // ADC task
    if (ADC_Timer >= Timer_interval_2khz) {
      // digitalWrite(24, HIGH);
      ADC_Timer = ADC_Timer - Timer_interval_2khz;
      ADC_SERVICE_ROUTINE();
      // digitalWrite(24, LOW);
    }
    // Heart beat task
    if (heart_beat_timer >= TIMER_INTERVAL_1s) {
      if (yo == 1) {
        yo = 0;
        digitalWrite(24, LOW);
        digitalWrite(25, HIGH);
      } else {

        yo = 1;
        digitalWrite(24, HIGH);
        digitalWrite(25, LOW);
      }
      heart_beat_timer = heart_beat_timer - TIMER_INTERVAL_1s;
      timer.Heart_Beat(timer);
    }
    if (LEDTESTING >= Testing) {
      LEDTESTING = LEDTESTING - Testing;

      digitalWrite(26, LOW);
      delay_5ns(1000000);

      digitalWrite(26, HIGH);
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
  pinMode(DIGITAL_MUX_IN, INPUT_PULLDOWN);
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(32, OUTPUT);
}
