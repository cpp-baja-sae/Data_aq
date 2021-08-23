#include <Arduino.h>
#include "ADS8588H.h"
#include "main.h"

IntervalTimer ADC_timer;
global_t global_values = {0};

int main()
{  
  setup_pins();
  Serial.begin(115200);
  ADS8588H_init(ADC_timer);

  while(1)
  {
    ADC_call_back(&global_values);
  }
  return 0;
}

void setup_pins()
{
  pinMode(ADC_CS_PIN,OUTPUT);
  pinMode(ADC_SCLK_PIN,OUTPUT);
  pinMode(ADC_RESET,OUTPUT);
  pinMode(ADC_CONV_AB,OUTPUT);
  pinMode(SG_ADC_1_DOUTA ,INPUT_PULLDOWN);
  pinMode(SG_ADC_2_DOUTA, INPUT_PULLDOWN);
}