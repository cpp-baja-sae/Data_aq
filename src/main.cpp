#include <Arduino.h>
#include "ADS8588H.h"
#include "main.h"

IntervalTimer ADC_timer;
global_t global_values = {0};

int main()
{  
  pinMode(ADC_CS_PIN,OUTPUT);
  pinMode(ADC_SCLK_PIN,OUTPUT);
  pinMode(ADC_RESET,OUTPUT);
  pinMode(ADC_CONV_AB,OUTPUT);
  pinMode(ADC_1_DOUTA ,INPUT_PULLDOWN);
  Serial.begin(115200);
  ADS8588H_init(ADC_timer);

  while(1)
  {
    ADC_call_back(&global_values);
  }
  return 0;
}