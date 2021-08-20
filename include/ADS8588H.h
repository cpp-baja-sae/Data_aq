#ifndef __ADS8588H__
#define __ADS8588H__

#define PWM_BIT_RESOLUTION 10
#define ADC_CS_PIN 0
#define ADC_SCLK_PIN 1
#define ADC_RESET 2
#define ADC_CONV_AB 3
#define ADC_1_DOUTB 18
#define ADC_1_DOUTA 19

#define approx_500NS 100
#define WAIT_OUT_CONVERSION_TIME_600us 120000

#define Timer_interval_2khz 500

#include "main.h"

void ADS8588H_init(IntervalTimer &ADC_TIMER);
int ADC_call_back(global_t *work);
void ADS8588H_CONV_AB();
void ADS8588H_reset();
void ADC_SERVICE_ROUTINE();
void ADS8588H_CS();
void ADS8588H_READ_8CH();
#endif