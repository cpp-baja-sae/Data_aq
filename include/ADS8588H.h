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

#define ADC_NEGATIVE_TRANSISTION_VALUE 0x8000
#define ADC_16BIT_CONVERSION_VALUE 0xFFFF
#define ADC_ABS_RANGE 10.0
#define CLK_CYCLES 15
#define QTY_DIFF_CHANNELS 8

enum ADC_DIFFERENTIAL_CHANNELS{
    DIFF_1 = 0,
    DIFF_2,
    DIFF_3,
    DIFF_4,
    DIFF_5,
    DIFF_6,
    DIFF_7,
    DIFF_8
};

#include "main.h"

void ADS8588H_init(IntervalTimer &ADC_TIMER);
int ADC_call_back(global_t *work);
void ADS8588H_CONV_AB();
void ADS8588H_reset();
void ADC_SERVICE_ROUTINE();
void ADS8588H_CS();
void ADS8588H_READ_8CH();

#endif