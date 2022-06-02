/*
 * ADS8588H.h
 *
 *  Created on: Jun 2, 2022
 *      Author: forre
 */

#ifndef INC_ADS8588H_H_
#define INC_ADS8588H_H_
#include "main.h"
#include "stdint.h"

#define BUSYWAIT 1800
#define SMALLWAIT 100
#define HalfDutyCycle (TIM14->ARR / 2)

#define PWM_BIT_RESOLUTION 10
#define ADC_CS_PIN 0
#define ADC_SCLK_PIN 1
#define ADC_RESET 2
#define ADC_CONV_AB 3
#define SG_ADC_1_DOUTB 18
#define SG_ADC_1_DOUTA 19
#define SG_ADC_2_DOUTB 15
#define SG_ADC_2_DOUTA 14
#define G_ADC_1_DOUTB 41
#define G_ADC_1_DOUTA 40
#define G_ADC_2_DOUTB 16
#define G_ADC_2_DOUTA 17

#define approx_500NS 100
#define WAIT_OUT_CONVERSION_TIME_600us 120000
#define WAIT_OUT_CONVERSION_TIME_200us 40000

#define Timer_interval_2khz 500
#define MUX_1_SECOND 2000

#define ADC_NEGATIVE_TRANSISTION_VALUE 0x8000
#define ADC_16BIT_CONVERSION_VALUE 0xFFFF
#define ADC_ABS_RANGE 10.0
#define CLK_CYCLES 15
#define QTY_DIFF_CHANNELS 8
#define NOT_TC_CHANNEL 2
#define TC_CHANNEL_0 0
#define TC_CHANNEL_1 1
#define TC_CHANNEL_1_DATA_SHIFT 4
#define TC_CHANNEL_SHIFT (8 - NOT_TC_CHANNEL)

enum ADC_DIFFERENTIAL_CHANNELS {
  DIFF_1 = 0,
  DIFF_2,
  DIFF_3,
  DIFF_4,
  DIFF_5,
  DIFF_6,
  DIFF_7,
  DIFF_8
};

enum ADC_TC_MUXING {
  TC_1 = 0,
  TC_2,
  TC_3,
  TC_4,
  TC_5,
  TC_6,
  TC_7,
  TC_8,
  SINGLE_ENDED_1,
  SINGLE_ENDED_2,
  SINGLE_ENDED_3,
  SINGLE_ENDED_4,
  SINGLE_ENDED_5,
  SINGLE_ENDED_6
};
typedef struct ADC_DATA_t {
	uint16_t ADC1_8;
	uint16_t ADC9_16;
	uint16_t ADC17_24;
	uint16_t DataPoint;
} ADC_DATA_t;

void ADS8588H_init();
void ADS8588H_CONV_AB();
void ADS8588H_reset();
void ADC_SERVICE_ROUTINE(ADC_DATA_t *ADC_DATA);
void ADS8588H_CS();
void ADS8588H_READ_8CH(ADC_DATA_t *ADC_DATA);
//void convertData(int channel, ADC_Regular_Data_t *ADC);


#endif /* INC_ADS8588H_H_ */
