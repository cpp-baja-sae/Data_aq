/*
 * ADS8588H.c
 *
 *  Created on: Jun 2, 2022
 *      Author: forre
 */


#include "ADS8588H.h"
#include "main.h"
#include "HelperCommand.h"

void ADC_SERVICE_ROUTINE(ADC_DATA_t *ADC_DATA) {
  // Begin conversion
  ADS8588H_CONV_AB();
  /** TODO, Should not need this,
   * Board needs a interrupt tied to falling edge of BUSY(?)
   * Wait out BUSY and OSR
   */
  // Replace with pin read instead.
  TIM14WaitUntil(BUSYWAIT);
  //delay_5ns(WAIT_OUT_CONVERSION_TIME_200us);

  ADS8588H_READ_8CH(ADC_DATA);
  return;
}

void ADS8588H_READ_8CH(ADC_DATA_t *ADC_DATA) {
  // Lower CS to enable data conversion
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, RESET);
  for (int channel = 0; channel < QTY_DIFF_CHANNELS; channel++)
  {
    for (int stepping = CLK_CYCLES; stepping >= 0; stepping--)
    {
    	//Timer is very sensitive to abstraction ???
		while(TIM14->CNT <=HalfDutyCycle/2);
		TIM14->CNT = RESET;
		HAL_GPIO_WritePin(ADC_SCLK_GPIO_Port,ADC_SCLK_Pin,RESET);

		// Reading GPIO pins directly and adjusting register value as needed
		ADC_DATA->ADC1_8[channel] |= (((ADC1_8_GPIO_Port->IDR & ADC1_8_Pin) >> 7 ) & 1U )<< stepping;

		ADC_DATA->ADC9_16[channel] |= (((ADC9_16_GPIO_Port->IDR & ADC9_16_Pin)>> 8 ) & 1U )<< stepping;

		ADC_DATA->ADC17_24[channel] |= (((ADC17_24_GPIO_Port->IDR & ADC17_24_Pin) >> 3 ) & 1U) << stepping;

		HAL_GPIO_WritePin(ADC_SCLK_GPIO_Port,ADC_SCLK_Pin,SET);
    }
  }
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, RESET);
  ADC_DATA->DataPoint++;
}



//void convertData(int channel, ADC_Regular_Data_t *ADC) {
//  ADC->raw_data[channel] = ADC->bit_collector;
//  // Translate raw value into float
//  if (ADC->raw_data[channel] <= ADC_NEGATIVE_TRANSISTION_VALUE) {
//    ADC->scaled_data[channel] =
//        ADC_ABS_RANGE * ADC->raw_data[channel] / ADC_16BIT_CONVERSION_VALUE;
//  } else {
//    ADC->scaled_data[channel] =
//        ADC_ABS_RANGE * ADC->raw_data[channel] / ADC_16BIT_CONVERSION_VALUE -
//        ADC_ABS_RANGE;
//  }
//  ADC->bit_collector = 0;
//}

void ADS8588H_CONV_AB() {
	HAL_GPIO_WritePin(ADC_CONV_AB_GPIO_Port, ADC_CONV_AB_Pin, RESET);
	TIM14WaitUntil(SMALLWAIT);
  	HAL_GPIO_WritePin(ADC_CONV_AB_GPIO_Port, ADC_CONV_AB_Pin, SET);
}

void ADS8588H_reset() {
	TIM14WaitUntil(SMALLWAIT);
	HAL_GPIO_WritePin(ADC_RESET_GPIO_Port, ADC_RESET_Pin, SET);
	TIM14WaitUntil(SMALLWAIT);
	HAL_GPIO_WritePin(ADC_RESET_GPIO_Port, ADC_RESET_Pin, RESET);
}

void ADS8588H_init() {
  /*
   * From section 7.4.1.6 of data sheet
   * Needed for settling internal references.
   */
  ADS8588H_reset();
  return;
}
