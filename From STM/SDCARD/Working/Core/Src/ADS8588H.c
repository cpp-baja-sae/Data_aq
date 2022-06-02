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
		while(TIM14->CNT <=HalfDutyCycle);
		TIM14->CNT = RESET;
		HAL_GPIO_WritePin(ADC_SCLK_GPIO_Port,ADC_SCLK_Pin,RESET);

		ADC_DATA->ADC1_8 |= (HAL_GPIO_ReadPin(ADC1_8_GPIO_Port,ADC1_8_Pin)<< stepping);
		ADC_DATA->ADC9_16 |= (HAL_GPIO_ReadPin(ADC9_16_GPIO_Port,ADC9_16_Pin)<< stepping);
		ADC_DATA->ADC17_24 |= (HAL_GPIO_ReadPin(ADC17_24_GPIO_Port,ADC17_24_Pin)<< stepping);
		ADC_DATA->DataPoint++;
		HAL_GPIO_WritePin(ADC_SCLK_GPIO_Port,ADC_SCLK_Pin,SET);

    }
//    convertData(channel, &ADC_Strain_Guage_1);
//    convertData(channel, &ADC_Strain_Guage_2);
//    convertData(channel, &ADC_General_Purpose_1);
  }
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, RESET);
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
