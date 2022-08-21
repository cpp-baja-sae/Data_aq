/*
 * ADS8588H.c
 *
 *  Created on: Jul 15, 2022
 *      Author: forre
 */
#include "ADS8588H.h"
#include "main.h"

#define CONVERSION_CODE	0xFFFF

void convert_data(ADS8588H_Interface_t *ADC);

void ADS8588H_Init_Struct(ADS8588H_Interface_t	*ADC, \
		TIM_HandleTypeDef	*htim,
		GPIO_TypeDef	*ADC_Reset_Port,	uint16_t 	ADC_Reset_pin,
		GPIO_TypeDef	*ADC_Conv_AB_Port,	uint16_t 	ADC_Conv_AB_pin,
		GPIO_TypeDef	*ADC_CS_Port,	uint16_t 	ADC_CS_pin,
		GPIO_TypeDef	*ADC_BUSY_1_Port, uint16_t 	ADC_BUSY_1_pin,
		GPIO_TypeDef	*ADC_BUSY_2_Port, uint16_t 	ADC_BUSY_2_pin,
		GPIO_TypeDef	*ADC_BUSY_3_Port, uint16_t 	ADC_BUSY_3_pin,
		GPIO_TypeDef	*ADC_OS0_Port, uint16_t 	ADC_OS0_pin,
		GPIO_TypeDef	*ADC_OS1_Port, uint16_t 	ADC_OS1_pin,
		GPIO_TypeDef	*ADC_OS2_Port, uint16_t 	ADC_OS2_pin,
		uint8_t OSR,
		OSPI_HandleTypeDef	*hopsi
		)
{
	/*
	 * Link timer source to ADC driver
	 */
	ADC->ADC_Time.ADC_htim = htim;

	/*
	 * Load time delay base defaults
	 */
	ADS8588H_Time_Delay_Base(ADC);

	/*
	 * Link OPSI source to ADC driver
	 */
	ADC->OPSI.hopsi = hopsi;

	/*
	 * Load OSR mask
	 */

	ADC->ADC_OSR.OSR = OSR;

	/*
	 * Link Reset GPIO pins to ADC driver
	 */
	ADC->ADC_GPIO.ADC_Reset_Port = ADC_Reset_Port;
	ADC->ADC_GPIO.ADC_Reset_pin = ADC_Reset_pin;

	/*
	 * Link Conv_AB GPIO pins to ADC driver
	 */
	ADC->ADC_GPIO.ADC_Conv_AB_Port = ADC_Conv_AB_Port;
	ADC->ADC_GPIO.ADC_Conv_AB_pin = ADC_Conv_AB_pin;

	/*
	 * Link CS GPIO pins to ADC driver
	 */
	ADC->ADC_GPIO.ADC_CS_Port = ADC_CS_Port;
	ADC->ADC_GPIO.ADC_CS_pin = ADC_CS_pin;

	/*
	 * Link BUSY 1 GPIO pins to ADC driver
	 */
	ADC->ADC_GPIO.ADC_BUSY_1_Port = ADC_BUSY_1_Port;
	ADC->ADC_GPIO.ADC_BUSY_1_pin = ADC_BUSY_1_pin;

	/*
	 * Link BUSY 2 GPIO pins to ADC driver
	 */
	ADC->ADC_GPIO.ADC_BUSY_2_Port = ADC_BUSY_2_Port;
	ADC->ADC_GPIO.ADC_BUSY_2_pin = ADC_BUSY_2_pin;

	/*
	 * Link BUSY 3 GPIO pins to ADC driver
	 */
	ADC->ADC_GPIO.ADC_BUSY_3_Port = ADC_BUSY_3_Port;
	ADC->ADC_GPIO.ADC_BUSY_3_pin = ADC_BUSY_3_pin;

	/*
	 * Link OSR 0 GPIO pins to ADC driver
	 */
	ADC->ADC_GPIO.ADC_OS0_Port = ADC_OS0_Port;
	ADC->ADC_GPIO.ADC_OS0_pin = ADC_OS0_pin;

	/*
	 * Link OSR 1 GPIO pins to ADC driver
	 */
	ADC->ADC_GPIO.ADC_OS1_Port = ADC_OS1_Port;
	ADC->ADC_GPIO.ADC_OS1_pin = ADC_OS1_pin;

	/*
	 * Link OSR 2 GPIO pins to ADC driver
	 */
	ADC->ADC_GPIO.ADC_OS2_Port = ADC_OS2_Port;
	ADC->ADC_GPIO.ADC_OS2_pin = ADC_OS2_pin;

}

void ADS8588H_Time_Delay_Base(ADS8588H_Interface_t *ADC)
{
	ADC->ADC_Time.Delay.convAB_Delay = DEFAULT_CONVAB_DELAY;
	ADC->ADC_Time.Delay.reset_Delay = DEFAULT_RESET_DELAY;
}

void ADS8588H_Init(ADS8588H_Interface_t *ADC)
{
	/*
	 * Initialize time delay source
	 */
	HAL_TIM_Base_Start(ADC->ADC_Time.ADC_htim);

	/*
	 * From section 7.3.7 of the data sheet
	 * Configure OSR bits for internal digital filtering
	 */
	ADS8588H_OSR_SETUP(ADC);

	/*
	* From section 7.4.1.6 of data sheet
	* Needed for settling internal references.
	*/
	ADS8588H_Reset(ADC);
}

void ADS8588H_OSR_SETUP(ADS8588H_Interface_t *ADC)
{
	/*
	 * First process bit field
	 */
	ADC->ADC_OSR.OS0 = ADC->ADC_OSR.OSR & OSR_BIT_0;
	ADC->ADC_OSR.OS1 = (ADC->ADC_OSR.OSR & OSR_BIT_1) >> OSR_BIT_1_MASK;
	ADC->ADC_OSR.OS2 = (ADC->ADC_OSR.OSR & OSR_BIT_2) >> OSR_BIT_2_MASK;

	/*
	 * Set OSR values.
	 */
	HAL_GPIO_WritePin(ADC->ADC_GPIO.ADC_OS0_Port,
			ADC->ADC_GPIO.ADC_OS0_pin,
			ADC->ADC_OSR.OS0);
	HAL_GPIO_WritePin(ADC->ADC_GPIO.ADC_OS1_Port,
			ADC->ADC_GPIO.ADC_OS1_pin,
			ADC->ADC_OSR.OS1);
	HAL_GPIO_WritePin(ADC->ADC_GPIO.ADC_OS2_Port,
			ADC->ADC_GPIO.ADC_OS2_pin,
			ADC->ADC_OSR.OS2);
}

void ADS8588H_Reset(ADS8588H_Interface_t *ADC)
{
	HAL_GPIO_WritePin(ADC->ADC_GPIO.ADC_Reset_Port,
			ADC->ADC_GPIO.ADC_Reset_pin,
			SET);

	ADC_Delay_us(ADC,ADC->ADC_Time.Delay.reset_Delay);

	HAL_GPIO_WritePin(ADC->ADC_GPIO.ADC_Reset_Port,
			ADC->ADC_GPIO.ADC_Reset_pin,
			RESET);
}

void convert_data(ADS8588H_Interface_t *ADC)
{
//	uint16_t temp1 = 0;
//	uint16_t temp2 = 0;

	for(int x = 0; x < 8; x++)
	{
//		for(int i = 0; i < 16; i++)
//		{
//			temp1 = temp1 | (uint16_t)((ADC->DATA.raw_data[i + 16*x] & 0x01) << (15 - i));
//			temp2 = temp2 | (uint16_t)((ADC->DATA.raw_data[i + 16*x] & 0x02) << (15 - i));
//		}
		if( (ADC->DATA.raw_data_16[x] & 0x8000) == 0x8000)
		{
			/*negative*/
			ADC->DATA.data[x] = ADC->DATA.raw_data_16[x] * 10.0/0xFFFF - 10.0;
		}
		else
		{
			/*positive*/
			ADC->DATA.data[x] = ADC->DATA.raw_data_16[x] * 10.0/0xFFFF;
		}
//		if( (temp2 & 0x8000) == 0x8000)
//		{
//			/*negative*/
//			ADC->DATA.data[x+4] = temp2 * 10.0/0xFFFF - 10.0;
//		}
//		else
//		{
//			/*positive*/
//			ADC->DATA.data[x+4] = temp2 * 10.0/0xFFFF;
//		}

	}




}

void ADC_SERVICE_ROUTINE(ADS8588H_Interface_t *ADC)
{
	ADS8588H_CONV_AB(ADC);
	ADS8588H_POLL_BUSY(ADC);
	ADS8588H_READ_ALL(ADC);
	convert_data(ADC);

}

void ADS8588H_READ_ALL(ADS8588H_Interface_t *ADC)
{

	/*
	 * Select ADC, enable low
	 */
	HAL_GPIO_WritePin(ADC->ADC_GPIO.ADC_CS_Port,
			ADC->ADC_GPIO.ADC_CS_pin,
			RESET);
	ADC_Delay_us(ADC,5);
	/*
	 * Read adc stuff
	 */
	for(int x = 0; x<8; x++)
	{
		ADC->DATA.raw_data_16[x] = 0;
		for(int i = 15; i >= 0; i--)
		{
			HAL_GPIO_WritePin(ADC_CLK_GPIO_Port, ADC_CLK_Pin,RESET);
			ADC_Delay_us(ADC,1);

			ADC->DATA.raw_data_16[x] |= HAL_GPIO_ReadPin(ADC_CH_A_GPIO_Port, ADC_CH_A_Pin) << i;

			HAL_GPIO_WritePin(ADC_CLK_GPIO_Port, ADC_CLK_Pin,SET);
			ADC_Delay_us(ADC,1);
		}

	}
//	ADC->OPSI.res = HAL_OSPI_Receive(ADC->OPSI.hopsi,ADC->DATA.raw_data ,HAL_MAX_DELAY-1);
//	if(ADC->OPSI.res != HAL_OK) Error_Handler();
//	ADC->OPSI.hopsi->State = HAL_OSPI_STATE_CMD_CFG;

	/*
	 * Release ADC
	 */
	HAL_GPIO_WritePin(ADC->ADC_GPIO.ADC_CS_Port,
			ADC->ADC_GPIO.ADC_CS_pin,
			SET);
}

void ADS8588H_CONV_AB(ADS8588H_Interface_t *ADC)
{
	HAL_GPIO_WritePin(ADC->ADC_GPIO.ADC_Conv_AB_Port,
			ADC->ADC_GPIO.ADC_Conv_AB_pin,
			RESET);

	ADC_Delay_us(ADC,ADC->ADC_Time.Delay.convAB_Delay);

	HAL_GPIO_WritePin(ADC->ADC_GPIO.ADC_Conv_AB_Port,
			ADC->ADC_GPIO.ADC_Conv_AB_pin,
			SET);
}

void ADS8588H_POLL_BUSY(ADS8588H_Interface_t *ADC)
{
	/*
	 * Wait until all 3 ADCs are ready
	 */
	while(HAL_GPIO_ReadPin(ADC->ADC_GPIO.ADC_BUSY_1_Port, ADC->ADC_GPIO.ADC_BUSY_1_pin));// && HAL_GPIO_ReadPin(ADC->ADC_GPIO.ADC_BUSY_2_Port, ADC->ADC_GPIO.ADC_BUSY_2_pin) && HAL_GPIO_ReadPin(ADC->ADC_GPIO.ADC_BUSY_3_Port, ADC->ADC_GPIO.ADC_BUSY_3_pin));
	//ADC_Delay_us(ADC,500);
}



void ADC_Delay_us(ADS8588H_Interface_t *ADC, uint16_t us)
{
	/*
	 * Reset timer counter
	 */
	__HAL_TIM_SET_COUNTER(ADC->ADC_Time.ADC_htim, WIPE_COUNTER);
	/*
	 * Wait till time expires
	 */
	while ((uint16_t)__HAL_TIM_GET_COUNTER(ADC->ADC_Time.ADC_htim) < us);
}



