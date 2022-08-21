/*
 * ADS8588H.c
 *
 *  Created on: Jul 15, 2022
 *      Author: forre
 */
#include "ADS8588H.h"
#include "main.h"

/*
 * Need to figure a way to auto find the pin location during the preprocessor,
 * value has to be updated manual for code changes
 */
#define READ_PIN_msk	12
#define FASTREAD(Port,PIN)	((Port->IDR & PIN)>>READ_PIN_msk)

#define CONVERSION_CODE	0xFFFF
#define PIN_NUM	(16U)

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
		uint8_t OSR
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

	ADC_Delay_ns(ADC,ADC->ADC_Time.Delay.reset_Delay);

	HAL_GPIO_WritePin(ADC->ADC_GPIO.ADC_Reset_Port,
			ADC->ADC_GPIO.ADC_Reset_pin,
			RESET);
}

void convert_data(ADS8588H_Interface_t *ADC)
{

	for(int x = 0; x < 8; x++)
	{
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
	int i;
	/*
	 * Select ADC, enable low
	 */
	HAL_GPIO_WritePin(ADC->ADC_GPIO.ADC_CS_Port,
			ADC->ADC_GPIO.ADC_CS_pin,
			RESET);

	/*
	 * Read ADC
	 */
	for(int x = 0; x<8; x++)
	{
		i = 15;
		ADC->DATA.raw_data_16[x] = 0;

		while(i >= 0)
		{
			/* RESET pin state, low level implementation */
			ADC_CLK_GPIO_Port->BSRR = (uint32_t)ADC_CLK_Pin << PIN_NUM;
			/* Read pin state and collect it, forming the message. */
			ADC->DATA.raw_data_16[x] |= FASTREAD(ADC_CH_A_GPIO_Port, ADC_CH_A_Pin) << i;
			/* SET pin state, low level implementation */
			ADC_CLK_GPIO_Port->BSRR = ADC_CLK_Pin;

			i--;
		}
	}

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
}


/* @brief Timer is based off of 275MHz clock for 3.64 nS ticks
 * 	Time to enter and setup function takes a lot of time so lower values
 * 	will not be accurate.
 */
void ADC_Delay_ns(ADS8588H_Interface_t *ADC, uint32_t ns_tick)
{
	/*
	 * Reset timer counter
	 */
	__HAL_TIM_SET_COUNTER(ADC->ADC_Time.ADC_htim, WIPE_COUNTER);
	/*
	 * Wait till time expires
	 */
	while (__HAL_TIM_GET_COUNTER(ADC->ADC_Time.ADC_htim) < ns_tick);
}



