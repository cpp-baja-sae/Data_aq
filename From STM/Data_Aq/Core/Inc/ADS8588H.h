/*
 * ADS8588H.h
 *
 *  Created on: Jul 15, 2022
 *      Author: forre
 */

#ifndef INC_ADS8588H_H_
#define INC_ADS8588H_H_

#include "main.h"
#include "ff.h"

/*
 * Derived from Table 1 From data sheet
 */
#define OSR_NONE	0b000
#define OSR_2		0b001
#define OSR_4		0b010
#define	OSR_8		0b011
#define OSR_16		0b100
#define	OSR_32		0b101
#define	OSR_64		0b110

#define	OSR_BIT_0	0b001
#define OSR_BIT_1	0b010
#define	OSR_BIT_2	0b100

#define	OSR_BIT_1_MASK	1
#define	OSR_BIT_2_MASK	2

#define DEFAULT_CONVAB_DELAY	1
#define DEFAULT_RESET_DELAY	1
#define DEFAULT_CS_DELAY	1

#define WIPE_COUNTER	0



typedef struct
{
	GPIO_TypeDef *ADC_Reset_Port;
	uint16_t	ADC_Reset_pin;
	GPIO_TypeDef *ADC_Conv_AB_Port;
	uint16_t	ADC_Conv_AB_pin;
	GPIO_TypeDef *ADC_CS_Port;
	uint16_t	ADC_CS_pin;
	GPIO_TypeDef *ADC_BUSY_1_Port;
	uint16_t	ADC_BUSY_1_pin;
	GPIO_TypeDef *ADC_BUSY_2_Port;
	uint16_t	ADC_BUSY_2_pin;
	GPIO_TypeDef *ADC_BUSY_3_Port;
	uint16_t	ADC_BUSY_3_pin;
	GPIO_TypeDef *ADC_OS0_Port;
	uint16_t	ADC_OS0_pin;
	GPIO_TypeDef *ADC_OS1_Port;
	uint16_t	ADC_OS1_pin;
	GPIO_TypeDef *ADC_OS2_Port;
	uint16_t	ADC_OS2_pin;
} ADS8588H_HAL_GPIO_t;

typedef struct
{
	uint8_t OSR;
	GPIO_PinState	OS0;
	GPIO_PinState	OS1;
	GPIO_PinState	OS2;
} ADS8588H_OSR_t;

typedef struct
{
	uint16_t convAB_Delay;
	uint16_t reset_Delay;
} ADS8588H_TIMER_DELAYS_t;

typedef struct
{
	ADS8588H_TIMER_DELAYS_t Delay;
	TIM_HandleTypeDef *ADC_htim;
} ADS8588H_HAL_TIME_t;

typedef struct
{
	OSPI_HandleTypeDef	*hopsi;
	FRESULT	res;
} ADS8588H_OSPI_HANDLER_t;

typedef struct
{
	uint8_t	raw_data[64];
} ADS8588H_ADC_DATA_t;

typedef struct
{
	ADS8588H_HAL_TIME_t ADC_Time;
	ADS8588H_HAL_GPIO_t ADC_GPIO;
	ADS8588H_OSR_t	ADC_OSR;
	ADS8588H_ADC_DATA_t	DATA;
	ADS8588H_OSPI_HANDLER_t	OPSI;
} ADS8588H_Interface_t;

void ADS8588H_Init(ADS8588H_Interface_t *ADC);
void ADC_SERVICE_ROUTINE();
void ADS8588H_Reset(ADS8588H_Interface_t *ADC);
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
		);
void ADC_Delay_us(ADS8588H_Interface_t *ADC, uint16_t us);
void ADS8588H_Time_Delay_Base(ADS8588H_Interface_t *ADC);
void ADS8588H_READ_ALL(ADS8588H_Interface_t *ADC);
void ADS8588H_OSR_SETUP(ADS8588H_Interface_t *ADC);
void ADS8588H_CONV_AB(ADS8588H_Interface_t *ADC);
void ADS8588H_POLL_BUSY(ADS8588H_Interface_t *ADC);
#endif /* INC_ADS8588H_H_ */
