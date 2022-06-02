/*
 * HelperCommands.c
 *
 *  Created on: Jun 2, 2022
 *      Author: forre
 */
#include "main.h"
#include "stm32h723xx.h"
/**
 * @brief Wait time tick is based off of what ever Timer14 is
 * configured for.
 */
void TIM14WaitUntil(uint16_t WaitLength){
	while((TIM14->CNT) <= WaitLength);
	TIM14->CNT = RESET;
}
