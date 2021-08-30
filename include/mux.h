#ifndef __MUX_DRIVER_H__
#define __MUX_DRIVER_H__

#define A0_TC_1_4 8
#define A1_TC_1_4 9
#define A0_TC_5_8 10
#define A1_TC_5_8 11

#define DIGITAL_MUX_IN 4
#define MUX_S0 5
#define MUX_S1 6
#define MUX_S2 7

enum Digital_Inputs {
  DIN_1 = 0,
  DIN_2,
  DIN_3,
  DIN_4,
  DIN_5,
  DIN_6,
  DIN_7,
  DIN_8
};

#include "main.h"

void ADC_MUX(ADC_Muxed_Data_t *ADC);
void digital_mux(Digital_Muxed_Data_t *Digital);
#endif