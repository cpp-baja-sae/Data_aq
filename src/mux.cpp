#include "mux.h"
#include "ADS8588H.h"
#include "main.h"
#include <Arduino.h>

void ADC_MUX(ADC_Muxed_Data_t *ADC) {
  switch (ADC->mux_channel_0) {
  case TC_1:
    digitalWrite(A0_TC_1_4, LOW);
    digitalWrite(A1_TC_1_4, HIGH);
    ADC->mux_channel_0++;
    break;
  case TC_2:
    digitalWrite(A0_TC_1_4, HIGH);
    digitalWrite(A1_TC_1_4, LOW);
    ADC->mux_channel_0++;
    break;
  case TC_3:
    digitalWrite(A0_TC_1_4, HIGH);
    digitalWrite(A1_TC_1_4, HIGH);
    ADC->mux_channel_0++;
    break;
  case TC_4:
    digitalWrite(A0_TC_1_4, LOW);
    digitalWrite(A1_TC_1_4, LOW);
    ADC->mux_channel_0 = 0;
    break;
  }
  switch (ADC->mux_channel_1) {
  case TC_1:
    digitalWrite(A0_TC_5_8, LOW);
    digitalWrite(A1_TC_5_8, HIGH);
    ADC->mux_channel_1++;
    break;
  case TC_2:
    digitalWrite(A0_TC_5_8, HIGH);
    digitalWrite(A1_TC_5_8, LOW);
    ADC->mux_channel_1++;
    break;
  case TC_3:
    digitalWrite(A0_TC_5_8, HIGH);
    digitalWrite(A1_TC_5_8, HIGH);
    ADC->mux_channel_1++;
    break;
  case TC_4:
    digitalWrite(A0_TC_5_8, LOW);
    digitalWrite(A1_TC_5_8, LOW);
    ADC->mux_channel_1 = 0;
    break;
  }
}

void digital_mux(Digital_Muxed_Data_t *Digital) {
  switch (Digital->mux_channel) {
  case DIN_1:
    digitalWrite(MUX_S0, LOW);
    digitalWrite(MUX_S1, LOW);
    digitalWrite(MUX_S2, LOW);
    Digital->mux_channel++;
    break;
  case DIN_2:
    digitalWrite(MUX_S0, HIGH);
    digitalWrite(MUX_S1, LOW);
    digitalWrite(MUX_S2, LOW);
    Digital->mux_channel++;
    break;
  case DIN_3:
    digitalWrite(MUX_S0, LOW);
    digitalWrite(MUX_S1, HIGH);
    digitalWrite(MUX_S2, LOW);
    Digital->mux_channel++;
    break;
  case DIN_4:
    digitalWrite(MUX_S0, HIGH);
    digitalWrite(MUX_S1, HIGH);
    digitalWrite(MUX_S2, LOW);
    Digital->mux_channel++;
    break;
  case DIN_5:
    digitalWrite(MUX_S0, LOW);
    digitalWrite(MUX_S1, LOW);
    digitalWrite(MUX_S2, HIGH);
    Digital->mux_channel++;
    break;
  case DIN_6:
    digitalWrite(MUX_S0, HIGH);
    digitalWrite(MUX_S1, LOW);
    digitalWrite(MUX_S2, HIGH);
    Digital->mux_channel++;
    break;
  case DIN_7:
    digitalWrite(MUX_S0, LOW);
    digitalWrite(MUX_S1, HIGH);
    digitalWrite(MUX_S2, HIGH);
    Digital->mux_channel++;
    break;
  case DIN_8:
    digitalWrite(MUX_S0, HIGH);
    digitalWrite(MUX_S1, HIGH);
    digitalWrite(MUX_S2, HIGH);
    Digital->mux_channel = 0;
    break;
  }
}