#include "ADS8588H.h"
#include "DMAChannel.h"
#include "main.h"
#include "mux.h"
#include <Arduino.h>

ADC_Regular_Data_t ADC_Strain_Guage_1 = {0};
ADC_Regular_Data_t ADC_Strain_Guage_2 = {0};
ADC_Regular_Data_t ADC_General_Purpose_1 = {0};
ADC_Muxed_Data_t ADC_General_Purpose_2 = {0};
void ADC_SERVICE_ROUTINE() {
  // Begin conversion
  ADS8588H_CONV_AB();
  /** TODO, Should not need this,
   * Board needs a interrupt tied to falling edge of BUSY(?)
   * Wait out BUSY and OSR
   */
  while (digitalRead(ADC_BUSY))
    ;
  // delay_5ns(WAIT_OUT_CONVERSION_TIME_200us);

  ADS8588H_READ_8CH();
  return;
}

void ADS8588H_READ_8CH() {
  // Lower CS to enable data conversion
  digitalWrite(ADC_CS_PIN, LOW);

  for (int channel = 0; channel < QTY_DIFF_CHANNELS; channel++) {
    for (int stepping = CLK_CYCLES; stepping >= 0; stepping--) {
      digitalWrite(ADC_SCLK_PIN, LOW);
      ADC_Strain_Guage_1.bit_collector |=
          (digitalReadFast(SG_ADC_1_DOUTA) << stepping);
      ADC_Strain_Guage_2.bit_collector |=
          (digitalReadFast(SG_ADC_2_DOUTA) << stepping);
      ADC_General_Purpose_1.bit_collector |=
          (digitalReadFast(G_ADC_1_DOUTA) << stepping);
      ADC_General_Purpose_2.bit_collector |=
          (digitalReadFast(G_ADC_2_DOUTA) << stepping);
      digitalWrite(ADC_SCLK_PIN, HIGH);
    }
    convertData(channel, &ADC_Strain_Guage_1);
    convertData(channel, &ADC_Strain_Guage_2);
    convertData(channel, &ADC_General_Purpose_1);
    convertData_Muxed(channel, &ADC_General_Purpose_2);
  }

  print_data(&ADC_Strain_Guage_1);
  // print_mux_data(&ADC_General_Purpose_2);
  digitalWrite(ADC_CS_PIN, HIGH);
  if (ADC_General_Purpose_2.mux_delay >= MUX_1_SECOND) {
    ADC_MUX(&ADC_General_Purpose_2);
    ADC_General_Purpose_2.mux_delay = 0;
  }
  ADC_General_Purpose_2.mux_delay++;
}

void print_data(ADC_Regular_Data_t *ADC) {
  Serial.printf(
      "CH1: %f CH2: %f CH:3 %f CH4: %f CH5: %f CH:6 %f CH:7 %f CH:8 %f\n",
      ADC->scaled_data[DIFF_1], ADC->scaled_data[DIFF_2],
      ADC->scaled_data[DIFF_3], ADC->scaled_data[DIFF_4],
      ADC->scaled_data[DIFF_5], ADC->scaled_data[DIFF_6],
      ADC->scaled_data[DIFF_7], ADC->scaled_data[DIFF_8]);
  // Serial.printf("CH1: %hX CH2: %hX CH:3 %hX CH4: %hX CH5: %hX CH:6 %hX CH:7
  // %hX CH:8 %hX \n", ADC_1.raw_data[0], ADC_1.raw_data[1], ADC_1.raw_data[2],
  // ADC_1.raw_data[3], ADC_1.raw_data[4], ADC_1.raw_data[5], ADC_1.raw_data[6],
  // ADC_1.raw_data[7]);
}
void print_mux_data(ADC_Muxed_Data_t *ADC) {
  Serial.printf(
      "CH1: %f CH2: %f CH:3 %f CH4: %f CH5: %f CH:6 %f CH:7 %f CH:8 %f CH9: %f "
      "CH10: %f CH:11 %f CH12: %f CH13: %f CH:14 %f\n",
      ADC->scaled_data[TC_1], ADC->scaled_data[TC_2], ADC->scaled_data[TC_3],
      ADC->scaled_data[TC_4], ADC->scaled_data[TC_5], ADC->scaled_data[TC_6],
      ADC->scaled_data[TC_7], ADC->scaled_data[TC_8],
      ADC->scaled_data[SINGLE_ENDED_1], ADC->scaled_data[SINGLE_ENDED_2],
      ADC->scaled_data[SINGLE_ENDED_3], ADC->scaled_data[SINGLE_ENDED_4],
      ADC->scaled_data[SINGLE_ENDED_5], ADC->scaled_data[SINGLE_ENDED_6]);
  // Serial.printf("CH1: %hX CH2: %hX CH:3 %hX CH4: %hX CH5: %hX CH:6 %hX CH:7
  // %hX CH:8 %hX \n", ADC_1.raw_data[0], ADC_1.raw_data[1], ADC_1.raw_data[2],
  // ADC_1.raw_data[3], ADC_1.raw_data[4], ADC_1.raw_data[5], ADC_1.raw_data[6],
  // ADC_1.raw_data[7]);
}

void convertData(int channel, ADC_Regular_Data_t *ADC) {
  ADC->raw_data[channel] = ADC->bit_collector;
  // Translate raw value into float
  if (ADC->raw_data[channel] <= ADC_NEGATIVE_TRANSISTION_VALUE) {
    ADC->scaled_data[channel] =
        ADC_ABS_RANGE * ADC->raw_data[channel] / ADC_16BIT_CONVERSION_VALUE;
  } else {
    ADC->scaled_data[channel] =
        ADC_ABS_RANGE * ADC->raw_data[channel] / ADC_16BIT_CONVERSION_VALUE -
        ADC_ABS_RANGE;
  }
  ADC->bit_collector = 0;
}
void convertData_Muxed(int channel, ADC_Muxed_Data_t *ADC) {
  if (channel == TC_CHANNEL_0) {
    ADC->raw_data[ADC->mux_channel_0] = ADC->bit_collector;
    channel = ADC->mux_channel_0;
  } else if (channel == TC_CHANNEL_1) {
    ADC->raw_data[TC_CHANNEL_1_DATA_SHIFT + ADC->mux_channel_1] =
        ADC->bit_collector;
    channel = TC_CHANNEL_1_DATA_SHIFT + ADC->mux_channel_1;
  } else if (channel >= NOT_TC_CHANNEL) {
    ADC->raw_data[TC_CHANNEL_SHIFT + channel] = ADC->bit_collector;
    channel = TC_CHANNEL_SHIFT + channel;
  }

  // Translate raw value into float
  if (ADC->raw_data[channel] <= ADC_NEGATIVE_TRANSISTION_VALUE) {
    ADC->scaled_data[channel] =
        ADC_ABS_RANGE * ADC->raw_data[channel] / ADC_16BIT_CONVERSION_VALUE;
  } else {
    ADC->scaled_data[channel] =
        ADC_ABS_RANGE * ADC->raw_data[channel] / ADC_16BIT_CONVERSION_VALUE -
        ADC_ABS_RANGE;
  }
  ADC->bit_collector = 0;
}

void ADS8588H_CS() {
  delay_5ns(approx_500NS);
  digitalWrite(ADC_CS_PIN, LOW);
  delay_5ns(approx_500NS);
  digitalWrite(ADC_CS_PIN, HIGH);
}

void ADS8588H_CONV_AB() {
  digitalWrite(ADC_CONV_AB, LOW);
  delay_5ns(approx_500NS);
  digitalWrite(ADC_CONV_AB, HIGH);
}

void ADS8588H_reset() {
  delay_5ns(approx_500NS);
  digitalWrite(ADC_RESET, HIGH);
  delay_5ns(approx_500NS);
  digitalWrite(ADC_RESET, LOW);
}

int ADC_call_back(global_t *work) { return 0; }

void ADS8588H_init() {
  /*
   * From section 7.4.1.6 of data sheet
   * Needed for settling internal references.
   */
  ADS8588H_reset();

  // Start up periodic task to service the ADC

  // ADC_TIMER.begin(ADC_SERVICE_ROUTINE, Timer_interval_2khz);
  return;
}
