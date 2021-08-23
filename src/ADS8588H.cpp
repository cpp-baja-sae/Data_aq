#include <Arduino.h>
#include "ADS8588H.h"
#include "main.h"
#include "DMAChannel.h"

int timer1 = 0;

int flip2 = 1;
int where = 0;
ADC_Strain_Gauge_t ADC_1 = {0};
ADC_Strain_Gauge_t ADC_2 = {0};
void ADC_SERVICE_ROUTINE()
{
    //Begin conversion
    ADS8588H_CONV_AB();
    /** TODO, Should not need this, 
     * Board needs a interrupt tied to falling edge of BUSY(?)
     * Wait out BUSY and OSR
     */
    delay_5ns(WAIT_OUT_CONVERSION_TIME_600us);
    
    ADS8588H_READ_8CH();
}

void ADS8588H_READ_8CH()
{
    //Lower CS to enable data conversion
    digitalWrite(ADC_CS_PIN,LOW);
    
    for(int channel = 0; channel < QTY_DIFF_CHANNELS; channel++)
    {
        for(int stepping = CLK_CYCLES; stepping >= 0; stepping--)
        {
            digitalWrite(ADC_SCLK_PIN,LOW);
            ADC_1.bit_collector |= (digitalReadFast(SG_ADC_1_DOUTA) << stepping);
            ADC_2.bit_collector |= (digitalReadFast(SG_ADC_2_DOUTA) << stepping);
            digitalWrite(ADC_SCLK_PIN, HIGH);
        }
        convertData_Strain_Guage(channel, &ADC_1);
        convertData_Strain_Guage(channel, &ADC_2);
    }
    print_data(&ADC_2);
    digitalWrite(ADC_CS_PIN,HIGH);
}

void print_data(ADC_Strain_Gauge_t *ADC)
{
    Serial.printf("CH1: %f CH2: %f CH:3 %f CH4: %f CH5: %f CH:6 %f CH:7 %f CH:8 %f\n",
    ADC->scaled_data[DIFF_1], ADC->scaled_data[DIFF_2], ADC->scaled_data[DIFF_3], ADC->scaled_data[DIFF_4], ADC->scaled_data[DIFF_5], ADC->scaled_data[DIFF_6], ADC->scaled_data[DIFF_7], ADC->scaled_data[DIFF_8]);
    // Serial.printf("CH1: %hX CH2: %hX CH:3 %hX CH4: %hX CH5: %hX CH:6 %hX CH:7 %hX CH:8 %hX \n",
    // ADC_1.raw_data[0], ADC_1.raw_data[1], ADC_1.raw_data[2], ADC_1.raw_data[3], ADC_1.raw_data[4], ADC_1.raw_data[5], ADC_1.raw_data[6], ADC_1.raw_data[7]);

}

void convertData_Strain_Guage(int channel, ADC_Strain_Gauge_t *ADC)
{
    ADC->raw_data[channel] = ADC->bit_collector;
    // Translate raw value into float
    if(ADC->raw_data[channel] <= ADC_NEGATIVE_TRANSISTION_VALUE){
        ADC->scaled_data[channel] = ADC_ABS_RANGE*ADC->raw_data[channel]/ADC_16BIT_CONVERSION_VALUE; 
    }else{
        ADC->scaled_data[channel] = ADC_ABS_RANGE*ADC->raw_data[channel]/ADC_16BIT_CONVERSION_VALUE - ADC_ABS_RANGE;
    }
    ADC->bit_collector = 0;
}

void ADS8588H_CS()
{
    delay_5ns(approx_500NS);
    digitalWrite(ADC_CS_PIN,LOW);
    delay_5ns(approx_500NS);
    digitalWrite(ADC_CS_PIN,HIGH);
}

void ADS8588H_CONV_AB()
{
    digitalWrite(ADC_CONV_AB,LOW);
    delay_5ns(approx_500NS);
    digitalWrite(ADC_CONV_AB,HIGH);
}

void ADS8588H_reset()
{
    delay_5ns(approx_500NS);
    digitalWrite(ADC_RESET,HIGH);
    delay_5ns(approx_500NS);
    digitalWrite(ADC_RESET,LOW);
}

int ADC_call_back(global_t *work){
    return 0;
}

void ADS8588H_init(IntervalTimer &ADC_TIMER)
{
    /*
    * From section 7.4.1.6 of data sheet
    * Needed for settling internal references.
    */
    ADS8588H_reset();

    // Start up periodic task to service the ADC

    ADC_TIMER.begin(ADC_SERVICE_ROUTINE, Timer_interval_2khz);

}
