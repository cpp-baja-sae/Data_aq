#include <Arduino.h>
#include "ADS8588H.h"
#include "main.h"
#include "DMAChannel.h"

int timer1 = 0;

int flip2 = 1;
int where = 0;
global_t ADC;
void ADC_SERVICE_ROUTINE()
{
    //Begin conversion
    ADS8588H_CONV_AB();
    /** TODO, Should not need this, 
     * Board needs a interrupt tied to falling edge of BUSY(?)
     * Wait out BUSY and OSR
     */
    delay_5ns(WAIT_OUT_CONVERSION_TIME_600us);
    //Lower CS to enable data conversion
    digitalWrite(ADC_CS_PIN,LOW);
    ADS8588H_READ_8CH();

    
}

void ADS8588H_READ_8CH()
{
    volatile int16_t data = 0;
    for(int channel =0; channel < 8; channel++)
    {
        for(int stepping = 15; stepping >= 0; stepping--)
        {
            int16_t temp = 0;
            digitalWrite(ADC_SCLK_PIN,LOW);
            temp = (digitalReadFast(ADC_1_DOUTA) << stepping);
            digitalWrite(ADC_SCLK_PIN,LOW);
            data |= temp;
        }
        ADC.channel_data[channel] = data;
        Serial.print(data,HEX);
        Serial.print("\n");
        data = 0;
    }
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
    work->huh = ADC.huh;
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
