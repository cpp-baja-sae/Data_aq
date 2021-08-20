#include <Arduino.h>
#include "main.h"

/**
 * @brief high precision wait loop. Fully blocking functionality
 * 
 * @note When using with GPIO registers for NS control. Timing will slightly off due to time to
 * control GPIO register/function safety headroom.
 * 
 * @param[in] duration Sets the wait time in multiples of 5NS
 */ 
void delay_5ns(uint32_t duration)
{
    for(uint32_t time_step = 0; time_step < duration; time_step++)
    {
        DELAY_5NS;
    }
}