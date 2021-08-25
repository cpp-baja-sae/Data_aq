#include "main.h"
#include <Arduino.h>

/**
 * @brief high precision wait loop. Fully blocking functionality
 *
 * @note When using with GPIO registers for NS control. Timing will slightly off
 * due to time to control GPIO register/function safety headroom.
 *
 * @param[in] duration Sets the wait time in multiples of 5NS
 */
void delay_5ns(uint32_t duration) {
  for (uint32_t time_step = 0; time_step < duration; time_step++) {
    DELAY_5NS;
  }
}

/**
 * @brief Function to service the heart beat indicitors onboard that signify
 * that the MCU is still alive.
 */
void dumb_scheduler::Heart_Beat(dumb_scheduler &heart_beating) {
  if (heart_beating.LED_STATUS == true) {
    digitalWrite(CORE_LED0_PIN, HIGH);
    heart_beating.LED_STATUS = 0;
  } else {
    digitalWrite(CORE_LED0_PIN, LOW);
    heart_beating.LED_STATUS = 1;
  }
}