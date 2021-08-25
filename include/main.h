#ifndef __MAIN_H__
#define __MAIN_H__
#include "unistd.h"
#include <Arduino.h>

#define heart_beat_compare_time_ms 1000
// Pauses execution of the program for a single clock cycle
#define DELAY_CLOCK_CYCLE __asm__("nop\n")
// Pauses execution of the program for 3 clock cycles. Our clock runs at 600MHz,
// so this results in a 5ns delay.
#define DELAY_5NS                                                              \
  DELAY_CLOCK_CYCLE;                                                           \
  DELAY_CLOCK_CYCLE;                                                           \
  DELAY_CLOCK_CYCLE

class global_t {
public:
  int hey;
};

class ADC_Regular_Data_t {
public:
  volatile uint16_t bit_collector;
  int16_t raw_data[8];
  float scaled_data[8];
};

class ADC_Muxed_Data_t {
public:
  volatile uint16_t bit_collector;
  int16_t raw_data[14];
  float scaled_data[14];
  uint8_t mux_channel_0;
  uint8_t mux_channel_1;
  uint16_t mux_delay;
};

class dumb_scheduler {
public:
  uint64_t heart_beat;
  uint8_t LED_STATUS;

  void Heart_Beat(dumb_scheduler &heart_beating);
};

void delay_5ns(uint32_t duration);
void setup_pins();
#endif