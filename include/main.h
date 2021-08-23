#ifndef __MAIN_H__
#define __MAIN_H__

// Pauses execution of the program for a single clock cycle
#define DELAY_CLOCK_CYCLE __asm__("nop\n")
// Pauses execution of the program for 3 clock cycles. Our clock runs at 600MHz,
// so this results in a 5ns delay.
#define DELAY_5NS                                                              \
    DELAY_CLOCK_CYCLE;                                                         \
    DELAY_CLOCK_CYCLE;                                                         \
    DELAY_CLOCK_CYCLE

class global_t{
    public:
    int hey;
};

class ADC_Strain_Gauge_t{
    public:
    volatile uint16_t bit_collector;
    int16_t raw_data[8];
    float scaled_data[8];
};

void delay_5ns(uint32_t duration);
#endif