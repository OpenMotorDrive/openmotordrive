#include "timing.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>

static uint32_t counts_per_us;
static uint32_t counts_per_ms;
static volatile uint32_t system_millis;

void sys_tick_handler(void);

void timing_init(void)
{
    counts_per_ms = rcc_ahb_frequency/1000UL;
    counts_per_us = rcc_ahb_frequency/1000000UL;
    systick_set_reload(counts_per_ms); // 1 ms
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    systick_interrupt_enable();
}

uint32_t millis(void) {
    return system_millis;
}

uint32_t micros(void) {
    uint32_t ms;
    uint32_t counter;

    do {
        ms = system_millis;
        counter = systick_get_value();
    } while(system_millis != ms);

    return ms*1000UL + (counts_per_ms-counter)/counts_per_us;
}


void sys_tick_handler(void)
{
    system_millis++;
}
