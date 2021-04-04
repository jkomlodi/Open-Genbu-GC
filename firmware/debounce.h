#ifndef DEBOUNCE_H
#define DEBOUNCE_H

#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "board_io.h"

#define DEBOUNCE_TIME_MS 10
// typedef int64_t (*pfn_debounce_t)(uint8_t);

/*
 * This handles what we should do right away to stop bouncing.
 * For now it disables the correspoinding GPIO interrupt
 */
static inline void debounce_start(uint8_t gpio)
{
    gpio_set_irq_enabled(gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
}

int64_t debounce_timer_elapsed(alarm_id_t id, void *arg);
void debounce_timer_start(uint32_t time, alarm_callback_t cb, void *cb_arg);

#endif
