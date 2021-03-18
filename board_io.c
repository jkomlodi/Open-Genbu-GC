#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/structs/iobank0.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

#include "prio_queue.h"
#include "pinmap.h"
#include "utils.h"
#include "io.h"
#include "dev_lowlevel.h"
#include "board_io.h"

board_io io_map[NUM_PINS] = {
    { .gpio = BOOSTER_R_U },
    { .gpio = BOOSTER_R_R },
    { .gpio = BOOSTER_R_D },
    { .gpio = BOOSTER_R_L },
    { .gpio = BOOSTER_R_PRESS },
    { .gpio = BOOSTER_L_U },
    { .gpio = BOOSTER_L_R },
    { .gpio = BOOSTER_L_D },
    { .gpio = BOOSTER_L_L },
    { .gpio = BOOSTER_L_PRESS },
    { .gpio = BTN_START }
};

/* When passing io_map around, we need to be aware of its size as well */
io_map_container io_container = {
    .io_map = io_map,
    .size = ARRAY_SIZE(io_map)
};

/*
 * Anything to be done before we send the data should be handled here, such as
 * starting a timer for each button that was pressed for debounce handling.
 * TODO: It might be wise to disable interrupts on the GPIOs that will be
 * bouncing.  Then when the timer elapses, that callback re-enables the
 * interrupt on the bouncing GPIO.
 */
__prio_queue void *board_io_usb_prewrite(void *args)
{
    DB_PRINT_L(3, "\n");

    io_map_container *ioc = (io_map_container *)args;
    proc_enqueue(usb_gamepad_format_and_send, ioc,
                 PRIORITY_LEVEL_HIGHEST);
}

__irq_handler void board_io_irq_handler(void)
{
    size_t i;
    bool state_changed = false;
    bool new_state;

    for (i = 0; i < ARRAY_SIZE(io_map); ++i) {
        new_state = gpio_get(io_map[i].gpio);
        if (io_map[i].state != new_state) {
            io_map[i].state = new_state;
            state_changed = true;
        }
    }

    DB_PRINT_L(3, "state_changed=%d\n", state_changed);

    if (state_changed) {
        proc_enqueue(board_io_usb_prewrite, &io_container,
                     PRIORITY_LEVEL_HIGHEST);
    }
}

static void io_map_init(void)
{
    size_t i;

    for (i = 0; i < ARRAY_SIZE(io_map); ++i) {
        gpio_set_input_enabled(io_map[i].gpio, true);
        io_map[i].state = gpio_get(io_map[i].gpio);
    }
}

static void isr_init(void)
{
    uint32_t ien;
    size_t i;
    uint32_t events = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;

    io_irq_ctrl_hw_t *irq_base = get_core_num() ?
                                 &iobank0_hw->proc1_irq_ctrl :
                                 &iobank0_hw->proc0_irq_ctrl;

    for (i = 0; i < io_container.size; ++i) {
        gpio_acknowledge_irq(io_container.io_map[i].gpio, events);
        ien = irq_base->inte[io_container.io_map[i].gpio / 8];
        events <<= 4 * (io_container.io_map[i].gpio % 8);

        reg_set_bit(ien, events);
    }
    irq_set_exclusive_handler(IO_IRQ_BANK0, board_io_irq_handler);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void board_io_init(void)
{
    io_map_init();
    isr_init();
}
