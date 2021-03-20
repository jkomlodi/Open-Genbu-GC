#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/structs/iobank0.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

#include "proc_queue.h"
#include "pinmap.h"
#include "utils.h"
#include "arm_utils.h"
#include "io.h"
#include "usb_gamepad.h"
#include "board_io.h"
#include "debounce.h"

board_io btn_map[NUM_BTN] = {
    { .gpio = BOOSTER_R_U },
    { .gpio = BOOSTER_R_R },
    { .gpio = BOOSTER_R_D },
    { .gpio = BOOSTER_R_L },
    { .gpio = BOOSTER_R_PRESS },
    { .gpio = BOOSTER_L_PRESS },
    { .gpio = BTN_START }
};

board_io dpad_map[NUM_DPAD] = {
    { .gpio = BOOSTER_L_U },
    { .gpio = BOOSTER_L_R },
    { .gpio = BOOSTER_L_D },
    { .gpio = BOOSTER_L_L }
};

/* When passing maps around, we need to be aware of their size as well */
io_map_container io_container = {
    .btn_map = btn_map,
    .btn_size = ARRAY_SIZE(btn_map),
    .dpad_map = dpad_map,
    .dpad_size = ARRAY_SIZE(dpad_map)
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

    __DISABLE_IRQ;

    for (i = 0; i < ARRAY_SIZE(btn_map); ++i) {
        new_state = gpio_get(btn_map[i].gpio);
        gpio_acknowledge_irq(btn_map[i].gpio,
                             GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
        if (btn_map[i].state != new_state) {
            btn_map[i].state = new_state;
            state_changed = true;
        }
    }

    for (i = 0; i < ARRAY_SIZE(dpad_map); ++i) {
        new_state = gpio_get(dpad_map[i].gpio);
        gpio_acknowledge_irq(dpad_map[i].gpio,
                             GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
        if (dpad_map[i].state != new_state) {
            dpad_map[i].state = new_state;
            state_changed = true;
        }
    }

    DB_PRINT_L(3, "state_changed=%d\n", state_changed);

    if (state_changed) {
        proc_enqueue(board_io_usb_prewrite, &io_container,
                     PRIORITY_LEVEL_HIGHEST);
    }

    __ENABLE_IRQ;
}

static void io_map_init(void)
{
    size_t i;

    for (i = 0; i < ARRAY_SIZE(btn_map); ++i) {
        gpio_set_input_enabled(btn_map[i].gpio, true);
        gpio_set_pulls(btn_map[i].gpio, true, false);
        btn_map[i].state = gpio_get(btn_map[i].gpio);
    }

    for (i = 0; i < ARRAY_SIZE(dpad_map); ++i) {
        gpio_set_input_enabled(dpad_map[i].gpio, true);
        gpio_set_pulls(dpad_map[i].gpio, true, false);
        dpad_map[i].state = gpio_get(dpad_map[i].gpio);
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

    for (i = 0; i < io_container.btn_size; ++i) {
        gpio_set_irq_enabled(io_container.btn_map[i].gpio, events, true);
    }

    for (i = 0; i < io_container.dpad_size; ++i) {
        gpio_set_irq_enabled(io_container.dpad_map[i].gpio, events, true);
    }
    irq_set_exclusive_handler(IO_IRQ_BANK0, board_io_irq_handler);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void board_io_init(void)
{
    io_map_init();
    isr_init();
}
