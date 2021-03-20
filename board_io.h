#ifndef BOARD_IO_H
#define BOARD_IO_H

#include <stdbool.h>
#include <stdint.h>

#define STATE_BUTTON_PRESSED 0
#define STATE_BUTTON_RELEASED 1

typedef struct {
    uint8_t gpio;
    bool state;
    bool latched;
} board_io;

typedef struct {
    /*
     * TODO: If we're clever about this, we can simplify some GPIO iterating
     * code by combining the button and d-pad maps and being able to iterate
     * over that combined map, and access the separate ones when needed.
     */
    board_io *btn_map;
    size_t btn_size;
    board_io *dpad_map;
    size_t dpad_size;
} io_map_container;

void *board_io_usb_prewrite(void *args);
void board_io_init(void);

#endif
