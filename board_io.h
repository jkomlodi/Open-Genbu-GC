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
    board_io *io_map;
    size_t size;
} io_map_container;

void *board_io_usb_prewrite(void *args);
void board_io_init(void);
