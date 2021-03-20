#include <stdlib.h>

#include "pico/stdlib.h"

#include "proc_queue.h"
#include "usb_driver.h"
#include "utils.h"
#include "arm_utils.h"
#include "board_io.h"

void control_loop(void)
{
    proc_info *proc;

    while (1) {
        proc = proc_next();
        if (proc) {
            proc->proc_fn(proc->pfn_args);
        } else {
            __DSB;
            __WFI;
        }
    }
}

int main(void) {
    stdio_init_all();
    usb_device_init();
    proc_init();
    board_io_init();

    /* Spin until USB is ready */
    while(!usb_is_configured());

    control_loop();

    return 0;
}