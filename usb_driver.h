#ifndef USB_DRIVER_H
#define USB_DRIVER_H

#include <stdint.h>
#include <stdlib.h>

#include "usb_common.h"

#include "descriptors.h"
#include "usb_gamepad.h" /* XXX: Needed for ep1_cb, instead we should
                            make a function that takes it in as an argument
                            and we add it so we don't need to include the
                            gamepad header
                           */
#define usb_hw_set hw_set_alias(usb_hw)
#define usb_hw_clear hw_clear_alias(usb_hw)

typedef void (*usb_ep_handler)(uint8_t *buf, uint16_t len);

/* Struct in which we keep the endpoint configuration */
struct usb_endpoint_configuration {
    const struct usb_endpoint_descriptor *descriptor;
    usb_ep_handler handler;

    /*
     * Pointers to endpoint + buffer control registers
     * in the USB controller DPSRAM
     */
    volatile uint32_t *endpoint_control;
    volatile uint32_t *buffer_control;
    volatile uint8_t *data_buffer;

    /* Toggle after each packet (unless replying to a SETUP) */
    uint8_t next_pid;
};

/* Struct in which we keep the device configuration */
struct usb_device_configuration {
    const struct usb_device_descriptor *device_descriptor;
    const struct usb_interface_descriptor *interface_descriptor;
    const struct usb_configuration_descriptor *config_descriptor;
    const struct usb_hid_descriptor *hid_descriptor;
    const unsigned char *lang_descriptor;
    const unsigned char **descriptor_strings;
    struct usb_endpoint_configuration endpoints[USB_NUM_ENDPOINTS];
};

/*
 * TODO: Integrate this state machine with other transfers, not just HID
 * report.
 * The Raspberry Pi example assumes all of their transfers complet in 64 bytes.
 */
typedef enum {
    XFER_STATE_DONE = 0x00,
    XFER_STATE_DEVICE_DESCR = 0x01,
    XFER_STATE_INTERFACE_DESCR = 0x02,
    XFER_STATE_CONFIG_DESCR = 0x03,
    XFER_STATE_ENDPOINT_DESCR = 0x04,
    XFER_STATE_HID_REPORT = 0x05
} USBXferState;

/* Callbacks for when the EP finishes transferring */
void ep0_in_cb(uint8_t *buf, uint16_t len);
void ep0_out_cb(uint8_t *buf, uint16_t len);

void usb_device_init(void);
void usb_start_transfer(struct usb_endpoint_configuration *ep, uint8_t *buf,
                        uint16_t len);
struct usb_endpoint_configuration *usb_get_endpoint_configuration(uint8_t addr);
void usb_bus_reset(void);
void ep0_in_cb(uint8_t *buf, uint16_t len);
void ep0_out_cb(uint8_t *buf, uint16_t len);

#endif
