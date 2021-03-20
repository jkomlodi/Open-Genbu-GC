/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef DEV_LOWLEVEL_H_
#define DEV_LOWLEVEL_H_

#include "usb_common.h"
#include "utils.h"
#include "proc_queue.h"

__prio_queue void *usb_gamepad_format_and_send(void *arg);
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

void usb_device_init(void);

#endif
