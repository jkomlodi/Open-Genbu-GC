#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "usb_common.h"
#include "hardware/regs/usb.h"
#include "hardware/structs/usb.h"
#include "hardware/irq.h"
#include "hardware/resets.h"
#include "dev_lowlevel.h"

#include "io.h"
#include "utils.h"
#include "arm_utils.h"
#include "prio_queue.h"
#include "board_io.h"
#include "pinmap.h"

#define usb_hw_set hw_set_alias(usb_hw)
#define usb_hw_clear hw_clear_alias(usb_hw)

typedef struct {
    uint8_t indeces[4];
    uint8_t bits[4];
    size_t num;
} gamepad_btn;

typedef struct {
    uint8_t *buf;
    size_t size;
} buf_t;

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

static USBXferState usb_xfer_state = XFER_STATE_DONE;

/* XXX: HID report variables */
size_t descr_sent;
size_t descr_expected;
/* Different from configured in USB terminology */
static bool hid_ready = false;

/* Callbacks for when the EP finishes transferring */
void ep0_in_cb(uint8_t *buf, uint16_t len);
void ep0_out_cb(uint8_t *buf, uint16_t len);
void ep1_in_cb(uint8_t *buf, uint16_t len);
void ep2_out_cb(uint8_t *buf, uint16_t len);

/* USB Globals */
static bool should_set_address = false;
static uint8_t dev_addr = 0;
static volatile bool configured = false;
static uint8_t ep0_buf[64];

/* Global needed so EP1 CB knows if it needs to re-send packets */
bool gamepad_held;

/* TODO: Write a test that ensures this map matches the io_map mapping */
/* Stolen from Genbu */
const gamepad_btn io_btn_map[NUM_BTN] = {
    { .indeces = {0},     .bits = {0x01}, .num = 1 }, /* RU */
    { .indeces = {0},     .bits = {0x02}, .num = 1 }, /* RR */
    { .indeces = {0},     .bits = {0x04}, .num = 1 }, /* RD */
    { .indeces = {0},     .bits = {0x08}, .num = 1 }, /* RL */
    { .indeces = {0},     .bits = {0x20}, .num = 1 }, /* RPRESS */
    { .indeces = {0, 15}, .bits = {0x10, 0xff}, .num = 2 }, /* LPRESS */
    { .indeces = {1},     .bits = {0x01}, .num = 1 }  /* START */
};

const gamepad_btn io_dpad_map[NUM_DPAD] = {
    { .indeces = {2, 9},     .bits = {0x08, 0xff}, .num = 2 }, /* LU */
    { .indeces = {2, 7},     .bits = {0x02, 0xff}, .num = 2 }, /* LR */
    { .indeces = {2, 10},    .bits = {0x04, 0xff}, .num = 2 }, /* LD */
    { .indeces = {2, 8},     .bits = {0x06, 0xff}, .num = 2 } /* LL */
};

const uint8_t gamepad_template[] = {
                           0x00, 0x00, 0x08, 0x80, 0x80, 0x80, 0x80, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x02, 0x80, 0x01, 0x00,
                           0x02, 0x00, 0x02
                           };

static struct usb_device_configuration dev_config = {
        .device_descriptor = &device_descriptor,
        .interface_descriptor = &interface_descriptor,
        .config_descriptor = &config_descriptor,
        .hid_descriptor = &hid_descriptor,
        .lang_descriptor = lang_descriptor,
        .descriptor_strings = descriptor_strings,
        .endpoints = {
                {
                        .descriptor = &ep0_out,
                        .handler = &ep0_out_cb,
                        .endpoint_control = NULL, /* NA for EP0 */
                        .buffer_control = &usb_dpram->ep_buf_ctrl[0].out,
                        /* EP0 in and out share a data buffer */
                        .data_buffer = &usb_dpram->ep0_buf_a[0],
                },
                {
                        .descriptor = &ep0_in,
                        .handler = &ep0_in_cb,
                        .endpoint_control = NULL, /* NA for EP0 */
                        .buffer_control = &usb_dpram->ep_buf_ctrl[0].in,
                        /* EP0 in and out share a data buffer */
                        .data_buffer = &usb_dpram->ep0_buf_a[0],
                },
                {
                        .descriptor = &ep1_in,
                        .handler = &ep1_in_cb,
                        .endpoint_control = &usb_dpram->ep_ctrl[0].in,
                        .buffer_control = &usb_dpram->ep_buf_ctrl[1].in,
                        /* First free EPx buffer */
                        .data_buffer = &usb_dpram->epx_data[0 * 64],
                },
                {
                        .descriptor = &ep2_out,
                        .handler = &ep2_out_cb,
                        .endpoint_control = &usb_dpram->ep_ctrl[1].out,
                        .buffer_control = &usb_dpram->ep_buf_ctrl[2].out,
                        /* Second free EPx buffer */
                        .data_buffer = &usb_dpram->epx_data[1 * 64],
                }
        }
};

/**
 * @brief Given an endpoint address, return the usb_endpoint_configuration of
 * that endpoint. Returns NULL if an endpoint of that address is not found.
 *
 * @param addr
 * @return struct usb_endpoint_configuration
 */
struct usb_endpoint_configuration *usb_get_endpoint_configuration(uint8_t addr)
{
    struct usb_endpoint_configuration *endpoints = dev_config.endpoints;
    for (int i = 0; i < USB_NUM_ENDPOINTS; i++) {
        if (endpoints[i].descriptor &&
            (endpoints[i].descriptor->bEndpointAddress == addr)) {
            return &endpoints[i];
        }
    }
    return NULL;
}

/**
 * @brief Given a C string, fill the EP0 data buf with a USB string descriptor
 * for that string.
 *
 * @param C string you would like to send to the USB host
 * @return the length of the string descriptor in EP0 buf
 */
/*
 * TODO: This can be done at compile time, I'm not sure why they decided to
 * do this at runtime.
 */
uint8_t usb_prepare_string_descriptor(const unsigned char *str)
{
    /*
     * (2 for bLength + bDescriptorType + strlen * 2) because host expects
     * UTF-16
     */
    uint8_t bLength = 2 + (strlen(str) * 2);
    static const uint8_t bDescriptorType = 0x03;

    volatile uint8_t *buf = &ep0_buf[0];
    *buf++ = bLength;
    *buf++ = bDescriptorType;

    uint8_t c;

    do {
        c = *str++;
        *buf++ = c;
        *buf++ = 0;
    } while (c != '\0');

    return bLength;
}

/**
 * @brief Take a buffer pointer located in the USB RAM and return as an offset
 * of the RAM.
 *
 * @param buf
 * @return uint32_t
 */
static inline uint32_t usb_buffer_offset(volatile uint8_t *buf)
{
    return (uint32_t) buf ^ (uint32_t) usb_dpram;
}

/**
 * @brief Set up the endpoint control register for an endpoint
 * (if applicable. Not valid for EP0).
 *
 * @param ep
 */
void usb_setup_endpoint(const struct usb_endpoint_configuration *ep)
{
    DB_PRINT_L(2, "Set up endpoint 0x%x with buffer address 0x%p\n",
           ep->descriptor->bEndpointAddress, ep->data_buffer);

    if (!ep->endpoint_control) {
        return;
    }

    /* Get the data buffer as an offset of the USB controller's DPRAM */
    uint32_t dpram_offset = usb_buffer_offset(ep->data_buffer);
    uint32_t reg = EP_CTRL_ENABLE_BITS
                   | EP_CTRL_INTERRUPT_PER_BUFFER
                   | (ep->descriptor->bmAttributes << EP_CTRL_BUFFER_TYPE_LSB)
                   | dpram_offset;
    *ep->endpoint_control = reg;
}

/**
 * @brief Set up the endpoint control register for each endpoint.
 */
void usb_setup_endpoints(void)
{
    const struct usb_endpoint_configuration *endpoints = dev_config.endpoints;
    for (int i = 0; i < USB_NUM_ENDPOINTS; i++) {
        if (endpoints[i].descriptor && endpoints[i].handler) {
            usb_setup_endpoint(&endpoints[i]);
        }
    }
}

/**
 * @brief Set up the USB controller in device mode, clearing any previous state.
 *
 */
void usb_device_init(void)
{
    /* Reset usb controller */
    reset_block(RESETS_RESET_USBCTRL_BITS);
    unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

    /* Clear any previous state in dpram just in case */
    memset(usb_dpram, 0, sizeof(*usb_dpram));

    /* Enable USB interrupt at processor */
    irq_set_enabled(USBCTRL_IRQ, true);

    /* Mux the controller to the onboard usb phy */
    usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;

    /* Force VBUS detect so the device thinks it is plugged into a host */
    usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_BITS |
                  USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;

    /* Enable the USB controller in device mode. */
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;

    /* Enable an interrupt per EP0 transaction */
    usb_hw->sie_ctrl = USB_SIE_CTRL_EP0_INT_1BUF_BITS;

    /*
     * Enable interrupts for when a buffer is done, when the bus is reset,
     * and when a setup packet is received
     */
    usb_hw->inte = USB_INTS_BUFF_STATUS_BITS |
                   USB_INTS_BUS_RESET_BITS |
                   USB_INTS_SETUP_REQ_BITS;

    /*
     * Set up endpoints (endpoint control registers)
     * described by device configuration
     */
    usb_setup_endpoints();

    /* Present full speed device by enabling pull up on DP */
    usb_hw_set->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;
}

/**
 * @brief Given an endpoint configuration, returns true if the endpoint
 * is transmitting data to the host (i.e. is an IN endpoint)
 *
 * @param ep, the endpoint configuration
 * @return true
 * @return false
 */
static inline bool ep_is_tx(struct usb_endpoint_configuration *ep)
{
    return ep->descriptor->bEndpointAddress & USB_DIR_IN;
}

/**
 * @brief Starts a transfer on a given endpoint.
 *
 * @param ep, the endpoint configuration.
 * @param buf, the data buffer to send. Only applicable if the endpoint is TX
 * @param len, the length of the data in buf (this example limits max len to
 *             one packet - 64 bytes)
 */
void usb_start_transfer(struct usb_endpoint_configuration *ep, uint8_t *buf,
                        uint16_t len)
{
    /*
     * Transfers larger than the EP max size should be handled by the EP
     * callback recognizing the state and continuing the transfer.
     */
    assert(len <= 64);

    DB_PRINT_L(3, "Starting transfer\n");
    for (size_t i = 0; i < len; ++i) {
        DB_PRINT(4, "%.2x%c", buf[i], (!(i % 0x08) && i > 0) ? '\n' : ' ');
    }
    DB_PRINT(4, "\n");

    /* Prepare buffer control register value */
    uint32_t val = len | USB_BUF_CTRL_AVAIL;

    if (ep_is_tx(ep)) {
        /* Need to copy the data from the user buffer to the usb memory */
        memcpy((void *) ep->data_buffer, (void *) buf, len);
        /* Mark as full */
        val |= USB_BUF_CTRL_FULL;
    }

    /* Set pid and flip for next transfer.  This swaps between buffers. */
    val |= ep->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
    ep->next_pid ^= 1u;

    *ep->buffer_control = val;
}

/**
 * @brief Send device descriptor to host
 */
void usb_handle_device_descriptor(volatile struct usb_setup_packet *pkt)
{
    size_t xfer_len;
    uint8_t *d = (uint8_t *) dev_config.device_descriptor;
    struct usb_endpoint_configuration *ep;
    ep = usb_get_endpoint_configuration(EP0_IN_ADDR);

    xfer_len = pkt->wLength <= sizeof(*dev_config.device_descriptor) ?
                               pkt->wLength :
                               sizeof(*dev_config.device_descriptor);

    /*
     * The host requested less length than the device descriptor size, we need
     * to deal with handling state now for the next transfer.
     *
     * Windows and Linux don't do this, but the Switch requests 8-byte packets
     * because either NVIDIA or Nintendo have to be a pain in the ass.
     */
    if (pkt->wLength + descr_sent < sizeof(*dev_config.device_descriptor)) {
//        usb_xfer_state = XFER_STATE_DEVICE_DESCR;
        descr_sent += pkt->wLength;
        descr_expected = sizeof(*dev_config.device_descriptor);
    } else {
        /* Otherwise we're done, reset state */
        usb_xfer_state = XFER_STATE_DONE;
        descr_sent = 0;
    }

    ep->next_pid = 1;
    usb_start_transfer(ep, d, xfer_len);
}

void usb_handle_device_descriptor_cont(void)
{
    uint8_t *d = (uint8_t *)dev_config.device_descriptor;
    size_t xfer_len;

    xfer_len = descr_expected - descr_sent;
    memcpy(ep0_buf, &hid_report_descriptor[descr_sent], xfer_len);
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR),
                       &d[descr_sent], xfer_len);
}

void usb_transfer_blocking(struct usb_endpoint_configuration *ep, uint8_t *buf,
                           uint16_t len)
{
    while (!reg_r32_mask(USBCTRL_REGS_BASE + USB_SIE_STATUS_OFFSET,
                        USB_SIE_STATUS_TRANS_COMPLETE_BITS));
    /* TRANS_COMPLETE is w1c */
    reg_w32(USBCTRL_REGS_BASE + USB_SIE_STATUS_OFFSET,
            USB_SIE_STATUS_TRANS_COMPLETE_BITS);

    usb_start_transfer(ep, buf, len);
}

/**
 * @brief Send the configuration descriptor (and potentially the configuration
 * and endpoint descriptors) to the host.
 *
 * @param pkt, the setup packet received from the host.
 */
void usb_handle_config_descriptor(volatile struct usb_setup_packet *pkt)
{
    uint8_t *buf = &ep0_buf[0];

    /* First request will want just the config descriptor */
    const struct usb_configuration_descriptor *d = dev_config.config_descriptor;
    const struct usb_endpoint_configuration *ep = dev_config.endpoints;
    memcpy((void *) buf, d, sizeof(struct usb_configuration_descriptor));
    buf += sizeof(struct usb_configuration_descriptor);

    /* If we have more than just the config descriptor copy it all */
    if (pkt->wLength >= d->wTotalLength) {
        memcpy((void *) buf, dev_config.interface_descriptor,
               sizeof(struct usb_interface_descriptor));
        buf += sizeof(struct usb_interface_descriptor);

        /* Send HID descriptor, if it exists */
        if (dev_config.hid_descriptor) {
            /* XXX: See FIXME in HID descriptor struct definition */
            assert(dev_config.hid_descriptor->bNumDescriptors == 1);

            memcpy(buf, dev_config.hid_descriptor,
                   sizeof(*dev_config.hid_descriptor));
            buf += sizeof(*dev_config.hid_descriptor);
        }

        /* Copy all the endpoint descriptors starting from EP1 */
        for (uint i = 2; i < USB_NUM_ENDPOINTS; i++) {
            if (ep[i].descriptor) {
                memcpy((void *) buf, ep[i].descriptor,
                       sizeof(struct usb_endpoint_descriptor));
                buf += sizeof(struct usb_endpoint_descriptor);
            }
        }
    }

    uint32_t len = (uint32_t) buf - (uint32_t) &ep0_buf[0];
    DB_PRINT_L(2, "Sending %d bytes\n", len);
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), &ep0_buf[0], len);
}

/**
 * @brief Handle a BUS RESET from the host by setting the device address back
 * to 0.
 */
void usb_bus_reset(void)
{
    dev_addr = 0;
    should_set_address = false;
    usb_hw->dev_addr_ctrl = 0;
    configured = false;
}

/**
 * @brief Send the requested string descriptor to the host.
 *
 * @param pkt, the setup packet from the host.
 */
void usb_handle_string_descriptor(volatile struct usb_setup_packet *pkt)
{
    uint8_t i = pkt->wValue & 0xff;
    uint8_t len = 0;

    if (i == 0) {
        len = 4;
        memcpy(&ep0_buf[0], dev_config.lang_descriptor, len);
    } else {
        /* Prepare fills in ep0_buf */
        /*
         * TODO: Jank leftover from the RPi foundation.
         * The string descriptors should already be UTF-16, we should just
         * copy the string descriptors to ep0_buf here.
         */
        len = usb_prepare_string_descriptor(dev_config.descriptor_strings[i - 1]);
    }

    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), &ep0_buf[0], len);
}

/**
 * @brief Handles a SET_ADDR request from the host. The actual setting of the
 *        device address in hardware is done in ep0_in_cb. This is because we
 *        have to acknowledge the request first as a device with address zero.
 *
 * @param pkt, the setup packet from the host.
 */
void usb_set_device_address(volatile struct usb_setup_packet *pkt)
{
    /*
     * Set address is a bit of a strange case because we have to send a 0 length
     * status packet first with address 0
     */
    dev_addr = (pkt->wValue & 0xff);
    DB_PRINT_L(2, "Set address %d\r\n", dev_addr);
    /* Will set address in the callback phase */
    should_set_address = true;
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
}

/**
 * @brief Handles a SET_CONFIGRUATION request from the host. Assumes one
 *        configuration so simply sends a zero length status packet back to the
 *        host.
 *
 * @param pkt, the setup packet from the host.
 */
void usb_set_device_configuration(volatile struct usb_setup_packet *pkt)
{
    /* Only one configuration so just acknowledge the request */
    DB_PRINT_L(1, "Device Enumerated\r\n");
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
    configured = true;
}

static void usb_handle_report_descriptor(volatile struct usb_setup_packet *pkt)
{
    uint16_t sent = 0;
    const uint8_t *p_desc = hid_report_descriptor;
    uint8_t xfer_len;

    xfer_len = 64 > pkt->wLength ? pkt->wLength : 64;
    memcpy(ep0_buf, hid_report_descriptor, xfer_len);
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR),
                       ep0_buf, xfer_len);

    /* Update global state so we know where we are */
    if (xfer_len == 64) {
        descr_sent = xfer_len;
        descr_expected = pkt->wLength > ARRAY_SIZE(genbu_report_desc) ?
                         ARRAY_SIZE(genbu_report_desc) : pkt->wLength;
        usb_xfer_state = XFER_STATE_HID_REPORT;
    } else {
        usb_xfer_state = XFER_STATE_DONE;
    }
}

static void usb_handle_report_descriptor_cont(void)
{
    uint8_t xfer_len;

    xfer_len = 64 > (descr_expected - descr_sent) ?
               (descr_expected - descr_sent) : 64;
    memcpy(ep0_buf, &hid_report_descriptor[descr_sent], xfer_len);
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR),
                       ep0_buf, xfer_len);

    /* update state */
    if (xfer_len == 64) {
        descr_sent += xfer_len;
    } else {
        usb_xfer_state = XFER_STATE_DONE;
        hid_ready = true;
    }
}

static void usb_handle_get_status(volatile struct usb_setup_packet *pkt)
{
    uint8_t status[2] = {0};
    uint8_t attrs = dev_config.config_descriptor->bmAttributes;
    if (attrs & 0x40) {
        status[0] |= 0x01;
    }
    if (attrs & 0x01) {
        status[0] |= 0x02;
    }

    usb_start_transfer(usb_get_endpoint_configuration(pkt->wIndex),
                       status, 2);
}

static void usb_handle_clear_feature(volatile struct usb_setup_packet *pkt)
{
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
}

/**
 * @brief Respond to a setup packet from the host.
 */
void usb_handle_setup_packet(void) {
    volatile struct usb_setup_packet *pkt = (volatile struct usb_setup_packet *)
                                            &usb_dpram->setup_packet;

    /*
     * TODO: The RPi foundation did not write this out correctly.
     * bmRequestType is a bitfield, and they omitted a lot of bRequest
     */
    uint8_t req_direction = pkt->bmRequestType;
    uint8_t req = pkt->bRequest;
    uint16_t descriptor_type;

    DB_PRINT_L(2, "Received setup packet:\n"
               " - bmRequestType = 0x%x\n"
               " - bRequest = 0x%x\n"
               " - wValue = 0x%x\n"
               " - wIndex = 0x%x\n"
               " - wLength = 0x%x\n",
               pkt->bmRequestType, pkt->bRequest, pkt->wValue, pkt->wIndex,
               pkt->wLength);

    /* Reset PID to 1 for EP0 IN */
    usb_get_endpoint_configuration(EP0_IN_ADDR)->next_pid = 1u;

    if (req_direction == USB_DIR_OUT) {
        if (req == USB_REQUEST_SET_ADDRESS) {
            usb_set_device_address(pkt);
        } else if (req == USB_REQUEST_SET_CONFIGURATION) {
            usb_set_device_configuration(pkt);
        } else {
            DB_PRINT_L(1, "Other OUT request (0x%x)\r\n", pkt->bRequest);
        }
    } else if (req_direction == USB_DIR_IN) {
        if (req == USB_REQUEST_GET_DESCRIPTOR) {
            descriptor_type = pkt->wValue >> 8;

            switch (descriptor_type) {
                case USB_DT_DEVICE:
                    usb_handle_device_descriptor(pkt);
                    DB_PRINT_L(1, "GET DEVICE DESCRIPTOR\r\n");
                    break;

                case USB_DT_CONFIG:
                    usb_handle_config_descriptor(pkt);
                    DB_PRINT_L(1, "GET CONFIG DESCRIPTOR\r\n");
                    break;

                case USB_DT_STRING:
                    usb_handle_string_descriptor(pkt);
                    DB_PRINT_L(1, "GET STRING DESCRIPTOR\r\n");
                    break;

                default:
                    DB_PRINT_L(1, "Unhandled GET_DESCRIPTOR type 0x%x\r\n", descriptor_type);
            }
        } else {
            DB_PRINT_L(1, "Other IN request (0x%x)\r\n", pkt->bRequest);
        }
    /* XXX: HID IN setup request, not sure what to call this variable */
    } else if (req_direction == USB_DIR_EP_IN) {
        descriptor_type = pkt->wValue >> 8;
        switch (descriptor_type) {
        case USB_HID_DESCRIPTOR_REPORT:
            DB_PRINT_L(1, "GET REPORT DESCRIPTOR\r\n");
            usb_handle_report_descriptor(pkt);
            break;

        default:
            DB_PRINT_L(1, "Unhandled hid GET_DESCRIPTOR type 0x%x\r\n", descriptor_type);
        }
    } else if (pkt->bmRequestType == 0x02) {
        switch (req) {
        case USB_REQUEST_GET_STATUS:
            usb_handle_get_status(pkt);
            break;
        case USB_REQUEST_CLEAR_FEATURE:
            usb_handle_clear_feature(pkt);
            break;

        }
    }
}

/**
 * @brief Notify an endpoint that a transfer has completed.
 *
 * @param ep, the endpoint to notify.
 */
static void usb_handle_ep_buff_done(struct usb_endpoint_configuration *ep)
{
    uint32_t buffer_control = *ep->buffer_control;
    uint16_t len = buffer_control & USB_BUF_CTRL_LEN_MASK;

    ep->handler((uint8_t *) ep->data_buffer, len);
}

/**
 * @brief Find the endpoint configuration for a specified endpoint number and
 *        direction and notify it that a transfer has completed.
 *
 * @param ep_num
 * @param in
 */
static void usb_handle_buff_done(uint ep_num, bool in)
{
    uint8_t ep_addr = ep_num | (in ? USB_DIR_IN : 0);
    DB_PRINT_L(3, "EP %d (in = %d) done\n", ep_num, in);
    for (uint i = 0; i < USB_NUM_ENDPOINTS; i++) {
        struct usb_endpoint_configuration *ep = &dev_config.endpoints[i];
        if (ep->descriptor && ep->handler) {
            if (ep->descriptor->bEndpointAddress == ep_addr) {
                usb_handle_ep_buff_done(ep);
                return;
            }
        }
    }
}

/**
 * @brief Handle a "buffer status" irq. This means that one or more
 *        buffers have been sent / received. Notify each endpoint where this
 *        is the case.
 */
static void usb_handle_buff_status() {
    uint32_t buffers = usb_hw->buf_status;
    uint32_t remaining_buffers = buffers;

    uint bit = 1u;
    for (uint i = 0; remaining_buffers && i < USB_NUM_ENDPOINTS * 2; i++) {
        if (remaining_buffers & bit) {
            // clear this in advance
            usb_hw_clear->buf_status = bit;
            // IN transfer for even i, OUT transfer for odd i
            usb_handle_buff_done(i >> 1u, !(i & 1u));
            remaining_buffers &= ~bit;
        }
        bit <<= 1u;
    }
}

/**
 * @brief USB interrupt handler
 */
__irq_handler void isr_usbctrl(void) {
    uint32_t status = usb_hw->ints;
    uint32_t handled = 0;

    DB_PRINT_L(3, "IRQ status: 0x%x\n", status);

    /* Setup packet received */
    if (status & USB_INTS_SETUP_REQ_BITS) {
        handled |= USB_INTS_SETUP_REQ_BITS;
        usb_hw_clear->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
        usb_handle_setup_packet();
    }

    /* Buffer status, one or more buffers have completed */
    if (status & USB_INTS_BUFF_STATUS_BITS) {
        handled |= USB_INTS_BUFF_STATUS_BITS;
        usb_handle_buff_status();
    }

    /* Bus is reset */
    if (status & USB_INTS_BUS_RESET_BITS) {
        DB_PRINT_L(1, "BUS RESET\n");
        handled |= USB_INTS_BUS_RESET_BITS;
        usb_hw_clear->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;
        usb_bus_reset();
    }

    if (status ^ handled) {
        panic("Unhandled IRQ 0x%x\n", (uint) (status ^ handled));
    }
}

void usb_handle_xfer(USBXferState state)
{
    switch(state) {
    case XFER_STATE_HID_REPORT:
        usb_handle_report_descriptor_cont();
        break;
    /*case XFER_STATE_DEVICE_DESCR:
        usb_handle_device_descriptor_cont();
        break;*/
    }
}

void ep0_in_cb(uint8_t *buf, uint16_t len)
{
    if (should_set_address) {
        /* Set actual device address in hardware */
        usb_hw->dev_addr_ctrl = dev_addr;
        should_set_address = false;
    } else {
        if (usb_xfer_state != XFER_STATE_DONE) {
            usb_handle_xfer(usb_xfer_state);
            /*usb_start_transfer(usb_get_endpoint_configuration(EP0_OUT_ADDR),
                               NULL, 0);*/
        } else {
            /* Receive a zero length status packet from the host on EP0 OUT */
            usb_start_transfer(usb_get_endpoint_configuration(EP0_OUT_ADDR),
                               NULL, 0);
        }
    }
}

void ep0_out_cb(uint8_t *buf, uint16_t len)
{
    ;
}

__prio_queue void *send_gamepad(void *buf)
{
    uint8_t *usb_buf = (uint8_t *)buf;

    DB_PRINT_L(3, "Starting transfer\n");
    for (size_t i = 0; i < ARRAY_SIZE(gamepad_template); ++i) {
        DB_PRINT(4, "%.2x%c", usb_buf[i], (!(i % 0x08) && i > 0) ? '\n' : ' ');
    }
    DB_PRINT(4, "\n");
    /*
     * XXX: Need a way to pass in size of buf, rather than assuming
     * (correctly) that it is a gamepad buffer
     */
    usb_start_transfer(usb_get_endpoint_configuration(EP1_IN_ADDR), usb_buf,
                       ARRAY_SIZE(gamepad_template));

    return NULL;
}

void ep1_in_cb(uint8_t *buf, uint16_t len)
{
    DB_PRINT_L(3, "gamepad_held=%d\n", gamepad_held);

    if (gamepad_held) {
        /*
         * This looks scary, but should be okay.
         * buf points to USB memory, which has a global instance.
         * This memory should not change between when this function happens and
         * when the function added to the queue is executed, since that would
         * mean another USB transfer would have to have happened on EP1.
         *
         * TODO: A more proper thing to do would to be contain buf and len in
         * a struct that has a longer lifetime.
         * It would have to be global, but that's ugly.
         */
        proc_enqueue(send_gamepad, buf, PRIORITY_LEVEL_HIGHEST);
    }
}

void ep2_out_cb(uint8_t *buf, uint16_t len)
{
    DB_PRINT_L(3, "EP2 RX:\n");
    for (size_t i = 0; i < len; ++i) {
        DB_PRINT(3, "%.2x%c", buf[i], (!(i % 0x08) && i > 0) ? '\n' : ' ');
    }
    /* Ignore the host on this EP */
}

void usb_btn_map_to_buf(const gamepad_btn *map, size_t index, uint8_t *buf)
{
    size_t i;
    const gamepad_btn *btn = &map[index];

    for (i = 0; i < btn->num; ++i) {
        buf[btn->indeces[i]] ^= btn->bits[i];
    }
}

bool usb_dpad_map_to_buf(const gamepad_btn *usb_map,
                         const io_map_container *ioc, uint8_t *usb_buf)
{
    size_t i, j;
    const board_io *io_map = ioc->dpad_map;
    uint8_t press_cnt = 0;
    uint8_t dpad_val = 0;
    const gamepad_btn *btn;

    /*
     * D-pad input needs special handling on byte 2.
     * U D L R each have an even number assigned  to them, and that number is
     * added, averaged, and put in the buffer.
     *
     * XXX: In the case where the d-pad value is 8, we set the buffer to 0
     * (0x08 ^ 0x08), unless there's another value, then it's averaged like any
     * other value, but it's an 8 instead of 0.
     * We do this in a clunky way, and there should be an obvious better way
     * to do this, because this edge case would be weird to write in RTL.
     *
     * If there are multiple inputs, we add the inputs and then divide by the
     * number of inputs.
     * This might break on combinations such as LR, UD, and 3-4 button combos,
     * but this can't happen in normal situations.
     *
     * Along with this, there's an FF byte set at a set location in the USB
     * buffer.
     */
    for (i = 0; i < ioc->dpad_size; ++i) {
        if (io_map[i].state == STATE_BUTTON_PRESSED) {
            ++press_cnt;
            btn = &usb_map[i];
            dpad_val += btn->bits[0];

            /* Now do the FF byte(s) */
            for (j = 1; j < btn->num; ++j) {
                usb_buf[btn->indeces[j]] ^= btn->bits[j];
            }
        }
    }

    /* Now add the d-pad input */
    /* XXX: See above 8-case */
    if (press_cnt == 1) {
        usb_buf[btn->indeces[0]] = dpad_val == 0x08 ? 0 : dpad_val;
    } else if (press_cnt > 1) {
        dpad_val /= press_cnt;
        usb_buf[btn->indeces[0]] = dpad_val;
    }

    return !!press_cnt;
}

__prio_queue void *usb_gamepad_format_and_send(void *arg)
{
    uint8_t usb_buf[ARRAY_SIZE(gamepad_template)];
    size_t i;
    io_map_container *ioc = (io_map_container *)arg;
    board_io *io = ioc->btn_map;
    bool all_high = true;

    DB_PRINT_L(3, "Formatting buffer\n");

    memcpy(usb_buf, gamepad_template, ARRAY_SIZE(usb_buf));
    for (i = 0; i < ioc->btn_size; ++i) {
        if (io[i].state == STATE_BUTTON_PRESSED) {
            usb_btn_map_to_buf(io_btn_map, i, usb_buf);
            all_high = false;
        }
    }
    all_high = !usb_dpad_map_to_buf(io_dpad_map, ioc, usb_buf);

    send_gamepad(usb_buf);

    /*
     * XXX: Maybe ugly?  I would need to revisit this.
     * If not all of the buttons are high, we need to add another send function
     * to the priority queue.  This is because we need to keep sending presses
     * to tell the host that we are holding the button down.
     *
     * This will be handled by the EP1 callback.  An alternative would be to
     * add a timer to the timer queue equal to the polling rate, and when that
     * elapses, the callback for that is usb_gamepad_format_and_send.
     * In order to communicate with ep1_cb(), we use a global telling it if we
     * have buttons held and need to keep sending packets.
     * Another alternative would be something that is evoked when the
     * USB transfer complete interrupt is set.
     *
     * When the user releases all buttons the GPIO IRQ will update the map
     * accordingly, meaning all buttons will be high and we stop re-sending
     * packets.
     */
    if (all_high) {
        gamepad_held = false;
    } else {
        gamepad_held = true;
    }

    return NULL;
}

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

    /* Spin until configured */
    while (!configured);

    control_loop();

    return 0;
}
