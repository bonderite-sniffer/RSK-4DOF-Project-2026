/* usb_link.h - byte FIFOs between USB CDC and the application.
 *
 *  RX: CDC_Receive_HS (USB interrupt) -> usb_link_rx_isr() -> rx FIFO -> command.c
 *  TX: telemetry/command -> usb_link_send_frame() -> tx FIFO -> usb_link_poll()
 *      -> CDC_Transmit_HS when the previous transfer has finished.
 * Only the RX FIFO is touched from interrupt context (single producer/consumer).
 */
#ifndef USB_LINK_H
#define USB_LINK_H

#include <stdint.h>

void usb_link_init(void);

/* call from CDC_Receive_HS() in usbd_cdc_if.c */
void usb_link_rx_isr(const uint8_t *buf, uint32_t len);

/* read up to max bytes received from the host; returns count */
uint32_t usb_link_read(uint8_t *dst, uint32_t max);

/* frame + queue for sending; 0 ok, -1 dropped (FIFO full / host not reading) */
int usb_link_send_frame(uint8_t type, const void *payload, uint8_t len);

/* push queued bytes into the USB stack; call every main-loop pass */
void usb_link_poll(void);

uint32_t usb_link_dropped(void);

#endif
