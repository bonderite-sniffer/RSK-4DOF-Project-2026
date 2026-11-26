#ifndef USBD_CDC_IF_H
#define USBD_CDC_IF_H
#include <stdint.h>
#define USBD_OK 0
#define USBD_BUSY 1
uint8_t CDC_Transmit_HS(uint8_t *buf, uint16_t len);
#endif
