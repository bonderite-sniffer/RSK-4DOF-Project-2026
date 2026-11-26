#include "usb_link.h"
#include "link.h"
#include "app_config.h"
#include "usbd_cdc_if.h"
#include <string.h>

/* power-of-two byte FIFO with free-running indices */
typedef struct {
	uint8_t *buf;
	uint32_t mask;
	volatile uint32_t head;
	volatile uint32_t tail;
} Fifo;

static uint8_t s_rx_mem[2048];
static uint8_t s_tx_mem[8192];
static Fifo s_rx = {s_rx_mem, sizeof(s_rx_mem) - 1, 0, 0};
static Fifo s_tx = {s_tx_mem, sizeof(s_tx_mem) - 1, 0, 0};
static uint8_t s_chunk[512];   /* must stay valid until the USB transfer completes */
static uint32_t s_dropped;

static uint32_t f_count(const Fifo *f)
{
	return f->head - f->tail;
}

static uint32_t f_space(const Fifo *f)
{
	return (f->mask + 1u) - f_count(f);
}

static int f_put(Fifo *f, const uint8_t *d, uint32_t n)
{
	if (f_space(f) < n) {
		return -1;
	}
	for (uint32_t i = 0; i < n; i++) {
		f->buf[(f->head + i) & f->mask] = d[i];
	}
	__DMB();
	f->head += n;
	return 0;
}

static uint32_t f_get(Fifo *f, uint8_t *d, uint32_t max)
{
	uint32_t n = f_count(f);

	if (n > max) {
		n = max;
	}
	for (uint32_t i = 0; i < n; i++) {
		d[i] = f->buf[(f->tail + i) & f->mask];
	}
	__DMB();
	f->tail += n;
	return n;
}

void usb_link_init(void)
{
	s_rx.head = s_rx.tail = 0;
	s_tx.head = s_tx.tail = 0;
	s_dropped = 0;
}

void usb_link_rx_isr(const uint8_t *buf, uint32_t len)
{
	(void)f_put(&s_rx, buf, len);   /* host flooding us -> drop */
}

uint32_t usb_link_read(uint8_t *dst, uint32_t max)
{
	return f_get(&s_rx, dst, max);
}

int usb_link_send_frame(uint8_t type, const void *payload, uint8_t len)
{
	uint8_t frame[LINK_MAX_PAYLOAD + LINK_OVERHEAD];
	size_t n = link_encode(type, payload, len, frame, sizeof(frame));

	if (n == 0 || f_put(&s_tx, frame, (uint32_t)n) != 0) {
		s_dropped++;
		return -1;
	}
	return 0;
}

void usb_link_poll(void)
{
	uint32_t n = f_count(&s_tx);

	if (n == 0) {
		return;
	}
	if (n > sizeof(s_chunk)) {
		n = sizeof(s_chunk);
	}
	/* peek: only consume the bytes once the USB stack accepted them */
	for (uint32_t i = 0; i < n; i++) {
		s_chunk[i] = s_tx.buf[(s_tx.tail + i) & s_tx.mask];
	}
	if (APP_CDC_TRANSMIT(s_chunk, (uint16_t)n) == USBD_OK) {
		s_tx.tail += n;
	}
	/* USBD_BUSY: previous transfer still running, try again next pass.
	 * Not connected: CDC returns FAIL/BUSY, FIFO fills, new frames get dropped. */
}

uint32_t usb_link_dropped(void)
{
	return s_dropped;
}
