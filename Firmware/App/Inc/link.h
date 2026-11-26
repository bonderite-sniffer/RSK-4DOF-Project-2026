/* Host <-> MCU byte framing. Pure C, no Zephyr deps (unit-tested on PC).
 *
 *  | 0xA5 | 0x5A | type | len | payload[len] | crc_lo | crc_hi |
 *
 *  CRC = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type, len, payload.
 *  All multi-byte payload fields are little-endian.
 */
#ifndef LINK_H_
#define LINK_H_

#include <stdint.h>
#include <stddef.h>

#define LINK_SOF0 0xA5
#define LINK_SOF1 0x5A
#define LINK_MAX_PAYLOAD 128
#define LINK_OVERHEAD 6

uint16_t link_crc16(const uint8_t *data, size_t len, uint16_t crc);

/* Returns total frame length written to out, or 0 if it doesn't fit. */
size_t link_encode(uint8_t type, const void *payload, uint8_t len,
		   uint8_t *out, size_t out_cap);

struct link_parser {
	uint8_t state;
	uint8_t type;
	uint8_t len;
	uint8_t idx;
	uint16_t crc_rx;
	uint32_t crc_errors;
	uint8_t buf[LINK_MAX_PAYLOAD];
};

void link_parser_init(struct link_parser *p);

/* Feed one byte. Returns 1 when a valid frame is ready in p->type/p->len/p->buf. */
int link_parser_feed(struct link_parser *p, uint8_t b);

#endif
