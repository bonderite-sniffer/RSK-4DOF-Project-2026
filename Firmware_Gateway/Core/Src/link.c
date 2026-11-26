#include "link.h"
#include <string.h>

enum { S_SOF0, S_SOF1, S_TYPE, S_LEN, S_PAY, S_CRC0, S_CRC1 };

uint16_t link_crc16(const uint8_t *data, size_t len, uint16_t crc)
{
	for (size_t i = 0; i < len; i++) {
		crc ^= (uint16_t)data[i] << 8;
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

size_t link_encode(uint8_t type, const void *payload, uint8_t len,
		   uint8_t *out, size_t out_cap)
{
	if (len > LINK_MAX_PAYLOAD || out_cap < (size_t)len + LINK_OVERHEAD) {
		return 0;
	}
	out[0] = LINK_SOF0;
	out[1] = LINK_SOF1;
	out[2] = type;
	out[3] = len;
	if (len) {
		memcpy(&out[4], payload, len);
	}
	uint16_t crc = link_crc16(&out[2], (size_t)len + 2, 0xFFFF);
	out[4 + len] = (uint8_t)(crc & 0xFF);
	out[5 + len] = (uint8_t)(crc >> 8);
	return (size_t)len + LINK_OVERHEAD;
}

void link_parser_init(struct link_parser *p)
{
	memset(p, 0, sizeof(*p));
	p->state = S_SOF0;
}

int link_parser_feed(struct link_parser *p, uint8_t b)
{
	switch (p->state) {
	case S_SOF0:
		if (b == LINK_SOF0) {
			p->state = S_SOF1;
		}
		break;
	case S_SOF1:
		p->state = (b == LINK_SOF1) ? S_TYPE : (b == LINK_SOF0 ? S_SOF1 : S_SOF0);
		break;
	case S_TYPE:
		p->type = b;
		p->state = S_LEN;
		break;
	case S_LEN:
		if (b > LINK_MAX_PAYLOAD) {
			p->state = S_SOF0;
			break;
		}
		p->len = b;
		p->idx = 0;
		p->state = b ? S_PAY : S_CRC0;
		break;
	case S_PAY:
		p->buf[p->idx++] = b;
		if (p->idx >= p->len) {
			p->state = S_CRC0;
		}
		break;
	case S_CRC0:
		p->crc_rx = b;
		p->state = S_CRC1;
		break;
	case S_CRC1: {
		p->crc_rx |= (uint16_t)b << 8;
		p->state = S_SOF0;
		uint8_t hdr[2] = {p->type, p->len};
		uint16_t crc = link_crc16(hdr, 2, 0xFFFF);
		crc = link_crc16(p->buf, p->len, crc);
		if (crc == p->crc_rx) {
			return 1;
		}
		p->crc_errors++;
		break;
	}
	default:
		p->state = S_SOF0;
	}
	return 0;
}
