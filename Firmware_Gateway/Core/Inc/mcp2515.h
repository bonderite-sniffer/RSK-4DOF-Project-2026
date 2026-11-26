/* mcp2515.h - minimal, polled MCP2515 driver (extended frames, 1 Mbit/s).
 *
 * - One-shot mode: a frame nobody ACKs is dropped instead of retried forever,
 *   so a missing motor can never lock the TX buffers.
 * - RX: both buffers accept everything (filtering is done in software),
 *   RXB0 rolls over into RXB1.
 * - INT pin (active low) asserts on RX or error; poll it from the main loop.
 */
#ifndef MCP2515_H
#define MCP2515_H

#include "spi_bus.h"
#include <stdint.h>

typedef struct {
	uint32_t id;      /* 29-bit if ext, else 11-bit */
	uint8_t ext;
	uint8_t dlc;
	uint8_t data[8];
} CanFrame;

typedef struct {
	uint32_t tx_sent;   /* frames handed to the chip            */
	uint32_t tx_fail;   /* one-shot aborted / error (no ACK...) */
	uint32_t tx_busy;   /* no free TX buffer when asked         */
	uint32_t rx_count;
	uint32_t rx_ovf;    /* RX buffer overflow (we were too slow) */
	uint32_t bus_off;   /* entries into bus-off                  */
	uint8_t eflg;       /* last EFLG register                    */
	uint8_t tec, rec;   /* error counters                        */
} McpStats;

typedef struct {
	SpiDev *spi;
	GPIO_TypeDef *int_port;
	uint16_t int_pin;
	uint8_t ok;         /* init succeeded (chip answered, timing verified) */
	uint8_t init_step;  /* last init step reached (debug, Live Expressions) */
	uint8_t init_canstat;
	uint8_t cnf_wr[3], cnf_rd[3];   /* CNF3, CNF2, CNF1 written / read back during init */
	McpStats st;
} Mcp2515;

/* Returns 0 ok, <0 error code (-1 SPI, -2 no reset/config mode, -3 CNF readback, -4 normal mode) */
int mcp2515_init(Mcp2515 *m, SpiDev *spi, GPIO_TypeDef *int_port, uint16_t int_pin);

/* 0 = queued, -1 = all 3 TX buffers busy, -2 = SPI error */
int mcp2515_send(Mcp2515 *m, const CanFrame *f);

/* 1 = no frame pending in any TX buffer */
int mcp2515_tx_idle(Mcp2515 *m);

/* 1 = frame read into *f, 0 = nothing, -1 = SPI error */
int mcp2515_recv(Mcp2515 *m, CanFrame *f);

/* INT pin asserted (low) */
int mcp2515_int_active(const Mcp2515 *m);

/* Read/clear error flags (call when INT is low but no frame was pending) */
void mcp2515_service_errors(Mcp2515 *m);

/* Refresh TEC/REC/EFLG into m->st (call ~10 Hz) */
void mcp2515_read_counters(Mcp2515 *m);

/* Bit timing registers for the configured crystal/bitrate (exposed for tests) */
void mcp2515_timing(uint8_t *cnf1, uint8_t *cnf2, uint8_t *cnf3);

/* Encode / decode the 4 ID bytes (SIDH, SIDL, EID8, EID0) - exposed for tests */
void mcp2515_encode_id(uint32_t id, uint8_t ext, uint8_t out[4]);
void mcp2515_decode_id(const uint8_t in[4], uint32_t *id, uint8_t *ext);

#endif
