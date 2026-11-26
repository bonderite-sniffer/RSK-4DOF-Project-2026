/* can_bus.h - two CAN buses (MCP2515 on SPI1 / SPI2): send, receive+decode,
 * health counters and motor identification (ID scan). */
#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "mcp2515.h"
#include <stdint.h>

typedef struct {
	SpiDev spi;
	Mcp2515 mcp;
	uint8_t idx;                 /* 0 = CAN1, 1 = CAN2 */
	int init_err;                /* result of mcp2515_init */
	uint32_t last_rx_ms;
	uint32_t spi_err_base;
	volatile uint8_t scanning;
	uint8_t scan_bitmap[16];     /* bit n = CAN ID n answered type-0 */
} CanBus;

extern CanBus g_can[2];

void can_bus_init_all(void);

/* Normal send: waits up to ~300 us for a free TX buffer. 0 ok, <0 fail. */
int can_bus_send(uint8_t bus, const CanFrame *f);

/* Ordered send: waits until the chip has nothing pending, sends, waits again.
 * Use for sequences whose order matters (STOP -> run_mode -> ENABLE, zero, set ID). */
int can_bus_send_ordered(uint8_t bus, const CanFrame *f);

/* Drain received frames and hand them to robstride.c. Call often from the main loop. */
void can_bus_poll(uint8_t bus);

/* Refresh TEC/REC/EFLG (call ~10 Hz). */
void can_bus_health_update(void);

/* Blocking (~120 ms) scan of CAN IDs 1..127 with RobStride type-0 "get ID".
 * Writes found IDs to ids[], returns count, or -1 if the bus is broken. */
int can_bus_scan(uint8_t bus, uint8_t *ids, uint8_t max_ids);

/* Called by robstride.c when a type-0 reply arrives. */
void can_bus_mark_seen(uint8_t bus, uint8_t motor_id);

/* microsecond helpers (DWT cycle counter) */
void app_dwt_init(void);
uint32_t app_micros(void);
void app_delay_us(uint32_t us);

#endif
