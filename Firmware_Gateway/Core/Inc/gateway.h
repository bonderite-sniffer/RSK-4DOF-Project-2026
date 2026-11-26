/* gateway.h - "Python commands, STM executes" mode (protocol v3).
 *
 * With the gateway ON the STM32 is a 2-channel USB <-> CAN bridge for the Python tools in
 * python/canable (rs05_gui.py, rs05_multi_gui.py, ...):
 *
 *   Python  --CMD_CAN_TX (bus, id, data)-->  per-bus TX queue  --MCP2515-->  CAN1 / CAN2
 *   Python  <--EVT_CAN_RX (bus, id, data, t_us)--  every frame received on CAN1 / CAN2
 *
 *  - Order is kept: a queued frame goes to the MCP2515 only when ALL its TX buffers are empty
 *    (the chip may otherwise send buffer 2 before buffer 0). STOP -> run_mode -> ENABLE stays in order.
 *  - The STM-side controller (motor_ctrl.c) is passive: no STOP polling, no parameter reads,
 *    no EVT_TELEM. Python's Controller does the control and safety logic.
 *  - Watchdog: no frame from the host for wd_ms -> queued frames are dropped and a STOP (type 4)
 *    goes to every CAN ID seen on each bus (+ the IDs in APP_MOTOR_IDS). EVT_EVENT EV_WATCHDOG.
 *  - Received frames are batched (up to GW_RX_BATCH per USB message) to save USB bandwidth.
 *
 * Gateway OFF (default after reset): the legacy protocol-v2 behaviour (CMD_POS / CMD_VEL /
 * CMD_MIT, STM-side modes, python/rs05_host.py) is unchanged.
 */
#ifndef GATEWAY_H
#define GATEWAY_H

#include "mcp2515.h"
#include <stdint.h>

#define GW_TXQ_LEN    64u     /* frames per bus, power of two */
#define GW_RX_BATCH   6u      /* CAN frames per EVT_CAN_RX message (6 x 19 = 114 bytes) */
#define GW_STATS_MS   200u

typedef struct {
	uint8_t enabled;
	uint8_t wd_tripped;          /* watchdog fired, cleared by the next host frame */
	uint32_t wd_ms;              /* 0 = off */
	uint32_t wd_trips;
	uint32_t tx_ok[2];           /* frames handed to the MCP2515 */
	uint32_t tx_drop[2];         /* host frames dropped: queue full or bus not initialised */
	uint32_t rx_fwd[2];          /* frames forwarded to the host */
	uint8_t txq_peak[2];         /* highest queue fill seen */
	uint8_t seen[2][32];         /* bit n = CAN ID n sent something on that bus (private protocol) */
} GwState;

extern volatile GwState g_gw;    /* Live Expressions */

void gw_init(void);

/* CMD_GATEWAY: enable / disable, host watchdog in ms. Returns ST_*. */
int gw_set(uint8_t enable, uint16_t wd_ms);

/* CMD_CAN_TX payload: n x 15 bytes. Returns ST_*. */
int gw_cmd_can_tx(const uint8_t *p, uint8_t len);

/* called by can_bus_poll() for every received frame */
void gw_on_rx(uint8_t bus, const CanFrame *f);

/* main loop: feed the MCP2515s, flush RX batch, watchdog, stats */
void gw_poll(void);

/* STOP every motor seen on both buses (watchdog, ESTOP) */
void gw_stop_all(void);

/* encoding helpers (exposed for tests) */
void gw_pack_rx(uint8_t *out19, uint8_t bus, const CanFrame *f, uint32_t t_us);
int gw_unpack_tx(const uint8_t *in15, uint8_t *bus, CanFrame *f);

#endif
