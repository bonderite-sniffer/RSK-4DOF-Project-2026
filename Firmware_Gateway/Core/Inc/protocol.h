/* protocol.h - Laptop (Python / MATLAB) <-> STM32 binary protocol, version 3.
 *
 * v3 = v2 + CAN gateway (CMD_GATEWAY, CMD_CAN_TX, EVT_CAN_RX, EVT_GW_STATS), see gateway.h.
 * Everything from v2 is unchanged, so python/rs05_host.py and matlab/RS05Link.m keep working
 * while the gateway is off.
 *
 * Framing (link.c): A5 5A | type | len | payload | crc16 (lo,hi)
 * CRC-16/CCITT-FALSE over type, len, payload. Payload little-endian, packed.
 * Motor mask: bit i = motor index i (bit0 = M1 ... bit3 = M4).
 * Units: rad, rad/s, Nm, A, degC.
 *
 * Keep in sync with python/rs05_host.py and matlab/RS05Link.m.
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define PROTO_VERSION 3u
#define FW_VERSION    0x00030000u   /* 3.0.0 (HAL bare-metal + CAN gateway) */

/* ---------------- host -> MCU ---------------- */
enum {
	CMD_PING        = 0x01, /* -                                            -> EVT_PONG   */
	CMD_ESTOP       = 0x02, /* -                                                          */
	CMD_HEARTBEAT   = 0x03, /* -  (no ACK, feeds watchdog)                                */
	CMD_SET_CFG     = 0x05, /* u8 key, f32 value                                          */
	CMD_SCAN        = 0x06, /* u8 bus_mask (bit0 CAN1, bit1 CAN2)           -> EVT_SCAN   */
	CMD_GET_HEALTH  = 0x07, /* -                                            -> EVT_HEALTH */
	CMD_DISABLE     = 0x11, /* u8 mask, u8 clear_fault                                    */
	CMD_POS         = 0x20, /* u8 mask, f32 target, f32 vel_lim, f32 cur_lim   (CSP)      */
	CMD_VEL         = 0x21, /* u8 mask, f32 vel, f32 cur_lim, f32 acc                     */
	CMD_MIT         = 0x22, /* u8 mask, f32 p, f32 v, f32 kp, f32 kd, f32 t               */
	CMD_READ_PARAM  = 0x30, /* u8 motor, u16 index                          -> EVT_PARAM  */
	CMD_WRITE_PARAM = 0x31, /* u8 motor, u16 index, u8 is_float, u32 raw                  */
	CMD_CALIB       = 0x40, /* u8 op, u8 mask, f32 a, f32 b, f32 c, f32 d   (calib.c)     */
	/* v3: gateway ("Python commands, STM executes", gateway.c) */
	CMD_GATEWAY     = 0x50, /* u8 enable, u16 wd_ms (0 = no watchdog)       -> EVT_ACK    */
	CMD_CAN_TX      = 0x51, /* n x gw_tx frame (15 bytes each, n = 1..8). ACK only on error */
};

/* CMD_CALIB ops */
enum {
	CAL_ZERO_HERE  = 1, /* mask                                                   */
	CAL_HOME       = 2, /* mask, a = dir (+1/-1), b = speed rad/s, c = cur A, d = backoff rad */
	CAL_SET_CAN_ID = 3, /* mask = exactly one motor, a = new CAN ID (1..127)       */
	CAL_SAVE       = 4, /* mask: ask motor to store params in flash (type 22)      */
};

/* CMD_SET_CFG keys */
enum {
	CFG_TSCALE     = 1,  /* type-1 torque field range +-Nm (RS05 = 5.5, default; RS02 = 17) */
	CFG_TELEM_DIV  = 2,
	CFG_SOFTLIM_EN = 3,
	CFG_POS_MIN    = 4,  /* rad */
	CFG_POS_MAX    = 5,
	CFG_TEMP_MAX   = 6,  /* degC */
	CFG_WD_MS      = 7,  /* host watchdog, 0 = off */
	CFG_HOST_ID    = 10,
	CFG_SAFE_KP    = 11,
	CFG_SAFE_KD    = 12,
	CFG_MOTOR_ID0  = 16, /* 16..19: CAN ID of motor index 0..3 (host-side table only) */
};

/* ---------------- MCU -> host ---------------- */
enum {
	EVT_ACK    = 0x80, /* u8 cmd, i8 status                                   */
	EVT_TELEM  = 0x81, /* struct telem                                        */
	EVT_SCAN   = 0x82, /* u8 bus, u8 status (0 ok, 1 bus error), u8 n, u8 ids[n] */
	EVT_PARAM  = 0x83, /* u8 motor, u16 index, u8 ok, u32 raw                 */
	EVT_EVENT  = 0x84, /* u8 motor (0xFF all), u8 code, f32 value             */
	EVT_PONG   = 0x85, /* struct pong                                         */
	EVT_HEALTH = 0x86, /* struct health                                       */
	EVT_CAN_RX = 0x87, /* n x gw_rx frame (19 bytes each, n = 1..GW_RX_BATCH)  */
	EVT_GW_STATS = 0x88, /* gw_stats_t, every 200 ms while the gateway is on  */
};

/* gateway frame layouts (little endian):
 *   gw_tx (host -> MCU, 15 B): u8 bus (0 CAN1, 1 CAN2), u8 flags, u8 dlc, u32 id, u8 data[8]
 *   gw_rx (MCU -> host, 19 B): u8 bus, u8 flags, u8 dlc, u32 id, u32 t_us, u8 data[8]
 *   flags: bit0 = extended (29-bit) id, bit1 = RTR (not supported, rejected) */
#define GW_TX_SIZE   15u
#define GW_RX_SIZE   19u
#define GW_FLAG_EXT  0x01u
#define GW_FLAG_RTR  0x02u

enum {
	ST_OK = 0, ST_BAD_LEN = -1, ST_BUSY = -2, ST_BAD_ARG = -4, ST_CAN_ERR = -5, ST_UNKNOWN = -6,
};

enum {
	EV_HOME_DONE = 1, EV_HOME_FAIL = 2, EV_SOFTLIMIT = 3, EV_OVERTEMP = 4, EV_FAULT = 5,
	EV_OFFLINE = 6, EV_WATCHDOG = 7, EV_ESTOP = 8, EV_CAN_ERROR = 9, EV_HOME_STALL = 10,
	EV_ZERO_DONE = 11, EV_ZERO_FAIL = 12, EV_ID_CHANGED = 13, EV_ID_FAIL = 14,
};

enum { MODE_IDLE = 0, MODE_MIT = 1, MODE_VEL = 2, MODE_POS = 3, MODE_CALIB = 4 };

#define TF_ONLINE  0x01
#define TF_SOFTLIM 0x02

typedef struct __attribute__((packed)) {
	uint8_t mode;     /* MODE_* */
	uint8_t rs_mode;  /* 0 reset, 1 cali, 2 run (reported by motor) */
	uint8_t fault;    /* motor fault bits */
	uint8_t flags;    /* TF_* */
	float pos, vel, torque, temp, iq, setpoint;
} telem_motor_t;      /* 28 bytes */

typedef struct __attribute__((packed)) {
	uint32_t t_ms;
	uint16_t seq;
	uint8_t host_ok;
	uint8_t n_motors;
	telem_motor_t m[4];
} telem_t;            /* 120 bytes */

typedef struct __attribute__((packed)) {
	uint8_t init_ok, eflg, tec, rec;
	uint32_t tx_sent, tx_fail, tx_busy, rx_count, rx_ovf, bus_off, spi_err;
} health_bus_t;       /* 32 bytes */

typedef struct __attribute__((packed)) {
	uint32_t t_ms;
	health_bus_t bus[2];
	float vbus[4];
	uint16_t fb_age_ms[4];
} health_t;           /* 92 bytes */

typedef struct __attribute__((packed)) {
	uint32_t t_ms;
	uint32_t tx_ok[2];       /* frames handed to the MCP2515 per bus            */
	uint32_t tx_drop[2];     /* host frames dropped (queue full / bus not up)   */
	uint32_t rx_fwd[2];      /* received frames forwarded to the host           */
	uint32_t usb_dropped;    /* USB messages dropped (host not reading)         */
	uint32_t wd_trips;       /* host watchdog stops                             */
	uint8_t txq_peak[2];
	uint8_t txq_len[2];
	uint8_t enabled;
	uint8_t wd_tripped;
	uint8_t rsv[2];
} gw_stats_t;         /* 44 bytes */

typedef struct __attribute__((packed)) {
	uint32_t fw_version;
	uint32_t ctrl_hz;
	uint8_t n_motors, n_bus, proto, rsv;
} pong_t;             /* 12 bytes */

#endif
