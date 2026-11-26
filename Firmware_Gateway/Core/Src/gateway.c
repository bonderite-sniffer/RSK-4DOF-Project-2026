/* gateway.c - USB <-> 2x CAN bridge for the Python tools (see gateway.h). */
#include "gateway.h"
#include "can_bus.h"
#include "usb_link.h"
#include "telemetry.h"
#include "motor_ctrl.h"
#include "robstride.h"
#include "protocol.h"
#include <string.h>

volatile GwState g_gw;

typedef struct {
	CanFrame f[GW_TXQ_LEN];
	uint32_t head, tail;           /* free running, only touched from the main loop */
} TxQueue;

static TxQueue s_q[2];
static uint8_t s_batch[GW_RX_BATCH * GW_RX_SIZE];
static uint8_t s_batch_n;
static uint32_t s_stats_ms;

_Static_assert((GW_TXQ_LEN & (GW_TXQ_LEN - 1u)) == 0, "GW_TXQ_LEN must be a power of two");
_Static_assert(GW_RX_BATCH * GW_RX_SIZE <= 128, "RX batch must fit LINK_MAX_PAYLOAD");
_Static_assert(sizeof(gw_stats_t) == 44, "gw_stats_t layout is shared with python/canable/rs05_stm.py");

/* ------------------------------------------------------------ 32-bit microsecond clock (wraps ~71 min) */
static uint32_t s_last_cyc, s_us, s_cyc_rem;

static uint32_t gw_time_us(void)
{
	uint32_t per = SystemCoreClock / 1000000u;
	uint32_t c = DWT->CYCCNT;
	uint32_t d = (c - s_last_cyc) + s_cyc_rem;   /* called every main-loop pass: no CYCCNT wrap in between */

	s_last_cyc = c;
	s_us += d / per;
	s_cyc_rem = d % per;
	return s_us;
}

/* ------------------------------------------------------------ queue */
static uint32_t q_count(const TxQueue *q)
{
	return q->head - q->tail;
}

static int q_push(TxQueue *q, const CanFrame *f)
{
	if (q_count(q) >= GW_TXQ_LEN) {
		return -1;
	}
	q->f[q->head & (GW_TXQ_LEN - 1u)] = *f;
	q->head++;
	return 0;
}

static void q_clear(TxQueue *q)
{
	q->tail = q->head;
}

/* ------------------------------------------------------------ packing (little endian) */
static void put_le32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

static uint32_t get_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

void gw_pack_rx(uint8_t *o, uint8_t bus, const CanFrame *f, uint32_t t_us)
{
	o[0] = bus;
	o[1] = f->ext ? GW_FLAG_EXT : 0u;
	o[2] = f->dlc > 8 ? 8 : f->dlc;
	put_le32(&o[3], f->id);
	put_le32(&o[7], t_us);
	memcpy(&o[11], f->data, 8);
}

int gw_unpack_tx(const uint8_t *in, uint8_t *bus, CanFrame *f)
{
	uint8_t flags = in[1];

	*bus = in[0];
	memset(f, 0, sizeof(*f));
	f->ext = (flags & GW_FLAG_EXT) ? 1u : 0u;
	f->dlc = in[2];
	f->id = get_le32(&in[3]);
	memcpy(f->data, &in[7], 8);
	if (*bus >= APP_NUM_BUS || f->dlc > 8 || (flags & GW_FLAG_RTR)) {
		return -1;                       /* RTR frames are not supported by the driver */
	}
	if ((f->ext && f->id > 0x1FFFFFFFu) || (!f->ext && f->id > 0x7FFu)) {
		return -1;
	}
	return 0;
}

/* ------------------------------------------------------------ RX batch -> USB */
static void flush_batch(void)
{
	if (s_batch_n == 0) {
		return;
	}
	if (usb_link_send_frame(EVT_CAN_RX, s_batch, (uint8_t)(s_batch_n * GW_RX_SIZE)) == 0) {
		for (uint8_t k = 0; k < s_batch_n; k++) {
			g_gw.rx_fwd[s_batch[k * GW_RX_SIZE] & 1u]++;
		}
	}
	/* else: USB FIFO full (host not reading) - usb_link counts the drop */
	s_batch_n = 0;
}

void gw_on_rx(uint8_t bus, const CanFrame *f)
{
	if (!g_gw.enabled || bus >= APP_NUM_BUS) {
		return;
	}
	if (f->ext) {
		uint8_t src = (uint8_t)((f->id >> 8) & 0xFF);   /* private protocol: sender in bits 15..8 */

		if (src != rs_host_id) {
			g_gw.seen[bus][src >> 3] |= (uint8_t)(1u << (src & 7u));
		}
	}
	gw_pack_rx(&s_batch[s_batch_n * GW_RX_SIZE], bus, f, gw_time_us());
	if (++s_batch_n >= GW_RX_BATCH) {
		flush_batch();
	}
}

/* ------------------------------------------------------------ host commands */
int gw_set(uint8_t enable, uint16_t wd_ms)
{
	if (enable) {
		if (mc_any_active()) {
			mc_disable(0xFF, 0);         /* STM-side modes off: STOP + IDLE */
		}
		q_clear(&s_q[0]);
		q_clear(&s_q[1]);
		s_batch_n = 0;
		g_gw.wd_ms = wd_ms;
		g_gw.wd_tripped = 0;
		(void)gw_time_us();
		g_gw.enabled = 1;
	} else {
		flush_batch();
		q_clear(&s_q[0]);
		q_clear(&s_q[1]);
		g_gw.enabled = 0;
	}
	return ST_OK;
}

int gw_cmd_can_tx(const uint8_t *p, uint8_t len)
{
	if (len == 0 || len % GW_TX_SIZE != 0) {
		return ST_BAD_LEN;
	}
	if (!g_gw.enabled) {
		return ST_BUSY;                  /* enable the gateway first (CMD_GATEWAY) */
	}
	int st = ST_OK;

	for (uint8_t k = 0; k < len / GW_TX_SIZE; k++) {
		uint8_t bus;
		CanFrame f;

		if (gw_unpack_tx(&p[k * GW_TX_SIZE], &bus, &f) != 0) {
			st = ST_BAD_ARG;
			continue;
		}
		if (!g_can[bus].mcp.ok || q_push(&s_q[bus], &f) != 0) {
			g_gw.tx_drop[bus]++;
			st = ST_CAN_ERR;
			continue;
		}
		uint32_t n = q_count(&s_q[bus]);
		if (n > g_gw.txq_peak[bus]) {
			g_gw.txq_peak[bus] = (uint8_t)n;
		}
	}
	return st;
}

void gw_stop_all(void)
{
	for (uint8_t b = 0; b < APP_NUM_BUS; b++) {
		q_clear(&s_q[b]);                /* drop pending host frames (could be ENABLE / MIT) */
		for (int i = 0; i < APP_NUM_MOTORS; i++) {
			if (g_motor[i].bus == b) {
				g_gw.seen[b][g_motor[i].can_id >> 3] |= (uint8_t)(1u << (g_motor[i].can_id & 7u));
			}
		}
		for (uint16_t id = 0; id < 256u; id++) {
			if ((g_gw.seen[b][id >> 3] & (1u << (id & 7u))) && id != rs_host_id) {
				CanFrame f;

				rs_make_frame(&f, RS_T_STOP, rs_host_id, (uint8_t)id);
				(void)q_push(&s_q[b], &f);
			}
		}
	}
}

/* ------------------------------------------------------------ main loop */
static void feed_bus(uint8_t b)
{
	TxQueue *q = &s_q[b];

	if (q_count(q) == 0 || !g_can[b].mcp.ok) {
		return;
	}
	/* strict order: next frame only when the chip has nothing pending in any TX buffer */
	if (!mcp2515_tx_idle(&g_can[b].mcp)) {
		return;
	}
	int r = mcp2515_send(&g_can[b].mcp, &q->f[q->tail & (GW_TXQ_LEN - 1u)]);

	if (r == 0) {
		q->tail++;
		g_gw.tx_ok[b]++;
	} else if (r == -2) {                /* SPI error: drop it, do not block the queue forever */
		q->tail++;
		g_gw.tx_drop[b]++;
	}
}

static void send_stats(uint32_t now)
{
	gw_stats_t s;

	memset(&s, 0, sizeof(s));
	s.t_ms = now;
	for (int b = 0; b < 2; b++) {
		s.tx_ok[b] = g_gw.tx_ok[b];
		s.tx_drop[b] = g_gw.tx_drop[b];
		s.rx_fwd[b] = g_gw.rx_fwd[b];
		s.txq_peak[b] = g_gw.txq_peak[b];
		s.txq_len[b] = (uint8_t)q_count(&s_q[b]);
	}
	s.usb_dropped = usb_link_dropped();
	s.wd_trips = g_gw.wd_trips;
	s.enabled = g_gw.enabled;
	s.wd_tripped = g_gw.wd_tripped;
	usb_link_send_frame(EVT_GW_STATS, &s, sizeof(s));
}

void gw_init(void)
{
	memset((void *)&g_gw, 0, sizeof(g_gw));
	memset(s_q, 0, sizeof(s_q));
	s_batch_n = 0;
	s_last_cyc = DWT->CYCCNT;
#if APP_BOOT_GATEWAY
	gw_set(1, APP_GW_WD_MS);
#endif
}

void gw_poll(void)
{
	uint32_t now = HAL_GetTick();

	(void)gw_time_us();                  /* keep the us clock running (CYCCNT wraps every ~10 s) */
	if (!g_gw.enabled) {
		return;
	}
	feed_bus(0);
	feed_bus(1);
	flush_batch();

	/* host watchdog */
	uint32_t silent = now - g_last_host_ms;
	if (g_gw.wd_ms && silent > g_gw.wd_ms) {
		if (!g_gw.wd_tripped) {
			g_gw.wd_tripped = 1;
			g_gw.wd_trips++;
			gw_stop_all();
			tm_send_event(0xFF, EV_WATCHDOG, (float)silent);
		}
	} else {
		g_gw.wd_tripped = 0;
	}

	if (now - s_stats_ms >= GW_STATS_MS) {
		s_stats_ms = now;
		send_stats(now);
	}
}
