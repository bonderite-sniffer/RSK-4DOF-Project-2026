/* sim.c - PC simulation of the hardware under App/:
 *   - time base (DWT cycle counter @ 400 MHz drives HAL_GetTick)
 *   - two MCP2515 chips emulated at register level (SPI instructions)
 *   - RobStride motors on each bus with a tiny physics model + hard stops
 *   - USB CDC: transmitted bytes collected for the test to parse
 */
#include "main.h"
#include "usbd_cdc_if.h"
#include "sim.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

GPIO_TypeDef GPIOA_s = {0}, GPIOC_s = {2}, GPIOD_s = {3};
SPI_HandleTypeDef hspi1 = {1}, hspi2 = {2};
TIM_HandleTypeDef htim6 = {6};
uint32_t SystemCoreClock = 400000000u;
CoreDebug_Type sim_coredebug;
static DWT_Type s_dwt;
static uint64_t s_cycles;

/* ------------------------------------------------------------ time */
static void advance_us(uint32_t us)
{
	s_cycles += (uint64_t)us * 400u;
	s_dwt.CYCCNT = (uint32_t)s_cycles;
}

DWT_Type *sim_dwt(void)
{
	advance_us(1);
	return &s_dwt;
}

uint32_t HAL_GetTick(void)
{
	return (uint32_t)(s_cycles / 400000u);
}

void HAL_Delay(uint32_t ms)
{
	advance_us(ms * 1000u);
}

uint64_t sim_now_us(void)
{
	return s_cycles / 400u;
}

void sim_advance_us(uint32_t us)
{
	advance_us(us);
}

HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *h)
{
	(void)h;
	return HAL_OK;
}

/* ------------------------------------------------------------ motors */
SimMotor g_sim_motor[SIM_MAX_MOTORS];
int g_sim_nmotors;

static float u2f(uint16_t u, float lo, float hi)
{
	return (float)u * (hi - lo) / 65535.0f + lo;
}

static uint16_t f2u(float x, float lo, float hi)
{
	if (x < lo) x = lo;
	if (x > hi) x = hi;
	return (uint16_t)((x - lo) * 65535.0f / (hi - lo) + 0.5f);
}

static void physics(SimMotor *m)
{
	uint64_t now = sim_now_us();
	float dt = (float)(now - m->last_us) * 1e-6f;

	m->last_us = now;
	if (!m->enabled) {
		m->vel = 0;
		return;
	}
	float p = m->raw_pos;
	if (m->run_mode == 2) {
		m->vel = m->spd_ref;
	} else if (m->run_mode == 5) {
		float target = m->loc_ref + m->zero;
		float e = target - p;
		float step = m->limit_spd * dt;
		m->vel = fabsf(e) < 1e-4f ? 0.0f : (e > 0 ? m->limit_spd : -m->limit_spd);
		if (fabsf(e) <= step) {
			p = target;
			m->vel = 0;
		} else {
			p += (e > 0 ? step : -step);
		}
		m->raw_pos = p;
	} else {
		m->vel = 0;
	}
	if (m->run_mode == 2) {
		p += m->vel * dt;
		if (p >= m->stop_hi && m->vel > 0) { p = m->stop_hi; m->vel = 0; }
		if (p <= m->stop_lo && m->vel < 0) { p = m->stop_lo; m->vel = 0; }
		m->raw_pos = p;
	}
}

static void make_fb(SimMotor *m, uint8_t host, uint32_t *id, uint8_t *d)
{
	physics(m);
	float pos = m->raw_pos - m->zero;
	*id = (2u << 24) | ((uint32_t)(m->enabled ? 2 : 0) << 22) | ((uint32_t)m->id << 8) | host;
	uint16_t p = f2u(pos, -12.566370614f, 12.566370614f), v = f2u(m->vel, -50, 50);
	uint16_t t = f2u(m->enabled ? 0.3f : 0.0f, -6, 6), tc = (uint16_t)(m->temp * 10);
	d[0] = p >> 8; d[1] = p; d[2] = v >> 8; d[3] = v; d[4] = t >> 8; d[5] = t; d[6] = tc >> 8; d[7] = tc;
}

/* a frame arrives at the motors on `bus`; returns number of replies written */
static int motors_rx(int bus, uint32_t id, const uint8_t *d, uint32_t *rid, uint8_t rd[][8], int *acked)
{
	uint8_t type = (id >> 24) & 0x1F, target = id & 0xFF, host = (id >> 8) & 0xFF;
	int n = 0;

	*acked = 0;
	for (int k = 0; k < g_sim_nmotors; k++) {
		SimMotor *m = &g_sim_motor[k];

		if (m->bus != bus || !m->present) continue;
		*acked = 1;   /* any node on the bus ACKs */
		if (m->id != target) continue;
		m->rx_frames++;
		switch (type) {
		case 0:
			rid[n] = ((uint32_t)m->id << 8) | 0xFE;
			memset(rd[n], 0, 8);
			n++;
			break;
		case 1: case 3: case 4: case 6: case 18:
			physics(m);
			if (type == 3) m->enabled = 1;
			if (type == 4) { m->enabled = 0; if (d[0]) m->fault = 0; }
			if (type == 6) m->zero = m->raw_pos;
			if (type == 18) {
				uint16_t idx = d[0] | (d[1] << 8);
				float f; memcpy(&f, &d[4], 4);
				if (idx == 0x7005) m->run_mode = d[4];
				if (idx == 0x700A) m->spd_ref = f;
				if (idx == 0x7016) m->loc_ref = f;
				if (idx == 0x7017) m->limit_spd = f;
			}
			make_fb(m, host, &rid[n], rd[n]);
			n++;
			break;
		case 7:
			m->id = (id >> 16) & 0xFF;
			rid[n] = ((uint32_t)m->id << 8) | 0xFE;   /* reply like type 0 */
			memset(rd[n], 0, 8);
			n++;
			break;
		case 17: {
			uint16_t idx = d[0] | (d[1] << 8);
			float v = idx == 0x701C ? 48.2f : (idx == 0x701A ? 0.42f : 0.0f);
			rid[n] = (17u << 24) | ((uint32_t)m->id << 8) | host;
			memset(rd[n], 0, 8);
			rd[n][0] = d[0]; rd[n][1] = d[1];
			memcpy(&rd[n][4], &v, 4);
			n++;
			break;
		}
		default:
			break;
		}
	}
	return n;
}

/* ------------------------------------------------------------ MCP2515 */
typedef struct {
	uint8_t reg[128];
	int cs_low;
	uint32_t overflow;
} SimMcp;

static SimMcp s_mcp[2];
uint32_t g_sim_rx_overflow[2];

static SimMcp *mcp_of(SPI_HandleTypeDef *h) { return &s_mcp[h->id == 1 ? 0 : 1]; }

static void mcp_reset(SimMcp *c)
{
	memset(c->reg, 0, sizeof(c->reg));
	c->reg[0x0E] = 0x80;
	c->reg[0x0F] = 0x87;
}

static void enc_id(uint32_t id, uint8_t *o)
{
	o[0] = id >> 21; o[1] = ((id >> 13) & 0xE0) | 0x08 | ((id >> 16) & 3); o[2] = id >> 8; o[3] = id;
}

static void mcp_rx_push(SimMcp *c, int bus, uint32_t id, const uint8_t *d)
{
	uint8_t base;

	if (!(c->reg[0x2C] & 0x01)) { base = 0x61; c->reg[0x2C] |= 0x01; }
	else if (!(c->reg[0x2C] & 0x02)) { base = 0x71; c->reg[0x2C] |= 0x02; }
	else { c->reg[0x2D] |= 0x40; c->reg[0x2C] |= 0x20; g_sim_rx_overflow[bus]++; return; }
	enc_id(id, &c->reg[base]);
	c->reg[base + 4] = 8;
	memcpy(&c->reg[base + 5], d, 8);
}

static void mcp_transmit(SimMcp *c, int bus, int n)
{
	uint8_t *t = &c->reg[0x31 + 0x10 * n];
	uint32_t id = ((uint32_t)t[0] << 21) | ((uint32_t)(t[1] & 0xE0) << 13) | ((uint32_t)(t[1] & 3) << 16) | ((uint32_t)t[2] << 8) | t[3];
	uint32_t rid[8]; uint8_t rd[8][8]; int acked;

	sim_advance_us(130);   /* frame on the wire */
	int nr = motors_rx(bus, id, &t[5], rid, rd, &acked);
	c->reg[0x30 + 0x10 * n] &= ~0x08;   /* TXREQ clear */
	if (!acked) c->reg[0x30 + 0x10 * n] |= 0x50;   /* one-shot: ABTF | TXERR */
	for (int k = 0; k < nr; k++) mcp_rx_push(c, bus, rid[k], rd[k]);
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, uint8_t *tx, uint8_t *rx, uint16_t n, uint32_t timeout)
{
	(void)timeout;
	SimMcp *c = mcp_of(h);
	int bus = h->id == 1 ? 0 : 1;
	uint8_t cmd = tx[0];

	sim_advance_us(2 + n);   /* ~6 MHz SPI */
	memset(rx, 0, n);
	if (cmd == 0xC0) {
		mcp_reset(c);
	} else if (cmd == 0x03) {
		for (int i = 2; i < n; i++) rx[i] = c->reg[(tx[1] + i - 2) & 0x7F];
	} else if (cmd == 0x02) {
		for (int i = 2; i < n; i++) {
			uint8_t a = (tx[1] + i - 2) & 0x7F;
			c->reg[a] = tx[i];
			if (a == 0x0F) c->reg[0x0E] = (c->reg[0x0E] & 0x1F) | (tx[i] & 0xE0);
		}
	} else if (cmd == 0x05) {
		uint8_t a = tx[1] & 0x7F;
		c->reg[a] = (c->reg[a] & ~tx[2]) | (tx[3] & tx[2]);
	} else if (cmd == 0xA0) {
		uint8_t s = 0, f = c->reg[0x2C];
		s |= f & 0x03;
		if (c->reg[0x30] & 0x08) s |= 0x04;
		if (c->reg[0x40] & 0x08) s |= 0x10;
		if (c->reg[0x50] & 0x08) s |= 0x40;
		rx[1] = s;
	} else if (cmd == 0xB0) {
		rx[1] = (uint8_t)(((c->reg[0x2C] & 1) ? 0x40 : 0) | ((c->reg[0x2C] & 2) ? 0x80 : 0));
	} else if (cmd == 0x90 || cmd == 0x94) {
		uint8_t base = cmd == 0x90 ? 0x61 : 0x71;
		for (int i = 1; i < n; i++) rx[i] = c->reg[base + i - 1];
		c->reg[0x2C] &= cmd == 0x90 ? ~0x01 : ~0x02;
	} else if ((cmd & 0xF8) == 0x40) {
		int b = (cmd >> 1) & 3;
		for (int i = 1; i < n; i++) c->reg[0x31 + 0x10 * b + i - 1] = tx[i];
	} else if ((cmd & 0xF8) == 0x80) {
		for (int b = 0; b < 3; b++) {
			if (cmd & (1 << b)) {
				c->reg[0x30 + 0x10 * b] = (c->reg[0x30 + 0x10 * b] & ~0x70) | 0x08;
				mcp_transmit(c, bus, b);
			}
		}
	}
	return HAL_OK;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState s)
{
	(void)port; (void)pin; (void)s;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
	SimMcp *c = (port == GPIOC) ? &s_mcp[0] : &s_mcp[1];
	(void)pin;
	sim_advance_us(1);
	return (c->reg[0x2C] & c->reg[0x2B]) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

/* ------------------------------------------------------------ USB */
uint8_t g_sim_usb_out[1 << 20];
uint32_t g_sim_usb_len;

uint8_t CDC_Transmit_HS(uint8_t *buf, uint16_t len)
{
	if (g_sim_usb_len + len < sizeof(g_sim_usb_out)) {
		memcpy(&g_sim_usb_out[g_sim_usb_len], buf, len);
		g_sim_usb_len += len;
	}
	return USBD_OK;
}
