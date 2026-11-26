#include "telemetry.h"
#include "usb_link.h"
#include "motor_ctrl.h"
#include "robstride.h"
#include "can_bus.h"
#include "protocol.h"
#include <string.h>

static uint16_t s_seq;
static uint32_t s_last_health_ms;

void tm_send_ack(uint8_t cmd, int8_t status)
{
	uint8_t p[2] = {cmd, (uint8_t)status};

	usb_link_send_frame(EVT_ACK, p, sizeof(p));
}

void tm_send_event(uint8_t motor, uint8_t code, float value)
{
	uint8_t p[6] = {motor, code};

	memcpy(&p[2], &value, 4);
	usb_link_send_frame(EVT_EVENT, p, sizeof(p));
}

void tm_send_param(uint8_t motor, uint16_t index, uint8_t ok, uint32_t raw)
{
	uint8_t p[8];

	p[0] = motor;
	memcpy(&p[1], &index, 2);
	p[3] = ok;
	memcpy(&p[4], &raw, 4);
	usb_link_send_frame(EVT_PARAM, p, sizeof(p));
}

void tm_send_pong(void)
{
	pong_t p = {
		.fw_version = FW_VERSION,
		.ctrl_hz = APP_CTRL_HZ,
		.n_motors = APP_NUM_MOTORS,
		.n_bus = APP_NUM_BUS,
		.proto = PROTO_VERSION,
	};

	usb_link_send_frame(EVT_PONG, &p, sizeof(p));
}

void tm_send_health(void)
{
	health_t h;
	uint32_t now = HAL_GetTick();

	memset(&h, 0, sizeof(h));
	h.t_ms = now;
	for (int b = 0; b < 2; b++) {
		const McpStats *s = &g_can[b].mcp.st;

		h.bus[b].init_ok = g_can[b].mcp.ok;
		h.bus[b].eflg = s->eflg;
		h.bus[b].tec = s->tec;
		h.bus[b].rec = s->rec;
		h.bus[b].tx_sent = s->tx_sent;
		h.bus[b].tx_fail = s->tx_fail;
		h.bus[b].tx_busy = s->tx_busy;
		h.bus[b].rx_count = s->rx_count;
		h.bus[b].rx_ovf = s->rx_ovf;
		h.bus[b].bus_off = s->bus_off;
		h.bus[b].spi_err = g_can[b].spi.err_count;
	}
	for (int i = 0; i < APP_NUM_MOTORS && i < 4; i++) {
		uint32_t age = now - g_motor[i].last_fb_ms;

		h.vbus[i] = g_motor[i].vbus;
		h.fb_age_ms[i] = (uint16_t)(age > 65535u ? 65535u : age);
	}
	usb_link_send_frame(EVT_HEALTH, &h, sizeof(h));
}

void tm_tick(uint32_t ctrl_tick)
{
	uint32_t now = HAL_GetTick();

	if (g_cfg.telem_div && (ctrl_tick % g_cfg.telem_div) == 0) {
		telem_t t;

		memset(&t, 0, sizeof(t));
		t.t_ms = now;
		t.seq = s_seq++;
		t.host_ok = (now - g_last_host_ms) < 500u;
		t.n_motors = APP_NUM_MOTORS;
		for (int i = 0; i < APP_NUM_MOTORS && i < 4; i++) {
			const MotorCtrl *c = &g_ctrl[i];
			telem_motor_t *tm = &t.m[i];

			tm->mode = c->mode;
			tm->rs_mode = g_motor[i].rs_mode;
			tm->fault = g_motor[i].fault;
			tm->flags = (uint8_t)(((now - g_motor[i].last_fb_ms) < 100u ? TF_ONLINE : 0) |
					      (c->softlimited ? TF_SOFTLIM : 0));
			tm->pos = g_motor[i].pos;
			tm->vel = g_motor[i].vel;
			tm->torque = g_motor[i].torque;
			tm->temp = g_motor[i].temp;
			tm->iq = g_motor[i].iq;
			switch (c->mode) {
			case MODE_MIT: tm->setpoint = c->t_ff; break;
			case MODE_VEL: tm->setpoint = c->vel_cmd; break;
			case MODE_POS: tm->setpoint = c->pos_target; break;
			default:       tm->setpoint = 0.0f; break;
			}
		}
		usb_link_send_frame(EVT_TELEM, &t, sizeof(t));
	}

	if (now - s_last_health_ms >= APP_HEALTH_MS) {
		s_last_health_ms = now;
		tm_send_health();
	}
}
