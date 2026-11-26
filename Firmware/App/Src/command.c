#include "command.h"
#include "link.h"
#include "usb_link.h"
#include "telemetry.h"
#include "motor_ctrl.h"
#include "calib.h"
#include "robstride.h"
#include "can_bus.h"
#include "protocol.h"
#include <string.h>

static struct link_parser s_parser;

static float f32(const uint8_t *p)
{
	float f;

	memcpy(&f, p, 4);
	return f;
}

void command_init(void)
{
	link_parser_init(&s_parser);
}

static int8_t set_cfg(uint8_t key, float v)
{
	switch (key) {
	case CFG_TSCALE:
		if (v < 1.0f || v > 200.0f) {
			return ST_BAD_ARG;
		}
		rs_t_cmd_max = v;
		return ST_OK;
	case CFG_TELEM_DIV:
		g_cfg.telem_div = v < 1.0f ? 1u : (uint32_t)v;
		return ST_OK;
	case CFG_SOFTLIM_EN:
		g_cfg.softlim_en = v > 0.5f;
		return ST_OK;
	case CFG_POS_MIN:
		g_cfg.pos_min = v;
		return ST_OK;
	case CFG_POS_MAX:
		g_cfg.pos_max = v;
		return ST_OK;
	case CFG_TEMP_MAX:
		g_cfg.temp_max = v;
		return ST_OK;
	case CFG_WD_MS:
		g_cfg.wd_ms = v < 0.0f ? 0u : (uint32_t)v;
		return ST_OK;
	case CFG_HOST_ID:
		rs_host_id = (uint8_t)v;
		return ST_OK;
	case CFG_SAFE_KP:
		g_cfg.safe_kp = v;
		return ST_OK;
	case CFG_SAFE_KD:
		g_cfg.safe_kd = v;
		return ST_OK;
	default:
		if (key >= CFG_MOTOR_ID0 && key < CFG_MOTOR_ID0 + APP_NUM_MOTORS) {
			int i = key - CFG_MOTOR_ID0;

			if (!mc_is_idle(i) || v < 1.0f || v > 127.0f) {
				return ST_BAD_ARG;
			}
			g_motor[i].can_id = (uint8_t)v;
			return ST_OK;
		}
		return ST_BAD_ARG;
	}
}

static void do_scan(uint8_t bus_mask)
{
	for (uint8_t b = 0; b < APP_NUM_BUS; b++) {
		if (!(bus_mask & (1u << b))) {
			continue;
		}
		uint8_t out[3 + 120];
		int n = can_bus_scan(b, &out[3], 120);

		out[0] = b;
		out[1] = n < 0 ? 1 : 0;
		out[2] = n < 0 ? 0 : (uint8_t)n;
		usb_link_send_frame(EVT_SCAN, out, (uint8_t)(3 + out[2]));
	}
}

void command_execute(uint8_t type, const uint8_t *p, uint8_t len)
{
	static const struct { uint8_t type, len; } LEN[] = {
		{CMD_PING, 0}, {CMD_ESTOP, 0}, {CMD_HEARTBEAT, 0}, {CMD_SET_CFG, 5}, {CMD_SCAN, 1},
		{CMD_GET_HEALTH, 0}, {CMD_DISABLE, 2}, {CMD_POS, 13}, {CMD_VEL, 13}, {CMD_MIT, 21},
		{CMD_READ_PARAM, 3}, {CMD_WRITE_PARAM, 8}, {CMD_CALIB, 18},
	};
	int known = 0;

	g_last_host_ms = HAL_GetTick();   /* any valid frame feeds the watchdog */

	for (unsigned k = 0; k < sizeof(LEN) / sizeof(LEN[0]); k++) {
		if (LEN[k].type == type) {
			known = 1;
			if (LEN[k].len != len) {
				tm_send_ack(type, ST_BAD_LEN);
				return;
			}
		}
	}
	if (!known) {
		tm_send_ack(type, ST_UNKNOWN);
		return;
	}

	int8_t st = ST_OK;
	uint8_t mask = len ? p[0] : 0;

	switch (type) {
	case CMD_HEARTBEAT:
		return;   /* no ACK */
	case CMD_PING:
		tm_send_pong();
		return;
	case CMD_GET_HEALTH:
		tm_send_health();
		return;
	case CMD_ESTOP:
		mc_estop();
		break;
	case CMD_SET_CFG:
		st = set_cfg(p[0], f32(&p[1]));
		break;
	case CMD_SCAN:
		if (mc_any_active()) {
			st = ST_BUSY;
		} else {
			do_scan(p[0]);
		}
		break;
	case CMD_DISABLE:
		st = (int8_t)mc_disable(mask, p[1]);
		break;
	case CMD_POS:
		st = (int8_t)mc_cmd_pos(mask, f32(&p[1]), f32(&p[5]), f32(&p[9]));
		break;
	case CMD_VEL:
		st = (int8_t)mc_cmd_vel(mask, f32(&p[1]), f32(&p[5]), f32(&p[9]));
		break;
	case CMD_MIT:
		st = (int8_t)mc_cmd_mit(mask, f32(&p[1]), f32(&p[5]), f32(&p[9]), f32(&p[13]), f32(&p[17]));
		break;
	case CMD_READ_PARAM: {
		uint16_t index;

		memcpy(&index, &p[1], 2);
		if (p[0] >= APP_NUM_MOTORS) {
			st = ST_BAD_ARG;
			break;
		}
		g_motor[p[0]].pending_param = index;
		rs_read(p[0], index);
		break;
	}
	case CMD_WRITE_PARAM: {
		uint16_t index;
		uint32_t raw;

		memcpy(&index, &p[1], 2);
		memcpy(&raw, &p[4], 4);
		if (p[0] >= APP_NUM_MOTORS) {
			st = ST_BAD_ARG;
			break;
		}
		rs_write_u32(p[0], index, raw, 1);
		break;
	}
	case CMD_CALIB:
		st = (int8_t)calib_start(p[0], p[1], f32(&p[2]), f32(&p[6]), f32(&p[10]), f32(&p[14]));
		break;
	default:
		st = ST_UNKNOWN;
	}
	tm_send_ack(type, st);
}

void command_poll(void)
{
	uint8_t buf[64];
	uint32_t n;

	while ((n = usb_link_read(buf, sizeof(buf))) > 0) {
		for (uint32_t i = 0; i < n; i++) {
			if (link_parser_feed(&s_parser, buf[i])) {
				command_execute(s_parser.type, s_parser.buf, s_parser.len);
			}
		}
	}
}
