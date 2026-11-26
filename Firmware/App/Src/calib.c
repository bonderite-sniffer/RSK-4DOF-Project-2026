#include "calib.h"
#include "motor_ctrl.h"
#include "robstride.h"
#include "can_bus.h"
#include "telemetry.h"
#include "protocol.h"
#include <math.h>
#include <string.h>

#define MS2T(ms) ((uint32_t)(((ms) * APP_CTRL_HZ) / 1000u))

enum { CS_NONE, CS_ZERO_VERIFY, CS_HOME_SEEK, CS_HOME_BACKOFF };

typedef struct {
	uint8_t state;
	int8_t dir;
	float speed, cur, backoff, target;
	uint32_t ticks, count;
} CalCtx;

static CalCtx s_cal[APP_NUM_MOTORS];

static int bit(uint8_t mask, int i)
{
	return (mask >> i) & 1u;
}

void calib_abort(int i)
{
	s_cal[i].state = CS_NONE;
}

static void finish(int i, uint8_t event, float value)
{
	s_cal[i].state = CS_NONE;
	mc_set_mode(i, MODE_IDLE);
	tm_send_event((uint8_t)i, event, value);
}

/* ------------------------------------------------------------ start */
static int start_zero(uint8_t mask)
{
	for (int i = 0; i < APP_NUM_MOTORS; i++) {
		if (bit(mask, i) && !mc_is_idle(i)) {
			return ST_BUSY;
		}
	}
	for (int i = 0; i < APP_NUM_MOTORS; i++) {
		if (!bit(mask, i)) {
			continue;
		}
		rs_stop(i, 0, 1);
		rs_set_zero(i);
		rs_write_u32(i, RS_IDX_ZERO_STA, 1, 1);
		memset(&s_cal[i], 0, sizeof(s_cal[i]));
		s_cal[i].state = CS_ZERO_VERIFY;
		mc_set_mode(i, MODE_CALIB);
	}
	return ST_OK;
}

static int start_home(uint8_t mask, float dir, float speed, float cur, float backoff)
{
	if (speed <= 0.0f || speed > 10.0f || cur <= 0.0f || cur > 11.0f || backoff < 0.0f ||
	    fabsf(dir) < 0.5f) {
		return ST_BAD_ARG;
	}
	for (int i = 0; i < APP_NUM_MOTORS; i++) {
		if (bit(mask, i) && !mc_is_idle(i)) {
			return ST_BUSY;
		}
	}
	for (int i = 0; i < APP_NUM_MOTORS; i++) {
		if (!bit(mask, i)) {
			continue;
		}
		CalCtx *k = &s_cal[i];

		memset(k, 0, sizeof(*k));
		k->dir = dir > 0.0f ? 1 : -1;
		k->speed = speed;
		k->cur = cur;
		k->backoff = backoff;
		k->state = CS_HOME_SEEK;
		mc_enter_run_mode(i, RS_MODE_VEL);
		rs_write_f32(i, RS_IDX_LIMIT_CUR, cur, 1);
		rs_write_f32(i, RS_IDX_ACC_RAD, 10.0f, 1);
		mc_set_mode(i, MODE_CALIB);
	}
	return ST_OK;
}

static int start_set_id(uint8_t mask, float new_id_f)
{
	int i = -1, n = 0;

	for (int k = 0; k < APP_NUM_MOTORS; k++) {
		if (bit(mask, k)) {
			i = k;
			n++;
		}
	}
	uint8_t new_id = (uint8_t)new_id_f;
	if (n != 1 || new_id_f < 1.0f || new_id_f > 127.0f) {
		return ST_BAD_ARG;
	}
	if (!mc_is_idle(i)) {
		return ST_BUSY;
	}
	int other = rs_find(g_motor[i].bus, new_id);
	if (other >= 0 && other != i) {
		return ST_BAD_ARG;   /* another motor in the table already uses that ID on this bus */
	}

	uint8_t bus = g_motor[i].bus;
	uint8_t old_id = g_motor[i].can_id;
	rs_stop(i, 0, 1);
	rs_set_can_id(i, new_id);
	HAL_Delay(50);

	/* verify: the motor must now answer "get ID" on the new ID */
	g_motor[i].can_id = new_id;
	memset(g_can[bus].scan_bitmap, 0, sizeof(g_can[bus].scan_bitmap));
	g_can[bus].scanning = 1;
	rs_get_id(i);
	uint32_t t0 = HAL_GetTick();
	while (HAL_GetTick() - t0 < 50) {
		can_bus_poll(bus);
	}
	g_can[bus].scanning = 0;

	if (g_can[bus].scan_bitmap[new_id >> 3] & (1u << (new_id & 7))) {
		tm_send_event((uint8_t)i, EV_ID_CHANGED, (float)new_id);
	} else {
		g_motor[i].can_id = old_id;   /* motor did not take it: keep talking to the old ID */
		tm_send_event((uint8_t)i, EV_ID_FAIL, (float)new_id);
	}
	return ST_OK;
}

int calib_start(uint8_t op, uint8_t mask, float a, float b, float c, float d)
{
	if ((mask & ((1u << APP_NUM_MOTORS) - 1u)) == 0) {
		return ST_BAD_ARG;
	}
	switch (op) {
	case CAL_ZERO_HERE:
		return start_zero(mask);
	case CAL_HOME:
		return start_home(mask, a, b, c, d);
	case CAL_SET_CAN_ID:
		return start_set_id(mask, a);
	case CAL_SAVE:
		for (int i = 0; i < APP_NUM_MOTORS; i++) {
			if (bit(mask, i)) {
				if (!mc_is_idle(i)) {
					return ST_BUSY;
				}
				rs_save(i);
			}
		}
		return ST_OK;
	default:
		return ST_BAD_ARG;
	}
}

/* ------------------------------------------------------------ step (APP_CTRL_HZ) */
void calib_step(int i)
{
	CalCtx *k = &s_cal[i];
	volatile RsMotor *m = &g_motor[i];

	k->ticks++;

	switch (k->state) {
	case CS_ZERO_VERIFY:
		/* motor is stopped; poll it so feedback arrives, then check pos ~ 0 */
		if (k->ticks % MS2T(10) == 0) {
			rs_stop(i, 0, 0);
		}
		if (k->ticks > MS2T(150)) {
			float p = m->pos;

			finish(i, fabsf(p) < 0.03f ? EV_ZERO_DONE : EV_ZERO_FAIL, p);
		}
		break;

	case CS_HOME_SEEK:
		rs_write_f32(i, RS_IDX_SPD_REF, (float)k->dir * k->speed, 0);
		if (k->ticks > MS2T(400)) {
			k->count = (fabsf(m->vel) < 0.25f * k->speed) ? k->count + 1 : 0;
		}
		if (k->count > MS2T(200)) {
			/* stalled on the hard stop: this is the mechanical reference */
			tm_send_event((uint8_t)i, EV_HOME_STALL, m->pos);
			rs_write_f32(i, RS_IDX_SPD_REF, 0.0f, 1);
			rs_stop(i, 0, 1);
			rs_set_zero(i);
			k->target = -(float)k->dir * k->backoff;
			mc_enter_run_mode(i, RS_MODE_CSP);
			rs_write_f32(i, RS_IDX_LIMIT_CUR, fminf(2.0f * k->cur, 11.0f), 1);
			rs_write_f32(i, RS_IDX_LIMIT_SPD, k->speed, 1);
			k->state = CS_HOME_BACKOFF;
			k->ticks = 0;
			k->count = 0;
		} else if (k->ticks > MS2T(20000)) {
			rs_stop(i, 0, 0);
			finish(i, EV_HOME_FAIL, 1.0f);
		}
		break;

	case CS_HOME_BACKOFF:
		rs_write_f32(i, RS_IDX_LOC_REF, k->target, 0);
		k->count = (fabsf(m->pos - k->target) < 0.03f) ? k->count + 1 : 0;
		if (k->count > MS2T(300)) {
			/* this point becomes the final zero; hold it */
			rs_stop(i, 0, 1);
			rs_set_zero(i);
			k->state = CS_NONE;
			mc_hold_position(i, 0.0f, k->speed, fminf(2.0f * k->cur, 11.0f));
			tm_send_event((uint8_t)i, EV_HOME_DONE, k->backoff);
		} else if (k->ticks > MS2T(10000)) {
			rs_stop(i, 0, 0);
			finish(i, EV_HOME_FAIL, 2.0f);
		}
		break;

	default:
		mc_set_mode(i, MODE_IDLE);
		break;
	}
}
