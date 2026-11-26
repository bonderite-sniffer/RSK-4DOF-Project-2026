#include "motor_ctrl.h"
#include "robstride.h"
#include "calib.h"
#include "telemetry.h"
#include "gateway.h"
#include <math.h>
#include <string.h>

#define MS2T(ms) ((uint32_t)(((ms) * APP_CTRL_HZ) / 1000u))

MotorCtrl g_ctrl[APP_NUM_MOTORS];
volatile LiveView g_live;
volatile uint32_t g_last_host_ms;

AppCfg g_cfg = {
	.telem_div = APP_TELEM_DIV,
	.softlim_en = 1,
	.pos_min = -2.0f,     /* ~ -115 deg, Python/MATLAB overwrite this */
	.pos_max = 2.0f,
	.temp_max = 75.0f,
	.wd_ms = 500,
	.safe_kp = 8.0f,
	.safe_kd = 0.4f,
};

static uint32_t s_tick;

static float clampf(float x, float lo, float hi)
{
	return x < lo ? lo : (x > hi ? hi : x);
}

static int bit(uint8_t mask, int i)
{
	return (mask >> i) & 1u;
}

/* ------------------------------------------------------------ helpers */
void mc_init(void)
{
	memset(g_ctrl, 0, sizeof(g_ctrl));
	for (int i = 0; i < APP_NUM_MOTORS; i++) {
		g_ctrl[i].mode = MODE_IDLE;
		rs_write_u32(i, RS_IDX_ZERO_STA, 1, 1);   /* angle reported in -pi..pi */
	}
}

void mc_enter_run_mode(int i, uint8_t run_mode)
{
	/* run_mode may only change while stopped; keep the order on the wire */
	rs_stop(i, 0, 1);
	rs_write_u32(i, RS_IDX_RUN_MODE, run_mode, 1);
	rs_write_u32(i, RS_IDX_ZERO_STA, 1, 1);
	rs_enable(i, 1);
	g_ctrl[i].mode_ticks = 0;
	g_ctrl[i].softlimited = 0;
}

void mc_set_mode(int i, uint8_t mode)
{
	g_ctrl[i].mode = mode;
	g_ctrl[i].mode_ticks = 0;
	g_ctrl[i].softlimited = 0;
}

void mc_hold_position(int i, float target, float vel_lim, float cur_lim)
{
	mc_enter_run_mode(i, RS_MODE_CSP);
	rs_write_f32(i, RS_IDX_LIMIT_CUR, cur_lim, 1);
	rs_write_f32(i, RS_IDX_LIMIT_SPD, vel_lim, 1);
	g_ctrl[i].pos_target = target;
	g_ctrl[i].vel_lim = vel_lim;
	g_ctrl[i].cur_lim = cur_lim;
	mc_set_mode(i, MODE_POS);
}

void mc_safety_stop(int i, uint8_t event_code, float value)
{
	rs_stop(i, 0, 0);
	mc_set_mode(i, MODE_IDLE);
	tm_send_event((uint8_t)i, event_code, value);
}

int mc_any_active(void)
{
	for (int i = 0; i < APP_NUM_MOTORS; i++) {
		if (g_ctrl[i].mode != MODE_IDLE) {
			return 1;
		}
	}
	return 0;
}

int mc_is_idle(int i)
{
	return g_ctrl[i].mode == MODE_IDLE;
}

/* ------------------------------------------------------------ host commands */
int mc_cmd_pos(uint8_t mask, float target, float vel_lim, float cur_lim)
{
	if (vel_lim <= 0.0f || vel_lim > RS_V_MAX || cur_lim <= 0.0f || cur_lim > 11.0f) {
		return ST_BAD_ARG;
	}
	if (g_cfg.softlim_en) {
		target = clampf(target, g_cfg.pos_min, g_cfg.pos_max);
	}
	for (int i = 0; i < APP_NUM_MOTORS; i++) {
		if (!bit(mask, i) || g_ctrl[i].mode == MODE_CALIB) {
			continue;
		}
		if (g_ctrl[i].mode != MODE_POS) {
			mc_enter_run_mode(i, RS_MODE_CSP);
			mc_set_mode(i, MODE_POS);
		}
		rs_write_f32(i, RS_IDX_LIMIT_CUR, cur_lim, 1);
		rs_write_f32(i, RS_IDX_LIMIT_SPD, vel_lim, 1);
		g_ctrl[i].pos_target = target;
		g_ctrl[i].vel_lim = vel_lim;
		g_ctrl[i].cur_lim = cur_lim;
	}
	return ST_OK;
}

int mc_cmd_vel(uint8_t mask, float vel, float cur_lim, float acc)
{
	if (fabsf(vel) > RS_V_MAX || cur_lim <= 0.0f || cur_lim > 11.0f || acc <= 0.0f) {
		return ST_BAD_ARG;
	}
	for (int i = 0; i < APP_NUM_MOTORS; i++) {
		if (!bit(mask, i) || g_ctrl[i].mode == MODE_CALIB) {
			continue;
		}
		if (g_ctrl[i].mode != MODE_VEL) {
			mc_enter_run_mode(i, RS_MODE_VEL);
			mc_set_mode(i, MODE_VEL);
		}
		rs_write_f32(i, RS_IDX_LIMIT_CUR, cur_lim, 1);
		rs_write_f32(i, RS_IDX_ACC_RAD, acc, 1);
		g_ctrl[i].vel_cmd = vel;
		g_ctrl[i].cur_lim = cur_lim;
		g_ctrl[i].acc = acc;
		g_ctrl[i].softlimited = 0;
	}
	return ST_OK;
}

int mc_cmd_mit(uint8_t mask, float p, float v, float kp, float kd, float t)
{
	for (int i = 0; i < APP_NUM_MOTORS; i++) {
		if (!bit(mask, i) || g_ctrl[i].mode == MODE_CALIB) {
			continue;
		}
		if (g_ctrl[i].mode != MODE_MIT) {
			mc_enter_run_mode(i, RS_MODE_MIT);
			mc_set_mode(i, MODE_MIT);
		}
		g_ctrl[i].p_des = clampf(p, -RS_P_MAX, RS_P_MAX);
		g_ctrl[i].v_des = clampf(v, -RS_V_MAX, RS_V_MAX);
		g_ctrl[i].kp = clampf(kp, 0.0f, RS_KP_MAX);
		g_ctrl[i].kd = clampf(kd, 0.0f, RS_KD_MAX);
		g_ctrl[i].t_ff = t;
		g_ctrl[i].softlimited = 0;
	}
	return ST_OK;
}

int mc_disable(uint8_t mask, int clear_fault)
{
	for (int i = 0; i < APP_NUM_MOTORS; i++) {
		if (bit(mask, i)) {
			calib_abort(i);
			rs_stop(i, clear_fault, 0);
			mc_set_mode(i, MODE_IDLE);
		}
	}
	return ST_OK;
}

void mc_estop(void)
{
	for (int i = 0; i < APP_NUM_MOTORS; i++) {
		calib_abort(i);
		rs_stop(i, 0, 0);
		mc_set_mode(i, MODE_IDLE);
	}
	tm_send_event(0xFF, EV_ESTOP, 0.0f);
}

/* ------------------------------------------------------------ tick */
static void step_motor(int i)
{
	MotorCtrl *c = &g_ctrl[i];
	volatile RsMotor *m = &g_motor[i];
	float pos = m->pos;
	uint8_t hi = g_cfg.softlim_en && pos > g_cfg.pos_max;
	uint8_t lo = g_cfg.softlim_en && pos < g_cfg.pos_min;

	c->mode_ticks++;

	switch (c->mode) {
	case MODE_IDLE:
		/* STOP is harmless when stopped and makes the motor answer with feedback
		 * (so position is visible while you move the arm by hand) */
		if ((s_tick + (uint32_t)i) % MS2T(20) == 0) {
			rs_stop(i, 0, 0);
		}
		break;

	case MODE_MIT:
		if ((hi || lo) && !c->softlimited) {
			c->softlimited = 1;
			c->p_des = hi ? g_cfg.pos_max : g_cfg.pos_min;
			c->v_des = 0.0f;
			c->kp = g_cfg.safe_kp;
			c->kd = g_cfg.safe_kd;
			c->t_ff = 0.0f;
			tm_send_event((uint8_t)i, EV_SOFTLIMIT, pos);
		}
		rs_mit(i, c->p_des, c->v_des, c->kp, c->kd, c->t_ff);
		break;

	case MODE_VEL:
		if ((hi && c->vel_cmd > 0.0f) || (lo && c->vel_cmd < 0.0f)) {
			c->vel_cmd = 0.0f;
			c->softlimited = 1;
			tm_send_event((uint8_t)i, EV_SOFTLIMIT, pos);
		}
		rs_write_f32(i, RS_IDX_SPD_REF, c->vel_cmd, 0);
		break;

	case MODE_POS:
		rs_write_f32(i, RS_IDX_LOC_REF, c->pos_target, 0);
		break;

	case MODE_CALIB:
		calib_step(i);
		break;
	}

	/* secondary reads, staggered per motor: current every 4th tick, VBUS every 100th */
	uint32_t k = s_tick + (uint32_t)i;
	if (k % 100u == 50u) {
		rs_read(i, RS_IDX_VBUS);
	} else if ((k & 3u) == 1u) {
		rs_read(i, RS_IDX_IQF);
	}

	/* safety (only while we drive the motor) */
	if (c->mode == MODE_IDLE) {
		c->last_fault = m->fault;
		return;
	}
	uint32_t age = HAL_GetTick() - m->last_fb_ms;
	if (c->mode_ticks > MS2T(300) && age > 200u) {
		calib_abort(i);
		mc_safety_stop(i, EV_OFFLINE, (float)age);
	} else if (m->temp > g_cfg.temp_max) {
		calib_abort(i);
		mc_safety_stop(i, EV_OVERTEMP, m->temp);
	} else if (m->fault != 0 && m->fault != c->last_fault) {
		calib_abort(i);
		mc_safety_stop(i, EV_FAULT, (float)m->fault);
	}
	c->last_fault = m->fault;
}

void mc_tick(void)
{
	/* gateway on: Python drives the motors through raw CAN frames. Do NOT send anything here
	 * (the IDLE STOP polling would disable motors Python just enabled). */
	if (!g_gw.enabled) {
		for (int i = 0; i < APP_NUM_MOTORS; i++) {
			step_motor(i);
		}
	}

	/* host watchdog: Python/MATLAB send a heartbeat at 20-25 Hz */
	if (!g_gw.enabled && g_cfg.wd_ms && mc_any_active() && (HAL_GetTick() - g_last_host_ms) > g_cfg.wd_ms) {
		for (int i = 0; i < APP_NUM_MOTORS; i++) {
			if (g_ctrl[i].mode != MODE_IDLE) {
				calib_abort(i);
				mc_safety_stop(i, EV_WATCHDOG, 0.0f);
			}
		}
	}

	/* live-expression mirror */
	uint32_t now = HAL_GetTick();
	for (int i = 0; i < APP_NUM_MOTORS; i++) {
		g_live.pos_deg[i] = g_motor[i].pos * 57.29578f;
		g_live.speed_rpm[i] = g_motor[i].vel * 9.549297f;
		g_live.torque_nm[i] = g_motor[i].torque;
		g_live.temp_c[i] = g_motor[i].temp;
		g_live.online[i] = (now - g_motor[i].last_fb_ms) < 100u;
	}

	tm_tick(s_tick);
	s_tick++;
}
