/* motor_ctrl.h - per-motor mode state machine, setpoints and safety.
 * mc_tick() runs at APP_CTRL_HZ from the main loop (flag set by TIM6). */
#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include "app_config.h"
#include "protocol.h"
#include <stdint.h>

typedef struct {
	uint8_t mode;          /* MODE_* (protocol.h) */
	uint32_t mode_ticks;
	uint8_t softlimited;
	uint8_t last_fault;
	/* MIT */
	float p_des, v_des, kp, kd, t_ff;
	/* VEL */
	float vel_cmd, cur_lim, acc;
	/* POS (CSP) */
	float pos_target, vel_lim;
} MotorCtrl;

typedef struct {
	uint32_t telem_div;
	uint8_t softlim_en;
	float pos_min, pos_max;   /* rad */
	float temp_max;           /* degC */
	uint32_t wd_ms;           /* host watchdog, 0 = off */
	float safe_kp, safe_kd;   /* MIT soft-limit spring */
} AppCfg;

extern MotorCtrl g_ctrl[APP_NUM_MOTORS];
extern AppCfg g_cfg;
extern volatile uint32_t g_last_host_ms;

/* Live-expression friendly mirror, in engineering units, updated every tick. */
typedef struct {
	float pos_deg[APP_NUM_MOTORS];     /* relative to zero */
	float speed_rpm[APP_NUM_MOTORS];
	float torque_nm[APP_NUM_MOTORS];
	float temp_c[APP_NUM_MOTORS];
	uint8_t online[APP_NUM_MOTORS];
} LiveView;
extern volatile LiveView g_live;

void mc_init(void);
void mc_tick(void);

/* host commands; return ST_* */
int mc_cmd_pos(uint8_t mask, float target, float vel_lim, float cur_lim);
int mc_cmd_vel(uint8_t mask, float vel, float cur_lim, float acc);
int mc_cmd_mit(uint8_t mask, float p, float v, float kp, float kd, float t);
int mc_disable(uint8_t mask, int clear_fault);
void mc_estop(void);
int mc_any_active(void);
int mc_is_idle(int i);

/* used by calib.c */
void mc_enter_run_mode(int i, uint8_t run_mode);
void mc_set_mode(int i, uint8_t mode);
void mc_hold_position(int i, float target, float vel_lim, float cur_lim);
void mc_safety_stop(int i, uint8_t event_code, float value);

#endif
