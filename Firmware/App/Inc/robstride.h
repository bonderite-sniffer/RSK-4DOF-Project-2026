/* robstride.h - RobStride private CAN protocol (EduLite 05) + motor table.
 *
 * Extended 29-bit ID:  bits 28..24 type | 23..8 data area 2 | 7..0 target ID
 * Source: RobStride EduLite 05 instruction manual (private protocol).
 * Items marked VERIFY are taken from other RobStride manuals (RS02/RS03) and
 * should be checked against your firmware before relying on them.
 */
#ifndef ROBSTRIDE_H
#define ROBSTRIDE_H

#include "app_config.h"
#include "mcp2515.h"
#include <stdint.h>

/* communication types */
enum {
	RS_T_GET_ID = 0,
	RS_T_MIT = 1,          /* p, v, kp, kd in data, torque in ID */
	RS_T_FEEDBACK = 2,
	RS_T_ENABLE = 3,
	RS_T_STOP = 4,         /* data[0] = 1 also clears faults */
	RS_T_SET_ZERO = 6,     /* data[0] = 1 */
	RS_T_SET_ID = 7,       /* VERIFY: new ID in ID bits 23..16 */
	RS_T_READ = 17,
	RS_T_WRITE = 18,       /* RAM only */
	RS_T_FAULT = 21,
	RS_T_SAVE = 22,        /* VERIFY: store params to flash */
};

/* run_mode (param 0x7005) */
enum { RS_MODE_MIT = 0, RS_MODE_PP = 1, RS_MODE_VEL = 2, RS_MODE_CSP = 5 };

/* parameter indices */
#define RS_IDX_RUN_MODE    0x7005
#define RS_IDX_IQ_REF      0x7006
#define RS_IDX_SPD_REF     0x700A
#define RS_IDX_LIMIT_TORQ  0x700B
#define RS_IDX_LOC_REF     0x7016
#define RS_IDX_LIMIT_SPD   0x7017
#define RS_IDX_LIMIT_CUR   0x7018
#define RS_IDX_MECH_POS    0x7019
#define RS_IDX_IQF         0x701A
#define RS_IDX_MECH_VEL    0x701B
#define RS_IDX_VBUS        0x701C
#define RS_IDX_ACC_RAD     0x7022
#define RS_IDX_ZERO_STA    0x7029

/* scaling */
#define RS_P_MAX     12.566370614f   /* +-4 pi rad */
#define RS_V_MAX     50.0f           /* rad/s      */
#define RS_KP_MAX    500.0f
#define RS_KD_MAX    5.0f
#define RS_T_MAX     5.5f            /* RS05 torque range (type-1 command and type-2 feedback) */

/* One motor. Everything the motor reports lands here; this is also what you
 * watch in STM32CubeIDE "Live Expressions" (g_motor[0].torque, ...). */
typedef struct {
	uint8_t bus;          /* 0 = CAN1, 1 = CAN2 */
	uint8_t can_id;
	/* feedback (type 2) */
	float pos;            /* rad, relative to the motor's zero */
	float vel;            /* rad/s */
	float torque;         /* Nm */
	float temp;           /* degC */
	uint8_t rs_mode;      /* 0 reset, 1 cali, 2 run */
	uint8_t fault;        /* 6 fault bits */
	/* parameter reads */
	float iq;             /* A  (0x701A) */
	float vbus;           /* V  (0x701C) */
	uint32_t last_fb_ms;
	uint32_t fb_count;
	uint32_t fault_word, warn_word;   /* type 21 */
	int32_t pending_param;            /* host-requested read, -1 none */
} RsMotor;

extern volatile RsMotor g_motor[APP_NUM_MOTORS];
extern float rs_t_cmd_max;   /* type-1 torque field range +-Nm, default RS_T_MAX (CFG_TSCALE) */
extern uint8_t rs_host_id;

void rs_init(void);

/* frame builders (pure, unit-tested) */
uint16_t rs_f2u(float x, float xmin, float xmax);
float rs_u2f(uint16_t u, float xmin, float xmax);
void rs_make_frame(CanFrame *f, uint8_t type, uint16_t data2, uint8_t target);
void rs_make_mit(CanFrame *f, uint8_t target, float p, float v, float kp, float kd, float t, float tmax);
void rs_make_read(CanFrame *f, uint8_t host, uint8_t target, uint16_t index);
void rs_make_write(CanFrame *f, uint8_t host, uint8_t target, uint16_t index, uint32_t raw);

/* motor commands (index 0..3). ordered = wait for previous frames to leave. */
int rs_enable(int i, int ordered);
int rs_stop(int i, int clear_fault, int ordered);
int rs_set_zero(int i);
int rs_write_u32(int i, uint16_t index, uint32_t v, int ordered);
int rs_write_f32(int i, uint16_t index, float v, int ordered);
int rs_read(int i, uint16_t index);
int rs_mit(int i, float p, float v, float kp, float kd, float t);
int rs_get_id(int i);
int rs_set_can_id(int i, uint8_t new_id);
int rs_save(int i);

/* decode one received frame from bus (called by can_bus_poll) */
void rs_on_frame(uint8_t bus, const CanFrame *f);

/* index of the motor with (bus, can_id), or -1 */
int rs_find(uint8_t bus, uint8_t can_id);

#endif
