#include "robstride.h"
#include "can_bus.h"
#include "telemetry.h"
#include <string.h>

volatile RsMotor g_motor[APP_NUM_MOTORS];
float rs_t_cmd_max = RS_T_MAX;  /* RS05 manual: torque field maps -5.5..+5.5 Nm */
uint8_t rs_host_id = APP_HOST_ID;

/* ------------------------------------------------------------ pure helpers */
static void put_be16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v >> 8);
	p[1] = (uint8_t)v;
}

static uint16_t get_be16(const uint8_t *p)
{
	return (uint16_t)((p[0] << 8) | p[1]);
}

static uint32_t get_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static float raw2f(uint32_t r)
{
	float f;

	memcpy(&f, &r, 4);
	return f;
}

uint16_t rs_f2u(float x, float xmin, float xmax)
{
	if (x < xmin) {
		x = xmin;
	}
	if (x > xmax) {
		x = xmax;
	}
	return (uint16_t)((x - xmin) * 65535.0f / (xmax - xmin) + 0.5f);
}

float rs_u2f(uint16_t u, float xmin, float xmax)
{
	return (float)u * (xmax - xmin) / 65535.0f + xmin;
}

void rs_make_frame(CanFrame *f, uint8_t type, uint16_t data2, uint8_t target)
{
	memset(f, 0, sizeof(*f));
	f->id = ((uint32_t)(type & 0x1F) << 24) | ((uint32_t)data2 << 8) | target;
	f->ext = 1;
	f->dlc = 8;
}

void rs_make_mit(CanFrame *f, uint8_t target, float p, float v, float kp, float kd, float t, float tmax)
{
	rs_make_frame(f, RS_T_MIT, rs_f2u(t, -tmax, tmax), target);
	put_be16(&f->data[0], rs_f2u(p, -RS_P_MAX, RS_P_MAX));
	put_be16(&f->data[2], rs_f2u(v, -RS_V_MAX, RS_V_MAX));
	put_be16(&f->data[4], rs_f2u(kp, 0.0f, RS_KP_MAX));
	put_be16(&f->data[6], rs_f2u(kd, 0.0f, RS_KD_MAX));
}

void rs_make_read(CanFrame *f, uint8_t host, uint8_t target, uint16_t index)
{
	rs_make_frame(f, RS_T_READ, host, target);
	f->data[0] = (uint8_t)index;          /* index low byte first */
	f->data[1] = (uint8_t)(index >> 8);
}

void rs_make_write(CanFrame *f, uint8_t host, uint8_t target, uint16_t index, uint32_t raw)
{
	rs_make_frame(f, RS_T_WRITE, host, target);
	f->data[0] = (uint8_t)index;
	f->data[1] = (uint8_t)(index >> 8);
	f->data[4] = (uint8_t)raw;            /* value low byte first; u8 params use data[4] */
	f->data[5] = (uint8_t)(raw >> 8);
	f->data[6] = (uint8_t)(raw >> 16);
	f->data[7] = (uint8_t)(raw >> 24);
}

/* ------------------------------------------------------------ table */
void rs_init(void)
{
	const uint8_t bus[APP_NUM_MOTORS] = APP_MOTOR_BUS;
	const uint8_t ids[APP_NUM_MOTORS] = APP_MOTOR_IDS;

	memset((void *)g_motor, 0, sizeof(g_motor));
	for (int i = 0; i < APP_NUM_MOTORS; i++) {
		g_motor[i].bus = bus[i];
		g_motor[i].can_id = ids[i];
		g_motor[i].pending_param = -1;
	}
}

int rs_find(uint8_t bus, uint8_t can_id)
{
	for (int i = 0; i < APP_NUM_MOTORS; i++) {
		if (g_motor[i].bus == bus && g_motor[i].can_id == can_id) {
			return i;
		}
	}
	return -1;
}

/* ------------------------------------------------------------ commands */
static int tx(int i, const CanFrame *f, int ordered)
{
	uint8_t bus = g_motor[i].bus;

	return ordered ? can_bus_send_ordered(bus, f) : can_bus_send(bus, f);
}

static int simple(int i, uint8_t type, uint8_t b0, int ordered)
{
	CanFrame f;

	rs_make_frame(&f, type, rs_host_id, g_motor[i].can_id);
	f.data[0] = b0;
	return tx(i, &f, ordered);
}

int rs_enable(int i, int ordered)
{
	return simple(i, RS_T_ENABLE, 0, ordered);
}

int rs_stop(int i, int clear_fault, int ordered)
{
	return simple(i, RS_T_STOP, clear_fault ? 1 : 0, ordered);
}

int rs_set_zero(int i)
{
	return simple(i, RS_T_SET_ZERO, 1, 1);
}

int rs_get_id(int i)
{
	return simple(i, RS_T_GET_ID, 0, 1);
}

int rs_write_u32(int i, uint16_t index, uint32_t v, int ordered)
{
	CanFrame f;

	rs_make_write(&f, rs_host_id, g_motor[i].can_id, index, v);
	return tx(i, &f, ordered);
}

int rs_write_f32(int i, uint16_t index, float v, int ordered)
{
	uint32_t raw;

	memcpy(&raw, &v, 4);
	return rs_write_u32(i, index, raw, ordered);
}

int rs_read(int i, uint16_t index)
{
	CanFrame f;

	rs_make_read(&f, rs_host_id, g_motor[i].can_id, index);
	return tx(i, &f, 0);
}

int rs_mit(int i, float p, float v, float kp, float kd, float t)
{
	CanFrame f;

	rs_make_mit(&f, g_motor[i].can_id, p, v, kp, kd, t, rs_t_cmd_max);
	return tx(i, &f, 0);
}

int rs_set_can_id(int i, uint8_t new_id)
{
	/* VERIFY: type 7, data area 2 = (new_id << 8) | host  ->  ID bits 23..16 new, 15..8 host */
	CanFrame f;

	rs_make_frame(&f, RS_T_SET_ID, (uint16_t)(((uint16_t)new_id << 8) | rs_host_id), g_motor[i].can_id);
	return tx(i, &f, 1);
}

int rs_save(int i)
{
	/* VERIFY: type 22 with data 01..08 */
	CanFrame f;

	rs_make_frame(&f, RS_T_SAVE, rs_host_id, g_motor[i].can_id);
	for (int k = 0; k < 8; k++) {
		f.data[k] = (uint8_t)(k + 1);
	}
	return tx(i, &f, 1);
}

/* ------------------------------------------------------------ decode */
void rs_on_frame(uint8_t bus, const CanFrame *f)
{
	uint8_t type = (uint8_t)((f->id >> 24) & 0x1F);
	uint8_t motor_id = (uint8_t)((f->id >> 8) & 0xFF);

	if (type == RS_T_GET_ID) {
		can_bus_mark_seen(bus, motor_id);
		return;
	}

	int i = rs_find(bus, motor_id);
	if (i < 0) {
		return;   /* frame from a motor that is not in our table */
	}
	volatile RsMotor *m = &g_motor[i];

	switch (type) {
	case RS_T_FEEDBACK:
		if (f->dlc < 8) {
			return;
		}
		m->rs_mode = (uint8_t)((f->id >> 22) & 0x03);
		m->fault = (uint8_t)((f->id >> 16) & 0x3F);
		m->pos = rs_u2f(get_be16(&f->data[0]), -RS_P_MAX, RS_P_MAX);
		m->vel = rs_u2f(get_be16(&f->data[2]), -RS_V_MAX, RS_V_MAX);
		m->torque = rs_u2f(get_be16(&f->data[4]), -RS_T_MAX, RS_T_MAX);
		m->temp = (float)get_be16(&f->data[6]) / 10.0f;
		m->last_fb_ms = HAL_GetTick();
		m->fb_count++;
		break;

	case RS_T_READ: {
		uint16_t index = (uint16_t)(f->data[0] | (f->data[1] << 8));
		uint8_t ok = (f->data[2] == 0);
		uint32_t raw = get_le32(&f->data[4]);

		if (ok && index == RS_IDX_IQF) {
			m->iq = raw2f(raw);
		} else if (ok && index == RS_IDX_VBUS) {
			m->vbus = raw2f(raw);
		}
		if (m->pending_param == (int32_t)index) {
			m->pending_param = -1;
			tm_send_param((uint8_t)i, index, ok, raw);
		}
		break;
	}

	case RS_T_FAULT:
		m->fault_word = get_le32(&f->data[0]);
		m->warn_word = get_le32(&f->data[4]);
		break;

	default:
		break;
	}
}
