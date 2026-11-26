#ifndef SIM_H
#define SIM_H
#include <stdint.h>

#define SIM_MAX_MOTORS 8

typedef struct {
	int bus, present;
	uint8_t id;
	float raw_pos, vel, zero, temp;
	float spd_ref, loc_ref, limit_spd;
	float stop_lo, stop_hi;        /* hard stops (rad, raw) */
	uint8_t run_mode, enabled, fault;
	uint64_t last_us;
	uint32_t rx_frames;
} SimMotor;

extern SimMotor g_sim_motor[SIM_MAX_MOTORS];
extern int g_sim_nmotors;
extern uint8_t g_sim_usb_out[1 << 20];
extern uint32_t g_sim_usb_len;
extern uint32_t g_sim_rx_overflow[2];

uint64_t sim_now_us(void);
void sim_advance_us(uint32_t us);

#endif
