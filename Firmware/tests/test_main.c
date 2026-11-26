/* End-to-end test: real App/ firmware + simulated MCP2515s + simulated motors.
 * A fake "host" talks to it through the USB FIFOs with the binary protocol. */
#include "sim.h"
#include "app.h"
#include "usb_link.h"
#include "link.h"
#include "protocol.h"
#include "motor_ctrl.h"
#include "robstride.h"
#include "can_bus.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int fails;
#define CHECK(c, ...) do { if (c) { printf("  ok   "); } else { printf("  FAIL "); fails++; } printf(__VA_ARGS__); printf("\n"); } while (0)

/* ---------------- host side */
static struct link_parser hp;
static uint32_t rd_pos, last_ms, last_hb;
static int hb_on = 1;
static telem_t T;
static health_t H;
static int n_telem, n_health, pong_ok;
static int8_t ack[256];
static int ack_seen[256];
static struct { uint8_t m, code; float v; } ev[256];
static int n_ev;
static uint8_t scan_ids[2][16], scan_n[2], scan_st[2];
static int scan_seen;

static void host_send(uint8_t type, const void *p, uint8_t len)
{
	uint8_t f[200];
	size_t n = link_encode(type, p, len, f, sizeof(f));
	usb_link_rx_isr(f, (uint32_t)n);
	ack_seen[type] = 0;
}

static void pump_host(void)
{
	while (rd_pos < g_sim_usb_len) {
		if (!link_parser_feed(&hp, g_sim_usb_out[rd_pos++])) continue;
		uint8_t *p = hp.buf;
		switch (hp.type) {
		case EVT_TELEM: memcpy(&T, p, sizeof(T)); n_telem++; break;
		case EVT_HEALTH: memcpy(&H, p, sizeof(H)); n_health++; break;
		case EVT_PONG: { pong_t g; memcpy(&g, p, sizeof(g)); pong_ok = g.n_motors == 4 && g.proto == 2; break; }
		case EVT_ACK: ack[p[0]] = (int8_t)p[1]; ack_seen[p[0]] = 1; break;
		case EVT_EVENT: if (n_ev < 256) { ev[n_ev].m = p[0]; ev[n_ev].code = p[1]; memcpy(&ev[n_ev].v, &p[2], 4); n_ev++; } break;
		case EVT_SCAN: scan_st[p[0]] = p[1]; scan_n[p[0]] = p[2]; memcpy(scan_ids[p[0]], &p[3], p[2]); scan_seen++; break;
		}
	}
}

static void run_ms(uint32_t ms)
{
	uint64_t end = sim_now_us() + (uint64_t)ms * 1000u;
	while (sim_now_us() < end) {
		uint32_t now = HAL_GetTick();
		while (last_ms < now) { last_ms++; app_tick_isr(); }
		if (hb_on && now - last_hb >= 40) { last_hb = now; host_send(CMD_HEARTBEAT, NULL, 0); }
		app_loop();
		sim_advance_us(20);
		pump_host();
	}
}

static int count_ev(uint8_t code, int motor)
{
	int n = 0;
	for (int k = 0; k < n_ev; k++) if (ev[k].code == code && (motor < 0 || ev[k].m == motor)) n++;
	return n;
}

static void cmd_f(uint8_t type, uint8_t mask, int nf, const float *f)
{
	uint8_t p[32]; p[0] = mask; memcpy(&p[1], f, 4 * nf);
	host_send(type, p, (uint8_t)(1 + 4 * nf));
}

static void calib(uint8_t op, uint8_t mask, float a, float b, float c, float d)
{
	uint8_t p[18]; float f[4] = {a, b, c, d};
	p[0] = op; p[1] = mask; memcpy(&p[2], f, 16);
	host_send(CMD_CALIB, p, 18);
}

int main(void)
{
	/* 2 motors per bus, IDs 1 and 2, hard stops at +-1.5 rad (raw) */
	g_sim_nmotors = 4;
	for (int k = 0; k < 4; k++) {
		g_sim_motor[k] = (SimMotor){.bus = k / 2, .present = 1, .id = (uint8_t)(1 + k % 2),
			.raw_pos = 0.3f * (float)k - 0.4f, .temp = 31.0f, .stop_lo = -1.5f, .stop_hi = 1.5f, .limit_spd = 1.0f};
	}
	link_parser_init(&hp);
	app_init();
	run_ms(100);

	printf("== boot\n");
	CHECK(g_can[0].mcp.ok && g_can[1].mcp.ok, "both MCP2515 initialised (err %d / %d)", g_can[0].init_err, g_can[1].init_err);
	host_send(CMD_PING, NULL, 0); run_ms(20);
	CHECK(pong_ok, "PONG: 4 motors, protocol 2");
	CHECK(n_telem > 5, "telemetry streaming (%d frames in 120 ms)", n_telem);
	CHECK(T.m[3].flags & TF_ONLINE, "motor 4 online while idle (STOP polling gives feedback)");

	printf("== identification scan\n");
	uint8_t bm = 3; host_send(CMD_SCAN, &bm, 1); run_ms(400);
	CHECK(scan_seen == 2 && scan_n[0] == 2 && scan_ids[0][0] == 1 && scan_ids[0][1] == 2, "CAN1 ids found: n=%d [%d %d]", scan_n[0], scan_ids[0][0], scan_ids[0][1]);
	CHECK(scan_n[1] == 2 && scan_st[1] == 0, "CAN2 ids found: n=%d", scan_n[1]);

	printf("== position (CSP) all 4 motors -> 0.5 rad @ 1 rad/s\n");
	float pf[3] = {0.5f, 1.0f, 3.0f}; cmd_f(CMD_POS, 0x0F, 3, pf); run_ms(2000);
	CHECK(ack_seen[CMD_POS] && ack[CMD_POS] == 0, "POS acked");
	int ok = 1; for (int i = 0; i < 4; i++) ok &= fabsf(T.m[i].pos - 0.5f) < 0.02f && T.m[i].mode == MODE_POS;
	CHECK(ok, "all reached 0.5 rad: %.3f %.3f %.3f %.3f", T.m[0].pos, T.m[1].pos, T.m[2].pos, T.m[3].pos);
	CHECK(fabsf(g_live.pos_deg[2] - 28.65f) < 1.5f, "live expression g_live.pos_deg[2] = %.2f deg", g_live.pos_deg[2]);

	printf("== velocity M1 2 rad/s (soft limit at 2.0 rad must stop it... stop at 1.5 first)\n");
	float vf[3] = {2.0f, 3.0f, 10.0f}; cmd_f(CMD_VEL, 0x01, 3, vf); run_ms(200);
	CHECK(fabsf(T.m[0].vel - 2.0f) < 0.05f && T.m[0].mode == MODE_VEL, "M1 vel %.2f rad/s, mode VEL", T.m[0].vel);
	uint8_t dis[2] = {0x0F, 0}; host_send(CMD_DISABLE, dis, 2); run_ms(100);
	CHECK(T.m[0].mode == MODE_IDLE && T.m[3].mode == MODE_IDLE, "all disabled");

	printf("== MIT M2\n");
	float mf[5] = {0.0f, 0.0f, 5.0f, 0.3f, 0.1f}; cmd_f(CMD_MIT, 0x02, 5, mf); run_ms(100);
	CHECK(T.m[1].mode == MODE_MIT && fabsf(T.m[1].setpoint - 0.1f) < 1e-4f, "M2 in MIT, tau_ff echoed");
	host_send(CMD_DISABLE, dis, 2); run_ms(50);

	printf("== zero here (all)\n");
	calib(CAL_ZERO_HERE, 0x0F, 0, 0, 0, 0); run_ms(400);
	CHECK(count_ev(EV_ZERO_DONE, -1) == 4, "ZERO_DONE x%d", count_ev(EV_ZERO_DONE, -1));
	ok = 1; for (int i = 0; i < 4; i++) ok &= fabsf(T.m[i].pos) < 0.01f;
	CHECK(ok, "positions now ~0: %.3f %.3f %.3f %.3f", T.m[0].pos, T.m[1].pos, T.m[2].pos, T.m[3].pos);

	printf("== homing M3 to the negative hard stop\n");
	calib(CAL_HOME, 0x04, -1.0f, 0.5f, 1.5f, 0.2f); run_ms(9000);
	CHECK(count_ev(EV_HOME_STALL, 2) == 1, "HOME_STALL seen");
	CHECK(count_ev(EV_HOME_DONE, 2) == 1, "HOME_DONE seen");
	CHECK(T.m[2].mode == MODE_POS && fabsf(T.m[2].pos) < 0.02f, "M3 holds new zero (pos %.3f, mode %d)", T.m[2].pos, T.m[2].mode);
	CHECK(fabsf(g_sim_motor[2].raw_pos - (-1.5f + 0.2f)) < 0.03f, "physical zero = stop + backoff (raw %.3f)", g_sim_motor[2].raw_pos);
	host_send(CMD_DISABLE, dis, 2); run_ms(50);

	printf("== change CAN ID of M2 (bus 1) 2 -> 5\n");
	calib(CAL_SET_CAN_ID, 0x02, 5, 0, 0, 0); run_ms(300);
	CHECK(count_ev(EV_ID_CHANGED, 1) == 1 && g_motor[1].can_id == 5 && g_sim_motor[1].id == 5, "ID changed, table + motor = 5");
	CHECK(T.m[1].flags & TF_ONLINE, "M2 still online under its new ID");
	calib(CAL_SET_CAN_ID, 0x02, 1, 0, 0, 0); run_ms(50);
	CHECK(ack[CMD_CALIB] == ST_BAD_ARG, "refuses ID already used by M1 on the same bus");

	printf("== watchdog\n");
	cmd_f(CMD_POS, 0x01, 3, pf); run_ms(200);
	hb_on = 0; run_ms(800);
	CHECK(count_ev(EV_WATCHDOG, 0) == 1 && T.m[0].mode == MODE_IDLE, "no heartbeat -> WATCHDOG stop");
	hb_on = 1;

	printf("== bus without motors\n");
	g_sim_motor[2].present = g_sim_motor[3].present = 0;
	bm = 2; scan_seen = 0; host_send(CMD_SCAN, &bm, 1); run_ms(400);
	CHECK(scan_seen == 1 && scan_st[1] == 1, "CAN2 scan reports bus error (nobody ACKs)");
	CHECK(H.bus[1].tx_fail > 100, "health: CAN2 tx_fail counted (%u)", (unsigned)H.bus[1].tx_fail);

	printf("== health / timing\n");
	run_ms(300);
	CHECK(n_health > 10, "health frames: %d", n_health);
	CHECK(H.bus[0].rx_ovf == 0 && g_sim_rx_overflow[0] == 0, "CAN1 no RX overflow (rx %u)", (unsigned)H.bus[0].rx_count);
	CHECK(fabsf(H.vbus[0] - 48.2f) < 0.01f, "VBUS read %.1f V", H.vbus[0]);
	CHECK(g_app.ctrl_overruns < 50, "control overruns %u (scan blocks briefly)", (unsigned)g_app.ctrl_overruns);
	printf("  info worst mc_tick %u us (simulated SPI timing)\n", (unsigned)g_app.ctrl_max_us);

	printf("\n%s (%d failures)\n", fails ? "SOME TESTS FAILED" : "ALL TESTS PASSED", fails);
	return fails ? 1 : 0;
}
