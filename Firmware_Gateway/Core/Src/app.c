#include "app.h"
#include "app_config.h"
#include "can_bus.h"
#include "robstride.h"
#include "motor_ctrl.h"
#include "command.h"
#include "usb_link.h"
#include "telemetry.h"
#include "mcp2515_diag.h"
#include "gateway.h"

volatile AppStatus g_app;

static volatile uint32_t s_tick_count;   /* incremented by TIM6 ISR */
static uint32_t s_tick_done;
static uint32_t s_loops, s_loop_t0, s_health_t0;

void app_init(void)
{
	app_dwt_init();
	usb_link_init();
	command_init();
	rs_init();
	mcp2515_diag_run();     /* self test -> g_diag[0..1] in Live Expressions */
	can_bus_init_all();
	g_app.can_init_err[0] = g_can[0].init_err;
	g_app.can_init_err[1] = g_can[1].init_err;
	mc_init();
	gw_init();

	s_loop_t0 = HAL_GetTick();
	HAL_TIM_Base_Start_IT(APP_TICK_TIM);
}

void app_tick_isr(void)
{
	s_tick_count++;
}

void app_loop(void)
{
	/* 1. CAN receive: drain both MCP2515s (cheap when INT pin is high) */
	can_bus_poll(0);
	can_bus_poll(1);

	/* 2. commands from Python / MATLAB */
	command_poll();

	/* 2b. gateway: host frames -> MCP2515 (in order), RX batch -> USB, watchdog */
	gw_poll();

	/* 3. control tick at APP_CTRL_HZ */
	uint32_t ticks = s_tick_count;
	if (ticks - s_tick_done >= (APP_TICK_HZ / APP_CTRL_HZ)) {
		if (ticks - s_tick_done >= 2u * (APP_TICK_HZ / APP_CTRL_HZ)) {
			g_app.ctrl_overruns++;
		}
		s_tick_done = ticks - (ticks % (APP_TICK_HZ / APP_CTRL_HZ));

		uint32_t t0 = app_micros();
		mc_tick();   /* also emits telemetry */
		uint32_t dt = app_micros() - t0;
		if (dt > g_app.ctrl_max_us) {
			g_app.ctrl_max_us = dt;
		}
	}

	/* 4. CAN health counters at 10 Hz */
	uint32_t now = HAL_GetTick();
	if (now - s_health_t0 >= 100u) {
		s_health_t0 = now;
		can_bus_health_update();
	}

	/* 5. push queued frames to USB */
	usb_link_poll();

	/* loop statistics */
	s_loops++;
	if (now - s_loop_t0 >= 1000u) {
		g_app.loop_count = s_loops;
		s_loops = 0;
		s_loop_t0 = now;
	}
}

/* HAL weak callback. NOTE: if you later switch the HAL timebase to a timer
 * (needed with FreeRTOS), CubeMX puts this function into main.c - then delete
 * it here and call app_tick_isr() from the main.c version instead. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == APP_TICK_TIM) {
		app_tick_isr();
	}
}
