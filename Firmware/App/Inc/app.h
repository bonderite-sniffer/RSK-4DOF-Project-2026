/* app.h - application entry points called from CubeMX main.c.
 *
 *   main.c  USER CODE BEGIN Includes :  #include "app.h"
 *   main.c  USER CODE BEGIN 2        :  app_init();
 *   main.c  USER CODE BEGIN 3        :  app_loop();      (inside while (1))
 *
 * Architecture (bare metal, HAL):
 *   TIM6 IRQ @ 1 kHz   -> only sets a flag (app_tick_isr)
 *   USB IRQ            -> only copies received bytes into a FIFO
 *   main loop          -> everything else: CAN RX polling, commands,
 *                         control tick @ APP_CTRL_HZ, telemetry, USB TX
 * Later (FreeRTOS): each block of app_loop() becomes a task.
 */
#ifndef APP_H
#define APP_H

#include <stdint.h>

typedef struct {
	uint32_t loop_count;      /* main-loop passes per second */
	uint32_t ctrl_max_us;     /* worst mc_tick() duration */
	uint32_t ctrl_overruns;   /* control tick started late */
	int can_init_err[2];      /* 0 ok, see mcp2515_init() */
} AppStatus;

extern volatile AppStatus g_app;   /* add to Live Expressions too */

void app_init(void);
void app_loop(void);
void app_tick_isr(void);           /* called from HAL_TIM_PeriodElapsedCallback */

#endif
