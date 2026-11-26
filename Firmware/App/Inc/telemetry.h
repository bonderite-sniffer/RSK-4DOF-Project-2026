/* telemetry.h - everything the STM32 sends to the laptop:
 * periodic telemetry (pos/vel/torque/temp/iq per motor), CAN health,
 * events, acks and parameter replies. */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

void tm_tick(uint32_t ctrl_tick);   /* called by mc_tick() at APP_CTRL_HZ */
void tm_send_health(void);
void tm_send_ack(uint8_t cmd, int8_t status);
void tm_send_event(uint8_t motor, uint8_t code, float value);
void tm_send_param(uint8_t motor, uint16_t index, uint8_t ok, uint32_t raw);
void tm_send_pong(void);

#endif
