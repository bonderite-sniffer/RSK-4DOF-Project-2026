/* command.h - decode binary commands from Python / MATLAB (protocol.h) and
 * execute them. Runs in the main loop, never in an interrupt. */
#ifndef COMMAND_H
#define COMMAND_H

#include <stdint.h>

void command_init(void);
void command_poll(void);   /* read USB RX FIFO, parse frames, execute */

/* exposed for unit tests: execute one already-validated frame */
void command_execute(uint8_t type, const uint8_t *payload, uint8_t len);

#endif
