/* calib.h - zeroing / calibration of the motors, triggered by CMD_CALIB from
 * Python / MATLAB.
 *
 *   CAL_ZERO_HERE  : current position becomes 0 (type 6), then verified
 *   CAL_HOME       : drive slowly into a hard stop, zero there, back off,
 *                    zero again at the back-off point, hold position 0
 *   CAL_SET_CAN_ID : change a motor's CAN ID (one motor at a time!)
 *   CAL_SAVE       : ask the motor to store its parameters in flash
 */
#ifndef CALIB_H
#define CALIB_H

#include <stdint.h>

/* returns ST_* (protocol.h). a..d meaning depends on op. */
int calib_start(uint8_t op, uint8_t mask, float a, float b, float c, float d);

/* called by motor_ctrl at APP_CTRL_HZ for motors in MODE_CALIB */
void calib_step(int i);

/* stop any running calibration on motor i (E-STOP, disable, safety) */
void calib_abort(int i);

#endif
