/* mcp2515_diag.h - step-by-step SPI / chip self test for both MCP2515s.
 * Runs once in app_init() BEFORE can_bus_init_all(). No motor / CAN bus needed:
 * the last step uses the MCP2515 internal LOOPBACK mode.
 *
 * Watch in Live Expressions:  g_diag   (g_diag[0] = CAN1/SPI1, g_diag[1] = CAN2/SPI2)
 *
 * verdict:
 *   1 OK                - SPI, registers and oscillator all fine
 *   2 MISO_HIGH         - every read = 0xFF: chip not driving SO (no power, CS wrong/not toggling,
 *                         SO not connected, chip held in reset)
 *   3 MISO_LOW          - every read = 0x00: SO shorted to GND or wrong pin, chip dead
 *   4 BAD_RESET_STATE   - chip answers but CANSTAT/CANCTRL after reset are wrong
 *                         (SPI mode/data size wrong -> look at canstat/canctrl values)
 *   5 WRITE_MISMATCH    - reads OK, writes corrupted: MOSI(SI)/SCK problem, 5 V module
 *                         (3.3 V signals too low), speed too high, CPHA wrong
 *   6 LOOPBACK_FAIL     - registers fine but no frame in loopback: crystal / oscillator
 *                         not running (bad solder after crystal swap?) or wrong osc-freq
 *   7 HAL_ERROR         - HAL_SPI_TransmitReceive itself failed (SPI not initialised,
 *                         timeout, wrong handle)
 *   8 BURST_FAIL        - 1-byte register access OK, but multi-byte (burst) transfers are
 *                         corrupted: compare bw/br (3 bytes) and lw/lr (13 bytes).
 *                         -> SPI GPIO speed, wire length, FIFO/Data Size settings
 */
#ifndef MCP2515_DIAG_H
#define MCP2515_DIAG_H

#include <stdint.h>

typedef struct {
	uint8_t verdict;          /* see table above */
	uint8_t canstat_reset;    /* expect 0x80 */
	uint8_t canctrl_reset;    /* expect 0x87 */
	uint8_t wr[4];            /* patterns written to CNF1 */
	uint8_t rd[4];            /* what came back */
	uint8_t loop_mode;        /* CANSTAT after requesting loopback, expect 0x40 */
	uint8_t loop_intf;        /* CANINTF after loopback TX, expect bit0 (RX0IF) set */
	uint8_t loop_ok;          /* 1 = own frame received back */
	uint32_t hal_errors;      /* HAL SPI errors during the test */
	uint8_t cs_readback;      /* level of CS pin read back while deselected, expect 1 */
	uint8_t bw[3], br[3];     /* burst: 3 bytes written to CNF3..CNF1 in ONE transfer, read back in one */
	uint8_t lw[13], lr[13];   /* long burst: 13 bytes into TXB0 registers, read back in one */
	uint8_t burst_ok;         /* 1 = both bursts read back identical */
} McpDiag;

extern volatile McpDiag g_diag[2];

void mcp2515_diag_run(void);

#endif
