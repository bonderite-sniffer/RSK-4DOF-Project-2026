#include "mcp2515.h"
#include <string.h>

/* SPI instructions */
#define CMD_RESET        0xC0
#define CMD_READ         0x03
#define CMD_WRITE        0x02
#define CMD_BITMOD       0x05
#define CMD_READ_STATUS  0xA0
#define CMD_RX_STATUS    0xB0
#define CMD_READ_RXB0    0x90   /* starts at RXB0SIDH, clears RX0IF on CS high */
#define CMD_READ_RXB1    0x94
#define CMD_LOAD_TXB(n)  (uint8_t)(0x40 | ((n) << 1))  /* starts at TXBnSIDH */
#define CMD_RTS(n)       (uint8_t)(0x80 | (1u << (n)))

/* registers */
#define REG_CANSTAT   0x0E
#define REG_CANCTRL   0x0F
#define REG_TEC       0x1C
#define REG_CNF3      0x28
#define REG_CANINTE   0x2B
#define REG_CANINTF   0x2C
#define REG_EFLG      0x2D
#define REG_TXBCTRL(n) (uint8_t)(0x30 + 0x10 * (n))
#define REG_RXB0CTRL  0x60
#define REG_RXB1CTRL  0x70

#define MODE_MASK     0xE0
#define MODE_NORMAL   0x00
#define MODE_CONFIG   0x80
#define CANCTRL_OSM   0x08      /* one-shot mode */

#define INTF_RX0IF    0x01
#define INTF_RX1IF    0x02
#define INTF_ERRIF    0x20
#define INTF_MERRF    0x80

#define EFLG_TXBO     0x20
#define EFLG_RX0OVR   0x40
#define EFLG_RX1OVR   0x80

#define TXB_ABTF      0x40
#define TXB_MLOA      0x20
#define TXB_TXERR     0x10
#define TXB_TXREQ     0x08

/* ------------------------------------------------------------ low level */
static int rd(Mcp2515 *m, uint8_t reg, uint8_t *out, uint8_t n)
{
	uint8_t tx[16] = {CMD_READ, reg};
	uint8_t rx[16];

	if (spi_xfer(m->spi, tx, rx, (uint16_t)(n + 2)) != 0) {
		return -1;
	}
	memcpy(out, &rx[2], n);
	return 0;
}

static int wr(Mcp2515 *m, uint8_t reg, const uint8_t *d, uint8_t n)
{
	uint8_t tx[16] = {CMD_WRITE, reg};

	memcpy(&tx[2], d, n);
	return spi_xfer(m->spi, tx, NULL, (uint16_t)(n + 2));
}

static int wr1(Mcp2515 *m, uint8_t reg, uint8_t v)
{
	return wr(m, reg, &v, 1);
}

static int bitmod(Mcp2515 *m, uint8_t reg, uint8_t mask, uint8_t val)
{
	uint8_t tx[4] = {CMD_BITMOD, reg, mask, val};

	return spi_xfer(m->spi, tx, NULL, 4);
}

static int status(Mcp2515 *m, uint8_t cmd, uint8_t *out)
{
	uint8_t tx[2] = {cmd, 0};
	uint8_t rx[2];

	if (spi_xfer(m->spi, tx, rx, 2) != 0) {
		return -1;
	}
	*out = rx[1];
	return 0;
}

/* ------------------------------------------------------------ helpers (pure) */
void mcp2515_timing(uint8_t *cnf1, uint8_t *cnf2, uint8_t *cnf3)
{
	/* TQ = 2 * (BRP + 1) / Fosc. Bit = SyncSeg(1) + PropSeg + PS1 + PS2.
	 * CNF1 = (SJW-1)<<6 | BRP
	 * CNF2 = BTLMODE(0x80) | SAM<<6 | (PS1-1)<<3 | (PropSeg-1)
	 * CNF3 = (PS2-1)                                                    */
#if (MCP2515_OSC_HZ == 16000000u) && (MCP2515_BITRATE == 1000000u)
	/* BRP=0 -> TQ=125 ns, 8 TQ: 1 + 2 + 3 + 2, sample point 75 % */
	*cnf1 = 0x00;
	*cnf2 = (uint8_t)(0x80 | ((3 - 1) << 3) | (2 - 1));
	*cnf3 = (uint8_t)(2 - 1);
#elif (MCP2515_OSC_HZ == 20000000u) && (MCP2515_BITRATE == 1000000u)
	/* BRP=0 -> TQ=100 ns, 10 TQ: 1 + 3 + 4 + 2, sample point 80 % */
	*cnf1 = 0x00;
	*cnf2 = (uint8_t)(0x80 | ((4 - 1) << 3) | (3 - 1));
	*cnf3 = (uint8_t)(2 - 1);
#elif (MCP2515_OSC_HZ == 16000000u) && (MCP2515_BITRATE == 500000u)
	/* BRP=1 -> TQ=250 ns, 8 TQ, 75 % */
	*cnf1 = 0x01;
	*cnf2 = (uint8_t)(0x80 | ((3 - 1) << 3) | (2 - 1));
	*cnf3 = (uint8_t)(2 - 1);
#elif (MCP2515_OSC_HZ == 8000000u) && (MCP2515_BITRATE == 1000000u)
	/* OUT OF SPEC (experimental): BRP=0 -> TQ=250 ns, only 4 TQ: 1 + 1 + 1 + 1, sample point 75 %.
	 * Datasheet wants PS2 >= 2 TQ, so resync margin is tiny. Usually OK on a short bench bus
	 * (< 1-2 m, good 120 R termination); expect error frames on long/noisy cables.
	 * Same values as the Arduino mcp_can "8MHz 1000kBPS" table. Real fix: 16 MHz crystal. */
	*cnf1 = 0x00;                 /* SJW = 1 TQ, BRP = 0 */
	*cnf2 = 0x80;                 /* BTLMODE, PS1 = 1 TQ, PropSeg = 1 TQ */
	*cnf3 = 0x00;                 /* PS2 = 1 TQ */
#elif (MCP2515_OSC_HZ == 8000000u) && (MCP2515_BITRATE == 500000u)
	/* In spec: BRP=0 -> TQ=250 ns, 8 TQ: 1 + 2 + 3 + 2, sample point 75 % */
	*cnf1 = 0x00;
	*cnf2 = (uint8_t)(0x80 | ((3 - 1) << 3) | (2 - 1));
	*cnf3 = (uint8_t)(2 - 1);
#else
#error "Unsupported MCP2515_OSC_HZ / MCP2515_BITRATE combination"
#endif
}

void mcp2515_encode_id(uint32_t id, uint8_t ext, uint8_t out[4])
{
	if (ext) {
		out[0] = (uint8_t)(id >> 21);                                       /* SIDH: ID28..21 */
		out[1] = (uint8_t)(((id >> 13) & 0xE0) | 0x08 | ((id >> 16) & 0x03)); /* SIDL: ID20..18, EXIDE, ID17..16 */
		out[2] = (uint8_t)(id >> 8);                                        /* EID8 */
		out[3] = (uint8_t)id;                                               /* EID0 */
	} else {
		out[0] = (uint8_t)(id >> 3);
		out[1] = (uint8_t)((id & 0x07) << 5);
		out[2] = 0;
		out[3] = 0;
	}
}

void mcp2515_decode_id(const uint8_t in[4], uint32_t *id, uint8_t *ext)
{
	if (in[1] & 0x08) {
		*ext = 1;
		*id = ((uint32_t)in[0] << 21) | ((uint32_t)(in[1] & 0xE0) << 13) |
		      ((uint32_t)(in[1] & 0x03) << 16) | ((uint32_t)in[2] << 8) | in[3];
	} else {
		*ext = 0;
		*id = ((uint32_t)in[0] << 3) | (in[1] >> 5);
	}
}

/* ------------------------------------------------------------ API */
int mcp2515_init(Mcp2515 *m, SpiDev *spi, GPIO_TypeDef *int_port, uint16_t int_pin)
{
	uint8_t v, cnf[3], back[3] = {0};

	memset(m, 0, sizeof(*m));
	m->spi = spi;
	m->int_port = int_port;
	m->int_pin = int_pin;

	uint8_t rst = CMD_RESET;
	m->init_step = 1;
	if (spi_xfer(spi, &rst, NULL, 1) != 0) {
		return -1;
	}
	HAL_Delay(10);  /* oscillator start-up */
	m->init_step = 2;

	/* "identification": after reset the chip must report configuration mode */
	if (rd(m, REG_CANSTAT, &v, 1) != 0) {
		return -1;
	}
	m->init_canstat = v;
	if ((v & MODE_MASK) != MODE_CONFIG) {
		return -2;   /* no chip, MISO stuck, wrong CS... */
	}

	mcp2515_timing(&cnf[2], &cnf[1], &cnf[0]);   /* register order: CNF3, CNF2, CNF1 */
	m->init_step = 3;
	wr(m, REG_CNF3, cnf, 3);
	int rr = rd(m, REG_CNF3, back, 3);
	memcpy(m->cnf_wr, cnf, 3);
	memcpy(m->cnf_rd, back, 3);
	if (rr != 0 || memcmp(cnf, back, 3) != 0) {
		return -3;   /* SPI corrupt (wiring / speed / level shifting) */
	}
	m->init_step = 4;

	wr1(m, REG_RXB0CTRL, 0x64);   /* RXM=11 accept all, BUKT=1 roll over to RXB1 */
	wr1(m, REG_RXB1CTRL, 0x60);   /* accept all */
	wr1(m, REG_CANINTE, INTF_RX0IF | INTF_RX1IF | INTF_ERRIF);
	wr1(m, REG_CANINTF, 0x00);
	wr1(m, REG_EFLG, 0x00);
	wr1(m, REG_CANCTRL, MODE_NORMAL | CANCTRL_OSM);   /* CLKOUT off */

	uint32_t t0 = HAL_GetTick();
	do {
		if (rd(m, REG_CANSTAT, &v, 1) == 0 && (v & MODE_MASK) == MODE_NORMAL) {
			m->ok = 1;
			return 0;
		}
	} while (HAL_GetTick() - t0 < 20);
	return -4;
}

int mcp2515_int_active(const Mcp2515 *m)
{
	return HAL_GPIO_ReadPin(m->int_port, m->int_pin) == GPIO_PIN_RESET;
}

int mcp2515_tx_idle(Mcp2515 *m)
{
	uint8_t s;

	if (status(m, CMD_READ_STATUS, &s) != 0) {
		return 0;
	}
	return (s & 0x54) == 0;   /* TXB0/1/2 TXREQ = bits 2, 4, 6 */
}

int mcp2515_send(Mcp2515 *m, const CanFrame *f)
{
	uint8_t s;

	if (status(m, CMD_READ_STATUS, &s) != 0) {
		return -2;
	}
	int n;
	if ((s & 0x04) == 0) {
		n = 0;
	} else if ((s & 0x10) == 0) {
		n = 1;
	} else if ((s & 0x40) == 0) {
		n = 2;
	} else {
		m->st.tx_busy++;
		return -1;
	}

	/* health: did the previous frame in this buffer fail? (one-shot -> ABTF/TXERR) */
	uint8_t ctrl;
	if (rd(m, REG_TXBCTRL(n), &ctrl, 1) == 0 && (ctrl & (TXB_ABTF | TXB_TXERR | TXB_MLOA))) {
		m->st.tx_fail++;
	}

	uint8_t tx[14];
	uint8_t dlc = f->dlc > 8 ? 8 : f->dlc;

	tx[0] = CMD_LOAD_TXB(n);
	mcp2515_encode_id(f->id, f->ext, &tx[1]);
	tx[5] = dlc;
	memcpy(&tx[6], f->data, dlc);
	if (spi_xfer(m->spi, tx, NULL, (uint16_t)(6 + dlc)) != 0) {
		return -2;
	}
	uint8_t rts = CMD_RTS(n);
	if (spi_xfer(m->spi, &rts, NULL, 1) != 0) {
		return -2;
	}
	m->st.tx_sent++;
	return 0;
}

int mcp2515_recv(Mcp2515 *m, CanFrame *f)
{
	uint8_t s;

	if (status(m, CMD_RX_STATUS, &s) != 0) {
		return -1;
	}
	uint8_t cmd;
	if (s & 0x40) {
		cmd = CMD_READ_RXB0;
	} else if (s & 0x80) {
		cmd = CMD_READ_RXB1;
	} else {
		return 0;
	}

	uint8_t tx[14] = {cmd};
	uint8_t rx[14];
	if (spi_xfer(m->spi, tx, rx, 14) != 0) {
		return -1;
	}
	mcp2515_decode_id(&rx[1], &f->id, &f->ext);
	f->dlc = rx[5] & 0x0F;
	if (f->dlc > 8) {
		f->dlc = 8;
	}
	memcpy(f->data, &rx[6], 8);
	m->st.rx_count++;
	return 1;
}

void mcp2515_service_errors(Mcp2515 *m)
{
	uint8_t intf, eflg;

	if (rd(m, REG_CANINTF, &intf, 1) != 0) {
		return;
	}
	if (intf & (INTF_ERRIF | INTF_MERRF)) {
		if (rd(m, REG_EFLG, &eflg, 1) == 0) {
			if (eflg & (EFLG_RX0OVR | EFLG_RX1OVR)) {
				m->st.rx_ovf++;
				bitmod(m, REG_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
			}
			if ((eflg & EFLG_TXBO) && !(m->st.eflg & EFLG_TXBO)) {
				m->st.bus_off++;
			}
			m->st.eflg = eflg;
		}
		bitmod(m, REG_CANINTF, INTF_ERRIF | INTF_MERRF, 0);
	}
}

void mcp2515_read_counters(Mcp2515 *m)
{
	uint8_t v[2], e;

	if (rd(m, REG_TEC, v, 2) == 0) {
		m->st.tec = v[0];
		m->st.rec = v[1];
	}
	if (rd(m, REG_EFLG, &e, 1) == 0) {
		if ((e & EFLG_TXBO) && !(m->st.eflg & EFLG_TXBO)) {
			m->st.bus_off++;
		}
		m->st.eflg = e;
	}
}
