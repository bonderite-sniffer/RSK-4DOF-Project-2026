#include "mcp2515_diag.h"
#include "mcp2515.h"
#include "spi_bus.h"
#include <string.h>

volatile McpDiag g_diag[2];

static SpiDev s_spi[2];

static int rd1(SpiDev *d, uint8_t reg, uint8_t *v)
{
	uint8_t tx[3] = {0x03, reg, 0x00}, rx[3] = {0};

	if (spi_xfer(d, tx, rx, 3) != 0) {
		return -1;
	}
	*v = rx[2];
	return 0;
}

static int wr1(SpiDev *d, uint8_t reg, uint8_t v)
{
	uint8_t tx[3] = {0x02, reg, v};

	return spi_xfer(d, tx, NULL, 3);
}

static uint8_t run_one(SpiDev *d, volatile McpDiag *g)
{
	static const uint8_t pat[4] = {0x55, 0xAA, 0x0F, 0xF0};
	uint8_t v;

	memset((void *)g, 0, sizeof(*g));
	g->cs_readback = (uint8_t)HAL_GPIO_ReadPin(d->cs_port, d->cs_pin);

	/* 1. reset, then the chip must be in configuration mode */
	uint8_t rst = 0xC0;
	if (spi_xfer(d, &rst, NULL, 1) != 0) {
		g->hal_errors = d->err_count;
		return 7;
	}
	HAL_Delay(10);
	if (rd1(d, 0x0E, &v) != 0) {
		g->hal_errors = d->err_count;
		return 7;
	}
	g->canstat_reset = v;
	rd1(d, 0x0F, &v);
	g->canctrl_reset = v;

	/* 2. write / read back patterns in CNF1 (a plain R/W register in config mode) */
	int bad = 0;
	for (int k = 0; k < 4; k++) {
		g->wr[k] = pat[k];
		wr1(d, 0x2A, pat[k]);
		rd1(d, 0x2A, &v);
		g->rd[k] = v;
		bad |= (v != pat[k]);
	}
	g->hal_errors = d->err_count;
	if (d->err_count) {
		return 7;
	}
	if (g->canstat_reset == 0xFF && g->canctrl_reset == 0xFF) {
		return 2;
	}
	if (g->canstat_reset == 0x00 && g->canctrl_reset == 0x00 && g->rd[0] == 0x00) {
		return 3;
	}
	if ((g->canstat_reset & 0xE0) != 0x80 || g->canctrl_reset != 0x87) {
		return 4;
	}
	if (bad) {
		return 5;
	}

	/* 2b. burst access: the real driver writes/reads several registers per transfer */
	{
		uint8_t tx[16] = {0x02, 0x28, 0x05, 0x9A, 0x01};   /* WRITE CNF3, CNF2, CNF1 */
		uint8_t q[16] = {0x03, 0x28}, a[16] = {0};
		int ok = 1;

		spi_xfer(d, tx, NULL, 5);
		spi_xfer(d, q, a, 5);
		for (int k = 0; k < 3; k++) {
			g->bw[k] = tx[2 + k];
			g->br[k] = a[2 + k];
			ok &= (a[2 + k] == tx[2 + k]);
		}

		uint8_t ltx[16] = {0x02, 0x31};                       /* WRITE TXB0SIDH..TXB0D7 */
		uint8_t lq[16] = {0x03, 0x31}, la[16] = {0};
		for (int k = 0; k < 13; k++) {
			ltx[2 + k] = (uint8_t)(0x11 * (k + 1));
		}
		ltx[2 + 4] = 0x08;                                    /* DLC register: keep it valid */
		spi_xfer(d, ltx, NULL, 15);
		spi_xfer(d, lq, la, 15);
		for (int k = 0; k < 13; k++) {
			uint8_t mask = (k == 1) ? 0xEB : (k == 4 ? 0x4F : 0xFF);   /* SIDL bit2/4, DLC bits 4-5,7 unimplemented */
			g->lw[k] = ltx[2 + k];
			g->lr[k] = la[2 + k];
			ok &= ((la[2 + k] & mask) == (ltx[2 + k] & mask));
		}
		g->burst_ok = (uint8_t)ok;
		g->hal_errors = d->err_count;
		if (d->err_count) {
			return 7;
		}
		if (!ok) {
			return 8;
		}
	}

	/* 3. loopback: needs the oscillator running and correct bit timing */
	uint8_t cnf1, cnf2, cnf3;
	mcp2515_timing(&cnf1, &cnf2, &cnf3);
	wr1(d, 0x2A, cnf1);
	wr1(d, 0x29, cnf2);
	wr1(d, 0x28, cnf3);
	wr1(d, 0x60, 0x60);          /* RXB0: accept all */
	wr1(d, 0x2B, 0x00);          /* no interrupts on the pin */
	wr1(d, 0x2C, 0x00);
	wr1(d, 0x0F, 0x40);          /* request loopback */
	HAL_Delay(2);
	rd1(d, 0x0E, &v);
	g->loop_mode = v;

	uint8_t tx[14] = {0x40};     /* LOAD TXB0 */
	mcp2515_encode_id(0x1ABCDE5u, 1, &tx[1]);
	tx[5] = 2;
	tx[6] = 0xC3;
	tx[7] = 0x3C;
	spi_xfer(d, tx, NULL, 8);
	uint8_t rts = 0x81;
	spi_xfer(d, &rts, NULL, 1);
	HAL_Delay(5);

	rd1(d, 0x2C, &v);
	g->loop_intf = v;
	if (v & 0x01) {
		uint8_t q[14] = {0x90}, a[14] = {0};
		uint32_t id;
		uint8_t ext;

		spi_xfer(d, q, a, 14);
		mcp2515_decode_id(&a[1], &id, &ext);
		g->loop_ok = (ext && id == 0x1ABCDE5u && a[6] == 0xC3 && a[7] == 0x3C);
	}
	wr1(d, 0x0F, 0x80);          /* back to config mode; can_bus_init_all() resets anyway */
	return g->loop_ok ? 1 : 6;
}

void mcp2515_diag_run(void)
{
	spi_dev_init(&s_spi[0], CAN1_SPI_HANDLE, CAN1_CS_GPIO_Port, CAN1_CS_Pin);
	spi_dev_init(&s_spi[1], CAN2_SPI_HANDLE, CAN2_CS_GPIO_Port, CAN2_CS_Pin);
	HAL_Delay(2);
	for (int b = 0; b < 2; b++) {
		g_diag[b].verdict = run_one(&s_spi[b], &g_diag[b]);
	}
}
