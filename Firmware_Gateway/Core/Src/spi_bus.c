#include "spi_bus.h"

#define SPI_MAX_XFER 32u

void spi_dev_init(SpiDev *d, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
	d->hspi = hspi;
	d->cs_port = cs_port;
	d->cs_pin = cs_pin;
	d->err_count = 0;
	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);   /* deselect */
}

int spi_xfer(SpiDev *d, const uint8_t *tx, uint8_t *rx, uint16_t n)
{
	uint8_t dummy[SPI_MAX_XFER];

	if (n == 0 || n > SPI_MAX_XFER) {
		return -1;
	}
	if (rx == NULL) {
		rx = dummy;
	}
	HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef s = HAL_SPI_TransmitReceive(d->hspi, (uint8_t *)tx, rx, n, 5);
	HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_SET);
	if (s != HAL_OK) {
		d->err_count++;
		return -1;
	}
	return 0;
}
