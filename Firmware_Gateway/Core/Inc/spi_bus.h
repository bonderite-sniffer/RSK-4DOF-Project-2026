/* spi_bus.h - one SPI device = SPI handle + software chip-select pin. */
#ifndef SPI_BUS_H
#define SPI_BUS_H

#include "app_config.h"
#include <stdint.h>

typedef struct {
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *cs_port;
	uint16_t cs_pin;
	uint32_t err_count;   /* HAL errors / timeouts */
} SpiDev;

void spi_dev_init(SpiDev *d, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);

/* Full-duplex transfer with CS asserted for the whole transfer.
 * rx may be NULL. Returns 0 on success, -1 on HAL error. */
int spi_xfer(SpiDev *d, const uint8_t *tx, uint8_t *rx, uint16_t n);

#endif
