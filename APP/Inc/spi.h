#ifndef __SPI_H__
#define __SPI_H__

#include "main.h"

#define SPI_SET_NSS_HIGH(x)          gpio_bit_set(x)
#define SPI_SET_NSS_LOW(x)           gpio_bit_reset(x)

int spi_transmit_receive(uint32_t spi_periph, uint16_t *tx_data, uint16_t *rx_data);

void SPI1_Init(void);
void SPI2_Init(void);

#endif /* __SPI_H__ */

