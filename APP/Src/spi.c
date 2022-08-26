#include "spi.h"

int spi_transmit_receive(uint32_t spi_periph, uint16_t *tx_data, uint16_t *rx_data)
{
  
  while (RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_TBE));

  spi_i2s_data_transmit(spi_periph, *tx_data);

  while (RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_RBNE));

  *rx_data = spi_i2s_data_receive(spi_periph);

  return 0;
}

void SPI1_Init(void)
{
  /* SPI1 for DRV8323RS */
  spi_parameter_struct spi_init_struct;

  spi_i2s_deinit(SPI1);

  spi_init_struct.device_mode = SPI_MASTER;
  spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
  spi_init_struct.frame_size = SPI_FRAMESIZE_16BIT;
  spi_init_struct.nss = SPI_NSS_SOFT;
  spi_init_struct.endian = SPI_ENDIAN_MSB;
  spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;
  spi_init_struct.prescale = SPI_PSC_8;

  spi_init(SPI1, &spi_init_struct);

  spi_enable(SPI1);
}

void SPI2_Init(void)
{
  /* SPI2 for AS5047P */
  spi_parameter_struct spi_init_struct;

  spi_i2s_deinit(SPI2);

  spi_init_struct.device_mode = SPI_MASTER;
  spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
  spi_init_struct.frame_size = SPI_FRAMESIZE_16BIT;
  spi_init_struct.nss = SPI_NSS_SOFT;
  spi_init_struct.endian = SPI_ENDIAN_MSB;
  spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;
  spi_init_struct.prescale = SPI_PSC_8;

  spi_init(SPI2, &spi_init_struct);

  spi_enable(SPI2);
}
