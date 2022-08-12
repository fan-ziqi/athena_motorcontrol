/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#ifdef STM32F446
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

extern SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_SPI1_Init(void);
void MX_SPI3_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#else
#define SPI_SET_NSS_HIGH(x)          gpio_bit_set(x)
#define SPI_SET_NSS_LOW(x)           gpio_bit_reset(x)

inline int spi_transmit_receive(uint32_t spi_periph, uint16_t *tx_data, uint16_t *rx_data)
{
  
  while (RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_TBE));

  spi_i2s_data_transmit(spi_periph, *tx_data);

  while (RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_RBNE));

  *rx_data = spi_i2s_data_receive(spi_periph);

  return 0;
}

void MX_SPI1_Init(void);
void MX_SPI2_Init(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

