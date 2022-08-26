#ifndef __ADC_H__
#define __ADC_H__

#include "main.h"


struct adc01_data_t
{
  uint16_t hall0;
  uint16_t hall3;
  uint16_t hall2;
  uint16_t hall5;
  uint16_t hall4;
  uint16_t hall1;
  uint16_t temp_pcb;
  uint16_t temp_motor;
};

union adc01_dma_t
{
  uint32_t raw[4];
  struct adc01_data_t data;
};

extern union adc01_dma_t adc01;
extern uint16_t vbus_voltage;

void ADC01_Init(void);
void ADC2_Init(void);


#endif /* __ADC_H__ */

