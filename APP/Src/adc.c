#include "adc.h"


// Face to AS5047P
// clockwise 6 4 15 5 7 14
//   14  6
// 7       4
//    5 15

// 8  TEMP_PCB
// 12 TEMP_MOTOR
// 13 VBUS_VOLTAGE

// ADC0  6 15 7 8
// ADC1  5 14 4 12
// ADC2  13


union adc01_dma_t adc01;
uint16_t vbus_voltage;

void ADC01_Init(void)
{
    dma_parameter_struct dma_data_parameter;
    
    /* ADC DMA_channel configuration */
    dma_deinit(DMA0, DMA_CH0);
    
    /* initialize DMA data mode */
    dma_data_parameter.periph_addr = (uint32_t)(&ADC_RDATA(ADC0));
    dma_data_parameter.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_data_parameter.memory_addr = (uint32_t)(&adc01.raw);
    dma_data_parameter.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_32BIT;
    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_32BIT;  
    dma_data_parameter.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_data_parameter.number = 4;
    dma_data_parameter.priority = DMA_PRIORITY_MEDIUM;
    dma_init(DMA0, DMA_CH0, &dma_data_parameter);

    dma_circulation_enable(DMA0, DMA_CH0);
  
    /* enable DMA channel */
    dma_channel_enable(DMA0, DMA_CH0);

  adc_deinit(ADC0);
  adc_deinit(ADC1);

  adc_mode_config(ADC_DAUL_REGULAL_PARALLEL_INSERTED_PARALLEL);

  // adc_tempsensor_vrefint_enable();

  adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
  adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, DISABLE);
  adc_special_function_config(ADC1, ADC_SCAN_MODE, ENABLE);
  adc_special_function_config(ADC1, ADC_CONTINUOUS_MODE, DISABLE);

  adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
  adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 4U);
  adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 1U);
  adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);
  adc_channel_length_config(ADC1, ADC_REGULAR_CHANNEL, 4U);
  adc_channel_length_config(ADC1, ADC_INSERTED_CHANNEL, 1U);

  adc_regular_channel_config(ADC0, 0U, ADC_CHANNEL_6, ADC_SAMPLETIME_1POINT5);
  adc_regular_channel_config(ADC0, 1U, ADC_CHANNEL_15, ADC_SAMPLETIME_1POINT5);
  adc_regular_channel_config(ADC0, 2U, ADC_CHANNEL_7, ADC_SAMPLETIME_1POINT5);
  adc_regular_channel_config(ADC0, 3U, ADC_CHANNEL_8, ADC_SAMPLETIME_1POINT5);
  adc_regular_channel_config(ADC1, 0U, ADC_CHANNEL_5, ADC_SAMPLETIME_1POINT5);
  adc_regular_channel_config(ADC1, 1U, ADC_CHANNEL_14, ADC_SAMPLETIME_1POINT5);
  adc_regular_channel_config(ADC1, 2U, ADC_CHANNEL_4, ADC_SAMPLETIME_1POINT5);
  adc_regular_channel_config(ADC0, 3U, ADC_CHANNEL_12, ADC_SAMPLETIME_1POINT5);

  adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE);
  adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);
  adc_external_trigger_source_config(ADC1, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE);
  adc_external_trigger_config(ADC1, ADC_REGULAR_CHANNEL, ENABLE);

  adc_inserted_channel_config(ADC0, 0U, ADC_CHANNEL_11, ADC_SAMPLETIME_1POINT5); // SOB
  adc_inserted_channel_config(ADC1, 0U, ADC_CHANNEL_10, ADC_SAMPLETIME_1POINT5); // SOC

  adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_2_EXTTRIG_INSERTED_NONE);
  adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, ENABLE);
  adc_external_trigger_source_config(ADC1, ADC_INSERTED_CHANNEL, ADC0_1_2_EXTTRIG_INSERTED_NONE);
  adc_external_trigger_config(ADC1, ADC_INSERTED_CHANNEL, ENABLE);

  adc_dma_mode_enable(ADC0);
  adc_dma_mode_enable(ADC1);
  adc_enable(ADC0);
  adc_enable(ADC1);
  delay_1ms(1);

  adc_calibration_enable(ADC0);
  adc_calibration_enable(ADC1);
}

void ADC2_Init(void)
{
    dma_parameter_struct dma_data_parameter;
    
    /* ADC DMA_channel configuration */
    dma_deinit(DMA1, DMA_CH4);
    
    /* initialize DMA data mode */
    dma_data_parameter.periph_addr = (uint32_t)(&ADC_RDATA(ADC2));
    dma_data_parameter.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_data_parameter.memory_addr = (uint32_t)(&vbus_voltage);
    dma_data_parameter.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_16BIT;  
    dma_data_parameter.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_data_parameter.number = 1;
    dma_data_parameter.priority = DMA_PRIORITY_MEDIUM;
    dma_init(DMA1, DMA_CH4, &dma_data_parameter);

    dma_circulation_enable(DMA1, DMA_CH4);
  
    /* enable DMA channel */
    dma_channel_enable(DMA1, DMA_CH4);

  adc_deinit(ADC2);

  adc_special_function_config(ADC2, ADC_SCAN_MODE, ENABLE);
  adc_special_function_config(ADC2, ADC_CONTINUOUS_MODE, DISABLE);

  adc_data_alignment_config(ADC2, ADC_DATAALIGN_RIGHT);
  adc_channel_length_config(ADC2, ADC_REGULAR_CHANNEL, 1U);
  adc_channel_length_config(ADC2, ADC_INSERTED_CHANNEL, 1U);

  adc_regular_channel_config(ADC2, 0U, ADC_CHANNEL_13, ADC_SAMPLETIME_1POINT5);

  adc_external_trigger_source_config(ADC2, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE);
  adc_external_trigger_config(ADC2, ADC_REGULAR_CHANNEL, ENABLE);

  adc_inserted_channel_config(ADC2, 0U, ADC_CHANNEL_13, ADC_SAMPLETIME_1POINT5);

  adc_external_trigger_source_config(ADC2, ADC_INSERTED_CHANNEL, ADC0_1_2_EXTTRIG_INSERTED_NONE);
  adc_external_trigger_config(ADC2, ADC_INSERTED_CHANNEL, ENABLE);

  adc_dma_mode_enable(ADC2);
  adc_enable(ADC2);
  delay_1ms(1);

  adc_calibration_enable(ADC2);
}
