/// high-bandwidth 3-phase motor control for robots
/// Written by Ben Katz, with much inspiration from Bayley Wang, Nick Kirkby, Shane Colton, David Otten, and others
/// Hardware documentation can be found at build-its.blogspot.com

#include "main.h"
#include "adc.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "gd32f30x.h"
#include "systick.h"

#include "structs.h"
#include <stdio.h>
#include <string.h>

#include "flash_writer.h"
#include "position_sensor.h"
#include "preference_writer.h"
#include "hw_config.h"
#include "user_config.h"
#include "fsm.h"
#include "drv8323.h"
#include "foc.h"
#include "math_ops.h"
#include "calibration.h"


#define VERSION_NUM 2.0f


//Flash寄存器
float __float_reg[64];
int __int_reg[256];
PreferenceWriter prefs;

int count = 0;

//一些控制使用到的结构体
ControllerStruct controller;
ObserverStruct observer;
COMStruct com;
FSMStruct state;
EncoderStruct comm_encoder;
DRVStruct drv;
CalStruct comm_encoder_cal;

can_trasnmit_message_struct can_tx;
can_receive_message_struct can_rx;

//初始化但不分配校准数组
int *error_array = NULL;
int *lut_array = NULL;

void RCU_Init(void);


int main(void)
{
  systick_config();

  RCU_Init();
  GPIO_Init();
  USART1_Init();
  TIM0_Init();
  CAN0_Init();
  SPI1_Init();
  SPI2_Init();
  ADC01_Init();
  ADC2_Init();
  EXTI_Init();

  info("\r\n\r\n\r\n\r\n\r\n");
  info(">> Athean Motor Controller <<\r\n");
  info(">> Version: %d.%d.%d <<\r\n",
      VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);

  //从Flash中加载设置
  preference_writer_init(&prefs, 6);
  preference_writer_load(prefs);

  //若Flash为空则重置配置
  if(E_ZERO==-1){E_ZERO = 0;}
  if(M_ZERO==-1){M_ZERO = 0;}
  if(isnan(I_BW) || I_BW==-1){I_BW = 1000;}
  if(isnan(I_MAX) || I_MAX ==-1){I_MAX=40;}
  if(isnan(I_FW_MAX) || I_FW_MAX ==-1){I_FW_MAX=0;}
  if(CAN_ID==-1){CAN_ID = 1;}
  if(CAN_MASTER==-1){CAN_MASTER = 0;}
  if(CAN_TIMEOUT==-1){CAN_TIMEOUT = 1000;}
  if(isnan(R_NOMINAL) || R_NOMINAL==-1){R_NOMINAL = 0.0f;}
  if(isnan(TEMP_MAX) || TEMP_MAX==-1){TEMP_MAX = 125.0f;}
  if(isnan(I_MAX_CONT) || I_MAX_CONT==-1){I_MAX_CONT = 14.0f;}
  if(isnan(I_CAL)||I_CAL==-1){I_CAL = 5.0f;}
  if(isnan(PPAIRS) || PPAIRS==-1){PPAIRS = 21.0f;}
  if(isnan(GR) || GR==-1){GR = 1.0f;}
  if(isnan(KT) || KT==-1){KT = 1.0f;}
  if(isnan(KP_MAX) || KP_MAX==-1){KP_MAX = 500.0f;}
  if(isnan(KD_MAX) || KD_MAX==-1){KD_MAX = 5.0f;}
  if(isnan(P_MAX)){P_MAX = 12.5f;}
  if(isnan(P_MIN)){P_MIN = -12.5f;}
  if(isnan(V_MAX)){V_MAX = 65.0f;}
  if(isnan(V_MIN)){V_MIN = -65.0f;}

  init_controller_params(&controller);

  //校准编码器零位
  memset(&comm_encoder_cal.cal_position, 0, sizeof(EncoderStruct));

  /*换向编码器设置*/
  comm_encoder.m_zero = M_ZERO;
  comm_encoder.e_zero = E_ZERO;
  comm_encoder.ppairs = PPAIRS;
	
	//编码器首次开启时清除噪声数据
  ps_warmup(&comm_encoder, 100);

  if (EN_ENC_LINEARIZATION)
	{
    //拷贝线性化查找表
    memcpy(&comm_encoder.offset_lut, &ENCODER_LUT, sizeof(comm_encoder.offset_lut));
  }
	else
	{
    memset(&comm_encoder.offset_lut, 0, sizeof(comm_encoder.offset_lut));
  }

#ifdef DEBUG_PS
  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 16; j++) {
      printf("%d ", comm_encoder.offset_lut[i * 16 + j]);
    }
    printf("\r\n");
  }
#endif


  drv_init_config(drv);

#ifdef DEBUG_ADC
  info("ADC OFFSET  B: %d  C: %d\r\n",
      controller.adc_b_offset, controller.adc_c_offset);
#endif

  /* CAN 初始化 */
  can_rx_init(&can_rx);
  can_tx_init(&can_tx);

  /* 开启中断 */
  nvic_irq_enable(TIMER0_UP_IRQn, 0U, 0);
  nvic_irq_enable(EXTI10_15_IRQn, 0U, 1);
  nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 0U, 2);
  nvic_irq_enable(USART1_IRQn, 2U, 0);

  /* 启动FSM */
  state.state = INIT_TEMP_MODE;
  state.next_state = MENU_MODE;
  state.ready = 1;

  uint32_t loop_count = 0;
  FlagStatus status = RESET;

  while (1)
  {
    delay_1ms(1000);
    loop_count += 1;

    if (drv.fault != 0)
      drv_print_faults(drv, loop_count);

    if (status == RESET && state.state != MOTOR_MODE)
		{
      gpio_bit_reset(GPIOC, GPIO_PIN_13);
    }
		else
		{
      gpio_bit_set(GPIOC, GPIO_PIN_13);
    }

#ifdef DEBUG_TIMER
    static uint32_t i = 0;
    debug("loop count %lu\r\n", controller.loop_count - i);
    i = controller.loop_count;
#endif

#ifdef DEBUG_ADC
    float temperature = (1.45 - adc01.data.temp * 3.3 / 4096) * 1000 / 4.1 + 25;
    float vref = (adc01.data.vref * 3.3 / 4096);

    info("hall: %u %u %u %u %u %u\r\n",
      adc01.data.hall0, adc01.data.hall1, adc01.data.hall2,
      adc01.data.hall3, adc01.data.hall4, adc01.data.hall5);
    info("temp: %2.2f  vref: %2.2f\r\n", temperature, vref);
    info("%u %u\r\n", adc01.data.temp, adc01.data.vref);
    float R1 = 15.0;
    float R2 = 1.0;
    info("vbus: %2.2f\r\n", (vbus_voltage / 4095.0f * 3.3) * (R1 + R2) / R2);

    info("i_a: %2.2f i_b: %2.2f i_c: %2.2f v_bus: %2.2f\r\n",
      controller.i_a, controller.i_b, controller.i_c, controller.v_bus);

    debug("%d %d\r\n", controller.adc_a_raw, controller.adc_b_raw);


    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
    adc_software_trigger_enable(ADC2, ADC_REGULAR_CHANNEL);
#endif

//		status = ~status;
    status = (FlagStatus)~status;
  }

}

void RCU_Init(void)
{
  rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);

  rcu_periph_clock_enable(RCU_AF);
  rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_GPIOC);
  rcu_periph_clock_enable(RCU_DMA0);
  rcu_periph_clock_enable(RCU_DMA1);
  rcu_periph_clock_enable(RCU_CAN0);
  rcu_periph_clock_enable(RCU_SPI1);
  rcu_periph_clock_enable(RCU_SPI2);
  rcu_periph_clock_enable(RCU_TIMER0);
  rcu_periph_clock_enable(RCU_ADC0);
  rcu_periph_clock_enable(RCU_ADC1);
  rcu_periph_clock_enable(RCU_ADC2);
  rcu_periph_clock_enable(RCU_USART1);
}

