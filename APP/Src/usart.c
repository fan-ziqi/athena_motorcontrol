#include <stdio.h>
#include "usart.h"

void USART1_Init(void)
{
    /* USART configure */
    usart_deinit(USART1);
    usart_baudrate_set(USART1, 115200U);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
    usart_enable(USART1);
    
    /* enable USART RBNE interrupt */ 
    usart_interrupt_enable(USART1, USART_INT_RBNE);
}

/* retarget the C library printf function to the USART */
int __io_putchar(int ch)
{
  usart_data_transmit(USART1, (uint8_t)ch);
  while(RESET == usart_flag_get(USART1, USART_FLAG_TBE));
  return ch;
}
