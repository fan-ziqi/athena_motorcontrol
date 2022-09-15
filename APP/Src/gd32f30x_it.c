/*!
    \file    gd32f30x_it.c
    \brief   interrupt service routines

    \version 2017-02-10, V1.0.0, firmware for GD32F30x
    \version 2018-10-10, V1.1.0, firmware for GD32F30x
    \version 2018-12-25, V2.0.0, firmware for GD32F30x
    \version 2020-09-30, V2.1.0, firmware for GD32F30x 
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f30x_it.h"
#include "main.h"
#include "systick.h"

#include "structs.h"
#include "usart.h"
#include "fsm.h"
#include "spi.h"
#include "gpio.h"
#include "adc.h"
#include "foc.h"
#include "can.h"
#include "position_sensor.h"
#include "hw_config.h"
#include "user_config.h"


/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
    delay_decrement();
}


void USBD_LP_CAN0_RX0_IRQHandler(void)
{
    can_message_receive(CAN0, CAN_FIFO0, &can_rx);

#ifdef DEBUG_CAN
    debug("sid: 0x%04lx eid: 0x%08lx format: %u type: %u length: %u\r\n",
          can_rx.rx_sfid, can_rx.rx_efid, can_rx.rx_ff, can_rx.rx_ft, can_rx.rx_dlen);
    debug("data: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\r\n",
          can_rx.rx_data[0], can_rx.rx_data[1], can_rx.rx_data[2], can_rx.rx_data[3],
          can_rx.rx_data[4], can_rx.rx_data[5], can_rx.rx_data[6], can_rx.rx_data[7]);
#endif

    // 包响应
    pack_reply(&can_tx, CAN_ID, comm_encoder.angle_multiturn[0] / GR, comm_encoder.velocity / GR, controller.i_q_filt * KT * GR);
    can_message_transmit(CAN0, &can_tx);

#ifdef DEBUG_CAN
    debug("CAN TX P:%.3f V:%.3f I:%.3f\r\n",
            comm_encoder.angle_multiturn[0] / GR, comm_encoder.velocity / GR, controller.i_q_filt * KT * GR);
#endif

    /* 检查特殊命令 */
    if (*((uint32_t *)&can_rx.rx_data[0]) == 0xFFFFFFFF && 
			 (*((uint32_t *)&can_rx.rx_data[4]) & 0x00FFFFFF) == 0x00FFFFFF)
		{
			switch (can_rx.rx_data[7])
			{
			case 0xFC:
					update_fsm(&state, MOTOR_CMD);
					break;
			
			case 0xFD:
					update_fsm(&state, ESC_CMD);
					break;
			
			case 0xFE:
					update_fsm(&state, ZERO_CMD);
					break;
			
			default:
					break;
			}
    }
		else
		{
			unpack_cmd(can_rx, controller.commands); // 命令解包
			controller.timeout = 0;                  // 重置超时计数器

#ifdef DEBUG_CAN
        debug("CAN RX P:%.3f V:%.3f KP:%.3f KD:%.3f I:%.3f\r\n",
                controller.p_des, controller.v_des, controller.kp, controller.kd, controller.t_ff);
#endif
    }
}

void TIMER0_UP_IRQHandler(void)
{
    timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_UP);
    
    if (state.state == MOTOR_MODE) {
        gpio_bit_set(GPIOC, GPIO_PIN_13);
    }

	/* ADC 采样 */
	analog_sample(&controller);

	/* 位置传感器采样 */
	ps_sample(&comm_encoder, DT);

	/* 运行有限状态机 Finite State Machine */
	run_fsm(&state);

#ifdef DEBUG_TIMER
                            //cyberdog layout      //mbed
    controller.dtc_u = 0.6; //PA8 /TIMER0_CH0/INHA //PA10
    controller.dtc_v = 0.4; //PA9 /TIMER0_CH1/INHB //PA9
    controller.dtc_w = 0.5; //PA10/TIMER0_CH2/INHC //PA8
    set_dtc(&controller);
#endif

	/* increment loop count */
	controller.loop_count++;
    
    if (state.state == MOTOR_MODE) {
        gpio_bit_reset(GPIOC, GPIO_PIN_13);
    }
}

void EXTI10_15_IRQHandler(void)
{
    exti_interrupt_flag_clear(EXTI_12);

    if (RESET == exti_interrupt_flag_get(EXTI_12)) 
		{
			// DRV8323 错误
			drv.fsr1 = drv_read_FSR1(drv);
			drv.fsr2 = drv_read_FSR2(drv);
			drv.fault = (drv.fsr1 >> 10) & 1;
			if (drv.fault)
			{
				debug("Fault FSR1: %04x FSR2: %04x\r\n", drv.fsr1, drv.fsr2);
			}
    } 
		else
		{
			// DRV8323 恢复, 清除错误位
			drv_clear_fault(drv);
			drv.fault = 0;
			debug("Recovery\r\n");
    }
}

void USART1_IRQHandler(void)
{
    if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE))
		{
			//串口控制有限状态机
			char c = usart_data_receive(USART1);
	    update_fsm(&state, c);
    }
}
