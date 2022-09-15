#ifndef HW_CONFIG_H
#define HW_CONFIG_H

/* 定时器和PWM */
#define TIM_PWM								TIMER0									// PWM定时器
#define TIM_CH_U							TIMER_CH_0							// U相通道
#define TIM_CH_V							TIMER_CH_1							// V相通道
#define TIM_CH_W							TIMER_CH_2							// W相通道
#define INVERT_DTC						1												// PWM 反向 (1) 或 不反向 (0)

/* 中断 */
#define PWM_ISR								TIMER0_UP_IRQn          // PWM 定时器 中断
#define CAN_ISR								USBD_LP_CAN0_RX0_IRQn		// CAN 接收 中断

/* ADC */
#define ADC_CH_MAIN						ADC0										// 驱动同步模式的 ADC channel handle
#define ADC_CH_IA							0												// A相电流检测的 ADC channel handle, 0为未使用
#define ADC_CH_IB							ADC0										// B相电流检测的 ADC channel handle. 0为未使用
#define ADC_CH_IC							ADC1										// C相电流检测的 ADC channel handle. 0为未使用
#define ADC_CH_VBUS						ADC2										// 总线电压的 ADC channel handle, 0为未使用

/* DRV 驱动 */
#define ENABLE_PIN 						GPIOA, GPIO_PIN_11 		  // DRV enable pin
#define DRV_SPI								SPI1										// DRV SPI handle
#define DRV_CS								GPIOB, GPIO_PIN_12			// DRV CS pin

/* SPI 编码器 */
#define ENC_SPI								SPI2										// SPI编码器 handle
#define ENC_CS								GPIOA, GPIO_PIN_15			// SPI编码器 CS pin
#define ENC_CPR								65536										// 编码器转一圈的计数值
#define INV_CPR								1.0f/ENC_CPR						
#define ENC_READ_WORD					0x00										// 编码器读取命令

/* GPIO */
#define LED         					GPIOC, GPIO_PIN_13			// LED Pin

/* CAN */
#define CAN_H									CAN0										// CAN handle

/* 其他与硬件相关的常量 */
#define I_SCALE 							0.0201416f							// 放大器增益为 40X 时每个 A/D 计数的安培数
#define V_SCALE 							0.0128906f							// 每个 A/D 计数的总线电压
#define DTC_MAX 							0.94f          					// 最大占空比
#define DTC_MIN 							0.0f          					// 最小占空比
#define DTC_COMP 							0.000f        				  // 死区补偿 (100 ns / 25 us)
#define DT										0.00003333f							// 循环周期
#define EN_ENC_LINEARIZATION 	1												// 启用/禁用编码器线性化


/* 电流控制器 */
#define L_D								 		.000003f								// D轴电感
#define L_Q 									.000003f								// Q轴电感
#define K_D 									.05f                    // 循环增益,  Volts/Amp
#define K_Q 									.05f                    // 循环增益,  Volts/Amp
#define K_SCALE 							0.0001f      		        // K_loop/Loop BW (Hz) 0.0042
#define KI_D 									0.045f           	      // PI zero, in radians per sample
#define KI_Q 									0.045f                  // PI zero, in radians per sample
#define OVERMODULATION 				1.15f    						    // 1.0 = no overmodulation
#define CURRENT_FILT_ALPHA		.1f	   								  // 一阶 d/q 电流滤波器（不用于控制）
#define VBUS_FILT_ALPHA				.1f											// 一阶母线电压滤波器

#define D_INT_LIM						  V_BUS/(K_D*KI_D) 			  // Amps*samples
#define Q_INT_LIM 						V_BUS/(K_Q*KI_Q) 		    // Amps*samples

#endif
