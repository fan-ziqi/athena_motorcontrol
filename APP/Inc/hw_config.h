#ifndef HW_CONFIG_H
#define HW_CONFIG_H

/* ��ʱ����PWM */
#define TIM_PWM								TIMER0									// PWM��ʱ��
#define TIM_CH_U							TIMER_CH_0							// U��ͨ��
#define TIM_CH_V							TIMER_CH_1							// V��ͨ��
#define TIM_CH_W							TIMER_CH_2							// W��ͨ��
#define INVERT_DTC						1												// PWM ���� (1) �� ������ (0)

/* �ж� */
#define PWM_ISR								TIMER0_UP_IRQn          // PWM ��ʱ�� �ж�
#define CAN_ISR								USBD_LP_CAN0_RX0_IRQn		// CAN ���� �ж�

/* ADC */
#define ADC_CH_MAIN						ADC0										// ����ͬ��ģʽ�� ADC channel handle
#define ADC_CH_IA							0												// A��������� ADC channel handle, 0Ϊδʹ��
#define ADC_CH_IB							ADC0										// B��������� ADC channel handle. 0Ϊδʹ��
#define ADC_CH_IC							ADC1										// C��������� ADC channel handle. 0Ϊδʹ��
#define ADC_CH_VBUS						ADC2										// ���ߵ�ѹ�� ADC channel handle, 0Ϊδʹ��

/* DRV ���� */
#define ENABLE_PIN 						GPIOA, GPIO_PIN_11 		  // DRV enable pin
#define DRV_SPI								SPI1										// DRV SPI handle
#define DRV_CS								GPIOB, GPIO_PIN_12			// DRV CS pin

/* SPI ������ */
#define ENC_SPI								SPI2										// SPI������ handle
#define ENC_CS								GPIOA, GPIO_PIN_15			// SPI������ CS pin
#define ENC_CPR								65536										// ������תһȦ�ļ���ֵ
#define INV_CPR								1.0f/ENC_CPR						
#define ENC_READ_WORD					0x00										// ��������ȡ����

/* GPIO */
#define LED         					GPIOC, GPIO_PIN_13			// LED Pin

/* CAN */
#define CAN_H									CAN0										// CAN handle

/* ������Ӳ����صĳ��� */
#define I_SCALE 							0.0201416f							// �Ŵ�������Ϊ 40X ʱÿ�� A/D �����İ�����
#define V_SCALE 							0.0128906f							// ÿ�� A/D ���������ߵ�ѹ
#define DTC_MAX 							0.94f          					// ���ռ�ձ�
#define DTC_MIN 							0.0f          					// ��Сռ�ձ�
#define DTC_COMP 							0.000f        				  // �������� (100 ns / 25 us)
#define DT										0.00003333f							// ѭ������
#define EN_ENC_LINEARIZATION 	1												// ����/���ñ��������Ի�


/* ���������� */
#define L_D								 		.000003f								// D����
#define L_Q 									.000003f								// Q����
#define K_D 									.05f                    // ѭ������,  Volts/Amp
#define K_Q 									.05f                    // ѭ������,  Volts/Amp
#define K_SCALE 							0.0001f      		        // K_loop/Loop BW (Hz) 0.0042
#define KI_D 									0.045f           	      // PI zero, in radians per sample
#define KI_Q 									0.045f                  // PI zero, in radians per sample
#define OVERMODULATION 				1.15f    						    // 1.0 = no overmodulation
#define CURRENT_FILT_ALPHA		.1f	   								  // һ�� d/q �����˲����������ڿ��ƣ�
#define VBUS_FILT_ALPHA				.1f											// һ��ĸ�ߵ�ѹ�˲���

#define D_INT_LIM						  V_BUS/(K_D*KI_D) 			  // Amps*samples
#define Q_INT_LIM 						V_BUS/(K_Q*KI_Q) 		    // Amps*samples

#endif
