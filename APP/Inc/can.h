#ifndef __CAN_H__
#define __CAN_H__

#include "main.h"

//#define P_MIN -12.5f
//#define P_MAX 12.5f
//#define V_MIN -65.0f
//#define V_MAX 65.0f
#define KP_MIN 0.0f
//#define KP_MAX 500.0f
#define KD_MIN 0.0f
//#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f
#ifdef STM32F446
/* USER CODE END Private defines */

void CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
typedef struct{
	uint8_t id;
	uint8_t data[8];
	CAN_RxHeaderTypeDef rx_header;
	CAN_FilterTypeDef filter;
}CANRxMessage ;

typedef struct{
	uint8_t id;
	uint8_t data[6];
	CAN_TxHeaderTypeDef tx_header;
}CANTxMessage ;

void can_rx_init(CANRxMessage *msg);
void can_tx_init(CANTxMessage *msg);
void pack_reply(CANTxMessage *msg, uint8_t id, float p, float v, float t);
void unpack_cmd(CANRxMessage msg, float *commands);
/* USER CODE END Prototypes */

#else
void can_rx_init(can_receive_message_struct *msg);
void can_tx_init(can_trasnmit_message_struct *msg);
void pack_reply(can_trasnmit_message_struct *msg, uint8_t id, float p, float v, float t);
void unpack_cmd(can_receive_message_struct msg, float *commands);

void CAN0_Init(void);
#endif

#endif /* __CAN_H__ */

