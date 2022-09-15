#include "can.h"
#include "hw_config.h"
#include "user_config.h"
#include "math_ops.h"


void CAN0_Init(void)
{
    can_parameter_struct            can_parameter;
    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    
    /* 初始化can寄存器 */
    can_deinit(CAN0);
    
    /* 初始化CAN */
    /* 波特率 1Mbps */
    can_parameter.working_mode = CAN_NORMAL_MODE;
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_5TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_4TQ;
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = DISABLE;
    can_parameter.auto_wake_up = DISABLE;
		can_parameter.auto_retrans = ENABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.prescaler = 6;
    can_init(CAN0, &can_parameter);

    can_interrupt_enable(CAN0, CAN_INT_RFNE0);
}

void can_rx_init(can_receive_message_struct *msg)
{
  can_struct_para_init(CAN_RX_MESSAGE_STRUCT, msg);

    can_filter_parameter_struct     can_filter;
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);

    /* 初始化can滤波器 */    
    can_filter.filter_list_high = CAN_ID << 5;
    can_filter.filter_list_low = 0x0000;
    can_filter.filter_mask_high = 0xFFE0;
    can_filter.filter_mask_low = 0x0000;  
    can_filter.filter_fifo_number = CAN_FIFO0;
    can_filter.filter_number = 0;
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    can_filter.filter_enable = ENABLE;
    can_filter_init(&can_filter);
}

void can_tx_init(can_trasnmit_message_struct *msg)
{
  can_struct_para_init(CAN_TX_MESSAGE_STRUCT, msg);
  
  msg->tx_sfid = CAN_MASTER;
  msg->tx_efid = 0U;
  msg->tx_ft = CAN_FT_DATA;
  msg->tx_ff = CAN_FF_STANDARD;
  msg->tx_dlen = 6U;
}

/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]]
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]
void pack_reply(can_trasnmit_message_struct *msg, uint8_t id, float p, float v, float t)
{
  int p_int = float_to_uint(p, P_MIN, P_MAX, 16);
  int v_int = float_to_uint(v, V_MIN, V_MAX, 12);
  int t_int = float_to_uint(t, -I_MAX * KT * GR, I_MAX * KT * GR, 12);
  msg->tx_data[0] = id;
  msg->tx_data[1] = p_int >> 8;
  msg->tx_data[2] = p_int & 0xFF;
  msg->tx_data[3] = v_int >> 4;
  msg->tx_data[4] = ((v_int & 0xF) << 4) + (t_int >> 8);
  msg->tx_data[5] = t_int & 0xFF;
  msg->tx_data[6] = 0;
  msg->tx_data[7] = 0;
}

/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]]
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]

void unpack_cmd(can_receive_message_struct msg, float *commands)
{
  int p_int = (msg.rx_data[0] << 8) | msg.rx_data[1];
  int v_int = (msg.rx_data[2] << 4) | (msg.rx_data[3] >> 4);
  int kp_int = ((msg.rx_data[3] & 0xF) << 8) | msg.rx_data[4];
  int kd_int = (msg.rx_data[5] << 4) | (msg.rx_data[6] >> 4);
  int t_int = ((msg.rx_data[6] & 0xF) << 8) | msg.rx_data[7];

  commands[0] = uint_to_float(p_int, P_MIN, P_MAX, 16);
  commands[1] = uint_to_float(v_int, V_MIN, V_MAX, 12);
  commands[2] = uint_to_float(kp_int, KP_MIN, KP_MAX, 12);
  commands[3] = uint_to_float(kd_int, KD_MIN, KD_MAX, 12);
  commands[4] = uint_to_float(t_int, -I_MAX * KT * GR, I_MAX * KT * GR, 12);
}
