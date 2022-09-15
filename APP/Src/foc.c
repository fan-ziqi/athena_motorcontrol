/*
 * foc.c
 *
 *  Created on: Aug 2, 2020
 *      Author: ben
 */

#include "foc.h"
#include "adc.h"
#include "tim.h"
#include "position_sensor.h"
#include "math_ops.h"
#include "hw_config.h"
#include "user_config.h"

void set_dtc(ControllerStruct *controller)
{

	/* 如果硬件是这样配置的，则反转占空比 */
	float dtc_u = controller->dtc_u;
	float dtc_v = controller->dtc_v;
	float dtc_w = controller->dtc_w;
	if(INVERT_DTC)
	{
		dtc_u = 1.0f - controller->dtc_u;
		dtc_v = 1.0f - controller->dtc_v;
		dtc_w = 1.0f - controller->dtc_w;
	}
	
	/* 处理相序交换，使电压/电流/转矩与编码器方向匹配 */
	if(!PHASE_ORDER)
	{
		timer_channel_output_pulse_value_config(TIMER0, TIM_CH_U, SVPWM_PERIOD * dtc_u);
		timer_channel_output_pulse_value_config(TIMER0, TIM_CH_V, SVPWM_PERIOD * dtc_v);
		timer_channel_output_pulse_value_config(TIMER0, TIM_CH_W, SVPWM_PERIOD * dtc_w);
	}
	else
	{
		timer_channel_output_pulse_value_config(TIMER0, TIM_CH_V, SVPWM_PERIOD * dtc_u);
		timer_channel_output_pulse_value_config(TIMER0, TIM_CH_U, SVPWM_PERIOD * dtc_v);
		timer_channel_output_pulse_value_config(TIMER0, TIM_CH_W, SVPWM_PERIOD * dtc_w);
	}
}

/* ADC采样 */
void analog_sample(ControllerStruct *controller)
{
	/* 处理相序交换，使电压/电流/转矩与编码器方向匹配 */
	if(!PHASE_ORDER)
	{
		controller->adc_b_raw = adc_inserted_data_read(ADC_CH_IB, ADC_INSERTED_CHANNEL_0);
		controller->adc_c_raw = adc_inserted_data_read(ADC_CH_IC, ADC_INSERTED_CHANNEL_0);
	}
	else
	{
		controller->adc_b_raw = adc_inserted_data_read(ADC_CH_IC, ADC_INSERTED_CHANNEL_0);
		controller->adc_c_raw = adc_inserted_data_read(ADC_CH_IB, ADC_INSERTED_CHANNEL_0);
	}

	//开启ADC
	adc_software_trigger_enable(ADC_CH_MAIN, ADC_INSERTED_CHANNEL);
	adc_software_trigger_enable(ADC_CH_VBUS, ADC_INSERTED_CHANNEL);
	while(!(SET == adc_flag_get(ADC_CH_MAIN, ADC_FLAG_EOIC)));
	while(!(SET == adc_flag_get(ADC_CH_VBUS, ADC_FLAG_EOIC)));
	adc_flag_clear(ADC_CH_MAIN, ADC_FLAG_EOIC);
	adc_flag_clear(ADC_CH_VBUS, ADC_FLAG_EOIC);

	controller->adc_vbus_raw = adc_inserted_data_read(ADC_CH_VBUS, ADC_INSERTED_CHANNEL_0);
  controller->v_bus = (float)controller->adc_vbus_raw * V_SCALE;

	//根据 ADC 读数计算相电流
	controller->i_b = controller->i_scale * (float)(controller->adc_b_raw - controller->adc_b_offset);
	controller->i_c = controller->i_scale * (float)(controller->adc_c_raw - controller->adc_c_offset);
	controller->i_a = -controller->i_b - controller->i_c;
}

/* dq坐标系 ---> dq_to_abc坐标系*/
void dq_to_abc( float theta, float d, float q, float *a, float *b, float *c)
{
	/* 相电流幅值 = dq 矢量的长度 如 iq = 1, id = 0, 峰值相电流为1 (等幅值变换)*/

	float ct = cos_lut(theta);
	float st = sin_lut(theta);

	*a =                       ct  * d -                       st  * q;
	*b = ( SQRT3_DIV2*st - .5f*ct) * d + ( SQRT3_DIV2*ct + .5f*st) * q;
	*c = (-SQRT3_DIV2*st - .5f*ct) * d + (-SQRT3_DIV2*ct + .5f*st) * q;
}

/* dq_to_abc坐标系 ---> dq坐标系 */
void abc_to_dq(float theta, float a, float b, float c, float *d, float *q)
{
	/* 相电流幅值 = dq 矢量的长度 如 iq = 1, id = 0, 峰值相电流为1 (等幅值变换)*/

	float ct = cos_lut(theta);
	float st = sin_lut(theta);

	*d = 0.6666667f*( ct*a + (SQRT3_DIV2 * st - .5f * ct) * b + (-SQRT3_DIV2 * st - .5f * ct) * c);
	*q = 0.6666667f*(-st*a + (SQRT3_DIV2 * ct + .5f * st) * b + (-SQRT3_DIV2 * ct + .5f * st) * c);
}

/*SVPWM (Space Vector Pulse Width Modulation) 空间矢量脉宽调制*/
void svm(float v_max, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w)
{
	//v,v,w幅值 = v_bus

	float v_offset = (fminf3(u, v, w) + fmaxf3(u, v, w))*0.5f;
	float v_midpoint = .5f*(DTC_MAX+DTC_MIN);

	*dtc_u = fast_fminf(fast_fmaxf((.5f*(u -v_offset)*OVERMODULATION/v_max + v_midpoint ), DTC_MIN), DTC_MAX);
	*dtc_v = fast_fminf(fast_fmaxf((.5f*(v -v_offset)*OVERMODULATION/v_max + v_midpoint ), DTC_MIN), DTC_MAX);
	*dtc_w = fast_fminf(fast_fmaxf((.5f*(w -v_offset)*OVERMODULATION/v_max + v_midpoint ), DTC_MIN), DTC_MAX);

}

/*测量零电流 ADC 偏移*/
void zero_current(ControllerStruct *controller)
{
	int adc_b_offset = 0;
	int adc_c_offset = 0;
	int n = 1000;
	controller->dtc_u = 0.f;
	controller->dtc_v = 0.f;
	controller->dtc_w = 0.f;
	set_dtc(controller);

	//平均n次采样
	for (int i = 0; i<n; i++)
	{
		analog_sample(controller);
		adc_b_offset += controller->adc_b_raw;
		adc_c_offset += controller->adc_c_raw;
	}
	controller->adc_b_offset = adc_b_offset/n;
	controller->adc_c_offset = adc_c_offset/n;
}

/*参数初始化*/
void init_controller_params(ControllerStruct *controller)
{
	controller->ki_d = KI_D;
	controller->ki_q = KI_Q;
	controller->k_d = K_SCALE*I_BW;
	controller->k_q = K_SCALE*I_BW;
	controller->alpha = 1.0f - 1.0f/(1.0f - DT*I_BW*PI_BY2_F);
	controller->ki_fw = .1f*controller->ki_d;
	controller->phase_order = PHASE_ORDER;
	
	if(I_MAX <= 40.0f)
	{
		controller->i_scale = I_SCALE;
	}
	else
	{
		controller->i_scale = 2.0f*I_SCALE;
	}
	
	// Approximate duty cycle linearization
	for(int i = 0; i<128; i++)	
	{
			controller->inverter_tab[i] = 1.0f + 1.2f*exp(-0.0078125f*i/.032f);
	}
}

void reset_foc(ControllerStruct *controller)
{
	timer_channel_output_pulse_value_config(TIMER0, TIM_CH_U, SVPWM_PERIOD * 0.5f);
	timer_channel_output_pulse_value_config(TIMER0, TIM_CH_V, SVPWM_PERIOD * 0.5f);
	timer_channel_output_pulse_value_config(TIMER0, TIM_CH_W, SVPWM_PERIOD * 0.5f);

	controller->i_d_des = 0;
	controller->i_q_des = 0;
	controller->i_d = 0;
	controller->i_q = 0;
	controller->i_q_filt = 0;
	controller->q_int = 0;
	controller->d_int = 0;
	controller->v_q = 0;
	controller->v_d = 0;
	controller->fw_int = 0;
	controller->otw_flag = 0;
}

/*重置观测器*/
void reset_observer(ObserverStruct *observer){
/*
    observer->temperature = 25.0f;
    observer->temp_measured = 25.0f;
    //observer->resistance = .1f;
*/
}

/*更新观测器*/
void update_observer(ControllerStruct *controller, ObserverStruct *observer)
{
	/*
    /// Update observer estimates ///
    // Resistance observer //
    // Temperature Observer //
    observer->delta_t = (float)observer->temperature - T_AMBIENT;
    float i_sq = controller->i_d*controller->i_d + controller->i_q*controller->i_q;
    observer->q_in = (R_NOMINAL*1.5f)*(1.0f + .00393f*observer->delta_t)*i_sq;
    observer->q_out = observer->delta_t*R_TH;
    observer->temperature += (INV_M_TH*DT)*(observer->q_in-observer->q_out);

    //float r_d = (controller->v_d*(DTC_MAX-DTC_MIN) + SQRT3*controller->dtheta_elec*(L_Q*controller->i_q))/(controller->i_d*SQRT3);
    float r_q = (controller->v_q*(DTC_MAX-DTC_MIN) - SQRT3*controller->dtheta_elec*(L_D*controller->i_d + WB))/(controller->i_q*SQRT3);
    observer->resistance = r_q;//(r_d*controller->i_d + r_q*controller->i_q)/(controller->i_d + controller->i_q); // voltages more accurate at higher duty cycles

    //observer->resistance = controller->v_q/controller->i_q;
    if(isnan(observer->resistance) || isinf(observer->resistance)){observer->resistance = R_NOMINAL;}
    float t_raw = ((T_AMBIENT + ((observer->resistance/R_NOMINAL) - 1.0f)*254.5f));
    if(t_raw > 200.0f){t_raw = 200.0f;}
    else if(t_raw < 0.0f){t_raw = 0.0f;}
    observer->temp_measured = .999f*observer->temp_measured + .001f*t_raw;
    float e = (float)observer->temperature - observer->temp_measured;
    observer->trust = (1.0f - .004f*fminf(abs(controller->dtheta_elec), 250.0f)) * (.01f*(fminf(i_sq, 100.0f)));
    observer->temperature -= observer->trust*.0001f*e;
    //printf("%.3f\n\r", e);

    if(observer->temperature > TEMP_MAX){controller->otw_flag = 1;}
    else{controller->otw_flag = 0;}
    */
}


/*DTC线性化*/
float linearize_dtc(ControllerStruct *controller, float dtc)
{
	float duty = fast_fmaxf(fast_fminf(fabs(dtc), .999f), 0.0f);;
	int index = (int) (duty*127.0f);
	float val1 = controller->inverter_tab[index];
	float val2 = controller->inverter_tab[index+1];
	return val1 + (val2 - val1)*(duty*128.0f - (float)index);
}

/*Field Weakening*/
void field_weaken(ControllerStruct *controller)
{
	controller->fw_int += controller->ki_fw*(controller->v_max - controller->v_ref);
	controller->fw_int = fast_fmaxf(fast_fminf(controller->fw_int, 0.0f), -I_FW_MAX);
	controller->i_d_des = controller->fw_int;
	float q_max = sqrtf(controller->i_max*controller->i_max - controller->i_d_des*controller->i_d_des);
	controller->i_q_des = fast_fmaxf(fast_fminf(controller->i_q_des, q_max), -q_max);
}


/*磁场定向控制FOC*/
void commutate(ControllerStruct *controller, EncoderStruct *encoder)
{
	controller->theta_elec = encoder->elec_angle;
	controller->dtheta_elec = encoder->elec_velocity;
	controller->dtheta_mech = encoder->velocity/GR;
	controller->theta_mech = encoder->angle_multiturn[0]/GR;

	//abc坐标系电流变换到dq坐标系电流
	abc_to_dq(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);    //abc_to_dq transform on currents - 3.8 us

	controller->i_q_filt = (1.0f-CURRENT_FILT_ALPHA)*controller->i_q_filt + CURRENT_FILT_ALPHA*controller->i_q;	// 这些不用于控制，但有时很适合调试
	controller->i_d_filt = (1.0f-CURRENT_FILT_ALPHA)*controller->i_d_filt + CURRENT_FILT_ALPHA*controller->i_d;
	controller->v_bus_filt = (1.0f-VBUS_FILT_ALPHA)*controller->v_bus_filt + VBUS_FILT_ALPHA*controller->v_bus;	// 用于电压饱和

	controller->v_max = OVERMODULATION*controller->v_bus_filt*(DTC_MAX-DTC_MIN)*SQRT3_DIV3;
	controller->i_max = I_MAX; //I_MAX*(!controller->otw_flag) + I_MAX_CONT*controller->otw_flag;

	//将i_d_des和i_q_des缩放到i_max范围内
	limit_norm(&controller->i_d_des, &controller->i_q_des, controller->i_max);	// 2.3 us

	//PI控制器
	float i_d_error = controller->i_d_des - controller->i_d;
	float i_q_error = controller->i_q_des - controller->i_q;

	//计算解耦前馈电压
	float v_d_ff = 0.0f;//-controller->dtheta_elec*L_Q*controller->i_q;
	float v_q_ff = 0.0f;//controller->dtheta_elec*L_D*controller->i_d;

	controller->v_d = controller->k_d*i_d_error + controller->d_int + v_d_ff;

	controller->v_d = fast_fmaxf(fast_fminf(controller->v_d, controller->v_max), -controller->v_max);

	controller->d_int += controller->k_d*controller->ki_d*i_d_error;
	controller->d_int = fast_fmaxf(fast_fminf(controller->d_int, controller->v_max), -controller->v_max);
	float vq_max = sqrtf(controller->v_max*controller->v_max - controller->v_d*controller->v_d);

	controller->v_q = controller->k_q*i_q_error + controller->q_int + v_q_ff;
	controller->q_int += controller->k_q*controller->ki_q*i_q_error;
	controller->q_int = fast_fmaxf(fast_fminf(controller->q_int, controller->v_max), -controller->v_max);
	controller->v_ref = sqrtf(controller->v_d*controller->v_d + controller->v_q*controller->v_q);
	controller->v_q = fast_fmaxf(fast_fminf(controller->v_q, vq_max), -vq_max);

	//将v_d和v_q缩放到v_max范围内
	limit_norm(&controller->v_d, &controller->v_q, controller->v_max);

	//abc坐标系电压变换到dq坐标系电压
	dq_to_abc(controller->theta_elec + 1.5f*DT*controller->dtheta_elec, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w);
	svm(controller->v_max, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w); //space vector modulation

	set_dtc(controller);

}

/*转矩控制*/
void torque_control(ControllerStruct *controller)
{
	float torque_des = controller->kp*(controller->p_des - controller->theta_mech) + controller->t_ff + controller->kd*(controller->v_des - controller->dtheta_mech);
	controller->i_q_des = fast_fmaxf(fast_fminf(torque_des/(KT*GR), controller->i_max), -controller->i_max);
	controller->i_d_des = 0.0f;
}

/*控制量清零*/
void zero_commands(ControllerStruct * controller){
	controller->t_ff = 0;
	controller->kp = 0;
	controller->kd = 0;
	controller->p_des = 0;
	controller->v_des = 0;
	controller->i_q_des = 0;
}
