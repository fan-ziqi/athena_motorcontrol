#include "tim.h"

void TIM0_Init(void)
{
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;
    timer_break_parameter_struct timer_breakpara;

    timer_deinit(TIMER0);

    /* TIMER0 configuration */
    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler         = 0;
    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_DOWN;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = SVPWM_PERIOD;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 1;
    timer_init(TIMER0, &timer_initpara);

    /* Clock source */
    timer_internal_clock_config(TIMER0);

    timer_master_output_trigger_source_select(TIMER0, TIMER_TRI_OUT_SRC_RESET);
    timer_master_slave_mode_config(TIMER0, TIMER_MASTER_SLAVE_MODE_DISABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER0);

    /* Channel output */
    timer_channel_output_struct_para_init(&timer_ocintpara);
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_LOW;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocintpara);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_fast_config(TIMER0, TIMER_CH_0, TIMER_OC_FAST_ENABLE);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 0);

    timer_channel_output_config(TIMER0, TIMER_CH_1, &timer_ocintpara);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_fast_config(TIMER0, TIMER_CH_1, TIMER_OC_FAST_ENABLE);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_1, TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, 0);

    timer_channel_output_config(TIMER0, TIMER_CH_2, &timer_ocintpara);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_fast_config(TIMER0, TIMER_CH_2, TIMER_OC_FAST_ENABLE);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_2, TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, 0);

    /* Break, Deadtime */
    timer_break_struct_para_init(&timer_breakpara);
    timer_breakpara.runoffstate      = TIMER_ROS_STATE_DISABLE;
    timer_breakpara.ideloffstate     = TIMER_IOS_STATE_DISABLE ;
    timer_breakpara.deadtime         = 0;
    timer_breakpara.breakpolarity    = TIMER_BREAK_POLARITY_HIGH;
    timer_breakpara.outputautostate  = TIMER_OUTAUTO_DISABLE;
    timer_breakpara.protectmode      = TIMER_CCHP_PROT_OFF;
    timer_breakpara.breakstate       = TIMER_BREAK_DISABLE;
    
    timer_break_config(TIMER0, &timer_breakpara);

    timer_primary_output_config(TIMER0,ENABLE);
    
    timer_interrupt_enable(TIMER0, TIMER_INT_UP);
    timer_enable(TIMER0);
}
