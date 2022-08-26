/*
 * drv8323.c
 *
 *  Created on: Aug 1, 2020
 *      Author: ben
 */


#include "drv8323.h"
#include <stdio.h>
#include "structs.h"
#include "hw_config.h"
#include "user_config.h"

uint16_t drv_spi_write(DRVStruct * drv, uint16_t val){

	drv->spi_tx_word = val;

	SPI_SET_NSS_LOW(DRV_CS);

	spi_transmit_receive(DRV_SPI, &drv->spi_tx_word, &drv->spi_rx_word);

	SPI_SET_NSS_HIGH(DRV_CS);

	return drv->spi_rx_word;

}
uint16_t drv_read_FSR1(DRVStruct drv){
	return drv_spi_write(&drv, (1 << 15) | (FSR1 << 11));
}

uint16_t drv_read_FSR2(DRVStruct drv){
	return drv_spi_write(&drv, (1 << 15) | (FSR2 << 11));
}

uint16_t drv_read_register(DRVStruct drv, int reg){
	return drv_spi_write(&drv, (1<<15)|(reg<<11));
}
void drv_write_register(DRVStruct drv, int reg, int val){
	drv_spi_write(&drv, (reg<<11)|val);
}
void drv_write_DCR(DRVStruct drv, int DIS_CPUV, int DIS_GDF, int OTW_REP, int PWM_MODE, int PWM_COM, int PWM_DIR, int COAST, int BRAKE, int CLR_FLT){
	uint16_t val = (DCR<<11) | (DIS_CPUV<<9) | (DIS_GDF<<8) | (OTW_REP<<7) | (PWM_MODE<<5) | (PWM_COM<<4) | (PWM_DIR<<3) | (COAST<<2) | (BRAKE<<1) | CLR_FLT;
	drv_spi_write(&drv, val);
}
void drv_write_HSR(DRVStruct drv, int LOCK, int IDRIVEP_HS, int IDRIVEN_HS){
	uint16_t val = (HSR<<11) | (LOCK<<8) | (IDRIVEP_HS<<4) | IDRIVEN_HS;
	drv_spi_write(&drv, val);
}
void drv_write_LSR(DRVStruct drv, int CBC, int TDRIVE, int IDRIVEP_LS, int IDRIVEN_LS){
	uint16_t val = (LSR<<11) | (CBC<<10) | (TDRIVE<<8) | (IDRIVEP_LS<<4) | IDRIVEN_LS;
	drv_spi_write(&drv, val);
}
void drv_write_OCPCR(DRVStruct drv, int TRETRY, int DEAD_TIME, int OCP_MODE, int OCP_DEG, int VDS_LVL){
	uint16_t val = (OCPCR<<11) | (TRETRY<<10) | (DEAD_TIME<<8) | (OCP_MODE<<6) | (OCP_DEG<<4) | VDS_LVL;
	drv_spi_write(&drv, val);
}
void drv_write_CSACR(DRVStruct drv, int CSA_FET, int VREF_DIV, int LS_REF, int CSA_GAIN, int DIS_SEN, int CSA_CAL_A, int CSA_CAL_B, int CSA_CAL_C, int SEN_LVL){
	uint16_t val = (CSACR<<11) | (CSA_FET<<10) | (VREF_DIV<<9) | (LS_REF<<8) | (CSA_GAIN<<6) | (DIS_SEN<<5) | (CSA_CAL_A<<4) | (CSA_CAL_B<<3) | (CSA_CAL_C<<2) | SEN_LVL;
	drv_spi_write(&drv, val);
}
void drv_enable_gd(DRVStruct drv){
	uint16_t val = (drv_read_register(drv, DCR)) & (~(0x1<<2));
	drv_write_register(drv, DCR, val);
}
void drv_disable_gd(DRVStruct drv){
	uint16_t val = (drv_read_register(drv, DCR)) | (0x1<<2);
	drv_write_register(drv, DCR, val);
}
void drv_calibrate(DRVStruct drv){
	uint16_t val = (0x1<<4) + (0x1<<3) + (0x1<<2);
	drv_write_register(drv, CSACR, val);
}
void drv_print_faults(DRVStruct drv, uint32_t loop_count){
    uint16_t val1 = drv_read_FSR1(drv);
    uint16_t val2 = drv_read_FSR2(drv);

	if ((val1 | val2) > 0) {
		printf("loop_count: %lu\r\n", loop_count);
	}

    if(val1 & (1<<10)){printf("\n\rFAULT\n\r");}

    if(val1 & (1<<9)){printf("VDS_OCP\n\r");}
    if(val1 & (1<<8)){printf("GDF\n\r");}
    if(val1 & (1<<7)){printf("UVLO\n\r");}
    if(val1 & (1<<6)){printf("OTSD\n\r");}
    if(val1 & (1<<5)){printf("VDS_HA\n\r");}
    if(val1 & (1<<4)){printf("VDS_LA\n\r");}
    if(val1 & (1<<3)){printf("VDS_HB\n\r");}
    if(val1 & (1<<2)){printf("VDS_LB\n\r");}
    if(val1 & (1<<1)){printf("VDS_HC\n\r");}
    if(val1 & (1)){printf("VDS_LC\n\r");}

    if(val2 & (1<<10)){printf("SA_OC\n\r");}
    if(val2 & (1<<9)){printf("SB_OC\n\r");}
    if(val2 & (1<<8)){printf("SC_OC\n\r");}
    if(val2 & (1<<7)){printf("OTW\n\r");}
    if(val2 & (1<<6)){printf("CPUV\n\r");}
    if(val2 & (1<<5)){printf("VGS_HA\n\r");}
    if(val2 & (1<<4)){printf("VGS_LA\n\r");}
    if(val2 & (1<<3)){printf("VGS_HB\n\r");}
    if(val2 & (1<<2)){printf("VGS_LB\n\r");}
    if(val2 & (1<<1)){printf("VGS_HC\n\r");}
    if(val2 & (1)){printf("VGS_LC\n\r");}

}

void drv_init_config(DRVStruct drv)
{
	/* DRV8323 setup */

	// Up to 40A use 40X amplifier gain
	// From 40-60A use 20X amplifier gain.  (Make this generic in the future)
	int CSA_GAIN = I_MAX <= 40.0f ? CSA_GAIN_40 : CSA_GAIN_20;

	// Enable DRV8323
	gpio_bit_set(ENABLE_PIN);
	delay_1ms(10);

	// Driver Control, Gate drive fault is disabled, 3x PWM mode, clear latched fault bits
	drv_write_DCR(drv, 0x0, DIS_GDF_EN, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, CLR_FLT_RST);

	// CSA Control, VREF_DIV=2, CSA_GAIN, CSA_CAL_A/B/C, SEN_LVL=0.25v
	drv_write_CSACR(drv, 0x0, VREF_DIV_2, 0x0, CSA_GAIN, 0x0, 0x1, 0x1, 0x1, SEN_LVL_0_25);

	zero_current(&controller);

	// CSA Control, VREF_DIV=2, CSA_GAIN, DIS_SEN, SEN_LVL=0.25v
	drv_write_CSACR(drv, 0x0, VREF_DIV_2, 0x0, CSA_GAIN, 0x1, 0x0, 0x0, 0x0, SEN_LVL_0_25);
	// drv_write_CSACR(drv, 0x0, 0x1, 0x0, CSA_GAIN, 0x0, 0x0, 0x0, 0x0, SEN_LVL_0_25);

	// OCP Contro, TRETRY=50us, DEAD_TIME=50us, OCP_MODE=retry, OCP_DEG=4us, VDS_LVL=0.45v
	drv_write_OCPCR(drv, TRETRY_50US, DEADTIME_50NS, OCP_RETRY, OCP_DEG_4US, VDS_LVL_0_45);

	// all MOSFETs in the Hi-Z state, disable output
	drv_disable_gd(drv);
}

void drv_clear_fault(DRVStruct drv)
{
	uint16_t val = (drv_read_register(drv, DCR)) | CLR_FLT_RST;
	drv_write_register(drv, DCR, val);
}

void EXTI_Init()
{
	gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOA, GPIO_PIN_SOURCE_12);

	exti_init(EXTI_12, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
	exti_interrupt_flag_clear(EXTI_12);
}
