/*
 * position_sensor.c
 *
 *  Created on: Jul 26, 2020
 *      Author: Ben
 */
#include <stdio.h>
#include <string.h>
#include "position_sensor.h"
#include "math_ops.h"
#include "hw_config.h"
#include "user_config.h"

#define PARD_BIT		0x8000
#define READ_BIT		0x4000
#define READ_NOP		(0x0000 | READ_BIT | PARD_BIT)
#define READ_DIAAGC		(0x3FFC | READ_BIT | PARD_BIT)
#define READ_MAG		(0x3FFD | READ_BIT)
#define READ_ANGLEUNC	(0x3FFE | READ_BIT)
#define READ_ANGLECOM	(0x3FFF | READ_BIT | PARD_BIT)

void ps_warmup(EncoderStruct * encoder, int n){

	for (int i = 0; i < n; i++)
	{
		encoder->spi_tx_word = READ_ANGLECOM;

		SPI_SET_NSS_LOW(ENC_CS);

		spi_transmit_receive(ENC_SPI, &encoder->spi_tx_word, &encoder->spi_rx_word);

		SPI_SET_NSS_HIGH(ENC_CS);

#ifdef DEBUG_PS
		uint16_t raw = encoder->spi_rx_word & 0x3FFF;
		info("ANGLECOM: raw: %04x angle: %d\r\n", raw, (int)(360.0f * raw / 0x3FFF));
		delay_1ms(200);
#endif
	}

#ifdef DEBUG_PS

	encoder->spi_tx_word = READ_DIAAGC;

	SPI_SET_NSS_LOW(ENC_CS);

	spi_transmit_receive(ENC_SPI, &encoder->spi_tx_word, &encoder->spi_rx_word);

	SPI_SET_NSS_HIGH(ENC_CS);

	info("DIAAGC: %04x\r\n", encoder->spi_rx_word);

	encoder->spi_tx_word = READ_MAG;

	SPI_SET_NSS_LOW(ENC_CS);

	spi_transmit_receive(ENC_SPI, &encoder->spi_tx_word, &encoder->spi_rx_word);

	SPI_SET_NSS_HIGH(ENC_CS);

	info("MAG: %04x\r\n", encoder->spi_rx_word);

	encoder->spi_tx_word = READ_ANGLEUNC;

	SPI_SET_NSS_LOW(ENC_CS);

	spi_transmit_receive(ENC_SPI, &encoder->spi_tx_word, &encoder->spi_rx_word);

	SPI_SET_NSS_HIGH(ENC_CS);

	info("ANGLEUNC: %04x\r\n", encoder->spi_rx_word);

	encoder->spi_tx_word = READ_ANGLECOM;

	SPI_SET_NSS_LOW(ENC_CS);

	spi_transmit_receive(ENC_SPI, &encoder->spi_tx_word, &encoder->spi_rx_word);

	SPI_SET_NSS_HIGH(ENC_CS);

	info("ANGLECOM: %04x\r\n", encoder->spi_rx_word);
#endif
}

void ps_sample(EncoderStruct * encoder, float dt){
	/* updates EncoderStruct encoder with the latest sample
	 * after elapsed time dt */

	/* Shift around previous samples */
	encoder->old_angle = encoder->angle_singleturn;
	for(int i = N_POS_SAMPLES-1; i>0; i--){encoder->angle_multiturn[i] = encoder->angle_multiturn[i-1];}
	//for(int i = N_POS_SAMPLES-1; i>0; i--){encoder->count_buff[i] = encoder->count_buff[i-1];}
	//memmove(&encoder->angle_multiturn[1], &encoder->angle_multiturn[0], (N_POS_SAMPLES-1)*sizeof(float)); // this is much slower for some reason

	/* SPI read/write */

	encoder->spi_tx_word = READ_ANGLECOM;

	SPI_SET_NSS_LOW(ENC_CS);

	spi_transmit_receive(ENC_SPI, &encoder->spi_tx_word, &encoder->spi_rx_word);

	SPI_SET_NSS_HIGH(ENC_CS);

	encoder->raw = (encoder->spi_rx_word & 0x3FFF) << 2;


	/* Linearization */
	int off_1 = encoder->offset_lut[(encoder->raw)>>9];				// lookup table lower entry
	int off_2 = encoder->offset_lut[((encoder->raw>>9)+1)%128];		// lookup table higher entry
	int off_interp = off_1 + ((off_2 - off_1)*(encoder->raw - ((encoder->raw>>9)<<9))>>9);     // Interpolate between lookup table entries
	encoder->count = encoder->raw + off_interp;


	/* Real angles in radians */
	encoder->angle_singleturn = ((float)(encoder->count-M_ZERO))/((float)ENC_CPR);
	int int_angle = encoder->angle_singleturn;
	encoder->angle_singleturn = PI_BY2_F*(encoder->angle_singleturn - (float)int_angle);
	//encoder->angle_singleturn = PI_BY2_F*fmodf(((float)(encoder->count-M_ZERO))/((float)ENC_CPR), 1.0f);
	encoder->angle_singleturn = encoder->angle_singleturn<0 ? encoder->angle_singleturn + PI_BY2_F : encoder->angle_singleturn;

	encoder->elec_angle = (encoder->ppairs*(float)(encoder->count-E_ZERO))/((float)ENC_CPR);
	int_angle = (int)encoder->elec_angle;
	encoder->elec_angle = PI_BY2_F*(encoder->elec_angle - (float)int_angle);
	//encoder->elec_angle = PI_BY2_F*fmodf((encoder->ppairs*(float)(encoder->count-E_ZERO))/((float)ENC_CPR), 1.0f);
	encoder->elec_angle = encoder->elec_angle<0 ? encoder->elec_angle + PI_BY2_F : encoder->elec_angle;	// Add 2*pi to negative numbers
	/* Rollover */
	int rollover = 0;
	float angle_diff = encoder->angle_singleturn - encoder->old_angle;
	if(angle_diff > PI_F){rollover = -1;}
	else if(angle_diff < -PI_F){rollover = 1;}
	encoder->turns += rollover;
	if(!encoder->first_sample){
		encoder->turns = 0;
		encoder->first_sample = 1;
	}



	/* Multi-turn position */
	encoder->angle_multiturn[0] = encoder->angle_singleturn + PI_BY2_F*(float)encoder->turns;

	/* Velocity */
	/*
	// Attempt at a moving least squares.  Wasn't any better
		float m = (float)N_POS_SAMPLES;
		float w = 1.0f/m;
		float q = 12.0f/(m*m*m - m);
		float c1 = 0.0f;
		float ibar = (m - 1.0f)/2.0f;
		for(int i = 0; i<N_POS_SAMPLES; i++){
			c1 += encoder->angle_multiturn[i]*q*(i - ibar);
		}
		encoder->vel2 = -c1/dt;
*/
	//encoder->velocity = vel2
	encoder->velocity = (encoder->angle_multiturn[0] - encoder->angle_multiturn[N_POS_SAMPLES-1])/(dt*(float)(N_POS_SAMPLES-1));
	encoder->elec_velocity = encoder->ppairs*encoder->velocity;

}

void ps_print(EncoderStruct * encoder, int loop_count){

	#define PS_PRINT_INTERVAL 10000
	static uint32_t ps_print_mark = 0;

	if (loop_count < ps_print_mark + PS_PRINT_INTERVAL) {
		return;
	}

	printf("Raw: %5d", encoder->raw);
	printf("  Linearized Count: %5d", encoder->count);
	printf("  Single Turn:% 4.3f", encoder->angle_singleturn);
	printf("  Multi Turn:% 5.3f", encoder->angle_multiturn[0]);
	printf("  Electrical:% 5.3f", encoder->elec_angle);
	printf("  Turns:% 2d\r\n", encoder->turns);

	ps_print_mark = loop_count;
}
