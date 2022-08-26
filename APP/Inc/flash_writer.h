/*
 * flash_writer.h
 *
 *  Created on: Apr 12, 2020
 *      Author: Ben
 */

#ifndef INC_FLASH_WRITER_H_
#define INC_FLASH_WRITER_H_

#include <stdint.h>
#include <stdbool.h>


typedef struct {
	bool ready;
	uint32_t base;
	uint32_t sector;
}FlashWriter;

void flash_writer_init(FlashWriter *fw, uint32_t sector);
bool flash_writer_ready(FlashWriter fw);
void flash_writer_open(FlashWriter *fw);
void flash_writer_write_int(FlashWriter fw, uint32_t index, int x);
void flash_writer_write_uint(FlashWriter fw, uint32_t index, unsigned int x);
void flash_writer_write_float(FlashWriter fw, uint32_t index, float x);
void flash_writer_close(FlashWriter *fw);
int flash_read_int(FlashWriter fw, uint32_t index);
uint32_t flash_read_uint(FlashWriter fw, uint32_t index);
float flash_read_float(FlashWriter fw, uint32_t index);


#endif /* INC_FLASH_WRITER_H_ */
