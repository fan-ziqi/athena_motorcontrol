#include "gd32f30x.h"
#define FMC_PAGE_SIZE    ((uint16_t)0x800)
#define DATA_START_BASE  ((uint32_t)0x08000000 + 120 * FMC_PAGE_SIZE)

#include "flash_writer.h"




void flash_writer_init(FlashWriter *fw, uint32_t sector) {
	if(sector>7) sector = 7;
	fw->sector = sector;
  fw->base = DATA_START_BASE; // start 240k, 16k data
	fw->ready = false;
}
bool flash_writer_ready(FlashWriter fw) {
    return fw.ready;
}

void flash_writer_open(FlashWriter * fw) {
    fmc_unlock();
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_END);
    fmc_page_erase(fw->base);
    fw->ready = true;
}

void flash_writer_write_int(FlashWriter fw, uint32_t index, int x) {
    union UN {int a; uint32_t b;};
    union UN un;
    un.a = x;
    fmc_word_program(fw.base + 4 * index, un.b);
}

void flash_writer_write_uint(FlashWriter fw, uint32_t index, unsigned int x) {
    fmc_word_program(fw.base + 4 * index, x);
}

void flash_writer_write_float(FlashWriter fw, uint32_t index, float x) {
    union UN {float a; uint32_t b;};
    union UN un;
    un.a = x;
    fmc_word_program(fw.base + 4 * index, un.b);
}

void flash_writer_close(FlashWriter * fw) {
    fmc_lock();
    fw->ready = false;
}

int flash_read_int(FlashWriter fw, uint32_t index) {
    return *(int*) (fw.base + 4 * index);
}

uint32_t flash_read_uint(FlashWriter fw, uint32_t index) {
    return *(uint32_t*) (fw.base + 4 * index);;
}

float flash_read_float(FlashWriter fw, uint32_t index) {
    return *(float*) (fw.base + 4 * index);
}


