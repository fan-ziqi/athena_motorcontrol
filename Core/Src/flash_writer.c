#ifdef STM32F446
#include "stm32f4xx_flash.h"
#else
#include "gd32f30x.h"
#define FMC_PAGE_SIZE    ((uint16_t)0x800)
#define DATA_START_BASE  ((uint32_t)0x08000000 + 120 * FMC_PAGE_SIZE)
#endif
#include "flash_writer.h"




void flash_writer_init(FlashWriter *fw, uint32_t sector) {
	if(sector>7) sector = 7;
	fw->sector = sector;
#ifdef STM32F446
	fw->base = __SECTOR_ADDRS[sector];
#else
    fw->base = DATA_START_BASE; // start 240k, 16k data
#endif
	fw->ready = false;
}
bool flash_writer_ready(FlashWriter fw) {
    return fw.ready;
}

void flash_writer_open(FlashWriter * fw) {
#ifdef STM32F446
    FLASH_Unlock();
    FLASH_ClearFlag( FLASH_FLAG_EOP |  FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    FLASH_EraseSector(__SECTORS[fw->sector], VoltageRange_3);
#else
    fmc_unlock();
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_END);
    fmc_page_erase(fw->base);
#endif
    fw->ready = true;
}

void flash_writer_write_int(FlashWriter fw, uint32_t index, int x) {
    union UN {int a; uint32_t b;};
    union UN un;
    un.a = x;
#ifdef STM32F446
    FLASH_ProgramWord(fw.base + 4 * index, un.b);
#else
    fmc_word_program(fw.base + 4 * index, un.b);
#endif
}

void flash_writer_write_uint(FlashWriter fw, uint32_t index, unsigned int x) {
#ifdef STM32F446
    FLASH_ProgramWord(fw.base + 4 * index, x);
#else
    fmc_word_program(fw.base + 4 * index, x);
#endif
}

void flash_writer_write_float(FlashWriter fw, uint32_t index, float x) {
    union UN {float a; uint32_t b;};
    union UN un;
    un.a = x;
#ifdef STM32F446
    FLASH_ProgramWord(fw.base + 4 * index, un.b);
#else
    fmc_word_program(fw.base + 4 * index, un.b);
#endif
}

void flash_writer_close(FlashWriter * fw) {
#ifdef STM32F446
    FLASH_Lock();
#else
    fmc_lock();
#endif
    fw->ready = false;
}

int flash_read_int(FlashWriter fw, uint32_t index) {
#ifdef STM32F446
    return *(int*) (__SECTOR_ADDRS[fw.sector] + 4 * index);
#else
    return *(int*) (fw.base + 4 * index);
#endif
}

uint32_t flash_read_uint(FlashWriter fw, uint32_t index) {
#ifdef STM32F446
    return *(uint32_t*) (__SECTOR_ADDRS[fw.sector] + 4 * index);
#else
    return *(uint32_t*) (fw.base + 4 * index);;
#endif
}

float flash_read_float(FlashWriter fw, uint32_t index) {
#ifdef STM32F446
    return *(float*) (__SECTOR_ADDRS[fw.sector] + 4 * index);
#else
    return *(float*) (fw.base + 4 * index);
#endif
}


