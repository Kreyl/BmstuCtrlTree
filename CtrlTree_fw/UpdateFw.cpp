/*
 * UpdateFw.cpp
 *
 *  Created on: May 5, 2020
 *      Author: layst
 */

#include "kl_crc.h"
#include "UpdateFw.h"
#include "kl_lib.h"
#include "shell.h"

// Constants, see datasheet
#define FLASH_START_ADDR        0x08000000UL
#define PAGE_SZ                 2048L       // bytes in page, see datasheet
#define PAGE_SZ_WORD64          (FLASH_PAGE_SIZE/8)
#define FLASH_BANK2_ADDR        0x08080000UL    // For L476xG with 1MB of Flash

uint8_t UpdateFw(uint8_t *ptr, int32_t Len, uint16_t CrcIn) {
    // Check crc
    uint16_t crc = Crc::CalculateCRC16HWDMA(ptr, Len);
    if(crc != CrcIn) return retvCRCError;

    // Unlock flash
    chSysLock();
    Flash::LockFlash(); // Otherwise HardFault occurs after flashing and without reset
    Flash::UnlockFlash();
    chSysUnlock();

    // ==== Erase other bank ====
    while(FLASH->SR & FLASH_SR_BSY); // Wait for flash to become idle
    FLASH->SR |= 0xC3FA; // Clean err flags
    // Select bank to clear
    if(FLASH->OPTR & FLASH_OPTR_BFB2) FLASH->CR |= FLASH_CR_MER1; // if current bank is B, clean A
    else FLASH->CR |= FLASH_CR_MER2; // else clean B
    // Clean it
    FLASH->CR |= FLASH_CR_STRT;
    while(FLASH->SR & FLASH_SR_BSY); // Wait for flash to become idle
    FLASH->CR &= ~(FLASH_CR_MER1 | FLASH_CR_MER2); // Clear MassErase flags

    // ==== Do it ====
    uint32_t Address = FLASH_BANK2_ADDR;
    uint32_t *p32 = (uint32_t*)ptr;
    uint32_t *PEnd = (uint32_t*)(ptr + Len);
    // Prepare fast write
    while(FLASH->SR & FLASH_SR_BSY); // Wait for flash to become idle
    FLASH->SR |= 0xC3FB; // Clean err flags
    FLASH->CR |= FLASH_CR_FSTPG;
    uint8_t Rslt = retvOk;
    // Write
    while(p32 < PEnd and Rslt == retvOk) {
        chSysLock();
        for(int i=0; i<32; i++) { // Write 32 DWords
            *(volatile uint32_t*)Address = *p32++;
            Address += 4;
            *(volatile uint32_t*)Address = *p32++;
            Address += 4;
            // Check for errors
            if(FLASH->SR & 0xC3FB) {
                Rslt = retvFail;
                PrintfI("SR: %X\r", FLASH->SR);
                break;
            }
        } // for
        chSysUnlock();
        while(FLASH->SR & FLASH_SR_BSY);
        if(FLASH->SR & FLASH_SR_EOP) FLASH->SR |= FLASH_SR_EOP;
    } // while
    FLASH->CR &= ~FLASH_CR_FSTPG;  // Disable flash writing
    Flash::LockFlash();
    return Rslt;
}

