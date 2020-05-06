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

uint8_t UpdateFw(uint8_t *ptr, uint32_t Len, uint16_t CrcIn) {
    // Check crc
//    uint16_t crc = Crc::CalculateCRC16HW(ptr, Len);
//    if(crc != CrcIn) return retvCRCError;
    // Unlock flash
    chSysLock();
    Flash::LockFlash(); // Otherwise HardFault occurs after flashing and without reset
    Flash::UnlockFlash();
    chSysUnlock();

    // ==== Erase other bank ====
    while(FLASH->SR & FLASH_SR_BSY); // Wait for flash to become idle
    // Clean err flags
    FLASH->SR |= FLASH_SR_OPTVERR | FLASH_SR_RDERR | FLASH_SR_FASTERR |
                FLASH_SR_MISERR | FLASH_SR_PGSERR | FLASH_SR_SIZERR |
                FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_PROGERR | FLASH_SR_OPERR;
    // Select bank to clear
    if(FLASH->OPTR & FLASH_OPTR_BFB2) FLASH->CR |= FLASH_CR_MER1; // if current bank is B, clean A
    else FLASH->CR |= FLASH_CR_MER2; // else clean B
    // Clean it
    FLASH->CR |= FLASH_CR_STRT;
    while(FLASH->SR & FLASH_SR_BSY); // Wait for flash to become idle
    FLASH->CR &= ~(FLASH_CR_MER1 | FLASH_CR_MER2); // Clear MassErase flags

    // ==== Do it ====
    uint32_t WAddr = FLASH_BANK2_ADDR;
    while(Len) {
        // Write memory
        uint32_t BytesCnt = MIN_(Len, PAGE_SZ);
        if(Flash::ProgramBuf32(WAddr, (uint32_t*)ptr, BytesCnt) != retvOk) {
            Printf("Write %X Fail: SR=%X\r", WAddr, FLASH->SR);
            return retvFail;
        }
        Printf(".");
        WAddr += BytesCnt;
        Len -= BytesCnt;
        ptr += BytesCnt;
    } // while
    // Switch to new bank
    uint32_t Optr = FLASH->OPTR;
    Optr = (Optr ^ FLASH_OPTR_BFB2) | FLASH_OPTR_DUALBANK; // switch BFB bit and enable dualbank just in case
    chSysLock();
    while(FLASH->SR & FLASH_SR_BSY);
    Flash::UnlockOptionBytes();
    FLASH->OPTR = Optr;
    FLASH->CR |= FLASH_CR_OPTSTRT;
    while(FLASH->SR & FLASH_SR_BSY);
    FLASH->CR |= FLASH_CR_OBL_LAUNCH; // Option byte loading requested
    Flash::LockOptionBytes(); // Will never be here
    chSysUnlock();
    return retvOk;
}

