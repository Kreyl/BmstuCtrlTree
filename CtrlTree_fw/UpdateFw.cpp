/*
 * UpdateFw.cpp
 *
 *  Created on: May 5, 2020
 *      Author: layst
 */

#include "kl_crc.h"
#include "UpdateFw.h"
#include "kl_lib.h"
#include <string.h> // for memcpy
#include "led.h"
#include "Sequences.h"

// Constants, see datasheet
#define FLASH_START_ADDR        0x08000000UL
#define TOTAL_FLASH_SZ          256000UL
#define PAGE_SZ                 2048L       // bytes in page, see datasheet
#define PAGE_SZ_WORD64          (FLASH_PAGE_SIZE/8)

// Flash map
#define BOOTLOADER_MAX_SZ       0x4000UL    // 16k bytes for bootloader
#define FLASH_APP_MAX_SZ        0x18000UL   // 96k bytes for app
#define FLASH_AREA1_ADDR        (FLASH_START_ADDR + BOOTLOADER_MAX_SZ)
#define FLASH_AREA2_ADDR        (FLASH_AREA1_ADDR + FLASH_APP_MAX_SZ)
#define FLASH_SETTINGS_ADDR     (FLASH_START_ADDR + 0x3F800UL) // Page 127

uint32_t ApplicationAddr;
extern LedBlinker_t Led;

//#define FW_SETTINGS_SZ_WORD64   2UL // 64 bytes
//static
//union FwSettings_t {
//    uint64_t DWord64[FW_SETTINGS_SZ_WORD64];
//    struct {
//        uint32_t AreaAddrToStartFrom;
//    };
//} FwSettings;

uint8_t UpdateFw(uint8_t *ptr, uint32_t Len, uint16_t CrcIn) {
    // Check crc
    uint16_t crc = Crc::CalculateCRC16HW(ptr, Len);
    if(crc != CrcIn) return retvCRCError;
    // Where to write?
    memcpy(&ApplicationAddr, (const void*)FLASH_SETTINGS_ADDR, sizeof(ApplicationAddr));
    if(ApplicationAddr != FLASH_AREA1_ADDR and ApplicationAddr != FLASH_AREA2_ADDR) {
        ApplicationAddr = FLASH_AREA1_ADDR;
    }
    // ==== Write file to flash ====
    uint32_t *Buf = (uint32_t*)ptr;
    uint32_t WAddr = (ApplicationAddr == FLASH_AREA1_ADDR)? FLASH_AREA2_ADDR : FLASH_AREA1_ADDR;
    ApplicationAddr = WAddr; // To save it later
    // Unlock flash
    chSysLock();
    Flash::LockFlash(); // Otherwise HardFault occurs after flashing and without reset
    Flash::UnlockFlash();
    chSysUnlock();
    Led.StartOrRestart(lsqWriting);
    uint8_t Rslt = retvOk;
    while(Len) {
        // Erase page
        uint32_t PageAddr = (WAddr - FLASH_START_ADDR) / PAGE_SZ;
        if(Flash::ErasePage(PageAddr) != retvOk) {
            Printf("\rErase %X fail\r", WAddr);
            Rslt = retvFail;
            break;
        }
        // Write memory
        uint32_t BytesCnt = MIN_(Len, PAGE_SZ);
        if(Flash::ProgramBuf32(WAddr, Buf, BytesCnt) != retvOk) {
            Printf("Write %X Fail\r", WAddr);
            Rslt = retvFail;
            break;
        }
        Printf(".");
        WAddr += BytesCnt;
        Len -= BytesCnt;
    } // while
    // Switch to new area
    if(Rslt == retvOk) {
        uint32_t PageAddr = (FLASH_SETTINGS_ADDR - FLASH_START_ADDR) / PAGE_SZ;
        if(Flash::ErasePage(PageAddr) != retvOk) {
            Printf("\rErase %X fail\r", FLASH_SETTINGS_ADDR);
            return retvFail;
        }
        if(Flash::ProgramBuf32(FLASH_SETTINGS_ADDR, &ApplicationAddr, sizeof(ApplicationAddr)) != retvOk) {
            Printf("Write %X Fail\r", FLASH_SETTINGS_ADDR);
            return retvFail;
        }
        Printf("*");
    }
    Printf("\r\n");
    return Rslt;
}

