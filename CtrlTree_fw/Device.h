/*
 * Device.h
 *
 *  Created on: May 2, 2020
 *      Author: layst
 */

#pragma once

#include "uart2.h"
#include "shell.h"
#include <deque>

#define ADDR_MASTER         0
#define ADDR_MIN            0
#define ADDR_MAX            32
#define DEV_CNT_MAX         32
#define DEV_NAME_LEN        15

#define EE_SELF_ADDR        0UL
#define EE_SETTINGS_ADDR    (EE_SELF_ADDR + sizeof(Device_t))
#define EE_DEVLIST_ADDR     128UL

#define SETTINGS_MAX_SZ     (EE_DEVLIST_ADDR - EE_SETTINGS_ADDR)

enum DevType_t : uint8_t {
    devtNone = 0,
    devtHFBlock = 1,
    devtLNA = 2,
    devtKUKonv = 3,
    devtMRL = 4,
    devtTriplexer = 5,
    devtIKS = 6,
};

bool AddrIsOk(int32_t Addr);
bool TypeIsOk(uint8_t AType);

class Device_t {
public:
    uint8_t Addr;
    DevType_t Type;
    char Name[DEV_NAME_LEN+1];
    void Print(Shell_t *PShell, const char* S) const;
    Device_t() : Addr(0), Type(devtNone), Name("") {}
    Device_t(uint8_t AAddr, DevType_t AType, const char* AName);
    uint8_t Save(uint32_t MemAddr) const;
    uint8_t Load(uint32_t MemAddr);
} __attribute__((packed));

class DeviceList_t {
private:
    std::deque<Device_t> IList;
public:
    int32_t Cnt() { return IList.size(); }
    Device_t& operator[](const int32_t AIndx);
    bool ContainsAddr(uint8_t Addr);
    Device_t* GetByAddr(uint8_t Addr);
    void Add(uint8_t Addr, DevType_t Type, char* Name);
    uint8_t Delete(uint8_t Addr);
    uint32_t GetLongestNameLen();
    void Load();
    void Save();
};

struct RegHMC821_t {
    uint32_t Addr : 8;
    uint32_t Value : 24;
} __attribute__((packed));

struct RegAdf5356_t {
    uint32_t Value : 28;
    uint32_t Addr: 4;
} __attribute__((packed));

#define REG_CNT         22

class Settings_t {
public:
    uint32_t PowerOnGPIO = 0;
    float TargetT = 0;
    uint8_t TControlEnabled = 0;
    // Regs
    union {
        RegHMC821_t RegsHMC[REG_CNT];
        RegAdf5356_t RegsADF[REG_CNT];
    };
    uint8_t SavedRegsCnt = 0;
    bool RegsAreSaved() { return SavedRegsCnt and (SavedRegsCnt <= REG_CNT); }
    uint8_t Load();
    uint8_t Save();
    void Reset();
} __attribute__((packed));

extern Device_t SelfInfo;
extern Settings_t Settings;
