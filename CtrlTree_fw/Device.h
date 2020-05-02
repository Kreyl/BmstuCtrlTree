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

#define ADDR_MIN        1
#define ADDR_MAX        254
#define DEV_CNT_MAX     32

#define TYPE_MIN        1
#define TYPE_MAX        6

#define DEV_NAME_LEN    15

class Device_t {
public:
    uint8_t Addr;
    uint8_t Type;
    char Name[DEV_NAME_LEN+1];
    void Print(Shell_t *PShell, const char* S) const;
    uint8_t Check() const;
    Device_t() : Addr(0), Type(0), Name("") {}
    Device_t(uint8_t AAddr, uint8_t AType, const char* AName);
} __attribute__((packed));

class DeviceList_t {
private:
    std::deque<Device_t> IList;
public:
    int32_t Cnt() { return IList.size(); }
    Device_t& operator[](const int32_t AIndx);
    bool ContainsAddr(uint8_t Addr);
    Device_t* GetByAddr(uint8_t Addr);
    void Add(uint8_t Addr, uint8_t Type, char* Name);
    uint8_t Delete(uint8_t Addr);
    void Load();
    void Save();
};
