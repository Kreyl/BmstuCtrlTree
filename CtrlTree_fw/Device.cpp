/*
 * Device.cpp
 *
 *  Created on: May 2, 2020
 *      Author: layst
 */

#include "kl_lib.h"
#include "Device.h"
#include "ee_i2c.h"
#include "uart2.h"
#include "shell.h"
#include "kl_i2c.h"

static const Device_t EmptyDevice;
EE_t ee{&i2c1};

Device_t SelfInfo;
Settings_t Settings;

bool AddrIsOk(int32_t Addr) {
    return (Addr >= ADDR_MIN and Addr <= ADDR_MAX);
}

bool TypeIsOk(uint8_t AType) {
    return (AType >= (uint8_t)devtHFBlock and AType <= (uint8_t)devtIKS);
}

#if 1 // ============================ Device ===================================
Device_t::Device_t(uint8_t AAddr, DevType_t AType, const char* AName) {
    Addr = AAddr;
    Type = AType;
    strcpy(Name, AName);
}

void Device_t::Print(Shell_t *PShell, const char* S) const {
    PShell->Print("%u, %u, %S%S", (Type == devtHFBlock)? 0:Addr, Type, Name, S);
}


uint8_t Device_t::Save(uint32_t MemAddr) const {
    return ee.Write(MemAddr, (void*)this, sizeof(Device_t));
}
uint8_t Device_t::Load(uint32_t MemAddr) {
    if(ee.Read(MemAddr, (void*)this, sizeof(Device_t)) == retvOk) {
        return (AddrIsOk(Addr) and TypeIsOk((uint8_t)Type))? retvOk : retvFail;
    }
    else return retvFail;
}

uint8_t Device_t::Check() const {
    // Todo
    return retvOk;
}

#endif

#if 1 // ======================== DeviceList ===================================
Device_t& DeviceList_t::operator[](const int32_t AIndx) {
    if(AIndx >= Cnt()) return *((Device_t*)&EmptyDevice);
    else return IList[AIndx];
}

Device_t* DeviceList_t::GetByAddr(uint8_t Addr) {
    for(Device_t& Dev : IList) {
        if(Dev.Addr == Addr) return &Dev;
    }
    return nullptr;
}

bool DeviceList_t::ContainsAddr(uint8_t Addr) {
    for(Device_t& Dev : IList) {
        if(Dev.Addr == Addr) return true;
    }
    return false;
}

void DeviceList_t::Add(uint8_t Addr, DevType_t Type, char* Name) {
    IList.emplace_back(Addr, Type, Name);
}

uint8_t DeviceList_t::Delete(uint8_t Addr) {
    for(uint32_t i=0; i<IList.size(); i++) {
        if(IList[i].Addr == Addr) {
            IList.erase(IList.begin() + i);
            return retvOk;
        }
    }
    return retvNotFound;
}

void DeviceList_t::Load() {
    uint8_t Cnt = 0;
    uint32_t MemAddr = EE_DEVLIST_ADDR;
    if(ee.Read<uint8_t>(MemAddr, &Cnt) != retvOk) return;
    if(Cnt >= DEV_CNT_MAX) return; // too big Cnt
    MemAddr++;
    for(uint8_t i=0; i<Cnt; i++) {
        Device_t Dev;
        if(Dev.Load(MemAddr) != retvOk) return;
        IList.push_back(Dev);
        MemAddr += sizeof(Dev);
    }
}

void DeviceList_t::Save() {
    uint32_t MemAddr = EE_DEVLIST_ADDR;
    uint8_t ACnt = IList.size();
    if(ee.Write<uint8_t>(MemAddr, &ACnt) != retvOk) return;
    MemAddr++;
    for(Device_t& Dev : IList) {
        if(Dev.Save(MemAddr) != retvOk) return;
        MemAddr += sizeof(Dev);
    }
}

#endif

#if 1 // ========================= Settings ====================================
uint8_t Settings_t::Load() {
    return ee.Read(EE_SETTINGS_ADDR, (void*)this, sizeof(Settings_t));
}

uint8_t Settings_t::Save() {
    return ee.Write(EE_SETTINGS_ADDR, (void*)this, sizeof(Settings_t));
}
#endif
