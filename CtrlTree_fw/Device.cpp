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

static const Device_t EmptyDevice;
extern EE_t ee;
extern CmdUart_t Uart;

#if 1 // ============================ Device ===================================
Device_t::Device_t(uint8_t AAddr, uint8_t AType, const char* AName) {
    Addr = AAddr;
    Type = AType;
    strcpy(Name, AName);
}

void Device_t::Print(Shell_t *PShell, const char* S) const {
    PShell->Print("%u, %u, %S%S", Addr, Type, Name, S);
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

void DeviceList_t::Add(uint8_t Addr, uint8_t Type, char* Name) {
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
//    uint8_t tmp = 0;
//    if(ee.Read<uint8_t>(0, &tmp) == retvOk) {
//        if(tmp >= 0 and tmp <= DEV_CNT_MAX) Cnt = tmp;
//        else Cnt = 0;
//    }
//    else {
//        Printf("EEPROM error\r\n");
//        return;
//    }


}

void DeviceList_t::Save() {

}

#endif
