/*
 * TStating.h
 *
 *  Created on: 16 мая 2020 г.
 *      Author: layst
 */

#pragma once

#include <inttypes.h>

class tSns_t {
public:
    uint8_t i2cAddr;
    float t = 0;
    bool IsOk = true;
    const char* Name;
    tSns_t(uint8_t Addr, const char* AName) : i2cAddr(Addr), Name(AName) {}
    uint8_t GetT();
};

class tControl_t {
private:
    enum Mode_t { modeDisabled, modeMeasure, modeControl };
    Mode_t Mode = modeDisabled, NewMode = modeDisabled;
public:
    tSns_t Sensors[3] = {
            {0x48, "t1"},
            {0x49, "t2"},
            {0x4A, "t3"},
    };
    float tAvg;
    void Init();
    void SetHeater(uint32_t Value);
    void Disable() { NewMode = modeDisabled; }
    void StartMeasure() { NewMode = modeMeasure; }
    void StartControl() { NewMode = modeControl; }
    // Inner use
    void ITask();
};

extern tControl_t tControl;
