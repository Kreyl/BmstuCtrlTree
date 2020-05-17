/*
 * TStating.h
 *
 *  Created on: 16 мая 2020 г.
 *      Author: layst
 */

#pragma once

#include <inttypes.h>

#define T_ADDR1     0x48
#define T_ADDR2     0x4A


namespace tControl {
    extern float Actual_t;
    extern const char* FailString;

    void Init();
    void SetHeater(uint32_t Value);
    void SetModeDisabled();
    void SetModeMeasure();
    void SetModeControl();
}

