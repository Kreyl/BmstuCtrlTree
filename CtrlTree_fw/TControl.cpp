/*
 * TStating.cpp
 *
 *  Created on: 16 мая 2020 г.
 *      Author: layst
 */

#include "TControl.h"
#include "ch.h"
#include "shell.h"
#include "kl_i2c.h"
#include "kl_pid.h"
#include "Device.h"

#define ti2c    i2c1

static PinOutputPWM_t HtrCtrl{HEATER_CTRL};
// Kp,  Ki, MaxI, MinI,  Kd
static PID_t PidHtr{72, 1, 306, -306, 0};
enum Mode_t { modeDisabled, modeMeasure, modeControl };
static Mode_t Mode = modeDisabled, NewMode = modeDisabled;

static uint8_t GetT(uint8_t i2cAddr, float* POut) {
    uint8_t PointerReg = 0; // read t reg
    uint8_t IBuf[2];
    if(ti2c.WriteRead(i2cAddr, &PointerReg, 1, IBuf, 2) == retvOk) {
        int16_t r = (((int16_t)IBuf[0]) << 8) | (int16_t)IBuf[1];
        r >>= 7;
        *POut = ((float)r) / 2.0;
        return retvOk;
    }
    else return retvFail;
}

static THD_WORKING_AREA(waTSnsThread, 256);

static void TControlThread(void *arg) {
    chRegSetThreadName("TControl");
    while(true) {
        chThdSleepMilliseconds(540);
        // Check if mode changed
        if(Mode != NewMode) {
            Mode = NewMode;
            if(Mode != modeControl) {
                PidHtr.Reset();
                HtrCtrl.Set(0);
            }
        }

        // Measure
        if(Mode != modeDisabled) { // Measure or Control
            float t1, t2;
            uint8_t r1 = GetT(T_ADDR1, &t1);
            uint8_t r2 = GetT(T_ADDR2, &t2);
            if(r1 == retvOk and r2 == retvOk) {
                tControl::Actual_t = (t1 + t2) / 2;
                tControl::FailString = nullptr;
//                Printf("%.1f %.1f %.1f\r", t1, t2, tControl::Actual_t);
            }
            else { // t reading failed
                if(Mode == modeControl) {
                    PidHtr.Reset();
                    HtrCtrl.Set(0);
                    Mode = modeMeasure;
                }
                if     (r1 != retvOk and r2 == retvOk) tControl::FailString = "Fail:tSns1";
                else if(r1 == retvOk and r2 != retvOk) tControl::FailString = "Fail:tSns2";
                else                                   tControl::FailString = "Fail:tSns1+tSns2";
            }
        } // if not disabled

        // Control
        if(Mode == modeControl) {
            float reg = PidHtr.Calculate(Settings.TargetT, tControl::Actual_t);
            if(reg < 0) reg = 0;
            else if(reg > HEATER_MAX_V) reg = HEATER_MAX_V;
            uint32_t h = (uint32_t)reg;
            HtrCtrl.Set(h);
        }
    } // while true
}

namespace tControl {
// Variables
float Actual_t = 25;
const char* FailString = nullptr;

void Init() {
    HtrCtrl.Init();
    // Create and start thread
    chThdCreateStatic(waTSnsThread, sizeof(waTSnsThread), NORMALPRIO, (tfunc_t)TControlThread, NULL);
}

void SetHeater(uint32_t Value) { HtrCtrl.Set(Value); }

void SetModeDisabled() { NewMode = modeDisabled; }
void SetModeMeasure()  { NewMode = modeMeasure; }
void SetModeControl()  { NewMode = modeControl; }

} // namespace
