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

tControl_t tControl;

static PinOutputPWM_t HtrCtrl{HEATER_CTRL};
// Kp,  Ki, MaxI, MinI,  Kd
static PID_t PidHtr{72, 1, 306, -306, 0};

uint8_t tSns_t::GetT() {
    uint8_t PointerReg = 0; // read t reg
    uint8_t IBuf[2];
    if(ti2c.WriteRead(i2cAddr, &PointerReg, 1, IBuf, 2) == retvOk) {
        int16_t r = (((int16_t)IBuf[0]) << 8) | (int16_t)IBuf[1];
        r >>= 7;
        t = ((float)r) / 2.0;
        IsOk = true;
        return retvOk;
    }
    else {
        IsOk = false;
        return retvFail;
    }
}

static THD_WORKING_AREA(waTSnsThread, 256);

static void TControlThread(void *arg) {
    chRegSetThreadName("TControl");
    tControl.ITask();
}

void tControl_t::ITask() {
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
            tAvg = 0;
            uint32_t Cnt = 0;
            for(auto &Sns : Sensors) {
                if(Sns.GetT() == retvOk) {
                    Cnt++;
                    tAvg += Sns.t;
                }
            }
            if(Cnt != 0) tAvg /= Cnt;
            if(Cnt == 0 and Mode == modeControl) { // Noone answers
                PidHtr.Reset();
                HtrCtrl.Set(0);
                Mode = modeMeasure;
            }
        } // if not disabled

        // Control
        if(Mode == modeControl) {
            float reg = PidHtr.Calculate(Settings.TargetT, tAvg);
            if(reg < 0) reg = 0;
            else if(reg > HEATER_MAX_V) reg = HEATER_MAX_V;
            uint32_t h = (uint32_t)reg;
            HtrCtrl.Set(h);
        }
    } // while true
}

void tControl_t::Init() {
    HtrCtrl.Init();
    // Create and start thread
    chThdCreateStatic(waTSnsThread, sizeof(waTSnsThread), NORMALPRIO, (tfunc_t)TControlThread, NULL);
}

void tControl_t::SetHeater(uint32_t Value) {
    HtrCtrl.Set(Value);
}
