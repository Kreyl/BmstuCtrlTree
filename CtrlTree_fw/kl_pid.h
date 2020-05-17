/*
 * kl_pid.h
 *
 *  Created on: 17 ���� 2016 �.
 *      Author: Kreyl
 */

#pragma once

#include "shell.h"

#define MAX_RSLT    1000.0

class PID_t {
private:
    float OldErr;   // Required for diff calculations
    float Integral, MaxI, MinI;
    // PID coeffs. "0" means "disabled".
    float Kp;
    float Ki;
    float Kd;
public:
    float Calculate(float TargetValue, float NewValue) {
        float Err = TargetValue - NewValue;
        float Rslt = 0, integ = 0, dif = 0;
        // Proportional
        Rslt += Kp * Err;
        // Integral
        if(Ki != 0) {
            Integral += Err;
            if(Integral > MaxI) Integral = MaxI;
            else if(Integral < MinI) Integral = MinI;
            integ = Ki * Integral;
            Rslt += integ;
        }
        // Differential
        if(Kd != 0) {
            dif = Kd * (Err - OldErr);
            Rslt += dif;
            OldErr = Err;   // Save current value
        }
        // Output limitation
        if(Rslt > MAX_RSLT) Rslt = MAX_RSLT;
        else if(Rslt < -MAX_RSLT) Rslt = -MAX_RSLT;
#if 0 // Verbose
//        Printf("t=%.1f; Err=%.1f; Rslt=%.1f\r\n", NewValue, Err, Rslt);
        Printf("%.1f; %.1f; %.1f", NewValue, Err, Rslt);
//        Printf("%.1f, %.1f", Err, Rslt);
        if(Kd != 0) Printf("; %.1f", dif);
        if(Ki != 0) Printf("; %.1f", integ);
        Printf("\r\n");
#endif
        return Rslt;
    }

    void Reset() {
        OldErr = 0;
        Integral = 0;
    }

    PID_t(float AKp, float AKi, float AMaxI, float AMinI, float AKd) :
        OldErr(0),
        Integral(0), MaxI(AMaxI), MinI(AMinI),
        Kp(AKp), Ki(AKi), Kd(AKd) {}
};
