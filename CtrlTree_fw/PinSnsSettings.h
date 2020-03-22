/*
 * SnsPins.h
 *
 *  Created on: 17 џэт. 2015 у.
 *      Author: Kreyl
 */

/* ================ Documentation =================
 * There are several (may be 1) groups of sensors (say, buttons and USB connection).
 *
 */

#pragma once

#include "SimpleSensors.h"

#ifndef SIMPLESENSORS_ENABLED
#define SIMPLESENSORS_ENABLED   FALSE
#endif

#if SIMPLESENSORS_ENABLED
#define SNS_POLL_PERIOD_MS      99

// Handlers
extern void ProcessUsbConnect(PinSnsState_t *PState, uint32_t Len);

const PinSns_t PinSns[] = {
        // USB
        {USB_DETECT_PIN, pudPullDown, ProcessUsbConnect},
};
#define PIN_SNS_CNT     countof(PinSns)

#endif  // if enabled
