/*
 * EvtMsgIDs.h
 *
 *  Created on: 21 ���. 2017 �.
 *      Author: Kreyl
 */

#pragma once

enum EvtMsgId_t {
    evtIdNone = 0, // Always

    evtIdEverySecond,

    // UART
    evtIdUartCmdRcvd,
    evtIdUsbCmdRcvd,

    // Usb
    evtIdUsbConnect,
    evtIdUsbDisconnect,
    evtIdUsbReady,
    evtIdUsbNewCmd,
    evtIdUsbInDone,
    evtIdUsbOutDone,

    // Misc periph
    evtIdButtons,
    evtIdAcc,
};
