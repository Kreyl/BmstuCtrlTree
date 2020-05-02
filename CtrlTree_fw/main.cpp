#include "ch.h"
#include "hal.h"
#include "MsgQ.h"
#include "uart2.h"
#include "shell.h"
#include "usb_cdc.h"
#include "SimpleSensors.h"
#include "led.h"
#include "Sequences.h"
#include "kl_i2c.h"
#include "ee_i2c.h"
#include "Device.h"

#if 1 // ======================== Variables & prototypes =======================
// Forever
bool OsIsInitialized = false;
EvtMsgQ_t<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;
void OnCmd(Shell_t *PShell);
void ITask();
// Hoard of UARTs
static const UartParams_t CmdUartParams(115200, CMD_UART_PARAMS);
static const UartParams_t RS232Params(115200, RS232_PARAMS);
static const UartParams_t RS485ExtParams(115200, RS485EXT_PARAMS);
//static const UartParams_t RS485IntParams(115200, RS485INT_PARAMS);
CmdUart_t Uart{CmdUartParams};
CmdUart485_t RS485Ext{RS485ExtParams, RS485_EXT_TXEN};
//CmdUart485_t RS485Int{RS485IntParams, RS485_INT_TXEN};
CmdUart_t RS232{RS232Params};

bool UsbIsConnected = false;

LedBlinker_t Led{LED_PIN};

DeviceList_t DevList;

static TmrKL_t TmrOneSecond {TIME_MS2I(999), evtIdEverySecond, tktPeriodic}; // Measure battery periodically
EE_t ee{&i2c1};
#endif

int main(void) {
    // Setup clock frequency
    Clk.SetCoreClk(cclk48MHz); // Todo: start HSE depending on
    Clk.SetupSai1Qas48MhzSrc();
    Clk.UpdateFreqValues();
    // Init OS
    halInit();
    chSysInit();
    OsIsInitialized = true;

    // ==== Init hardware ====
    EvtQMain.Init();
    Uart.Init();
    Printf("\r%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));
    Clk.PrintFreqs();

    Led.Init();
    Led.StartOrRestart(lsqCmd);

    // I2C and sns power
    PinSetupOut(SNS_PWR_CTRL, omPushPull);
    PinSetLo(SNS_PWR_CTRL);
    i2c1.Init();
//    i2c1.ScanBus();

    DevList.Load();

    // Uarts
    RS485Ext.Init();
//    RS485Int.Init();
    RS232.Init();

    UsbCDC.Init();
    SimpleSensors::Init();
//    TmrOneSecond.StartOrRestart();

    // Main cycle
    ITask();
}

__noreturn
void ITask() {
    while(true) {
        EvtMsg_t Msg = EvtQMain.Fetch(TIME_INFINITE);
        switch(Msg.ID) {
            case evtIdUartCmdRcvd:
                switch(Msg.Value) {
                    case 1: if(Uart.TryParseRxBuff()     == retvOk) OnCmd((Shell_t*)&Uart); break;
                    case 2: if(RS485Ext.TryParseRxBuff() == retvOk) OnCmd((Shell_t*)&RS485Ext); break;
//                    case 3: if(RS485Int.TryParseRxBuff() == retvOk) OnCmd((Shell_t*)&RS485Int); break;
                    case 5: if(RS232.TryParseRxBuff()    == retvOk) OnCmd((Shell_t*)&RS232); break;
                }
                break;

            case evtIdUsbCmdRcvd:
                OnCmd((Shell_t*)&UsbCDC);
                UsbCDC.SignalCmdProcessed();
                break;

            case evtIdEverySecond:
                Printf("Second\r");
                break;


#if 1 // ======= USB =======
            case evtIdUsbConnect:
                Printf("USB connect\r");
                UsbCDC.Connect();
                break;
            case evtIdUsbDisconnect:
                UsbCDC.Disconnect();
                Printf("USB disconnect\r");
                break;
            case evtIdUsbReady:
                Printf("USB ready\r");
                break;
#endif
            default: break;
        } // switch
    } // while true
}

void ProcessUsbConnect(PinSnsState_t *PState, uint32_t Len) {
    if((*PState == pssRising or *PState == pssHi) and !UsbIsConnected) {
        UsbIsConnected = true;
        EvtQMain.SendNowOrExit(EvtMsg_t(evtIdUsbConnect));
    }
    else if((*PState == pssFalling or *PState == pssLo) and UsbIsConnected) {
        UsbIsConnected = false;
        EvtQMain.SendNowOrExit(EvtMsg_t(evtIdUsbDisconnect));
    }
}

#if 1 // ======================= Command processing ============================
void OnCmd(Shell_t *PShell) {
    Led.StartOrRestart(lsqCmd);
	Cmd_t *PCmd = &PShell->Cmd;
    // Handle command
    if(PCmd->NameIs("Ping")) PShell->Ack(retvOk);
    else if(PCmd->NameIs("Version")) PShell->Print("%S %S\r\n", APP_NAME, XSTRINGIFY(BUILD_TIME));
    else if(PCmd->NameIs("mem")) PrintMemoryInfo();

#if 1 // ==== Common ====
    else if(PCmd->NameIs("SetAddr")) {
        uint8_t FAddr;
        if(PCmd->GetNext<uint8_t>(&FAddr) != retvOk) { PShell->Ack(retvCmdError); return; }
        if(FAddr < ADDR_MIN or FAddr > ADDR_MAX) { PShell->Ack(retvBadValue); return; }
        // Save it
//        if(ee.Write(EE_ADDR_ADDR, &FAddr, 1) == retvOk) {
//            SelfInfo.Addr = FAddr;
//            PShell->Ack(retvOk);
//        }
//        else PShell->Ack(retvFail);
    }
    else if(PCmd->NameIs("GetAddr")) {
//        PShell->Print("Addr %u\r\n", SelfInfo.Addr);
    }

    else if(PCmd->NameIs("SetType")) {
        uint8_t FAddr, FType;
        if(PCmd->GetNext<uint8_t>(&FAddr) != retvOk) return;
//        if(FAddr != SelfInfo.Addr) return; // ignore alien addresses
        if(PCmd->GetNext<uint8_t>(&FType) != retvOk) { PShell->Ack(retvCmdError); return; }
        if(FType < TYPE_MIN or FType > TYPE_MAX) { PShell->Ack(retvBadValue); return; }
        // Save it
//        if(ee.Write(EE_ADDR_TYPE, &FType, 1) == retvOk) {
//            SelfInfo.Type = FType;
//            PShell->Ack(retvOk);
//        }
//        else PShell->Ack(retvFail);
    }

    else if(PCmd->NameIs("SetName")) {
        uint8_t FAddr;
        if(PCmd->GetNext<uint8_t>(&FAddr) != retvOk) return;
//        if(FAddr != SelfInfo.Addr) return; // ignore alien addresses
//        char *S;
//        if(PCmd->GetNextString(&S) != retvOk) { PShell->Ack(retvCmdError); return; }
//        strncpy(SelfInfo.Name, S, DEV_NAME_LEN);
//        // Save it
//        if(ee.Write(EE_ADDR_ADDR, &SelfInfo, sizeof(SelfInfo)) == retvOk) PShell->Ack(retvOk);
//        else PShell->Ack(retvFail);
    }

    else if(PCmd->NameIs("GetInfo")) {
        uint8_t FAddr;
        if(PCmd->GetNext<uint8_t>(&FAddr) != retvOk) return;
//        if(FAddr != SelfInfo.Addr) return; // ignore alien addresses
//        PShell->Print("Info %u, %u, \"%S\"\r\n", SelfInfo.Addr, SelfInfo.Type, SelfInfo.Name);
    }
#endif

#if 1 // ==== Master commands ====
    else if(PCmd->NameIs("Scan")) {
        // Todo
        PShell->Print("NoDevices\r\n");
    }

    else if(PCmd->NameIs("GetDeviceList")) {
        if(DevList.Cnt() == 0) PShell->Print("NoDevices\r\n");
        else for(int32_t i=0; i<DevList.Cnt(); i++) {
            uint8_t CheckRslt = DevList[i].Check();
            if(CheckRslt == retvOk) DevList[i].Print(PShell, ", OK\n");
            else if(CheckRslt == retvCollision) DevList[i].Print(PShell, " DifferentParams\n");
            else DevList[i].Print(PShell, " NoAnswer\n");
        }
    }

    else if(PCmd->NameIs("AddDevice")) {
        if(DevList.Cnt() >= DEV_CNT_MAX) { PShell->Print("TableFull\r\n"); return; }
        uint8_t Addr = 0, Type = 0;
        if(PCmd->GetNext<uint8_t>(&Addr) != retvOk or (Addr < ADDR_MIN and Addr > ADDR_MAX)) { PShell->Print("BadParam\r\n"); return; }
        if(PCmd->GetNext<uint8_t>(&Type) != retvOk or (Type < TYPE_MIN and Type > TYPE_MAX)) { PShell->Print("BadParam\r\n"); return; }
        if(DevList.ContainsAddr(Addr)) { PShell->Print("DeviceExists\r\n"); return; }
        char *PName;
        if(PCmd->GetNextString(&PName) != retvOk) { PShell->Print("BadParam\r\n"); return; }
        int Len = strlen(PName);
        if(Len > DEV_NAME_LEN) { PShell->Print("BadParam\r\n"); return; }
        // All is finally ok
        DevList.Add(Addr, Type, PName);
        PShell->Ack(retvOk);
        DevList.Save();
    }

    else if(PCmd->NameIs("PutDevice")) {
        uint8_t Addr = 0, Type = 0;
        if(PCmd->GetNext<uint8_t>(&Addr) != retvOk or (Addr < ADDR_MIN and Addr > ADDR_MAX)) { PShell->Print("BadParam\r\n"); return; }
        if(PCmd->GetNext<uint8_t>(&Type) != retvOk or (Type < TYPE_MIN and Type > TYPE_MAX)) { PShell->Print("BadParam\r\n"); return; }
        char *PName;
        if(PCmd->GetNextString(&PName) != retvOk) { PShell->Print("BadParam\r\n"); return; }
        int Len = strlen(PName);
        if(Len > DEV_NAME_LEN) { PShell->Print("BadParam\r\n"); return; }
        // Params are ok
        Device_t* PDev = DevList.GetByAddr(Addr);
        if(PDev) { // Exists
            PDev->Type = Type;
            strcpy(PDev->Name, PName);
        }
        else { // Not exists, try to add
            if(DevList.Cnt() >= DEV_CNT_MAX) { PShell->Print("TableFull\r\n"); return; }
             DevList.Add(Addr, Type, PName);
        }
        PShell->Ack(retvOk);
        DevList.Save();
    }

    else if(PCmd->NameIs("DelDevice")) {
        uint8_t Addr = 0;
        if(PCmd->GetNext<uint8_t>(&Addr) != retvOk) { PShell->Print("BadParam\r\n"); return; }
        if(DevList.Delete(Addr) == retvOk) {
            PShell->Ack(retvOk);
            DevList.Save();
        }
        else PShell->Print("NoDevices\r\n");
    }


#endif

    else PShell->Ack(retvCmdUnknown);
}
#endif
