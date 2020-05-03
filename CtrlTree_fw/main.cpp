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

void OnMasterCmd(Shell_t *PShell, Cmd_t *PCmd);
void OnSlaveCmd(Shell_t *PShell, Cmd_t *PCmd);
void OnCommonCmd(Shell_t *PShell, Cmd_t *PCmd);

static TmrKL_t TmrOneSecond {TIME_MS2I(999), evtIdEverySecond, tktPeriodic}; // Measure battery periodically
#endif

class GpioReg_t {
private:
    uint32_t IReg = 0;
public:
    void Set(uint32_t AReg) {
        IReg = AReg;
        uint32_t PortE = IReg & 0xFFFF; // Lower bits
        uint32_t PortD = ((IReg >> 16) & 0x7FF) << 5; // higher bits
        chSysLock();
        PortD |= GPIOD->ODR & 0x1FUL; // Get values of lower bits of ODR
        GPIOE->ODR = PortE;
        GPIOD->ODR = PortD;
        chSysUnlock();
    }
    uint32_t Get() { return IReg; }
    void Init() {
        // GPIOE
        PinClockEnable(GPIOE);
        GPIOE->OTYPER = 0;              // Push-Pull
        GPIOE->PUPDR = 0;               // no PullUp/PullDown
        GPIOE->OSPEEDR = 0x55555555;    // Medium speed
        GPIOE->MODER = 0x55555555;      // Mode = output
        // GPIOD
        PinClockEnable(GPIOD);
        GPIOD->OTYPER  = (GPIOD->OTYPER   & 0x001F); // Clear bits [5;15], do not touch [0;4]
        GPIOD->PUPDR   = (GPIOD->PUPDR    & 0x000003FF); // Clear bits [5;15], do not touch [0;4]. No PullUp/PullDown
        GPIOD->OSPEEDR = (GPIOD->OSPEEDR  & 0x000003FF) | 0x55555400; // Medium speed
        GPIOD->MODER   = (GPIOD->MODER    & 0x000003FF) | 0x55555400; // Mode = output
    }
} GpioReg;

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

    SelfInfo.Load(EE_SELF_ADDR);
    DevList.Load();
    Settings.Load();

    GpioReg.Init();
    GpioReg.Set(Settings.PowerOnGPIO);

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
#if 1 // ==== Direct commands ====
    if(PCmd->NameIs("Version")) PShell->Print("%S %S\r\n", APP_NAME, XSTRINGIFY(BUILD_TIME));
    else if(PCmd->NameIs("mem")) PrintMemoryInfo();

    else if(PCmd->NameIs("SetAddr")) {
        uint8_t Addr;
        if(PCmd->GetNext<uint8_t>(&Addr) != retvOk or !AddrIsOk(Addr)) { PShell->Print("BadParam\r\n"); return; }
        SelfInfo.Addr = Addr;
        PShell->Ack(SelfInfo.Save(EE_SELF_ADDR));
    }
    else if(PCmd->NameIs("SetType")) {
        uint8_t Type;
        if(PCmd->GetNext<uint8_t>(&Type) != retvOk or !TypeIsOk(Type)) { PShell->Print("BadParam\r\n"); return; }
        SelfInfo.Type = (DevType_t)Type;
        PShell->Ack(SelfInfo.Save(EE_SELF_ADDR));
    }
    else if(PCmd->NameIs("SetName")) {
        char *S;
        if(PCmd->GetNextString(&S) != retvOk) { PShell->Ack(retvCmdError); return; }
        strncpy(SelfInfo.Name, S, DEV_NAME_LEN);
        PShell->Ack(SelfInfo.Save(EE_SELF_ADDR));
    }
    else if(PCmd->NameIs("GetInfo")) { SelfInfo.Print(PShell, "\r\n"); }
#endif
    // ==== Master commands ====
    else if(SelfInfo.Type == devtHFBlock) OnMasterCmd(PShell, PCmd);
    // ==== Slave commands ====
    else {
        // Check if address matches
        uint8_t FAddr;
        if(PCmd->GetNext<uint8_t>(&FAddr) == retvOk and FAddr == SelfInfo.Addr) OnSlaveCmd(PShell, PCmd);
    }
}

void OnMasterCmd(Shell_t *PShell, Cmd_t *PCmd) {
    if(PCmd->NameIs("Ping")) PShell->Ack(retvOk);
    else if(PCmd->NameIs("Scan")) {
        // Todo
        PShell->Print("NoDevices\r\n");
    }

    else if(PCmd->NameIs("GetDeviceList")) {
        if(DevList.Cnt() == 0) PShell->Print("NoDevices\r\n");
        else {
            for(int32_t i=0; i<DevList.Cnt(); i++) {
                uint8_t CheckRslt = DevList[i].Check();
                if(CheckRslt == retvOk) DevList[i].Print(PShell, ", OK\n");
                else if(CheckRslt == retvCollision) DevList[i].Print(PShell, " DifferentParams\n");
                else DevList[i].Print(PShell, " NoAnswer\n");
            }
            PShell->Print("\r");
        }
    }

    else if(PCmd->NameIs("AddDevice")) {
        if(DevList.Cnt() >= DEV_CNT_MAX) { PShell->Print("TableFull\r\n"); return; }
        uint8_t Addr = 0, Type = 0;
        if(PCmd->GetNext<uint8_t>(&Addr) != retvOk or !AddrIsOk(Addr)) { PShell->Print("BadParam\r\n"); return; }
        if(PCmd->GetNext<uint8_t>(&Type) != retvOk or !TypeIsOk(Type)) { PShell->Print("BadParam\r\n"); return; }
        if(DevList.ContainsAddr(Addr)) { PShell->Print("DeviceExists\r\n"); return; }
        char *PName;
        if(PCmd->GetNextString(&PName) != retvOk) { PShell->Print("BadParam\r\n"); return; }
        int Len = strlen(PName);
        if(Len > DEV_NAME_LEN) { PShell->Print("BadParam\r\n"); return; }
        // All is finally ok
        DevList.Add(Addr, (DevType_t)Type, PName);
        // Try to change real device parameters
        // Todo
        PShell->Ack(retvOk);
        DevList.Save();
    }

    else if(PCmd->NameIs("PutDevice")) {
        uint8_t Addr = 0, Type = 0;
        if(PCmd->GetNext<uint8_t>(&Addr) != retvOk or !AddrIsOk(Addr)) { PShell->Print("BadParam\r\n"); return; }
        if(PCmd->GetNext<uint8_t>(&Type) != retvOk or !TypeIsOk(Type)) { PShell->Print("BadParam\r\n"); return; }
        char *PName;
        if(PCmd->GetNextString(&PName) != retvOk) { PShell->Print("BadParam\r\n"); return; }
        int Len = strlen(PName);
        if(Len > DEV_NAME_LEN) { PShell->Print("BadParam\r\n"); return; }
        // Params are ok
        Device_t* PDev = DevList.GetByAddr(Addr);
        if(PDev) { // Exists
            PDev->Type = (DevType_t)Type;
            strcpy(PDev->Name, PName);
        }
        else { // Not exists, try to add
            if(DevList.Cnt() >= DEV_CNT_MAX) { PShell->Print("TableFull\r\n"); return; }
             DevList.Add(Addr, (DevType_t)Type, PName);
        }
        // Try to change real device parameters
        // Todo
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

    else if(PCmd->NameIs("GetAllStates")) {
        PShell->Print("0, %u, %S, GPIO: %X\n", SelfInfo.Type, SelfInfo.Name, GpioReg.Get());
        for(int32_t i=0; i<DevList.Cnt(); i++) {
            uint8_t CheckRslt = DevList[i].Check();
            if(CheckRslt == retvOk) {
                DevList[i].Print(PShell, ", ");
                //  todo: getstate
                PShell->Print("\n");
            }
            else if(CheckRslt == retvCollision) DevList[i].Print(PShell, " DifferentParams\n");
            else DevList[i].Print(PShell, " NoAnswer\n");
        }
        PShell->Print("\r");
    }
    else OnCommonCmd(PShell, PCmd);
}

void OnSlaveCmd(Shell_t *PShell, Cmd_t *PCmd) {
#if 1 // ==== Addr, type, name ====
    if(PCmd->NameIs("Ping")) PShell->Ack(retvOk);
    else if(PCmd->NameIs("ChangeAddr")) {
        uint8_t Addr;
        if(PCmd->GetNext<uint8_t>(&Addr) != retvOk or !AddrIsOk(Addr)) { PShell->Print("BadParam\r\n"); return; }
        SelfInfo.Addr = Addr;
        PShell->Ack(SelfInfo.Save(EE_SELF_ADDR));
    }
    else if(PCmd->NameIs("ChangeType")) {
        uint8_t Type;
        if(PCmd->GetNext<uint8_t>(&Type) != retvOk or !TypeIsOk(Type)) { PShell->Print("BadParam\r\n"); return; }
        SelfInfo.Type = (DevType_t)Type;
        PShell->Ack(SelfInfo.Save(EE_SELF_ADDR));
    }
    else if(PCmd->NameIs("ChangeName")) {
        char *S;
        if(PCmd->GetNextString(&S) != retvOk) { PShell->Ack(retvCmdError); return; }
        strncpy(SelfInfo.Name, S, DEV_NAME_LEN);
        PShell->Ack(SelfInfo.Save(EE_SELF_ADDR));
    }
    else if(PCmd->NameIs("GetTypeName")) { PShell->Print("%u, %S\r\n", SelfInfo.Type, SelfInfo.Name); }
#endif
#if 1 // ==== Thermostating ====

#endif
    else OnCommonCmd(PShell, PCmd);
}

void OnCommonCmd(Shell_t *PShell, Cmd_t *PCmd) {
    // ==== GPIO Reg ====
    if(PCmd->NameIs("SetGPIO")) {
         uint32_t Reg;
         if(PCmd->GetNext<uint32_t>(&Reg) != retvOk) { PShell->Print("BadParam\r\n"); return; }
         GpioReg.Set(Reg);
         PShell->Ack(retvOk);
    }
    else if(PCmd->NameIs("GetGPIO")) { PShell->Print("GetGPIO 0x%X\r\n", GpioReg.Get()); }

    else if(PCmd->NameIs("SetPowerOnGPIO")) {
        uint32_t Reg;
        if(PCmd->GetNext<uint32_t>(&Reg) != retvOk) { PShell->Print("BadParam\r\n"); return; }
        Settings.PowerOnGPIO = Reg;
        PShell->Ack(Settings.Save());
   }
   else if(PCmd->NameIs("GetPowerOnGPIO")) { PShell->Print("GetPowerOnGPIO 0x%X\r\n", Settings.PowerOnGPIO); }
}

#endif
