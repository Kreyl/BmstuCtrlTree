#include "kl_crc.h"
#include "TControl.h"
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
#include <string>
#include "UpdateFw.h"
#include "Adf5356.h"

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
static const UartParams_t RS485IntParams(115200, RS485INT_PARAMS);

// Control from outside
CmdUart_t Uart{CmdUartParams};
CmdUart485_t RS485Ext{RS485ExtParams, RS485_EXT_TXEN};
CmdUart_t RS232{RS232Params};
// Internal host
HostUart485_t RS485Int{RS485IntParams, RS485_INT_TXEN};
#define TIMEOUT_SHORT_ms  54UL
#define TIMEOUT_MID_ms    360UL
#define TIMEOUT_LONG_ms   9999UL

bool UsbIsConnected = false;
LedBlinker_t Led{LED_PIN};
DeviceList_t DevList;

uint8_t ProcessMasterCmd(Shell_t *PShell, Cmd_t *PCmd);
void OnSlaveCmd(Shell_t *PShell, Cmd_t *PCmd);
void ProcessCmdForSlave(Shell_t *PShell, Cmd_t *PCmd, uint32_t Addr);

static TmrKL_t TmrOneSecond {TIME_MS2I(999), evtIdEverySecond, tktPeriodic}; // Measure battery periodically
uint8_t FileBuf[FILEBUF_SZ];
#endif

#if 1 // ============================= Classes =================================
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

#define SPI_BUF_SZ  32
class DevSpi_t {
private:
    SPI_TypeDef *PSpi;
    uint8_t TxBuf[SPI_BUF_SZ];
    uint8_t RxBuf[SPI_BUF_SZ];
    GPIO_TypeDef *PGpio;
    uint32_t Sck, Miso, Mosi, Cs;
    AlterFunc_t AF;
public:
    DevSpi_t(SPI_TypeDef *ASpi,
            GPIO_TypeDef *AGpio, uint32_t ASck, uint32_t AMiso, uint32_t AMosi, uint32_t ACs, AlterFunc_t AAF) :
                PSpi(ASpi), PGpio(AGpio), Sck(ASck), Miso(AMiso), Mosi(AMosi), Cs(ACs), AF(AAF) {}
    void Init(bool CsActiveLowIdleHi) {
        if      (PSpi == SPI1) { rccEnableSPI1(FALSE); }
        else if (PSpi == SPI2) { rccEnableSPI2(FALSE); }
        PinSetupOut(PGpio, Cs, omPushPull);
        // Deactivate CS
        if(CsActiveLowIdleHi) PinSetHi(PGpio, Cs);
        else PinSetLo(PGpio, Cs);
        PinSetupAlterFunc(PGpio, Sck,  omPushPull, pudNone, AF, psVeryHigh);
        PinSetupAlterFunc(PGpio, Miso, omPushPull, pudNone, AF, psVeryHigh);
        PinSetupAlterFunc(PGpio, Mosi, omPushPull, pudNone, AF, psVeryHigh);
    }

    void Transmit(uint8_t Params, uint8_t *ptr, uint32_t Len) {
        bool CsActiveLowIdleHi = (Params & 0x08) == 0;
        // Activate CS
        if(CsActiveLowIdleHi) PinSetLo(PGpio, Cs);
        else PinSetHi(PGpio, Cs);
        // Setup SPI
        PSpi->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR;
        if(Params & 0x80) PSpi->CR1 |= SPI_CR1_LSBFIRST; // 0 = MSB, 1 = LSB
        if(Params & 0x40) PSpi->CR1 |= SPI_CR1_CPOL;     // 0 = IdleLow, 1 = IdleHigh
        if(Params & 0x20) PSpi->CR1 |= SPI_CR1_CPHA;     // 0 = FirstEdge, 1 = SecondEdge
        PSpi->CR1 |= (Params & 0x07) << 3; // Setup divider
        PSpi->CR2 = ((uint16_t)0b0111 << 8) | SPI_CR2_FRXTH;   // 8 bit, RXNE generated when 8 bit is received
        (void)PSpi->SR; // Read Status reg to clear some flags
        // Do it
        PSpi->CR1 |=  SPI_CR1_SPE; // Enable SPI
        while(Len) {
            *((volatile uint8_t*)&PSpi->DR) = *ptr;
            while(!(PSpi->SR & SPI_SR_RXNE));  // Wait for SPI transmission to complete
            *ptr = *((volatile uint8_t*)&PSpi->DR);
            ptr++;
            Len--;
        }
        // Deactivate CS
        if(CsActiveLowIdleHi) PinSetHi(PGpio, Cs);
        else PinSetLo(PGpio, Cs);
        PSpi->CR1 &= ~SPI_CR1_SPE; // Disable SPI
    }
};

DevSpi_t Spi1{SPI1, GPIOA, 5,6,7,4, AF5};
DevSpi_t Spi2{SPI2, GPIOB, 13,14,15,12, AF5};

class Power_t {
public:
    uint32_t Current = 0;
} Power;

class Hmc821_t {
private:
    PinOutput_t SEN{GPIOA, 4, omPushPull}; // Serial port enable, Active High (!)
    Spi_t ISpi{SPI1};
public:
    void Init() {
        // Init GPIOs
        SEN.Init();
        PinSetupOut(GPIOA, 5, omPushPull); // SCK must be low first
        chThdSleepMilliseconds(45); // let it wake
        // Rise SEN before SCK: select HMC Serial Mode
        SEN.SetHi();
        chThdSleepMilliseconds(4);
        PinSetHi(GPIOA, 5); // SCK HI
        chThdSleepMilliseconds(4);
        PinSetLo(GPIOA, 5); // SCK Lo
        SEN.SetLo(); // SEN Active High
        chThdSleepMilliseconds(4);
        // Init SPI
        PinSetupAlterFunc(GPIOA, 5, omPushPull, pudNone, AF5, psVeryHigh); // SCK
//        PinSetupAlterFunc(GPIOA, 6, omPushPull, pudNone, AF5, psVeryHigh); // MISO
        PinSetupAlterFunc(GPIOA, 7, omPushPull, pudNone, AF5, psVeryHigh); // MOSI
        rccEnableSPI1(FALSE);
        ISpi.Setup(boMSB, cpolIdleLow, cphaFirstEdge, 1000000, bitn16);
        ISpi.Enable();
    }
    void WriteReg(uint32_t Addr, uint32_t Value) {
        uint32_t Word = ((Addr & 0x3FUL) << 25) | ((Value & 0xFFFFFFUL) << 1); // 0=wr, 6xAddr, 24xValue, 1xDontCare
        SEN.SetHi(); // SEN is active hi
        ISpi.ReadWriteWord((Word >> 16) & 0xFFFF);
        ISpi.ReadWriteWord(Word & 0xFFFF);
        SEN.SetLo();
//        Printf("HMC: %X = %X; %X\r", Addr, Value, Word);
    }
} Hmc821;

Adf5356_t Adf5356;
#endif

int main(void) {
#if 1 // ==== Setup clock frequency ====
    Clk.EnablePrefetch();
    Clk.SwitchToMSI();
    Clk.SetVoltageRange(mvrHiPerf);
    Clk.SetupFlashLatency(48, mvrHiPerf);
    // Try quartz
    if(Clk.EnableHSE() == retvOk) {
        Clk.SetupPllSrc(pllsrcHse);
        Clk.SetupPll(8, 2, 2); // 12MHz / 1 = 12; 12 * 8 / 2 => 48
    }
    else { // Quartz failed
        Clk.SetupPllSrc(pllsrcMsi);
        Clk.SetupPll(24, 2, 2); // 4MHz / 1 = 4; 4 * 24 / 2 => 48
    }
    // Try start PLL
    if(Clk.EnablePLL() == retvOk) {
        Clk.EnablePllROut();
        Clk.EnablePllQOut();
        Clk.SwitchToPLL();
        Clk.SetupPllQas48MhzSrc();
    }
    Clk.UpdateFreqValues();
#endif

    // Init OS
    halInit();
    chSysInit();
    OsIsInitialized = true;

    // ==== Init hardware ====
    EvtQMain.Init();
    Uart.Init();
    Printf("\r%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));
    if(Clk.GetPllSrc() == pllsrcHse) Printf("Quartz ok\r\n");
    else Printf("No Quartz\r\n");
    Printf("%S\r\n", (FLASH->OPTR & FLASH_OPTR_BFB2)? "BankB" : "BankA");
    Clk.PrintFreqs();

    if(sizeof(Settings_t) > SETTINGS_MAX_SZ) Printf("*** Too large Settings ***\r\n");

//    Printf("sz of Settings_t: %u; max: %u\r", sizeof(Settings_t), SETTINGS_MAX_SZ);

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
//    Printf("SavedRegsCnt: %u, %u\r", Settings.SavedRegsCnt, Settings.RegsAreSaved());

    // Uarts
    RS485Ext.Init();
    RS485Int.Init();
    RS232.Init();
    UsbCDC.Init();

    Crc::InitHWDMA();
    SimpleSensors::Init();

    // T control
    tControl.Init();
    if(SelfInfo.Type == devtLNA or SelfInfo.Type == devtTriplexer) {
        if(Settings.TControlEnabled) tControl.StartControl();
        else tControl.StartMeasure();
    }

    // Always
    GpioReg.Init();
    GpioReg.Set(Settings.PowerOnGPIO);
    Spi2.Init(Settings.SpiSetup.CS2ActiveLow);

    // Spi1 utilized by MRL and KuKonv, init it if type id another
    if(SelfInfo.Type == devtMRL) {
        Hmc821.Init();
        // Load regs if they are saved
        if(Settings.WhatSaved == SAVED_REGS_FLAG and Settings.SavedRegsCnt <= REG_CNT) {
            for(uint32_t i=0; i<Settings.SavedRegsCnt; i++) {
                Hmc821.WriteReg(Settings.Hmc821.Regs[i].Addr, Settings.Hmc821.Regs[i].Value);
            }
        }
    }
    else if(SelfInfo.Type == devtKUKonv or SelfInfo.Type == devtIKS) {
        Adf5356.Init();
        // Load regs if they are saved
        if(Settings.WhatSaved == SAVED_REGS_FLAG and Settings.SavedRegsCnt <= REG_CNT) {
            for(uint32_t i=0; i<Settings.SavedRegsCnt; i++) {
                Adf5356.WriteReg(Settings.Adf5356.Regs[i]);
//                Printf("%X\r", Settings.Adf5356.Regs[i]);
            }
        }
        // Or calc and set regs if params were saved
        else if(Settings.WhatSaved == SAVED_PARAMS_FLAG) {
            Adf5356.fref = Settings.Adf5356.fref;
            Adf5356.step = Settings.Adf5356.step;
            Adf5356.fd   = Settings.Adf5356.fd;
            Adf5356.fvco = Settings.Adf5356.fvco;
            Adf5356.CalcRegs();
            Adf5356.SetRegs();
//            Adf5356.PrintRegs();
        }
    }
    else Spi1.Init(Settings.SpiSetup.CS1ActiveLow);

    // Main cycle
    ITask();
}

__noreturn
void ITask() {
    while(true) {
        EvtMsg_t Msg = EvtQMain.Fetch(TIME_INFINITE);
        switch(Msg.ID) {
            case evtIdUartCmdRcvd:
                if(((CmdUart_t*)Msg.Ptr)->TryParseRxBuff() == retvOk) OnCmd((Shell_t*)((CmdUart_t*)Msg.Ptr));
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
void TryToSaveSelfInfo(Shell_t *PShell) {
    if(SelfInfo.Save(EE_SELF_ADDR) == retvOk) PShell->Ok();
    else PShell->Failure();
}

void SetType(DevType_t NewType, Shell_t *PShell) {
    SelfInfo.Type = NewType;
    // Reset everything
    Settings.Reset();
    Spi1.Init(true);
    tControl.Stop(); // Stop TStating
    switch(NewType) {
        case devtHFBlock:                            break;
        case devtLNA:       tControl.StartMeasure(); break;
        case devtKUKonv:    Adf5356.Init();          break;
        case devtMRL:       Hmc821.Init();           break;
        case devtTriplexer: tControl.StartMeasure(); break;
        case devtIKS:       Adf5356.Init();          break;
        default: break;
    } // switch
    TryToSaveSelfInfo(PShell);
}

void OnCmd(Shell_t *PShell) {
    Led.StartOrRestart(lsqCmd);
	Cmd_t *PCmd = &PShell->Cmd;
//	Printf("Cmd: %S\r", PCmd->Name);
#if 1 // ==== Direct commands ====
    if(PCmd->NameIs("Version")) PShell->Print("%S %S\r\n", APP_NAME, XSTRINGIFY(BUILD_TIME));
    else if(PCmd->NameIs("mem")) PrintMemoryInfo();

    else if(PCmd->NameIs("bfbsw")) {
        Printf("%X\r", FLASH->OPTR);
        chThdSleepMilliseconds(99);
        Flash::ToggleBootBankAndReset();
    }

    else if(PCmd->NameIs("SetTemp")) {
//        float t;
//        if(PCmd->Get("%f") == 1)
    }

    else if(PCmd->NameIs("htr"))  {
        uint32_t v;
        if(PCmd->GetNext<uint32_t>(&v) == retvOk) {
            tControl.SetHeater(v);
            PShell->Ok();
        }
        else PShell->BadParam();
    }

    else if(PCmd->NameIs("SetAddr")) {
        uint8_t Addr;
        if(PCmd->GetNext<uint8_t>(&Addr) != retvOk or !AddrIsOk(Addr)) { PShell->Print("BadParam\r\n"); return; }
        SelfInfo.Addr = Addr;
        TryToSaveSelfInfo(PShell);
    }
    else if(PCmd->NameIs("SetType")) {
        uint8_t Type;
        if(PCmd->GetNext<uint8_t>(&Type) != retvOk or !TypeIsOk(Type)) { PShell->Print("BadParam\r\n"); return; }
        SetType((DevType_t)Type, PShell);
    }
    else if(PCmd->NameIs("SetName")) {
        char *S =PCmd->GetNextString();
        if(!S or strlen(S) > DEV_NAME_LEN) { PShell->BadParam(); return; }
        strcpy(SelfInfo.Name, S);
        TryToSaveSelfInfo(PShell);
    }
    else if(PCmd->NameIs("GetInfo")) { SelfInfo.Print(PShell, "\r\n"); }
#endif
    else { // Command is not direct
        uint8_t FAddr;
        if(SelfInfo.Type == devtHFBlock) { // if we are master
            if(ProcessMasterCmd(PShell, PCmd) != retvOk) { // if cmd was not found in master cmds
                if(PCmd->GetNext<uint8_t>(&FAddr) == retvOk) { // Got address
                    if(FAddr == ADDR_MASTER) OnSlaveCmd(PShell, PCmd); // Universal or slave cmd
                    else ProcessCmdForSlave(PShell, PCmd, FAddr);
                }
                else PShell->BadParam();
            }
        }
        else { // We are slave
            if(PCmd->GetNext<uint8_t>(&FAddr) == retvOk) {
//                Printf("Addr: %u\r", FAddr);
                if(FAddr == SelfInfo.Addr) OnSlaveCmd(PShell, PCmd);
            }
        }
    } // not direct
}

void TryToChangeRealDeviceParams(Cmd_t *PCmd, uint8_t Addr, uint8_t Type, char *PName) {
    if(RS485Int.SendCmd(TIMEOUT_SHORT_ms, "GetTypeName", Addr) == retvOk) {
        uint8_t CurType;
        if(RS485Int.Reply.GetNext<uint8_t>(&CurType) == retvOk) {
            char *CurName = RS485Int.Reply.GetNextString();
            if(strcmp(CurName, PName) != 0) RS485Int.SendCmd(TIMEOUT_SHORT_ms, "ChangeName", Addr, "%S", PName);
            if(CurType != Type) RS485Int.SendCmd(TIMEOUT_SHORT_ms, "ChangeType", Addr, "%u", Type);
        }
    } // If device replied. Do nothing if not.
}

uint8_t ProcessMasterCmd(Shell_t *PShell, Cmd_t *PCmd) {
    if(PCmd->NameIs("Scan")) {
        PShell->Print("Addr Type Name\n");
        bool SomeoneFound = false;
        for(uint32_t i=ADDR_MIN; i<=ADDR_MAX; i++) {
            if(i == ADDR_MASTER) continue;
            if(RS485Int.SendCmd(TIMEOUT_SHORT_ms, "GetTypeName", i) == retvOk) {
                PShell->Print("%4u    %S %S\n", i, RS485Int.Reply.GetNextString(), RS485Int.Reply.GetNextString());
               SomeoneFound = true;
            }
        }
        if(SomeoneFound) PShell->EOL();
        else PShell->Print("NoDevices\r\n");
    }

    else if(PCmd->NameIs("GetDeviceList")) {
        if(DevList.Cnt() == 0) PShell->Print("NoDevices\r\n");
        else {
            uint32_t LongestNameLen = DevList.GetLongestNameLen();
            PShell->Print("Addr Type %*S State\n", LongestNameLen, "Name");
            for(int32_t i=0; i<DevList.Cnt(); i++) {
                PShell->Print("%4u %4u %*S ", DevList[i].Addr, DevList[i].Type, LongestNameLen, DevList[i].Name);
                if(RS485Int.SendCmd(TIMEOUT_SHORT_ms, "GetTypeName", DevList[i].Addr) == retvOk) {
                    // Check if type and name are same
                    uint8_t Type = 0;
                    if(RS485Int.Reply.GetNext<uint8_t>(&Type) == retvOk and Type == DevList[i].Type) {
                        if(strcmp(RS485Int.Reply.GetNextString(), DevList[i].Name) == 0) {
                            PShell->Print("OK\n");
                            continue;
                        }
                    }
                    PShell->Print("DifferentParams\n");
                }
                else PShell->Print("NoAnswer\n"); // Notice \n here
            } // for
            PShell->EOL();
        }
    }

    else if(PCmd->NameIs("AddDevice")) {
        if(DevList.Cnt() >= DEV_CNT_MAX) { PShell->Print("TableFull\r\n"); return retvOk; }
        uint8_t Addr = 0, Type = 0;
        if(PCmd->GetNext<uint8_t>(&Addr) != retvOk or !AddrIsOk(Addr)) { PShell->BadParam(); return retvOk; }
        if(PCmd->GetNext<uint8_t>(&Type) != retvOk or !TypeIsOk(Type)) { PShell->BadParam(); return retvOk; }
        if(DevList.ContainsAddr(Addr)) { PShell->Print("DeviceExists\r\n"); return retvOk; }
        char *PName = PCmd->GetNextString();
        if(!PName) { PShell->BadParam(); return retvOk; }
        int Len = strlen(PName);
        if(Len > DEV_NAME_LEN) { PShell->BadParam(); return retvOk; }
        // All is finally ok
        DevList.Add(Addr, (DevType_t)Type, PName);
        TryToChangeRealDeviceParams(PCmd, Addr, Type, PName);
        PShell->Ok();
        DevList.Save();
    }

    else if(PCmd->NameIs("PutDevice")) {
        uint8_t Addr = 0, Type = 0;
        if(PCmd->GetNext<uint8_t>(&Addr) != retvOk or !AddrIsOk(Addr)) { PShell->BadParam(); return retvOk; }
        if(PCmd->GetNext<uint8_t>(&Type) != retvOk or !TypeIsOk(Type)) { PShell->BadParam(); return retvOk; }
        char *PName = PCmd->GetNextString();
        if(!PName) { PShell->BadParam(); return retvOk; }
        int Len = strlen(PName);
        if(Len > DEV_NAME_LEN) { PShell->BadParam(); return retvOk; }
        // Params are ok
        Device_t* PDev = DevList.GetByAddr(Addr);
        if(PDev) { // Exists
            PDev->Type = (DevType_t)Type;
            strcpy(PDev->Name, PName);
        }
        else { // Not exists, try to add
            if(DevList.Cnt() >= DEV_CNT_MAX) { PShell->Print("TableFull\r\n"); return retvOk; }
             DevList.Add(Addr, (DevType_t)Type, PName);
        }
        TryToChangeRealDeviceParams(PCmd, Addr, Type, PName);
        PShell->Ok();
        DevList.Save();
    }

    else if(PCmd->NameIs("DelDevice")) {
        uint8_t Addr = 0;
        if(PCmd->GetNext<uint8_t>(&Addr) != retvOk) { PShell->BadParam(); return retvOk; }
        if(DevList.Delete(Addr) == retvOk) {
            PShell->Ok();
            DevList.Save();
        }
        else PShell->Print("NoDevices\r\n");
    }

    else if(PCmd->NameIs("GetAllStates")) {
        uint32_t LongestNameLen = DevList.GetLongestNameLen();
        PShell->Print("Addr Type %*S State\n", LongestNameLen, "Name");
        PShell->Print("%4u %4u %*S %S GPIO=0x%X\n", ADDR_MASTER, SelfInfo.Type,
                LongestNameLen, SelfInfo.Name, XSTRINGIFY(BUILD_TIME), GpioReg.Get());
        for(int32_t i=0; i<DevList.Cnt(); i++) {
            PShell->Print("%4u %4u %*S ", DevList[i].Addr, DevList[i].Type, LongestNameLen, DevList[i].Name);
            if(RS485Int.SendCmd(TIMEOUT_SHORT_ms, "GetTypeName", DevList[i].Addr) == retvOk) {
                // Check if type and name are same
                uint8_t Type = 0;
                if(RS485Int.Reply.GetNext<uint8_t>(&Type) == retvOk and Type == DevList[i].Type) {
                    if(RS485Int.SendCmd(TIMEOUT_SHORT_ms, "GetState", DevList[i].Addr) == retvOk) {
                        PShell->Print("%S\n", RS485Int.Reply.GetRemainder());
                        continue;
                    }
                    else { PShell->Print("NoAnswer\n"); continue; } // Notice \n here
                }
                PShell->Print("DifferentParams\n");
            }
            else PShell->Print("NoAnswer\n"); // Notice \n here
        } // for
        PShell->EOL();
    }
    else return retvNotFound;
    return retvOk;
}

void OnSlaveCmd(Shell_t *PShell, Cmd_t *PCmd) {
#if 1 // ==== Addr, type, name ====
    if(PCmd->NameIs("Ping")) PShell->Ok();
    else if(PCmd->NameIs("ChangeAddr")) {
        uint8_t Addr;
        if(PCmd->GetNext<uint8_t>(&Addr) != retvOk or !AddrIsOk(Addr)) { PShell->Print("BadParam\r\n"); return; }
        SelfInfo.Addr = Addr;
        TryToSaveSelfInfo(PShell);
    }
    else if(PCmd->NameIs("ChangeType")) {
        uint8_t Type;
        if(PCmd->GetNext<uint8_t>(&Type) != retvOk or !TypeIsOk(Type)) { PShell->Print("BadParam\r\n"); return; }
        SetType((DevType_t)Type, PShell);
    }
    else if(PCmd->NameIs("ChangeName")) {
        char *S = PCmd->GetNextString();
        if(!S or strlen(S) > DEV_NAME_LEN) { PShell->BadParam(); return; }
        strcpy(SelfInfo.Name, S);
        TryToSaveSelfInfo(PShell);
    }
    else if(PCmd->NameIs("GetTypeName")) { PShell->Print("RGetTypeName %u %S\r\n", SelfInfo.Type, SelfInfo.Name); }
#endif
#if 1 // ==== GPIO Reg ====
    else if(PCmd->NameIs("SetGPIO")) {
         uint32_t Reg;
         if(PCmd->GetNext<uint32_t>(&Reg) != retvOk) { PShell->BadParam(); return; }
         GpioReg.Set(Reg);
         PShell->Ok();
    }
    else if(PCmd->NameIs("GetGPIO")) { PShell->Print("RGetGPIO 0x%X\r\n", GpioReg.Get()); }

    else if(PCmd->NameIs("SetPowerOnGPIO")) {
        uint32_t Reg;
        if(PCmd->GetNext<uint32_t>(&Reg) != retvOk) { PShell->BadParam(); return; }
        Settings.PowerOnGPIO = Reg;
        if(Settings.Save() == retvOk) PShell->Ok();
        else PShell->Failure();
    }
    else if(PCmd->NameIs("GetPowerOnGPIO")) { PShell->Print("RGetPowerOnGPIO 0x%X\r\n", Settings.PowerOnGPIO); }
#endif
#if 1 // ==== SPI ====
    else if(PCmd->NameIs("WRSPI")) {
        uint8_t Params;
        if(PCmd->GetNext<uint8_t>(&Params) != retvOk) { PShell->BadParam(); return; }
        // SPI1 is not allowed for direct writing
        if((SelfInfo.Type == devtMRL or SelfInfo.Type == devtKUKonv) and ((Params & 0x10) == 0)) {
            PShell->BadParam();
            return;
        }
        uint8_t *p = FileBuf;
        while(PCmd->GetNext<uint8_t>(p) == retvOk) p++; // Get what to send
        uint32_t Len = p - FileBuf;
        if(Params & 0x10) Spi2.Transmit(Params, FileBuf, Len);
        else Spi1.Transmit(Params, FileBuf, Len);
        // Reply
        p = FileBuf;
        PShell->Print("RWRSPI");
        while(Len--) PShell->Print(" 0x%02X", *p++);
        PShell->EOL();
    }

    else if(PCmd->NameIs("wSPIFile")) {
        uint8_t Params;
        if(PCmd->GetNext<uint8_t>(&Params) != retvOk) { PShell->BadParam(); return; }
        // SPI1 is not allowed for direct writing
        if((SelfInfo.Type == devtMRL or SelfInfo.Type == devtKUKonv) and ((Params & 0x10) == 0)) {
            PShell->BadParam();
            return;
        }
        uint32_t Len;
        if(PCmd->GetNext<uint32_t>(&Len) != retvOk or Len > FILEBUF_SZ) { PShell->BadParam(); return; }
        Led.StartOrRestart(lsqWriting);
        // Receive data
        if(PShell->ReceiveBinaryToBuf(FileBuf, Len, TIMEOUT_LONG_ms) == retvOk) {
            if(Params & 0x10) Spi2.Transmit(Params, FileBuf, Len);
            else Spi1.Transmit(Params, FileBuf, Len);
            PShell->Ok();
        }
        else PShell->Timeout();
        Led.StartOrRestart(lsqCmd);
    }

    else if(PCmd->NameIs("rSpiFile")) {
        uint32_t Len;
        if(PCmd->GetNext<uint32_t>(&Len) == retvOk and Len <= FILEBUF_SZ) {
            Led.StartOrRestart(lsqWriting);
            if(PShell->TransmitBinaryFromBuf(FileBuf, Len, TIMEOUT_LONG_ms) != retvOk) PShell->Timeout();
            Led.StartOrRestart(lsqCmd);
        }
        else PShell->BadParam();
    }

    else if(PCmd->NameIs("SetCS1")) {
        uint32_t CsActiveLow;
        if(PCmd->GetNext<uint32_t>(&CsActiveLow) == retvOk) {
            Settings.SpiSetup.CS1ActiveLow = (bool)CsActiveLow;
            Spi1.Init(CsActiveLow);
            if(Settings.Save() == retvOk) PShell->Ok();
            else PShell->Failure();
        }
        else PShell->BadParam();
    }
    else if(PCmd->NameIs("SetCS2")) {
        uint32_t CsActiveLow;
        if(PCmd->GetNext<uint32_t>(&CsActiveLow) == retvOk) {
            Settings.SpiSetup.CS2ActiveLow = (bool)CsActiveLow;
            Spi2.Init(CsActiveLow);
            if(Settings.Save() == retvOk) PShell->Ok();
            else PShell->Failure();
        }
        else PShell->BadParam();
    }

    else if(PCmd->NameIs("GetCS1")) { PShell->Print("RGetCS1 %u\r\n", Settings.SpiSetup.CS1ActiveLow); }
    else if(PCmd->NameIs("GetCS2")) { PShell->Print("RGetCS2 %u\r\n", Settings.SpiSetup.CS2ActiveLow); }
#endif

    else if(PCmd->NameIs("UpdateFW")) {
        uint32_t Len, CrcIn;
        if(PCmd->GetNext<uint32_t>(&Len) != retvOk or Len > FILEBUF_SZ) { PShell->BadParam(); return; }
        if(PCmd->GetNext<uint32_t>(&CrcIn) != retvOk) { PShell->BadParam(); return; }
        // Receive data
        Led.StartOrRestart(lsqWriting);
        if(PShell->ReceiveBinaryToBuf(FileBuf, Len, TIMEOUT_LONG_ms) == retvOk) {
            uint8_t rslt = UpdateFw(FileBuf, Len, CrcIn);
            if(rslt == retvOk) {
                PShell->Ok();
                chThdSleepMilliseconds(72); // Let it end transmisison
                Flash::ToggleBootBankAndReset();
            }
            else if(rslt == retvCRCError) PShell->CRCError();
            else PShell->Failure();
        }
        else PShell->Timeout();
        Led.StartOrRestart(lsqCmd);
    }

    else if(PCmd->NameIs("GetState")) {
        PShell->Print("RGetState %S GPIO=0x%X", XSTRINGIFY(BUILD_TIME), GpioReg.Get());
        switch(SelfInfo.Type) {
            case devtNone:
            case devtHFBlock:
                break;
            case devtLNA:
            case devtTriplexer:
                PShell->Print(" t=%.1f TStating=%d", tControl.tAvg, Settings.TControlEnabled);
                for(auto &Sns : tControl.Sensors) {
                    if(Sns.IsOk) PShell->Print(" %S=%.1f", Sns.Name, Sns.t);
                    else         PShell->Print(" %S=err", Sns.Name);
                }
                break;
            case devtKUKonv:
                break;
            case devtMRL:
                break;
            case devtIKS:
                break;
        }
        PShell->EOL();
    }

#if 1 // ==== Thermostating ====
    else if(PCmd->NameIs("SetTTS")) {
        float t;
        if(PCmd->GetNextFloat(&t) != retvOk) { PShell->BadParam(); return; }
        if(t < 0 or t > 125) { PShell->BadParam(); return; }
        Settings.TargetT = t;
        if(Settings.Save() == retvOk) PShell->Ok();
        else PShell->Failure();
    }
    else if(PCmd->NameIs("GetTTS")) { PShell->Print("RGetTTS %.1f\r\n", Settings.TargetT); }

    else if(PCmd->NameIs("SetTstating")) {
        if(SelfInfo.Type == devtLNA or SelfInfo.Type == devtTriplexer) {
            uint8_t OnOff;
            if(PCmd->Get("%u8", &OnOff) != 1) { PShell->BadParam(); return; }
            if(OnOff > 0) OnOff = 1;
            Settings.TControlEnabled = OnOff;
            if(OnOff) tControl.StartControl();
            else tControl.StartMeasure();
            if(Settings.Save() == retvOk) PShell->Ok();
            else PShell->Failure();
        }
        else PShell->CmdError();
    }
    else if(PCmd->NameIs("GetTstating")) { PShell->Print("RGetTstating %u\r\n", Settings.TControlEnabled); }
#endif

#if 1 // ==== HMC821 ====
    else if(SelfInfo.Type == devtMRL) {
        if(PCmd->NameIs("SetRegs")) {
            uint32_t Cnt = 0, Addr, Value;
            while(true) {
                if(PCmd->GetNext<uint32_t>(&Addr) != retvOk) break;
                if(PCmd->GetNext<uint32_t>(&Value) != retvOk) { PShell->BadParam(); return; } // Addr exsits, value does not
                Hmc821.WriteReg(Addr, Value);
                Cnt++;
            }
            if(Cnt) PShell->Ok();
            else PShell->BadParam();
        }
        else if(PCmd->NameIs("SaveRegs")) {
            uint32_t Cnt = 0, Addr[REG_CNT], Value[REG_CNT];
            // Get regs
            while(true) {
                if(Cnt >= REG_CNT) { PShell->BadParam(); return; }
                if(PCmd->GetNext<uint32_t>(&Addr[Cnt]) != retvOk) break;
                if(PCmd->GetNext<uint32_t>(&Value[Cnt]) != retvOk) { PShell->BadParam(); return; } // Addr exsits, value is not
                Cnt++;
            }
            if(!Cnt) { PShell->BadParam(); return; }
            // Put regs to settings
            for(uint32_t i=0; i<Cnt; i++) {
                Settings.Hmc821.Regs[i].Addr = Addr[i];
                Settings.Hmc821.Regs[i].Value = Value[i];
            }
            // Save settings
            Settings.WhatSaved = SAVED_REGS_FLAG;
            Settings.SavedRegsCnt = Cnt;
            if(Settings.Save() == retvOk) PShell->Ok();
            else PShell->Failure();
        }
        else if(PCmd->NameIs("GetRegs")) {
            if(Settings.WhatSaved == SAVED_REGS_FLAG and Settings.SavedRegsCnt <= REG_CNT) {
                PShell->Print("RGetRegs ");
                for(uint32_t i=0; i<Settings.SavedRegsCnt; i++) {
                    PShell->Print("0x%X 0x%X, ", Settings.Hmc821.Regs[i].Addr, Settings.Hmc821.Regs[i].Value);
                }
                PShell->EOL();
            }
            else PShell->Print("RGetRegs Empty\r\n");
        }
        else if(PCmd->NameIs("ClearRegs")) {
            Settings.WhatSaved = SAVED_NONE_FLAG;
            if(Settings.Save() == retvOk) PShell->Ok();
            else PShell->Failure();
        }
    }
#endif
#if 1 // ==== ADF5356 (KuKonv & IKS) ====
    else if(SelfInfo.Type == devtKUKonv or SelfInfo.Type == devtIKS) {
        if(PCmd->NameIs("SetRegs")) {
            uint32_t Cnt = 0, Value;
            while(true) {
                if(PCmd->GetNext<uint32_t>(&Value) != retvOk) break;
                Adf5356.WriteReg(Value);
                Cnt++;
            }
            if(Cnt) PShell->Ok();
            else PShell->BadParam();
        }
        else if(PCmd->NameIs("SaveRegs")) {
            uint32_t Cnt = 0, Value[REG_CNT];
            // Get regs
            while(true) {
                if(Cnt >= REG_CNT) { PShell->BadParam(); return; }
                if(PCmd->GetNext<uint32_t>(&Value[Cnt]) != retvOk) break;
                Cnt++;
            }
            if(!Cnt) { PShell->BadParam(); return; }
            // Put regs to settings
            for(uint32_t i=0; i<Cnt; i++) {
                Settings.Adf5356.Regs[i] = Value[i];
//                Printf("%X\r", Value[i]);
            }
            // Save settings
            Settings.WhatSaved = SAVED_REGS_FLAG;
            Settings.SavedRegsCnt = Cnt;
            if(Settings.Save() == retvOk) PShell->Ok();
            else PShell->Failure();
        }
        else if(PCmd->NameIs("GetRegs")) {
            if(Settings.WhatSaved == SAVED_REGS_FLAG and Settings.SavedRegsCnt <= REG_CNT) {
                PShell->Print("RGetRegs ");
                for(uint32_t i=0; i<Settings.SavedRegsCnt; i++) {
                    PShell->Print("0x%X ", Settings.Adf5356.Regs[i]);
                }
                PShell->EOL();
            }
            else PShell->Print("RGetRegs Empty\r\n");
        }
        else if(PCmd->NameIs("ClearRegs")) {
            Settings.WhatSaved = SAVED_NONE_FLAG;
            if(Settings.Save() == retvOk) PShell->Ok();
            else PShell->Failure();
        }

        else if(PCmd->NameIs("CalcRegs")) {
            if(PCmd->GetNextDouble(&Adf5356.fref) != retvOk) { PShell->BadParam(); return; }
            if(PCmd->GetNextDouble(&Adf5356.step) != retvOk) { PShell->BadParam(); return; }
            if(PCmd->GetNextDouble(&Adf5356.fd  ) != retvOk) { PShell->BadParam(); return; }
            if(PCmd->GetNextDouble(&Adf5356.fvco) != retvOk) { PShell->BadParam(); return; }
            Adf5356.CalcRegs();
            Adf5356.PrintRegs();
            PShell->Ok();
        }
        else if(PCmd->NameIs("CalcRegsAndSet")) {
            if(PCmd->GetNextDouble(&Adf5356.fref) != retvOk) { PShell->BadParam(); return; }
            if(PCmd->GetNextDouble(&Adf5356.step) != retvOk) { PShell->BadParam(); return; }
            if(PCmd->GetNextDouble(&Adf5356.fd  ) != retvOk) { PShell->BadParam(); return; }
            if(PCmd->GetNextDouble(&Adf5356.fvco) != retvOk) { PShell->BadParam(); return; }
            Adf5356.CalcRegs();
            Adf5356.SetRegs();
            PShell->Ok();
        }
        else if(PCmd->NameIs("SaveSynthParams")) {
            double fref, step, fd, fvco;
            if(PCmd->GetNextDouble(&fref) != retvOk) { PShell->BadParam(); return; }
            if(PCmd->GetNextDouble(&step) != retvOk) { PShell->BadParam(); return; }
            if(PCmd->GetNextDouble(&fd  ) != retvOk) { PShell->BadParam(); return; }
            if(PCmd->GetNextDouble(&fvco) != retvOk) { PShell->BadParam(); return; }
            Settings.Adf5356.fref = fref;
            Settings.Adf5356.step = step;
            Settings.Adf5356.fd   = fd;
            Settings.Adf5356.fvco = fvco;
            Settings.WhatSaved = SAVED_PARAMS_FLAG;
            if(Settings.Save() == retvOk) PShell->Ok();
            else PShell->Failure();
        }
    }
#endif
}

uint8_t Ping(uint8_t Addr) {
    if(RS485Int.SendCmd(TIMEOUT_MID_ms, "Ping", Addr) == retvOk) {
        if(RS485Int.Reply.NameIs("Ok")) return retvOk;
    }
    return retvNoAnswer;
}

void ProcessCmdForSlave(Shell_t *PShell, Cmd_t *PCmd, uint32_t Addr) {
    // Do not waste time if there is no device on the bus
    if(Ping(Addr) != retvOk) {
        PShell->NoAnswer();
        return;
    }

    // ==== Special cases ====
    if(PCmd->NameIs("wSPIFile")) {
        uint8_t Params;
        if(PCmd->GetNext<uint8_t>(&Params) != retvOk) { PShell->BadParam(); return; }
        uint32_t Len;
        if(PCmd->GetNext<uint32_t>(&Len) != retvOk or Len > FILEBUF_SZ) { PShell->BadParam(); return; }
        // Receive data from host
        if(PShell->ReceiveBinaryToBuf(FileBuf, Len, TIMEOUT_LONG_ms) == retvOk) {
//            PShell->Print("File rcvd\r\n");
            // Send data away
            if(RS485Int.SendCmdAndTransmitBuf(TIMEOUT_LONG_ms, FileBuf, Len, PCmd->Name, Addr, "%u %u", Params, Len) == retvOk) {
                if(RS485Int.Reply.NameIs("Ok")) PShell->Ok();
                else PShell->Failure();
            }
            else PShell->Timeout();
        }
        else PShell->Timeout();
    }

    else if(PCmd->NameIs("rSpiFile")) {
        uint32_t Len;
        if(PCmd->GetNext<uint32_t>(&Len) != retvOk or Len > FILEBUF_SZ) { PShell->BadParam(); return; }
        // Get data from there
        if(RS485Int.SendCmdAndReceiveBuf(TIMEOUT_LONG_ms, FileBuf, Len, PCmd->Name, Addr, "%u", Len) == retvOk) {
            // Send data to host
            if(PShell->TransmitBinaryFromBuf(FileBuf, Len, TIMEOUT_LONG_ms) != retvOk) PShell->Timeout();
        }
        else PShell->Timeout();
    }

    if(PCmd->NameIs("UpdateFW")) {
        uint32_t Len, CrcIn;
        if(PCmd->GetNext<uint32_t>(&Len) != retvOk or Len > FILEBUF_SZ) { PShell->BadParam(); return; }
        if(PCmd->GetNext<uint32_t>(&CrcIn) != retvOk) { PShell->BadParam(); return; }
        // Receive data
        Led.StartOrRestart(lsqWriting);
        if(PShell->ReceiveBinaryToBuf(FileBuf, Len, TIMEOUT_LONG_ms) == retvOk) {
            PShell->Print("FileRcvd\r\n");
            // Check CRC
            uint16_t crc = Crc::CalculateCRC16HWDMA(FileBuf, Len);
            if(crc == CrcIn) {
                // Send data away
                if(RS485Int.SendCmdAndTransmitBuf(TIMEOUT_LONG_ms, FileBuf, Len, PCmd->Name, Addr, "%u 0x%X", Len, CrcIn) == retvOk) {
                    if(RS485Int.Reply.NameIs("Ok")) PShell->Ok();
                    else PShell->Failure();
                }
                else PShell->Timeout();
            }
            else PShell->CRCError();
        }
        else PShell->Timeout();
        Led.StartOrRestart(lsqCmd);
    }

    // ==== Non-special command ====
    else {
        if(RS485Int.SendCmd(TIMEOUT_MID_ms, PCmd->Name, Addr, PCmd->GetRemainder()) == retvOk) {
            PShell->Print("%S %S\r\n", RS485Int.Reply.Name, RS485Int.Reply.GetRemainder());
        }
        else PShell->NoAnswer();
    }
}
#endif
