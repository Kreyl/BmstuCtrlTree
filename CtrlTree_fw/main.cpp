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

__attribute__((section ("DATA_RAM")))
CmdUart_t Uart{CmdUartParams};
__attribute__((section ("DATA_RAM")))
CmdUart485_t RS485Ext{RS485ExtParams, RS485_EXT_TXEN};
//CmdUart485_t RS485Int{RS485IntParams, RS485_INT_TXEN};
__attribute__((section ("DATA_RAM")))
CmdUart_t RS232{RS232Params};

bool UsbIsConnected = false;
LedBlinker_t Led{LED_PIN};
DeviceList_t DevList;

uint8_t OnMasterCmd(Shell_t *PShell, Cmd_t *PCmd);
void OnSlaveCmd(Shell_t *PShell, Cmd_t *PCmd);
void ProcessCmdForSlave(Shell_t *PShell, Cmd_t *PCmd);

static TmrKL_t TmrOneSecond {TIME_MS2I(999), evtIdEverySecond, tktPeriodic}; // Measure battery periodically
__attribute__((section ("DATA_RAM")))
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
    void Init() {
        if      (PSpi == SPI1) { rccEnableSPI1(FALSE); }
        else if (PSpi == SPI2) { rccEnableSPI2(FALSE); }
        PinSetupOut(PGpio, Cs, omPushPull);
        PinSetHi(PGpio, Cs);
        PinSetupAlterFunc(PGpio, Sck,  omPushPull, pudNone, AF, psVeryHigh);
        PinSetupAlterFunc(PGpio, Miso, omPushPull, pudNone, AF, psVeryHigh);
        PinSetupAlterFunc(PGpio, Mosi, omPushPull, pudNone, AF, psVeryHigh);
    }

    void Transmit(uint8_t Params, uint8_t *ptr, uint32_t Len) {
        PinSetLo(PGpio, Cs);
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
        PinSetHi(PGpio, Cs);
        PSpi->CR1 &= ~SPI_CR1_SPE; // Disable SPI
    }
};

DevSpi_t Spi1{SPI1, GPIOA, 5,6,7,4, AF5};
DevSpi_t Spi2{SPI2, GPIOB, 13,14,15,12, AF5};

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

    SelfInfo.Load(EE_SELF_ADDR);
    DevList.Load();
    Settings.Load();

    GpioReg.Init();
    GpioReg.Set(Settings.PowerOnGPIO);

    Spi1.Init();
    Spi2.Init();

    FileBuf[0] = 0xAA;

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
	Printf("Cmd: %S\r", PCmd->Name);
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
    else { // Command is not direct
        uint8_t FAddr;
        if(SelfInfo.Type == devtHFBlock) { // if we are master
            if(OnMasterCmd(PShell, PCmd) != retvOk) { // if cmd was not found in master cmds
                if(PCmd->GetNext<uint8_t>(&FAddr) == retvOk) { // Got address
                    if(FAddr == ADDR_MASTER) OnSlaveCmd(PShell, PCmd);
                    else ProcessCmdForSlave(PShell, PCmd);
                }
                else PShell->Print("BadParam\r\n");
            }
        }
        else { // We are Slave
            if(PCmd->GetNext<uint8_t>(&FAddr) == retvOk and FAddr == SelfInfo.Addr) {
                OnSlaveCmd(PShell, PCmd);
            }
        }
    } // not direct
}

uint8_t OnMasterCmd(Shell_t *PShell, Cmd_t *PCmd) {
    if(PCmd->NameIs("Scan")) {
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
            PShell->Print("\r\n");
        }
    }

    else if(PCmd->NameIs("AddDevice")) {
        if(DevList.Cnt() >= DEV_CNT_MAX) { PShell->Print("TableFull\r\n"); return retvOk; }
        uint8_t Addr = 0, Type = 0;
        if(PCmd->GetNext<uint8_t>(&Addr) != retvOk or !AddrIsOk(Addr)) { PShell->Print("BadParam\r\n"); return retvOk; }
        if(PCmd->GetNext<uint8_t>(&Type) != retvOk or !TypeIsOk(Type)) { PShell->Print("BadParam\r\n"); return retvOk; }
        if(DevList.ContainsAddr(Addr)) { PShell->Print("DeviceExists\r\n"); return retvOk; }
        char *PName;
        if(PCmd->GetNextString(&PName) != retvOk) { PShell->Print("BadParam\r\n"); return retvOk; }
        int Len = strlen(PName);
        if(Len > DEV_NAME_LEN) { PShell->Print("BadParam\r\n"); return retvOk; }
        // All is finally ok
        DevList.Add(Addr, (DevType_t)Type, PName);
        // Try to change real device parameters
        // Todo
        PShell->Ack(retvOk);
        DevList.Save();
    }

    else if(PCmd->NameIs("PutDevice")) {
        uint8_t Addr = 0, Type = 0;
        if(PCmd->GetNext<uint8_t>(&Addr) != retvOk or !AddrIsOk(Addr)) { PShell->Print("BadParam\r\n"); return retvOk; }
        if(PCmd->GetNext<uint8_t>(&Type) != retvOk or !TypeIsOk(Type)) { PShell->Print("BadParam\r\n"); return retvOk; }
        char *PName;
        if(PCmd->GetNextString(&PName) != retvOk) { PShell->Print("BadParam\r\n"); return retvOk; }
        int Len = strlen(PName);
        if(Len > DEV_NAME_LEN) { PShell->Print("BadParam\r\n"); return retvOk; }
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
        // Try to change real device parameters
        // Todo
        PShell->Ack(retvOk);
        DevList.Save();
    }

    else if(PCmd->NameIs("DelDevice")) {
        uint8_t Addr = 0;
        if(PCmd->GetNext<uint8_t>(&Addr) != retvOk) { PShell->Print("BadParam\r\n"); return retvOk; }
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
        PShell->Print("\r\n");
    }
    else return retvNotFound;
    return retvOk;
}

void WRSPI(Shell_t *PShell, Cmd_t *PCmd, DevSpi_t &ASpi) {
    uint8_t Params;
    if(PCmd->GetNext<uint8_t>(&Params) != retvOk) { PShell->Print("BadParam\r\n"); return; }
    uint8_t *p = FileBuf;
    while(PCmd->GetNext<uint8_t>(p) == retvOk) p++; // Get what to send
    uint32_t Len = p - FileBuf;
    ASpi.Transmit(Params, FileBuf, Len);
    // Reply
    p = FileBuf;
    PShell->Print("%S", PCmd->Name);
    while(Len--) PShell->Print(" 0x%02X", *p++);
    PShell->Print("\r\n");
}

void wSpiFile(Shell_t *PShell, Cmd_t *PCmd, DevSpi_t &ASpi) {
    uint8_t Params;
    if(PCmd->GetNext<uint8_t>(&Params) != retvOk) { PShell->Print("BadParam\r\n"); return; }
    uint32_t Len;
    if(PCmd->GetNext<uint32_t>(&Len) != retvOk or Len > FILEBUF_SZ) { PShell->Print("BadParam\r\n"); return; }
    // Receive data
    if(PShell->ReceiveBinaryToBuf(FileBuf, Len, 9999) == retvOk) {
        ASpi.Transmit(Params, FileBuf, Len);
        PShell->Ack(0);
    }
    else PShell->Print("Timeout\r\n");
}

void rSpiFile(Shell_t *PShell, Cmd_t *PCmd, DevSpi_t &ASpi) {
    uint32_t Len;
    if(PCmd->GetNext<uint32_t>(&Len) != retvOk or Len > FILEBUF_SZ) { PShell->Print("BadParam\r\n"); return; }
    if(PShell->TransmitBinaryFromBuf(FileBuf, Len, 9999) != retvOk) PShell->Print("Timeout\r\n");
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
#if 1 // ==== Common ====
    // ==== GPIO Reg ====
    else if(PCmd->NameIs("SetGPIO")) {
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

    // ==== SPI ====
    else if(PCmd->NameIs("WRSPI1")) WRSPI(PShell, PCmd, Spi1);
    else if(PCmd->NameIs("WRSPI2")) WRSPI(PShell, PCmd, Spi2);
    else if(PCmd->NameIs("wSPIFile1")) wSpiFile(PShell, PCmd, Spi1);
    else if(PCmd->NameIs("wSPIFile2")) wSpiFile(PShell, PCmd, Spi2);
    else if(PCmd->NameIs("rSpiFile1")) rSpiFile(PShell, PCmd, Spi1);
    else if(PCmd->NameIs("rSpiFile2")) rSpiFile(PShell, PCmd, Spi2);


#endif

#if 1 // ==== Thermostating ====

#endif

}

void ProcessCmdForSlave(Shell_t *PShell, Cmd_t *PCmd) {
    Printf("%S: %S\r\n", __FUNCTION__, PCmd->Name);
}

#endif
