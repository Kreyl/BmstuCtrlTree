/*
 * cmd_uart.cpp
 *
 *  Created on: 15.04.2013
 *      Author: kreyl
 */

#include "MsgQ.h"
#include <string.h>
#include "uart2.h"
#include "kl_lib.h"

#if 1 // ========================= Base UART ===================================
#if 1 // ==== TX ====

#if UART_USE_TXE_IRQ // ============ TxC IRQ =============
static ftVoidVoid ITxC1IrqCallback = nullptr;
static ftVoidVoid ITxC2IrqCallback = nullptr;
static ftVoidVoid ITxC3IrqCallback = nullptr;
#if defined UART4
static ftVoidVoid ITxC4IrqCallback = nullptr;
#endif
#if defined UART5
static ftVoidVoid ITxC5IrqCallback = nullptr;
#endif
#if defined USART6
static ftVoidVoid ITxC6IrqCallback = nullptr;
#endif

void BaseUart_t::EnableTCIrq(const uint32_t Priority, ftVoidVoid ACallback) {
    ITxC1IrqCallback = ACallback;
    if(Params->Uart == USART1) {
        ITxC1IrqCallback = ACallback;
        nvicEnableVector(USART1_IRQn, Priority);
    }
    else if(Params->Uart == USART2) {
        ITxC2IrqCallback = ACallback;
        nvicEnableVector(USART2_IRQn, Priority);
    }
    else if(Params->Uart == USART3) {
        ITxC3IrqCallback = ACallback;
#if defined STM32L4XX
        nvicEnableVector(USART3_IRQn, Priority);
#elif defined STM32F0XX
        nvicEnableVector(USART3_4_IRQn, Priority);
#endif
    }
#if defined UART4
    else if(Params->Uart == UART4) {
        ITxC4IrqCallback = ACallback;
        nvicEnableVector(UART4_IRQn, Priority);
    }
#endif
#if defined UART5
    else if(Params->Uart == UART5) {
        ITxC5IrqCallback = ACallback;
        nvicEnableVector(UART5_IRQn, Priority);
    }
#endif
#if defined USART6
    else if(Params->Uart == USART6) {
        ITxC6IrqCallback = ACallback;
        nvicEnableVector(USART6_IRQn, Priority);
    }
#endif
    Params->Uart->CR1 |= USART_CR1_TCIE;
}
#endif

#if 1 // ===== IRQs =====
extern "C" {
void VectorD4() {   // USART1
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = USART1->ISR;
    if(SR & USART_ISR_CMF) {
        USART1->ICR = USART_ICR_CMCF;
        EvtQMain.SendNowOrExitI(EvtMsg_t(evtIdUartCmdRcvd, 1)); // 1 means USART1
    }
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

void VectorD8() {   // USART2
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = USART2->ISR;
    if(SR & USART_ISR_CMF) {
        USART2->ICR = USART_ICR_CMCF;
        EvtQMain.SendNowOrExitI(EvtMsg_t(evtIdUartCmdRcvd, 2));
    }
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

void VectorDC() {   // USART3
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = USART3->ISR;
    if(SR & USART_ISR_CMF) {
        USART3->ICR = USART_ICR_CMCF;
        EvtQMain.SendNowOrExitI(EvtMsg_t(evtIdUartCmdRcvd, 3));
    }
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

void Vector110() {   // UART4
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = UART4->ISR;
    if(SR & USART_ISR_CMF) {
        UART4->ICR = USART_ICR_CMCF;
        EvtQMain.SendNowOrExitI(EvtMsg_t(evtIdUartCmdRcvd, 4));
    }
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}
void Vector114() {   // UART5
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = UART5->ISR;
    if(SR & USART_ISR_CMF) {
        UART5->ICR = USART_ICR_CMCF;
        EvtQMain.SendNowOrExitI(EvtMsg_t(evtIdUartCmdRcvd, 5));
    }
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

} // extern C
#endif

// Wrapper for TX IRQ
extern "C"
void DmaUartTxIrq(void *p, uint32_t flags) { ((BaseUart_t*)p)->IRQDmaTxHandler(); }

// ==== TX DMA IRQ ====
void BaseUart_t::IRQDmaTxHandler() {
    dmaStreamDisable(PDmaTx);    // Registers may be changed only when stream is disabled
    IFullSlotsCount -= ITransSize;
    PRead += ITransSize;
    if(PRead >= (TXBuf + UART_TXBUF_SZ)) PRead = TXBuf; // Circulate pointer

    if(IFullSlotsCount == 0) {  // Nothing left to send
        IDmaIsIdle = true;
    }
    else ISendViaDMA();
}

void BaseUart_t::ISendViaDMA() {
    uint32_t PartSz = (TXBuf + UART_TXBUF_SZ) - PRead; // Cnt from PRead to end of buf
    ITransSize = MIN_(IFullSlotsCount, PartSz);
    if(ITransSize != 0) {
        IDmaIsIdle = false;
        dmaStreamSetMemory0(PDmaTx, PRead);
        dmaStreamSetTransactionSize(PDmaTx, ITransSize);
        dmaStreamSetMode(PDmaTx, Params->DmaModeTx);
        dmaStreamEnable(PDmaTx);
    }
}

uint8_t BaseUart_t::IPutByte(uint8_t b) {
    if(IFullSlotsCount >= UART_TXBUF_SZ) return retvOverflow;
    *PWrite++ = b;
    if(PWrite >= &TXBuf[UART_TXBUF_SZ]) PWrite = TXBuf;   // Circulate buffer
    IFullSlotsCount++;
    return retvOk;
}

void BaseUart_t::IStartTransmissionIfNotYet() {
    if(IDmaIsIdle) ISendViaDMA();
}

uint8_t BaseUart_t::IPutByteNow(uint8_t b) {
    while(!(Params->Uart->ISR & USART_ISR_TXE));
    Params->Uart->TDR = b;
    while(!(Params->Uart->ISR & USART_ISR_TXE));
    return retvOk;
}
#endif // TX

#if 1 // ==== RX ====
uint8_t BaseUart_t::GetByte(uint8_t *b) {
#if defined STM32F2XX || defined STM32F4XX || defined STM32F7XX
    int32_t WIndx = UART_RXBUF_SZ - PDmaRx->stream->NDTR;
#else
    int32_t WIndx = UART_RXBUF_SZ - PDmaRx->channel->CNDTR;
#endif
    int32_t BytesCnt = WIndx - RIndx;
    if(BytesCnt < 0) BytesCnt += UART_RXBUF_SZ;
    if(BytesCnt == 0) return retvEmpty;
    *b = IRxBuf[RIndx++];
    if(RIndx >= UART_RXBUF_SZ) RIndx = 0;
    return retvOk;
}
#endif // RX

#if 1 // ==== Init ====
void BaseUart_t::Init() {
    AlterFunc_t PinAF = AF1;
#if 1 // ==== Tx pin ====
#if defined STM32L4XX || defined STM32L1XX || defined STM32F2XX
    PinAF = AF7;
#if defined UART4
    if(Params->Uart == UART4) PinAF = AF8;
#endif
#if defined UART5
    if(Params->Uart == UART5) PinAF = AF8;
#endif
#if defined USART6
    if(Params->Uart == USART6) PinAF = AF8;
#endif

#elif defined STM32F0XX
    if(Params->PGpioTx == GPIOA) PinAF = AF1;
    else if(Params->PGpioTx == GPIOB) PinAF = AF0;
#elif defined STM32F7XX
    if(Params->Uart == USART1) { // AF4, AF7
        if(Params->PGpioTx == GPIOB and Params->PinTx == 14) PinAF = AF4;
        else PinAF = AF7;
    }
    else if(Params->Uart == USART2 or Params->Uart == USART3) PinAF = AF7;
    else if(Params->Uart == UART4) { // AF6, AF8
        if(Params->PGpioTx == GPIOA and Params->PinTx == 12) PinAF = AF6;
        else PinAF = AF8;
    }
    else if(Params->Uart == UART5) { // AF1, AF7, AF8
        if(Params->PGpioTx == GPIOB and Params->PinTx == 6) PinAF = AF1;
        else if(Params->PGpioTx == GPIOB and Params->PinTx == 9) PinAF = AF7;
        else PinAF = AF8;
    }
    else if(Params->Uart == USART6 or Params->Uart == UART8) PinAF = AF8;
    else if(Params->Uart == UART7) { // AF8, AF12
        if(Params->PGpioTx == GPIOA or Params->PGpioTx == GPIOB) PinAF = AF12;
        else PinAF = AF8;
    }
#else
#error "UART AF not defined"
#endif
    PinSetupAlterFunc(Params->PGpioTx, Params->PinTx, omPushPull, pudNone, PinAF);
#endif

#if 1 // Setup independent clock if possible and required
#if defined STM32F072xB
    if(Params->UseIndependedClock) {
        Clk.EnableHSI();    // HSI used as independent clock
        if     (Params->Uart == USART1) RCC->CFGR3 |= RCC_CFGR3_USART1SW_HSI;
        else if(Params->Uart == USART2) RCC->CFGR3 |= RCC_CFGR3_USART2SW_HSI;
    }
#elif defined STM32L4XX
    uint32_t Offset = 0; // Usart1
    if(Params->Uart == USART2) Offset = 2;
    else if(Params->Uart == USART3) Offset = 4;
    else if(Params->Uart == UART4) Offset = 6;
    else if(Params->Uart == UART5) Offset = 8;
    RCC->CCIPR &= ~(0b11UL << Offset); // Clear current bits
    RCC->CCIPR |= ((uint32_t)Params->ClkSrc) << Offset;
    // Enable HSI if needed
    if(Params->ClkSrc == uartclkHSI) Clk.EnableHSI();
#elif defined STM32F7XX
    uint32_t Offset = 0; // Usart1
    if(Params->Uart == USART2) Offset = 2;
    else if(Params->Uart == USART3) Offset = 4;
    else if(Params->Uart == UART4) Offset = 6;
    else if(Params->Uart == UART5) Offset = 8;
    else if(Params->Uart == USART6) Offset = 10;
    else if(Params->Uart == UART7) Offset = 12;
    else if(Params->Uart == UART8) Offset = 14;
    RCC->DCKCFGR2 &= ~(0b11UL << Offset); // Clear current bits
    RCC->DCKCFGR2 |= ((uint32_t)Params->ClkSrc) << Offset;
    // Enable HSI if needed
    if(Params->ClkSrc == uartclkHSI) Clk.EnableHSI();
#endif
#endif // Independent clock

#if 1 // ==== Clock ====
    if     (Params->Uart == USART1) { rccEnableUSART1(FALSE); }
    else if(Params->Uart == USART2) { rccEnableUSART2(FALSE); }
#if defined USART3
    else if(Params->Uart == USART3) { rccEnableUSART3(FALSE); }
#endif
#if defined UART4
    else if(Params->Uart == UART4) { rccEnableUART4(FALSE); }
#endif
#if defined UART5
    else if(Params->Uart == UART5) { rccEnableUART5(FALSE); }
#endif
#if defined USART6
    else if(Params->Uart == USART6) { rccEnableUSART6(FALSE); }
#endif
#if defined UART7
    else if(Params->Uart == UART7) { rccEnableUART7(FALSE); }
#endif
#if defined UART8
    else if(Params->Uart == UART8) { rccEnableUART8(FALSE); }
#endif
#endif // Clock

    OnClkChange();  // Setup baudrate

    Params->Uart->CR2 = 0;  // Nothing that interesting there
    // ==== DMA ====
    // Remap DMA request if needed
#if defined STM32F0XX
    if(Params->PDmaTx == STM32_DMA1_STREAM4) SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1TX_DMA_RMP;
#endif
    PDmaTx = dmaStreamAlloc(Params->DmaTxID, IRQ_PRIO_MEDIUM, DmaUartTxIrq, this);
    dmaStreamSetPeripheral(PDmaTx, &Params->Uart->TDR);
    dmaStreamSetMode      (PDmaTx, Params->DmaModeTx);
    IDmaIsIdle = true;

    // ==== RX ====
    Params->Uart->CR1 = USART_CR1_TE | USART_CR1_RE;        // TX & RX enable
    Params->Uart->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;    // Enable DMA at TX & RX
    // ==== Rx pin ====
#if defined STM32L4XX || defined STM32L1XX || defined STM32F2XX
    PinAF = AF7; // for all USARTs save 4/5/6
#if defined UART4
    if(Params->Uart == UART4) PinAF = AF8;
#endif
#if defined UART5
    if(Params->Uart == UART5) PinAF = AF8;
#endif
#if defined USART6
    if(Params->Uart == USART6) PinAF = AF8;
#endif
#elif defined STM32F0XX
    if(Params->PGpioRx == GPIOA) PinAF = AF1;
    else if(Params->PGpioRx == GPIOB) PinAF = AF0;
#elif defined STM32F7XX
    if(Params->Uart == USART1) { // AF4, AF7
        if(Params->PGpioTx == GPIOB and Params->PinTx == 15) PinAF = AF4;
        else PinAF = AF7;
    }
    else if(Params->Uart == USART2 or Params->Uart == USART3) PinAF = AF7;
    else if(Params->Uart == UART4) { // AF6, AF8
        if(Params->PGpioTx == GPIOA and Params->PinTx == 11) PinAF = AF6;
        else PinAF = AF8;
    }
    else if(Params->Uart == UART5) { // AF1, AF7, AF8
        if(Params->PGpioTx == GPIOB and Params->PinTx == 5) PinAF = AF1;
        else if(Params->PGpioTx == GPIOB and Params->PinTx == 8) PinAF = AF7;
        else PinAF = AF8;
    }
    else if(Params->Uart == USART6 or Params->Uart == UART8) PinAF = AF8;
    else if(Params->Uart == UART7) { // AF8, AF12
        if(Params->PGpioTx == GPIOA or Params->PGpioTx == GPIOB) PinAF = AF12;
        else PinAF = AF8;
    }
#else
#error "UART AF not defined"
#endif
    PinSetupAlterFunc(Params->PGpioRx, Params->PinRx, omOpenDrain, pudPullUp, PinAF);
    // Remap DMA request if needed
#if defined STM32F0XX
    if(Params->PDmaRx == STM32_DMA1_STREAM5) SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1RX_DMA_RMP;
#endif
    // DMA
    PDmaRx = dmaStreamAlloc(Params->DmaRxID, IRQ_PRIO_MEDIUM, nullptr, NULL);
    dmaStreamSetPeripheral(PDmaRx, &Params->Uart->RDR);
    dmaStreamSetMemory0   (PDmaRx, IRxBuf);
    dmaStreamSetTransactionSize(PDmaRx, UART_RXBUF_SZ);
    dmaStreamSetMode      (PDmaRx, Params->DmaModeRx);
    dmaStreamEnable       (PDmaRx);

    // Enable IRQ on <LF>
    Params->Uart->CR2 |= ((uint32_t)('\n')) << 24; // What to recoghize
    Params->Uart->CR1 |= USART_CR1_CMIE; // Enable IRQ on match
    if(Params->Uart == USART1) nvicEnableVector(USART1_IRQn, IRQ_PRIO_LOW);
    else if(Params->Uart == USART2) nvicEnableVector(USART2_IRQn, IRQ_PRIO_LOW);
    else if(Params->Uart == USART3) nvicEnableVector(USART3_IRQn, IRQ_PRIO_LOW);
    else if(Params->Uart == UART4) nvicEnableVector(UART4_IRQn, IRQ_PRIO_LOW);
    else if(Params->Uart == UART5) nvicEnableVector(UART5_IRQn, IRQ_PRIO_LOW);

    Params->Uart->CR1 |= USART_CR1_UE;    // Enable USART
}

void BaseUart_t::Shutdown() {
    Params->Uart->CR1 &= ~USART_CR1_UE; // UART Disable
    if     (Params->Uart == USART1) { rccDisableUSART1(); }
    else if(Params->Uart == USART2) { rccDisableUSART2(); }
#if defined USART3
    else if(Params->Uart == USART3) { rccDisableUSART3(); }
#endif
#if defined UART4
    else if(Params->Uart == UART4) { rccDisableUART4(); }
#endif
#if defined UART5
    else if(Params->Uart == UART5) { rccDisableUART5(); }
#endif
#if defined USART6
    else if(Params->Uart == USART6) { rccDisableUSART6(); }
#endif
#if defined UART7
    else if(Params->Uart == UART7) { rccDisableUART7(); }
#endif
#if defined UART8
    else if(Params->Uart == UART8) { rccDisableUART8(); }
#endif
}

void BaseUart_t::OnClkChange() {
#if defined STM32L1XX || defined STM32F1XX
    if(Params->Uart == USART1) Params->Uart->BRR = Clk.APB2FreqHz / Params->Baudrate;
    else                       Params->Uart->BRR = Clk.APB1FreqHz / Params->Baudrate;
#elif defined STM32F072xB
    if(Params->Uart == USART1 or Params->Uart == USART2) Params->Uart->BRR = HSI_FREQ_HZ / Params->Baudrate;
    else Params->Uart->BRR = Clk.APBFreqHz / Params->Baudrate;
#elif defined STM32F0XX
    Params->Uart->BRR = Clk.APBFreqHz / IBaudrate;
#elif defined STM32F2XX || defined STM32F4XX
    if(Params->Uart == USART1 or Params->Uart == USART6) Params->Uart->BRR = Clk.APB2FreqHz / Params->Baudrate;
    else Params->Uart->BRR = Clk.APB1FreqHz / Params->Baudrate;
#elif defined STM32L4XX
    switch(Params->ClkSrc) {
        case uartclkPCLK:
            if(Params->Uart == USART1) Params->Uart->BRR = Clk.APB2FreqHz / Params->Baudrate;
            else Params->Uart->BRR = Clk.APB1FreqHz / Params->Baudrate;
            break;
        case uartclkSYSCLK:
            Params->Uart->BRR = Clk.GetSysClkHz() / Params->Baudrate;
            break;
        case uartclkHSI:
            Params->Uart->BRR = HSI_FREQ_HZ / Params->Baudrate;
            break;
        case uartclkLSE:
            Params->Uart->BRR = LSE_FREQ_HZ / Params->Baudrate;
            break;
    } // switch
#elif defined STM32F7XX
    switch(Params->ClkSrc) {
        case uartclkPCLK:
            if(Params->Uart == USART1 or Params->Uart == USART6)
                Params->Uart->BRR = Clk.APB2FreqHz / Params->Baudrate;
            else Params->Uart->BRR = Clk.APB1FreqHz / Params->Baudrate;
            break;
        case uartclkSYSCLK:
            Params->Uart->BRR = Clk.GetSysClkHz() / Params->Baudrate;
            break;
        case uartclkHSI:
            Params->Uart->BRR = HSI_FREQ_HZ / Params->Baudrate;
            break;
        case uartclkLSE:
            Params->Uart->BRR = LSE_FREQ_HZ / Params->Baudrate;
            break;
    } // switch
#else
#error "UART BRR not defined"
#endif
}
#endif // Init

#endif // Base UART

#if BYTE_UART_EN // ========================= Byte UART ========================
static const UartParams_t ByteUartParams = {
        FT_UART,
        FT_GPIO, FT_TX,
        FT_GPIO, FT_RX,
        // DMA
        FT_UART_DMA_TX, FT_UART_DMA_RX,
        UART_DMA_TX_MODE(FT_UART_DMA_CHNL), UART_DMA_RX_MODE(FT_UART_DMA_CHNL),
#if defined STM32F072xB || defined STM32L4XX
        false    // Use independed clock
#endif
};

ByteUart_t ByteUart(&ByteUartParams);
thread_reference_t IByteRxThd = nullptr;

static THD_WORKING_AREA(waByteUartRxThread, 128);
__noreturn
static void ByteUartRxThread(void *arg) {
    chRegSetThreadName("ByteUartRx");
    while(true) {
        chThdSleepMilliseconds(UART_RX_POLLING_MS);
        ByteUart.IRxTask();
    }
}

void ByteUart_t::IRxTask() {
    if(CmdProcessInProgress) return;    // Busy processing cmd
    // Iterate received bytes
//    Printf("1\r");
    uint8_t b;
    while(GetByte(&b) == retvOk) {
        if(Cmd.PutChar(b) == pdrNewCmd) {
            EvtMsg_t Msg(evtIdByteCmd, (ByteShell_t*)this);
            CmdProcessInProgress = (EvtQMain.SendNowOrExit(Msg) == retvOk);
        }
    }
}

void ByteUart_t::Init(uint32_t ABaudrate) {
    BaseUart_t::Init(ABaudrate);
#if UART_RX_ENABLED
    // Create RX Thread if not created
    if(IByteRxThd == nullptr) {
        IByteRxThd = chThdCreateStatic(waByteUartRxThread, sizeof(waByteUartRxThread),
                NORMALPRIO, ByteUartRxThread, NULL);
    }
#endif
}
#endif