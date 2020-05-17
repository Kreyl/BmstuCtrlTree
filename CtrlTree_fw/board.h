/*
 * board.h
 *
 *  Created on: 05 08 2018
 *      Author: Kreyl
 */

#pragma once

// ==== General ====
#define APP_NAME            "BMSTU CtrlTree"

// MCU type as defined in the ST header.
#define STM32L476xx

// Freq of external crystal if any. Leave it here even if not used.
#define CRYSTAL_FREQ_HZ         12000000

// OS timer settings
#define STM32_ST_IRQ_PRIORITY   2
#define STM32_ST_USE_TIMER      5
#define SYS_TIM_CLK             (Clk.APB1FreqHz)    // Timer 5 is clocked by APB1

//  Periphery
#define I2C1_ENABLED            TRUE
#define I2C2_ENABLED            FALSE
#define I2C3_ENABLED            FALSE
#define SIMPLESENSORS_ENABLED   TRUE
#define BUTTONS_ENABLED         FALSE

#define ADC_REQUIRED            FALSE
#define STM32_DMA_REQUIRED      TRUE    // Leave this macro name for OS

#define FILEBUF_SZ              65535

#if 1 // ========================== GPIO =======================================
// EXTI
#define INDIVIDUAL_EXTI_IRQ_REQUIRED    FALSE

// DBG UART
#define UART_GPIO       GPIOB
#define UART_TX_PIN     6
#define UART_RX_PIN     7
// Ext RS485
#define RS485_EXT_TXEN  GPIOA, 1, AF7
#define RS485_EXT_TX    GPIOA, 2
#define RS485_EXT_RX    GPIOA, 3
// Ext RS232
#define RS232_TX        GPIOC, 12
#define RS232_RX        GPIOD, 2
// Int RS485
#define RS485_INT_TXEN  GPIOB, 1, AF7
#define RS485_INT_TX    GPIOC, 4
#define RS485_INT_RX    GPIOC, 5

// DBG LED
#define LED_PIN         GPIOB, 3, omPushPull

// TControl
#define HEATER_TOP      1000
#define HEATER_MAX_V    702.0 // To allow 2A max
#define HEATER_CTRL     GPIOB, 0, TIM3, 3, invNotInverted, omPushPull, (HEATER_TOP-1)

// I2C
#define I2C1_GPIO       GPIOB
#define I2C1_SCL        8
#define I2C1_SDA        9

#define SNS_PWR_CTRL    GPIOB, 5

// I2C Alternate Function
#define I2C_AF          AF4

// USB
#define USB_DM          GPIOA, 11
#define USB_DP          GPIOA, 12
#define USB_AF          AF10
// USB detect
#define USB_DETECT_PIN  GPIOA, 9

#endif // GPIO

#if 1 // =========================== I2C =======================================
// i2cclkPCLK1, i2cclkSYSCLK, i2cclkHSI
#define I2C_CLK_SRC     i2cclkHSI
#define I2C_BAUDRATE_HZ 400000
#endif

#if ADC_REQUIRED // ======================= Inner ADC ==========================
// Clock divider: clock is generated from the APB2
#define ADC_CLK_DIVIDER		adcDiv2

// ADC channels
#define ADC_BATTERY_CHNL 	14
// ADC_VREFINT_CHNL
#define ADC_CHANNELS        { ADC_BATTERY_CHNL, ADC_VREFINT_CHNL }
#define ADC_CHANNEL_CNT     2   // Do not use countof(AdcChannels) as preprocessor does not know what is countof => cannot check
#define ADC_SAMPLE_TIME     ast24d5Cycles
#define ADC_OVERSAMPLING_RATIO  64   // 1 (no oversampling), 2, 4, 8, 16, 32, 64, 128, 256
#endif

#if 1 // =========================== DMA =======================================
// ==== Uart ====
// Remap is made automatically if required
#define UART_DMA_TX     STM32_DMA_STREAM_ID(1, 4)
#define UART_DMA_RX     STM32_DMA_STREAM_ID(1, 5)
#define UART_DMA_CHNL   2
#define UART_DMA_TX_MODE(Chnl) (STM32_DMA_CR_CHSEL(Chnl) | DMA_PRIORITY_LOW | STM32_DMA_CR_MSIZE_BYTE | STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MINC | STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_TCIE)
#define UART_DMA_RX_MODE(Chnl) (STM32_DMA_CR_CHSEL(Chnl) | DMA_PRIORITY_MEDIUM | STM32_DMA_CR_MSIZE_BYTE | STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MINC | STM32_DMA_CR_DIR_P2M | STM32_DMA_CR_CIRC)

#define RS485EXT_DMA_TX STM32_DMA_STREAM_ID(1, 7)
#define RS485EXT_DMA_RX STM32_DMA_STREAM_ID(1, 6)

#define RS485INT_DMA_TX STM32_DMA_STREAM_ID(1, 2)
#define RS485INT_DMA_RX STM32_DMA_STREAM_ID(1, 3)

#define RS232_DMA_TX    STM32_DMA_STREAM_ID(2, 1)
#define RS232_DMA_RX    STM32_DMA_STREAM_ID(2, 2)

// ==== I2C ====
#define I2C1_DMA_TX     STM32_DMA_STREAM_ID(2, 7)
#define I2C1_DMA_RX     STM32_DMA_STREAM_ID(2, 6)
#define I2C1_DMA_CHNL   5
#define I2C2_DMA_TX     STM32_DMA_STREAM_ID(1, 4)
#define I2C2_DMA_RX     STM32_DMA_STREAM_ID(1, 5)
#define I2C2_DMA_CHNL   3
#define I2C3_DMA_TX     STM32_DMA_STREAM_ID(1, 2)
#define I2C3_DMA_RX     STM32_DMA_STREAM_ID(1, 3)
#define I2C3_DMA_CHNL   3

#if ADC_REQUIRED
#define ADC_DMA         STM32_DMA1_STREAM1
#define ADC_DMA_MODE    STM32_DMA_CR_CHSEL(0) |   /* DMA1 Stream1 Channel0 */ \
                        DMA_PRIORITY_LOW | \
                        STM32_DMA_CR_MSIZE_HWORD | \
                        STM32_DMA_CR_PSIZE_HWORD | \
                        STM32_DMA_CR_MINC |       /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_P2M |    /* Direction is peripheral to memory */ \
                        STM32_DMA_CR_TCIE         /* Enable Transmission Complete IRQ */
#endif // ADC

#endif // DMA

#if 1 // ========================== USART ======================================
#define PRINTF_FLOAT_EN TRUE
#define UART_TXBUF_SZ   4096
#define UART_RXBUF_SZ   1024
#define CMD_BUF_SZ      1024

#define CMD_UART        USART1

#define CMD_UART_PARAMS \
    USART1, UART_GPIO, UART_TX_PIN, UART_GPIO, UART_RX_PIN, \
    UART_DMA_TX, UART_DMA_RX, UART_DMA_TX_MODE(UART_DMA_CHNL), UART_DMA_RX_MODE(UART_DMA_CHNL), \
    uartclkHSI // Use independent clock

#define RS485EXT_PARAMS \
    USART2, RS485_EXT_TX, RS485_EXT_RX, \
    RS485EXT_DMA_TX, RS485EXT_DMA_RX, UART_DMA_TX_MODE(UART_DMA_CHNL), UART_DMA_RX_MODE(UART_DMA_CHNL), \
    uartclkHSI // Use independent clock

#define RS485INT_PARAMS \
    USART3, RS485_INT_TX, RS485_INT_RX, \
    RS485INT_DMA_TX, RS485INT_DMA_RX, UART_DMA_TX_MODE(UART_DMA_CHNL), UART_DMA_RX_MODE(UART_DMA_CHNL), \
    uartclkHSI // Use independent clock

#define RS232_PARAMS \
    UART5, RS232_TX, RS232_RX, \
    RS232_DMA_TX, RS232_DMA_RX, UART_DMA_TX_MODE(UART_DMA_CHNL), UART_DMA_RX_MODE(UART_DMA_CHNL), \
    uartclkHSI // Use independent clock

#endif
