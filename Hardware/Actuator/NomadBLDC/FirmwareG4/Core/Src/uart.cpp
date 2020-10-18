/*
 * uart.cpp
 *
 *  Created on: September 15, 2020
 *      Author: Quincy Jones
 *
 * Copyright (c) <2020> <Quincy Jones - quincy@implementedrobotics.com/>
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 */

// Primary Include
#include <Peripherals/uart.h>
#include <Utilities/crc16.h>

// C++ Includes
#include <functional>

// C System Files
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Project Includes
#include <main.h> // STM32 Driver Includes

// RTOS Variables
osThreadId_t rx_thread_id = 0;

/* C Functions for RTOS */
void start_uart_rx_thread(void *arg)
{
    UARTDevice *uart = static_cast<UARTDevice *>(arg);

    // uart_send_str("\r\n\r\nNomad Firmware v2.0 STM32G4 Beta\r\n");

    for (;;)
    {
        // Wait for Receive Signal Event On Interrupt
        osThreadFlagsWait(UARTDevice::UART_RX_DATA, osFlagsWaitAll, osWaitForever);

        // LL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        // Read our receive buffer

        uart->ReceiveHandler();
    }
}

// TODO: These should be generic...
/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
extern "C" void DMA1_Channel1_IRQHandler(void)
{
  // DMA Half Complete Callback
  if (LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_CHANNEL_1) && LL_DMA_IsActiveFlag_HT1(DMA1))
  {
    LL_DMA_ClearFlag_HT1(DMA1);                        // Clear Flag
    osThreadFlagsSet(rx_thread_id, UARTDevice::UART_RX_DATA);
  }

  // DMA Full Complete Callback
  if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_1) && LL_DMA_IsActiveFlag_TC1(DMA1))
  {
    LL_DMA_ClearFlag_TC1(DMA1);                        // Clear Flag
    osThreadFlagsSet(rx_thread_id, UARTDevice::UART_RX_DATA);
  }
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
extern "C" void USART2_IRQHandler(void)
{
  // Check for IDLE Interrupt
  if (LL_USART_IsEnabledIT_IDLE(USART2) && LL_USART_IsActiveFlag_IDLE(USART2))
  {
    LL_USART_ClearFlag_IDLE(USART2);                   // Clear Flag
    osThreadFlagsSet(rx_thread_id, UARTDevice::UART_RX_DATA);
  }
}

UARTDevice::UARTDevice(USART_TypeDef *UART, GPIO_t rx_pin, GPIO_t tx_pin) : UART_(UART), rx_pin_(rx_pin), tx_pin_(tx_pin), old_rx_buffer_pos_(0), hdlc_(0)
{
    rx_dma_ = true;
    tx_dma_ = false;

    // Update Baud Rate
    SetBaud(115200);

    // Update Mode
    SetMode(ASCII_MODE);

}

void UARTDevice::SetBaud(uint32_t baud)
{
    baud_ = baud;
}

void UARTDevice::SetMode(UART_DATA_MODE mode)
{
    using namespace std::placeholders;
    switch (mode)
    {
    case UART_DATA_MODE::HDLC_MODE:
        if(hdlc_ == nullptr)
            hdlc_ = new HDLCFrame();

        RegisterRXCallback(std::bind(&HDLCFrame::Decode, hdlc_, _1, _2));
        break;

    case UART_DATA_MODE::ASCII_MODE:
        RegisterRXCallback(std::bind(&UARTDevice::ReceiveEcho, this, _1, _2));
        break;

    case UART_DATA_MODE::BINARY_MODE:
        RegisterRXCallback(std::bind(&UARTDevice::ReceiveEcho, this, _1, _2));
        break;

    default:
        RegisterRXCallback(std::bind(&UARTDevice::ReceiveEcho, this, _1, _2));
    }

    // Set Mode
    mode_ = mode;
}

void UARTDevice::ReceiveEcho(const uint8_t *data, size_t length)
{
    // Echo Back
    Send(data, length);
}

void UARTDevice::RegisterRXCallback(const std::function<void(const uint8_t*, size_t)>& callback)
{
    callback_rx_ = callback;
}

void UARTDevice::RegisterHDLCCommandCB(const std::function<void(const uint8_t*, size_t)>& callback)
{
    if(hdlc_ != nullptr)
        hdlc_->RegisterFrameHandler(callback);
}

            
bool UARTDevice::Init()
{
    // Init Device
    USART_Init();
    osDelay(100);

    // Setup RTOS Threads

    // RX Thread Attributes
    osThreadAttr_t task_attributes;
    memset(&task_attributes, 0, sizeof(osThreadAttr_t));
    task_attributes.name = "UART_RX_TASK";
    task_attributes.priority = (osPriority_t)osPriorityNormal;
    task_attributes.stack_size = kStackSizeRX;

    // Start RX Thread
    rx_thread_id = osThreadNew(start_uart_rx_thread, (void *)this, &task_attributes);

    return true;
}

void UARTDevice::ReceiveHandler()
{

    // TODO: This is for DMA only.  Could be polling or interrupt based later
    // TODO: Make us a "buffer object" to handle this
    // Compute where we are in the DMA buffer
    size_t pos;
    pos = kBufferSizeRX - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_1);
    if (pos != old_rx_buffer_pos_) // We have new data in buffer
    {
        if (pos > old_rx_buffer_pos_) // Standard buffer read no wrapping.
        {
            // We are in "linear" mode
            // Process data directly by subtracting "pointers"
            callback_rx_(&uart_rx_buffer_[old_rx_buffer_pos_], pos - old_rx_buffer_pos_);
        }
        else // We are have wrapped the buffer
        {
            // Process "linear" to end of buffer
            callback_rx_(&uart_rx_buffer_[old_rx_buffer_pos_], kBufferSizeRX - old_rx_buffer_pos_);
            // Read rest of bytes from the beginning of the buffer to get the wrapped bytes
            if (pos > 0)
            {
                callback_rx_(&uart_rx_buffer_[0], pos);
            }
        }
    }
    old_rx_buffer_pos_ = pos; // Update current/old position pointer

    // Check if we moved to end of buffer and need to wrap back to beginning
    if (old_rx_buffer_pos_ == kBufferSizeRX)
    {
        old_rx_buffer_pos_ = 0;
    }
}

uint32_t UARTDevice::Send(const uint8_t *data, size_t length)
{
    // TODO: DMA Send Mode
    uint32_t sent_bytes = 0;
    for (; length > 0; --length, ++data)
    {
        LL_USART_TransmitData8(UART_, *data);
        while (!LL_USART_IsActiveFlag_TXE(UART_))
        {
        }
        sent_bytes++;
    }
    while (!LL_USART_IsActiveFlag_TC(UART_))
    {
    }
    return sent_bytes;
}
uint32_t UARTDevice::SendBytes(const uint8_t *data, size_t length)
{
    if(mode_ == HDLC_MODE) // Send HDLC Encoded
    {
        static uint8_t frame_data[hdlc_->kPacketSizeLimit];
        uint16_t frame_length = hdlc_->Encode(data, length, frame_data);
        return Send(frame_data, frame_length);
    }
    else // Send Normal
    {
        return Send(data, length);
    }
}

uint32_t UARTDevice::SendString(const char *string)
{
    return Send((uint8_t *)string, strlen(string));
}

void UARTDevice::USART_Init()
{
    // Init DMA
    /* Init with LL driver */
    /* DMA controller clock enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    LL_USART_InitTypeDef USART_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    // TODO: Make all this more generic.  Need some pinmapping/case checking for the device and which clocks, channels etc to enable
    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

    GPIO_InitStruct.Pin = tx_pin_.pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(tx_pin_.port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = rx_pin_.pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(rx_pin_.port, &GPIO_InitStruct);

    /* USART2 DMA Init */

    /* USART2_RX Init */
    // TODO: This is if doing DMA.  Should Move this out, InitDMA
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_USART2_RX);

    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_HIGH);

    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);

    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);

    /* USART2 interrupt Init */
    NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(USART2_IRQn);

    /* Configure the DMA functional parameters for reception */
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1,
                           LL_USART_DMA_GetRegAddr(UART_, LL_USART_DMA_REG_DATA_RECEIVE),
                           (uint32_t)uart_rx_buffer_,
                           LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1));
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, kBufferSizeRX);

    /* Enable DMA transfer complete/error interrupts  */
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);

    /* Enable USART idle line interrupts */
    LL_USART_EnableIT_IDLE(UART_);
    
    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate = baud_;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(UART_, &USART_InitStruct);
    LL_USART_SetTXFIFOThreshold(UART_, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_SetRXFIFOThreshold(UART_, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_DisableFIFO(UART_);
    LL_USART_ConfigAsyncMode(UART_);

    // TODO: Move this to EnableUART()
    LL_USART_Enable(UART_);

    /* Polling USART2 initialisation */
    while ((!(LL_USART_IsActiveFlag_TEACK(UART_))) || (!(LL_USART_IsActiveFlag_REACK(UART_))))
    {
    }

    // TODO: Depends if using DMA, Polling or Interrupt
    /* Enable DMA RX Interrupt */
    LL_USART_EnableDMAReq_RX(UART_);

    /* Enable DMA Channel Rx */
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}


