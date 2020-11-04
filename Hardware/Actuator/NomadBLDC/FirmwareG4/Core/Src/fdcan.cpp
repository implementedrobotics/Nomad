
/*
 * fdcan.cpp
 *
 *  Created on: November 3, 2020
 *      Author: Quincy Jones
 *
 * Copyright (c) <2020> <Quincy Jones - quincy@implementedrobotics.com/>
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions
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
#include <Peripherals/fdcan.h>

// C System Files

// C++ System Files

// Project Includes

static FDCANDevice* g_ISR_VTABLE[FDCANDevice::kMaxInterrupts];

FDCANDevice::FDCANDevice(FDCAN_GlobalTypeDef *FDCAN) : FDCAN_(FDCAN), enable_interrupt_(false)
{
}

// Enable FDCAN
void FDCANDevice::Enable()
{
  hfdcan_.Instance = FDCAN_;
  hfdcan_.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan_.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan_.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan_.Init.AutoRetransmission = DISABLE;
  hfdcan_.Init.TransmitPause = DISABLE;
  hfdcan_.Init.ProtocolException = DISABLE;
  hfdcan_.Init.NominalPrescaler = 1; // Between 1 and 512
  hfdcan_.Init.NominalTimeSeg1 = 135; // Between 2 and 256
  hfdcan_.Init.NominalTimeSeg2 = 34; // Between 2 and 128
  hfdcan_.Init.NominalSyncJumpWidth = 34; // Between 1 and 128
  hfdcan_.Init.DataPrescaler = 17; // Between 1 and 32
  hfdcan_.Init.DataTimeSeg1 = 2; // Between 1 and 32
  hfdcan_.Init.DataTimeSeg2 = 2; // Between 1 and 16
  hfdcan_.Init.DataSyncJumpWidth = 2; // Between 1 and 16
  hfdcan_.Init.StdFiltersNbr = 1;
  hfdcan_.Init.ExtFiltersNbr = 0;
  hfdcan_.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

  if (HAL_FDCAN_Init(&hfdcan_) != HAL_OK)
  {
    Error_Handler();
  }
}

void FDCANDevice::EnableIT()
{
    // Find IRQ Number
    if(FDCAN_ == FDCAN1)
    {
        IRQn_ = FDCAN1_IT0_IRQn;
    }
    else if(FDCAN_ == FDCAN2)
    {
        IRQn_ = FDCAN2_IT0_IRQn;
    }
    else if(FDCAN_ == FDCAN3)
    {
        IRQn_ = FDCAN3_IT0_IRQn;
    }
    else // Invalid
    {
        return;
    }

    // Make sure IRQ is enabled
    // TODO: Priority...
    NVIC_SetPriority(IRQn_, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
    NVIC_EnableIRQ(IRQn_);

    // Update ISR Table
    g_ISR_VTABLE[IRQn_] = this;

    enable_interrupt_ = true;

    // Not Sure Why Dynamic Does not work
    // __disable_irq();
    // NVIC_SetVector(IRQn, (uint32_t)&IRQ);
    // __enable_irq();
}

// Interrupts

