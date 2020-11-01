

/*
 * Logger.cpp
 *
 *  Created on: March 27, 2020
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
#include <nomad_app.h>

// C System Files
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stddef.h>

// C++ System Files

// Project Includes
#include "main.h"
#include <Peripherals/uart.h>
#include <Peripherals/spi.h>
#include <Peripherals/gpio.h>
#include <Peripherals/flash.h>
#include <Peripherals/thermistor.h>
#include <Peripherals/cordic.h>
#include <Peripherals/adc.h>
#include <nomad_hw.h>

#include <DRV8323.h>
#include <LEDService.h>
#include <Logger.h>
#include <motor_controller_interface.h>
#include <MotorController.h>


UARTDevice *uart;
Cordic cordic2;
void StartCommunicationThreads()
{
    // Setup some pins
    GPIO_t rx = {USART_RX_GPIO_Port, USART_RX_Pin};
    GPIO_t tx = {USART_TX_GPIO_Port, USART_TX_Pin};

    uart = new UARTDevice(USART2, rx, tx);
    uart->Init();
    uart->SetMode(UARTDevice::ASCII_MODE);
    uart->RegisterHDLCCommandCB(&CommandHandler::ProcessPacket);
    
    // TODO: Need to make this a proper class
    CommandHandler::SetUART(uart);
    // Start CAN
}

void StartLEDService()
{
    // Start LED Service
    osThreadAttr_t task_attributes;
    memset(&task_attributes, 0, sizeof(osThreadAttr_t));
    task_attributes.name = "LED_TASK";
    task_attributes.priority = (osPriority_t)osPriorityNormal;
    task_attributes.stack_size = 2048;

    osThreadNew(status_led_thread, NULL, &task_attributes);
}

void StartMotorControlThread()
{
    // Start Motor Control Thread
    init_motor_controller();
}

void StartPollingThread()
{
    // Start Motor Control Thread
    osThreadAttr_t task_attributes;
    memset(&task_attributes, 0, sizeof(osThreadAttr_t));
    task_attributes.name = "POLL_TASK";
    task_attributes.priority = (osPriority_t)osPriorityNormal;
    task_attributes.stack_size = 2048;

    osThreadNew(ms_poll_task, NULL, &task_attributes);
}

void DebugTask()
{
}


extern "C" int app_main() //
{
    // Start Communications Threads (UART/CAN)
    StartCommunicationThreads();
    osDelay(100);

     // Start Logger Service
    Logger::Instance().Enable(true);
    Logger::Instance().SetUART(uart);

    Logger::Instance().Print("Starting!\r\n");

  Logger::Instance().Print("Clock: %d\r\n", HAL_RCC_GetPCLK1Freq());
    FDCAN_FilterTypeDef sFilterConfig;
    FDCAN_TxHeaderTypeDef TxHeader;
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t TxData0[] = {0x25, 0x00, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF};
    uint8_t RxData[8];

    /* Configure standard ID reception filter to Rx FIFO 0 */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x100;
    sFilterConfig.FilterID2 = 0x200;
    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig) != HAL_OK)
    {
      Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
      Error_Handler();
    }

    if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan3, 408, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }


    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
    {
          Logger::Instance().Print("CAN START ERROR!\r\n");
      Error_Handler();
    }

    Logger::Instance().Print("CAN START!\r\n");




    // // Init LED Service Task
    // StartLEDService();
    // osDelay(50);

    // // Start Motor Control Task
    // StartMotorControlThread();
    // osDelay(100);

    // // Start Misc Polling Task
    // StartPollingThread();
    // osDelay(5);

    // osDelay(3000);

    // Init a temp debug Task
    //DebugTask();

    //  float theta = PI;
    // int i = 0;

    // uint32_t start_ticks;
    // uint32_t stop_ticks;
    // uint32_t elapsed_ticks;

    // Infinite Loop.
    for (;;)
    {
        // start_ticks = SysTick->VAL;
        // temp = fet_temp->SampleTemperature();

        // LL_GPIO_SetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);

        // Logger::Instance().Print("Test 0x%lX.\r\n", LL_DBGMCU_GetDeviceID());

        // LL_GPIO_ResetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);
        // stop_ticks = SysTick->VAL;
        // elapsed_ticks = start_ticks-stop_ticks;
        //  uint32_t span = DWT->CYCCNT;

    /* Add message to Tx FIFO */
    TxHeader.Identifier = 0x144;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_1;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData0) != HAL_OK)
    {
      Error_Handler();
    }

   //Logger::Instance().Print("ADDED!\r\n");

  // osDelay(1000);

    /* Wait transmissions complete */
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3) != 3) {
       // Logger::Instance().Print("WAITING!\r\n");
    }

  osDelay(1);
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData0) != HAL_OK)
    {
      Error_Handler();
    }

    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3) != 3) {
       // Logger::Instance().Print("WAITING!\r\n");
    }
  osDelay(10000);
  //  Logger::Instance().Print("OUT!\r\n");



    // /* Check one message is received in Rx FIFO 0 */
    //   if(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan3, FDCAN_RX_FIFO0) != 1)
    //   {
    // 	//Logger::Instance().Print("BAILED FILL\r\n");
    //     Error_Handler();
    //   }

    //   /* Retrieve message from Rx FIFO 0 */
    //   if (HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    //   {
    // 	 //Logger::Instance().Print("BAILED RX\r\n");
    //     Error_Handler();
    //   }
  	// Logger::Instance().Print("Got: %s\r\n", RxData);

    // RxData[0] = '1';
    // RxData[1] = '2';
    //     osDelay(100);
    }



    // Should not get here
    return 0;
}