

/*
 * nomad_app.cpp
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
#include <Peripherals/fdcan.h>
#include <nomad_hw.h>

#include <DRV8323.h>
#include <LEDService.h>
#include <Logger.h>
#include <motor_controller_interface.h>
#include <MotorController.h>


UARTDevice *uart;
FDCANDevice *fdcan;

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

    // Start Logger Service
    Logger::Instance().Enable(true);
    Logger::Instance().SetUART(uart);

    // Start CAN (1mbps Nominal Rate w/ 5mbps Data Rate)
    fdcan = new FDCANDevice(FDCAN3, 0x123, 1e6, 5e6);
    fdcan->Init();
    fdcan->Enable();
    fdcan->EnableIT();
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

    uint8_t Tx_Data[10] = {0x5,0x10,0x11,0x12,0x12,0x12,0x12,0x12,0x12,0x12};
    uint8_t Rx_Data[64];
    int i =0;
    // Infinite Loop.
    for (;;)
    {
            fdcan->Send(0x001, Tx_Data, 10);
            osDelay(100);
            //uint16_t length;
           // fdcan->Receive(Rx_Data, length);
            Logger::Instance().Print("Here: %d\r\n", i++);
            //osDelay(50);
        // start_ticks = SysTick->VAL;
        // temp = fet_temp->SampleTemperature();

        // LL_GPIO_SetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);

        // Logger::Instance().Print("Test 0x%lX.\r\n", LL_DBGMCU_GetDeviceID());

        // LL_GPIO_ResetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);
        // stop_ticks = SysTick->VAL;
        // elapsed_ticks = start_ticks-stop_ticks;
        //  uint32_t span = DWT->CYCCNT;
    }



    // Should not get here
    return 0;
}