

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
#include <LEDService.h>
#include <Logger.h>

void StartCommunicationThreads()
{
    // Start UART
    osThreadAttr_t task_attributes;
    memset(&task_attributes, 0, sizeof(osThreadAttr_t));
    task_attributes.name = "UART_TASK";
    task_attributes.priority = (osPriority_t) osPriorityNormal;
    task_attributes.stack_size = 512;
    osThreadNew(init_uart_threads, (void *)USART2, &task_attributes);

    // Start CAN
}

void StartLEDService()
{
    // Start LED Service
    osThreadAttr_t task_attributes;
    memset(&task_attributes, 0, sizeof(osThreadAttr_t));
    task_attributes.name = "LED_TASK";
    task_attributes.priority = (osPriority_t) osPriorityNormal;
    task_attributes.stack_size = 2048;

    osThreadNew(status_led_thread, NULL, &task_attributes);
}

void DebugTask()
{
//     for(;;)
//     {
//         LEDService::Instance().Blink(100,900);
//         osDelay(100);
//     }

// Spi TEST

// SPI1 = Encoder
GPIO_t mosi = {ENC_MOSI_GPIO_Port, ENC_MOSI_Pin};
GPIO_t miso = {ENC_MISO_GPIO_Port, ENC_MISO_Pin};
GPIO_t nss = {ENC_CS_GPIO_Port, ENC_CS_Pin};

SPIDevice encoder_dev(SPI1, mosi, miso, nss);
encoder_dev.Enable();
uint16_t position = 0;

for (;;)
{
    encoder_dev.Select();
    position = encoder_dev.Receive16();
    position &= 0x3FFF; // Data in last 14 bits.
    encoder_dev.Deselect();

    Logger::Instance().Print("Pos: %d\r\n", position);
    osDelay(100);
}
}

extern "C" int app_main()
{

    Logger::Instance().Enable(true);
    // Start Communications Threads (UART/CAN)
    StartCommunicationThreads();

    // Delay
    osDelay(1000);
    
    // Init LED Service Task
    StartLEDService();

    // Delay
    osDelay(2500);
    
    DebugTask();
    // Init Misc Polling Task

    // Init Motor Control Task

    // Init a temp debug Task
    
    // Infinite Loop.
    for (;;)
    {
    }

    // Should not get here
    return 0;
}