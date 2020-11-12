

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
#include <RegisterInterface.h>
#include <variant>
UARTDevice *uart;
FDCANDevice *fdcan;


    // Motor State
    struct Test_Struct
    {
        uint32_t a;
        uint32_t b;
        uint32_t c;
    };


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
    fdcan->Attach(&RegisterInterface::HandleCommand);

    Test_Struct test_me;
    test_me.a = 12345;
    test_me.b = 4321;
    test_me.c = 1111;

    Test_Struct test_2;
    test_2.a = 4444;
    test_2.b = 3333;
    test_2.c = 2222;

    RegisterInterface::register_command_t test;
    test.rwx = 2;
    test.address = 0;
    test.data_type = 1;

    memcpy(&test.cmd_data, (uint8_t *)&test_me, sizeof(Test_Struct));


    FDCANDevice::FDCAN_msg_t msg;
    memcpy(msg.data, test.data, 64);

    uint16_t pole_count = 12;
    uint16_t encoder_count = 1600;

    Register motor_config_register;
    motor_config_register.AddDataField(&pole_count);
    motor_config_register.AddDataField(&encoder_count);

    Register full_reg;
    full_reg.AddDataField((uint8_t *)&test_me);
    full_reg.Set(0, (uint8_t*)&test_2, 12);

    // uint32_t newBuf = 1245;
    // data.Set((uint16_t)15);
    // data.Set((uint8_t *)&newBuf);

    Logger::Instance().Print("From Reg: %d\r\n", motor_config_register.Get<uint16_t>(0));

    Logger::Instance().Print("Got: %d\r\n", pole_count);

    RegisterInterface reg_interface;
    reg_interface.AddRegister(0x0, &motor_config_register);

    //test = 10;
    //reg.AddRegister(0x0, &test);
    RegisterInterface::HandleCommand(msg);

    Logger::Instance().Print("From Reg: %d\r\n", motor_config_register.Get<uint16_t>(0));

    // Logger::Instance().Print("A: %d and %d\r\n", a, k);
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

   // int i =0;
    // Infinite Loop.
    for (;;)
    {
            fdcan->Send(0x001, Tx_Data, 10);
            osDelay(100);
            //uint16_t length;
           // fdcan->Receive(Rx_Data, length);
           // Logger::Instance().Print("Here: %d\r\n", i++);
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