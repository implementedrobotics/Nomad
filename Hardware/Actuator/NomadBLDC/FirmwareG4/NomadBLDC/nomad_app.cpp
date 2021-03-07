

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



// TODO: Where to put these globals?
UARTDevice *uart;
FDCANDevice *fdcan;

DeviceStatusRegister1_t DSR1; // Device Status Register 1
DeviceStatusRegister2_t DSR2; // Device Status Register 2

void StartCommunicationThreads()
{
    // Setup some pins
    GPIO_t rx = {USART_RX_GPIO_Port, USART_RX_Pin};
    GPIO_t tx = {USART_TX_GPIO_Port, USART_TX_Pin};

    uart = new UARTDevice(USART2, rx, tx);
    uart->Init();
    uart->SetMode(UARTDevice::HDLC_MODE);
    uart->RegisterHDLCCommandCB(&CommandHandler::ProcessPacket);
    
    // TODO: Need to make this a proper class
    CommandHandler::SetUART(uart);

    // Start Logger Service
    Logger::Instance().Enable(true);
    Logger::Instance().SetUART(uart);

    // Start CAN (1mbps Nominal Rate w/ 2mbps Data Rate)
    fdcan = new FDCANDevice(FDCAN3, 0x123, 1e6, 2e6);
    fdcan->Init();
    fdcan->Enable();
    fdcan->EnableIT();
    fdcan->Attach(&RegisterInterface::HandleCommand);
    //Logger::Instance().Print("Device ID: %d\r\n", LL_DBGMCU_GetDeviceID());
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

void SetupDeviceRegisters()
{

    // Update Versioning
    DSR2.fw_major = VERSION_MAJOR;
    DSR2.fw_minor = VERSION_MINOR;

    //Get UID
    uint32_t *uid = (uint32_t *)UID_BASE;
    DSR2.uid1 = uid[0];
    DSR2.uid2 = uid[1];
    DSR2.uid3 = uid[2];

    // Add Register Addresses

    // Device Status Register 1
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceStatusRegister1, new Register(&DSR1, true));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceFault, new Register(&DSR1.fault_mode));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceControlMode, new Register(&DSR1.control_mode));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceVoltageBus, new Register(&DSR1.V_bus));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceCurrentBus, new Register(&DSR1.I_bus));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceFETTemp, new Register(&DSR1.fet_temp));

    // Device Status Register 2
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceStatusRegister2, new Register(&DSR2, true));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceFirmwareMajor, new Register(&DSR2.fw_major));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceFirmwareMinor, new Register(&DSR2.fw_minor));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceUID1, new Register(&DSR2.uid1));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceUID2, new Register(&DSR2.uid2));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceUID3, new Register(&DSR2.uid3));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceUptime, new Register(&DSR2.uptime));

    RegisterInterface::register_command_t test;
    test.header.rwx = 0;
    test.header.address = DeviceRegisters_e::DeviceUID1;
    test.header.data_type = 1;
    //test.header.reserved = 1;
    uint32_t new_val = 64001;


    memcpy(&test.cmd_data, (uint32_t *)&new_val, sizeof(uint32_t));

   // memcpy(&test.cmd_data, (uint8_t *)&test_me, sizeof(Test_Struct));

   FDCANDevice::FDCAN_msg_t msg;
   memcpy(msg.data, &test, 64);

   Logger::Instance().Print("Message New: %x\r\n", test);

   Register *reg = RegisterInterface::GetRegister(DeviceRegisters_e::DeviceUID1);
   Logger::Instance().Print("From Reg: %d\r\n", reg->Get<uint32_t>());

   RegisterInterface::HandleCommand(msg, fdcan);
   Logger::Instance().Print("Got New: %d\r\n", reg->Get<uint32_t>());
}

void DebugTask()
{
}


extern "C" int app_main() //
{
    // Start Communications Threads (UART/CAN)
    StartCommunicationThreads();
    osDelay(100);

    // Init LED Service Task
    StartLEDService();
    osDelay(50);

    // Setup Device Registers
    SetupDeviceRegisters();
    osDelay(5);

    // Start Motor Control Task
    StartMotorControlThread();
    osDelay(100);

    // Start Misc Polling Task
    StartPollingThread();
    osDelay(5);

    // osDelay(3000);

    // Init a temp debug Task
    //DebugTask();

    // float theta = PI;
    // int i = 0;

    // uint32_t start_ticks;
    // uint32_t stop_ticks;
    // uint32_t elapsed_ticks;

    uint8_t Tx_Data[15] = {0x5, 0x10, 0x11, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};

    int i =0;
    // Infinite Loop.
    for (;;)
    {
      //  fdcan->Send(0x001, Tx_Data, 15);

        // Update Device Stats
        //DSR2.uptime = HAL_GetTick() / 1000;
        //osDelay(1000);
        //uint16_t length;
        // fdcan->Receive(Rx_Data, length);
       // Logger::Instance().Print("Here: %d\r\n", i++);
        //osDelay(500);
        // start_ticks = SysTick->VAL;
        // temp = fet_temp->SampleTemperature();

        // LL_GPIO_SetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);

       //  Logger::Instance().Print("Test 0x%lX.\r\n", LL_DBGMCU_GetDeviceID());

        // LL_GPIO_ResetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);
        // stop_ticks = SysTick->VAL;
        // elapsed_ticks = start_ticks-stop_ticks;
        //  uint32_t span = DWT->CYCCNT;
    }

    // Should not get here
    return 0;
}