

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
#include <NomadFlash.h>
#include <CommandHandler.h>
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

    // Load CAN Config
    FDCANDevice::Config_t config;

    // Open it.
    if(!NomadFlash::Open())
    {
        Logger::Instance().Print("Unable to load CAN Configuration!\r\n");
        // Load some defaults

        //Start CAN (1mbps Nominal Rate w/ 2mbps Data Rate)
        config.bitrate = 1e6;
        config.d_bitrate = 2e6;
        config.id = 0x123;
        config.mode_fd = 1;
        config.sample_point = 0.80f;    // 80%
        config.d_sample_point = 0.625f; // 62.5%
    }
    else
    {
        NomadFlash::LoadCANConfig(config);
        //Logger::Instance().Print("Loaded CAN: %d\r\n", config.id);
    }

    // Close it.
    NomadFlash::Close();

    // TODO: Make Instance...
    fdcan = new FDCANDevice(FDCAN3, config);
    fdcan->Init();
    fdcan->Enable();
    fdcan->EnableIT();
    fdcan->Attach(&RegisterInterface::HandleCommand);

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
    // Load from Flash
    // Open it.
    if(NomadFlash::Open())
    {
        // Load All
        Save_format_t load_data;
        NomadFlash::LoadAll(load_data);
        
        // Start Motor Control Thread
        init_motor_controller(&load_data);
    }
    else
    {
        Logger::Instance().Print("No Valid Load Data Found.  Please configure and save!\r\n");
        // Start Motor Control Thread
        init_motor_controller(NULL);
    }
    
    NomadFlash::Close();
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
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceStatusRegister1, new Register(&DSR1, true, sizeof(DeviceStatusRegister1_t)));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceFault, new Register(&DSR1.fault_mode));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceControlMode, new Register(&DSR1.control_mode));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceVoltageBus, new Register(&DSR1.V_bus));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceCurrentBus, new Register(&DSR1.I_bus));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceFETTemp, new Register(&DSR1.fet_temp));

    // Device Status Register 2
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceStatusRegister2, new Register(&DSR2, true, sizeof(DeviceStatusRegister2_t)));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceFirmwareMajor, new Register(&DSR2.fw_major));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceFirmwareMinor, new Register(&DSR2.fw_minor));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceUID1, new Register(&DSR2.uid1));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceUID2, new Register(&DSR2.uid2));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceUID3, new Register(&DSR2.uid3));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceUptime, new Register(&DSR2.uptime));

    // System Management Registers
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceSaveConfig, new Register(std::bind(&save_configuration)));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceLoadConfig, new Register(std::bind(&load_configuration)));
    RegisterInterface::AddRegister(DeviceRegisters_e::DeviceRestart, new Register(std::bind(&reboot_system)));

    // Register  *test_reg = RegisterInterface::GetRegister(DeviceRegisters_e::DeviceStatusRegister1);
    // DeviceStatusRegister1_t *d_reg1 = (DeviceStatusRegister1_t *)RegisterInterface::GetRegister(DeviceRegisters_e::DeviceStatusRegister1)->GetDataPtr<uint8_t *>();

    // Register *reg = RegisterInterface::GetRegister(DeviceRegisters_e::DeviceUID1);
    // Logger::Instance().Print("From Reg: %d\r\n", reg->Get<uint32_t>());
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

    //uint8_t Tx_Data[15] = {0x5, 0x10, 0x11, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};

    //save_configuration();
    //int i = 0;
    // Infinite Loop.
    for (;;)
    {
      //  fdcan->Send(0x001, Tx_Data, 15);

        // Update Device Stats
        //DSR2.uptime = HAL_GetTick() / 1000;
        //osDelay(1000);
        //uint16_t length;
        // fdcan->Receive(Rx_Data, length);
        //Logger::Instance().Print("Here: %d\r\n", i++);
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


void ms_poll_task(void *arg)
{
    // Main millisecond polling loop
    for (;;)
    {
        // Sample bus voltage
        motor_controller->SampleBusVoltage();

        // Sample FET Thermistor for Temperature
        motor_controller->SampleFETTemperature();

        // Delay 1 ms
        osDelay(1);
    }
}

FDCANDevice *get_can_device()
{
    return fdcan;
}

int8_t save_configuration()
{
    Logger::Instance().Print("\r\nSaving Configuration...\r\n");
    if (NomadFlash::Open(true))
    {
        // If we are writing a config assume for now we are calibrated
        // TODO: Do something better so we don't have to make this assumption
        motor->config_.calibrated = 1;
        //status = NomadFlash::SaveMotorConfig(motor->config_);
        if (!NomadFlash::SaveMotorConfig(motor->config_))
        {
            NomadFlash::Close();
            return 0;
        }
        Logger::Instance().Print("Saved Motor Config.\r\n");
       // status = NomadFlash::SavePositionSensorConfig(motor->PositionSensor()->config_);
        if (!NomadFlash::SavePositionSensorConfig(motor->PositionSensor()->config_))
        {
            NomadFlash::Close();
            return 0;
        }
        Logger::Instance().Print("Saved Position Config.\r\n");
      //  status = NomadFlash::SaveControllerConfig(motor_controller->config_);
        if (!NomadFlash::SaveControllerConfig(motor_controller->config_))
        {
            NomadFlash::Close();
            return 0;
        }
        Logger::Instance().Print("Saved Controller Config.\r\n");
      //  status = NomadFlash::SaveCANConfig(fdcan->ReadConfig());
        if (!NomadFlash::SaveCANConfig(fdcan->ReadConfig()))
        {
            NomadFlash::Close();
            return 0;
        }
        Logger::Instance().Print("Saved CAN Config.\r\n");
    }
    else
    {
        Logger::Instance().Print("Unable to Open Flash!\r\n");
        return 0;
    }

    NomadFlash::Close();
    return 1;
}

int8_t load_configuration()
{
    // // Open it.
    // if(!NomadFlash::Open())
    // {
    //     Logger::Instance().Print("No Valid Load Data Found.  Please configure and save!\r\n");
    //     NomadFlash::Close();
    //     return false;
    // }

    // // Load All
    // Save_format_t load_data;
    // NomadFlash::LoadAll(load_data);

    // // Load Motor Config
    // NomadFlash::LoadMotorConfig(motor->config_);
    // motor->PrintConfig();

    // // Load Position Sensor Config
    // NomadFlash::LoadPositionSensorConfig(motor->PositionSensor()->config_);
    // motor->PositionSensor()->SetPolePairs(motor->config_.num_pole_pairs);
    // motor->PositionSensor()->PrintConfig();

    // // Load Controller Config
    // NomadFlash::LoadControllerConfig(motor_controller->config_);
    // motor_controller->PrintConfig();

    // // Close it.
    // NomadFlash::Close();

    return true;
}


int8_t reboot_system()
{
    NVIC_SystemReset();
    return 0;
}