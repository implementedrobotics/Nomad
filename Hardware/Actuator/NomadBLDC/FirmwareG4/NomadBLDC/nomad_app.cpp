

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
#include <nomad_hw.h>

#include <DRV8323.h>
#include <LEDService.h>
#include <Logger.h>
#include <motor_controller_interface.h>
#include <MotorController.h>


UARTDevice *uart;

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
    osThreadAttr_t task_attributes;
    memset(&task_attributes, 0, sizeof(osThreadAttr_t));
    task_attributes.name = "MOTOR_CONTROL_TASK";
    task_attributes.priority = (osPriority_t)osPriorityRealtime1;
    task_attributes.stack_size = 2048;

    osThreadNew(motor_controller_thread_entry, NULL, &task_attributes);
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



/**
  * @brief This function handles ADC3 global interrupt.
  */
extern "C" void ADC3_IRQHandler(void)
{
    //LL_GPIO_SetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);
    /* USER CODE BEGIN ADC3_IRQn 0 */
    // TODO: Save this count as a metric to verify no aliasing
    //uint32_t count = DWT->CYCCNT;
    if (LL_ADC_IsActiveFlag_EOC(ADC1))
    {
        //LL_GPIO_TogglePin(DEBUG_PIN_GPIO_Port, DEBUG_PIN_Pin);

        //LL_GPIO_ResetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_GPIO_Port);
        /* Clear flag ADC group regular end of unitary conversion */
        LL_ADC_ClearFlag_EOC(ADC1);

        // LL_GPIO_SetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);
        // for (int i = 0; i < 100; ++i)
        // {
        //     __NOP();
        // }
        // LL_GPIO_ResetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);

        //value_adc1 = LL_ADC_REG_ReadConversionData12(ADC1);
       // count_adc1 = count;
    }

    if (LL_ADC_IsActiveFlag_EOC(ADC2))
    {
        //LL_GPIO_TogglePin(DEBUG_PIN_GPIO_Port, DEBUG_PIN_Pin);

        //LL_GPIO_ResetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_GPIO_Port);
        /* Clear flag ADC group regular end of unitary conversion */
        LL_ADC_ClearFlag_EOC(ADC2);

        // LL_GPIO_SetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);
        // for (int i = 0; i < 100; ++i)
        // {
        //   __NOP();
        // }
        // LL_GPIO_ResetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);

        //value_adc2 = LL_ADC_REG_ReadConversionData12(ADC2);
        //count_adc2 = count;
    }

    if (LL_ADC_IsActiveFlag_EOC(ADC3))
    {
        //LL_GPIO_TogglePin(DEBUG_PIN_GPIO_Port, DEBUG_PIN_Pin);

        //LL_GPIO_ResetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_GPIO_Port);
        /* Clear flag ADC group regular end of unitary conversion */
        LL_ADC_ClearFlag_EOC(ADC3);
        // Send it
        //osThreadFlagsSet(CURRENT_MEASUREMENT_COMPLETE_SIGNAL);
       
        
        // for (int i = 0; i < 10; ++i)
        // {
        //   __NOP();
        // }
        

         current_measurement_cb();
         
        //value_adc3 = LL_ADC_REG_ReadConversionData12(ADC3);
       // count_adc3 = count;
        //LEDService::Instance().Toggle();
    }
    //LL_GPIO_ResetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);
    /* USER CODE END ADC3_IRQn 0 */
    /* USER CODE BEGIN ADC3_IRQn 1 */

    /* USER CODE END ADC3_IRQn 1 */
}

extern "C" int app_main() //
{
    // Start Communications Threads (UART/CAN)
    StartCommunicationThreads();
    osDelay(100);

     // Start Logger Service
    Logger::Instance().Enable(true);
    Logger::Instance().SetUART(uart);

    // Init LED Service Task
    StartLEDService();
    osDelay(1);

    // // Start Motor Control Task
    // StartMotorControlThread();
    // osDelay(100);

    // // Start Misc Polling Task
    // StartPollingThread();
    // osDelay(5);


   // osDelay(2000);
    //measure_motor_inductance();

    //measure_motor_resistance();
    //measure_motor_phase_order();
    //measure_motor_parameters();;

    //start_voltage_control();

    // Init a temp debug Task

    //DebugTask();
    
   int i = 0;
    // Infinite Loop.
    for (;;)
    {
    //     DWT->CYCCNT = 0;
    //     temp = fet_temp->SampleTemperature();
    //     uint32_t span = DWT->CYCCNT;
    //     Logger::Instance().Print("Thermal Perf: %d\r\n", span);

    //    // Logger::Instance().Print("Test 0x%lX.\r\n", LL_DBGMCU_GetDeviceID());
       //Logger::Instance().Print("Temp: %d, %d\r\n", i++, Cordic::ConvertAngle(-4.71239));
       
        osDelay(1000);
    }

    // Should not get here
    return 0;
}