

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

#include <DRV8323.h>
#include <LEDService.h>
#include <Logger.h>
#include <motor_controller_interface.h>
#include <MotorController.h>

__IO uint16_t value_adc1 = 0;
__IO uint16_t value_adc2 = 0;
__IO uint16_t value_adc3 = 0;

__IO uint16_t count_adc1 = 0;
__IO uint16_t count_adc2 = 0;
__IO uint16_t count_adc3 = 0;

osThreadId_t sig_thread;
extern "C" void signal_set(void *arg)
{
    Logger::Instance().Print("Start: \r\n");
    for (;;)
    {
        uint32_t flag = osThreadFlagsWait(0x3, osFlagsWaitAll, 3000);
        Logger::Instance().Print("Flag: %d\r\n", flag);
    }

    osThreadExit();
}

void StartCommunicationThreads()
{
    // Start UART
    osThreadAttr_t task_attributes;
    memset(&task_attributes, 0, sizeof(osThreadAttr_t));
    task_attributes.name = "UART_TASK";
    task_attributes.priority = (osPriority_t)osPriorityNormal;
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
    task_attributes.priority = (osPriority_t)osPriorityNormal;
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
    // for(;;)
    // {
    //     //LEDService::Instance().Blink(100,900);
    //     LEDService::Instance().On();
    //     osDelay(500);
    //     LEDService::Instance().Off();
    //     osDelay(500);
    //     LEDService::Instance().Blink(3000, 2000);
    //     osDelay(6000);
    // }

    // Spi TEST

    // // SPI1 = Encoder
    // GPIO_t mosi = {ENC_MOSI_GPIO_Port, ENC_MOSI_Pin};
    // GPIO_t miso = {ENC_MISO_GPIO_Port, ENC_MISO_Pin};
    // GPIO_t nss = {ENC_CS_GPIO_Port, ENC_CS_Pin};

    // SPIDevice encoder_dev(SPI1, mosi, miso, nss);
    // encoder_dev.Enable();
    // uint16_t position = 0;

    // for (;;)
    // {
    //     encoder_dev.Select();
    //     position = encoder_dev.Receive16();
    //     position &= 0x3FFF; // Data in last 14 bits.
    //     encoder_dev.Deselect();

    //     Logger::Instance().Print("Pos: %d\r\n", position);
    //     osDelay(100);
    // }

    // // Signal Test
    // osThreadAttr_t task_attributes;
    // memset(&task_attributes, 0, sizeof(osThreadAttr_t));
    // task_attributes.name = "SIGNAL_TEST";
    // task_attributes.priority = (osPriority_t) osPriorityNormal;
    // task_attributes.stack_size = 2048;

    // sig_thread = osThreadNew(signal_set, NULL, &task_attributes);

    // Setup Timers
   // LL_TIM_EnableIT_UPDATE(TIM8); // IT Updates

//     LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3); // Enable Channels

//     LL_TIM_EnableCounter(TIM8); // Enable Counting

//     LL_TIM_EnableAllOutputs(TIM8); // Advanced Timers turn on Outputs

//     // Enable Timers
//     // Set Frequency
//     float freq = 40000;
//     uint16_t period_ticks = 0;

//     period_ticks = SystemCoreClock / (2 * freq);
//     LL_TIM_SetPrescaler(TIM8, 0);             // No Prescaler
//     LL_TIM_SetAutoReload(TIM8, period_ticks); // Set Period
//     LL_TIM_SetRepetitionCounter(TIM8, 1);     // Loop Counter Decimator
//     LL_TIM_OC_SetCompareCH1(TIM8, 400);       // Set Duty Cycle Channel 1
//     LL_TIM_OC_SetCompareCH2(TIM8, 900);       // Set Duty Cycle Channel 2
//     LL_TIM_OC_SetCompareCH3(TIM8, 1500);      // Set Duty Cycle Channel 3


//     EnableADC(ADC1);
//     EnableADC(ADC2);
//     EnableADC(ADC3);
//     //EnableADC(ADC4);
//     //EnableADC(ADC5);

//     // Turn on Interrupts for ADC3 as it is not shared.
//     // Should Generate less Interrupts
//     LL_ADC_EnableIT_EOC(ADC3);

//     LL_ADC_REG_StartConversion(ADC1);
//     LL_ADC_REG_StartConversion(ADC2);
//     LL_ADC_REG_StartConversion(ADC3);
}

void FlashTest()
{
    // test_save save;
    // save.signature=100;
    // save.version=10;

    // bool status_open = FlashDevice::Instance().Open(ADDR_FLASH_PAGE_252, sizeof(save), FlashDevice::WRITE);
    // bool status = FlashDevice::Instance().Write(0, (uint8_t *)&save, sizeof(save));
    // FlashDevice::Instance().Close();

    // Logger::Instance().Print("Status: %d\r\n", status);

    // test_save load;
    // load.signature = 0;
    // load.version = 0;
    // load.b = 0;
    // FlashDevice::Instance().Open(ADDR_FLASH_PAGE_252, sizeof(load), FlashDevice::READ);
    // FlashDevice::Instance().Read(0, (uint8_t *)&load, sizeof(load));
    // FlashDevice::Instance().Close();

    // Logger::Instance().Print("Load: %d\r\n", load.signature);
}

void DRV_Test()
{
    // Spi TEST

    // SPI1 = Encoder
    GPIO_t mosi = {DRV_MOSI_GPIO_Port, DRV_MOSI_Pin};
    GPIO_t miso = {DRV_MISO_GPIO_Port, DRV_MISO_Pin};
    GPIO_t nss = {DRV_CS_GPIO_Port, DRV_CS_Pin};

    GPIO_t enable = {DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin};
    GPIO_t n_fault = {DRV_nFAULT_GPIO_Port, DRV_nFAULT_Pin};

    SPIDevice drv_spi(SPI2, mosi, miso, nss);
    drv_spi.Enable(); // Enable SPI

    DRV8323 drv_dev(&drv_spi, enable, n_fault);

    // Enable DRV
    drv_dev.EnableDriver();

    // Give it time to power up
    osDelay(10);

    // Init Driver Settings
    drv_dev.Init();

    // //for (;;)
    // //{

    //     uint16_t test = drv_dev.test();
    //    Logger::Instance().Print("Status: %X\r\n", 10);
    //     osDelay(100);
    // //}
    // drv_dev.DisableDriver();
    // osDelay(10000);
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

// /**
//   * @brief This function handles TIM8 update interrupt.
//   */
// extern "C" void TIM8_UP_IRQHandler(void)
// {
//     /* USER CODE BEGIN TIM8_UP_IRQn 0 */
//     if (LL_TIM_IsActiveFlag_UPDATE(TIM8))
//     {
//         LL_TIM_ClearFlag_UPDATE(TIM8);

//         // Do Callback
//        // LEDService::Instance().Toggle();
//     }
//     //LL_GPIO_SetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);
//     // for (int i = 0; i < 10; ++i)
//     // {
//     //     __NOP();
//     // }
//     // LL_GPIO_ResetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);
//     /* USER CODE END TIM8_UP_IRQn 0 */
//     /* USER CODE BEGIN TIM8_UP_IRQn 1 */

//     /* USER CODE END TIM8_UP_IRQn 1 */
// }

extern "C" int app_main() //
{
    // Start Logger Service
    Logger::Instance().Enable(true);

    // Start Communications Threads (UART/CAN)
    StartCommunicationThreads();
    osDelay(100);

    // Init LED Service Task
    StartLEDService();
    osDelay(500);

    // Start Motor Control Task
    StartMotorControlThread();
    osDelay(500);

    // Start Misc Polling Task
    StartPollingThread();
    osDelay(500);

    measure_motor_parameters();

    //measure_motor_phase_order();
    //measure_motor_parameters();
    //DRV_Test();
    // FlashTest();

   // start_voltage_control();

    // Init a temp debug Task
    DebugTask();

    // Infinite Loop.
    for (;;)
    {
    //     // volt= __LL_ADC_CALC_DATA_TO_VOLTAGE(3300UL, raw, LL_ADC_RESOLUTION_12B);
        // Logger::Instance().Print("Test.\r\n");
         osDelay(200);
    }

    // Should not get here
    return 0;
}