

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
Cordic cordic2;
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
void dqtest(float theta, float a, float b, float c, float *d, float *q) __attribute__((section(".ccmram")));

void dqtest(float theta, float a, float b, float c, float *d, float *q) 
{
    // DQ0 Transform
    // Phase current amplitude = length of dq vector
    // i.e. iq = 1, id = 0, peak phase current of 1
    float cf, sf;
    cordic2.CosSin(theta, cf, sf);
    //float cf = cosf(theta);
    //float sf = sinf(theta);
    *d = 0.6666667f * (cf * a + (0.86602540378f * sf - .5f * cf) * b + (-0.86602540378f * sf - .5f * cf) * c); ///Faster DQ0 Transform
    *q = 0.6666667f * (-sf * a - (-0.86602540378f * cf - .5f * sf) * b - (0.86602540378f * cf - .5f * sf) * c);
}

void ParkInverseTransform(float theta, float d, float q, float *alpha, float *beta) __attribute__((section(".ccmram")));
void ParkInverseTransform(float theta, float d, float q, float *alpha, float *beta)
{
    float cos_theta, sin_theta;
    cordic2.CosSin(theta, cos_theta, sin_theta);
    //float cos_theta = arm_cos_f32(theta);
    //float sin_theta = arm_sin_f32(theta);

    *alpha = d * cos_theta - q * sin_theta;
    *beta = q * cos_theta + d * sin_theta;
}



/**
  * @brief This function handles ADC3 global interrupt.
  */
extern "C" void ADC3_IRQHandler(void)
{
    
    /* USER CODE BEGIN ADC3_IRQn 0 */
    // TODO: Save this count as a metric to verify no aliasing
    //uint32_t count = DWT->CYCCNT;
    if (LL_ADC_IsActiveFlag_EOC(ADC1))
    {
       // LL_GPIO_SetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);
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

        //LL_GPIO_SetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);
        /* Clear flag ADC group regular end of unitary conversion */
        LL_ADC_ClearFlag_EOC(ADC3);
        // Send it
        //osThreadFlagsSet(CURRENT_MEASUREMENT_COMPLETE_SIGNAL);
       
        
        // for (int i = 0; i < 10; ++i)
        // {
        //   __NOP();
        // }
        

         current_measurement_cb();
      //  LL_GPIO_ResetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);
        //value_adc3 = LL_ADC_REG_ReadConversionData12(ADC3);
       // count_adc3 = count;
        //LEDService::Instance().Toggle();
    }
    
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

    // Start Motor Control Task
    StartMotorControlThread();
    osDelay(100);

    // Start Misc Polling Task
    StartPollingThread();
    osDelay(5);


    osDelay(2000);
    //measure_motor_inductance();

    //measure_motor_resistance();
    //measure_motor_phase_order();
    //measure_motor_parameters();;

   // start_voltage_control();

    // Init a temp debug Task

    //DebugTask();


     
    //  cordic.Init();
    //  cordic.SetPrecision(LL_CORDIC_PRECISION_6CYCLES);
//     //Cordic::Instance().Init(); 
//     //Cordic::Instance().SetPrecision(LL_CORDIC_PRECISION_6CYCLES);
     float theta = PI;
    int i = 0;

    float sin, cos;
    float I_d, I_q;

    float v_alpha, v_beta;
    float v_d = 0.0f;
    float v_q = 2.0f;
//    uint32_t start_ticks;
// uint32_t stop_ticks;
// uint32_t elapsed_ticks;

    // Infinite Loop.
    for (;;)
    {
//          start_ticks = SysTick->VAL;
//     //     temp = fet_temp->SampleTemperature();

   //     LL_GPIO_SetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);

        
     // Logger::Instance().Print("Test 0x%lX.\r\n", LL_DBGMCU_GetDeviceID());
        
        //cordic.CosSin(theta, cos, sin);
     //   ParkInverseTransform(theta, v_d, v_q, &v_alpha, &v_beta);

     //   dqtest(theta, 1.0f, 2.0f, -3.0f, &I_d, &I_q); 
//     sin = arm_sin_f32(theta);
 //    cos = arm_cos_f32(theta);

//        // sin = arm_sin_f32(theta);
//        // cos = arm_cos_f32(theta);
     //    arm_sin_cos_f32(theta, &sin, &cos);

  //  LL_GPIO_ResetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);
//         stop_ticks = SysTick->VAL;
// elapsed_ticks = start_ticks-stop_ticks;
//    // uint32_t span = DWT->CYCCNT;
//     Logger::Instance().Print("Thermal Perf: %d\r\n", elapsed_ticks);
       // Logger::Instance().Print("%d - Cos: %f Sin: %f | Theta: %f\r\n", i++, I_d, I_q, theta);
       // theta += .128747472f;
        osDelay(1000);
    }

    // Should not get here
    return 0;
}