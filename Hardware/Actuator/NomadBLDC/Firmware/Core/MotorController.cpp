/*
 * MotorConroller.cpp
 *
 *  Created on: August 25, 2019
 *      Author: Quincy Jones
 *
 * Copyright (c) <2019> <Quincy Jones - quincy@implementedrobotics.com/>
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
#include "MotorController.h"

// C System Files

// C++ System Files

// Project Includes
#include "mbed.h"
#include "rtos.h"
#include "FastPWM.h"
#include "../../math_ops.h"

//MotorDevice *motor = 0;
MotorController *motor_controller = 0;

// Globals
static int32_t g_adc1_offset;
static int32_t g_adc2_offset;

extern "C"
{
#include "motor_controller_interface.h"
}
void motor_controller_thread_entry()
{
    printf("Motor RT Controller Task Up.\n\r");

    // Init Motor and Position Sensor
    // Init Motor Controller
    motor_controller = new MotorController();
    motor_controller->Init();

    // Begin Control Loop
    motor_controller->StartControlFSM();
}

void current_measurement_cb()
{
    // Measure Currents/Bus Voltage
    // TODO: Filter v_bus current measurements
    //controller.v_bus = 0.95f * controller.v_bus + 0.05f * ((float)controller.adc3_raw) * V_SCALE;
    
    // Make sure control thread is ready
    if (motor_controller != 0 && motor_controller->ControlThreadReady())
    {
        osSignalSet(motor_controller->GetThreadID(), CURRENT_MEASUREMENT_COMPLETE_SIGNAL);
    }
}


bool zero_current_sensors()
{
    g_adc1_offset = 0;
    g_adc2_offset = 0;

    int32_t num_samples = 1024;
    for(int32_t i = 0; i < num_samples; i++) // Average num_samples of the ADC
    {
        osEvent test;
        if ((test = osSignalWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, CURRENT_MEASUREMENT_TIMEOUT)).status != osEventSignal)
        {
            // TODO: Error here for timing
            printf("ERROR: Zero Current FAILED!\r\n");
            return false;
        }
        g_adc1_offset += ADC1->DR;
        g_adc2_offset += ADC2->DR;

        // TODO: Function for setting duty cycles
        TIM1->CCR3 = (PWM_COUNTER_PERIOD_TICKS >> 1) * (1.0f); // Write duty cycles
        TIM1->CCR2 = (PWM_COUNTER_PERIOD_TICKS >> 1) * (1.0f);
        TIM1->CCR1 = (PWM_COUNTER_PERIOD_TICKS >> 1) * (1.0f);
    }
   
    g_adc1_offset = g_adc1_offset / num_samples;
    g_adc2_offset = g_adc2_offset / num_samples;
    
    return true;
}


// Control Loop Timer Interrupt Synced with PWM
extern "C" void TIM1_UP_TIM10_IRQHandler(void)
{
    if (TIM1->SR & TIM_SR_UIF) // TIM1 is up, and update interrupts are enabled on TIM1
    {
        ADC1->CR2 |= 0x40000000;  // Begin ADC Conversion
        current_measurement_cb(); // Callback
    }
    TIM1->SR = 0x0; // reset the status register
}

MotorController::MotorController()
{
    //current_meas_period_ = CURRENT_LOOP_PERIOD;
    //current_meas_freq_ = CURRENT_LOOP_FREQ;

    // Update Conductance
    //current_sense_conductance = 1.0f / SENSE_RESISTANCE;

    control_thread_id_ = 0;
    control_thread_ready_ = false;
    control_initialized_ = false;
    control_enabled_ = false;
}

void MotorController::Init()
{
    printf("MotorController::Init() - Motor Controller Initializing...\n\r");

    // Update Control Thread State
    control_thread_id_ = osThreadGetId();
    control_thread_ready_ = true;

    // Setup Gate Driver
    spi_handle_ = new SPI(PA_7, PA_6, PA_5);
    cs_ = new DigitalOut(PA_4);
    gate_enable_ = new DigitalOut(PA_11);
    gate_driver_ = new DRV832x(spi_handle_, cs_);

    gate_enable_->write(1);
    wait_us(100);
    gate_driver_->calibrate();
    wait_us(100);
    gate_driver_->write_DCR(0x0, 0x0, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1);
    wait_us(100);
    gate_driver_->write_CSACR(0x0, 0x1, 0x0, CSA_GAIN_40, 0x0, 0x0, 0x0, 0x0, SEN_LVL_1_0);
    wait_us(100);
    gate_driver_->write_OCPCR(TRETRY_4MS, DEADTIME_200NS, OCP_RETRY, OCP_DEG_8US, VDS_LVL_1_88);

    // Start PWM
    StartPWM();
    osDelay(150); // Delay for a bit to let things stabilize

    // Start ADCs
    StartADCs();
    osDelay(150); // Delay for a bit to let things stabilize

    //gate_driver_->enable_gd();
    zero_current_sensors(); // Measure current sensor zero-offset
    gate_driver_->disable_gd();

    // Do Motor Calibration

    // Check for Calibration of Motor (Could be loaded)
    // if(!motor_->is_calibrated_)
    // {
    // 	DoMotorCalibration();
    // }

    control_initialized_ = true;

    printf("MotorController::Init() - Motor Controller Initialized Successfully!\n\r");
}

void MotorController::StartControlFSM()
{
    for (;;)
    {
        // Update Motor Currents
        //rotor_sensor.Update();
        DoMotorControl();
        osDelay(1000);
    }
}
void MotorController::DoMotorControl()
{
    control_enabled_ = true;
    while (control_enabled_) // Do Forever
    {
        if (osSignalWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, CURRENT_MEASUREMENT_TIMEOUT).status != osEventSignal)
        {
            // TODO: Error here for timing
            printf("ERROR: Motor Controller Timeout!\r\n");
            break;
        }
    }
}

void MotorController::StartPWM()
{
    // TODO: I think this does not belong here
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable the clock to GPIOC
    RCC->APB1ENR |= 0x00000001;          // Enable TIM2 clock (TODO: What is on TIM2?)
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;  // Enable TIM1 clock

    // TODO: Move to main, elsewhere
    GPIOC->MODER |= (1 << 10); // set pin 5 to be general purpose output for LED

    // Setup PWM Pins
    PWM_u_ = new FastPWM(PIN_U);
    PWM_v_ = new FastPWM(PIN_V);
    PWM_w_ = new FastPWM(PIN_W);

    // Enable Interrupt Service Routines:
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn); //Enable TIM1/10 IRQ

    TIM1->DIER |= TIM_DIER_UIE; // Enable Update Interrupt
    TIM1->CR1 = 0x40;           // CMS = 10, Interrupt only when counting up
    TIM1->CR1 |= TIM_CR1_UDIS;  // Start Update Disable (TODO: Refactor to our "Enable PWM")
    TIM1->CR1 |= TIM_CR1_ARPE;  // Auto Reload Timer
    TIM1->RCR |= 0x001;         // Update event once per up count and down count.  This can be modified to have the control loop run at lower rates.
    TIM1->EGR |= TIM_EGR_UG;    // Generate an update event to reload the Prescaler/Repetition Counter immediately

    // PWM Setup
    TIM1->PSC = 0x0;                      // Set Prescaler to none.  Timer will count in sync with APB Block
    TIM1->ARR = PWM_COUNTER_PERIOD_TICKS; // Set Auto Reload Timer Value.  TODO: User Configurable.  For now 40khz.
    TIM1->CCER |= ~(TIM_CCER_CC1NP);      // Interupt when low side is on.
    TIM1->CR1 |= TIM_CR1_CEN;             // Enable TIM1
    TIM1->CR1 ^= TIM_CR1_UDIS;

    // This makes sure PWM is stopped if we have debug point/crash
    //__HAL_DBGMCU_FREEZE_TIM1();
}
void MotorController::StartADCs()
{
    // ADC Setup
    RCC->APB2ENR |= RCC_APB2ENR_ADC3EN; // clock for ADC3
    RCC->APB2ENR |= RCC_APB2ENR_ADC2EN; // clock for ADC2
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // clock for ADC1

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable clock for GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable clock for GPIOA

    ADC->CCR = 0x00000016;      // Regular simultaneous mode only
    ADC1->CR2 |= ADC_CR2_ADON;  //0x00000001;                    // ADC1 ON
    ADC1->SQR3 = 0x000000A;     // use PC_0 as input- ADC1_IN0
    ADC2->CR2 |= ADC_CR2_ADON;  //0x00000001;                    // ADC2 ON
    ADC2->SQR3 = 0x0000000B;    // use PC_1 as input - ADC2_IN11
    ADC3->CR2 |= ADC_CR2_ADON;  // ADC3 ON
    ADC3->SQR3 = 0x00000000;    // use PA_0, - ADC3_IN0
    GPIOC->MODER |= 0x0000000f; // Alternate function, PC_0, PC_1 are analog inputs
    GPIOA->MODER |= 0x3;        // PA_0 as analog input

    ADC1->SMPR1 |= 0x1; // 15 cycles on CH_10, 0b 001
    ADC2->SMPR1 |= 0x8; // 15 cycles on CH_11, 0b 0001 000
    ADC3->SMPR2 |= 0x1; // 15 cycles on CH_0, 0b 001;
}

void MotorController::EnablePWM(bool enable)
{
    if (enable)
    {
        TIM1->BDTR |= (TIM_BDTR_MOE);
    }
    else // Disable PWM Timer Unconditionally
    {
        TIM1->BDTR &= ~(TIM_BDTR_MOE);
    }

    // TODO: Here for when project ported from MBED to CUBEMX
    //enable ? __HAL_TIM_MOE_ENABLE(&htim8) : __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
}
