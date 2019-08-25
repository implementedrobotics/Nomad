/*
 * PositionSensor.cpp
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
#include "../../math_ops.h"

//MotorDevice *motor = 0;
MotorController *motor_controller = 0;

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

    //gate_driver_->enable_gd();
    zero_current(&controller.adc1_offset, &controller.adc2_offset); // Measure current sensor zero-offset
    gate_driver_->disable_gd();

	// Start PWM
	//StartPWM();

	// Start ADC()
	//StartADCs();

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
    for(;;)
    {
        // Update Motor Currents
        //rotor_sensor.Update();
        //printf(" Mechanical Angle:  %f    Electrical Angle:  %f    Raw:  %d\n\r", rotor_sensor.GetMechanicalPosition(), rotor_sensor.GetElectricalPosition(), rotor_sensor.GetRawPosition());
        printf("Hello :%f\r\n", current_scale);
        DoMotorControl();
        osDelay(1000);
    }
}
void MotorController::DoMotorControl()
{
    control_enabled_ = true;
	while(control_enabled_) // Do Forever
	{
		// voltage loop can wait forever
        osEvent test;
		if((test = osSignalWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, CURRENT_MEASUREMENT_TIMEOUT)).status != osEventSignal)
        {
		 	// TODO: Error here for timing
            printf("Loop Timeout :%f\r\n", current_scale);
		 	break;
		}
        printf("Looped :%d\r\n", test.status);
	}
}


void MotorController::StartPWM()
{
    
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;                        // enable the clock to GPIOC
    RCC->APB1ENR |= 0x00000001;                                 // enable TIM2 clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;                         // enable TIM1 clock

    GPIOC->MODER |= (1 << 10);                                  // set pin 5 to be general purpose output for LED
    //gpio->enable = new DigitalOut(ENABLE_PIN);
    gpio->pwm_u = new FastPWM(PIN_U);
    gpio->pwm_v = new FastPWM(PIN_V);
    gpio->pwm_w = new FastPWM(PIN_W);
    
    
    
     //ISR Setup     
    
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);                         //Enable TIM1 IRQ

    TIM1->DIER |= TIM_DIER_UIE;                                 // enable update interrupt
    TIM1->CR1 = 0x40;                                           // CMS = 10, interrupt only when counting up
    TIM1->CR1 |= TIM_CR1_UDIS;
    TIM1->CR1 |= TIM_CR1_ARPE;                                  // autoreload on, 
    TIM1->RCR |= 0x001;                                         // update event once per up/down count of tim1 
    TIM1->EGR |= TIM_EGR_UG;
 
    //PWM Setup

    TIM1->PSC = 0x0;                                            // no prescaler, timer counts up in sync with the peripheral clock
    TIM1->ARR = PWM_ARR;                                          // set auto reload, 40 khz
    TIM1->CCER |= ~(TIM_CCER_CC1NP);                            // Interupt when low side is on.
    TIM1->CR1 |= TIM_CR1_CEN;                                   // enable TIM1
}