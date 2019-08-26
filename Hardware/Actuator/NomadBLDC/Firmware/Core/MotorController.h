/*
 * MotorController.h
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

#ifndef CORE_MOTOR_CONTROLLER_H_
#define CORE_MOTOR_CONTROLLER_H_

// Some Constants
#define ADC_RES 12                    // ADC Resolution (12-Bits)
#define CURRENT_MEASUREMENT_TIMEOUT 2 //ms
#define VBUS_DIVIDER 16.0f            // (150K+10K/10K)
#define SENSE_RESISTANCE (1e-3)       // 1 milliohm sense resistor
#define SENSE_CONDUCTANCE (1000)      // SENSE_RESISTANCE^-1
#define CURRENT_SENSE_GAIN 40                  // Gain from current amplifier.  TODO: A Parameter

// Hardware Pins
#define PIN_U PA_10 // PWM Ouput PIN U
#define PIN_V PA_9  // PWM Ouput PIN V
#define PIN_W PA_8  // PWM Ouput PIN W
#define ENABLE_PIN PA_11 // DRV8323 Enable Pin

// Duty Cycle Min/Max
#define DTC_MAX 0.94f // Max phase duty cycle
#define DTC_MIN 0.0f  // Min phase duty cycle

// TODO: User configuratable.  Default to 40khz
#define PWM_COUNTER_PERIOD_TICKS 0x8CA // PWM Timer Auto Reload Value

//#define PWM_FREQ (float)SYS_CLOCK_FREQ * (1.0f / (2 * PWM_COUNTER_PERIOD_TICKS))
//#define CURRENT_LOOP_FREQ (PWM_FREQ/(PWM_INTERRUPT_DIVIDER))
//#define CURRENT_LOOP_PERIOD (1.0f/(float)CURRENT_LOOP_FREQ)

// C System Files

// C++ System Files

// Project Includes
#include "mbed.h"
#include "rtos.h"
#include "FastPWM.h"
#include "../DRV8323/DRV.h"

static const float voltage_scale = 3.3f * VBUS_DIVIDER / (float)(1 << ADC_RES);
static const float current_scale = 3.3f / (float)(1 << ADC_RES) * SENSE_CONDUCTANCE * 1.0f / CURRENT_SENSE_GAIN;

// Signals
typedef enum {
	CURRENT_MEASUREMENT_COMPLETE_SIGNAL = 0x1,
	MOTOR_ERROR_SIGNAL = 0x2
} thread_signal_type_t;

class MotorController
{

public:
    // Motor Controller Parameters
    struct Config_t
    {
        float k_d;            // Current ControlCURRENT_MEASUREMENT_TIMEOUTler Loop Gain (D Axis)
        float k_q;            // Current ControlCURRENT_MEASUREMENT_TIMEOUTler Loop Gain (Q Axis)
        float k_i_d;          // Current ControlCURRENT_MEASUREMENT_TIMEOUTler Integrator Gain (D Axis)
        float k_i_q;          // Current Controller Integrator Gain (Q Axis)
        float overmodulation; // Overmodulation Amount
        float current_limit;  // Max Current Limit
    };

    MotorController(); // TODO: Pass in motor object

    void Init();            // Init Controller
    void StartControlFSM(); // Begin Control Loop
    void StartPWM();        // Setup PWM Timers/Registers
    void StartADCs();       // Start ADC Inputs
    void EnablePWM(bool enable); // Enable/Disable PWM Timers

    void ZeroCurrentSensors();   // Zero Current Sensor Offsets (TODO: Next PCB Revision connect DC_CAL pin to DRV832)

    osThreadId GetThreadID() { return control_thread_id_; }
    bool IsInitialized() { return control_initialized_; }
	bool ControlThreadReady() { return control_thread_ready_; }

    bool WriteConfig(); // Write Configuration to Flash Memory
    bool ReadConfig();  // Read Configuration from Flash Memory

private:

    void DoMotorControl();              // Motor Control Loop

    Config_t config_;                   // Position Sensor Configuration Parameters

    float voltage_bus_;                 // Bus Voltage (Volts)
    float controller_update_period_;    // Controller Update Period (Seconds)

    osThreadId control_thread_id_;      // Controller Thread ID
	bool control_thread_ready_;         // Controller thread ready/active
	bool control_initialized_;          // Controller thread initialized
	volatile bool control_enabled_;     // Controller thread enabled

    DRV832x *gate_driver_;              // Gate Driver Device for DRV8323
    SPI *spi_handle_;                   // SPI Handle for Communication to Gate Driver
    DigitalOut *cs_;                    // Chip Select Pin
    DigitalOut *gate_enable_;           // Enable Pin for Gate Driver

    // Make Static?
    FastPWM *PWM_u_;                    // PWM Output Pin
    FastPWM *PWM_v_;                    // PWM Output Pin
    FastPWM *PWM_w_;                    // PWM Output Pin

    bool dirty_;                        // Have unsaved changed to config
};

#endif // CORE_MOTOR_CONTROLLER_H_