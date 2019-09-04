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
#define CURRENT_SENSE_GAIN 40         // Gain from current amplifier.  TODO: A Parameter

// Hardware Pins
#define PIN_A PA_10      // PWM Ouput PIN A
#define PIN_B PA_9       // PWM Ouput PIN B
#define PIN_C PA_8       // PWM Ouput PIN C
#define ENABLE_PIN PA_11 // DRV8323 Enable Pin

// Duty Cycle Min/Max
#define DTC_MAX 0.94f // Max phase duty cycle
#define DTC_MIN 0.0f  // Min phase duty cycle

// TODO: User configuratable.  Default to 40khz
#define PWM_COUNTER_PERIOD_TICKS 0x8CA // PWM Timer Auto Reload Value

// TODO: User Configurable Parameter
#define CONTROL_LOOP_FREQ 40000.0f
#define CONTROL_LOOP_PERIOD 1.0f / CONTROL_LOOP_FREQ

//#define PWM_FREQ (float)SYS_CLOCK_FREQ * (1.0f / (2 * PWM_COUNTER_PERIOD_TICKS))
//#define CURRENT_LOOP_FREQ (PWM_FREQ/(PWM_INTERRUPT_DIVIDER))
//#define CURRENT_LOOP_PERIOD (1.0f/(float)CURRENT_LOOP_FREQ)

// C System Files
#include <arm_math.h>

// C++ System Files

// Project Includes
#include "mbed.h"
#include "rtos.h"
#include "FastPWM.h"
#include "Motor.h"
#include "../DRV8323/DRV.h"

static const float voltage_scale = 3.3f * VBUS_DIVIDER / (float)(1 << ADC_RES);
static const float current_scale = 3.3f / (float)(1 << ADC_RES) * SENSE_CONDUCTANCE * 1.0f / CURRENT_SENSE_GAIN;

extern Motor *motor;
extern MotorController *motor_controller;

// Signals
typedef enum
{
    CURRENT_MEASUREMENT_COMPLETE_SIGNAL = 0x1,
    MOTOR_ERROR_SIGNAL = 0x2,
    CHANGE_MODE_SIGNAL = 0x3,
} thread_signal_type_t;

typedef enum
{
    IDLE_MODE = 0,
    ERROR_MODE = 1,
    CALIBRATION_MODE = 2,
    FOC_CURRENT_MODE = 3,
    FOC_VOLTAGE_MODE = 4,
    FOC_TORQUE_MODE = 5,
    ENCODER_DEBUG = 6,
} control_mode_type_t;

typedef enum
{
    FOC_TIMING_ERROR = 0,
    OVERVOLTAGE_ERROR = 1,
    UNDERVOLTAGE_ERROR = 2,
    OVERTEMPERATURE_ERROR = 3
} error_type_t;

class MotorController
{
    // TODO: Setpoint references, etc.
public:
    // Motor Controller Parameters
    struct __attribute__((__packed__))  Config_t
    {
        float k_d;               // Current Controller Loop Gain (D Axis)
        float k_q;               // Current Controller Loop Gain (Q Axis)
        float k_i_d;             // Current Controller Integrator Gain (D Axis)
        float k_i_q;             // Current Controller Integrator Gain (Q Axis)
        float overmodulation;    // Overmodulation Amount
        float velocity_limit;    // Limit on maximum velocity
        float current_limit;     // Max Current Limit
        float current_bandwidth; // Current Loop Bandwidth (200 to 2000 hz)
    };

    MotorController(Motor *motor, float sample_time); // TODO: Pass in motor object

    void Init();            // Init Controller
    void StartControlFSM(); // Begin Control Loop
    void StartPWM();        // Setup PWM Timers/Registers
    void StartADCs();       // Start ADC Inputs

    void EnablePWM(bool enable); // Enable/Disable PWM Timers

    void SetModulationOutput(float theta, float v_d, float v_q);  // Helper Function to compute PWM Duty Cycles directly from D/Q Voltages
    void SetModulationOutput(float v_alpha, float v_beta);        // Helper Function to compute PWM Duty Cycles directly from Park Inverse Transformed Alpha/Beta Voltages
    void SetDuty(float duty_A, float duty_B, float duty_C);       // Set PWM Duty Cycles Directly

    // Transforms
    void dqInverseTransform(float theta, float d, float q, float *a, float *b, float *c); // DQ Transfrom -> A, B, C voltages

    void ParkInverseTransform(float theta, float d, float q, float *alpha, float *beta);
    void ParkTransform(float theta, float alpha, float beta, float *d, float *q);
    void ClarkeInverseTransform(float alpha, float beta, float *a, float *b, float *c);
    void ClarkeTransform(float I_a, float I_b, float *alpha, float *beta);

    void SVM(float a, float b, float c, float *dtc_a, float *dtc_b, float *dtc_c);

    inline osThreadId GetThreadID() { return control_thread_id_; }
    inline bool IsInitialized() { return control_initialized_; }
    inline bool ControlThreadReady() { return control_thread_ready_; }

    inline void SetControlMode(control_mode_type_t mode) {control_mode_ = mode;}

    bool WriteConfig(Config_t config); // Write Configuration to Flash Memory
    bool ReadConfig(Config_t config);  // Read Configuration from Flash Memory

    // Public for now...  TODO: Need something better here
    float voltage_bus_; // Bus Voltage (Volts)
    volatile control_mode_type_t control_mode_; // Controller Mode

    Config_t config_; // Position Sensor Configuration Parameters
    
private:

    void DoMotorControl(); // Motor Control Loop

    float controller_update_period_;            // Controller Update Period (Seconds)
    float current_max_;                         // Maximum allowed current before clamped by sense resistor

    bool control_thread_ready_;                 // Controller thread ready/active
    bool control_initialized_;                  // Controller thread initialized
    volatile bool control_enabled_;             // Controller thread enabled

    osThreadId control_thread_id_;              // Controller Thread ID

    DRV832x *gate_driver_;    // Gate Driver Device for DRV8323
    SPI *spi_handle_;         // SPI Handle for Communication to Gate Driver
    DigitalOut *cs_;          // Chip Select Pin
    DigitalOut *gate_enable_; // Enable Pin for Gate Driver

    // Make Static?
    FastPWM *PWM_A_; // PWM Output Pin
    FastPWM *PWM_B_; // PWM Output Pin
    FastPWM *PWM_C_; // PWM Output Pin

    Motor *motor_; // Motor Object
    bool dirty_;   // Have unsaved changed to config
};

#endif // CORE_MOTOR_CONTROLLER_H_