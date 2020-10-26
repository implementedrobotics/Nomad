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
#define CURRENT_MEASUREMENT_TIMEOUT 2 // 2 ms
#define CALIBRATION_MEASUREMENT_TIMEOUT 15000 // 15 seconds
#define VBUS_DIVIDER 16           // (150K+10K/10K)
#define SENSE_RESISTANCE (5e-4)       // 0.5 milliohm sense resistor
#define SENSE_CONDUCTANCE (2000)      // SENSE_RESISTANCE^-1
#define CURRENT_SENSE_GAIN 40         // Gain from current amplifier.  TODO: A Parameter

// Hardware Pins
#define PIN_A PA_10      // PWM Ouput PIN A
#define PIN_B PA_9       // PWM Ouput PIN B
#define PIN_C PA_8       // PWM Ouput PIN C
#define ENABLE_PIN PC_9 // DRV8323 Enable Pin

// Duty Cycle Min/Max
#define DTC_MAX 0.94f // Max phase duty cycle
#define DTC_MIN 0.0f  // Min phase duty cycle

// C System Files
#include <arm_math.h>

// C++ System Files

// Project Includes
#include "cmsis_os2.h"
#include "Motor.h"
#include "RMSCurrentLimiter.h"
#include "DRV8323.h"
#include <Peripherals/spi.h>
#include <Peripherals/gpio.h>
#include <nomad_hw.h>

static const float voltage_scale = 3.3f * VBUS_DIVIDER / (float)(1 << ADC_RES);
static const float current_scale = 3.3f / (float)(1 << ADC_RES) * SENSE_CONDUCTANCE * 1.0f / CURRENT_SENSE_GAIN;

extern Motor *motor;
extern MotorController *motor_controller;

class ADCDevice;
class Thermistor;
class NomadBLDCFSM;

// Measurement Struct
typedef union 
{
    float f32;
    int32_t i32;
    uint32_t u32;
} measurement_t;

// Signals
typedef enum
{
    CURRENT_MEASUREMENT_COMPLETE_SIGNAL = 0x1,
    MOTOR_ERROR_SIGNAL = 0x2,
    CHANGE_MODE_SIGNAL = 0x3,
    CALIBRATION_MEASUREMENT_COMPLETE_SIGNAL = 0x4
} thread_signal_type_t;

typedef enum
{
    MEASURE_RESISTANCE_COMPLETE = 0,
    MEASURE_INDUCTANCE_COMPLETE = 1,
    MEASURE_PHASE_ORDER_COMPLETE = 2,
    MEASURE_ENCODER_OFFSET_COMPLETE = 3
} command_feedback_t;

typedef enum
{
    STARTUP_MODE = 0,
    IDLE_MODE = 1,
    ERROR_MODE = 2,
    MEASURE_RESISTANCE_MODE = 3,
    MEASURE_INDUCTANCE_MODE = 4,
    MEASURE_PHASE_ORDER_MODE = 5,
    MEASURE_ENCODER_OFFSET_MODE = 6,
    CALIBRATION_MODE = 7,
    FOC_CURRENT_MODE = 8,
    FOC_VOLTAGE_MODE = 9,
    FOC_TORQUE_MODE = 10,
    FOC_SPEED_MODE = 11
} control_mode_type_t;

typedef enum
{
    SUCCESSFUL = 0,
    FOC_TIMING_ERROR = 1,
    OVERVOLTAGE_ERROR = 2,
    UNDERVOLTAGE_ERROR = 3,
    OVERTEMPERATURE_ERROR = 4,
    NOT_CALIBRATED_ERROR = 5,
    MEASUREMENT_OUT_OF_RANGE = 6,
    MEASUREMENT_TIMEOUT = 7,

} error_type_t;

class MotorController
{
    // TODO: Setpoint references, etc.
public:

    // Motor Controller Parameters
    struct Config_t
    {
        float k_d;               // Current Controller Loop Gain (D Axis)
        float k_q;               // Current Controller Loop Gain (Q Axis)
        float k_i_d;             // Current Controller Integrator Gain (D Axis)
        float k_i_q;             // Current Controller Integrator Gain (Q Axis)
        float alpha;             // Current Reference Filter Coefficient
        float overmodulation;    // Overmodulation Amount
        float velocity_limit;    // Limit on maximum velocity
        float position_limit;    // Limit on position input
        float torque_limit;      // Torque Limit
        float current_limit;     // Max Current Limit
        float current_bandwidth; // Current Loop Bandwidth (200 to 2000 hz)
        float K_p_min;           // Position Gain Minimum
        float K_p_max;           // Position Gain Maximum
        float K_d_min;           // Velocity Gain Minimum
        float K_d_max;           // Velocity Gain Maximum
        float pwm_freq;          // PWM Switching Frequency
        uint32_t foc_ccl_divider; // Divider to use for FOC Current control loop frequency
    };

    struct State_t
    {
        // Current Control
        float I_d;                   // Transformed Current (D Axis)
        float I_q;                   // Transformed Current (Q Axis)
        float I_d_filtered;          // Measured Current Filtered (D Axis)
        float I_q_filtered;          // Measured Current Filtered (Q Axis)

        float I_d_ref;               // Current Reference (D Axis)
        float I_q_ref;               // Current Reference (Q Axis)
        float I_d_ref_filtered;      // Current Reference Filtered (D Axis)
        float I_q_ref_filtered;      // Current Reference Filtered (Q Axis)

        float d_int;                 // Current Integral Error
        float q_int;                 // Current Integral Error

        float V_d;                   // Voltage (D Axis)
        float V_q;                   // Voltage (Q Axis)
        volatile float V_d_ref;      // Voltage Reference (D Axis)
        volatile float V_q_ref;      // Voltage Reference (Q Axis)
        volatile float Voltage_bus;  // Bus Voltage

        // Torque Control
        volatile float Pos_ref;      // Position Setpoint Reference
        volatile float Vel_ref;      // Velocity Setpoint Reference
        volatile float K_p;          // Position Gain N*m/rad
        volatile float K_d;          // Velocity Gain N*m/rad/s
        volatile float T_ff;         // Feed Forward Torque Value N*m

        // Duty Cycles
        float dtc_A;                 // Duty Cycle for A phase
        float dtc_B;                 // Duty Cycle for B phase
        float dtc_C;                 // Duty Cycle for C phase

        // RMS Limiting
        float I_rms;                 // Motor RMS Current Value
        float I_max;                 // Maximum Allowable Commanded Current in next Time Step

        // Temps
        float fet_temp;              // FET Temperature

        // Timeouts
        uint32_t timeout;            // Keep up with number of controller timeouts for missed deadlines
    };

    MotorController(Motor *motor);

    static MotorController* GetInstance() { return singleton_; } // Singleton Instance
    inline Motor* GetMotor() { return motor_; }

    void Init();            // Init Controller
    void Reset();           // Reset Controller

    void CurrentMeasurementCB() CCM_ATTRIBUTE; // Callback from ADC Current Measurement
    bool RunControlFSM() CCM_ATTRIBUTE;   // Do an FSM Step
    void StartPWM();        // Setup PWM Timers/Registers
    void StartADCs();       // Start ADC Inputs

    void SampleBusVoltage();  // Poll/Update Bus Voltage ADC value
    void SampleFETTemperature();  // Poll/Update FET Thermistor value

    // TODO: Moved to PWM Generator
    void EnablePWM(bool enable); // Enable/Disable PWM Timers
    void SetModulationOutput(float theta, float v_d, float v_q) CCM_ATTRIBUTE;  // Helper Function to compute PWM Duty Cycles directly from D/Q Voltages
    void SetModulationOutput(float v_alpha, float v_beta) CCM_ATTRIBUTE;        // Helper Function to compute PWM Duty Cycles directly from Park Inverse Transformed Alpha/Beta Voltages
    void SetDuty(float duty_A, float duty_B, float duty_C) ;       // Set PWM Duty Cycles Directly
    void SVM(float a, float b, float c, float *dtc_a, float *dtc_b, float *dtc_c) CCM_ATTRIBUTE;

    void UpdateControllerGains();                                 // Controller Gains from Measured Motor Parameters

    // TOOD: Move This outTransforms
    void dqInverseTransform(float theta, float d, float q, float *a, float *b, float *c) CCM_ATTRIBUTE; // DQ Transfrom -> A, B, C voltages
    void dq0(float theta, float a, float b, float c, float *d, float *q) CCM_ATTRIBUTE;
    
    // TODO: Move All This Out
    void ParkInverseTransform(float theta, float d, float q, float *alpha, float *beta) CCM_ATTRIBUTE;
    void ParkTransform(float theta, float alpha, float beta, float *d, float *q) CCM_ATTRIBUTE;
    void ClarkeInverseTransform(float alpha, float beta, float *a, float *b, float *c) CCM_ATTRIBUTE;
    void ClarkeTransform(float I_a, float I_b, float *alpha, float *beta) CCM_ATTRIBUTE;

    inline bool IsInitialized() { return control_initialized_; }

    // TODO: Move this and set FSM
    inline void SetControlMode(control_mode_type_t mode) {control_mode_ = mode;}
    inline control_mode_type_t GetControlMode() {return control_mode_;}

    bool CheckErrors();                 // Check for Controller Errors

    inline float GetControlUpdatePeriod() {return controller_update_period_;}

    ADCDevice* GetADC1() const { return adc_1_; }
    ADCDevice* GetADC2() const { return adc_2_; }
    ADCDevice* GetADC3() const { return adc_3_; }

    DRV8323* GetGateDriver() const { return gate_driver_; }

    bool WriteConfig(Config_t config); // Write Configuration to Flash Memory
    bool ReadConfig(Config_t config);  // Read Configuration from Flash Memory

    void CurrentControl(); // Current Control Loop
    void TorqueControl(); // Torque Control Fucntion

    // Public for now...  TODO: Need something better here
    volatile control_mode_type_t control_mode_; // Controller Mode

    Config_t config_; // Controller Configuration Parameters
    State_t state_;   // Controller State Struct

    // PWM Variables
    uint16_t pwm_counter_period_ticks_;
    float controller_loop_freq_;
    float controller_update_period_;            // Controller Update Period (Seconds)
    
private:


    
    float current_max_;                         // Maximum allowed current before clamped by sense resistor

    bool control_thread_ready_;                 // Controller thread ready/active
    bool control_initialized_;                  // Controller thread initialized
    volatile bool control_enabled_;             // Controller thread enabled

    DRV8323 *gate_driver_;    // Gate Driver Device for DRV8323
    SPIDevice *spi_handle_;         // SPI Handle for Communication to Gate Driver

    Motor *motor_; // Motor Object
    bool dirty_;   // Have unsaved changed to config

    RMSCurrentLimiter *current_limiter_;
    float rms_current_sample_period_;

    // ADCs
    ADCDevice *adc_1_;
    ADCDevice *adc_2_;
    ADCDevice *adc_3_;
    ADCDevice *vbus_adc_;

    // Thermistors
    Thermistor *fet_therm_;

    // Control FSM
    NomadBLDCFSM* control_fsm_;

    //float control_loop_period_;
    static MotorController *singleton_; // Singleton
};

#endif // CORE_MOTOR_CONTROLLER_H_