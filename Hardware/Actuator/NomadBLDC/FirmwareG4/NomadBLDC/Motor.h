/*
 * Motor.h
 *
 *  Created on: August 24, 2019
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

#ifndef CORE_MOTOR_H_
#define CORE_MOTOR_H_

// C System Files

// C++ System Files

// Project Includes
#include "mbed.h"
#include "rtos.h"
#include "PositionSensor.h"

class MotorController;
class Motor
{
friend class MotorController;

public:
    // Motor State
    struct State_t
    {
        float I_a;            // Phase A Currents
        float I_b;            // Phase B Currents
        float I_c;            // Phase C Currents

        //float V_a;          // Phase A Voltage
        //float V_b;          // Phase B Voltage
        //float V_c;          // Phase C Voltage

        float theta_mech;     // Mechanical Position @ Output (Radians)
        float theta_mech_dot; // Mechanical Velocity @ Output (Radians/Sec)
        float theta_mech_true;// Mechanical Position @ Output w/ Offset (Radians) 
        float theta_elec;     // Electrical Position @ Rotor (Radians)
        float theta_elec_dot; // Electrical Velocity @ Rotor (Radians/Sec)

        float windings_temp;  // Motor Windings Temperature (Degrees Celcius)
    };

    // Motor Parameters
    struct Config_t
    {
        uint32_t num_pole_pairs;  // Pole Pairs of Motor (PAIRS)
        float continuous_current_max; // Thermally calibrated Allowable Continuous Current (A)
        float continuous_current_tau; // Time Constant for Continuous Current (Seconds)
        float phase_resistance;   // Phase Resistance (Ohms)
        float phase_inductance_d; // D Axis Phase Inductance (Henries)
        float phase_inductance_q; // Q Axis Phase Inductance (Henries)

        // TODO: Just flux linkage?
        float K_v;                // Motor KV Rating (RPM/V)
        float flux_linkage;       // Rotor Flux Linkage (Webers)
        float K_t;                // Torque Constant (N*m/A)

        // TOOD: No need to store this.  K_t * K_t_out
        float K_t_out;            // Torque Constant @ Output (N*m/A)
        // TODO: Custom override for torques if measured experimentally?
        float gear_ratio;         // Gear Box Ratio
        int32_t phase_order;      // Winding Phase Order

        // TODO: Kill these 2 and pass in per calibration
        float calib_current;      // Calibration Current
        float calib_voltage;      // Calibration Voltage

        // TODO: This has to be better
        int32_t calibrated;          // Calibrated
    };

    Motor(float sample_time=0.000025f, float K_v = 100, uint32_t pole_pairs = 21);

    void SetPolePairs(uint32_t pole_pairs);            // Set Motor Pole Count
    void SetKV(float K_v);                             // Set Motor KV Rating
    void ZeroOutputPosition();                         // Zero Mechanical Position Offset
    bool Calibrate(MotorController *controller);       // Calibrate Motor Routine
    void SetSampleTime(float sample_time);             // Update controller sample rate
    void Update();                                     // Update Motor State

    inline PositionSensorAS5x47* PositionSensor() { return rotor_sensor_; }
    //bool WriteConfig(); // Write Configuration to Flash Memory
    //bool ReadConfig();  // Read Configuration from Flash Memory
    void PrintPosition();
    
    State_t state_;  // Motor State
    Config_t config_; // Motor Params
    
private:
    
    // Measure Routines
    bool MeasureMotorResistance(MotorController *controller, float test_current, float max_voltage);
    bool MeasureMotorInductance(MotorController *controller, float voltage_low, float voltage_high);
    bool CalibrateEncoderOffset(MotorController *controller); // Calibrate Encoder Offset
    bool OrderPhases(MotorController *controller);     // Check Phase Order
    bool LockRotor(MotorController *controller, float lock_duration, float lock_voltage); // Lock Rotor to A/D Axis

    float sample_time_; // Update Sample Time (=Current Control Update Rate)
    bool dirty_; // Has unsaved changes to config

    PositionSensorAS5x47 *rotor_sensor_; // Rotor Position Sensor
};

#endif // CORE_MOTOR_H_