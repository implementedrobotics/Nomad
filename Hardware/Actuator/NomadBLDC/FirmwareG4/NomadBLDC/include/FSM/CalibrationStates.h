/*
 * CalibrationStates.h
 *
 *  Created on: October 21, 2020
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
 */

#ifndef NOMADBLDC_FSM_CALIBRATIONSTATES_H_
#define NOMADBLDC_FSM_CALIBRATIONSTATES_H_

// C System Files

// C++ System Files

// Third Party Includes

// Project Include Files
#include <FSM/NomadBLDCState.h>

class MeasureResistanceState : public NomadBLDCState
{

public:
    MeasureResistanceState();

    // Called upon a state change and we enter this state
    // current_time = current controller tick time
    void Enter_(uint32_t current_time);

    // current_time = current controller tick time
    // Called upon a state change and we are exiting this state
    void Exit_(uint32_t current_time);

    // Logic to run each iteration of the state machine run
    // dt = time step for this iteration
    void Run_(float dt);

private:    
    
    // Cycles to run for Resistance Test
    uint32_t num_measure_cycles_;

    // Current test voltage @ step
    float test_voltage_;  
};

class MeasureInductanceState : public NomadBLDCState
{

public:
    MeasureInductanceState();

    // Called upon a state change and we enter this state
    // current_time = current controller tick time
    void Enter_(uint32_t current_time);

    // current_time = current controller tick time
    // Called upon a state change and we are exiting this state
    void Exit_(uint32_t current_time);

    // Logic to run each iteration of the state machine run
    // dt = time step for this iteration
    void Run_(float dt);

private:    
    
    // Low/High Current Steps on Alpha Axis
    float I_alpha_[2];
    
    // Cycles to run for Resistance Test
    uint32_t num_measure_cycles_;

    int step_id_;
};

class MeasurePhaseOrderState : public NomadBLDCState
{

public:
    typedef enum
    {
        LOCK_ROTOR = 0,
        MEASURE_PHASE_ORDER = 1
    } state_t;

    MeasurePhaseOrderState();

    // Called upon a state change and we enter this state
    // current_time = current controller tick time
    void Enter_(uint32_t current_time);

    // current_time = current controller tick time
    // Called upon a state change and we are exiting this state
    void Exit_(uint32_t current_time);

    // Logic to run each iteration of the state machine run
    // dt = time step for this iteration
    void Run_(float dt);

private:    
    
    // Mechnical Theta Start Angle
    float theta_start_;

    // Mechanical Theta End Angle after Scan
    float theta_end_;

    // Scan Test Voltage
    float test_voltage_;

    // Current Reference Angle
    float reference_angle_; 

    // Current State of Calibration
    state_t state_;
};



class MeasureEncoderOffsetState : public NomadBLDCState
{

public:
    typedef enum
    {
        LOCK_ROTOR = 0,
        CALIBRATE_FORWARD = 1,
        CALIBRATE_BACKWARD = 2
    } state_t;

    static constexpr int16_t kLUTSize = 128;
 
    MeasureEncoderOffsetState();
    ~MeasureEncoderOffsetState();

    // Called upon a state change and we enter this state
    // current_time = current controller tick time
    void Enter_(uint32_t current_time);

    // current_time = current controller tick time
    // Called upon a state change and we are exiting this state
    void Exit_(uint32_t current_time);

    // Logic to run each iteration of the state machine run
    // dt = time step for this iteration
    void Run_(float dt);

    // Run Initilization Setup, Allocate Memory Etc
    void Setup();

protected:
    void ComputeOffsetLUT();

private:    

    Motor *motor_;

    // Constants

    // Sampling Window Size
    int32_t window_size_;

    // Number of samples per window.  Should be multiple of number of pole pairs
    int32_t num_samples_;

    // Sub Sample Smoothing
    int32_t num_sub_samples_;     
    
    // Mechnical Theta Actual (Non Compensated) Angle
    float theta_actual_;

    // Scan Test Voltage
    float test_voltage_;

    // Current Reference Angle
    float reference_angle_; 

    /* Compensation Table Variables */
    // Error Vector Forward Rotation
    float *error_forward_;  

    // Error Vector Backward Rotation
    float *error_backward_;

    // Lookup Table
    int8_t *LUT_;

    // Raw Positions (Counts)
    int32_t *raw_forward_;
    int32_t *raw_backward_;

    float *error_;
    float *error_filtered_;

    // Current State of Calibration
    state_t state_;
};



#endif // NOMADBLDC_FSM_CALIBRATIONSTATES_H_
