/*
 * FOCState.h
 *
 *  Created on: October 23, 2020
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

#ifndef NOMADBLDC_FSM_FOCSTATES_H_
#define NOMADBLDC_FSM_FOCSTATES_H_

// C System Files

// C++ System Files

// Third Party Includes

// Project Include Files
#include <FSM/NomadBLDCState.h>

class FOCState : public NomadBLDCState
{

public:
    FOCState();

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
    
    // kI for current step ramping  [(V/s)/A]
    float kI_; 
    
    // Cycles to run for Resistance Test
    uint32_t num_measure_cycles_;

    // Current test voltage @ step
    float test_voltage_;  
};

#endif // NOMADBLDC_FSM_FOCSTATES_H_
