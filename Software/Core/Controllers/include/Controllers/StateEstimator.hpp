/*
 * StateEstimator.hpp
 *
 *  Created on: July 13, 2019
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
 */

#ifndef NOMAD_CORE_CONTROLLERS_STATEESTIMATOR_H_
#define NOMAD_CORE_CONTROLLERS_STATEESTIMATOR_H_

// C System Files
#include <stdint.h>

// C++ System Files
#include <iostream>
#include <string>

// Third Party Includes
#include <Eigen/Dense>
#include <Communications/Messages/double_vec_t.hpp>

// Project Includes
#include <Realtime/RealTimeTask.hpp>

namespace Controllers
{
namespace Estimators
{
class StateEstimator : public Realtime::RealTimeTaskNode
{

public:
    enum OutputPort
    {
        STATE_HAT = 0 // State Estimate
    };

    enum InputPort
    {
        IMU = 0,    // IMU Sensor Input
        LEG_KINEMATICS = 1, // Leg Kinematics Input
        VISUAL_ODOM = 2 // Visual Odometry Sensor Input
    };

    // TODO: Move to a State class
    enum Idx
    {
        X = 0,     // X Position
        Y = 1,     // Y Position
        Z = 2,     // Z Position
        X_DOT = 3, // X Velocity
        Y_DOT = 4, // Y Velocity
        Z_DOT = 5, // Z Velocity
        PHI = 6,   // Roll
        THETA = 7, // Pitch
        PSI = 8,   // Yaw
        W_X = 9,   // Angular Vel (Roll)
        W_Y = 10,  // Angular Vel (Pitch)
        W_Z = 11,  // Angular Vel (Yaw)
        GRAVITY = 12 // Augmented Gravity
    };


    // Base Class State Estimator Task Node
    // name = Task Name
    // stack_size = Task Thread Stack Size
    // rt_priority = Task Thread Priority
    // rt_period = Task Execution Period (microseconds), default = 10000uS/100hz
    // rt_core_id = CPU Core to pin the task.  -1 for no affinity
    StateEstimator(const std::string &name = "State_Estimator_Task",
                   const long rt_period = 10000,
                   const unsigned int rt_priority = Realtime::Priority::MEDIUM,
                   const int rt_core_id = -1,
                   const unsigned int stack_size = PTHREAD_STACK_MIN);

protected:
    // Overriden Run Function
    virtual void Run();

    // Pre-Run Setup Routine.  Setup any one time initialization here.
    virtual void Setup();

    // Number of states
    unsigned int num_states_;

    // Input (State Estimate)
    double_vec_t x_hat_in_;

    // (Output) State Estimate
    double_vec_t output_state_;
};
} // namespace Estimators
} // namespace Controllers

#endif // NOMAD_CORE_CONTROLLERS_STATEESTIMATOR_H_
