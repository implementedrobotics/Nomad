/*
 * NomadDynamics.hpp
 *
 *  Created on: Jule 2, 2020
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

#ifndef ROBOT_NOMAD_NOMADDYNAMICS_H_
#define ROBOT_NOMAD_NOMADDYNAMICS_H_

// C System Files
#include <stdint.h>

// C++ System Files
#include <iostream>
#include <string>

// Third Party Includes
#include <Eigen/Dense>
#include <Communications/Messages/double_vec_t.hpp>
#include <Communications/Messages/int32_vec_t.hpp>
#include <Communications/Messages/generic_msg_t.hpp>

// Project Includes
#include <Realtime/RealTimeTask.hpp>
#include <Controllers/LegController.hpp>

namespace Robot
{
    namespace Nomad
    {
        namespace Dynamics
        {
            class NomadDynamics : public Realtime::RealTimeTaskNode
            {
                struct nomad_full_state_t
                {
                    double q[18];             // Robot Position State
                    double q_dot[18];         // Robot Velocity State
                    double foot_pos[12];      // Foot Position State
                    double foot_vel[12];      // Foot Velocity State

                    double M[18*18];          // Mass Inertia Matrix
                    double g[18];             // Gravity Terms Vector
                    double b[18];             // Coriolis Terms Vector
                    double J_c[12*18];         // Contact Jacobian
                };

            public:
                enum OutputPort
                {
                    FULL_STATE = 0,
                    NUM_OUTPUTS = 1
                };

                enum InputPort
                {
                    LEG_STATE = 0,
                    BODY_STATE = 1,
                    NUM_INPUTS = 2
                };

                // Base Class State Estimator Task Node
                // name = Task Name
                // stack_size = Task Thread Stack Size
                // rt_priority = Task Thread Priority
                // rt_period = Task Execution Period (microseconds), default = 10000uS/100hz
                // rt_core_id = CPU Core to pin the task.  -1 for no affinity
                NomadDynamics(const std::string &name = "Nomad_Dynamics_Handler",
                             const long rt_period = 10000,
                             const unsigned int rt_priority = Realtime::Priority::MEDIUM,
                             const int rt_core_id = -1,
                             const unsigned int stack_size = PTHREAD_STACK_MIN);

            protected:
                // Overriden Run Function
                virtual void Run();

                // Pre-Run Setup Routine.  Setup any one time initialization here.
                virtual void Setup();

                // Input
                generic_msg_t leg_state_msg_;
                ::Controllers::Locomotion::leg_controller_cmd_t leg_state_;

                // Output
                generic_msg_t nomad_full_state_msg;
                nomad_full_state_t full_state_;

            };
        } // namespace Dynamics
    }     // namespace Nomad
} // namespace Robot

#endif // ROBOT_NOMAD_NOMADDYNAMICS_H_
