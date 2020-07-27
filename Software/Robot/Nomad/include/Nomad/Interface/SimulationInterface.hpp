/*
 * SimulationInterface.hpp
 *
 *  Created on: July 16, 2020
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

#ifndef ROBOT_NOMAD_SIMULATIONINTERFACE_H_
#define ROBOT_NOMAD_SIMULATIONINTERFACE_H_

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

namespace Robot
{
    namespace Nomad
    {
        namespace Interface
        {
            struct joint_control_cmd_t
            {
                double torque_ff[12]; // Torque Feedforward
                double q[12];         // Joint Position
                double q_d[12];       // Joint Velocity
                double k_p_joint[12]; // P Gains (Joint)
                double k_d_joint[12]; // D Gains (Joint)
            };

            struct joint_state_t
            {
                double q_hat[12];      // Joint Position Estimate
                double q_dot_hat[12];  // Joint Velocity Estimate
                double torque_hat[12]; // Torque Estimate
            };

            struct imu_state_t
            {
                double orientation[4]; // Orientation Quaternion
                double accel[3];       // Linear Acceleration
                double omega[3];       // Angular Velocity
            };


            class SimulationInterface : public Realtime::RealTimeTaskNode
            {

            public:
                enum OutputPort
                {
                    JOINT_STATE = 0,   // Joint State Estimate from Plant
                    IMU_STATE = 1,     // IMU State from Sensors
                    GROUND_TRUTH = 2,  // Ground Truth Plant State from Sim
                    NUM_OUTPUTS = 2
                };

                enum InputPort
                {
                    JOINT_CONTROL = 0, // Control Message for Servo
                    IMU_READ = 1,
                    JOINT_STATE_READ = 2,
                    NUM_INPUTS = 3
                };

                // Plant Simulation Task Node
                // name = Task Name
                // stack_size = Task Thread Stack Size
                // rt_priority = Task Thread Priority
                // rt_period = Task Execution Period (microseconds), default = 10000uS/100hz
                // rt_core_id = CPU Core to pin the task.  -1 for no affinity
                SimulationInterface(const std::string &name = "Nomad_Simulation_Interface",
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
                generic_msg_t joint_control_msg_;
                double_vec_t imu_read_msg_;
                double_vec_t joint_state_read_msg_;

                // Output
                generic_msg_t joint_state_msg_;
                generic_msg_t imu_state_msg_;
                generic_msg_t ground_truth_msg_;

                joint_state_t joint_state_;
                imu_state_t imu_state_;
            };
        } // namespace Interface
    }     // namespace Nomad
} // namespace Robot

#endif // ROBOT_NOMAD_SIMULATIONINTERFACE_H_
