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

// Project Includes
#include <Realtime/RealTimeTask.hpp>
#include <Nomad/MessageTypes/imu_data_t.hpp>
#include <Nomad/MessageTypes/com_state_t.hpp>
#include <Nomad/MessageTypes/joint_state_t.hpp>
#include <Nomad/MessageTypes/joint_control_cmd_t.hpp>
#include <Nomad/MessageTypes/sim_data_t.hpp>

namespace Robot::Nomad::Interface
{
    class SimulationInterface : public Realtime::RealTimeTaskNode
    {

    public:
        enum OutputPort
        {
            JOINT_STATE = 0, // Joint State Estimate from Plant
            IMU_STATE = 1,   // IMU State from Sensors
            COM_STATE = 2,   // Ground Truth Body Pose State from Sim
            JOINT_CONTROL_CMD_OUT = 3, // Control Message for Servo -> Sim
            NUM_OUTPUTS = 4
        };

        enum InputPort
        {
            JOINT_CONTROL_CMD_IN = 0, // Control Message for Servo -> Sim
            SIM_DATA = 1,             // SIM Data State <- Sim
            NUM_INPUTS = 2
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

        // Messages
        sim_data_t sim_data_;
        imu_data_t imu_data_;
        com_state_t com_state_;
        joint_state_t joint_state_;
        joint_control_cmd_t joint_command_;
    };
} // namespace Robot::Nomad::Interface

#endif // ROBOT_NOMAD_SIMULATIONINTERFACE_H_
