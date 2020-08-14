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

#ifndef ROBOT_NOMAD_STANDCONTROLLER_H_
#define ROBOT_NOMAD_STANDCONTROLLER_H_

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
#include <Nomad/MessageTypes/sim_data_t.hpp>
#include <Nomad/MessageTypes/joint_control_cmd_t.hpp>

namespace Robot::Nomad::Interface
{
    class StandController : public Realtime::RealTimeTaskNode
    {

    public:
        enum OutputPort
        {
            JOINT_CONTROL_CMD_OUT = 0, // Control Message for Servo -> Sim
            NUM_OUTPUTS = 1
        };

        enum InputPort
        {
            SIM_DATA = 0,         // SIM Data <- Sim
            NUM_INPUTS = 1
        };

        // Plant Simulation Task Node
        // name = Task Name
        // stack_size = Task Thread Stack Size
        // rt_priority = Task Thread Priority
        // rt_period = Task Execution Period (microseconds), default = 10000uS/100hz
        // rt_core_id = CPU Core to pin the task.  -1 for no affinity
        StandController(const std::string &name = "Nomad_Stand_Controller",
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
        imu_data_t imu_data_;
        com_state_t com_state_;
        joint_state_t joint_state_;
        sim_data_t sim_data_;
        joint_control_cmd_t joint_command_;
    };
} // namespace Robot::Nomad::Interface

#endif // ROBOT_NOMAD_SIMULATIONINTERFACE_H_
