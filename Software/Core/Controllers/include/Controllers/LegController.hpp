/*
 * LegController.hpp
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

#ifndef NOMAD_CORE_CONTROLLERS_LEGCONTROLLER_H_
#define NOMAD_CORE_CONTROLLERS_LEGCONTROLLER_H_

// C System Files
#include <stdint.h>

// C++ System Files
#include <iostream>
#include <string>

// Third Party Includes
#include <Eigen/Dense>
#include <Communications/Messages/double_vec_t.hpp>
#include <Communications/Messages/generic_msg_t.hpp>

#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>

// Project Includes
#include <Realtime/RealTimeTask.hpp>

namespace Controllers::Locomotion
{
    struct leg_controller_cmd_t
    {
        double torque_ff[12];        // Torque Feedforward
        double force_ff[12];         // Force Feedforward
        double q[12];                // Joint Position
        double q_d[12];              // Joint Velocity
        double foot_pos_desired[12]; // Foot Position Desired
        double foot_vel_desired[12]; // Foot Velocity Desired
        double foot_pos[12];         // Foot Position Actual
        double foot_vel[12];         // Foot Velocity Actual
        double k_p_joint[12];        // P Gains (Joint)
        double k_d_joint[12];        // D Gains (Joint)
        double k_p_cartesian[12];    // P Gains (Cartesian)
        double k_d_cartesian[12];    // D Gains (Cartesian)
    };

    class LegController : public Realtime::RealTimeTaskNode
    {

    public:
        // TODO: Evaluate breaking these down into single message type, LegControllerSetpoint_t, LegControllerState_t etc
        enum OutputPort
        {
            SERVO_COMMAND = 0,
            NUM_OUTPUTS = 1
        };

        enum InputPort
        {
            LEG_COMMAND = 0,
            NUM_INPUTS = 1
        };
        // FL = 0, FR = 1, RL = 2, RR = 3

        // Base Class Leg ControllerTask Node
        // name = Task Name
        // stack_size = Task Thread Stack Size
        // rt_priority = Task Thread Priority
        // rt_period = Task Execution Period (microseconds), default = 10000uS/100hz
        // rt_core_id = CPU Core to pin the task.  -1 for no affinity
        LegController(const std::string &name = "Leg_Controller_Task",
                      const long rt_period = 10000,
                      const unsigned int rt_priority = Realtime::Priority::HIGH,
                      const int rt_core_id = -1,
                      const unsigned int stack_size = PTHREAD_STACK_MIN);

        // Set Dart Robot Skeleton
        void SetRobotSkeleton(dart::dynamics::SkeletonPtr robot);

    protected:
        // Overriden Run Function
        virtual void Run();

        // Pre-Run Setup Routine.  Setup any one time initialization here.
        virtual void Setup();

        // Number of legs
        unsigned int num_legs_;

        // Number of dofs per each leg
        unsigned int num_dofs_;

        // Total number of dofs
        unsigned int total_dofs_;

        // Input (Messages)
        generic_msg_t leg_command_input_;

        // Output (Messages)
        generic_msg_t servo_command_output_;

        // TODO: Set these at compile time?
        Eigen::VectorXd torque_ff_out_;
        Eigen::VectorXd q_out_;
        Eigen::VectorXd q_d_out_;

        Eigen::MatrixXd k_P_joint_;
        Eigen::MatrixXd k_D_joint_;

        Eigen::MatrixXd k_P_cartesian_;
        Eigen::MatrixXd k_D_cartesian_;

        Eigen::VectorXd foot_pos_;
        Eigen::VectorXd foot_vel_;

        // Jacobian Matrix
        Eigen::MatrixXd J_;

        // Dart Helpers
        dart::dynamics::SkeletonPtr robot_;

        void ResetState();
    };
} // namespace Controllers::Locomotion

#endif // NOMAD_CORE_CONTROLLERS_STATEESTIMATOR_H_
