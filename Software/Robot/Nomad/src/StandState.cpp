
/*
 * StandState.cpp
 *
 *  Created on: June 27, 2020
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

// C System Files

// C++ System Files
#include <iostream>

// Third Party Includes

// Project Include Files
#include <Nomad/FSM/StandState.hpp>
#include <Nomad/NomadControl.hpp>
#include <Controllers/Messages/leg_controller_cmd_t.hpp>


using Robot::Nomad::Controllers::NomadControl;

namespace Robot::Nomad::FSM
{
    double stance_height = .35; // TODO: From Parameter/ControlData
    double stance_time = 1.0;
    StandState::StandState() : NomadState("STAND", 2)
    {
    }
    void StandState::Run_(double dt)
    {
        //std::cout << "Stand Running: " << elapsed_time_ << std::endl;
        
        // if (input_->Receive(nomad_state_))
        // {
        // }
        static full_state_t nomad_state_;
        GetInputPort(NomadControl::InputPort::FULL_STATE)->Receive(nomad_state_);

        // Zero out leg command
        leg_controller_cmd_t leg_command;
        memset(&leg_command, 0, sizeof(leg_controller_cmd_t));

        for (int leg_id = 0; leg_id < Robot::Nomad::NUM_LEGS; leg_id++)
        {
            double h_t = stand_traj_[leg_id].Position(elapsed_time_);
            double a_t = stand_traj_[leg_id].Acceleration(elapsed_time_);

            int foot_id = leg_id * 3;


            // Copy Initial
            Eigen::Vector3d foot_pos = Eigen::Map<Eigen::Vector3d>(&nomad_state_.foot_pos[foot_id]);
            Eigen::Vector3d foot_pos_desired = Eigen::Map<Eigen::Vector3d>(&nomad_state_initial_.foot_pos[foot_id]);
            foot_pos_desired.z() = h_t;

            //std::cout << "HT" << foot_pos_desired << std::endl;

            Eigen::Map<Eigen::VectorXd>(&leg_command.foot_pos_desired[foot_id], 3) = foot_pos_desired;
            Eigen::Map<Eigen::VectorXd>(&leg_command.foot_pos[foot_id], 3) = foot_pos;

            //std::cout << leg_command.foot_pos_desired[2] << std::endl;

            // F = ma
            // Eigen::Vector3d force_ff = 8.0 / 4 * Eigen::Vector3d(0, 0, -9.81);
            // if (elapsed_time_ <= stance_time)
            // {
            //     //std::cout << g_Controller->Skeleton()->getMass() << " : " << std::endl;
            //     force_ff += 8.0 / 4 * Eigen::Vector3d(0, 0, -a_t);
            // }
            // Eigen::Map<Eigen::VectorXd>(&leg_command.force_ff [foot_id], 3) = force_ff;

            //std::cout << "Got: " << leg_command.foot_pos_desired[foot_id] << std::endl;
            //std::cout << "Got2: " << nomad_state_initial_.foot_pos[leg_id * 3+2] << std::endl;
        }

        Eigen::Map<Eigen::VectorXd>(leg_command.k_p_cartesian, 12) = Eigen::VectorXd::Ones(12) * 850;
        Eigen::Map<Eigen::VectorXd>(leg_command.k_d_cartesian, 12) = Eigen::VectorXd::Ones(12) * 200;
        

        // Get Trimmed Jaobian
        Eigen::MatrixXd J_c = Eigen::Map<Eigen::MatrixXd>(nomad_state_.J_c, 12, 18).rightCols(12);
        Eigen::Map<Eigen::MatrixXd>(leg_command.J_c, Robot::Nomad::NUM_LEGS * 3, 12) = J_c;
        //std::cout << J_c << std::endl;

        // Output Leg Command
        GetOutputPort(NomadControl::OutputPort::LEG_COMMAND)->Send(leg_command);
       // std::cout << "Sent: " << sent << std::endl;
    }
    void StandState::Enter_(double current_time)
    {
        std::cout << "Entering Stand State!!! | " << current_time << std::endl;
        std::cout << "Stand Running: " << elapsed_time_ << std::endl;

        if(!GetInputPort(NomadControl::InputPort::FULL_STATE)->Receive(nomad_state_initial_, std::chrono::milliseconds(2000))) // Wait on input
        {
            std::cout << "COULD NOT GET INITIAL STATE" << std::endl;
        }

       // std::cout << "Got Message: " << nomad_state_initial_.foot_pos[2] << std::endl;;
        
        // Create Cubic Trajectory
        stand_traj_[Robot::Nomad::FRONT_LEFT].Generate(nomad_state_initial_.foot_pos[Robot::Nomad::FOOT_FL_Z], -stance_height, 0.0, 0.0, 0.0, stance_time);
        stand_traj_[Robot::Nomad::FRONT_RIGHT].Generate(nomad_state_initial_.foot_pos[Robot::Nomad::FOOT_FR_Z], -stance_height, 0.0, 0.0, 0.0, stance_time);
        stand_traj_[Robot::Nomad::REAR_LEFT].Generate(nomad_state_initial_.foot_pos[Robot::Nomad::FOOT_RL_Z], -stance_height, 0.0, 0.0, 0.0, stance_time);
        stand_traj_[Robot::Nomad::REAR_RIGHT].Generate(nomad_state_initial_.foot_pos[Robot::Nomad::FOOT_RR_Z], -stance_height, 0.0, 0.0, 0.0, stance_time);
    }
} // namespace Robot::Nomad::FSM
