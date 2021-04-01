/*
 * FOCState.cpp
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

// C System Files

// C++ System Files

// Third Party Includes

// Project Include Files
#include <Logger.h>
#include <FSM/FOCState.h>
#include <Utilities/math.h>

FOCState::FOCState() : NomadBLDCState(NomadBLDCStateID::STATE_FOC)
{
}

void FOCState::Run_(float dt)
{
    // Enter Critical Section
    __disable_irq();

    // DWT->CYCCNT = 0;
    // Get Motor Ref
    Motor *motor = data_->controller->GetMotor();
    MotorController *controller = data_->controller;

    float position_error = controller->state_.Pos_ref - motor->state_.theta_mech;
    float control_torque = 0.0f;
    float control_vel = controller->state_.Vel_ref;
    float velocity_error = 0.0f;

   // Logger::Instance().Print("TORQUE NOW: %f\r\n", control_torque);

    // Integrating Position/Velocity Controller Branch
    switch (data_->controller->GetControlMode())
    {
        case (control_mode_type_t::POSITION_MODE):
        {
            control_vel += controller->state_.K_p * position_error;
        }
        case (control_mode_type_t::VELOCITY_MODE):
        {
            // TODO: Clamp velocity
            //control_vel = std::clamp(control_vel, -controller->config_.velocity_limit, controller->config_.velocity_limit);
            velocity_error = control_vel - motor->state_.theta_mech_dot;
            control_torque = controller->state_.K_d * velocity_error;
            control_torque += controller->state_.Vel_int;

           // Logger::Instance().Print("KD: %f\r\n", controller->state_.K_d);
           // Logger::Instance().Print("KP: %f\r\n", controller->state_.K_p);
           // Logger::Instance().Print("CT: %f/%f|%f\r\n", control_torque, controller->state_.Vel_int, velocity_error*controller->state_.K_d*0 );
        }
        default:
            break;
    }

    // PD Control Branch
    switch (data_->controller->GetControlMode())
    {
        case (control_mode_type_t::PD_MODE):
        {
            velocity_error = control_vel - motor->state_.theta_mech_dot;
            control_torque = controller->state_.K_p * (position_error) + controller->state_.K_d * (velocity_error);
        }
        case (control_mode_type_t::POSITION_MODE): // Handle Feedforwards from first stage -> Torque
        case (control_mode_type_t::VELOCITY_MODE):
        case (control_mode_type_t::TORQUE_MODE):
        {
            //Logger::Instance().Print("TORQUE MODE!\r\n");
            control_torque += controller->state_.T_ff;
         
            // Check joint soft stop limits here
            // Check Position Limits
            if (motor->state_.theta_mech <= controller->config_.pos_limit_min)
            {
                control_torque = controller->config_.K_p_limit * (controller->config_.pos_limit_min - motor->state_.theta_mech) + controller->config_.K_d_limit * (0.0f - motor->state_.theta_mech_dot);
                //in_limit_min_ = true;
            }
            else if (motor->state_.theta_mech >= controller->config_.pos_limit_max)
            {
                control_torque = controller->config_.K_p_limit * (controller->config_.pos_limit_max - motor->state_.theta_mech) + controller->config_.K_d_limit * (0.0f - motor->state_.theta_mech_dot);
                //in_limit_max_ = true;
            }

            // Clamp Torques to Limit
            control_torque = std::clamp(control_torque, -controller->config_.torque_limit, controller->config_.torque_limit);
            controller->state_.I_q_ref = control_torque / (motor->config_.K_t * motor->config_.gear_ratio);
            controller->state_.I_d_ref = 0.0f;


        }
        case (control_mode_type_t::CURRENT_MODE):
        {
            //Logger::Instance().Print("TORQUE: %f|%f\r\n", control_torque, position_error);
            controller->CurrentControl();
            break;
        }
        default:
            break;
    }

    // Update Integrator
    if(data_->controller->GetControlMode() == control_mode_type_t::POSITION_MODE
    || data_->controller->GetControlMode() == control_mode_type_t::VELOCITY_MODE)
    {
        controller->state_.Vel_int += (controller->config_.k_i_vel * dt) * velocity_error;
    }

    // if(in_limit_max_) // Check Hysteresis
    // {
    //     if(motor->state_.theta_mech < config_.pos_limit_max-deadband)// || torque_ref_in < 0)
    //         in_limit_max_ = false;
    // }
    // else if(in_limit_min_) // Check Hysteresis
    // {
    //     if(motor->state_.theta_mech > config_.pos_limit_min+deadband)// || torque_ref_in > 0)
    //         in_limit_min_ = false;
    // }


        // case (control_mode_type_t::FOC_VOLTAGE_MODE):
        // {
        //     controller->state_.V_d = controller->state_.V_d_ref;
        //     controller->state_.V_q = controller->state_.V_q_ref;

        //     // TODO: Voltage Limit for I_rms.  V_max = I_max * Phase_R -> Limit Norm/Modulus
        //     controller->SetModulationOutput(motor->state_.theta_elec, controller->state_.V_d_ref, controller->state_.V_q_ref);

        //     // Update V_d/V_q
        //     controller->state_.V_d = controller->state_.I_d * motor->config_.phase_resistance;
        //     controller->state_.V_q = controller->state_.I_q * motor->config_.phase_resistance;
        //     break;
        // }

     //   default:
       //     break;
  //  }
    // uint32_t span = DWT->CYCCNT;

    // Exit Critical
    __enable_irq();
}

void FOCState::Enter_(uint32_t current_time)
{
    // Check Motor Calibribration
    if (!data_->controller->GetMotor()->config_.calibrated)
    {
        Logger::Instance().Print("[FOCState]: ERROR: Motor NOT Calibrated!\r\nPlease calibrate and save a valid configuration.\r\n");
        data_->controller->SetControlMode(control_mode_type_t::ERROR_MODE);
        return;
    }

    float output_pos = data_->controller->GetMotor()->state_.theta_mech;
    if (output_pos <= data_->controller->config_.pos_limit_min || output_pos >= data_->controller->config_.pos_limit_max)
    {
        Logger::Instance().Print("[FOCState]: ERROR: Motor Position Limit Triggered!. Please rezero output.\r\n");
        data_->controller->SetControlMode(control_mode_type_t::ERROR_MODE);
        return;
    }

    // Turn Status LED On
    LEDService::Instance().On();

    // Enable Gate Driver
    data_->controller->GetGateDriver()->EnableDriver();

    // Turn On PWM Outputs
    data_->controller->EnablePWM(true);

    // Set Idle/"Zero" PWM
    data_->controller->SetDuty(0.5f, 0.5f, 0.5f);

    // Reset Controller Setpoints/Data
    data_->controller->Reset();
}

void FOCState::Exit_(uint32_t current_time)
{
    // Enable Gate Driver
    data_->controller->GetGateDriver()->DisableDriver();

    // Turn On PWM Outputs
    data_->controller->EnablePWM(false);

    // Set Idle/"Zero" PWM
    data_->controller->SetDuty(0.5f, 0.5f, 0.5f);

    // Reset Controller Setpoints/Data
    data_->controller->Reset();
}
