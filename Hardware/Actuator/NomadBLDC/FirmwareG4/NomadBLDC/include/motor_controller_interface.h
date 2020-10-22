/*
 * MotorControllerInterface.h
 *
 *  Created on: August 25, 2019
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
 * 
 */

#ifndef CORE_MOTOR_CONTROLLER_INTERFACE_H_
#define CORE_MOTOR_CONTROLLER_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

// Entry point to facilitate transition to C++ for RTOS Task
void init_motor_controller();
void ms_poll_task(void *arg);//  __attribute__((section(".ccmram")));

// void debug_thread_entry();

// Callbacks
void current_measurement_cb();

// UI Callbacks
bool measure_motor_resistance();
bool measure_motor_inductance();
bool measure_motor_phase_order();
bool measure_encoder_offset();

bool measure_motor_parameters();
bool save_configuration();
void load_configuration();
void restore_configuration();
void reboot_system();
void start_torque_control();
void start_current_control();
void start_speed_control();
void start_voltage_control();
void enter_idle();
void zero_encoder_offset();

//
void set_control_mode(int mode);
void set_controller_debug(bool debug);
bool get_controller_debug();
void set_torque_control_ref(float K_p, float K_d, float Pos_des, float Vel_des, float T_ff);
void set_current_control_ref(float I_d, float I_q);
void set_voltage_control_ref(float V_d, float V_q);

#ifdef __cplusplus
}
#endif

#endif // CORE_MOTOR_CONTROLLER_INTERFACE_H_
