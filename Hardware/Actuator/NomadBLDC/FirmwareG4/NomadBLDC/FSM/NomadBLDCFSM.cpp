/*
 * NomadBLDCFSM.cpp
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

// Third Party Includes

// Project Include Files
#include <FSM/NomadBLDCFSM.h>
#include <FSM/StartupState.h>
#include <FSM/CalibrationStates.h>
#include <FSM/IdleState.h>
#include <FSM/ErrorState.h>
#include <FSM/FOCState.h>
#include <Logger.h>

// Transition Events For States
NomadBLDCTransitionEvent::NomadBLDCTransitionEvent(NomadBLDCData* data)
    : TransitionEvent(), data_(data)
{
    data_ = data;
}

NomadBLDCFSM::NomadBLDCFSM() : FiniteStateMachine()
{
    // Create Data Pointer
    data_ = new NomadBLDCData();

    _CreateFSM();
}

bool NomadBLDCFSM::Run(float dt)
{
    // Now run base FSM code
    return FiniteStateMachine::Run(dt);
}

NomadBLDCData* NomadBLDCFSM::GetData() const
{
    return data_;
}

void NomadBLDCFSM::_CreateFSM()
{

    ///////////////////////// Define Our States
    StartupState *startup = new StartupState();
    startup->SetControllerData(data_);
    startup->SetParentFSM(this);

    // Measure Resistance
    MeasureResistanceState *measure_resistance = new MeasureResistanceState();
    measure_resistance->SetControllerData(data_);
    measure_resistance->SetParentFSM(this);

    // Measure Inductance
    MeasureInductanceState *measure_inductance = new MeasureInductanceState();
    measure_inductance->SetControllerData(data_);
    measure_inductance->SetParentFSM(this);

    // Measure Phase Order
    MeasurePhaseOrderState *measure_phase_order = new MeasurePhaseOrderState();
    measure_phase_order->SetControllerData(data_);
    measure_phase_order->SetParentFSM(this);

    // Measure Encoder Offset
    MeasureEncoderOffsetState *measure_encoder_offset = new MeasureEncoderOffsetState();
    measure_encoder_offset->SetControllerData(data_);
    measure_encoder_offset->SetParentFSM(this);

    // Motor Drive States
    
    // Field Oriented Control - Voltage
    FOCState *foc_mode = new FOCState();
    foc_mode->SetControllerData(data_);
    foc_mode->SetParentFSM(this);

    // Idle
    IdleState *idle = new IdleState();
    idle->SetControllerData(data_);
    idle->SetParentFSM(this);

    // Error
    ErrorState *error = new ErrorState();
    error->SetControllerData(data_);
    error->SetParentFSM(this);

    ///////////////////////// Define Our Transitions
    CommandModeEvent *transition_idle = new CommandModeEvent(control_mode_type_t::IDLE_MODE, data_);
    CommandModeEvent *transition_measure_resistance = new CommandModeEvent(control_mode_type_t::MEASURE_RESISTANCE_MODE, data_);
    CommandModeEvent *transition_measure_inductance = new CommandModeEvent(control_mode_type_t::MEASURE_INDUCTANCE_MODE, data_);
    CommandModeEvent *transition_measure_phase_order = new CommandModeEvent(control_mode_type_t::MEASURE_PHASE_ORDER_MODE, data_);
    CommandModeEvent *transition_measure_encoder_offset = new CommandModeEvent(control_mode_type_t::MEASURE_ENCODER_OFFSET_MODE, data_);
    CommandModeEvent *transition_foc_position = new CommandModeEvent(control_mode_type_t::POSITION_MODE, data_);
    CommandModeEvent *transition_foc_velocity = new CommandModeEvent(control_mode_type_t::VELOCITY_MODE, data_);
    CommandModeEvent *transition_foc_pd_mode = new CommandModeEvent(control_mode_type_t::PD_MODE, data_);
    CommandModeEvent *transition_foc_torque = new CommandModeEvent(control_mode_type_t::TORQUE_MODE, data_);
    CommandModeEvent *transition_foc_current = new CommandModeEvent(control_mode_type_t::CURRENT_MODE, data_);
    CommandModeEvent *transition_foc_voltage = new CommandModeEvent(control_mode_type_t::VOLTAGE_MODE, data_);

    // Error Transitions
    FaultModeEvent *transition_error = new FaultModeEvent(data_);

    /////////////////////////  Setup Transitions
    startup->AddTransitionEvent(transition_idle, idle);
    measure_resistance->AddTransitionEvent(transition_idle, idle);
    measure_inductance->AddTransitionEvent(transition_idle, idle);
    measure_phase_order->AddTransitionEvent(transition_idle, idle);
    measure_encoder_offset->AddTransitionEvent(transition_idle, idle);
    foc_mode->AddTransitionEvent(transition_idle, idle);
    foc_mode->AddTransitionEvent(transition_error, error);

    // Enter Measure Resistance Transition
    //idle->AddTransitionEvent(transition_error, error);
    idle->AddTransitionEvent(transition_measure_resistance, measure_resistance);

    // Enter Measure Inductance Transition
    idle->AddTransitionEvent(transition_measure_inductance, measure_inductance);

    // Enter Measure Phase Order Transition
    idle->AddTransitionEvent(transition_measure_phase_order, measure_phase_order);

    // Enter Measure Encoder Offset Transition
    idle->AddTransitionEvent(transition_measure_encoder_offset, measure_encoder_offset);

    // Enter Position Control Mode Transition
    idle->AddTransitionEvent(transition_foc_position, foc_mode);
    
    // Enter Velocity Control Mode Transition
    idle->AddTransitionEvent(transition_foc_velocity, foc_mode);
    
    // Enter PD Control Mode Transition
    idle->AddTransitionEvent(transition_foc_pd_mode, foc_mode);

    // Enter Torque Control Mode Transition
    idle->AddTransitionEvent(transition_foc_torque, foc_mode);
    
    // Enter Current Mode Transition
    idle->AddTransitionEvent(transition_foc_current, foc_mode);
    
    // Enter Voltage Mode Transition
    idle->AddTransitionEvent(transition_foc_voltage, foc_mode);

    // Add the states to the FSM
    AddState(startup);
    AddState(idle);
    AddState(measure_resistance);
    AddState(measure_inductance);
    AddState(measure_phase_order);
    AddState(measure_encoder_offset);
    AddState(foc_mode);
    AddState(error);
    
    // Set Initials State
    SetInitialState(startup);
}
