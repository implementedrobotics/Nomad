/*
 * MotorController.cpp
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

// Primary Include
#include "MotorController.h"

// C System Files

// C++ System Files
#include <algorithm>

// Project Includes
#include "Motor.h"
#include "nomad_hw.h"

#include <Peripherals/thermistor.h>
#include <Peripherals/flash.h>
#include <Peripherals/cordic.h>
#include <Peripherals/adc.h>
#include <Peripherals/thermistor.h>
#include <RegisterInterface.h>

#include <Utilities/utils.h>
#include <Utilities/lpf.h>

#include <FSM/NomadBLDCFSM.h>
#include "LEDService.h"
#include "Logger.h"

#define FLASH_VERSION 2

Motor *motor = 0;
MotorController *motor_controller = 0;

// TODO: All the below needs to go
Cordic cordic;

extern "C"
{
#include "motor_controller_interface.h"
}

// Flash Save Struct.  TODO: Move to own file
#define FLASH_SAVE_SIGNATURE 0x78D5FC00

struct __attribute__((__aligned__(8))) Save_format_t
{
    uint32_t signature;
    uint32_t version;
    Motor::Config_t motor_config;
    uint8_t motor_reserved[128]; // Reserved;
    PositionSensorAS5x47::Config_t position_sensor_config;
    uint8_t position_reserved[128]; // Reserved;
    MotorController::Config_t controller_config;
    uint8_t controller_reserved[128]; // Reserved;
    //FDCANDevice::Config_t can_config;
    //uint8_t can_reserved[128]; // Reserved;
};

void ms_poll_task(void *arg)
{
    // Main millisecond polling loop
    for (;;)
    {
        // Sample bus voltage
        motor_controller->SampleBusVoltage();

        // Sample FET Thermistor for Temperature
        motor_controller->SampleFETTemperature();

        // Delay 1 ms
        osDelay(1000);
    }
}

void init_motor_controller()
{
    Logger::Instance().Print("Motor RT Controller Task Up.\r\n");

    // Init CORDIC Routines
    //Cordic::Instance().Init();
    // TODO: Will have to move this
    cordic.Init();
    cordic.SetPrecision(LL_CORDIC_PRECISION_6CYCLES);

    // Init Motor and Implicitly Position Sensor
    motor = new Motor(0.000025f, 80, 20);

    // Init Motor Controller
    motor_controller = new MotorController(motor);

    // Load Config Here...
    load_configuration();

    motor_controller->Init();

    //Update Sample Time For Motor
    motor->SetSampleTime(motor_controller->GetControlUpdatePeriod());


    // motor_controller->PrintConfig();
    // motor->PrintConfig();
    // motor->PositionSensor()->PrintConfig();

    // //save_configuration();
}

// Controller Mode Interface
void set_control_mode(int mode)
{
    motor_controller->SetControlMode((control_mode_type_t)mode);
}

void set_torque_control_ref(float K_p, float K_d, float Pos_des, float Vel_des, float T_ff)
{
    motor_controller->state_.K_p = K_p;
    motor_controller->state_.K_d = K_d;
    motor_controller->state_.Pos_ref = Pos_des;
    motor_controller->state_.Vel_ref = Vel_des;
    motor_controller->state_.T_ff = T_ff;
}
void set_current_control_ref(float I_d, float I_q)
{
    motor_controller->state_.I_d_ref = I_d;
    motor_controller->state_.I_q_ref = I_q;
}
void set_voltage_control_ref(float V_d, float V_q)
{
    motor_controller->state_.V_d_ref = V_d;
    motor_controller->state_.V_q_ref = V_q;
}

// Menu Callbacks
bool measure_motor_parameters()
{
    set_control_mode(CALIBRATION_MODE); // Put in calibration mode
    return true;
}
bool measure_motor_resistance()
{
    set_control_mode(MEASURE_RESISTANCE_MODE);
    return true;
}
bool measure_motor_inductance()
{
    set_control_mode(MEASURE_INDUCTANCE_MODE);
    return true;
}
bool measure_motor_phase_order()
{
    set_control_mode(MEASURE_PHASE_ORDER_MODE);
    return true;
}
bool measure_encoder_offset()
{
    set_control_mode(MEASURE_ENCODER_OFFSET_MODE);
    return true;
}
bool save_configuration()
{
    Logger::Instance().Print("\r\nSaving Configuration...\r\n");

    bool status = false;
    Save_format_t save;
    save.signature = FLASH_SAVE_SIGNATURE;
    save.version = FLASH_VERSION; // Set Version

    // If we are writing a config assume for now we are calibrated
    // TODO: Do something better so we don't have to make this assumption
    motor->config_.calibrated = 1;
    save.motor_config = motor->config_;
    save.position_sensor_config = motor->PositionSensor()->config_;
    save.controller_config = motor_controller->config_;

    // Write Flash
    FlashDevice::Instance().Open(ADDR_FLASH_PAGE_60, sizeof(save), FlashDevice::WRITE);
    status = FlashDevice::Instance().Write(0, (uint8_t *)&save, sizeof(save));
    FlashDevice::Instance().Close();

    Logger::Instance().Print("\r\nSaved Configuration: %d\r\n",status);

    return status;
}
void load_configuration()
{
    Save_format_t load;

    FlashDevice::Instance().Open(ADDR_FLASH_PAGE_60, sizeof(load), FlashDevice::READ);
    bool status = FlashDevice::Instance().Read(0, (uint8_t *)&load, sizeof(load));
    FlashDevice::Instance().Close();

    if (load.signature != FLASH_SAVE_SIGNATURE || load.version != FLASH_VERSION)
    {
        Logger::Instance().Print("ERROR: No Valid Configuration Found!  Please run setup before enabling drive: %d\r\n", status);
        return;
    }

    motor->config_ = load.motor_config;
    motor->PositionSensor()->config_ = load.position_sensor_config;
    motor->PositionSensor()->SetPolePairs(motor->config_.num_pole_pairs);
    motor_controller->config_ = load.controller_config;

    motor->ZeroOutputPosition();
}
void reboot_system()
{
    NVIC_SystemReset();
}

void start_torque_control()
{
    set_control_mode(FOC_TORQUE_MODE);
}

void start_current_control()
{
    set_control_mode(FOC_CURRENT_MODE);
}

void start_speed_control()
{
    set_control_mode(FOC_SPEED_MODE);
}

void start_voltage_control()
{
    set_control_mode(FOC_VOLTAGE_MODE);
}

void enter_idle()
{
    set_control_mode(IDLE_MODE);
}

void zero_encoder_offset()
{
    motor->ZeroOutputPosition();
}
// Statics
MotorController *MotorController::singleton_ = nullptr;

MotorController::MotorController(Motor *motor) : motor_(motor)
{
    control_initialized_ = false;
    control_enabled_ = false;

    // Null FSM
    control_fsm_ = nullptr;

    // Zero State
    memset(&state_, 0, sizeof(state_));

    // Defaults
    config_.k_d = 0.0f;
    config_.k_q = 0.0f;
    config_.k_i_d = 0.0f;
    config_.k_i_q = 0.0f;
    //config_.alpha = 0.186350f;
    config_.overmodulation = 1.0f;
    config_.pos_limit_min = -12.5f;
    config_.pos_limit_max = 12.5f;
    config_.velocity_limit = 10.0f; // +/-
    config_.torque_limit = 10.0f; // +/-
    config_.current_limit = 20.0f;  // +/-
    config_.current_bandwidth = 1000.0f;

    //config_.K_p_min = 0.0f;
    config_.K_p_max = 500.0f;
    //config_.K_d_min = 0.0f;
    config_.K_d_max = 5.0f; 

    config_.K_p_limit = 1.0f;
    config_.K_i_limit = 0.0f;
    config_.K_d_limit = 0.0f;


    config_.pwm_freq = 40000.0f; // 40 khz
    config_.foc_ccl_divider = 1; // Default to not divide.  Current loops runs at same freq as PWM

    state_.Voltage_bus = 24.0f;

    // TOOD: In Register
    in_limit_max_ = false;
    in_limit_min_ = false;
    // TODO: Parameter
    rms_current_sample_period_ = 1.0f/10.0f;
    controller_loop_freq_ = (config_.pwm_freq / config_.foc_ccl_divider);
    controller_update_period_ = (1.0f) / controller_loop_freq_;

    // Setup Registers
    RegisterInterface::AddRegister(ControllerConfigRegisters_e::ControllerConfigRegister1, new Register((ControllerConfigRegister1_t *)&config_, true));
    RegisterInterface::AddRegister(ControllerConfigRegisters_e::K_LOOP_D, new Register(&config_.k_d));
    RegisterInterface::AddRegister(ControllerConfigRegisters_e::K_LOOP_Q, new Register(&config_.k_q));
    RegisterInterface::AddRegister(ControllerConfigRegisters_e::K_I_D, new Register(&config_.k_i_d));
    RegisterInterface::AddRegister(ControllerConfigRegisters_e::K_I_Q, new Register(&config_.k_i_q));
    RegisterInterface::AddRegister(ControllerConfigRegisters_e::CurrentBandwidth, new Register(&config_.current_bandwidth));
    RegisterInterface::AddRegister(ControllerConfigRegisters_e::Overmodulation, new Register(&config_.overmodulation));
    RegisterInterface::AddRegister(ControllerConfigRegisters_e::PWM_Frequency, new Register(&config_.pwm_freq));
    RegisterInterface::AddRegister(ControllerConfigRegisters_e::FOC_Divider, new Register(&config_.foc_ccl_divider));

    RegisterInterface::AddRegister(ControllerConfigRegisters_e::ControllerConfigRegister2, new Register((ControllerConfigRegister2_t *)&config_.K_p_max, true));
    RegisterInterface::AddRegister(ControllerConfigRegisters_e::K_P_Max, new Register(&config_.K_p_max));
    RegisterInterface::AddRegister(ControllerConfigRegisters_e::K_D_Max, new Register(&config_.K_d_max));
    RegisterInterface::AddRegister(ControllerConfigRegisters_e::PositionLimitMin, new Register(&config_.pos_limit_min));
    RegisterInterface::AddRegister(ControllerConfigRegisters_e::PositionLimitMax, new Register(&config_.pos_limit_max));
    RegisterInterface::AddRegister(ControllerConfigRegisters_e::VelocityLimit, new Register(&config_.velocity_limit));
    RegisterInterface::AddRegister(ControllerConfigRegisters_e::TorqueLimit, new Register(&config_.torque_limit));
    RegisterInterface::AddRegister(ControllerConfigRegisters_e::CurrentLimit, new Register(&config_.current_limit));

    RegisterInterface::AddRegister(ControllerStateRegisters_e::ControllerStateRegister1, new Register((ControllerStateRegister1_t *)&state_, true));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::I_D, new Register(&state_.I_d));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::I_Q, new Register(&state_.I_q));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::V_D, new Register(&state_.V_d));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::V_Q, new Register(&state_.V_q));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::IntegratorError_D, new Register(&state_.d_int));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::IntegratorError_Q, new Register(&state_.q_int));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::DutyCycleA, new Register(&state_.dtc_A));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::DutyCycleB, new Register(&state_.dtc_B));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::DutyCycleC, new Register(&state_.dtc_C));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::CurrentRMS, new Register(&state_.I_rms));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::MaxCurrent, new Register(&state_.I_max));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::Timeout, new Register(&state_.timeout));

    RegisterInterface::AddRegister(ControllerStateRegisters_e::ControllerStateRegister2, new Register((ControllerStateRegister2_t *)&state_.V_d_ref, true));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::VoltageControlModeRegister, new Register((VoltageControlModeRegister_t *)&state_.V_d_ref, true));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::V_Setpoint_D, new Register(&state_.V_d_ref));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::V_Setpoint_Q, new Register(&state_.V_q_ref));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::CurrenteControlModeRegister, new Register((CurrentControlModeRegister_t *)&state_.I_d_ref, true));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::I_Setpoint_D, new Register(&state_.I_d_ref));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::I_Setpoint_Q, new Register(&state_.I_q_ref));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::TorqueControlModeRegister, new Register((TorqueControlModeRegister_t *)&state_.Pos_ref, true));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::PositionSetpoint, new Register(&state_.Pos_ref));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::VelocitySetpoint, new Register(&state_.Vel_ref));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::K_P, new Register(&state_.K_p));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::K_D, new Register(&state_.K_d));
    RegisterInterface::AddRegister(ControllerStateRegisters_e::Torque_FF, new Register(&state_.T_ff));
}

void MotorController::PrintConfig()
{
     // Print Configs
    Logger::Instance().Print("Controller Config: K_d: %f, K_q: %f, K_i_d: %f, K_i_q: %f, overmodulation: %f\r\n", config_.k_d, config_.k_q, config_.k_i_d, config_.k_i_q, config_.overmodulation);
    Logger::Instance().Print("Controller Config: Vel_Limit: %f, Pos_Limit: %f, Tau_Limit: %f, Current_Limit: %f, Current BW: %f\r\n", config_.velocity_limit, config_.pos_limit_min, config_.torque_limit, config_.current_limit, config_.current_bandwidth);
    Logger::Instance().Print("Controller Config: K_p_max: %f, K_d_max: %f, PWM Freq: %f, FOC Divder: %d\r\n", config_.K_p_max, config_.K_d_max, config_.pwm_freq, config_.foc_ccl_divider);
}
void MotorController::Reset()
{
    SetModulationOutput(0.0f, 0.0f);

    state_.I_d = 0.0f;
    state_.I_q = 0.0f;
    state_.I_d_ref = 0.0f;
    state_.I_q_ref = 0.0f;
    state_.V_d_ref = 0.0f;
    state_.V_q_ref = 0.0f;

    state_.d_int = 0.0f;
    state_.q_int = 0.0f;

    state_.V_d = 0.0f;
    state_.V_q = 0.0f;

    state_.timeout = 0;

    state_.Pos_ref = 0.0f;
    state_.Vel_ref = 0.0f;
    state_.K_p = 0.0f;
    state_.K_d = 0.0f;
    state_.T_ff = 0.0f;

    in_limit_min_ = false;
    in_limit_max_ = false;
}

void MotorController::CurrentMeasurementCB()
{
    // Make Sure We are Fully Initialized
    if (!IsInitialized())
        return;

    // Performance Measure
    //LL_GPIO_SetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);

    // Start Position Sampling
   // motor_->PositionSensor()->StartUpdate();

    // Depends on PWM duty cycles
    if (motor_->config_.phase_order) // Check Phase Ordering
    {
        motor_->state_.I_a = current_scale * static_cast<float>(adc_2_->Read());
        motor_->state_.I_b = current_scale * static_cast<float>(adc_3_->Read());
        motor_->state_.I_c = current_scale * static_cast<float>(adc_1_->Read());
    }
    else
    {
        motor_->state_.I_a = current_scale * static_cast<float>(adc_2_->Read());
        motor_->state_.I_b = current_scale * static_cast<float>(adc_1_->Read());
        motor_->state_.I_c = current_scale * static_cast<float>(adc_3_->Read());
    }
    // We have some time to do things here.  We should squeeze in RMS current here
    
    // Update Motor/Rotor Position State
    motor_->Update();

    // Update I_d, I_q values
    dq0(motor->state_.theta_elec, motor->state_.I_a, motor->state_.I_b, motor->state_.I_c, &state_.I_d, &state_.I_q); //dq0 transform on currents
    
    // Compute Bus Current
    // TODO: This is technically from previous time step but that should be okay.
    float I_motor = Core::Math::Vector2d::Magnitude(state_.I_d, state_.I_q);
    float P_motor = I_motor * I_motor * motor_->config_.phase_resistance;
    state_.I_bus = P_motor / state_.Voltage_bus;

    // TODO: Also need to make this work for voltage mode Vrms=Irms * R should work hopefully
    // Update Current Limiter
    current_limiter_->AddCurrentSample(I_motor);
    state_.I_rms = current_limiter_->GetRMSCurrent();
    state_.I_max = current_limiter_->GetMaxAllowableCurrent();

    // Kirchoffs Current Law to compute 3rd unmeasured current.
    //motor_->state_.I_a = -motor_->state_.I_b - motor_->state_.I_c;

    // TODO: Use Longest Duty Cycle Measurements for which of the 3 phases to use

    // TODO: Move to DMA Continuous sampling + oversampling?
    // Sample bus voltage
    //SampleBusVoltage();
 
    // Finish Position Sensor Update
    //LL_GPIO_SetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);
    //motor_->PositionSensor()->EndUpdate();
    //LL_GPIO_ResetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);



    // Run FSM for timestep
    RunControlFSM();
}


void MotorController::SampleBusVoltage()
{
    if(!IsInitialized())
        return;

    // Sample Bus Voltage
    state_.Voltage_bus = static_cast<float>(vbus_adc_->Sample()) * voltage_scale;
}

void MotorController::SampleFETTemperature()
{
    if(!IsInitialized())
        return;
    // Sample FET Thermistor for Temperature
    state_.fet_temp = fet_therm_->SampleTemperature();
}

void MotorController::Init()
{
    Logger::Instance().Print("MotorController::Init() - Motor Controller Initializing...\r\n");

    // Compute Maximum Allowed Current
    float margin = 1.0f;
    float max_input = margin * 0.3f * SENSE_CONDUCTANCE;
    float max_swing = margin * 1.6f * SENSE_CONDUCTANCE * (1.0f / CURRENT_SENSE_GAIN);
    current_max_ = std::min(max_input, max_swing);

    // TODO: Make sure this is in a valid range?

    // Setup DRV Pins
    GPIO_t mosi = {DRV_MOSI_GPIO_Port, DRV_MOSI_Pin};
    GPIO_t miso = {DRV_MISO_GPIO_Port, DRV_MISO_Pin};
    GPIO_t nss = {DRV_CS_GPIO_Port, DRV_CS_Pin};

    GPIO_t enable = {DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin};
    GPIO_t n_fault = {DRV_nFAULT_GPIO_Port, DRV_nFAULT_Pin};

    // Setup Gate Driver
    spi_handle_ = new SPIDevice(SPI2, mosi, miso, nss);
    spi_handle_->Enable();

    gate_driver_ = new DRV8323(spi_handle_, enable, n_fault);

    // Power Up Device
    gate_driver_->EnablePower();
    osDelay(10);

    // Setup Initial DRV Values
    gate_driver_->Init();
    osDelay(10);

    // Calibrate Sense Amplifier Bias/Offset
    gate_driver_->Calibrate();
    osDelay(10);
    
    // TODO: Member Function with Callback for Command Handler
    // Load Configuration
   // load_configuration();

    // Compute PWM Parameters
    pwm_counter_period_ticks_ = SystemCoreClock / (2 * config_.pwm_freq);

    // Update Controller Sample Time
    controller_loop_freq_ = (config_.pwm_freq / config_.foc_ccl_divider);
    controller_update_period_ = (1.0f) / controller_loop_freq_;

    // Start PWM
    StartPWM();
    osDelay(150); // Delay for a bit to let things stabilize
       
    // Start ADCs
    StartADCs();
    osDelay(150); // Delay for a bit to let things stabilize

    // Initialize FET Thermistor
    fet_therm_ = new Thermistor(ADC4, FET_THERM_BETA, FET_THERM_RESISTANCE, FET_THERM_RESISTANCE_BAL,FET_THERM_LUT_SIZE);

    // Set Filter Alpha. TODO: Variable for this.  For now ms sampling with 1hz cutoff frequency
    fet_therm_->SetFilterAlpha(LowPassFilter::ComputeAlpha(1e-3, 1.0f));
    fet_therm_->GenerateTable();

    // Default Mode Startup:
    control_mode_ = STARTUP_MODE;

    // Initialize RMS Current limiter
    uint32_t sub_sample_count = rms_current_sample_period_/controller_update_period_;
    current_limiter_ = new RMSCurrentLimiter(motor_->config_.continuous_current_max, motor_->config_.continuous_current_tau, rms_current_sample_period_, sub_sample_count);
    current_limiter_->Reset();

    // Create Control FSM
    control_fsm_ = new NomadBLDCFSM();

    // Update FSM Data.  This could be better
    control_fsm_->GetData()->controller = this;
    control_fsm_->Setup();
    control_fsm_->Start(SysTick->VAL);

    control_initialized_ = true;

    // Set Singleton
    singleton_ = this;

    Logger::Instance().Print("MotorController::Init() - Motor Controller Initialized Successfully!\r\n");
}

bool MotorController::CheckErrors()
{
   // if (gate_driver_->CheckFaults())
     //   return true;

    return false;
}
bool MotorController::RunControlFSM()
{
    if(control_fsm_ == nullptr)
        return false;
    
    control_fsm_->Run(controller_update_period_);
    return true;
}

void MotorController::CurrentControl()
{
    dq0(motor_->state_.theta_elec, motor_->state_.I_a, motor_->state_.I_b, motor_->state_.I_c, &state_.I_d, &state_.I_q); //dq0 transform on current

    float curr_limit = std::min(state_.I_max, config_.current_limit);
    Core::Math::Vector2d::Limit(&state_.I_d_ref, &state_.I_q_ref, curr_limit);

    // PI Controller
    float i_d_error = state_.I_d_ref - state_.I_d;
    float i_q_error = state_.I_q_ref - state_.I_q; //  + cogging_current;

    // Calculate feed-forward voltages
    //float v_d_ff = SQRT3 * (1.0f * controller->i_d_ref * R_PHASE - controller->dtheta_elec * L_Q * controller->i_q); //feed-forward voltages
    //float v_q_ff = SQRT3 * (1.0f * controller->i_q_ref * R_PHASE + controller->dtheta_elec * (L_D * controller->i_d + 1.0f * WB));

    // Integrate Error
    state_.d_int += config_.k_d * config_.k_i_d * i_d_error;
    state_.q_int += config_.k_q * config_.k_i_q * i_q_error;

    state_.d_int = std::max(std::min(state_.d_int, config_.overmodulation * state_.Voltage_bus), -config_.overmodulation * state_.Voltage_bus);
    state_.q_int = std::max(std::min(state_.q_int, config_.overmodulation * state_.Voltage_bus), -config_.overmodulation * state_.Voltage_bus);

    //limit_norm(&controller->d_int, &controller->q_int, config_.overmodulation * state_->Voltage_bus);
    state_.V_d = config_.k_d * i_d_error + state_.d_int; //+ v_d_ff;
    state_.V_q = config_.k_q * i_q_error + state_.q_int; //+ v_q_ff;

    Core::Math::Vector2d::Limit(&state_.V_d, &state_.V_q, config_.overmodulation * state_.Voltage_bus); // Normalize voltage vector to lie within circle of radius v_bus

    // TODO: Do we need this linearization?
    //float v_ref = sqrt(controller->v_d * controller->v_d + controller->v_q * controller->v_q)
    //float dtc = v_ref / state_.Voltage_bus;

    float dtc_d = state_.V_d / state_.Voltage_bus;
    float dtc_q = state_.V_q / state_.Voltage_bus;

    state_.V_d = dtc_d * state_.Voltage_bus;
    state_.V_q = dtc_q * state_.Voltage_bus;

    SetModulationOutput(motor->state_.theta_elec + 0.0f * controller_update_period_ * motor->state_.theta_elec_dot, state_.V_d, state_.V_q);
}
void MotorController::StartPWM()
{
    LL_TIM_EnableAllOutputs(TIM8); 
    LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3); // Enable Channels

    LL_TIM_EnableCounter(TIM8); // Enable Counting

    // Enable Timers
    LL_TIM_SetPrescaler(TIM8, 0);             // No Prescaler
    LL_TIM_SetAutoReload(TIM8, pwm_counter_period_ticks_); // Set Period
    LL_TIM_SetRepetitionCounter(TIM8, (config_.foc_ccl_divider * 2) - 1);     // Loop Counter Decimator
    
    // Set Zer0 Duty Cycle
    SetDuty(0.5f, 0.5f, 0.5f);  // Zero Duty

    // Make Sure PWM is initially Disabled
    EnablePWM(false);

    // This makes sure PWM is stopped if we have debug point/crash
    __HAL_DBGMCU_FREEZE_TIM1();
}
void MotorController::StartADCs()
{
    // ADC Setup
    adc_1_ = new ADCDevice(ADC1);
    adc_2_ = new ADCDevice(ADC2);
    adc_3_ = new ADCDevice(ADC3);
    vbus_adc_ = new ADCDevice(ADC5);

    // Setup Filters
    // Filter to desired closed loop current control bandwidth
    float alpha = LowPassFilter::ComputeAlpha(controller_update_period_, config_.current_bandwidth);
    adc_1_->GetFilter().SetAlpha(alpha);
    adc_1_->GetFilter().Init(0.0f); 
    adc_2_->GetFilter().SetAlpha(alpha);
    adc_2_->GetFilter().Init(0.0f);
    adc_3_->GetFilter().SetAlpha(alpha);
    adc_3_->GetFilter().Init(0.0f);
    
    // Set Bus Filter Alpha. For now ms sampling with 1000hz cutoff frequency
    vbus_adc_->GetFilter().SetAlpha(LowPassFilter::ComputeAlpha(controller_update_period_, config_.current_bandwidth));
    vbus_adc_->GetFilter().Init(24.0f / voltage_scale); // 24 volts/Read default system voltage
    
    // Enable ADCs
    adc_1_->Enable();
    adc_2_->Enable();
    adc_3_->Enable();
    adc_3_->EnableIT();

    // Enable Bus Voltage ADC
    vbus_adc_->Enable();

    // Attach Callback
    adc_3_->Attach(std::bind(&MotorController::CurrentMeasurementCB, this));

    // Start ADC Continuous Conversions from Timer Interrupt
    adc_1_->Start();
    adc_2_->Start();
    adc_3_->Start();
}

void MotorController::EnablePWM(bool enable)
{
    if (enable)
    {
        LL_TIM_EnableAllOutputs(TIM8); // Advanced Timers turn on Outputs
    }
    else // Disable PWM Timer Unconditionally
    {
        LL_TIM_DisableAllOutputs(TIM8); // Advanced Timers turn on Outputs
    }
    osDelay(100);
  //  enable ? __HAL_TIM_MOE_ENABLE(&htim8) : __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
}

void MotorController::UpdateControllerGains()
{
    float crossover_freq = config_.current_bandwidth * controller_update_period_ * 2 * M_PI;
    float k_i = 1 - exp(-motor_->config_.phase_resistance * controller_update_period_ / motor->config_.phase_inductance_q);
    float k = motor->config_.phase_resistance * ((crossover_freq) / k_i);

    config_.k_d = config_.k_q = k;
    config_.k_i_d = config_.k_i_q = k_i;
    //config_.alpha = 1.0f - 1.0f / (1.0f - controller_update_period_ * config_.current_bandwidth * 2.0f * M_PI);

    dirty_ = true;
}
void MotorController::SetDuty(float duty_A, float duty_B, float duty_C)
{
    // TODO: Perf compare these.  Direct Reg vs LL
    if (motor_->config_.phase_order) // Check which phase order to use
    { 
        LL_TIM_OC_SetCompareCH1(TIM8, (uint16_t)(pwm_counter_period_ticks_) * (1.0f - duty_A)); // Set Duty Cycle Channel 1
        LL_TIM_OC_SetCompareCH2(TIM8, (uint16_t)(pwm_counter_period_ticks_) * (1.0f - duty_B)); // Set Duty Cycle Channel 2
        LL_TIM_OC_SetCompareCH3(TIM8, (uint16_t)(pwm_counter_period_ticks_) * (1.0f - duty_C)); // Set Duty Cycle Channel 3
    }
    else
    {
        LL_TIM_OC_SetCompareCH1(TIM8, (uint16_t)(pwm_counter_period_ticks_) * (1.0f - duty_A)); // Set Duty Cycle Channel 1
        LL_TIM_OC_SetCompareCH3(TIM8, (uint16_t)(pwm_counter_period_ticks_) * (1.0f - duty_B)); // Set Duty Cycle Channel 2
        LL_TIM_OC_SetCompareCH2(TIM8, (uint16_t)(pwm_counter_period_ticks_) * (1.0f - duty_C)); // Set Duty Cycle Channel 3
    }
}

// Transform Functions
// TODO: Remove
void MotorController::dqInverseTransform(float theta, float d, float q, float *a, float *b, float *c)
{
    // Inverse DQ0 Transform
    ///Phase current amplitude = length of dq vector///
    ///i.e. iq = 1, id = 0, peak phase current of 1///

    *a = d * arm_cos_f32(theta) - q * arm_sin_f32(theta);
    *b = d * arm_cos_f32(theta - (2.0f * M_PI / 3.0f)) - q * arm_sin_f32(theta - (2.0f * M_PI / 3.0f));
    *c = d * arm_cos_f32(theta + (2.0f * M_PI / 3.0f)) - q * arm_sin_f32(theta + (2.0f * M_PI / 3.0f));
}
void MotorController::dq0(float theta, float a, float b, float c, float *d, float *q)
{
    // DQ0 Transform
    // Phase current amplitude = length of dq vector
    // i.e. iq = 1, id = 0, peak phase current of 1

    float cf, sf;
    cordic.CosSin(theta, cf, sf);

    *d = 0.6666667f * (cf * a + (0.86602540378f * sf - .5f * cf) * b + (-0.86602540378f * sf - .5f * cf) * c); ///Faster DQ0 Transform
    *q = 0.6666667f * (-sf * a - (-0.86602540378f * cf - .5f * sf) * b - (0.86602540378f * cf - .5f * sf) * c);
}

void MotorController::ParkInverseTransform(float theta, float d, float q, float *alpha, float *beta)
{
    float cos_theta, sin_theta;
    cordic.CosSin(theta, cos_theta, sin_theta);

    *alpha = d * cos_theta - q * sin_theta;
    *beta = q * cos_theta + d * sin_theta;
}
void MotorController::ParkTransform(float theta, float alpha, float beta, float *d, float *q)
{
    float cos_theta, sin_theta;
    cordic.CosSin(theta, cos_theta, sin_theta);

    *d = alpha * cos_theta + beta * sin_theta;
    *q = beta * cos_theta - alpha * sin_theta;
}
void MotorController::ClarkeInverseTransform(float alpha, float beta, float *a, float *b, float *c)
{
    *a = alpha;
    *b = 0.5f * (-alpha + 1.73205080757f * beta);
    *c = 0.5f * (-alpha - 1.73205080757f * beta);
}
void MotorController::ClarkeTransform(float I_a, float I_b, float *alpha, float *beta)
{
    // Ialpha = Ia
    // Ibeta = 1/sqrt(3)(Ia + 2Ib)
    *alpha = I_a;
    *beta = 0.57735026919f * (I_a + 2.0f * I_b);
}

void MotorController::SVM(float a, float b, float c, float *dtc_a, float *dtc_b, float *dtc_c)
{
    // Space Vector Modulation
    // a,b,c amplitude = Bus Voltage for Full Modulation Depth
    float v_offset = (Core::Math::Vector3d::Min(a, b, c) + Core::Math::Vector3d::Max(a, b, c)) * 0.5f;

    *dtc_a = std::min(std::max(((a - v_offset) / state_.Voltage_bus + 0.5f), DTC_MIN), DTC_MAX);
    *dtc_b = std::min(std::max(((b - v_offset) / state_.Voltage_bus + 0.5f), DTC_MIN), DTC_MAX);
    *dtc_c = std::min(std::max(((c - v_offset) / state_.Voltage_bus + 0.5f), DTC_MIN), DTC_MAX);
}

void MotorController::SetModulationOutput(float theta, float v_d, float v_q)
{
    //dqInverseTransform(0.0f, lock_voltage, 0.0f, &U, &V, &W); // Test voltage to D-Axis
    float v_alpha, v_beta;
    ParkInverseTransform(theta, v_d, v_q, &v_alpha, &v_beta);
    SetModulationOutput(v_alpha, v_beta);
}

void MotorController::SetModulationOutput(float v_alpha, float v_beta)
{
    float A, B, C;
    //float dtc_A, dtc_B, dtc_C = 0.5f;
    ClarkeInverseTransform(v_alpha, v_beta, &A, &B, &C);
    SVM(A, B, C, &state_.dtc_A, &state_.dtc_B, &state_.dtc_C); // Space Vector Modulation
    SetDuty(state_.dtc_A, state_.dtc_B, state_.dtc_C);
}

void MotorController::TorqueControl()
{
    float deadband = 0.1f;
    float torque_ref_in = state_.T_ff + state_.K_p * (state_.Pos_ref - motor->state_.theta_mech) + state_.K_d * (state_.Vel_ref - motor->state_.theta_mech_dot);
    float torque_ref = 0.0f;
    // Check Position Limits
    if(motor->state_.theta_mech <= config_.pos_limit_min)
    {
        torque_ref = config_.K_p_limit * (config_.pos_limit_min - motor->state_.theta_mech) + config_.K_d_limit * (0.0f - motor->state_.theta_mech_dot);
        in_limit_min_ = true;
    }
    else if(motor->state_.theta_mech >= config_.pos_limit_max)
    {
        torque_ref = config_.K_p_limit * (config_.pos_limit_max - motor->state_.theta_mech) + config_.K_d_limit * (0.0f - motor->state_.theta_mech_dot);
        in_limit_max_ = true;
    }

    if(in_limit_max_) // Check Hysteresis
    {
        if(motor->state_.theta_mech < config_.pos_limit_max-deadband || torque_ref_in < 0)
            in_limit_max_ = false;
    }
    else if(in_limit_min_) // Check Hysteresis
    {
        if(motor->state_.theta_mech > config_.pos_limit_min+deadband || torque_ref_in > 0)
            in_limit_min_ = false;
    }

    
    // TODO: Also check velocity limits
    if(!in_limit_min_ && !in_limit_max_) // Not Limiting
    {
        torque_ref =  state_.T_ff + state_.K_p * (state_.Pos_ref - motor->state_.theta_mech) + state_.K_d * (state_.Vel_ref - motor->state_.theta_mech_dot);
    }
    
    state_.I_q_ref = torque_ref / (motor->config_.K_t * motor->config_.gear_ratio);
    state_.I_d_ref = 0.0f;
    CurrentControl(); // Do Current Controller
}
