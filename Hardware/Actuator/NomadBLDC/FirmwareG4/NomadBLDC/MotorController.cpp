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

#include <Utilities/utils.h>
#include <Utilities/lpf.h>

#include <FSM/NomadBLDCFSM.h>
#include "LEDService.h"
#include "Logger.h"
#include "math_ops.h"

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
    //CANHandler::Config_t can_config;
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

      //  Logger::Instance().Print("Pos: %f\r\n", motor->PositionSensor()->GetMechanicalPosition());
        // Delay 1 ms
        osDelay(1);
    }
}

void init_motor_controller()
{
    //printf("Motor RT Controller Task Up.\n\r");
    Logger::Instance().Print("Motor RT Controller Task Up.\r\n");

    // Init CORDIC Routines
    //Cordic::Instance().Init();
    // TODO: Will have to move this
    cordic.Init();
    cordic.SetPrecision(LL_CORDIC_PRECISION_6CYCLES);

    // Init Motor and Implicitly Position Sensor
    motor = new Motor(0.000025f, 80, 21);

    // Init Motor Controller
    motor_controller = new MotorController(motor);
    motor_controller->Init();

    //Update Sample Time For Motor
    motor->SetSampleTime(motor_controller->GetControlUpdatePeriod());
    
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
    config_.overmodulation = 1.0f;
    config_.position_limit = 12.5f; // +/-
    config_.velocity_limit = 10.0f; // +/-
    config_.torque_limit = 10.0f; // +/-
    config_.current_limit = 20.0f;  // +/-
    config_.current_bandwidth = 1000.0f;

    config_.K_p_min = 0.0f;
    config_.K_p_max = 500.0f;
    config_.K_d_min = 0.0f;
    config_.K_d_max = 5.0f; 

    config_.pwm_freq = 40000.0f; // 40 khz
    config_.foc_ccl_divider = 1; // Default to not divide.  Current loops runs at same freq as PWM

    // TODO: Parameter
    rms_current_sample_period_ = 1.0f/10.0f;
    controller_loop_freq_ = (config_.pwm_freq / config_.foc_ccl_divider);
    controller_update_period_ = (1.0f) / controller_loop_freq_;

}
void MotorController::Reset()
{
    SetModulationOutput(0.0f, 0.0f);

    state_.I_d = 0.0f;
    state_.I_q = 0.0f;
    state_.I_d_filtered = 0.0f;
    state_.I_q_filtered = 0.0f;
    state_.I_d_ref = 0.0f;
    state_.I_q_ref = 0.0f;
    state_.I_d_ref_filtered = 0.0f;
    state_.I_q_ref_filtered = 0.0f;
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
}

void MotorController::CurrentMeasurementCB()
{
    // Performance Measure
    //LL_GPIO_SetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);
    
    // Depends on PWM duty cycles
    if (motor_->config_.phase_order) // Check Phase Ordering
    {
        motor_->state_.I_a = current_scale * static_cast<float>(adc_2_->Read());
        motor_->state_.I_b = current_scale * static_cast<float>(adc_3_->Read());
        motor_->state_.I_c = current_scale * static_cast<float>(adc_1_->Read());
    }
    else
    {
        motor_->state_.I_a = current_scale * static_cast<float>(adc_1_->Read());
        motor_->state_.I_b = current_scale * static_cast<float>(adc_3_->Read());
        motor_->state_.I_c = current_scale * static_cast<float>(adc_2_->Read());
    }
    
    // Kirchoffs Current Law to compute 3rd unmeasured current.
    //motor_->state_.I_a = -motor_->state_.I_b - motor_->state_.I_c;

    // TODO: Use Longest Duty Cycle Measurements for which of the 3 phases to use

    // Always Update Motor State
    motor_->Update();

    // Run FSM for timestep
    RunControlFSM();
}


void MotorController::SampleBusVoltage()
{
    // Sample Bus Voltage
    state_.Voltage_bus = static_cast<float>(vbus_adc_->Sample()) * voltage_scale;
}

void MotorController::SampleFETTemperature()
{
    // Sample FET Thermistor for Temperature
    this->state_.fet_temp = fet_therm_->SampleTemperature();
}

void MotorController::Init()
{
    Logger::Instance().Print("MotorController::Init() - Motor Controller Initializing...\r\n");

    // Compute Maximum Allowed Current
    float margin = 1.0f;
    float max_input = margin * 0.3f * SENSE_CONDUCTANCE;
    float max_swing = margin * 1.6f * SENSE_CONDUCTANCE * (1.0f / CURRENT_SENSE_GAIN);
    current_max_ = fminf(max_input, max_swing);

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
    load_configuration();

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
// void MotorController::StartControlFSM()
// {

    // // Disable PWM
    // EnablePWM(false);
    // for (;;)
    // {
    //     if (control_mode_ != IDLE_MODE && CheckErrors())
    //     {
    //         Logger::Instance().Print("Bailing Here CHECK ERROR\n");
    //         control_mode_ = ERROR_MODE;
    //     }

    //     // Check Current Mode
    //     switch (control_mode_)
    //     {

    //     case (ERROR_MODE):
    //         if (current_control_mode != control_mode_)
    //         {
    //             current_control_mode = control_mode_;
    //             //TODO: Flash a code... LEDService::Instance().Off();
    //             //printf("Controller Error: \r\n");

    //             gate_driver_->PrintFaults();

    //             Reset(); // Reset Controller to Zero

    //             gate_driver_->DisableDriver();
    //             EnablePWM(false);
    //         }

    //         osDelay(1);
    //         break;
    
    //     case (FOC_CURRENT_MODE):
    //     case (FOC_VOLTAGE_MODE):
    //     case (FOC_TORQUE_MODE):
    //     case (FOC_SPEED_MODE):
    //     {
    //         if (current_control_mode != control_mode_)
    //         {
    //             current_control_mode = control_mode_;

    //             // Make sure we are calibrated:
    //             if (!motor_->config_.calibrated)
    //             {
    //                 //printf("Motor is NOT Calibrated.  Please run Motor Calibration before entering control modes!\r\n");
    //                 Logger::Instance().Print("NOt Calibrated ERROR\n");
    //                 control_mode_ = ERROR_MODE;
    //                 continue;
    //             }

    //             // Reset
    //             Reset();

    //             LEDService::Instance().On();
    //             gate_driver_->EnableDriver();
    //             EnablePWM(true);
    //         }
    //         if (osThreadFlagsWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, osFlagsWaitAny, CURRENT_MEASUREMENT_TIMEOUT) != CURRENT_MEASUREMENT_COMPLETE_SIGNAL)
    //         {
    //             // TODO: Should have a check for number of missed deadlines, then bail.  Leaky Integrator
    //             //printf("ERROR: Motor Controller Timeout!\r\n");
    //             control_mode_ = ERROR_MODE;
    //             continue;
    //         }
    //          LL_GPIO_ResetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);

    //         //LL_GPIO_SetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);
    //        // DWT->CYCCNT = 0;
    //         // Measure
    //         // TODO: Transform to Id/Iq here?.  For now use last sample.  
    //         // TOOD: Do this in the current control loop
    //         // TODO: Also need to make this work for voltage mode Vrms=IrmsR should work hopefully
    //         static float I_sample = 0.0f;
    //         I_sample = sqrt(motor_controller->state_.I_d * motor_controller->state_.I_d + motor_controller->state_.I_q * motor_controller->state_.I_q);
    //         // Update Current Limiter
    //         current_limiter->AddCurrentSample(I_sample);
    //         motor_controller->state_.I_rms = current_limiter->GetRMSCurrent();
    //         motor_controller->state_.I_max = current_limiter->GetMaxAllowableCurrent();
            
    //         DoMotorControl();
    //        // LL_GPIO_ResetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);
    //        // uint32_t span = DWT->CYCCNT;
    //         //Logger::Instance().Print("Count: %f\r\n", j);
    //         break;
    //     }
    //     default:
    //         if (current_control_mode != control_mode_)
    //         {
    //             current_control_mode = control_mode_;
    //             LEDService::Instance().Off();
    //             gate_driver_->DisableDriver();
    //             EnablePWM(false);
    //         }
    //         Logger::Instance().Print("Unhandled Control Mode: %d!\r\n", control_mode_);
    //         osDelay(1);
    //         break;
    //     }
    // }
// }
void MotorController::DoMotorControl()
{
  //  NVIC_DisableIRQ(USART2_IRQn);
    __disable_irq();
    if (control_mode_ == FOC_VOLTAGE_MODE)
    {
        motor_controller->state_.V_d = motor_controller->state_.V_d_ref;
        motor_controller->state_.V_q = motor_controller->state_.V_q_ref;
        SetModulationOutput(motor->state_.theta_elec, motor_controller->state_.V_d_ref, motor_controller->state_.V_q_ref);

        // Update V_d/V_q
        // TODO: Should probably have this more universal somewhere
        dq0(motor_->state_.theta_elec, motor_->state_.I_a, motor_->state_.I_b, motor_->state_.I_c, &state_.I_d, &state_.I_q); //dq0 transform on currents
        motor_controller->state_.V_d = motor_controller->state_.I_d * motor->config_.phase_resistance;
        motor_controller->state_.V_q = motor_controller->state_.I_q * motor->config_.phase_resistance;
    }
    else if (control_mode_ == FOC_CURRENT_MODE)
    {
        CurrentControl();
    }
    else if (control_mode_ == FOC_TORQUE_MODE)
    {
        TorqueControl();
    }
   // LL_GPIO_ResetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);
   __enable_irq();
  //  NVIC_EnableIRQ(USART2_IRQn);
}
void MotorController::CurrentControl()
{
    dq0(motor_->state_.theta_elec, motor_->state_.I_a, motor_->state_.I_b, motor_->state_.I_c, &state_.I_d, &state_.I_q); //dq0 transform on currents

    state_.I_d_filtered = 0.95f * state_.I_d_filtered + 0.05f * state_.I_d;
    state_.I_q_filtered = 0.95f * state_.I_q_filtered + 0.05f * state_.I_q;
    

    // Filter the current references to the desired closed-loop bandwidth
    state_.I_d_ref_filtered = (1.0f - config_.alpha) * state_.I_d_ref_filtered + config_.alpha * state_.I_d_ref;
    state_.I_q_ref_filtered = (1.0f - config_.alpha) * state_.I_q_ref_filtered + config_.alpha * state_.I_q_ref;

    float curr_limit = std::min(motor_controller->state_.I_max, config_.current_limit);
    limit_norm(&state_.I_d_ref, &state_.I_q_ref, curr_limit);

    // PI Controller
    float i_d_error = state_.I_d_ref - state_.I_d;
    float i_q_error = state_.I_q_ref - state_.I_q; //  + cogging_current;

    // Calculate feed-forward voltages
    //float v_d_ff = SQRT3 * (1.0f * controller->i_d_ref * R_PHASE - controller->dtheta_elec * L_Q * controller->i_q); //feed-forward voltages
    //float v_q_ff = SQRT3 * (1.0f * controller->i_q_ref * R_PHASE + controller->dtheta_elec * (L_D * controller->i_d + 1.0f * WB));

    // Integrate Error
    state_.d_int += config_.k_d * config_.k_i_d * i_d_error;
    state_.q_int += config_.k_q * config_.k_i_q * i_q_error;

    state_.d_int = fmaxf(fminf(state_.d_int, config_.overmodulation * state_.Voltage_bus), -config_.overmodulation * state_.Voltage_bus);
    state_.q_int = fmaxf(fminf(state_.q_int, config_.overmodulation * state_.Voltage_bus), -config_.overmodulation * state_.Voltage_bus);

    //limit_norm(&controller->d_int, &controller->q_int, OVERMODULATION*controller->v_bus);
    motor_controller->state_.V_d = config_.k_d * i_d_error + state_.d_int; //+ v_d_ff;
    motor_controller->state_.V_q = config_.k_q * i_q_error + state_.q_int; //+ v_q_ff;

    //controller->v_ref = sqrt(controller->v_d * controller->v_d + controller->v_q * controller->v_q);
    limit_norm(&motor_controller->state_.V_d, &motor_controller->state_.V_q, config_.overmodulation * state_.Voltage_bus); // Normalize voltage vector to lie within circle of radius v_bus

    // TODO: Do we need this linearization?
    //float v_ref = sqrt(controller->v_d * controller->v_d + controller->v_q * controller->v_q)
    //float dtc = v_ref / state_.Voltage_bus;

    float dtc_d = motor_controller->state_.V_d / state_.Voltage_bus;
    float dtc_q = motor_controller->state_.V_q / state_.Voltage_bus;
    //LinearizeDTC(&dtc_d);
    //LinearizeDTC(&dtc_q);

    motor_controller->state_.V_d = dtc_d * state_.Voltage_bus;
    motor_controller->state_.V_q = dtc_q * state_.Voltage_bus;

    SetModulationOutput(motor->state_.theta_elec + 0.0f * controller_update_period_ * motor->state_.theta_elec_dot, motor_controller->state_.V_d, motor_controller->state_.V_q);
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

    // Setup Filters?
    // float alpha = LowPassFilter::ComputeAlpha(controller_update_period_, 125e3);
    // adc_1_->SetFilter(alpha);
    // adc_2_->SetFilter(alpha);
    // adc_3_->SetFilter(alpha);
    // Set Filter Alpha. TODO: Variable for this.  For now ms sampling with 1000hz cutoff frequency
    vbus_adc_->GetFilter().SetAlpha(LowPassFilter::ComputeAlpha(1e-3, 1000.0f));
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
    float crossover_freq = config_.current_bandwidth * controller_update_period_ * 2 * PI;
    float k_i = 1 - exp(-motor_->config_.phase_resistance * controller_update_period_ / motor->config_.phase_inductance_q);
    float k = motor->config_.phase_resistance * ((crossover_freq) / k_i);

    config_.k_d = config_.k_q = k;
    config_.k_i_d = config_.k_i_q = k_i;
    config_.alpha = 1.0f - 1.0f / (1.0f - controller_update_period_ * config_.current_bandwidth * 2.0f * PI);

    dirty_ = true;
}
void MotorController::SetDuty(float duty_A, float duty_B, float duty_C)
{
    // TODO: Perf compare these.  Direct Reg vs LL
    if (motor_->config_.phase_order) // Check which phase order to use
    { 
        // TIM8->CCR1 = (uint16_t)(pwm_counter_period_ticks_) * (1.0f - duty_A); // Write duty cycles
        // TIM8->CCR2 = (uint16_t)(pwm_counter_period_ticks_) * (1.0f - duty_B);
        // TIM8->CCR3 = (uint16_t)(pwm_counter_period_ticks_) * (1.0f - duty_C);
        LL_TIM_OC_SetCompareCH1(TIM8, (uint16_t)(pwm_counter_period_ticks_) * (1.0f - duty_A)); // Set Duty Cycle Channel 1
        LL_TIM_OC_SetCompareCH2(TIM8, (uint16_t)(pwm_counter_period_ticks_) * (1.0f - duty_B)); // Set Duty Cycle Channel 2
        LL_TIM_OC_SetCompareCH3(TIM8, (uint16_t)(pwm_counter_period_ticks_) * (1.0f - duty_C)); // Set Duty Cycle Channel 3
    }
    else
    {
        // TIM8->CCR1 = (uint16_t)(pwm_counter_period_ticks_) * (1.0f - duty_A);
        // TIM8->CCR3 = (uint16_t)(pwm_counter_period_ticks_) * (1.0f - duty_B);
        // TIM8->CCR2 = (uint16_t)(pwm_counter_period_ticks_) * (1.0f - duty_C);
        LL_TIM_OC_SetCompareCH1(TIM8, (uint16_t)(pwm_counter_period_ticks_) * (1.0f - duty_A)); // Set Duty Cycle Channel 1
        LL_TIM_OC_SetCompareCH3(TIM8, (uint16_t)(pwm_counter_period_ticks_) * (1.0f - duty_B)); // Set Duty Cycle Channel 2
        LL_TIM_OC_SetCompareCH2(TIM8, (uint16_t)(pwm_counter_period_ticks_) * (1.0f - duty_C)); // Set Duty Cycle Channel 3
    }
}

// Transform Functions
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

    //float cf = arm_cos_f32(theta);
    //float sf = arm_sin_f32(theta);

    float cf, sf;
    cordic.CosSin(theta, cf, sf);

    *d = 0.6666667f * (cf * a + (0.86602540378f * sf - .5f * cf) * b + (-0.86602540378f * sf - .5f * cf) * c); ///Faster DQ0 Transform
    *q = 0.6666667f * (-sf * a - (-0.86602540378f * cf - .5f * sf) * b - (0.86602540378f * cf - .5f * sf) * c);
}

void MotorController::ParkInverseTransform(float theta, float d, float q, float *alpha, float *beta)
{
    float cos_theta = arm_cos_f32(theta);
    float sin_theta = arm_sin_f32(theta);

    //float cos_theta, sin_theta;
    //cordic.CosSin(theta, cos_theta, sin_theta);

    *alpha = d * cos_theta - q * sin_theta;
    *beta = q * cos_theta + d * sin_theta;
}
void MotorController::ParkTransform(float theta, float alpha, float beta, float *d, float *q)
{
   // float cos_theta = arm_cos_f32(theta);
   // float sin_theta = arm_sin_f32(theta);

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
    float v_offset = (fminf3(a, b, c) + fmaxf3(a, b, c)) * 0.5f;

    *dtc_a = fminf(fmaxf(((a - v_offset) / state_.Voltage_bus + 0.5f), DTC_MIN), DTC_MAX);
    *dtc_b = fminf(fmaxf(((b - v_offset) / state_.Voltage_bus + 0.5f), DTC_MIN), DTC_MAX);
    *dtc_c = fminf(fmaxf(((c - v_offset) / state_.Voltage_bus + 0.5f), DTC_MIN), DTC_MAX);
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
    float torque_ref = state_.K_p * (state_.Pos_ref - motor->state_.theta_mech) + state_.T_ff + state_.K_d * (state_.Vel_ref - motor->state_.theta_mech_dot);
    state_.I_q_ref = torque_ref / (motor->config_.K_t * motor->config_.gear_ratio);
    state_.I_d_ref = 0.0f;
    CurrentControl(); // Do Current Controller
}
