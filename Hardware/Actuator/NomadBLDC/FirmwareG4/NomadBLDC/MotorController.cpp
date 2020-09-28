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
//#include "UserMenu.h"
#include <Peripherals/flash.h>
#include "LEDService.h"
#include "Logger.h"
#include "math_ops.h"

#define FLASH_VERSION 2

Motor *motor = 0;
MotorController *motor_controller = 0;

RMSCurrentLimiter *current_limiter = 0;

// Globals
static int32_t g_adc1_offset;
static int32_t g_adc2_offset;
static int32_t g_adc3_offset;

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

void motor_controller_thread_entry(void *arg)
{
    //printf("Motor RT Controller Task Up.\n\r");
    Logger::Instance().Print("Motor RT Controller Task Up.\r\n");

    // Init Motor and Implicitly Position Sensor
    motor = new Motor(0.000025f, 100, 21);

    // Init Motor Controller
    motor_controller = new MotorController(motor);
    motor_controller->Init();

    motor->SetSampleTime(motor_controller->GetControlUpdatePeriod());
    //Logger::Instance().Print("CONTROL LOOP: %f\n\r", CONTROL_LOOP_FREQ);
    //Logger::Instance().Print("PWM FREQ :%f\n\r", PWM_FREQ);

// for(; ; )
// {
//     osDelay(100);
// }
    // Begin Control Loop
    motor_controller->StartControlFSM();
    
}

// void debug_thread_entry()
// {
//     // TODO: Print Speed/Rate
//     // TODO: Move Encoder Debug Print Here
//     while (1)
//     {
//         if (motor_controller->GetDebugMode() == true)
//         {
//             if (motor_controller->GetControlMode() == FOC_TORQUE_MODE)
//             {
//                 MotorController::State_t state = motor_controller->state_;
//                 Motor::Config_t config = motor->config_;
//                 //printf("\r\nI_d: %.3f/%.3f A\tI_q: %.3f/%.3f A\t| Torque: %.2f/%.2f N*m\r\n", 
//                 //state.I_d_filtered, state.I_d_ref, 
//                 //state.I_q_filtered, state.I_q_ref, 
//                 //(state.I_q_filtered * config.K_t_out), (state.I_q_ref * motor->config_.K_t_out));

//                 //printf("Voltage Bus: %.3f V\r\n", state.Voltage_bus);
//                 // TODO: Power, temperatures etc.
//             }
//         }
//         osDelay(500);
//     }
// }
// Controller Mode Interface
void set_control_mode(int mode)
{
    motor_controller->SetControlMode((control_mode_type_t)mode);
}

void set_controller_debug(bool debug)
{
    motor_controller->SetDebugMode(debug);
}

bool get_controller_debug()
{
    return motor_controller->GetDebugMode();
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
void current_measurement_cb()
{
    // // Measure Currents/Bus Voltage
    // int32_t adc2_raw = ADC2->DR; // Current Sense Measurement 1
    // int32_t adc1_raw = ADC1->DR; // Current Sense Measurement 2
    // int32_t adc3_raw = ADC3->DR; // Voltage Bus Measurement.  TODO: Move this to a different/slower timer

    // // TODO: Not sure this is necessasry?
    // Depends on PWM duty cycles
    // if (motor->config_.phase_order) // Check Phase Ordering
    // {
    //     motor->state_.I_b = current_scale * (float)(adc2_raw - g_adc2_offset);
    //     motor->state_.I_c = current_scale * (float)(adc1_raw - g_adc1_offset);
    // }
    // else
    // {
    //     motor->state_.I_b = current_scale * (float)(adc1_raw - g_adc1_offset);
    //     motor->state_.I_c = current_scale * (float)(adc2_raw - g_adc2_offset);
    // }
    // // Kirchoffs Current Law to compute 3rd unmeasured current.
    // motor->state_.I_a = -motor->state_.I_b - motor->state_.I_c;

    // // Always Update Motor State
    // motor->Update();

    // // Filter bus measurement a bit
    // motor_controller->state_.Voltage_bus = 0.95f * motor_controller->state_.Voltage_bus + 0.05f * ((float)adc3_raw) * voltage_scale;

    motor_controller->state_.Voltage_bus = 24;
    // Make sure control thread is ready
    if (motor_controller != 0 && motor_controller->ControlThreadReady())
    {
      
       osThreadFlagsSet(motor_controller->GetThreadID(), CURRENT_MEASUREMENT_COMPLETE_SIGNAL);
        LL_GPIO_ResetOutputPin(USER_GPIO_GPIO_Port, USER_GPIO_Pin);
    }
}

bool zero_current_sensors(uint32_t num_samples)
{
    // Zero Offsets
    g_adc1_offset = 0;
    g_adc2_offset = 0;
    g_adc3_offset = 0;

    // Set Idle/"Zero" PWM
    motor_controller->SetDuty(0.5f, 0.5f, 0.5f);
    for (uint32_t i = 0; i < num_samples; i++) // Average num_samples of the ADC
    {
        if (osThreadFlagsWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, osFlagsWaitAny, CURRENT_MEASUREMENT_TIMEOUT) != CURRENT_MEASUREMENT_COMPLETE_SIGNAL)
        {
            // TODO: Error here for timing
            //printf("ERROR: Zero Current FAILED!\r\n");
            Logger::Instance().Print("ERROR: Zero Current Sensor Failed!\r\n");
            return false;
        }
        g_adc3_offset += ADC3->DR;
        g_adc2_offset += ADC2->DR;
        g_adc1_offset += ADC1->DR;
    }
    g_adc1_offset = g_adc1_offset / num_samples;
    g_adc2_offset = g_adc2_offset / num_samples;
    g_adc3_offset = g_adc3_offset / num_samples;

    Logger::Instance().Print("ADC OFFSET: %d and %d and %d\r\n", g_adc1_offset, g_adc2_offset, g_adc3_offset);
    return true;
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
    //printf("\r\nSaving Configuration...\r\n");

    bool status = false;
    Save_format_t save;
    save.signature = FLASH_SAVE_SIGNATURE;
    save.version = FLASH_VERSION; // Set Version
    save.motor_config = motor->config_;
    save.position_sensor_config = motor->PositionSensor()->config_;
    save.controller_config = motor_controller->config_;

    // Write Flash
    FlashDevice::Instance().Open(ADDR_FLASH_PAGE_252, sizeof(save), FlashDevice::WRITE);
    status = FlashDevice::Instance().Write(0, (uint8_t *)&save, sizeof(save));
    FlashDevice::Instance().Close();

    return status;
    //printf("\r\nSaved.  Press ESC to return to menu.\r\n");
}
void load_configuration()
{
    //printf("\r\nLoading Configuration...\r\n");
    Save_format_t load;

    FlashDevice::Instance().Open(ADDR_FLASH_PAGE_252, sizeof(load), FlashDevice::READ);
    FlashDevice::Instance().Read(0, (uint8_t *)&load, sizeof(load));
    FlashDevice::Instance().Close();

    if (load.signature != FLASH_SAVE_SIGNATURE || load.version != FLASH_VERSION)
    {
        //printf("\r\nERROR: No Configuration Found!.  Press ESC to return to menu.\r\n\r\n");
        Logger::Instance().Print("ERROR: No Valid Configuration Found!  Please run setup before enabling drive.");
        return;
    }

    motor->config_ = load.motor_config;
    motor->PositionSensor()->config_ = load.position_sensor_config;
    motor_controller->config_ = load.controller_config;

    motor->ZeroOutputPosition();

    //printf("READ Version: %d\r\n", load.version);
    //printf("\r\nLoaded.\r\n");
}
void reboot_system()
{
    HAL_NVIC_SystemReset();
}

void start_torque_control()
{
    //printf("\r\nFOC Torque Mode Enabled. Press ESC to stop.\r\n\r\n");
    set_control_mode(FOC_TORQUE_MODE);
}

void start_current_control()
{
    //printf("\r\nFOC Current Mode Enabled. Press ESC to stop.\r\n\r\n");
    set_control_mode(FOC_CURRENT_MODE);
}

void start_speed_control()
{
    //printf("\r\nFOC Speed Mode Enabled. Press ESC to stop.\r\n\r\n");
    set_control_mode(FOC_SPEED_MODE);
}

void start_voltage_control()
{
    //printf("\r\nFOC Voltage Mode Enabled. Press ESC to stop.\r\n\r\n");
    set_control_mode(FOC_VOLTAGE_MODE);
}

void show_encoder_debug()
{
    //printf("\r\nEncoder Debug Mode. Press ESC to stop.\r\n\r\n");
    set_control_mode(ENCODER_DEBUG);
}
void enter_idle()
{
    set_control_mode(IDLE_MODE);
}

void zero_encoder_offset()
{
    //printf("\r\n\r\nZeroing Mechanical Offset...\r\n\r\n");
    motor->ZeroOutputPosition();
    //motor->PrintPosition();
    //printf("\r\nEncoder Output Zeroed!. Press ESC to return to menu.\r\n\r\n");
}

void show_motor_config()
{
    // printf("\r\nMotor Configuration:\r\n");

    // printf("\r\n Calibration Status: ");
    // if (motor->config_.calibrated)
    //     printf("CALIBRATED\r\n");
    // else
    //     printf("UNCALIBRATED\r\n");

    // printf("\r\n Phase Order: ");
    // if (motor->config_.phase_order == 1)
    //     printf("CORRECT\r\n");
    // else
    //     printf("REVERSED\r\n");

    // printf("\r\nMotor Constants:\r\n");

    // printf("\r\n Pole Pairs: %u    K_v: %.2f RPM/V    K_t: %.4f (N*m)/A    K_t_out: %.4f (N*m)/A     Flux Linkage: %.4f Webers\r\n",
    //        motor->config_.num_pole_pairs,
    //        motor->config_.K_v,
    //        motor->config_.K_t,
    //        motor->config_.K_t_out,
    //        motor->config_.flux_linkage);

    // printf("\r\n Phase Resistance: %.5f ohms    Phase Inductance D: %.5f H    Phase Inductance Q: %.5f H\r\n",
    //        motor->config_.phase_resistance,
    //        motor->config_.phase_inductance_d,
    //        motor->config_.phase_inductance_q);

    // printf("\r\nCalibration Parameters: \r\n");
    // printf("\r\n Calibration Current: %.4f\tCalibration Voltage: %.4f\r\n",
    //        motor->config_.calib_current,
    //        motor->config_.calib_voltage);

    // printf("\r\nPress ESC to return to menu.\r\n\r\n");
}
void show_controller_config()
{
    // printf("\r\nMotor Controller Configuration:\r\n");

    // printf("\r\nBus Voltage: %.4f V\r\n", motor_controller->state_.Voltage_bus);

    // printf("\r\nController Timings:\r\n");
    // //printf("\r\n PWM Freqency: %.4f khz   Control Loop Frequency: %.4f khz  Control Loop Period: %.6f s",
    // //       motor_controller->config_.pwm_freq / 1000.0f,
    // //       motor_controller->control_loop_freq_ / 1000.0f,
    // //       motor_controller->control_loop_period_);
    // printf("\r\n\r\nController Gains:\r\n");

    // printf("\r\n Loop Bandwidth: %.4f hz    Loop Gain(kd/kq): %.4f/%.4f    Integrator Gain(k_i_d/k_i_q): %.4f/%.4f\r\n",
    //        motor_controller->config_.current_bandwidth,
    //        motor_controller->config_.k_d,
    //        motor_controller->config_.k_q,
    //        motor_controller->config_.k_i_d,
    //        motor_controller->config_.k_i_q);

    // printf("\r\nController Limits: \r\n");
    // printf("\r\n Current Limit: %.4f A    Velocity Limit: %.4f rad/s   Overmodulation: %.4f\r\n",
    //        motor_controller->config_.current_limit,
    //        motor_controller->config_.velocity_limit,
    //        motor_controller->config_.overmodulation);
    // printf("\r\nPress ESC to return to menu.\r\n\r\n");
}
void show_encoder_config()
{
    // printf("\r\nAM5147 Position Sensor Configuration:\r\n");
    // printf("\r\n Encoder CPR: %d", motor->PositionSensor()->config_.cpr);
    // printf("\r\n\r\nSensor Offsets:\r\n");

    // printf("\r\n Electrical Offset: %.4f rad    Mechanical Offset: %.4f rad\r\n",
    //        motor->PositionSensor()->config_.offset_elec,
    //        motor->PositionSensor()->config_.offset_mech);

    // printf("\n\r Encoder non-linearity compensation table\n\r");
    // printf(" Lookup Index : Lookup Value\n\r\n\r");
    // for (int32_t i = 0; i < 128; i++) // Build Lookup Table
    // {
    //     printf("%ld\t\t%ld\n\r", i, motor->PositionSensor()->config_.offset_lut[i]);
    // }
    // printf("\r\nPress ESC to return to menu.\r\n\r\n");
}
// Control Loop Timer Interrupt Synced with PWM
// extern "C" void TIM1_UP_TIM10_IRQHandler(void)
// {
//     if (TIM1->SR & TIM_SR_UIF) // TIM1 is up, and update interrupts are enabled on TIM1
//     {
//         ADC1->CR2 |= 0x40000000;  // Begin ADC Conversion
//         current_measurement_cb(); // Callback
//     }
//     TIM1->SR = 0x0; // reset the status register
// }
// Statics
MotorController *MotorController::singleton_ = nullptr;

MotorController::MotorController(Motor *motor) : motor_(motor)
{
    control_thread_id_ = 0;
    control_thread_ready_ = false;
    control_initialized_ = false;
    control_enabled_ = false;
    control_debug_ = false;

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
void MotorController::LinearizeDTC(float *dtc)
{
    /// linearizes the output of the inverter, which is not linear for small duty cycles ///
    float sgn = 1.0f - (2.0f * (*dtc < 0));
    if (abs(*dtc) >= .01f)
    {
        *dtc = *dtc * .986f + .014f * sgn;
    }
    else
    {
        *dtc = 2.5f * (*dtc);
    }
}

void MotorController::Init()
{
    //printf("MotorController::Init() - Motor Controller Initializing...\n\r");

    // Update Control Thread State
    control_thread_id_ = osThreadGetId();
    control_thread_ready_ = true;
    control_mode_ = CALIBRATION_MODE; // Start in "Calibration" mode.

    // Compute Maximum Allowed Current
    float margin = 1.0f;
    float max_input = margin * 0.3f * SENSE_CONDUCTANCE;
    float max_swing = margin * 1.6f * SENSE_CONDUCTANCE * (1.0f / CURRENT_SENSE_GAIN);
    current_max_ = fminf(max_input, max_swing);
    //Logger::Instance().Print("MAX: %f\n", current_max_);
    // TODO: Make sure this is in a valid range?

    // Setup DRV Pins
    GPIO_t mosi = {DRV_MOSI_GPIO_Port, DRV_MOSI_Pin};
    GPIO_t miso = {DRV_MISO_GPIO_Port, DRV_MISO_Pin};
    GPIO_t nss = {DRV_CS_GPIO_Port, DRV_CS_Pin};

    GPIO_t enable = {DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin};
    GPIO_t n_fault = {DRV_nFAULT_GPIO_Port, DRV_nFAULT_Pin};

    spi_handle_ = new SPIDevice(SPI2, mosi, miso, nss);
    gate_driver_ = new DRV8323(spi_handle_, enable, n_fault);

    // Setup Gate Driver
    spi_handle_ = new SPIDevice(SPI2, mosi, miso, nss);
    spi_handle_->Enable();

    gate_driver_ = new DRV8323(spi_handle_, enable, n_fault);
    gate_driver_->EnableDriver();
    osDelay(10);
    gate_driver_->Init();
    osDelay(10);
    gate_driver_->Calibrate();
    osDelay(10);
    
    // Load Configuration
    load_configuration();

    // Compute PWM Parameters
    pwm_counter_period_ticks_ = SystemCoreClock / (2 * config_.pwm_freq);

    // Update Controller Sample Time
    controller_loop_freq_ = (config_.pwm_freq / config_.foc_ccl_divider);
    controller_update_period_ = (1.0f) / controller_loop_freq_;

    //Logger::Instance().Print("\r\nPWM FREQ :%d\r\n", pwm_counter_period_ticks_);
    // Start PWM
    StartPWM();
    osDelay(150); // Delay for a bit to let things stabilize

    // Start ADCs
    StartADCs();
    osDelay(150); // Delay for a bit to let things stabilize

    EnablePWM(true);            // Start PWM
    zero_current_sensors(1024); // Measure current sensor zero-offset
    EnablePWM(false);           // Stop PWM

    Logger::Instance().Print("We out!\r\n");
    // Default Mode Idle:
    control_mode_ = IDLE_MODE;

    // Initialize RMS Current limiter
    uint32_t sub_sample_count = rms_current_sample_period_/controller_update_period_;
    current_limiter = new RMSCurrentLimiter(motor_->config_.continuous_current_max, motor_->config_.continuous_current_tau, rms_current_sample_period_, sub_sample_count);
    current_limiter->Reset();

    current_limiter->AddCurrentSample(10.0f);
    //Logger::Instance().Print("Count: %d.  %f to %f\n", sub_sample_count, motor_->config_.continuous_current_max, motor_->config_.continuous_current_tau);
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
void MotorController::StartControlFSM()
{
    // Start IDLE and PWM/Gate Driver Off
    static control_mode_type_t current_control_mode = IDLE_MODE;
    gate_driver_->DisableDriver();
    EnablePWM(false);
    for (;;)
    {
        if (control_mode_ != IDLE_MODE && CheckErrors())
        {
            Logger::Instance().Print("Bailing Here CHECK ERROR\n");
            control_mode_ = ERROR_MODE;
        }

        // Check Current Mode
        switch (control_mode_)
        {
        case (IDLE_MODE):
            if (current_control_mode != control_mode_)
            {
                current_control_mode = control_mode_;
                LEDService::Instance().Off();
                gate_driver_->DisableDriver();
                EnablePWM(false);

                Reset(); // Reset Controller to Zero
            }

            osDelay(1);
            break;
        case (ERROR_MODE):
            if (current_control_mode != control_mode_)
            {
                current_control_mode = control_mode_;
                //TODO: Flash a code... LEDService::Instance().Off();
                //printf("Controller Error: \r\n");

                gate_driver_->PrintFaults();

                Reset(); // Reset Controller to Zero

                gate_driver_->DisableDriver();
                EnablePWM(false);
            }

            osDelay(1);
            break;
        case (CALIBRATION_MODE):
            if (current_control_mode != control_mode_) // Need PWM Enabled for Calibration
            {
                current_control_mode = control_mode_;
                LEDService::Instance().On();
                gate_driver_->EnableDriver();
                EnablePWM(true);

                //printf("\r\n\r\nMotor Calibration Starting...\r\n\r\n");
                motor->Calibrate(motor_controller);
                UpdateControllerGains();

                // TODO: Check Errors
                //printf("\r\nMotor Calibration Complete.  Press ESC to return to menu.\r\n");
                control_mode_ = IDLE_MODE;
            }
            //printf("Calib Mode!\r\n");
            osDelay(1);
            break;
        case (MEASURE_RESISTANCE_MODE):
        {
            if (current_control_mode != control_mode_) // Need PWM Enabled for Calibration
            {
                current_control_mode = control_mode_;
                LEDService::Instance().On();
                gate_driver_->EnableDriver();
                EnablePWM(true);

                motor->MeasureMotorResistance(motor_controller, motor->config_.calib_current, motor->config_.calib_voltage);
                // TODO: Check Errors
                //printf("\r\nMotor Calibration Complete.  Press ESC to return to menu.\r\n");
                control_mode_ = IDLE_MODE;
            }
            osDelay(1);
            break;
        }
        case (MEASURE_INDUCTANCE_MODE):
        {
            if (current_control_mode != control_mode_) // Need PWM Enabled for Calibration
            {
                current_control_mode = control_mode_;
                LEDService::Instance().On();
                gate_driver_->EnableDriver();
                EnablePWM(true);
                NVIC_DisableIRQ(USART2_IRQn);
                motor->MeasureMotorInductance(motor_controller, -motor_->config_.calib_voltage, motor_->config_.calib_voltage);
                NVIC_EnableIRQ(USART2_IRQn);
                // TODO: Check Errors
                //printf("\r\nMotor Calibration Complete.  Press ESC to return to menu.\r\n");
                control_mode_ = IDLE_MODE;
            }
            osDelay(1);
            break;
        }
        case (MEASURE_PHASE_ORDER_MODE):
        {
            if (current_control_mode != control_mode_) // Need PWM Enabled for Calibration
            {
                current_control_mode = control_mode_;
                LEDService::Instance().On();
                gate_driver_->EnableDriver();
                EnablePWM(true);
                motor->OrderPhases(this);
                // TODO: Check Errors
                control_mode_ = IDLE_MODE;
            }
            osDelay(1);
            break;
        }
        case (MEASURE_ENCODER_OFFSET_MODE):
        {
            if (current_control_mode != control_mode_) // Need PWM Enabled for Calibration
            {
                current_control_mode = control_mode_;
                LEDService::Instance().On();
                gate_driver_->EnableDriver();
                EnablePWM(true);
                motor->CalibrateEncoderOffset(this);
                // TODO: Check Errors
                control_mode_ = IDLE_MODE;
            }
            osDelay(1);
            break;
        }
        case (FOC_CURRENT_MODE):
        case (FOC_VOLTAGE_MODE):
        case (FOC_TORQUE_MODE):
        case (FOC_SPEED_MODE):
        {
            if (current_control_mode != control_mode_)
            {
                current_control_mode = control_mode_;

                // Make sure we are calibrated:
                if (!motor_->config_.calibrated)
                {
                    //printf("Motor is NOT Calibrated.  Please run Motor Calibration before entering control modes!\r\n");
                    Logger::Instance().Print("NOt Calibrated ERROR\n");
                    control_mode_ = ERROR_MODE;
                    continue;
                }

                // Reset
                Reset();

                LEDService::Instance().On();
                gate_driver_->EnableDriver();
                EnablePWM(true);
            }
            if (osThreadFlagsWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, osFlagsWaitAny, CURRENT_MEASUREMENT_TIMEOUT) != CURRENT_MEASUREMENT_COMPLETE_SIGNAL)
            {
                // TODO: Should have a check for number of missed deadlines, then bail.  Leaky Integrator
                //printf("ERROR: Motor Controller Timeout!\r\n");
                control_mode_ = ERROR_MODE;
                continue;
            }

            // Measure
            // TODO: Transform to Id/Iq here?.  For now use last sample.  
            // TOOD: Do this in the current control loop
            // TODO: Also need to make this work for voltage mode Vrms=IrmsR should work hopefully
            static float I_sample = 0.0f;
            arm_sqrt_f32(motor_controller->state_.I_d * motor_controller->state_.I_d + motor_controller->state_.I_q * motor_controller->state_.I_q, &I_sample);
            // Update Current Limiter
            current_limiter->AddCurrentSample(I_sample);
            motor_controller->state_.I_rms = current_limiter->GetRMSCurrent();
            motor_controller->state_.I_max = current_limiter->GetMaxAllowableCurrent();

            DoMotorControl();
            break;
        }
        // case (ENCODER_DEBUG):
        //     if (current_control_mode != control_mode_)
        //     {
        //         current_control_mode = control_mode_;
        //         gate_driver_->Disable();
        //         EnablePWM(false);
        //     }
        //     motor->Update();
        //     motor->PrintPosition();
        //     osDelay(100);
        //     break;
        default:
            if (current_control_mode != control_mode_)
            {
                current_control_mode = control_mode_;
                LEDService::Instance().Off();
                gate_driver_->DisableDriver();
                EnablePWM(false);
            }
            Logger::Instance().Print("Unhandled Control Mode: %d!\r\n", control_mode_);
            osDelay(1);
            break;
        }
    }
}
void MotorController::DoMotorControl()
{
    NVIC_DisableIRQ(USART2_IRQn);
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
    NVIC_EnableIRQ(USART2_IRQn);
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
    LL_TIM_SetRepetitionCounter(TIM8, 1);//(config_.foc_ccl_divider * 2) - 1);     // Loop Counter Decimator
    
    // Set Zer0 Duty Cycle
    SetDuty(0.5f, 0.5f, 0.5f);  // Zero Duty

    // Make Sure PWM is initially Disabled
    EnablePWM(false);

    // This makes sure PWM is stopped if we have debug point/crash
    __HAL_DBGMCU_FREEZE_TIM1();
}
void MotorController::StartADCs()
{
    // // ADC Setup
    // RCC->APB2ENR |= RCC_APB2ENR_ADC3EN; // clock for ADC3
    // RCC->APB2ENR |= RCC_APB2ENR_ADC2EN; // clock for ADC2
    // RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // clock for ADC1

    // RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable clock for GPIOC
    // RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable clock for GPIOA

    // ADC->CCR = 0x00000016;      // Regular simultaneous mode only
    // ADC1->CR2 |= ADC_CR2_ADON;  //0x00000001;                    // ADC1 ON
    // ADC1->SQR3 = 0x000000A;     // use PC_0 as input- ADC1_IN0
    // ADC2->CR2 |= ADC_CR2_ADON;  //0x00000001;                    // ADC2 ON
    // ADC2->SQR3 = 0x0000000B;    // use PC_1 as input - ADC2_IN11
    // ADC3->CR2 |= ADC_CR2_ADON;  // ADC3 ON
    // ADC3->SQR3 = 0x00000000;    // use PA_0, - ADC3_IN0
    // GPIOC->MODER |= 0x0000000f; // Alternate function, PC_0, PC_1 are analog inputs
    // GPIOA->MODER |= 0x3;        // PA_0 as analog input

    // ADC1->SMPR1 |= 0x1; // 15 cycles on CH_10, 0b 001
    // ADC2->SMPR1 |= 0x8; // 15 cycles on CH_11, 0b 0001 000
    // ADC3->SMPR2 |= 0x1; // 15 cycles on CH_0, 0b 001;

    EnableADC(ADC1);
    EnableADC(ADC2);
    EnableADC(ADC3);
    //EnableADC(ADC4);
    //EnableADC(ADC5);

    // Turn on Interrupts for ADC3 as it is not shared.
    // Should Generate less Interrupts
    LL_ADC_EnableIT_EOC(ADC3);

    LL_ADC_REG_StartConversion(ADC1);
    LL_ADC_REG_StartConversion(ADC2);
    LL_ADC_REG_StartConversion(ADC3);

}

void MotorController::EnablePWM(bool enable)
{
    if (enable)
    {
        LL_TIM_EnableAllOutputs(TIM8); // Advanced Timers turn on Outputs
        //TIM1->CR1 ^= TIM_CR1_UDIS;
        //TIM1->BDTR |= (TIM_BDTR_MOE);
    }
    else // Disable PWM Timer Unconditionally
    {
        LL_TIM_DisableAllOutputs(TIM8); // Advanced Timers turn on Outputs
        //TIM1->CR1 |= TIM_CR1_UDIS;
        //TIM1->BDTR &= ~(TIM_BDTR_MOE);
    }
    osDelay(100);

    // TODO: Here for when project ported from MBED to CUBEMX
    //enable ? __HAL_TIM_MOE_ENABLE(&htim8) : __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
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
    { /
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

    float cf = arm_cos_f32(theta);
    float sf = arm_sin_f32(theta);
    *d = 0.6666667f * (cf * a + (0.86602540378f * sf - .5f * cf) * b + (-0.86602540378f * sf - .5f * cf) * c); ///Faster DQ0 Transform
    *q = 0.6666667f * (-sf * a - (-0.86602540378f * cf - .5f * sf) * b - (0.86602540378f * cf - .5f * sf) * c);
}

void MotorController::ParkInverseTransform(float theta, float d, float q, float *alpha, float *beta)
{
    float cos_theta = arm_cos_f32(theta);
    float sin_theta = arm_sin_f32(theta);

    *alpha = d * cos_theta - q * sin_theta;
    *beta = q * cos_theta + d * sin_theta;
}
void MotorController::ParkTransform(float theta, float alpha, float beta, float *d, float *q)
{
    float cos_theta = arm_cos_f32(theta);
    float sin_theta = arm_sin_f32(theta);

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

    //Logger::Instance().Print("VD: %f, VQ: %f!\r\n", v_d, v_q);
   //  Logger::Instance().Print("Alpha: %f, Beta: %f!\r\n", v_alpha, v_beta);
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
