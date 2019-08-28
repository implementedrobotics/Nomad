/*
 * MotorConroller.cpp
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

// Project Includes
#include "mbed.h"
#include "rtos.h"
#include "FastPWM.h"
#include "Motor.h"
#include "../../math_ops.h"


Motor *motor = 0;
MotorController *motor_controller = 0;

// Globals
static int32_t g_adc1_offset;
static int32_t g_adc2_offset;

extern "C"
{
#include "motor_controller_interface.h"
}
void motor_controller_thread_entry()
{
    printf("Motor RT Controller Task Up.\n\r");

    // Init Motor and Implicitly Position Sensor
    motor = new Motor(CONTROL_LOOP_PERIOD, 100, 21);

    // Init Motor Controller
    motor_controller = new MotorController(motor, CONTROL_LOOP_PERIOD);
    motor_controller->Init();

    // Begin Control Loop
    motor_controller->StartControlFSM();
}

// Controller Mode Interface
void set_control_mode(int mode)
{
    motor_controller->SetControlMode((control_mode_type_t)mode);
}

void current_measurement_cb()
{
    // Measure Currents/Bus Voltage
    int32_t adc2_raw = ADC2->DR; // Current Sense Measurement 1
    int32_t adc1_raw = ADC1->DR; // Current Sense Measurement 2
    int32_t adc3_raw = ADC3->DR; // Voltage Bus Measurement.  TODO: Move this to a different/slower timer

    // TODO: Not sure this is necessasry?
    if (motor->config_.phase_order) // Check Phase Ordering
    {
        motor->state_.I_b = current_scale * (float)(adc2_raw - g_adc2_offset);
        motor->state_.I_c = current_scale * (float)(adc1_raw - g_adc1_offset);
    }
    else
    {
        motor->state_.I_b = current_scale * (float)(adc1_raw - g_adc1_offset);
        motor->state_.I_c = current_scale * (float)(adc2_raw - g_adc2_offset);
    }
    // Kirchoffs Current Law to compute 3rd unmeasured current.
    motor->state_.I_a = -motor->state_.I_b - motor->state_.I_c;

    // Always Update Motor State
    motor->Update();
    // TODO: Filter v_bus current measurements
    motor_controller->voltage_bus_ = 0.95f * motor_controller->voltage_bus_ + 0.05f * ((float)adc3_raw) * voltage_scale;

    // Make sure control thread is ready

    if (motor_controller != 0 && motor_controller->ControlThreadReady())
    {
       // printf("SIGNAL SEND!\r\n");
        osSignalSet(motor_controller->GetThreadID(), CURRENT_MEASUREMENT_COMPLETE_SIGNAL);
    }
}

bool zero_current_sensors(uint32_t num_samples)
{
    // Zero Offsets
    g_adc1_offset = 0;
    g_adc2_offset = 0;

    // Set Idle/"Zero" PWM
    motor_controller->SetDuty(0.5f, 0.5f, 0.5f);
    for (uint32_t i = 0; i < num_samples; i++) // Average num_samples of the ADC
    {
        osEvent test;
        if ((test = osSignalWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, CURRENT_MEASUREMENT_TIMEOUT)).status != osEventSignal)
        {
            // TODO: Error here for timing
            printf("ERROR: Zero Current FAILED!\r\n");
            return false;
        }
        g_adc2_offset += ADC2->DR;
        g_adc1_offset += ADC1->DR;
    }
    g_adc1_offset = g_adc1_offset / num_samples;
    g_adc2_offset = g_adc2_offset / num_samples;
    printf("ADC OFFSET: %d and %d\r\n", g_adc1_offset, g_adc2_offset);
    return true;
}

bool calibrate_motor()
{
    printf("Motor Calibrate Begin.\n\r");

    // Make sure we have no PWM period
	motor_controller->SetDuty(0.5f,0.5f,0.5f);

    // Turn on PWM outputs
	motor_controller->EnablePWM(true);

    motor->Calibrate(motor_controller);

    // Shutdown the phases
	motor_controller->SetDuty(0.5f,0.5f,0.5f);

	// Turn off PWM outputs
	motor_controller->EnablePWM(false); // enable pwm outputs

	//osDelay(100);
    
    printf("Motor Control Calibrate End.\n\r");
    return true;
}
// Control Loop Timer Interrupt Synced with PWM
extern "C" void TIM1_UP_TIM10_IRQHandler(void)
{
    if (TIM1->SR & TIM_SR_UIF) // TIM1 is up, and update interrupts are enabled on TIM1
    {
        ADC1->CR2 |= 0x40000000;  // Begin ADC Conversion
        current_measurement_cb(); // Callback
    }
    TIM1->SR = 0x0; // reset the status register
}

MotorController::MotorController(Motor *motor, float sample_time) : controller_update_period_(sample_time), motor_(motor)
{
    //current_meas_freq_ = CURRENT_LOOP_FREQ;
    control_thread_id_ = 0;
    control_thread_ready_ = false;
    control_initialized_ = false;
    control_enabled_ = false;
}

void MotorController::Init()
{
    printf("MotorController::Init() - Motor Controller Initializing...\n\r");

    // Update Control Thread State
    control_thread_id_ = osThreadGetId();
    control_thread_ready_ = true;

    // Compute Maximum Allowed Current
    float margin = 0.90f;
    float max_input = margin * 0.3f * SENSE_CONDUCTANCE;
    float max_swing = margin * 1.6f * SENSE_CONDUCTANCE * (1.0f / CURRENT_SENSE_GAIN);
    current_max_ = fminf(max_input, max_swing);

    // Setup Gate Driver
    spi_handle_ = new SPI(PA_7, PA_6, PA_5);
    cs_ = new DigitalOut(PA_4);
    gate_enable_ = new DigitalOut(PA_11);
    gate_driver_ = new DRV832x(spi_handle_, cs_);

    gate_enable_->write(1);
    wait_us(100);
    gate_driver_->calibrate();
    wait_us(100);
    gate_driver_->write_DCR(0x0, 0x0, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1);
    wait_us(100);
    gate_driver_->write_CSACR(0x0, 0x1, 0x0, CSA_GAIN_40, 0x0, 0x0, 0x0, 0x0, SEN_LVL_1_0);
    wait_us(100);
    gate_driver_->write_OCPCR(TRETRY_4MS, DEADTIME_200NS, OCP_RETRY, OCP_DEG_8US, VDS_LVL_1_88);

    // Start PWM
    StartPWM();
    osDelay(150); // Delay for a bit to let things stabilize

    // Start ADCs
    StartADCs();
    osDelay(150); // Delay for a bit to let things stabilize

    //gate_driver_->enable_gd();
    EnablePWM(true);            // Start PWM
    SetDuty(0.5f, 0.5f, 0.5f);  // Zero Duty
    zero_current_sensors(1024); // Measure current sensor zero-offset
    EnablePWM(false);           // Stop PWM


    // TODO: This should only happen when commanded.  Just Testing
    // Do Motor Calibration
    calibrate_motor();

    // TODO: Compute Loop Gains With measured motor parameters

    // Check for Calibration of Motor (Could be loaded)
    // if(!motor_->is_calibrated_)
    // {
    // 	DoMotorCalibration();
    // }

    // Default Mode Idle:
    gate_driver_->disable_gd(); // Disable Gate Driver.  TODO: A Bit Reduntant
    control_mode_ = IDLE_MODE;
    control_initialized_ = true;
    printf("MotorController::Init() - Motor Controller Initialized Successfully!\n\r");
}

void MotorController::StartControlFSM()
{
    printf("STARTING FSM\r\n");
    // TODO: Check for DRV Errors Here -> ERROR MODE
    static control_mode_type_t current_control_mode = IDLE_MODE;
    for (;;)
    {
        // Check Current Mode
        switch (control_mode_)
        {
        case (IDLE_MODE):
            if (current_control_mode != control_mode_)
            {
                current_control_mode = control_mode_;
                gate_driver_->disable_gd();
                EnablePWM(false);
            }
            //printf("IDLE Mode!\r\n");
            osDelay(1);
            break;
        case (ERROR_MODE):
            if (current_control_mode != control_mode_)
            {
                current_control_mode = control_mode_;
                gate_driver_->disable_gd();
                EnablePWM(false);
            }
            printf("Error Mode!\r\n");
            osDelay(1);
            break;
        case (FOC_CURRENT_MODE):
        case (FOC_VOLTAGE_MODE):
        case (FOC_TORQUE_MODE):
            if (current_control_mode != control_mode_)
            {
                current_control_mode = control_mode_;
                gate_driver_->enable_gd();
                EnablePWM(true);
            }
            if (osSignalWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, CURRENT_MEASUREMENT_TIMEOUT).status != osEventSignal)
            {
                // TODO: Should have a check for number of missed deadlines, then bail
                printf("ERROR: Motor Controller Timeout!\r\n");
                control_mode_ = ERROR_MODE;
            }
            DoMotorControl();
            break;
        default:
            if (current_control_mode != control_mode_)
            {
                current_control_mode = control_mode_;
                gate_driver_->disable_gd();
                EnablePWM(false);
            }
            printf("Unhandled Control Mode: %d!\r\n", control_mode_);
            osDelay(1);
            break;
        }
    }
}
void MotorController::DoMotorControl()
{
    //control_enabled_ = true;
    //while (control_enabled_) // Do Forever
    //{
    // TODO: Error Checking
    //if (osSignalWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, CURRENT_MEASUREMENT_TIMEOUT).status != osEventSignal)
    //{
    // TODO: Error here for timing
    //printf("ERROR: Motor Controller Timeout!\r\n");
    //break;
    //}
    //}
}

void MotorController::StartPWM()
{
    // TODO: I think this does not belong here
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable the clock to GPIOC
    RCC->APB1ENR |= 0x00000001;          // Enable TIM2 clock (TODO: What is on TIM2?)
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;  // Enable TIM1 clock

    // TODO: Move to main, elsewhere
    //GPIOC->MODER |= (1 << 10); // set pin 5 to be general purpose output for LED

    // Setup PWM Pins
    PWM_u_ = new FastPWM(PIN_U);
    PWM_v_ = new FastPWM(PIN_V);
    PWM_w_ = new FastPWM(PIN_W);

    // Enable Interrupt Service Routines:
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn); //Enable TIM1/10 IRQ

    // Set Priority
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2); // Commutation is highest priority interrupt

    TIM1->DIER |= TIM_DIER_UIE; // Enable Update Interrupt
    TIM1->CR1 = 0x40;           // CMS = 10, Interrupt only when counting up
    //TIM1->CR1 |= TIM_CR1_UDIS;  // Start Update Disable (TODO: Refactor to our "Enable PWM")
    TIM1->CR1 |= TIM_CR1_ARPE; // Auto Reload Timer
    TIM1->RCR |= 0x001;        // Update event once per up count and down count.  This can be modified to have the control loop run at lower rates.
    TIM1->EGR |= TIM_EGR_UG;   // Generate an update event to reload the Prescaler/Repetition Counter immediately

    // PWM Setup
    TIM1->PSC = 0x0;                      // Set Prescaler to none.  Timer will count in sync with APB Block
    TIM1->ARR = PWM_COUNTER_PERIOD_TICKS; // Set Auto Reload Timer Value.  TODO: User Configurable.  For now 40khz.
    TIM1->CCER |= ~(TIM_CCER_CC1NP);      // Interupt when low side is on.
    TIM1->CR1 |= TIM_CR1_CEN;             // Enable TIM1

    // Start Disabled
    EnablePWM(false);

    // This makes sure PWM is stopped if we have debug point/crash
    __HAL_DBGMCU_FREEZE_TIM1();
}
void MotorController::StartADCs()
{
    // ADC Setup
    RCC->APB2ENR |= RCC_APB2ENR_ADC3EN; // clock for ADC3
    RCC->APB2ENR |= RCC_APB2ENR_ADC2EN; // clock for ADC2
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // clock for ADC1

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable clock for GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable clock for GPIOA

    ADC->CCR = 0x00000016;      // Regular simultaneous mode only
    ADC1->CR2 |= ADC_CR2_ADON;  //0x00000001;                    // ADC1 ON
    ADC1->SQR3 = 0x000000A;     // use PC_0 as input- ADC1_IN0
    ADC2->CR2 |= ADC_CR2_ADON;  //0x00000001;                    // ADC2 ON
    ADC2->SQR3 = 0x0000000B;    // use PC_1 as input - ADC2_IN11
    ADC3->CR2 |= ADC_CR2_ADON;  // ADC3 ON
    ADC3->SQR3 = 0x00000000;    // use PA_0, - ADC3_IN0
    GPIOC->MODER |= 0x0000000f; // Alternate function, PC_0, PC_1 are analog inputs
    GPIOA->MODER |= 0x3;        // PA_0 as analog input

    ADC1->SMPR1 |= 0x1; // 15 cycles on CH_10, 0b 001
    ADC2->SMPR1 |= 0x8; // 15 cycles on CH_11, 0b 0001 000
    ADC3->SMPR2 |= 0x1; // 15 cycles on CH_0, 0b 001;
}

void MotorController::EnablePWM(bool enable)
{
    if (enable)
    {
        TIM1->CR1 ^= TIM_CR1_UDIS;
        //TIM1->BDTR |= (TIM_BDTR_MOE);
    }
    else // Disable PWM Timer Unconditionally
    {
        TIM1->CR1 |= TIM_CR1_UDIS;
        //TIM1->BDTR &= ~(TIM_BDTR_MOE);
    }
    osDelay(100);

    // TODO: Here for when project ported from MBED to CUBEMX
    //enable ? __HAL_TIM_MOE_ENABLE(&htim8) : __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
}

void MotorController::SetDuty(float duty_U, float duty_V, float duty_W)
{
    // TODO: We should just reverse the "encoder direcion to simplify this"
    if (motor_->config_.phase_order)
    {                                                                        // Check which phase order to use,
        TIM1->CCR3 = (uint16_t)(PWM_COUNTER_PERIOD_TICKS) * (1.0f - duty_U); // Write duty cycles
        TIM1->CCR2 = (uint16_t)(PWM_COUNTER_PERIOD_TICKS) * (1.0f - duty_V);
        TIM1->CCR1 = (uint16_t)(PWM_COUNTER_PERIOD_TICKS) * (1.0f - duty_W);
    }
    else
    {
        TIM1->CCR3 = (uint16_t)(PWM_COUNTER_PERIOD_TICKS) * (1.0f - duty_U);
        TIM1->CCR1 = (uint16_t)(PWM_COUNTER_PERIOD_TICKS) * (1.0f - duty_V);
        TIM1->CCR2 = ((uint16_t)PWM_COUNTER_PERIOD_TICKS) * (1.0f - duty_W);
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

void MotorController::SVM(float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w)
{
    // Space Vector Modulation
    // u,v,w amplitude = Bus Voltage for Full Modulation Depth
    float v_offset = (fminf3(u, v, w) + fmaxf3(u, v, w)) * 0.5f;

    *dtc_u = fminf(fmaxf(((u - v_offset) / voltage_bus_ + 0.5f), DTC_MIN), DTC_MAX);
    *dtc_v = fminf(fmaxf(((v - v_offset) / voltage_bus_ + 0.5f), DTC_MIN), DTC_MAX);
    *dtc_w = fminf(fmaxf(((w - v_offset) / voltage_bus_ + 0.5f), DTC_MIN), DTC_MAX);
}
