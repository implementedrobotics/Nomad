/*
 * Motor.cpp
 *
 *  Created on: August 26, 2019
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
#include "Motor.h"

// C System Files

// C++ System Files

// Project Includes
#include "mbed.h"
#include "MotorController.h"
#include "../../math_ops.h"

Motor::Motor(float sample_time, float K_v, uint32_t pole_pairs) : sample_time_(sample_time),
                                                                  dirty_(false)
{
    // Zero State
    memset(&state_, 0, sizeof(state_));

    // TODO: Check for FLASH, otherwise defaults

    // Setup Default Configs
    config_.num_pole_pairs = pole_pairs;
    config_.phase_resistance = 0.0f;
    config_.phase_inductance_d = 0.0f;
    config_.phase_inductance_q = 0.0f;
    config_.phase_order = 0;
    config_.calibrated = false;

    // Update KV Calulations
    SetKV(K_v);

    // Setup Position Sensor
    rotor_sensor_ = new PositionSensorAS5x47(sample_time_, config_.num_pole_pairs);
}

void Motor::Update()
{
    // Update Position Sensor
    rotor_sensor_->Update(sample_time_);

    // Update State
    state_.theta_mech = rotor_sensor_->GetMechanicalPosition();
    state_.theta_mech_true = rotor_sensor_->GetMechanicalPositionTrue();
    state_.theta_mech_dot = rotor_sensor_->GetMechanicalVelocity();
    state_.theta_elec = rotor_sensor_->GetElectricalPosition();
    state_.theta_elec_dot = rotor_sensor_->GetElectricalVelocity();

    // Update Temparature Observer
}

void Motor::PrintPosition()
{
    printf(" Mechanical Angle:  %f/%f    Electrical Angle:  %f    Raw:  %ld\n\r", state_.theta_mech, state_.theta_mech_true, state_.theta_elec, rotor_sensor_->GetRawPosition());
}
bool Motor::Calibrate(MotorController *controller)
{

    controller->SetDuty(0.5f, 0.5f, 0.5f); // Make sure we have no PWM period

    // TODO: Check Error Here
    // Measure Resistance
    MeasureMotorResistance(controller, 15.0f, 3.0f);

    // AND Here
    // Measure Inductance
    MeasureMotorInductance(controller, -2.0f, 2.0f);

    controller->SetDuty(0.5f, 0.5f, 0.5f); // Make sure we have no PWM period

    // Order Phases
    OrderPhases(controller);

    // Offset Calibration
    CalibrateEncoderOffset(controller);

    config_.calibrated = true; // Update Flag
    dirty_ = true;

    return true;
}

bool Motor::MeasureMotorInductance(MotorController *controller, float voltage_low, float voltage_high)
{
    printf("Measure Motor Inductance:\n\r");

    float test_voltages[2] = {voltage_low, voltage_high};
    float Ialphas[2] = {0.0f};
    float U = 0, V = 0, W = 0;
    float dtc_U = 0, dtc_V = 0, dtc_W = 0;

    static const int num_cycles = 1.5f / sample_time_; // Test runs for 3s;

    // Shutdown Phases
    controller->SetDuty(0.5f, 0.5f, 0.5f);

    for (int t = 0; t < num_cycles; ++t)
    {
        for (int i = 0; i < 2; ++i)
        {
            osEvent evt = osSignalWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, CURRENT_MEASUREMENT_TIMEOUT);
            if (evt.status != osEventSignal)
            {
                // motor->error = ERROR_PHASE_INDUCTANCE_MEASUREMENT_TIMEOUT;
                printf("ERROR: Phase Inductance Measurement Timeout\n\r");
                return false;
            }
            if (i == 1) // When you step you are reading the previous step.  TODO: Make this Better!
                Ialphas[0] += state_.I_a;
            else
                Ialphas[1] += state_.I_a;

            // Test voltage along phase A
            controller->dqInverseTransform(0.0f, test_voltages[i], 0.0f, &U, &V, &W);
            controller->SVM(U, V, W, &dtc_U, &dtc_V, &dtc_W);
            controller->SetDuty(dtc_U, dtc_V, dtc_W);
            //SetVoltageTimings(test_voltages[i], 0.0f);
        }
    }

    // Shutdown phases
    controller->SetDuty(0.5f, 0.5f, 0.5f);
    //printf("I Alpha: %f/%f\n\r", Ialphas[0], Ialphas[1]);
    float v_L = 0.5f * (voltage_high - voltage_low);

    // Note: A more correct formula would also take into account that there is a finite timestep.
    // However, the discretisation in the current control loop inverts the same discrepancy
    float dI_by_dt = (Ialphas[1] - Ialphas[0]) / (sample_time_ * (float)num_cycles);
    float L = v_L / dI_by_dt;

    // TODO arbitrary values set for now
    if (L < 1e-6f || L > 500e-6f)
    {
        //motor->error = ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE;
        printf("ERROR: Inductance Measurement Out of Range: %f\n\r", L);
        return false;
    }

    // PMSM D/Q Inductance are the same.
    config_.phase_inductance_d = L;
    config_.phase_inductance_q = L;
    printf("Phase Inductance: %f Henries\r\n", config_.phase_inductance_d);

    return true;
}

bool Motor::MeasureMotorResistance(MotorController *controller, float test_current, float max_voltage)
{
    static const float kI = 10.0f;                          // [(V/s)/A]
    static const int num_test_cycles = 3.0f / sample_time_; // Test runs for 3s
    float test_voltage = 0.0f;
    float U = 0, V = 0, W = 0;
    float dtc_U = 0, dtc_V = 0, dtc_W = 0;
    //int sum = 0;
    printf("Measure Motor Resistance:\n\r");

    for (int i = 0; i < num_test_cycles; ++i)
    {
        osEvent evt = osSignalWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, CURRENT_MEASUREMENT_TIMEOUT);
        if (evt.status != osEventSignal)
        {
            // motor->error = ERROR_PHASE_RESISTANCE_MEASUREMENT_TIMEOUT;
            printf("ERROR: Phase Resistance Measurement Timeout\n\r");
            return false;
        }

        float Ialpha = state_.I_a;
        test_voltage += (kI * sample_time_) * (test_current - Ialpha);

        if (test_voltage > max_voltage)
            test_voltage = max_voltage; // Clamp Current
        if (test_voltage < -max_voltage)
            test_voltage = -max_voltage; // Clamp Current

        // Test voltage along phase A
        controller->dqInverseTransform(0.0f, test_voltage, 0.0f, &U, &V, &W);
        controller->SVM(U, V, W, &dtc_U, &dtc_V, &dtc_W);
        controller->SetDuty(dtc_U, dtc_V, dtc_W);
        //controller->ParkInverseTransform(0.0f, test_voltage, 0.0f,  &v_alpha, &v_beta); // No need for a transform A axis = Alpha axis
        //controller->ClarkeInverseTransform(v_alpha, v_beta,
        //SetVoltageTimings(test_voltage, 0.0f);
    }

    float R = test_voltage / test_current;
    if (fabs(test_voltage) == fabs(max_voltage) || R < 0.01f || R > 1.0f)
    {
        // motor->error = ERROR_PHASE_RESISTANCE_OUT_OF_RANGE;
        printf("ERROR: Resistance Measurement Out of Range: %f\n\r", R);
        return false;
    }
    config_.phase_resistance = R;
    printf("Phase Resistance: %f ohms\r\n", config_.phase_resistance);

    // Shutdown the phases
    controller->SetDuty(0.5f, 0.5f, 0.5f);

    osDelay(200);
    return true;
}

bool Motor::OrderPhases(MotorController *controller)
{
    printf("\n\rRunning phase direction scan.\n\r");

    float rotor_lock_duration = 2.0f; // Time needed for rotor to lock and settle on D-Axis

    float theta_start = 0;
    float theta_end = 0;
    float U, V, W = 0;
    float dtc_U, dtc_V, dtc_W = 0.5f; // Default to idle
    float test_voltage = 1.0f;

    float scan_step_size = 1.0f/5000.0f; // Amount to step in open loop
	float scan_range = 4.0f * M_PI; // Scan range for phase order (electrical range)

    printf("Locking Rotor to D-Axis:\n\r");
	// go to encoder zero phase for rotor_lock_duration to get ready to scan
	for (int i = 0; i < rotor_lock_duration*(float)sample_time_; ++i) {
		if (osSignalWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, CURRENT_MEASUREMENT_TIMEOUT).status != osEventSignal) {
			return false;
		}
        controller->dqInverseTransform(0.0f, test_voltage, 0.0f, &U, &V, &W); // Test voltage to D-Axis
        controller->SVM(U, V, W, &dtc_U, &dtc_V, &dtc_W);
        controller->SetDuty(dtc_U, dtc_V, dtc_W);
        //controller->SetModulationOutput();
		//ParkInverseTransform(0.0f, calib_voltage, v_q, &v_alpha, &v_beta);
		//SetVoltageTimings(calib_voltage, 0.0f);
	}

    printf("Rotor stabilized.  Running phase direciton scan: \n\r");
    
    Update(); // Update State/Position Sensor

    osDelay(1); // Wait a ms

    for(float ref_angle = 0; ref_angle < scan_range; ref_angle += scan_step_size)
	{
		//for (int i = 0; i < step_dt*(float)current_meas_freq_; ++i) {
			if (osSignalWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, CURRENT_MEASUREMENT_TIMEOUT).status != osEventSignal) {
				return false;
			}
            controller->dqInverseTransform(ref_angle, test_voltage, 0.0f, &U, &V, &W);
            controller->SVM(U, V, W, &dtc_U, &dtc_V, &dtc_W);
            controller->SetDuty(dtc_U, dtc_V, dtc_W);

            Update(); // Update State/Position Sensor

            if(ref_angle == 0) // TODO: This seems unnecessary.  Since rotor is already locked we should just be able to do this outside the loop.
            {
                theta_start = state_.theta_mech_true;
            }
			//ParkInverseTransform(ref_angle, calib_voltage, 0.0f,  &v_alpha, &v_beta);
			//SetVoltageTimings(v_alpha, v_beta);
		//}
	}
    theta_end = state_.theta_mech_true;

    printf("Angle Start: %f, Angle End: %f\n\r", theta_start, theta_end);
	if(theta_end - theta_start > 0)
	{
		printf("Phase Order is correct!\n\r");
        rotor_sensor_->SetDirection(1);
        config_.phase_order = 1;
	}
	else
	{
		printf("Phase Order is incorrect!\n\r");
		rotor_sensor_->SetDirection(-1);
        config_.phase_order = 0;
	}
    return true;
}

bool Motor::CalibrateEncoderOffset(MotorController *controller)
{
    printf("\n\rRunning Encoder Offset/Eccentricity Calibration.\n\r");

    float *error_forward;       // Error Vector Forward Rotation
    float *error_backward;      // Error Vector Backward Rotation
    int32_t *lookup_table;      // Lookup Table
    int32_t *raw_forward;
    int32_t *raw_backward;
    float *error;
    float *error_filtered;

    const int32_t window = 128;
    const int32_t num_samples = 128 * config_.num_pole_pairs;    // Num samples per mechanical rotation.  Multiple of NPP for filtering reasons (see later)
    const int32_t sub_samples = 40;                     // increments between saved samples (for smoothing motion)
    float delta = 2 * PI * config_.num_pole_pairs / (num_samples * sub_samples); // change in angle between samples

    error_forward = new float[num_samples]();  
    error_backward = new float[num_samples]();

    error = new float[num_samples];
    error_filtered = new float[num_samples];
    //float cogging_current[window] = {0};

    const int32_t num_lookups = 128;
    lookup_table = new int32_t[num_lookups]; // Clear the previous lookup table.
    memset(&lookup_table, 0, sizeof(lookup_table));
    rotor_sensor_->SetOffsetLUT(lookup_table);
    raw_forward = new int32_t[num_samples]();
    raw_backward = new int32_t[num_samples]();

    float rotor_lock_duration = 2.0f; // Time needed for rotor to lock and settle on D-Axis

    float theta_ref = 0;
    float theta_actual = 0;
    float U, V, W = 0;
    float dtc_U, dtc_V, dtc_W = 0.5f; // Default to idle
    float v_d = 1.0f;
    float v_q = 0.0f;

    printf("Locking Rotor to D-Axis:\n\r");
	// Go to encoder zero phase for rotor_lock_duration to get ready for calibraion
	for (int32_t i = 0; i < rotor_lock_duration*(float)sample_time_; ++i) {
		if (osSignalWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, CURRENT_MEASUREMENT_TIMEOUT).status != osEventSignal) {
			return false;
		}
        controller->dqInverseTransform(0.0f, v_d, v_q, &U, &V, &W); // Test voltage to D-Axis
        controller->SVM(U, V, W, &dtc_U, &dtc_V, &dtc_W);
        controller->SetDuty(dtc_U, dtc_V, dtc_W);
        //controller->SetModulationOutput();
		//ParkInverseTransform(0.0f, calib_voltage, v_q, &v_alpha, &v_beta);
		//SetVoltageTimings(calib_voltage, 0.0f);
	}
    printf("Rotor stabilized.  Running encoder offset calibration: \n\r");
    Update(); // Update State/Position Sensor
    osDelay(1); // Wait a ms

    // TODO: Cogging Current

    // Rotate Forward
    for (int32_t i = 0; i < num_samples; i++)
    {
        for (int32_t j = 0; j < sub_samples; j++)
        {
            // if (osSignalWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, CURRENT_MEASUREMENT_TIMEOUT).status != osEventSignal) {
			//     return false;
		    // }
            theta_ref += delta;
            
            controller->dqInverseTransform(theta_ref, v_d, v_q, &U, &V, &W); // Test voltage to D-Axis
            controller->SVM(U, V, W, &dtc_U, &dtc_V, &dtc_W);
            controller->SetDuty(dtc_U, dtc_V, dtc_W);

            wait_us(100);

            Update(); // Update State/Position Sensor
        }
        Update(); // Update State/Position Sensor

        theta_actual = rotor_sensor_->GetMechanicalPositionTrue(); // Get Mechanical Position
        error_forward[i] = theta_ref / config_.num_pole_pairs - theta_actual;
        raw_forward[i] = rotor_sensor_->GetRawPosition();
        printf("%.4f   %.4f    %ld\n\r", theta_ref / (config_.num_pole_pairs), theta_actual, raw_forward[i]);
        //theta_ref += delta;
    }
    // Rotate Backwards
    for (int32_t i = 0; i < num_samples; i++)
    {
        for (int32_t j = 0; j < sub_samples; j++)
        {
            // if (osSignalWait(CURRENT_MEASUREMENT_COMPLETE_SIGNAL, CURRENT_MEASUREMENT_TIMEOUT).status != osEventSignal) {
			//     return false;
		    // }
            theta_ref -= delta;
            
            controller->dqInverseTransform(theta_ref, v_d, v_q, &U, &V, &W); // Test voltage to D-Axis
            controller->SVM(U, V, W, &dtc_U, &dtc_V, &dtc_W);
            controller->SetDuty(dtc_U, dtc_V, dtc_W);

            wait_us(100);

            Update(); // Update State/Position Sensor
        }
        Update(); // Update State/Position Sensor

        theta_actual = rotor_sensor_->GetMechanicalPositionTrue(); // Get Mechanical Position
        error_backward[i] = theta_ref / config_.num_pole_pairs - theta_actual;
        raw_backward[i] = rotor_sensor_->GetRawPosition();
        printf("%.4f   %.4f    %ld\n\r", theta_ref / (config_.num_pole_pairs), theta_actual, raw_backward[i]);
        //theta_ref -= delta;
    }

    // Compute Electrical Offset
    float offset = 0;
    for (int32_t i = 0; i < num_samples; i++)
    {
        offset += (error_forward[i] + error_backward[num_samples - 1 - i]) / (2.0f * num_samples); // calclate average position sensor offset
    }
    offset = fmod(offset * config_.num_pole_pairs, 2 * PI); // convert mechanical angle to electrical angle

    rotor_sensor_->SetElectricalOffset(offset); // Set Offset

    // Perform filtering to linearize position sensor eccentricity
    // FIR n-sample average, where n = number of samples in one electrical cycle
    // This filter has zero gain at electrical frequency and all integer multiples
    // So cogging effects should be completely filtered out.

    float mean = 0;
    // Average Forward and Backward Directions
    for (int32_t i = 0; i < num_samples; i++)
    { 
        error[i] = 0.5f * (error_forward[i] + error_backward[num_samples - i - 1]);
    }

    for (int32_t i = 0; i < num_samples; i++)
    {
        for (int32_t j = 0; j < window; j++)
        {
            int32_t index = -window / 2 + j + i; // Indices from -window/2 to + window/2
            if (index < 0)
            {
                index += num_samples;
            } // Moving average wraps around
            else if (index > num_samples - 1)
            {
                index -= num_samples;
            }
            error_filtered[i] += error[index] / (float)window;
        }
        // if (i < window)
        // {
        //     cogging_current[i] = current * sinf((error[i] - error_filtered[i]) * config_.num_pole_pairs);
        // }
        //printf("%.4f   %4f    %.4f   %.4f\n\r", error[i], error_filt[i], error_f[i], error_b[i]);
        mean += error_filtered[i] / num_samples;
    }

    int32_t raw_offset = (raw_forward[0] + raw_backward[num_samples - 1]) / 2; //Insensitive to errors in this direction, so 2 points is plenty

    printf("\n\r Encoder non-linearity compensation table\n\r");
    printf(" Sample Number : Lookup Index : Lookup Value\n\r\n\r");
    for (int32_t i = 0; i < num_lookups; i++)
    { // build lookup table
        int32_t index = (raw_offset >> 7) + i;
        if (index > (num_lookups - 1))
        {
            index -= num_lookups;
        }
        lookup_table[index] = (int32_t)((error_filtered[i * config_.num_pole_pairs] - mean) * (float)(rotor_sensor_->GetCPR()) / (2.0f * PI));
        printf("%ld   %ld   %ld \n\r", i, index, lookup_table[index]);
        wait(.001);
    }
    rotor_sensor_->SetOffsetLUT(lookup_table); // Write Compensated Lookup Table

    //memcpy(controller->cogging, cogging_current, sizeof(controller->cogging));  //compensation doesn't actually work yet....
    printf("\n\rEncoder Electrical Offset (rad) %f\n\r", offset);

    delete[] error_forward; //gotta free up that ram
    delete[] error_backward;
    delete[] lookup_table;
    delete[] raw_forward;
    delete[] raw_backward;
    delete[] error;
    delete[] error_filtered;

    return true;
}

void Motor::SetPolePairs(uint32_t pole_pairs)
{
    config_.num_pole_pairs = pole_pairs;

    // Update Rotor
    rotor_sensor_->SetPolePairs(pole_pairs);
    dirty_ = true;
}

void Motor::SetKV(float K_v)
{
    config_.K_v = K_v;

    // Compute other parameters
    config_.flux_linkage = 60.0f / (SQRT3 * config_.K_v * PI * config_.num_pole_pairs * 2);
    config_.K_t = config_.flux_linkage * config_.num_pole_pairs * 1.5f; // rotor_flux_*Pole_Pairs*3/2

    dirty_ = true;
}
