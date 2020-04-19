/*
 * RMSCurrentLimiter.cpp
 *
 *  Created on: April 6, 2020
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
 * 
 */

// Primary Include
#include "RMSCurrentLimiter.h"

// C System Files

// C++ System Files
#include <arm_math.h>
//#include <math.h>

// Project Includes
#include "mbed.h"

RMSCurrentLimiter::RMSCurrentLimiter(float continuous_current, float period, float sample_time, uint32_t sub_sample_count) : I_continuous(continuous_current),
                                                                                                                             I_rms(0.0f),
                                                                                                                             I_max(0.0f),
                                                                                                                             T_(period),
                                                                                                                             d_t_(sample_time),
                                                                                                                             running_sum_(0.0f),
                                                                                                                             sub_sample_sum_(0.0f),
                                                                                                                             sub_sample_idx_(0),
                                                                                                                             num_sub_samples_(sub_sample_count)
{
    window_size_ = (uint32_t)T_ / d_t_;
    sampling_window_ = new RingBuffer(window_size_);

    // Precomputes
    I_CONT_SQUARED = I_continuous * I_continuous;
    ONE_OVER_DT = 1.0f / d_t_;
    ONE_OVER_T = 1.0f / T_;

    Reset();
}

void RMSCurrentLimiter::Reset()
{
   // I_rms = 0.0f;
   // I_max = I_continuous;

    sub_sample_idx_ = 0;
    sub_sample_sum_ = 0.0f;
    running_sum_ = 0.0f;
    sampling_window_->reset();

    ComputeRMSCurrent();
    ComputeMaxAllowableCurrent();
}

void RMSCurrentLimiter::AddCurrentSample(float sample)
{
    //std::cout << "Adding: " << sample << std::endl;


    // Update sub sample
    sub_sample_sum_ += sample;
    sub_sample_idx_++;
    
    if (sub_sample_idx_ >= num_sub_samples_) // Subsample average computed
    {
        // Compute sub sample current average
        float I_avg = sub_sample_sum_ / num_sub_samples_;
        if (sampling_window_->full()) // Full?  Pop off and subtract from running average
        {
            running_sum_ -= sampling_window_->peak();
        }

        float sample_dt = (I_avg * I_avg) * d_t_;
        running_sum_ += sample_dt; // Update Running sum
        sampling_window_->put(sample_dt); // Add value to RMS current window

        // Compute RMS Values at current time step
        ComputeRMSCurrent();
        ComputeMaxAllowableCurrent();

        // Reset sub sample datar
        sub_sample_sum_ = 0.0f;
        sub_sample_idx_ = 0;
    }
}

float RMSCurrentLimiter::GetRMSCurrent()
{
    return I_rms;
}

float RMSCurrentLimiter::GetMaxAllowableCurrent()
{
    return I_max;
}

void RMSCurrentLimiter::ComputeRMSCurrent()
{
    arm_sqrt_f32(running_sum_ * ONE_OVER_T, &I_rms);
}

void RMSCurrentLimiter::ComputeMaxAllowableCurrent()
{
    float mean_square_val = (ONE_OVER_DT) * ((I_CONT_SQUARED)*T_ - running_sum_);
    mean_square_val = mean_square_val < 0 ? 0 : mean_square_val; // Clamp negative values to zero
    // TODO: If this is negative we actually are over.  Possible error out here?
    arm_sqrt_f32(mean_square_val, &I_max);
}