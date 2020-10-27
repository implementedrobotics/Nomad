/*
 * RMSCurrentLimiter.h
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

#ifndef RMS_CURRENT_LIMITER_H
#define RMS_CURRENT_LIMITER_H

// C System Files

// C++ System Files

// Project Includes
#include <nomad_hw.h>
#include <RingBuffer.h>

class RMSCurrentLimiter
{
public:

    RMSCurrentLimiter(float continuous_current=20.0f, float period=60.0f, float sample_time=1.0f/10.0f, uint32_t sub_sample_count=1);

    void AddCurrentSample(float sample) CCM_ATTRIBUTE; // Add current sample to our window
    void Reset() CCM_ATTRIBUTE; // Reset Running RMS Calculation

    inline float GetRMSCurrent() { return I_rms; };          // Get the current RMS Current Value
    inline float GetMaxAllowableCurrent() { return I_max; }; // Get the maximum allowable current for the next time step that keeps value in allowable thermal envelope

private:

    float I_continuous; // RMS Continuous Current Value
    float I_rms;        // Current RMS Value
    float I_max;        // Max allowable current command
    float T_;  // RMS Period Window
    float d_t_; // RMS Sampling Time
    float running_sum_;     // Optimized running sum for RMS calculations.  Prevents summing full window each time step
    float sub_sample_sum_;   // Hold sub sample sum for averaging in between RMS period
    uint32_t sub_sample_idx_; // Current sub sample index
    uint32_t num_sub_samples_; // How many subsampes to use
    uint32_t window_size_;   // Size of samples in buffer for window
    RingBuffer *sampling_window_; // Buffer to hold current samples for our RMS window


    // Optimizations variable
    float I_CONT_SQUARED;
    float ONE_OVER_DT;
    float ONE_OVER_T;

    void ComputeRMSCurrent() CCM_ATTRIBUTE;
    void ComputeMaxAllowableCurrent() CCM_ATTRIBUTE;
};

#endif // RMS_CURRENT_LIMITER_H