/*
 * lpf.h
 *
 *  Created on: October 16, 2020
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

#ifndef CORE_UTILITIES_LPF_H_
#define CORE_UTILITIES_LPF_H_

// C System Files
#include <stdlib.h>
#include <stdint.h>

// C++ System Files

// STM32 System Files
#include <main.h>

// Project Includes
#include <Utilities/math.h>

class LowPassFilter
{

public:
    
    // Setup with precomputed alpha
    LowPassFilter(const float alpha = 0.0f) : alpha_(alpha), output_(0.0f) {}

    // Setup with Sample Time/Desired cutoff frequency radians
    LowPassFilter(float d_t, float f_c) : output_(0.0f)
    {
        // Compute RC
        float RC = 1.0f/(Core::Math::k2PI *f_c);

        // Compute Alpha
        alpha_ = d_t / (d_t + RC);
    }

    // Init Filter to Desired Output
    void Init(const float output)
    {
        output_ = output;
    }

    // Filter Sample
    inline const float Filter(const float sample)  __attribute__((always_inline))
    { 
        // Update Filter Output
        output_ = output_ + (sample - output_) * alpha_;

        // Return Output
        return output_;
    }

    // Get Filter Output
    const float Output() const
    {
        return output_;
    }

    // Get Filter Alpha
    const float Alpha() const
    {
        return alpha_;
    }

private:

    // Filter Alpha
    float alpha_;

    // Filter Output
    float output_;
};

#endif // CORE_UTILITIES_LPF_H_
