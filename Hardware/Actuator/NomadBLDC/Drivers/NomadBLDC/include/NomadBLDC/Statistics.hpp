  
/*
 * Time.hpp
 *
 *  Created on: August 4, 2020
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

#ifndef NOMAD_COMMON_STATISTICS_H
#define NOMAD_COMMON_STATISTICS_H

// C System Files
#include <math.h>

// C++ System Files
#include <iostream>
#include <string>
#include <chrono>

// Third Party Includes

// Project Include Files
namespace Statistics
{
    template <class T>
    class RollingStats
    {
    public:
        // Constructor
        RollingStats() : samples_(0),
                         max_sample_(-std::numeric_limits<T>::infinity()),
                         min_sample_(std::numeric_limits<T>::infinity())
        {
        }

        void Add(T sample)
        {
            samples_++;
            if (samples_ == 1)
            {
                M_prev_ = M_next_ = sample;
                S_prev_ = 0;
            }
            else
            {
                M_next_ = M_prev_ + (sample - M_prev_) / samples_;
                S_next_ = S_prev_ + (sample - M_prev_) * (sample - M_next_);

                M_prev_ = M_next_;
                S_prev_ = S_next_;
            }

            min_sample_ = sample < min_sample_ ? sample : min_sample_;
            max_sample_ = sample > max_sample_ ? sample : max_sample_;
        }

        unsigned long int SampleCount() const
        {
            return samples_;
        }

        T Mean() const
        {
            return (samples_ > 0) ? M_next_ : 0;
        }

        T Variance() const
        {
            return ((samples_ > 1) ? S_next_ / (samples_ - 1) : 0);
        }

        T StandardDeviation() const
        {
            return sqrt(Variance());
        }

        T Max() const
        {
            return max_sample_;
        }

        T Min() const
        {
            return min_sample_;
        }

        void Reset()
        {
            samples_ = 0;
            max_sample_ = -std::numeric_limits<double>::infinity();
            min_sample_ = std::numeric_limits<double>::infinity();
        }

    private:
        unsigned long int samples_;
        T M_prev_;
        T M_next_;
        T S_prev_;
        T S_next_;

        T max_sample_;
        T min_sample_;
    };
} // namespace Statistics
#endif // NOMAD_COMMON_STATISTICS_H