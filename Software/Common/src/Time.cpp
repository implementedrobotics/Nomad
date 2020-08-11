/*
 * Time.cpp
 *
 *  Created on: August 9, 2019
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
 */

// Primary Include
#include <Common/Time.hpp>

// C System Includes

// C++ System Includes
#include <iostream>
#include <string>
#include <chrono>

// Third Party Includes

// Project Include Files

namespace Systems
{
    Time::Time()
    {
        start_ = std::chrono::high_resolution_clock::now();
    }
    Time::~Time()
    {
        end_ = std::chrono::high_resolution_clock::now();
        duration_ = end_ -  start_;

        std::cout << "Timer Took: " << duration_.count()*1e6 << "uS" << std::endl;
    }
    uint64_t Time::GetTimeStamp()
    {
        static auto start_time = std::chrono::high_resolution_clock::now(); 

        auto time_now = std::chrono::high_resolution_clock::now(); 
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(time_now - start_time); 
        return duration.count();
    }

}