/*
 * math.h
 *
 *  Created on: October 17, 2020
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

#ifndef CORE_UTILITIES_MATH_H_
#define CORE_UTILITIES_MATH_H_
 
// C System Files
#include <cmath>
#include <cstdint>

// C++ System Files

// Project Includes

namespace Core::Math {

    // Some Const Expressions for Computations
    constexpr float kPI   = 3.14159265359f;
    constexpr float k2PI  = 6.28318530718f;
    constexpr float kPI_2 = 1.57079632679f;
    constexpr float kQ31  = 2147483648.0f;
    constexpr float kQ15  = 32768.0f;
    constexpr float kSqrt3 = 1.73205080757f;

    namespace FixedPoint
    {
        // Convert Floating to Fixed Point
        int32_t inline Convert(float value, uint8_t bits)
        {
            return static_cast<int32_t>(value * static_cast<float>((1 << bits) - 1));
        }

        // Convert Floating to Fixed Point of specified range from min to max
        int32_t inline Convert(float value, float min_value, float max_value, uint8_t bits)
        {
            float range = max_value - min_value;
            return static_cast<int32_t>(((value-min_value) * static_cast<float>((1 << bits) - 1))/range);
        }

        // Convert Fixed to Floating Point
        float inline Convert(int32_t value, uint8_t bits)
        {
            return static_cast<float>(value) / (static_cast<float>((1 << bits) - 1));
        }

        // Convert Fixed to Floating Point of specified range from min to max
        float inline Convert(int32_t value, float min_value, float max_value, uint8_t bits)
        {
            float range = max_value - min_value;
            return static_cast<float>(value)*range / (static_cast<float>((1 << bits) - 1))+min_value;
        }
    }
    namespace Vector2d
    {
        // Vector Magnitude
        float inline Magnitude(float &x, float &y)
        {
            return sqrt(x * x + y * y);
        }
        void inline Limit(float *x, float *y, float max)
        {
            float length_squared = *x * *x + *y * *y;
            if((length_squared > max * max) && length_squared > 0)
            {
                float ratio = max / sqrt(length_squared);
                *x *= ratio;
                *y *= ratio;
            }
        }
    } // namespace Vector2d

    namespace Vector3d
    {
        // Vector Magnitude
        float inline Magnitude(float &x, float &y, float &z)
        {
            return sqrt(x * x + y * y + z * z);
        }
        // Return Maximum component of vector
        float inline Max(float &x, float &y, float &z)
        {
            return (x > y ? (x > z ? x : z) : (y > z ? y : z));
        }
        // Return Minimum component of vector
        float inline Min(float &x, float &y, float &z)
        {
            return (x < y ? (x < z ? x : z) : (y < z ? y : z));
        }
    } // namespace Vector3d

}
#endif // CORE_UTILITIES_MATH_H_

