/*
 * cordic.h
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

#ifndef CORE_PERIPHERAL_CORDIC_H_
#define CORE_PERIPHERAL_CORDIC_H_

// C System Files
#include <stdlib.h>
#include <stdint.h>

// C++ System Files

// STM32 System Files
#include <main.h>

// Project Includes

// Currently only uses q1.31.  Would be nice to have support for q1.15
// TODO: q1.15 support
// TODO: Handle support in software for Non-Cordic STM Devices
class Cordic
{

public:
    
    // Cordic Mode
    typedef enum
    {
        COSINE = 0,
        SINE = 1,
        COSINE_SIN = 2,
        PHASE = 3,
        MODULUS = 4,
        NATRUAL_LOG = 5,
        SQRT = 6
    } CORDIC_MODE;

    Cordic();

    // Static Instance
    static Cordic &Instance();

    // Initialize Defaults
    void Init();

    // Update CORDIC Mode
    void SetMode(CORDIC_MODE mode);// __attribute__((section(".ccmram")));

    // Update Cordic Precision
    void SetPrecision(uint32_t precision) { precision_ = precision; };

    // Compute Cosine (float)
    float Cos(float theta);

    // Compute Cosine (q1.31)
    int32_t Cos(int32_t theta);

    // Compute Sine (float)
    float Sin(float theta);

    // Compute Sine (q1.31)
    int32_t Sin(int32_t theta);

    // Compute Cos/Sin (float)
    void CosSin(float theta, float &cos, float &sin);// __attribute__((section(".ccmram")));

    // Compute Cos/Sin (q1.31)
    void CosSin(int32_t theta, int32_t &cos, int32_t &sin);

    // Compute Phase (float)
    float Phase(float x, float y);

    // Compute Phase (q1.31)
    int32_t Phase(int32_t x, int32_t y);

    // Compute Modulus (float)
    float Modulus(float x, float y);

    // Compute Modulus (q1.31)
    int32_t Modulus(int32_t x, int32_t y);

    // Compute Natural Log (float)
    float NaturalLog(float x);

    // Compute Natural Log (q1.31)
    int32_t NaturalLog(int32_t x);

    // Compute SQRT (float)
    float Sqrt(float x);

    // Compute SQRT (q1.31)
    int32_t Sqrt(int32_t x);

    // TODO: Move this into its own file?
    // Helpers
    // From float radians to q1.31 (Handles scaling/mapping to [-1 to 1])
    static int32_t ConvertAngle(float rad);// __attribute__((section(".ccmram")));;

    // From q1.31 to float
    static float ConvertToFloat(int32_t val);// __attribute__((section(".ccmram")));

private:
    // CORDIC Precision
    uint32_t precision_;

    // CORDIC Scale Factor
    uint32_t scale_;

    // CORDIC Mode
    uint32_t mode_;
};

#endif // CORE_PERIPHERAL_CORDIC_H_
