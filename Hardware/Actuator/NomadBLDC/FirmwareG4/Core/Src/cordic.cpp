/*
 * cordic.cpp
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

// Primary Include
#include <Peripherals/cordic.h>

// C++ Includes
#include <cmath>
#include <algorithm>

// C System Files

// Project Includes
#include <Utilities/math.h>
#include <main.h> // STM32 Driver Includes
#include <Logger.h>

using namespace Core::Math;
Cordic::Cordic() : precision_(LL_CORDIC_PRECISION_1CYCLE), scale_(LL_CORDIC_SCALE_0), mode_(COSINE)
{
}

// Singleton Insance
Cordic &Cordic::Instance()
{
    static Cordic instance;
    return instance;
}

void Cordic::Init()
{
    // Init Cordic Clocks
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CORDIC);
}

void Cordic::SetMode(CORDIC_MODE mode)
{
    // Requested Mode Already Set, Return
    if(mode_ == mode)
        return;

    uint32_t function = 0;
    uint32_t num_input = 1;
    uint32_t num_output = 2;

    switch (mode)
    {

    case CORDIC_MODE::COSINE:
        function = LL_CORDIC_FUNCTION_COSINE; // Cosine Function
        scale_ = LL_CORDIC_SCALE_0;           // No Scale
        num_input = LL_CORDIC_NBWRITE_1;      // Cosine Input Angle
        num_output = LL_CORDIC_NBREAD_1;      // Cosine Value
        break;

    case CORDIC_MODE::SINE:
        function = LL_CORDIC_FUNCTION_SINE; // Sine Function
        scale_ = LL_CORDIC_SCALE_0;         // No Scale
        num_input = LL_CORDIC_NBWRITE_1;    // Sine Input Angle
        num_output = LL_CORDIC_NBREAD_1;    // Sine Value
        break;

    case CORDIC_MODE::COSINE_SIN:
        function = LL_CORDIC_FUNCTION_COSINE; // Cosine Function
        scale_ = LL_CORDIC_SCALE_0;           // No Scale
        num_input = LL_CORDIC_NBWRITE_1;      // Cosine Input Angle
        num_output = LL_CORDIC_NBREAD_2;      // Cosine Value, Followed by Sine Value
        break;

    case CORDIC_MODE::MODULUS:
        function = LL_CORDIC_FUNCTION_MODULUS; // Cosine Function
        scale_ = LL_CORDIC_SCALE_0;            // No Scale
        num_input = LL_CORDIC_NBWRITE_2;       // X Coordinate, Y Coordinate
        num_output = LL_CORDIC_NBREAD_1;       // Modulus (Sqrt(x^2+y^2))
        break;

    default:
        break;
    }

    // Config Cordic
    LL_CORDIC_Config(CORDIC,
                     function,                  // Cordic function
                     precision_,                // Cordic precision
                     scale_,                    // Cordic Scale
                     num_input,                 // Cordic Input Arguments
                     num_output,                // Cordic Output Values
                     LL_CORDIC_INSIZE_32BITS,   // q1.31 format for input data
                     LL_CORDIC_OUTSIZE_32BITS); // q1.31 format for output data
    
    // Update Mode
    mode_ = mode;
}

int32_t Cordic::ConvertAngle(float rad)
{
    //float wrapped = rad - k2PI * static_cast<int>((rad + kPI) / k2PI);
    // Wrap and Scale for CORDIC -1 to 1 = -pi to pi
    float wrapped = (rad - k2PI * static_cast<int>((rad + kPI) / k2PI)) / kPI;
    
    // Return q1.31 formatted
    return static_cast<int32_t>(wrapped * kQ31);
}

float Cordic::ConvertToFloat(int32_t val)
{
    return static_cast<float>(val) * 1.0f / kQ31;
}

float Cordic::Cos(float theta)
{
    // Update Mode
    SetMode(CORDIC_MODE::COSINE);

    // Convert to q1.31
    LL_CORDIC_WriteData(CORDIC, ConvertAngle(theta));

    // Read Cosine
    int32_t c_q31 = static_cast<int32_t>(LL_CORDIC_ReadData(CORDIC));

    return ConvertToFloat(c_q31);
}

int32_t Cordic::Cos(int32_t theta_q31)
{
    // Update Mode
    SetMode(CORDIC_MODE::COSINE);

    // Write theta to CORDIC register
    LL_CORDIC_WriteData(CORDIC, theta_q31);

    // Read/Return Cosine
    return static_cast<int32_t>(LL_CORDIC_ReadData(CORDIC));
}

float Cordic::Sin(float theta)
{
    // Update Mode
    SetMode(CORDIC_MODE::SINE);

    // Convert to q1.31
    LL_CORDIC_WriteData(CORDIC, ConvertAngle(theta));

    // Read Sine
    int32_t s_q31 = static_cast<int32_t>(LL_CORDIC_ReadData(CORDIC));

    return ConvertToFloat(s_q31);
}

int32_t Cordic::Sin(int32_t theta_q31)
{
    // Update Mode
    SetMode(CORDIC_MODE::SINE);

    // Write theta to CORDIC register
    LL_CORDIC_WriteData(CORDIC, theta_q31);

    // Read/Return Sine
    return static_cast<int32_t>(LL_CORDIC_ReadData(CORDIC));
}

void Cordic::CosSin(float theta, float &cos, float &sin)
{

    //static uint32_t span;
    //DWT->CYCCNT = 0;
    // Update Mode
    SetMode(CORDIC_MODE::COSINE_SIN);

    // Convert to q1.31
    LL_CORDIC_WriteData(CORDIC, ConvertAngle(theta));

    // Read Cosine
    int32_t c_q31 = static_cast<int32_t>(LL_CORDIC_ReadData(CORDIC));

    // Read Sine
    int32_t s_q31 = static_cast<int32_t>(LL_CORDIC_ReadData(CORDIC));

    // Convert Out
    cos = ConvertToFloat(c_q31);

    sin = ConvertToFloat(s_q31);

  //  uint32_t span = DWT->CYCCNT;
  //  Logger::Instance().Print("Perf: %d\r\n", span);
}

void Cordic::CosSin(int32_t theta_q31, int32_t &c_q31, int32_t &s_q31)
{
    // Update Mode
    SetMode(CORDIC_MODE::COSINE_SIN);

    // Write theta to CORDIC register
    LL_CORDIC_WriteData(CORDIC, theta_q31);

    // Read Cosine
    c_q31 = static_cast<int32_t>(LL_CORDIC_ReadData(CORDIC));

    // Read Sine
    s_q31 = static_cast<int32_t>(LL_CORDIC_ReadData(CORDIC));

}