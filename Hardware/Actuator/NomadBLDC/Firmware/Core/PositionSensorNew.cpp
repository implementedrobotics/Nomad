/*
 * PositionSensor.cpp
 *
 *  Created on: August 24, 2019
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
#include "PositionSensor.hpp"

// C System Files

// C++ System Files

// Project Includes
#include "mbed.h"
#include "../../math_ops.h"

PositionSensorAM5147::PositionSensorAM5147(int CPR, float offset, int ppairs) :  {
    //_CPR = CPR;
    _CPR = CPR;
    _ppairs = ppairs;
    ElecOffset = offset;
    rotations = 0;
    spi = new SPI(PC_12, PC_11, PC_10);
    spi->format(16, 1);                                                          // mbed v>127 breaks 16-bit spi, so transaction is broken into 2 8-bit words
    spi->frequency(25000000);
    
    cs = new DigitalOut(PA_15);
    cs->write(1);
    MechOffset = offset;
    modPosition = 0;
    oldModPosition = 0;
    oldVel = 0;
    raw = 0;
    }
    