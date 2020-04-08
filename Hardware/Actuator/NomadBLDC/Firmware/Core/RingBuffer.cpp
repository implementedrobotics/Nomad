/*
 * RingBuffer.cpp
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
#include "RingBuffer.h"

// C System Files
//#include <math.h>
//#include <stdio.h>
//#include <string.h>

// C++ System Files
//#include <iostream>

// Project Includes
#include "mbed.h"

RingBuffer::RingBuffer(size_t buffer_size) : head_(0),
                                             tail_(0),
                                             buffer_size_(buffer_size),
                                             full_(false)
{
    buffer_ = new float[buffer_size](); // Init with zeros
}

bool RingBuffer::empty() const
{
    return (!full_ && (head_ == tail_));
}
bool RingBuffer::full() const
{
    return full_;
}

void RingBuffer::reset()
{
    head_ = tail_;
    full_ = false;
    memset(buffer_, 0, sizeof(float) * buffer_size_);
}

size_t RingBuffer::capacity() const
{
    return buffer_size_;
}

size_t RingBuffer::size() const
{
    size_t size = buffer_size_;

    if (!full_)
    {
        if (head_ >= tail_)
        {
            size = head_ - tail_;
        }
        else
        {
            size = buffer_size_ + head_ - tail_;
        }
    }
    return size;
}

void RingBuffer::put(float val)
{
    buffer_[head_] = val;
    if (full_)
    {
        //tail_ = (tail_ + 1) % buffer_size_;
        if (++tail_ >= buffer_size_)
            tail_ = 0;
    }
    //head_ = (head_ + 1) % buffer_size_;
    //if (++head_ >= buffer_size_)
    //    head_ -= buffer_size_;
    if (++head_ >= buffer_size_)
        head_ = 0;
    
    full_ = head_ == tail_;
}
float RingBuffer::peak()
{
    return buffer_[tail_];
}

float RingBuffer::get()
{
    if (empty())
    {
        return 0.0f;
    }

    //Read data and advance the tail (we now have a free space)
    auto val = buffer_[tail_];
    full_ = false;
    //tail_ = (tail_ + 1) % buffer_size_;
    if (++tail_ >= buffer_size_)
        tail_ = 0;

    return val;
}
