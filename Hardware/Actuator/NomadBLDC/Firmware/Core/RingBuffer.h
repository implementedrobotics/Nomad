/*
 * RingBuffer.h
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

#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

// C System Files
#include <cstdio>

// C++ System Files

// Project Includes

// TODO: Template this.  For now floats
class RingBuffer
{
public:
    RingBuffer(size_t buffer_size);

    void put(float val);
    float get();
    float peak();
    bool full() const;
    bool empty() const;
    void reset();
    size_t capacity() const;
    size_t size() const;
    float& operator[] ( size_t pos )
    {
        auto p = ( head_ + pos ) % buffer_size_ ;
        return buffer_[p] ;
    }

private:
    float *buffer_;
    size_t head_;
    size_t tail_; 
    const size_t buffer_size_;
    bool full_;
};

#endif // RING_BUFFER_H_
