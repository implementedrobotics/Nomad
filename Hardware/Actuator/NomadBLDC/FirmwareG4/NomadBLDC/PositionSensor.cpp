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
#include "PositionSensor.h"

// C System Files

// C++ System Files
#include <cstring>

// Project Includes
#include "main.h"
#include <Peripherals/gpio.h>
#include <Utilities/math.h>
#include <Logger.h>

PositionSensorAS5x47::PositionSensorAS5x47(float sample_time, uint32_t pole_pairs, uint32_t cpr) : position_electrical_(0),
                                                                                                   position_mechanical_(0),
                                                                                                   position_normalized_(0),
                                                                                                   velocity_electrical_(0),
                                                                                                   velocity_electrical_filtered_(0),
                                                                                                   velocity_mechanical_(0),
                                                                                                   position_raw_(0),
                                                                                                   num_rotations_(0),
                                                                                                   pole_pairs_(pole_pairs),
                                                                                                   filter_size_(40),
                                                                                                   sample_time_(sample_time),
                                                                                                   dirty_(false)
{

    config_.offset_elec = 0;
    config_.offset_mech = 0;
    config_.cpr = cpr;
    // config_.direction = 1;
    memset(config_.offset_lut, 0, sizeof(config_.offset_lut));

    // Init SPI Driver Handle for Encoder Sensor
    GPIO_t mosi = { ENC_MOSI_GPIO_Port, ENC_MOSI_Pin };
    GPIO_t miso = { ENC_MISO_GPIO_Port, ENC_MISO_Pin };
    GPIO_t nss = { ENC_CS_GPIO_Port, ENC_CS_Pin };
    spi_dev_ = new SPIDevice(SPI1, mosi, miso, nss);
    spi_dev_->Enable();

    velocity_samples_ = new float[filter_size_];
}
void PositionSensorAS5x47::Reset()
{
    // Reset Values
    memset(config_.offset_lut, 0, sizeof(config_.offset_lut));

    config_.offset_elec = 0;
    config_.offset_mech = 0;
    dirty_ = true;
}
/* Setters */
void PositionSensorAS5x47::SetSampleTime(float sample_time)
{
    sample_time_ = sample_time;
}
void PositionSensorAS5x47::SetCPR(int32_t cpr)
{
    config_.cpr = cpr;
    dirty_ = true;
}

void PositionSensorAS5x47::SetElectricalOffset(float offset)
{
    config_.offset_elec = offset;
    dirty_ = true;
}

void PositionSensorAS5x47::SetMechanicalOffset(float offset)
{
    config_.offset_mech = offset;
    dirty_ = true;
}
// void PositionSensorAS5x47::SetDirection(int32_t direction)
// {
//     //config_.direction = direction;
//     dirty_ = true;
// }

// TODO: Support custom LUT Sizes
void PositionSensorAS5x47::SetOffsetLUT(int8_t lookup_table[128])
{
    memcpy(config_.offset_lut, lookup_table, sizeof(config_.offset_lut));
    dirty_ = true;
}

void PositionSensorAS5x47::SetPolePairs(uint32_t pole_pairs)
{
    pole_pairs_ = pole_pairs;
}

void PositionSensorAS5x47::ZeroPosition()
{
    num_rotations_ = 0;
    config_.offset_mech = 0.0f;
    Update(sample_time_);
    config_.offset_mech = GetMechanicalPosition();
    dirty_ = true;
}

void PositionSensorAS5x47::StartUpdate()
{
    spi_dev_->Select();
    spi_dev_->Transmit16(0xFFFF);
}
void PositionSensorAS5x47::EndUpdate()
{
    position_raw_ = spi_dev_->Receive16();
    spi_dev_->Deselect();
    Update(sample_time_);
}

void PositionSensorAS5x47::Update()
{
    spi_dev_->Select();
    position_raw_ = spi_dev_->TransmitReceive16(0xFFFF);
    position_raw_ &= 0x3FFF; // Data in last 14 bits.
    spi_dev_->Deselect();
    Update(sample_time_);
    
}
void PositionSensorAS5x47::Update(float Ts)
{
    using namespace Core::Math;

    static float prev_position_norm = 0;
    static int32_t prev_counts = 0;

    spi_dev_->Select();
    position_raw_ = spi_dev_->TransmitReceive16(0xFFFF);
    position_raw_ &= 0x3FFF; // Data in last 14 bits.
    spi_dev_->Deselect();
    
    // Double Check High/Low Values
    if (position_raw_ == 0 || position_raw_ == 0x3FFF)
    {
        spi_dev_->Select();
        position_raw_ = spi_dev_->TransmitReceive16(0xFFFF);
        position_raw_ &= 0x3FFF; // Data in last 14 bits.
        spi_dev_->Deselect();
    }
    
    // Interpolate position offset from Lookup Table
    int32_t offset_1 = config_.offset_lut[position_raw_ >> 7];
    int32_t offset_2 = config_.offset_lut[((position_raw_ >> 7) + 1) % 128];
    int32_t offset_interp = offset_1 + ((offset_2 - offset_1) * (position_raw_ - ((position_raw_ >> 7) << 7)) >> 7);

    int32_t current_counts = position_raw_ + offset_interp; // Compensate for non linearities from the sensor calibration

    // Wrap angle rotations and count them
    if (current_counts - prev_counts > config_.cpr / 2)
    {
        num_rotations_ -= 1;
    }
    else if (current_counts - prev_counts < -config_.cpr / 2)
    {
        num_rotations_ += 1;
    }

    // Update previous values for next sample period
    prev_counts = current_counts;
    prev_position_norm = position_normalized_;

    // Compute normalized sensor position 0 to 2PI
    position_normalized_ = ((k2PI * ((float)current_counts)) / (float)config_.cpr);

    // Compute cumulative sensor position
    float current_position = (k2PI * ((float)current_counts + (config_.cpr * num_rotations_))) / (float)config_.cpr;

    // Compute mechanical position
    position_mechanical_ = current_position - config_.offset_mech;

    // Compute electrical position
    position_electrical_ = ((k2PI / (float)config_.cpr) * (float)((pole_pairs_ * current_counts) % config_.cpr)) + config_.offset_elec;

    //  Wrap 0 to 2PI
    if (position_electrical_ < 0)
    {
        position_electrical_ += k2PI;
    }
    else if (position_electrical_ > k2PI)
    {
        position_electrical_ -= k2PI;
    }

    // Compute Velocity
    float velocity = 0.0f;
    if ((position_normalized_ - prev_position_norm) < -3.0f)
    {
        velocity = (position_normalized_ - prev_position_norm + k2PI) / Ts;
    }
    else if ((position_normalized_ - prev_position_norm) > 3.0f)
    {
        velocity = (position_normalized_ - prev_position_norm - k2PI) / Ts;
    }
    else
    {
        velocity = (position_normalized_ - prev_position_norm) / Ts;
    }

    // Estimate/Filter Velocity
    // TODO: Change this to a PLL?
    float vel_sum = velocity;
    for (int32_t i = 1; i < (filter_size_); i++)
    {
        velocity_samples_[filter_size_ - i] = velocity_samples_[filter_size_ - i - 1];
        vel_sum += velocity_samples_[filter_size_ - i];
    }
    velocity_samples_[0] = velocity;

    // Update Velocities
    velocity_mechanical_ = vel_sum / ((float)filter_size_);
    velocity_electrical_ = velocity_mechanical_ * pole_pairs_;
    // TODO: Add in out LPF Filter
    velocity_electrical_filtered_ = 0.99f * velocity_electrical_filtered_ + 0.01f * velocity_electrical_;
    
}
