/*
 * PositionSensor.h
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

#ifndef CORE_POSITION_SENSOR_H_
#define CORE_POSITION_SENSOR_H_

// C System Files

// C++ System Files

// Project Includes
#include "mbed.h"

class PositionSensorAS5x47
{

public:
    // Motor Parameters
    struct __attribute__((__packed__)) Config_t
    {
        float offset_elec;       // Electrical Position Offset (Radians)
        float offset_mech;       // Mechanical Position Offset (Radians)
        int32_t cpr;             // Sensor Counts Per Revolution
        int32_t direction;       // Sensor Direction for Positive Rotation
        int32_t offset_lut[128]; // Offset Lookup Table
    };

    PositionSensorAS5x47(float sample_time, uint32_t pole_pairs = 21, uint32_t cpr = 16384); // Constructor with some initial defaults.

    void SetCPR(int32_t cpr);                     // Set Sensor Counts Per Revolution
    void SetElectricalOffset(float offset);       // Set Sensor Electrical Position Offset, Distance to A Axis (radians)
    void SetMechanicalOffset(float offset);       // Set Sensor Mechanical Position Offset, Output "zero" point (radians)
    void SetOffsetLUT(int32_t lookup_table[128]); // Non-Linearity Lookup Table (Helps with position sensor installation offset)
    void SetPolePairs(uint32_t pole_pairs);       // Set Pole Pair Count for Electrical Position Calculations
    void SetDirection(int32_t direction);         // Sensor Direction
    
    void ZeroPosition(); // Zero Mechanical Position Offset

    inline int32_t GetRawPosition() { return position_raw_; }                                 // Get Sensor Raw Position (counts)
    inline float GetElectricalPosition() { return position_electrical_ * config_.direction; }                    // Get Sensor Electrical Position w/ Offset, Distance to A Axis (radians)
    inline float GetMechanicalPosition() { return position_mechanical_ * config_.direction; }                    // Get Sensor Mechanical Position w/ Offset, (radians)
    inline float GetMechanicalPositionTrue() { return (position_mechanical_ + config_.offset_mech) * config_.direction; } // Get Sensor Real Mechanical Position without Offset, (radians)
    inline float GetElectricalVelocity() { return velocity_electrical_ * config_.direction; }                    // Get Sensor Electrical Velocity (radians/sec)
    inline float GetMechanicalVelocity() { return velocity_mechanical_ * config_.direction; }                    // Get Sensor Mechanical Velocity (radians/sec)

    void Update();         // Update Position Sensor State w/ Implciit Sample Time (for velocity estimation)
    void Update(float Ts); // Update Position Sensor State w/ Sample Time (for velocity estimation)

    bool WriteConfig(Config_t config); // Write Configuration to Flash Memory
    bool ReadConfig(Config_t config);  // Read Configuration from Flash Memory

    Config_t config_;     // Position Sensor Configuration Parameters
    
private:
    float position_electrical_;          // Sensor Electrical Position (radians)
    float position_mechanical_;          // Sensor Mechanical Position (radians)
    float position_normalized_;          // Sensor Mechanical Position Modulo/Normalized [0-2*PI] (radians)
    float velocity_electrical_;          // Sensor Electrical Velocity (radians/sec)
    float velocity_electrical_filtered_; // Sensor Filtered Electrical Velocity (radians/sec)
    float velocity_mechanical_;          // Sensor Mechanical Velocity (radians/sec)
    float *velocity_samples_;            // Array to hold velocity samples for filtering/estimation
    int32_t position_raw_;               // Sensor Raw Position (counts)
    int32_t num_rotations_;              // Keep Track of Sensor Rotations
    uint32_t pole_pairs_;                // Pole Pairs in Motor to Compute Electrical Positions

    int32_t filter_size_; // Velocity Filter Window Size
    
    SPI *spi_handle_; // SPI Handle for Communication to Sensor

    float sample_time_; // Update Sample Time (=Current Control Update Rate)
    bool dirty_; // Have unsaved changes to config
};

#endif // CORE_POSITION_SENSOR_H_