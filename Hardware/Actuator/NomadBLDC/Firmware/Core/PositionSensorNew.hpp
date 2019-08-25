/*
 * PositionSensor.hpp
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

class PositionSensorAM5147 {

public:

    // Motor Parameters
    struct Config_t {
        float offset_elec;        // Electrical Position Offset (Radians)
        float offset_mech;        // Mechanical Position Offset (Radians)
        uint32_t cpr;             // Sensor Counts Per Revolution
        int32_t offset_lut[128];  // Offset Lookup Table
    };

    PositionSensorAM5147(uint32_t cpr = 16384, int pole_pairs); 
    
    void SetCPR(uint32_t cpr);               // Set Sensor Counts Per Revolution
    void SetElectricalOffset(float offset);  // Set Sensor Electrical Position Offset, Distance to A Axis (radians)
    void SetMechanicalOffset(float offset);  // Set Sensor Mechanical Position Offset, Output "zero" point (radians)
    void SetOffsetLUT(int32_t lookup_table[128]); // Non-Linearity Lookup Table (Helps with position sensor installation offset)
    void ZeroPosition();                     // Zero Mechanical Position Offset

    float GetElectricalPosition() const;     // Get Sensor Electrical Position w/ Offset, Distance to A Axis (radians)
    float GetMechanicalPosition() const;     // Get Sensor Mechanical Position w/ Offset, (radians)
    float GetMechanicalPositionTrue() const; // Get Sensor Real Mechanical Position without Offset, (radians)
    float GetRawPosition() const;            // Get Sensor Raw Position (counts)

    float GetElectricalVelocity() const;     // Get Sensor Electrical Velocity (radians/sec)
    float GetMechanicalVelocity() const;     // Get Sensor Mechanical Velocity (radians/sec)

    void Update(float Ts);              // Sample and Update Position Sensor State

    bool WriteConfig(); // Write Configuration to Flash Memory
    bool ReadConfig();  // Read Configuration from Flash Memory

private:

    float position_electrical_;     // Sensor Electrical Position (radians)
    float position_mechanical_;     // Sensor Mechanical Position (radians)
    float position_normalized_;     // Sensor Mechanical Position Modulo/Normalized (radians)
    float velocity_electrical_;          // Sensor Electrical Velocity (radians/sec)
    float velocity_electrical_filtered_; // Sensor Filtered Electrical Velocity (radians/sec)
    float velocity_mechanical_;          // Sensor Mechanical Velocity (radians/sec)

    int32_t position_raw_;      // Sensor Raw Position (counts)
    int32_t num_rotations_;     // Keep Track of Sensor Rotations
    uint32_t pole_pairs_;       // Pole Pairs in Motor to Compute Electrical Positions
    
    Config_t config_; // Position Sensor Configuration Parameters

    SPI *spi_handle_;
};

#endif // CORE_POSITION_SENSOR_H_