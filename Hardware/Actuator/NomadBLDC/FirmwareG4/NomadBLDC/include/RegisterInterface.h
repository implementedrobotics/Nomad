/*
 * RegisterInterface.h
 *
 *  Created on: November 7, 2020
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

#ifndef REGISTER_INTERFACE_H_
#define REGISTER_INTERFACE_H_

// C System Files

// C++ System Files
#include <variant>
#include <vector>
#include <functional>
#include <cstring>

// Project Includes
#include <Peripherals/uart.h>
#include <Peripherals/fdcan.h>
#include <Logger.h>

/* Device Registers */
typedef enum
{
    DeviceStatusRegister1 = 0x00,
    // Device Status Register 1 Offsets
    DeviceFault = 0x01,
    DeviceControlMode = 0x02,
    DeviceVoltageBus = 0x03,
    DeviceCurrentBus = 0x04,
    DeviceFETTemp = 0x05,
    DeviceWindingTemp = 0x06,
    // Reserved = 0x07,
    // Reserved = 0x08,
    // Reserved = 0x09,
    // Reserved = 0x0A,
    // Reserved = 0x0B,
    // End Device Status Register 1

    DeviceStatusRegister2 = 0x0C,
    // Device Status Register2 Offsets
    DeviceFirmwareMajor = 0x0D,
    DeviceFirmwareMinor = 0x0E,
    DeviceUID1 = 0x0F,
    DeviceUID2 = 0x10,
    DeviceUID3 = 0x11,
    DeviceUptime = 0x12,
    // Reserved = 0x13,
    // Reserved = 0x14,
    // Reserved = 0x15,
    // End Device Status Register 2
} DeviceRegisters_e;


/* Motor Controller Registers */
typedef enum
{
    ControllerConfigRegister1 = 0x16,
    // Controller Config Register 1 Offsets
    K_LOOP_D = 0x17,              // Current Controller Loop Gain (D Axis)
    K_LOOP_Q = 0x18,              // Current Controller Loop Gain (Q Axis)
    K_I_D = 0x19,            // Current Controller Integrator Gain (D Axis)
    K_I_Q = 0x1A,            // Current Controller Integrator Gain (Q Axis)
    CurrentBandwidth = 0x1B, // Current Loop Bandwidth (200 to 2000 hz)
    Overmodulation = 0x1C,   // Overmodulation Amount
    PWM_Frequency = 0x1D,     // PWM Switching Frequency
    FOC_Divider = 0x1E,      // Divider to use for FOC Current control loop frequency
    // Reserved = 0x1F,
    // Reserved = 0x20,
    // Reserved = 0x21,
    // End Controller Config Register 1

    ControllerConfigRegister2 = 0x22,
    // Controller Config Register 2 Offsets
    K_P_Min = 0x23,
    K_P_Max = 0x24,
    K_D_Min = 0x25,
    K_D_Max = 0x26,
    VelocityLimit = 0x27, // Limit on maximum velocity
    PositionLimit = 0x28, // Limit on maximum position
    TorqueLimit = 0x29,   // Limit on maximum torque
    CurrentLimit = 0x2A,  // Limit on maximum current
    // Reserved = 0x2B,
    // Reserved = 0x2C,
    // Reserved = 0x2D,
    // Reserved = 0x2E,
    // Reserved = 0x2F,
    // End Device Status Register 2
} ControllerConfigRegisters_e;

typedef enum
{
    ControllerStateRegister1 = 0x30,
    // Controller State Register 1 Offsets
    I_D = 0x31,               // Transformed Current (D Axis)
    I_Q = 0x32,               // Transformed Current (Q Axis)
    V_D = 0x33,               // Voltage (D Axis)
    V_Q = 0x34,               // Voltage (Q Axis)
    IntegratorError_D = 0x35, // Current Integral Error (D Axis)
    IntegratorError_Q = 0x36, // Current Integral Error (Q Axis)
    DutyCycleA = 0x37,        // Duty Cycle for A phase
    DutyCycleB = 0x38,        // Duty Cycle for B phase
    DutyCycleC = 0x39,        // Duty Cycle for C phase
    CurrentRMS = 0x3A, // Motor RMS Current Value
    MaxCurrent = 0x3B, // Maximum Allowable Commanded Current in next Time Step
    Timeout = 0x3C, // Missed Input Control Deadline Count
    // Reserved = 0x3D,
    // Reserved = 0x3E,
    // Reserved = 0x3F,
    // End Controller State Register 1

    ControllerStateRegister2 = 0x40,
    // Controller State Register 2 Offsets
        VoltageControlModeRegister = 0x41,
            // Voltage Control Register Offsets
            V_Setpoint_D = 0x42, // Voltage Reference (D Axis)
            V_Setpoint_Q = 0x43, // Voltage Reference (Q Axis)
        // Current Control Register Offsets
        CurrenteControlModeRegister = 0x44,
            I_Setpoint_D = 0x45, // Current Reference (D Axis)
            I_Setpoint_Q = 0x46, // Current Reference (Q Axis)
        // Torque Control Register Offsets
        TorqueControlModeRegister = 0x47,
            PositionSetpoint = 0x48, // Position Setpoint Reference
            VelocitySetpoint = 0x49, // Velocity Setpoint Reference
            K_P = 0x4A, // Position Gain N*m/rad
            K_D = 0x4B, // Velocity Gain N*m/rad/s
            Torque_FF = 0x4C, // Feed Forward Torque Value N*m
    // Reserved = 0x4D,
    // Reserved = 0x4E,
    // Reserved = 0x4F,
    // End Controller State Register 2
} ControllerStateRegisters_e;

/* Motor Registers */
typedef enum
{
    MotorConfigRegister1 = 0x50,
    // Motor Config Register 1 Offsets
    PolePairs = 0x51,        // Pole Pairs of Motor (PAIRS)
    K_v = 0x52,              // Motor KV Rating (RPM/V)
    K_t = 0x53,              // Torque Constant (N*m/A)
    K_t_out = 0x54,          // Torque Constant @ Output (N*m/A)
    FluxLinkage = 0x55,      // Rotor Flux Linkage (Webers)
    PhaseResistance = 0x56,  // Phase Resistance (Ohms)
    PhaseInductanceD = 0x57, // D Axis Phase Inductance (Henries)
    PhaseInductanceQ = 0x58, // Q Axis Phase Inductance (Henries)
    GearRatio = 0x59,
    PhaseOrder = 0x5A,
    // End Motor Config Register 1

    MotorThermalConfigRegister = 0x5B, // Thermal Config Register
    // Motor Thermal Config Register Offsets
    ContinuousCurrentLimit = 0x5C, // Thermally calibrated Allowable Continuous Current (A)
    ContinuousCurrentTau = 0x5D,   // Time Constant for Continuous Current (Seconds)
    //Reserved = 0x5E,  // Observer Temp
    //Reserved = 0x5F,  // Observer Resistance
    //Reserved = 0x60,  // Thermal Capacitance
    //Reserved = 0x61,  // R Theta
    //Reserved = 0x62,
    //Reserved = 0x63,
    // End Motor Thermal Config Register

    MotorCalibrationConfigRegister = 0x64, // Motor Calibration Config Register
    // Motor Calibration Config Register Offsets
    CalibrationCurrent = 0x65,
    CalibrationVoltage = 0x66,
    Calibrated = 0x67,
    //Reserved = 0x68,
    //Reserved = 0x69,
    //Reserved = 0x6A,
    //Reserved = 0x6B,
    //Reserved = 0x6C,
    // End Motor Calibration Config Register
} MotorConfigRegisters_e;

typedef enum
{
    MotorStateRegister1 = 0x6D,
    // Motor State Register 1 Offsets
    I_A = 0x6E, // Phase A Currents
    I_B = 0x6F, // Phase B Currents
    I_C = 0x70, // Phase C Currents
    //V_A = 0x71,               // Phase A Voltages
    //V_B = 0x72,               // Phase B Voltages
    //V_C = 0x73,               // Phase C Voltages
    WindingsTemp = 0x74,        // Motor Windings Temperature (Degrees Celcius)
    OutputPosition = 0x75,     // Mechanical Position @ Output (Radians)
    OutputPositionTrue = 0x76, // Mechanical Position @ Output w/ Offset (Radians)
    OutputVelocity = 0x77,     // Mechanical Velocity @ Output (Radians/Sec)
    ElectricalPosition = 0x78, // Electrical Position @ Rotor (Radians)
    ElectricalVelocity = 0x79, // Electrical Velocity @ Rotor (Radians/Sec)
                               // Reserved = 0x7A,
                               // Reserved = 0x7B,
                               // Reserved = 0x7C,
    // End Motor State Register 1
} MotorStateRegisters_e;

/* Encoder Registers */
typedef enum
{
    EncoderConfigRegister1 = 0x80,
    // Encoder Config Register Offsets
    ElectricalOffset = 0x81, // Electrical Position Offset (Radians)
    MechanicalOffset = 0x82, // Mechanical Position Offset (Radians)
    CPR = 0x83,              // Counts Per Revolution
    // Reserved = 0x84,
    // Reserved = 0x85,
    // Reserved = 0x86,
    // Reserved = 0x87,
    // End Encoder Config Register 1
    EncoderConfigOffsetLUT1 = 0x88, // 1st 32-bytes
    EncoderConfigOffsetLUT2 = 0x89, // 2nd 32-bytes
    EncoderConfigOffsetLUT3 = 0x8A, // 3rd 32-bytes
    EncoderConfigOffsetLUT4 = 0x8B, // Last 32-bytes (128 Bytes Total)

} EncoderConfigRegisters_e;


/* Error State Registers */

/* Gate Drive Registers */



// TODO: Where to put this?
struct DeviceStatusRegister1_t
{
    uint8_t fw_major;          // Firmware Version Major
    uint8_t fw_minor;          // Firmware Version Minor
    uint32_t uid1;             // Device Unique ID 1
    uint32_t uid2;             // Device Unique ID 2
    uint32_t uid3;             // Device Unique ID 3
    uint32_t uptime;           // Device Uptime
};


// TODO: A way to make register field read only?
class RegisterData
{
public:
    template <typename T>
    RegisterData(T value)
    {
        data_ = value;
        data_size_ = data_sizes_[data_.index()];
       // Logger::Instance().Print("Data Now: %d\r\n", *value);
    }

    template <typename T>
    RegisterData(T value, size_t size) // For struct/class types
    {
        data_ = value;
        data_size_ = size;
       // Logger::Instance().Print("Data Now: %d\r\n", *value);
    }

    void SetFromBytes(uint8_t *value)
    {
        if (auto data = std::get_if<uint8_t *>(&data_))
        {
            Logger::Instance().Print("Byte Array Copy: %d\r\n", data_size_);
            memcpy(data, value, data_size_);
        }
    }
    void Set(uint8_t *value)
    {
        if (auto data = std::get_if<uint8_t *>(&data_))
        {
            //**data = *((uint8_t *)value);
            memcpy(*data, value, Size());
            Logger::Instance().Print("Variant Value8: %d\r\n", **data);
        }
        else if (auto data = std::get_if<uint16_t *>(&data_))
        {
            //**data = *((uint16_t *)value);
            memcpy(*data, value, Size());
            Logger::Instance().Print("Variant Value16: %d\r\n", **data);
        }
        else if (auto data = std::get_if<uint32_t *>(&data_))
        {
            //**data = *((uint32_t *)value);
            memcpy(*data, value, Size());
            Logger::Instance().Print("Variant Value32: %d\r\n", **data);
        }
        else if (auto data = std::get_if<int8_t *>(&data_))
        {
            //**data = *((uint8_t *)value);
            memcpy(*data, value, Size());
            Logger::Instance().Print("Variant Value8: %d\r\n", **data);
        }
        else if (auto data = std::get_if<int16_t *>(&data_))
        {
            //**data = *((int16_t *)value);
            memcpy(*data, value, Size());
            Logger::Instance().Print("Variant Value16: %d\r\n", **data);
        }
        else if (auto data = std::get_if<int32_t *>(&data_))
        {
            //**data = *((int32_t *)value);
            memcpy(*data, value, Size());
            Logger::Instance().Print("Variant Value32: %d\r\n", **data);
        }
        else if (auto data = std::get_if<float *>(&data_))
        {
            //**data = *((float *)value);
            memcpy(*data, value, Size());
            Logger::Instance().Print("Float Value32: %d\r\n", **data);
        }
    }
    template <typename T>
    void Set(T value)
    {
        // TODO: Type Ambiguity(int types) Here to resolve from stored type?
        if (auto data = std::get_if<T *>(&data_))
        {
            **data = value;
            Logger::Instance().Print("Setting Value: %d\r\n", **data);
        }
        else
        {
            Logger::Instance().Print("UNABLE TO SET TYPE\r\n");
        }
    }

    template <typename T>
    T Get()
    {
        // TODO: Type Ambiguity Here to resolve from stored type?
        if (auto data = std::get_if<T *>(&data_))
        {
            return **data;
        }

        Logger::Instance().Print("UNABLE TO GET TYPE\r\n");
        return 0;
    }

    uint16_t Get(uint8_t *bytes)
    {
        if (auto data = std::get_if<uint8_t *>(&data_))
        {
            std::memcpy(bytes, *data, Size());
            return Size();
        }
        else if (auto data = std::get_if<uint16_t *>(&data_))
        {
            std::memcpy(bytes, *data, Size());
            return Size();
        }
        else if (auto data = std::get_if<uint32_t *>(&data_))
        {
            std::memcpy(bytes, *data, Size());
            return Size();
        }
        else if (auto data = std::get_if<int8_t *>(&data_))
        {
            std::memcpy(bytes, *data, Size());
            return Size();
        }
        else if (auto data = std::get_if<int16_t *>(&data_))
        {
            std::memcpy(bytes, *data, Size());
            return Size();
        }
        else if (auto data = std::get_if<int32_t *>(&data_))
        {
            std::memcpy(bytes, *data, Size());
            return Size();
        }
        else if (auto data = std::get_if<float *>(&data_))
        {
            std::memcpy(bytes, *data, Size());
            return Size();
        }

        Logger::Instance().Print("UNABLE TO GET BYTESD\r\n");
        return 0;
    }

    size_t inline Size() const
    {
        //return data_sizes_[data_.index()];
        return data_size_;
    }

private:
    // Pointer to register memory location
    std::variant<uint8_t *, uint16_t *, uint32_t *, int8_t *, int16_t *, int32_t *, float *, std::function<void((void*))>> data_;

    // Mirror Type Sizes for Lookups
    static constexpr size_t data_sizes_[8] = {sizeof(uint8_t),
                                             sizeof(uint16_t),
                                             sizeof(uint32_t),
                                             sizeof(int8_t),
                                             sizeof(int16_t),
                                             sizeof(int32_t),
                                             sizeof(float),
                                             sizeof(void *)};
    
    // Data Size For Register
    size_t data_size_;

    // TODO: Setup max/min ranges for register data
    // Make for now always a float and convert types for checks
    // Or we could use a variant here
    float max_value_;
    float min_value_;
};

class Register
{

public:

    // Single Valued Register
    template <typename T>
    Register(T value)
    {
        AddDataField(value);
    }

    // Struct Valued Register
    template <typename T>
    Register(T value, bool is_struct)
    {
        if(is_struct)
            AddStructField(value);
    }

    template <typename T>
    void AddDataField(T value)
    {
        fields_.push_back(RegisterData(value));
    }

    template <typename T>
    void AddStructField(T value)
    {
       fields_.push_back(RegisterData((uint8_t *)value, sizeof(T)));
    }

    template <typename T>
    void Set(T value, uint16_t offset = 0)
    {
        fields_[offset].Set(value);
    }

    // Set from Byte Array
    template <typename T>
    void SetFromBytes(T *data, uint16_t offset = 0)
    {
        // TODO: Should just cache the size of the register
        fields_[offset].SetFromBytes((uint8_t *)data);
    }

    // Get from Byte Array
    uint16_t Get(uint8_t *data, uint16_t offset = 0)
    {
        // TODO: Should just cache the size of the register
        return fields_[offset].Get(data);
    }

    // void Set(uint8_t *data, size_t length)
    // {
    //     size_t offset = 0;
    //     for(auto reg : fields_)
    //     {
    //         reg.Set(data + offset);
    //         offset += reg.Size();
    //     }
    // }

    template <typename T>
    T Get(uint16_t offset = 0)
    {
        return fields_[offset].Get<T>();
    }

private:
    std::vector<RegisterData> fields_;
};


class RegisterInterface
{
public:
    static constexpr uint16_t kMaxRegisters = (1 << 12); // 12-bit addressing

    // TODO: Address field is bit too large.  But keeping clean alignment math here
    struct register_command_t
    {
        union // Union for mapping the full 64-byte data packet
        {
            struct
            {
                uint32_t rwx : 2;       // Read/Write/Execute
                uint32_t address : 12;  // 12-bit address (4096 Max Addresses)
                uint32_t data_type : 2; // Data Type: 12-bit fixed, 16-bit fixed, 32-bit fixed, 32-bit float
                uint8_t cmd_data[60];   // Data Buffer
                // 2 reserved Bytes
            }; 
            uint8_t data[64];           // Full 64-byte command buffer FDCAN
        };
    };

    struct register_reply_t
    {
        union // Union for mapping the full 64-byte data packet
        {
            struct
            {
                uint32_t id : 8;        // Node ID Reply
                uint32_t address : 12;  // 12-bit address (4096 Max Addresses)
                uint32_t code : 4;      // Return Code
                uint8_t cmd_data[60];   // Data Buffer Return
            }; 
            uint8_t data[64];           // Full 64-byte command buffer FDCAN
        };
    };

   // RegisterInterface(); 

    // Register Offset Map
    static void AddRegister(uint16_t address, Register *reg);

    static Register* GetRegister(uint16_t address);


    // Store Base Memory Address
    // Register Will Offset From There
    // Support Float Return Precision Options
    static void HandleCommand(FDCANDevice::FDCAN_msg_t &command);

private:
    static Register *register_map_[10];

};     // namespace RegisterInterface
#endif // REGISTER_INTERFACE_H_