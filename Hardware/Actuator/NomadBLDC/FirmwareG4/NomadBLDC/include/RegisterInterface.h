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

typedef enum
{
    DeviceStatusRegister1 = 0x00,
    // Device Status Register1 Offsets
    DeviceFirmwareMajor = 0x01,
    DeviceFirmwareMinor = 0x02,
    DeviceUID1 = 0x03,
    DeviceUID2 = 0x04,
    DeviceUID3 = 0x05,
    DeviceUptime = 0x06,
    // Reserved = 0x07,
    // Reserved = 0x08,
    // Reserved = 0x09,
    // End Device Status Register
} DeviceRegisters_e;

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
    std::variant<uint8_t *, uint16_t *, uint32_t *, int8_t *, int16_t *, int32_t *, float *, std::function<void(void)>> data_;

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
       Logger::Instance().Print("Got: %d\r\n", sizeof(T));
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