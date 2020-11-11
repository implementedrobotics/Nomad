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

// Project Includes
#include <Peripherals/uart.h>
#include <Peripherals/fdcan.h>
#include <Logger.h>

class RegisterData
{
public:
    template <typename T>
    RegisterData(T value)
    {
        data_ = value;
       // Logger::Instance().Print("Data: %f\r\n", *value);
    }

    void Set(uint8_t *value)
    {
        if (auto data = std::get_if<uint8_t *>(&data_))
        {
            **data = *((uint8_t *)value);
            Logger::Instance().Print("Variant Value8: %d\r\n", **data);
        }
        else if (auto data = std::get_if<uint16_t *>(&data_))
        {
            **data = *((uint16_t *)value);
            Logger::Instance().Print("Variant Value16: %d\r\n", **data);
        }
        else if (auto data = std::get_if<uint32_t *>(&data_))
        {
            **data = *((uint32_t *)value);
            Logger::Instance().Print("Variant Value32: %d\r\n", **data);
        }
        else if (auto data = std::get_if<int8_t *>(&data_))
        {
            **data = *((uint8_t *)value);
            Logger::Instance().Print("Variant Value8: %d\r\n", **data);
        }
        else if (auto data = std::get_if<int16_t *>(&data_))
        {
            **data = *((int16_t *)value);
            Logger::Instance().Print("Variant Value16: %d\r\n", **data);
        }
        else if (auto data = std::get_if<int32_t *>(&data_))
        {
            **data = *((int32_t *)value);
            Logger::Instance().Print("Variant Value32: %d\r\n", **data);
        }
        else if (auto data = std::get_if<float *>(&data_))
        {
            **data = *((float *)value);
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

    size_t Size() const
    {
        return data_sizes_[data_.index()];
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
};

class Register
{

public:

    Register() : num_fields_(0)
    {

    }
    Register(uint16_t num_fields) : num_fields_(num_fields)
    {
        fields_.reserve(num_fields_);
    }

    template <typename T>
    void AddDataField(uint16_t offset, T value)
    {
        fields_[offset] = RegisterData(value);
    }

    template <typename T>
    void Set(uint16_t offset, T value)
    {
        fields_[offset].Set(value);
    }

    void Set(uint16_t offset, uint8_t *data)
    {
        fields_[offset].Set(data);
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
    T Get(uint16_t offset)
    {
        return fields_[offset].Get<T>();
    }

private:
    std::vector<RegisterData> fields_;
    uint16_t num_fields_;
};

class RegisterInterface
{
public:
    static constexpr uint16_t kMaxRegisters = (1<<12);

    // TODO: Address field is too large.  But keeping clean alignment math here
    struct register_command_t
    {
        union
        {
            struct
            {
                uint32_t rwx : 2;       // Read/Write/Execute
                uint32_t address : 12;  // 12-bit address (4096 Max Addresses)
                uint32_t data_type : 2; // Data Type: 12-bit fixed, 16-bit fixed, 32-bit fixed, 32-bit float
            };
            uint8_t data[64];
        };
    };

    typedef enum
    {
        MotorStatus = 0x00,
        ControllerStatus = 0x01,
    } RegisterAddress_e;


    static void HandleCommand(FDCANDevice::FDCAN_msg_t &command);

    RegisterInterface();

    // Register Offset Map
    void AddRegister(uint16_t lookup_address, Register *reg);

    // Store Base Memory Address
    // Register Will Offset From There
    // Support Float Return Precision Options

private:

    static Register* register_map_[10];

};     // namespace RegisterInterface
#endif // REGISTER_INTERFACE_H_