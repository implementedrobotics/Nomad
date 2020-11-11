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

// Project Includes
#include <Peripherals/uart.h>
#include <Peripherals/fdcan.h>




// C System Files

// C++ System Files
#include <variant>
#include <vector>
#include <cstdint>
#include <iostream>
#include <memory>
// Project Includes
/*

class RegisterData
{
public:

    template <typename T>
    RegisterData(T value)
    {
        data_ = value;
        printf("Data: %d\n", *value);
    }

  // template <typename T>
   void Set(uint8_t *value)
   {
       if (auto data = std::get_if<uint32_t *>(&data_))
       {
           **data = *((uint32_t*)value);
           std::cout << "variant value: " << **data << '\n';
       }
   }

private:
    // Pointer to register memory location
    std::variant<uint8_t *, uint32_t *> data_;
};

class Register
{
public:
    void Get();

    template <typename T>
    void Set(uint16_t offset, T &value)
    {
        //fields_[offset].
    }

private:
    std::vector<RegisterData> fields_;
};


int main()
{
    uint32_t a;
    a = 10;
    printf("Pointer: %p\n", &a);
   // std::cout << "Hello: " << &a << std::endl;
    RegisterData data(&a);

    uint32_t newBuf = 100005;
    data.Set((uint8_t *)&newBuf);

    printf("Got: %d\n", a);
    return 0;
}



*/
class RegisterData
{

public:
    template <typename T>
    void Set(T &value)
    {
        if(type_ == DataType::FLOAT)
        {
            (float *)data_ = value;
        }
        std::get_if
    }

    private:
    // Pointer to register memory location
    std::variant<uint8_t*, uint16_t*, uint32_t*, int8_t*, int16_t*, int32_t*, float*> data_;
};

class Register
{
public:
    void Get();

    template <typename T>
    void Set(uint16_t offset, T &value)
    {
        //fields_[offset].
    }

private:
    std::vector<RegisterData> fields_;
};

class RegisterInterface
{
    //struct RegisterData {
    //     // void *data; // Pointer to register memory location
    //     // void Set(T &value)
    //     // {
    //     //     // If Type is float then do -> Check Conversion
    //     // }

    //     // template <typename T>
    //     // T Get()
    //     // {

    //     // }
    //     // std::variant<uint8_t, uint16_t, uint32_t, float> value;
    // };
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
    void AddRegister(uint16_t lookup_address, int *mem_address);
    // Store Base Memory Address
    // Register Will Offset From There
    // Support Float Return Precision Options

private:

    uint32_t register_map_[10];

};     // namespace RegisterInterface
#endif // REGISTER_INTERFACE_H_