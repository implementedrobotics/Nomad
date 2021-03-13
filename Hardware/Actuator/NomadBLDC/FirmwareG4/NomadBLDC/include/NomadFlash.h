/*
 * NomadFlash.h
 *
 *  Created on: March 12, 2021
 *      Author: Quincy Jones
 *
 * Copyright (c) <2021> <Quincy Jones - quincy@implementedrobotics.com/>
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

#ifndef CORE_NOMAD_FLASH_H_
#define CORE_NOMAD_FLASH_H_

// C System Files

// C++ System Files

// Project Includes
#include <Peripherals/flash.h>
#include <Peripherals/fdcan.h>
#include <Motor.h>
#include <MotorController.h>
#include <nomad_hw.h>

// Flash Save Struct.  TODO: Move to own file
#define FLASH_VERSION 2
#define FLASH_SAVE_SIGNATURE 0x78D5FC00
#define FLASH_BASE_ADDRESS ADDR_FLASH_PAGE_60


// Must be 8-byte aligned for easy flash read/write
struct Save_format_t
{
    uint32_t signature;
    uint32_t version;
    Motor::Config_t motor_config __attribute__((__aligned__(8))) ;
    uint8_t motor_reserved[128]; // Reserved;
    PositionSensorAS5x47::Config_t position_sensor_config __attribute__((__aligned__(8))) ;
    uint8_t position_reserved[128]; // Reserved;
    MotorController::Config_t controller_config __attribute__((__aligned__(8))) ;
    uint8_t controller_reserved[128]; // Reserved;
    FDCANDevice::Config_t can_config __attribute__((__aligned__(8))) ;
    uint8_t can_reserved[128]; // Reserved;
};

class NomadFlash
{

public:
    static bool Open(bool write = false)
    {
        Save_format_t flash;
        bool status;
        if (write) // Writing
        {
            FlashDevice::Instance().Open(FLASH_BASE_ADDRESS, sizeof(flash), FlashDevice::WRITE);

            // Update Signatures
            flash.signature = FLASH_SAVE_SIGNATURE;
            flash.version = FLASH_VERSION; // Set Version

            // Write Flash
            status = FlashDevice::Instance().Write(0, (uint8_t *)&flash, 8);
        }
        else // Reading
        {
            FlashDevice::Instance().Open(FLASH_BASE_ADDRESS, sizeof(flash), FlashDevice::READ);

            // Read signature
            status = FlashDevice::Instance().Read(0, (uint8_t *)&flash, sizeof(Save_format_t));

            if (!status)
            {
                return false;
            }

            bool loaded = (flash.signature == FLASH_SAVE_SIGNATURE && flash.version == FLASH_VERSION);
            if (!loaded)
            {
                return false;
            }
        }
        return status;
    }
    static bool LoadMotorConfig(Motor::Config_t &config)
    {
        bool status = FlashDevice::Instance().Read(offsetof(struct Save_format_t, motor_config), (uint8_t *)&config, sizeof(Motor::Config_t));
        return status;
    }

    static bool SaveMotorConfig(Motor::Config_t &config)
    {
        bool status = FlashDevice::Instance().Write(offsetof(struct Save_format_t, motor_config), (uint8_t *)&config, sizeof(Motor::Config_t));
        return status;
    }

    static bool LoadControllerConfig(MotorController::Config_t &config)
    {
        bool status = FlashDevice::Instance().Read(offsetof(struct Save_format_t, controller_config), (uint8_t *)&config, sizeof(MotorController::Config_t));
        return status;
    }
    static bool SaveControllerConfig(MotorController::Config_t &config)
    {
        bool status = FlashDevice::Instance().Write(offsetof(struct Save_format_t, controller_config), (uint8_t *)&config, sizeof(MotorController::Config_t));
        return status;
    }

    static bool LoadPositionSensorConfig(PositionSensorAS5x47::Config_t &config)
    {
        bool status = FlashDevice::Instance().Read(offsetof(struct Save_format_t, position_sensor_config), (uint8_t *)&config, sizeof(PositionSensorAS5x47::Config_t));
        return status;
    }

    static bool SavePositionSensorConfig(PositionSensorAS5x47::Config_t &config)
    {
        bool status = FlashDevice::Instance().Write(offsetof(struct Save_format_t, position_sensor_config), (uint8_t *)&config, sizeof(PositionSensorAS5x47::Config_t));
        return status;
    }

    static bool LoadCANConfig(FDCANDevice::Config_t &config)
    {        
        bool status = FlashDevice::Instance().Read(offsetof(struct Save_format_t, can_config), (uint8_t *)&config, sizeof(FDCANDevice::Config_t));
        return status;
    }
    static bool SaveCANConfig(FDCANDevice::Config_t &config)
    {
        bool status = FlashDevice::Instance().Write(offsetof(struct Save_format_t, can_config), (uint8_t *)&config, sizeof(FDCANDevice::Config_t));
        return status;
    }

    static bool LoadAll(Save_format_t &load_data)
    {        
        bool status = FlashDevice::Instance().Read(0, (uint8_t *)&load_data, sizeof(Save_format_t));
        return status;
    }
    
    static bool Close()
    {
        return FlashDevice::Instance().Close();
    }
    
private:
 
};

#endif // CORE_NOMAD_FLASH_H_