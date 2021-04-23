/*
 * FlashInterface.cpp
 *
 *  Created on: August 29, 2019
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
#include <Peripherals/flash.h>

// C System Files
#include <cstring>

// C++ System Files

// Project Includes
#include "stm32g4xx_hal_flash.h"

FlashDevice::FlashDevice() : ready_(false), mode_(FlashDevice::READ), base_address_(0)
{
}

FlashDevice::~FlashDevice()
{
    // Close if opened
    Close();
}
bool FlashDevice::Open(uint32_t address, size_t size, mode_type_t mode)
{

    // Check for Valid Page
    if ((address + size) > ADDR_FLASH_PAGE_END)
        return false;
    else if (ready_ == true) // Flash opened already, can't open again!
        return false;

    // Set Sector Address/Base Values
    base_address_ = address;
    mode_ = mode;                       // Set Mode
    if (mode_ == FlashDevice::WRITE) // Open for write
    {
        /* Unlock to enable control register access */
        HAL_FLASH_Unlock(); // Unlock Flash
        
        /* Clear OPTVERR bit set on virgin samples */
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

        /* Get Base Page */
        uint32_t base_page = GetPage(base_address_);
        
        /* Get Total Number of Pages */
        uint32_t total_pages = GetPage(base_address_ + size) - base_page + 1;
    
        FLASH_EraseInitTypeDef EraseInitStruct;

        /* Fill EraseInit structure*/
        EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
        EraseInitStruct.Banks = FLASH_BANK_1;
        EraseInitStruct.Page = base_page;
        EraseInitStruct.NbPages = total_pages;

        uint32_t error;
        if(HAL_FLASHEx_Erase(&EraseInitStruct, &error) != HAL_OK)
        {
            return false;
        }
    }

    ready_ = true;
    return true; // Sucessfully Opened
}

bool FlashDevice::Write(uint32_t index, uint8_t *buffer, size_t length)
{
    if (!ready_ || mode_ != FlashDevice::WRITE) // Need to be Ready and in Write Mode
        return false;

    uint32_t write_addr = base_address_ + index;
    uint64_t data_64;
    for (uint32_t i = 0; i < length; i=i+8) // Loop buffer and write out in bytes
    {
        memcpy(&data_64, buffer, 8); 
        //HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, write_addr, ((uint64_t *)buffer)[i]);
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, write_addr, data_64) != HAL_OK)
        {
            return false;
        }
        write_addr = write_addr + 8;
        buffer = buffer + 8;
    }
    return true;
}

bool FlashDevice::Read(uint32_t index, void *buffer, size_t length)
{
    if (!ready_) // Can always read even in write mode...
        return false;

    memcpy(buffer, (uint8_t *)(base_address_ + index), sizeof(uint8_t) * length);

    return true;
}

bool FlashDevice::Close()
{
    if (!ready_) // If not opened nothing to do, return
        return false;

    if (mode_ == FlashDevice::WRITE) // Lock Flash if we were in write mode
        HAL_FLASH_Lock();

    mode_ = FlashDevice::READ; // Default in read mode
    ready_ = false;

    return true;
}

uint32_t FlashDevice::GetPage(uint32_t address)
{
  uint32_t page = 0;

  if (address < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (address - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (address - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

FlashDevice &FlashDevice::Instance()
{
    static FlashDevice instance;
    return instance;
}