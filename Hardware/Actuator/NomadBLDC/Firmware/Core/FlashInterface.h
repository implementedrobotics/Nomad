/*
 * FlashInterface.h
 *
 *  Created on: August 28, 2019
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

#ifndef CORE_FLASH_INTERFACE_H_
#define CORE_FLASH_INTERFACE_H_

#include "stm32f4xx_flash.h"

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0 ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1 ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2 ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3 ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4 ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5 ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6 ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7 ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */

static const uint32_t __SECTOR_ADDRS[] = {
    ADDR_FLASH_SECTOR_0,
    ADDR_FLASH_SECTOR_1,
    ADDR_FLASH_SECTOR_2,
    ADDR_FLASH_SECTOR_3,
    ADDR_FLASH_SECTOR_4,
    ADDR_FLASH_SECTOR_5,
    ADDR_FLASH_SECTOR_6,
    ADDR_FLASH_SECTOR_7};

static const uint32_t __SECTORS[] = {
    FLASH_Sector_0,
    FLASH_Sector_1,
    FLASH_Sector_2,
    FLASH_Sector_3,
    FLASH_Sector_4,
    FLASH_Sector_6,
    FLASH_Sector_6,
    FLASH_Sector_7};

// C System Files

// C++ System Files

// Project Includes
#include "mbed.h"
#include "rtos.h"

class FlashInterface
{
public:
    typedef enum
    {
        READ = 0,
        WRITE = 1
    } mode_type_t;

    FlashInterface();
    ~FlashInterface();

    bool Open(uint32_t sector, mode_type_t mode = FlashInterface::READ);   // Open the flash sector for read/write
    bool Write(uint32_t index, uint8_t* buffer, size_t length); // Flash Write
    bool Read(uint32_t index, void *buffer, size_t length); // Flash Read
    bool Close();

    static FlashInterface &Instance();

private:

    bool ready_;        // Flash is Unlocked/Ready for R/W Operations
    mode_type_t mode_;   // In Write Mode
    uint32_t sector_;   // Sector ID
    uint32_t base_;     // Sector Address Base

};

#endif // CORE_FLASH_INTERFACE_H_