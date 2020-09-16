
/*
 * Logger.cpp
 *
 *  Created on: March 27, 2020
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

// Primary Include
#include "Logger.h"

// C System Files

// C++ System Files
#include <cstdarg>
#include <string>
#include <vector>

// Project Includes
#include "CommandHandler.h"

Logger::Logger() : enable_logging_(false) 
{

}

// Singleton Insance
Logger &Logger::Instance()
{
    static Logger instance;
    return instance;
}

// Enable/Disable Logging
void Logger::Enable(bool enable)
{
    enable_logging_ = enable;
}

// Formatted Logging print function
void Logger::Print(const char *format ...) 
{
    if(!enable_logging_) // Logging not currently enabled
        return;

    // Variable argument array list
    va_list vaArgs;
    va_start(vaArgs, format);

    va_list vaCopy;
    va_copy(vaCopy, vaArgs);
    const int iLen = std::vsnprintf(NULL, 0, format, vaCopy);
    va_end(vaCopy);

    // Return formatted string
    std::vector<char> zc(iLen + 1);
    std::vsnprintf(zc.data(), zc.size(), format, vaArgs);
    va_end(vaArgs);

    // Log command
    CommandHandler::LogCommand(std::string(zc.data(), zc.size()));
} 
