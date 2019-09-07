/*
 * UserMenu.h
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

#ifndef CORE_USER_MENU_H_
#define CORE_USER_MENU_H_

// C System Files

// C++ System Files
#include <string>
#include <vector>

// Project Includes
#include "mbed.h"

class MenuItem
{
    friend class UserMenu;

public:
    MenuItem(const std::string name, char command_char, MenuItem *parent, void (*cb_ptr)() = nullptr);

    void AddSubMenu(MenuItem *menu);

protected:
    virtual void Show();
    virtual void Execute();
    virtual void Close() {}
    virtual void Handle(char c); // Handle Character Input from Serial Interrupt Handler
    inline char CommandID() { return command_; }
    inline const std::string &Name() { return name_; }

    // Variables
    std::vector<MenuItem *> sub_menus_;
    std::string name_;
    MenuItem *parent_menu_;
    char command_;

    // Callback Function
    void (*cb_ptr_)();
};

class MainMenu : public MenuItem
{

public:
    MainMenu(const std::string name, char command_char, MenuItem *parent, void (*cb_ptr)() = nullptr);

protected:
    virtual void Close();
};

class TorqueControlMenu : public MenuItem
{
public:
    TorqueControlMenu(const std::string name, char command_char, MenuItem *parent, void (*cb_ptr)() = nullptr);

protected:
    virtual void Handle(char c); // Handle Character Input from Serial Interrupt Handler
    virtual void Execute();
    int32_t buffer_index;
    char command_buffer[100];
};

class UserMenu
{

public:
    UserMenu(Serial *uart, MenuItem *root);
    void Show();
    static Serial* GetSerial() { return serial_; }
    static void Interrupt();
    static MenuItem *current_menu_;

private:
    static Serial *serial_;
    MenuItem *root_;
};

#endif // CORE_USER_MENU_H_