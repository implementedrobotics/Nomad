/*
 * UserMenu.cpp
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

// Primary Include
#include "UserMenu.h"

// C System Files

// C++ System Files

// Project Includes
#include "mbed.h"
#include "rtos.h"

// Statics
MenuItem *UserMenu::current_menu_ = nullptr;

MenuItem::MenuItem(const std::string name, char command_char, MenuItem *parent, void (*cb_ptr)())
{
    sub_menus_.reserve(10); // Reserve/Preallocate some menu item holders
    command_ = command_char;
    name_ = name;
    parent_menu_ = 0;
    if(parent != nullptr)
        parent->AddSubMenu(this);

    // Set Callback
    cb_ptr_ = cb_ptr;
}

void MenuItem::AddSubMenu(MenuItem *menu)
{
    menu->parent_menu_ = this;
    sub_menus_.push_back(menu);
}

void MenuItem::Show()
{
    printf("\n\r");
    printf(" %s\r\n", name_.c_str()); // Menu Name
    for(uint32_t i = 0; i < sub_menus_.size(); i++)
    {
        printf("  %c - %s\r\n", sub_menus_[i]->CommandID(), sub_menus_[i]->Name().c_str()); // Sub Menus
        wait_us(10);
    }

    if(parent_menu_ != nullptr)
    {
        printf("  esc - Exit Menu\r\n"); // Sub Menus
        wait_us(10);
    }
}

// Main Menu
MainMenu::MainMenu(const std::string name, char command_char, MenuItem *parent, void (*cb_ptr)()) : MenuItem(name, command_char, parent, cb_ptr)
{
}

// TODO: Pass Command
void MainMenu::Execute()
{
    if(sub_menus_.size() > 0)
    {
        Show();
    }
    if(cb_ptr_ != nullptr) 
    {
        cb_ptr_(); // Callback
    }
}

void MainMenu::Close()
{
    printf("CLOSING\r\n");
}

Serial *UserMenu::serial_ = nullptr;
UserMenu::UserMenu(Serial *uart, MenuItem *menu)
{
    root_ = menu;                        // Set Root Menu
    current_menu_ = menu;                // Set Current Menu 
    serial_ = uart;                        // UART Handler
    serial_->attach(&UserMenu::Interrupt); // attach serial interrupt
}

void UserMenu::Show()
{
    root_->Show();
}

void UserMenu::Interrupt()
{
    while (serial_->readable())
    {
        char c = serial_->getc();
        if(c == 27 && current_menu_->parent_menu_ != nullptr) // Escape
        {
            current_menu_ = current_menu_->parent_menu_;
            current_menu_->Show();
        }
        for(uint32_t i = 0; i < current_menu_->sub_menus_.size(); i++)
        {
            if(current_menu_->sub_menus_[i]->CommandID() == c)
            {
                MenuItem *selected_menu = current_menu_->sub_menus_[i];
                current_menu_ = selected_menu;
                current_menu_->Execute();
            }
        }
    }
}