/*
 * msg_helper.hpp
 *
 *  Created on: May 10, 2021
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
 */

#ifndef NOMAD_MSG_HELPERS_H_
#define NOMAD_MSG_HELPERS_H_

// C Includes

// C++ Includes
#include <iostream>

// Project Includes
#include <Communications/Messages/com_state_t.hpp>
#include <Communications/Messages/joint_state_t.hpp>
#include <Communications/Messages/joint_control_cmd_t.hpp>
#include <Communications/Messages/full_state_t.hpp>
#include <Communications/Messages/sim_data_t.hpp>
#include <Communications/Messages/teleop_data_t.hpp>
#include <Communications/Messages/imu_data_t.hpp>
#include <Communications/Messages/leg_controller_cmd_t.hpp>
#include <Communications/Messages/double_vec_t.hpp>

// Third Party Includes


namespace Communications
{
    template <typename T>
    struct conv;

    template <>
    struct conv<double_vec_t>
    {
        static void Demux()
        {
            std::cout << "DVT1" << std::endl;
        }
    };

    template <>
    struct conv<com_state_t>
    {
        static void Demux()
        {
            std::cout << "DVT2" << std::endl;
        }
    };

        template <>
    struct conv<full_state_t>
    {
        static void Demux()
        {
            std::cout << "DVT3" << std::endl;
        }
    };

        template <>
    struct conv<joint_state_t>
    {
        static void Demux()
        {
            std::cout << "DVT4" << std::endl;
        }
    };

            template <>
    struct conv<joint_control_cmd_t>
    {
        static void Demux()
        {
            std::cout << "DVT5" << std::endl;
        }
    };

                template <>
    struct conv<sim_data_t>
    {
        static void Demux()
        {
            std::cout << "DVT6" << std::endl;
        }
    };

                    template <>
    struct conv<teleop_data_t>
    {
        static void Demux()
        {
            std::cout << "DVT7" << std::endl;
        }
    };


                    template <>
    struct conv<imu_data_t>
    {
        static void Demux()
        {
            std::cout << "DVT8" << std::endl;
        }
    };


                    template <>
    struct conv<leg_controller_cmd_t>
    {
        static void Demux()
        {
            std::cout << "DVT9" << std::endl;
        }
    };

} // namespace Communications

#endif // NOMAD_MSG_HELPERS_H_
