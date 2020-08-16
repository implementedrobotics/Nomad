/*
 * SystemBlock.hpp
 *
 *  Created on: August 15, 2020
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
 */

#ifndef NOMAD_CORE_CONTROLLERS_SYSTEMBLOCK_H_
#define NOMAD_CORE_CONTROLLERS_SYSTEMBLOCK_H_

// C System Files

// C++ System Files
#include <iostream>
#include <string>
#include <memory>

// Project Include Files
#include <Eigen/Dense>
#include <Communications/Port.hpp>
#include <Communications/Messages/double_vec_t.hpp>

namespace Controllers::Systems
{
    class SystemBlock
    {
        static const int MAX_PORTS = 16;

    public:
        // Base Class System Block Node
        // name = Task Name
        SystemBlock(const std::string &name);

        // Add Subsystem to System Block
        void AddSubSystem();

        // Get Output Port
        std::shared_ptr<Communications::Port> GetOutputPort(const int port_id) const;

        // Get Input Port
        std::shared_ptr<Communications::Port> GetInputPort(const int port_id) const;

        // Set Transport Configuration for Port
        void SetPortOutput(const int port_id, const Communications::Port::TransportType transport, const std::string &transport_url, const std::string &channel);

        // Overriden Run Function
        virtual void Run();

        // Overriden Setup Function
        virtual void Setup();

        // Connect Function
        virtual void Connect(std::shared_ptr<Communications::Port> output, std::shared_ptr<Communications::Port> input);

    protected:



        // Update function for stateful outputs
        virtual void UpdateStateOutputs() {}

        // Update function for stateless outputs
        virtual void UpdateStatelessOutputs() {} 

        // Update fucntion for next state from inputs
        virtual void UpdateState() {}

        // Sampling Time (s)
        double T_s_;

        // System Name
        std::string name_;

        // Input Port Map
        std::shared_ptr<Communications::Port> input_port_map_[MAX_PORTS];

        // Output Port Map
        std::shared_ptr<Communications::Port> output_port_map_[MAX_PORTS];

    };

    class ConstantBlock : public SystemBlock
    {

    public:
        // Constant System Block Node
        // name = Task Name
        ConstantBlock(const Eigen::VectorXd &value) : SystemBlock("CONSTANT")
        {
            constant_.length = value.size();
            constant_.data.resize(constant_.length);
            
            // Map to output message.  Would love to have this be eigen types...
            Eigen::Map<Eigen::VectorXd>(constant_.data.data(), constant_.length) = value;

            // Create Output Port
            output_port_map_[0] = Communications::Port::CreateOutput("CONSTANT", T_s_);
        }

    protected:

        // Update function for stateful outputs
        void UpdateStateOutputs()
        {

        }

        // Update function for stateless outputs
        void UpdateStatelessOutputs()
        {
            GetOutputPort(0)->Send(constant_);

            std::cout << "SENDING!" << std::endl;
        }

        // Update fucntion for next state from inputs
        void UpdateState()
        {

        }

        //
        //Eigen::VectorXd constant_;

        double_vec_t constant_; 
    };

    class AddBlock : public SystemBlock
    {

    public:
        // Constant System Block Node
        // name = Task Name
        AddBlock() : SystemBlock("ADD")
        {

            // TODO: From Connect/Verify all vectors same length
            result.length = 3;//value.size();
            result.data.resize(result.length);

            // TODO: Move to Connect/Setup
            operands[0].length = 3;//value.size();
            operands[0].data.resize(operands[0].length);

            operands[1].length = 3;//value.size();
            operands[1].data.resize(operands[1].length);
            
            input_port_map_[0] = Communications::Port::CreateInput<double_vec_t>("A", T_s_);
            input_port_map_[1] = Communications::Port::CreateInput<double_vec_t>("B", T_s_);

            // Zero Outputs
            Eigen::Map<Eigen::VectorXd>(operands[0].data.data(), operands[0].length) = Eigen::Vector3d::Zero(3);
            Eigen::Map<Eigen::VectorXd>(operands[1].data.data(), operands[0].length) = Eigen::Vector3d::Zero(3);

            Eigen::Map<Eigen::VectorXd>(result.data.data(), result.length) = Eigen::Vector3d::Zero(3);


            // Create Output Port
            output_port_map_[0] = Communications::Port::CreateOutput("ADD", T_s_);
        }

    protected:

        // Update function for stateful outputs
        void UpdateStateOutputs()
        {

        }

        // Update function for stateless outputs
        void UpdateStatelessOutputs()
        {
            
            // Read Input
            GetInputPort(0)->Receive(operands[0]);
            GetInputPort(1)->Receive(operands[1]);
            
            Eigen::VectorXd A = Eigen::Map<Eigen::VectorXd>(operands[0].data.data(), 3);
            Eigen::VectorXd B = Eigen::Map<Eigen::VectorXd>(operands[1].data.data(), 3);

            Eigen::VectorXd out_vec = A + B;

            // Map to output message.  Would love to have this be eigen types...
            Eigen::Map<Eigen::VectorXd>(result.data.data(), result.length) = out_vec;


            GetOutputPort(0)->Send(result);

            std::cout << "SENDING: " << out_vec << std::endl;
        }

        // Update fucntion for next state from inputs
        void UpdateState()
        {

        }

        //
        //Eigen::VectorXd constant_;
        double_vec_t operands[2]; 
        double_vec_t result;
    };


} // namespace Controllers::Systems

#endif // NOMAD_CORE_CONTROLLERS_SYSTEMBLOCK_H_
