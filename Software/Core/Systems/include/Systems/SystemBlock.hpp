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



namespace Core::Systems
{
    class BlockDiagram;
    class SystemBlock
    {
        
        friend class BlockDiagram;

    public:
        // Base Class System Block Node
        // name = Task Name
        // T_s = Sample Time (-1 for inherit)
        SystemBlock(const std::string &name, const double T_s = -1);

        // Add Subsystem to System Block
        void AddSubSystem();

        // Get Output Port
        std::shared_ptr<Communications::Port> GetOutputPort(const int port_id) const;

        // Get Input Port
        std::shared_ptr<Communications::Port> GetInputPort(const int port_id) const;

        // Set Transport Configuration for Port
        void SetPortOutput(const int port_id, const Communications::Port::TransportType transport, const std::string &transport_url, const std::string &channel);

        // Overriden Run Function
        virtual void Run(double d_t);

        // Overriden Setup Function
        virtual void Setup();

        // Return System Name
        const std::string& Name() const { return name_;}

    protected:

        static const int MAX_PORTS = 16;
        
        // Update function for stateful outputs
        virtual void UpdateStateOutputs() {}

        // Update function for stateless outputs
        virtual void UpdateStatelessOutputs() {} 

        // Update fucntion for next state from inputs
        virtual void UpdateState() {}

        // Sampling Time (s)
        double T_s_;

        // Current Time (s)
        double T_;

        // Last Sample Time (s)
        double T_prev_;

        // Parent Block Diagram/System
        BlockDiagram *parent_;

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
        ConstantBlock(const Eigen::VectorXd &value, const double T_s = -1) : SystemBlock("CONSTANT", T_s)
        {
            constant_.length = value.size();
            constant_.data.resize(constant_.length);
            
            // Map to output message.  Would love to have this be eigen types...
            Eigen::Map<Eigen::VectorXd>(constant_.data.data(), constant_.length) = value;

            // Create Output Port
            output_port_map_[0] = std::move(Communications::Port::CreateOutput("CONSTANT", T_s_));
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

        // Transport Type Enum
        enum OperandType
        {
            ADD = 0,
            MINUS
        };

        // Constant System Block Node
        // name = Task Name
        AddBlock(const double T_s = -1) : SystemBlock("ADD", T_s)
        {
            // Create Output Port
            output_port_map_[0] = Communications::Port::CreateOutput("ADD", T_s_);
        }

        void AddInput(OperandType op_type, const int dimension)
        {
            // Update Dimension
            if(!operands_.empty())
            {
                assert(dimension_ == dimension || operands_.size() > MAX_PORTS);
            }
            else 
            {
                dimension_ = dimension;
                result_.length = dimension_;
                result_.data.resize(result_.length);

                // Zero output result
                Eigen::Map<Eigen::VectorXd>(result_.data.data(), result_.length) = Eigen::Vector3d::Zero(dimension_);
            }

            // Create Port
            input_port_map_[operands_.size()] = Communications::Port::CreateInput<double_vec_t>(std::to_string(operands_.size()), T_s_);
            
            // Update Operation Type
            operation_types_.push_back(op_type);

            // Add Operand Input
            double_vec_t operand;
            operand.length = dimension;
            operand.data.resize(operand.length);

            // Zero initial state
            Eigen::Map<Eigen::VectorXd>(operand.data.data(), operand.length) = Eigen::Vector3d::Zero(dimension_);
            operands_.push_back(operand);
        }
        

    protected:

        // Update function for stateful outputs
        void UpdateStateOutputs()
        {

        }

        // Update function for stateless outputs
        void UpdateStatelessOutputs()
        {
            if(operands_.empty())
            {
                return;
            }          
            
            // Read Input
            GetInputPort(0)->Receive(operands_[0]);
            Eigen::VectorXd out_result = Eigen::Map<Eigen::VectorXd>(operands_[0].data.data(), dimension_);

            for(int i = 1; i < operands_.size(); i++)
            {
                GetInputPort(i)->Receive(operands_[i]);
                Eigen::VectorXd operand = Eigen::Map<Eigen::VectorXd>(operands_[i].data.data(), dimension_);
                switch (operation_types_[i])
                {
                case MINUS:
                    operand = operand * -1; // Negate it
                    break;
                
                default:
                    break;
                }

                out_result = out_result + operand;
            }

            // Map to output message.  Would love to have this be eigen types...
            Eigen::Map<Eigen::VectorXd>(result_.data.data(), result_.length) = out_result;

            GetOutputPort(0)->Send(result_);

            std::cout << "SENDING: " << out_result << std::endl;
        }

        // Update function for next state from inputs
        void UpdateState()
        {

        }

        // Dimension of operand vectors.  Must be equal 
        int dimension_;

        std::vector<double_vec_t> operands_;

        std::vector<OperandType> operation_types_;
        
        double_vec_t result_;
    };


} // namespace Controllers::Systems

#endif // NOMAD_CORE_CONTROLLERS_SYSTEMBLOCK_H_
