/*
 * NomadBLDC.h
 *
 *  Created on: March 20, 2021
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

#ifndef NOMADBLDC_LIB_H_
#define NOMADBLDC_LIB_H_

// C System Files

// C++ System Files
#include <future>

// Project Includes
#include <CAN/Registers.h>
#include <CAN/CANDevice.h>


// TODO: Timeout Pass
class RequestReply
{
public:
    RequestReply(uint32_t address, uint32_t timeout) : address_(address), timeout_(timeout)
    {
        reply_= promise_.get_future();
    }

    const uint32_t GetAddress() const { return address_; }
    bool Get(register_reply_t &reply)
    {
        auto status = reply_.wait_for(std::chrono::microseconds(timeout_));
        if (status == std::future_status::ready)
        {
            // TODO: Should we keep up with data sizes?  For now just mem copy all 60 cmd data bytes
            reply = reply_.get();
            return true;
            //memcpy(data, register_reply.cmd_data, register_reply.header.length);
            //return true;
        }
        else
        {
            std::cout << "Timed Out" << std::endl;
        }
        return false;
    }
    bool Set(register_reply_t &reply)
    {
        promise_.set_value(reply);
        return true;
    }

private:
    std::promise<register_reply_t> promise_;
    std::future<register_reply_t> reply_;
    uint32_t address_;
    uint32_t timeout_;
};

class NomadBLDC
{

public:

    NomadBLDC();
    NomadBLDC(int master_id, int servo_id, CANDevice *transport = nullptr);
    bool SetTransport(CANDevice *dev);
    bool Connect();
    void Disconnect();
    bool Reset();
    bool ClosedLoopTorqueCommand(float k_p, float k_d, float pos_ref, float vel_ref, float torque_ff);
    bool SetControlMode(uint32_t mode);
    void SetName(const std::string &name) { name_ = name; }
    const std::string& GetName() const { return name_; }

    // Servo State
    float GetPosition() { return joint_state_.Pos; }
    float GetVelocity() { return joint_state_.Vel; }
    float GetTorque() { return joint_state_.T_est; }

    uint32_t GetServoId() const { return servo_id_; }

    bool ReadRegister(uint32_t address, uint8_t *data);
    bool WriteRegister(uint32_t address, uint8_t *data, size_t size);

    // TODO: Request/Reply Wrapper Class
    bool ExecuteRegister(uint32_t address, uint8_t *parameter_data = nullptr, size_t size = 0, uint8_t *return_data = 0);

    // Force sync of all async request(when we implement it)
    bool Sync();

    void PrintState();

protected:

    // TODO: Servo "Pretty Name"
    int servo_id_;
    int master_id_;
    CANDevice *transport_;

    std::string name_;

    // Registers
    DeviceStatusRegister1_t dsr1_;
    DeviceStatusRegister2_t dsr2_;

    TorqueControlModeRegister_t tcmr_;

    JointState_t joint_state_;

    uint32_t control_mode_; // TODO: To Register

private:
    
    void ReceiveMessage(CANDevice::CAN_msg_t &msg);

    // Response for each register
    //std::promise<register_reply_t> promise_[1 << 8];

    std::vector<RequestReply> request_queue_;

    std::mutex requests_lock;
    // Connect Status
    bool connected_;
};

#endif // NOMADBLDC_LIB_H_