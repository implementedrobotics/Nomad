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
#include <algorithm>

// Project Includes
#include <CAN/Registers.h>
#include <CAN/CANDevice.h>


// TODO: Timeout Pass
// TODO: Not all request expect a reply
class RequestReply
{
public:

    RequestReply(uint32_t address, uint32_t timeout) : timeout_(timeout), num_requests_(1), num_replies_(0)
    {
        // Make sure new promises and futures
        promises_.push_back({});
        futures_.push_back({});

        // Link them
        futures_.back() = promises_.back().get_future();
        
        // Add requested address
        requests_.push_back(address);

        synced_future_ = synced_promise_.get_future();


    }

    RequestReply(std::vector<uint32_t> addresses, uint32_t timeout) :  timeout_(timeout), num_replies_(0)
    {
        // Reserve for address size
        requests_.reserve(addresses.size());
        for (auto &address : addresses) // Loop and add promise/futures
        {
            // Make sure new promises and futures
            promises_.push_back({});
            futures_.push_back({});

            // Link Them
            futures_.back() = promises_.back().get_future();

            // Add requested address
            requests_.insert(requests_.end(), addresses.begin(), addresses.end());
        }

        synced_future_ = synced_promise_.get_future();

        // TODO: Can probably remove this
        num_requests_ = requests_.size();
        
    }
    
    inline bool CheckAddress(uint32_t address) 
    {
        ptrdiff_t pos = std::distance(requests_.begin(), std::find(requests_.begin(), requests_.end(), address));
        if (pos >= requests_.size())
        {
            return false;
        }
        return true;
    }

    inline bool Get(std::vector<register_reply_t> &replies)
    {
        for(auto& future : futures_)
        {
            auto status = future.wait_for(std::chrono::microseconds(timeout_));
            if (status == std::future_status::ready)
            {
                // TODO: Should we keep up with data sizes?
                register_reply_t reply = future.get();
                replies.push_back(reply);
                //memcpy(data, register_reply.cmd_data, register_reply.header.length);
            }
            else
            {
                std::cout << "Timed Out" << std::endl;
                return false;
            }
        }
        return true;
    }

    inline bool Set(register_reply_t &reply)
    {
        // Find Address Location
        ptrdiff_t pos = std::distance(requests_.begin(), std::find(requests_.begin(), requests_.end(), reply.header.address));
        if (pos >= requests_.size())
        {
            // Not there.  Bail
            return false;
        }
        // Update promise
        promises_[pos].set_value(reply);
        
        // Update valid replies
        num_replies_++;

        return true;
    }

    inline bool isFulfilled() const
    {
        return num_replies_ >= requests_.size();
    }

    inline void Sync()
    {
       synced_promise_.set_value();
    }

    bool Wait(int32_t timeout = -1)
    {
        if (timeout < 0)
        {
            synced_future_.wait();
            return true;
        }
        else
        {
            auto status = synced_future_.wait_for(std::chrono::microseconds(timeout));
            if (status == std::future_status::ready)
            {
                return true;
            }
            else
            {
                std::cout << "Timed Out" << std::endl;
                return false;
            }
        }
    }

private:
    std::vector<std::promise<register_reply_t>> promises_;
    std::vector<std::future<register_reply_t>> futures_;

    std::promise<void> synced_promise_;
    std::future<void> synced_future_;

    // TODO: Make this Register(address, memory)
    std::vector<uint32_t> requests_;
    uint32_t timeout_;
    uint32_t uuid_;
    uint32_t num_requests_;

    // Number of replies received
    uint32_t num_replies_;
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

    void UpdateRegisters(RequestReply &reply);

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
    static constexpr uint16_t kMaxRegisters = (1 << 8); // 8-bit addressing
    void ReceiveMessage(CANDevice::CAN_msg_t &msg);
    void SetupRegisterMap();

    // Response for each register
    //std::promise<register_reply_t> promise_[1 << 8];

    std::vector<RequestReply> request_queue_;

    std::mutex requests_lock;

    std::mutex update_lock;

    
    std::vector<uint8_t *> register_map_;

    // Connect Status
    bool connected_;
};

#endif // NOMADBLDC_LIB_H_