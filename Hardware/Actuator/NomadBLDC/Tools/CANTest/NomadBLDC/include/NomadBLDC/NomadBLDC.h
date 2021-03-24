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

    // RequestReply(uint32_t address, uint32_t timeout) : timeout_(timeout), num_requests_(1), num_replies_(0)
    // {
    //     // Make sure new promises and futures
    //     promises_.push_back({});
    //     futures_.push_back({});

    //     // Link them
    //     futures_.back() = promises_.back().get_future();
        
    //     // Add requested address
    //     requests_.push_back(address);

    //     synced_future_ = synced_promise_.get_future();


    // }

    RequestReply(std::vector<Register> &registers, uint32_t timeout) :  timeout_(timeout), num_replies_(0)
    {
        // Reserve for address size
        request_regs_.reserve(registers.size());
        for (auto &reg : registers) // Loop and add promise/futures
        {
            // Make sure new promises and futures
            promises_.push_back({});
            futures_.push_back({});

            // Link Them
            futures_.back() = promises_.back().get_future();

            // Add requested address
            //requests_.insert(requests_.end(), addresses.begin(), addresses.end());
            request_regs_.push_back(reg);
            //requests_.push_back(reg.address);
        }

        synced_future_ = synced_promise_.get_future();
        
    }
    
    inline bool CheckAddress(uint32_t address) 
    {
        // Find Address Location
        ptrdiff_t pos = std::distance(request_regs_.begin(), std::find_if(request_regs_.begin(),
                                                                          request_regs_.end(),
                                                                          [&address](const Register f) -> bool { return f.address == address; }));
        //  ptrdiff_t pos = std::distance(requests_.begin(), std::find(requests_.begin(), requests_.end(), address));
        if (pos >= request_regs_.size())
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
         uint32_t address = reply.header.address;
        // Find Address Location

        //ptrdiff_t pos = std::distance(requests_.begin(), std::find(requests_.begin(), requests_.end(), reply.header.address));

        ptrdiff_t pos = std::distance(request_regs_.begin(), std::find_if(request_regs_.begin(),
                                                                          request_regs_.end(),
                                                                          [&address](const Register f) -> bool { return f.address == address; }));

        if (pos >= request_regs_.size())
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
        return num_replies_ >= request_regs_.size();
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
    std::vector<Register> request_regs_;
    //std::vector<uint32_t> requests_;
    uint32_t timeout_;

    // Number of replies received
    uint32_t num_replies_;
};

// Request register of CAN bus
class Requester {

    public:
        Requester(CANDevice *transport, uint32_t servo_id, uint32_t master_id) : transport_(transport), servo_id_(servo_id), master_id_(master_id)
        {

        }
        RequestReply &CreateRequest(std::vector<Register> &registers, uint16_t request_type, uint32_t timeout)
        {
            // Hold Request Messages
            std::vector<CANDevice::CAN_msg_t> request_msgs;

            // TODO: Pass Registers
            // Loop requested address and cache our request messages
            for (Register& reg : registers)
            {
                register_command_t read_cmd;
                read_cmd.header.rwx = request_type; // 0 Read, 1 Write, 2 Executure
                read_cmd.header.address = reg.address;
                read_cmd.header.data_type = 1; // TODO: Everything is 32-bit for now...
                read_cmd.header.sender_id = master_id_;
                read_cmd.header.length = 0;

                CANDevice::CAN_msg_t msg;
                msg.id = servo_id_;
                msg.length = sizeof(request_header_t);
                memcpy(msg.data, &read_cmd, msg.length);

                // Save it
                request_msgs.push_back(msg);
            }

            // Lock Request Queue
            requests_lock_.lock();
            // Generate Request
            // TODO: Only generate these for request expecting replies
            request_queue_.push_back({registers, timeout});
            auto &request = request_queue_.back();
            requests_lock_.unlock();

            // Send Request Message
            for (CANDevice::CAN_msg_t &msg : request_msgs)
            {
                transport_->Send(msg);
            }

            return request;
        }
        // bool SendRequest()
        // {
        // }

        void ProcessReply(register_reply_t *reply)
        {
            requests_lock_.lock();
            for (int i = request_queue_.size() - 1; i >= 0; i--)
            {
                auto &request = request_queue_[i];

                // TODO: Check for Stale request.  Timed out, already fulfilled etc
                if (request.isFulfilled())
                {
                    request_queue_.erase(request_queue_.begin() + i);
                    continue;
                }
                if (request.CheckAddress(reply->header.address))
                {
                    request.Set(*reply);
                    if (request.isFulfilled())
                    {
                        // Fire Update Handler
                        //UpdateRegisters(request);
                        update_cb_(request);
                        std::cout << "Completed Request" << std::endl;
                    }
                }
            }
            requests_lock_.unlock();
        }
        inline std::vector<RequestReply>& GetRequests()
        {
            return request_queue_;
        }
        inline void SetTransport(CANDevice *dev)
        {
            transport_ = dev;
        }
        inline void RegisterCompleteCB(const std::function<void(RequestReply&)> &update_cb)
        {
            update_cb_ = update_cb;
        }

    private:
        CANDevice *transport_;
        std::vector<RequestReply> request_queue_;
        std::mutex requests_lock_;
        int32_t servo_id_;
        int32_t master_id_;

        std::function<void(RequestReply&)> update_cb_ = [=](RequestReply&) {};
        //std::function<void(RequestReply&)> update_cb_;
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

    //bool ReadRegister(uint32_t address, uint8_t *data);
    bool ReadRegisters(std::vector<Register> addresses, uint32_t timeout = 3000);
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
    Requester *requester_;
    std::vector<RequestReply> request_queue_;

    std::mutex requests_lock;

    std::mutex update_lock;

    
    std::vector<Register> register_map_;

    // Connect Status
    bool connected_;
};

#endif // NOMADBLDC_LIB_H_