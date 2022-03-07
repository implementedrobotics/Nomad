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
#include <NomadBLDC/Registers.h>
#include <CAN/CANDevice.h>



typedef enum : uint8_t
{
    STARTUP_MODE = 0,
    IDLE_MODE = 1,
    ERROR_MODE = 2,
    MEASURE_RESISTANCE_MODE = 3,
    MEASURE_INDUCTANCE_MODE = 4,
    MEASURE_PHASE_ORDER_MODE = 5,
    MEASURE_ENCODER_OFFSET_MODE = 6,
    CALIBRATION_MODE = 7,
    POSITION_MODE = 8,
    VELOCITY_MODE = 9,
    PD_MODE = 10,
    TORQUE_MODE = 11,
    CURRENT_MODE = 12,
    VOLTAGE_MODE = 13
} control_mode_type_t;

typedef enum
{
    SUCCESSFUL = 0,
    FOC_TIMING_ERROR = 1,
    OVERVOLTAGE_ERROR = 2,
    UNDERVOLTAGE_ERROR = 3,
    OVERTEMPERATURE_ERROR = 4,
    NOT_CALIBRATED_ERROR = 5,
    MEASUREMENT_OUT_OF_RANGE = 6,
    MEASUREMENT_TIMEOUT = 7,
    WATCHDOG_TIMEOUT = 8,
    OVERSPEED_ERROR = 9,
    POSITION_LIMIT_EXCEEDED = 10,
    TORQUE_LIMIT_EXCEEDED = 11,
    CURRENT_LIMIT_EXCEEDED = 12,
} error_type_t;


// TODO: Timeout Pass
// TODO: Not all request expect a reply
class RequestReply
{
public:

    RequestReply(std::vector<Register> &registers, uint32_t id, uint32_t timeout) :  id_(id), timeout_(timeout), num_replies_(0)
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
            //request_regs_.insert(request_regs_.end(), registers.begin(), registers.end());
            request_regs_.push_back(reg);
        }
        synced_future_ = synced_promise_.get_future();
    }
    
    inline bool CheckReply(uint32_t id, uint32_t address) 
    {
        //std::cout << "THIS REPLY ID: " << id << " to: " << id_ << std::endl;
        if(id != id_)
            return false;
        
        // Find Address Location
        ptrdiff_t pos = std::distance(request_regs_.begin(), std::find_if(request_regs_.begin(),
                                                                          request_regs_.end(),
                                                                          [&address](const Register f) -> bool { return f.address == address; }));
        
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
                register_reply_t reply = future.get();
                replies.push_back(reply);
            }
            else
            {
                // Timed out
                return false;
            }
        }
        return true;
    }

    inline bool Set(register_reply_t &reply)
    {
        uint32_t address = reply.header.address;
        // Find Address Location
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

    std::vector<Register> request_regs_;

    uint32_t id_; // Request ID
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
        // TODO: Switch from Address matching to UUID Packet ID?  It is a bit of a problem assuming register address will match in reply
        // This is the main reason for the below functions not being generic enough
        
        // TODO: Hacked this in for now.  I need a more generic request function to handle reads/writes/executes
        RequestReply &CreateExecuteRequest(std::vector<Register> &param_registers, std::vector<Register> &return_registers, uint16_t request_type, uint32_t timeout)
        {

            // Hold Request Messages
            std::vector<CANDevice::CAN_msg_t> request_msgs;

            // Loop requested address and cache our request messages
            for (Register& reg : param_registers)
            {
                register_command_t reg_cmd;
                reg_cmd.header.rwx = request_type; // 0 Read, 1 Write, 2 Execute
                reg_cmd.header.address = reg.address;
                reg_cmd.header.data_type = 1; // TODO: Everything is 32-bit for now...
                reg_cmd.header.sender_id = master_id_;
                reg_cmd.header.msg_id = id_counter_;

                memcpy(&reg_cmd.cmd_data, reg.data, reg.size);
                CANDevice::CAN_msg_t msg;
                msg.id = servo_id_;
                msg.length = sizeof(request_header_t) + reg.size;
                memcpy(msg.data.data(), &reg_cmd, msg.length);

                // Save it
                request_msgs.push_back(msg);

              //  std::cout << "Create Request EXEC REPLY: " << id_counter_ << " : " << reg.address << std::endl;
              //  std::cout << "Execute: " << reg.address << " " << request_type << "size: " << reg.size << std::endl;;
            }

            // TODO: Need error checking if we have multiply request?  Something breaks if multiple items in queue
            // Lock Request Queue
            requests_lock_.lock();
            // Generate Request
            // TODO: Only generate these for request expecting replies
            // TODO: Always push back.  Set a no reply.  Which will satisfy fullfilled and remove from queue next cycle
            request_queue_.push_back({return_registers, id_counter_, timeout});
            auto &request = request_queue_.back();
            requests_lock_.unlock();

            // Send Request Message
            for (CANDevice::CAN_msg_t &msg : request_msgs)
            {
                transport_->Send(msg);
            }

            // Increment Counter
            if (id_counter_++ > 16383)
                id_counter_ = 0;

            return request;
        }
        RequestReply &CreateRequest(std::vector<Register> &registers, uint16_t request_type, uint32_t timeout)
        {
            // Hold Request Messages
            std::vector<CANDevice::CAN_msg_t> request_msgs;

            // Loop requested address and cache our request messages
            for (Register& reg : registers)
            {
                register_command_t reg_cmd;
                reg_cmd.header.rwx = request_type; // 0 Read, 1 Write, 2 Execute
                reg_cmd.header.address = reg.address;
                reg_cmd.header.data_type = 1; // TODO: Everything is 32-bit for now...
                reg_cmd.header.sender_id = master_id_;
                reg_cmd.header.msg_id = id_counter_;

                // if(request_type == RequestType_e::Read)
                //     reg_cmd.header.length = 0;
                // else // Write or Execute, i.e. Sending Data
                //     reg_cmd.header.length = reg.size;

                memcpy(&reg_cmd.cmd_data, reg.data, reg.size);
                CANDevice::CAN_msg_t msg;
                msg.id = servo_id_;
                msg.length = sizeof(request_header_t) + reg.size;
                memcpy(msg.data.data(), &reg_cmd, msg.length);

                // Save it
                request_msgs.push_back(msg);

               // std::cout << "Create Request REPLY: " << id_counter_ << " : " << reg.address << std::endl;
            }

            // TODO: Need error checking if we have multiple request?  Something breaks if multiple items in queue
            // Lock Request Queue
            requests_lock_.lock();

            // Generate Request
            // TODO: Only generate these for request expecting replies
            //if(request_type != RequestType_e::Write)
                request_queue_.push_back({registers, id_counter_, timeout});
            
            auto &request = request_queue_.back();
            requests_lock_.unlock();

            // Send Request Message
            for (CANDevice::CAN_msg_t &msg : request_msgs)
            {
                transport_->Send(msg);
            }

            // Increment Counter
            if (id_counter_++ > 16383)
                id_counter_ = 0;

            return request;
        }

        void ProcessReply(register_reply_t *reply)
        {
            //std::cout << "PROCESS REPLY: " << request_queue_.size() << std::endl;
            std::lock_guard<std::mutex> lock(requests_lock_);
            for (int i = request_queue_.size() - 1; i >= 0; i--)
            {
                auto &request = request_queue_[i];
                
                // TODO: Check for Stale request.  Timed out, already fulfilled etc
                if (request.isFulfilled())
                {
                    request_queue_.erase(request_queue_.begin() + i);
                    continue;
                }
                //std::cout << "CHECK REPLY: " << reply->header.msg_id << " : " << reply->header.address << std::endl;
                if (request.CheckReply(reply->header.msg_id, reply->header.address))
                {
                   // std::cout << "SUCCESS!" << std::endl;
                    request.Set(*reply);
                    if (request.isFulfilled())
                    {
                        // Fire Update Handler
                        update_cb_(request);

                        //std::cout << "QUEUE SIZE PRE ERASE: " << request_queue_.size() << std::endl;
                        request_queue_.erase(request_queue_.begin() + i);
                        //std::cout << "QUEUE SIZE ERASE: " << request_queue_.size() << std::endl;
                    }
                }
            }
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
        uint32_t id_counter_ = 0; // Send message "unique" ID Counter
        std::function<void(RequestReply&)> update_cb_ = [=](RequestReply&) {};
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
    bool ZeroOutput();
    bool SetMaxTorque(float tau);
    float GetMaxTorque();

    bool SetMaxCurrent(float current);
    float GetMaxCurrent();

    bool SaveConfig();

    bool SetControlMode(uint32_t mode);
    void SetName(const std::string &name) { name_ = name; }
    const std::string& GetName() const { return name_; }

    // Servo State
    float GetPosition() { return joint_state_.Pos; }
    float GetVelocity() { return joint_state_.Vel; }
    float GetTorque() { return joint_state_.T_est; }

    uint32_t GetServoId() const { return servo_id_; }

    bool ReadRegisters(std::vector<Register> registers, uint32_t timeout = 3000);
    bool WriteRegisters(std::vector<Register> registers, uint32_t timeout = 3000);
    bool ExecuteRegisters(std::vector<Register> param_registers, std::vector<Register> return_registers = {}, uint32_t timeout = 3000);

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

    float torque_limit_;
    float current_limit_;

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