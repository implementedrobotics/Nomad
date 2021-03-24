// C System Files
#include <unistd.h>

// C++ System Files
#include <cstring>
#include <iostream>
#include <future>

// Project Includes
#include <NomadBLDC/NomadBLDC.h>
#include <CAN/Registers.h>

NomadBLDC::NomadBLDC() :  master_id_(-1), servo_id_(-1), transport_(nullptr), connected_(false), control_mode_(0)
{
    memset(&joint_state_,0,sizeof(JointState_t));
}
NomadBLDC::NomadBLDC(int master_id, int servo_id, CANDevice *transport) : master_id_(master_id), servo_id_(servo_id), transport_(transport), connected_(false), control_mode_(0)
{
    using namespace std::placeholders;
    if(transport_ != nullptr)
    {
        transport_->RegisterListenerCB(std::bind(&NomadBLDC::ReceiveMessage, this, _1));
    }
    memset(&joint_state_,0,sizeof(JointState_t));
}

bool NomadBLDC::Connect()
{
    auto start_time = std::chrono::high_resolution_clock::now();

    // Try to read device status
    bool status = ReadRegister(DeviceRegisters_e::DeviceStatusRegister1, (uint8_t*)&dsr1_);

    if(!status)
        return false;

    // Read next device status
    status = ReadRegister(DeviceRegisters_e::DeviceStatusRegister2, (uint8_t*)&dsr2_);

    if(!status)
        return false;

    connected_ = true;
    return true;
}

bool NomadBLDC::SetControlMode(uint32_t control_mode)
{
    // TODO: State Checking Here? i.e. make sure you only transition from idle->other modes etc
    control_mode_ = control_mode;
    bool status = WriteRegister(ControllerStateRegisters_e::ControlMode, (uint8_t*)&control_mode_, sizeof(control_mode_));
    return true;
}

bool NomadBLDC::ClosedLoopTorqueCommand(float k_p, float k_d, float pos_ref, float vel_ref, float torque_ff)
{
    // TODO: Support a local maximum torque?  Or just on the motor controller side?
    tcmr_.K_p = k_p;
    tcmr_.K_d = k_d;
    tcmr_.Pos_ref = pos_ref;
    tcmr_.Vel_ref = vel_ref;
    tcmr_.T_ff = torque_ff;

    bool status = ExecuteRegister(ControllerCommandRegisters_e::ClosedLoopTorqueCommand, (uint8_t*)&tcmr_, sizeof(tcmr_), (uint8_t*)&joint_state_);
    return true;
}

// TODO: ReadRegisterAsync, WriteRegisterAsync, ExecuteRegisterAsync
bool NomadBLDC::ReadRegister(uint32_t address, uint8_t *data)
{
    if(transport_ == nullptr)
        return false;

    register_command_t read_cmd;
    read_cmd.header.rwx = 0; // 0 Read, 1 Write, 2 Executure
    read_cmd.header.address = address ;
    read_cmd.header.data_type = 1; // TODO: Everything is 32-bit for now...
    read_cmd.header.sender_id = master_id_;
    read_cmd.header.length = 0;

    CANDevice::CAN_msg_t msg;
    msg.id = servo_id_;
    msg.length = sizeof(request_header_t);
    memcpy(msg.data, &read_cmd, msg.length);

    transport_->Send(msg);

    // Create Request
    // TODO: CreateRequest(Address, timeout)
    requests_lock.lock();
    request_queue_.push_back({read_cmd.header.address, 2000});
    auto& request = request_queue_.back();
    requests_lock.unlock();

    auto start_time = std::chrono::high_resolution_clock::now();
    register_reply_t register_reply;
    bool status = request.Get(register_reply);

    auto time_now = std::chrono::high_resolution_clock::now();
    auto total_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    std::cout << "Duration: " << total_elapsed << "us" << std::endl;

    if (status)
    {
        memcpy(data, register_reply.cmd_data, register_reply.header.length);
        return true;
    }
    return false;

    // Wait for Reply
    // Reset Promise
    //promise_[read_cmd.header.address] = std::promise<register_reply_t>();
    //std::future<register_reply_t> future_reply = promise_[read_cmd.header.address].get_future();

    // auto status = future_reply.wait_for(std::chrono::milliseconds(100));
    // if (status == std::future_status::ready)
    // {
    //     // TODO: Should we keep up with data sizes?  For now just mem copy all 60 cmd data bytes
    //     register_reply_t register_reply = future_reply.get();
    //     memcpy(data, register_reply.cmd_data, register_reply.header.length);
    //     return true;
    // }
    //return false;
}

bool NomadBLDC::WriteRegister(uint32_t address, uint8_t *data, size_t size)
{
    if(transport_ == nullptr)
        return false;

    register_command_t write_cmd;
    write_cmd.header.rwx = 1; // 0 Read, 1 Write, 2 Executure
    write_cmd.header.address = address;
    write_cmd.header.data_type = 1; // TODO: Everything is 32-bit for now...
    write_cmd.header.sender_id = master_id_;
    write_cmd.header.length = size;

    memcpy(&write_cmd.cmd_data, data, size);
    CANDevice::CAN_msg_t msg;
    msg.id = servo_id_;
    msg.length = sizeof(request_header_t) + size;
    memcpy(msg.data, &write_cmd, msg.length);

    transport_->Send(msg);

    // TODO: Reply?

    return true;

}

bool NomadBLDC::ExecuteRegister(uint32_t address, uint8_t *parameter_data, size_t size, uint8_t *return_data)
{
    if(transport_ == nullptr)
        return false;

    register_command_t execute_cmd;
    execute_cmd.header.rwx = 2; // 0 Read, 1 Write, 2 Execute
    execute_cmd.header.address = address;
    execute_cmd.header.data_type = 1; // TODO: Everything is 32-bit for now...
    execute_cmd.header.sender_id = master_id_;
    execute_cmd.header.length = size;

    memcpy(&execute_cmd.cmd_data, parameter_data, size);

    CANDevice::CAN_msg_t msg;
    msg.id = servo_id_;
    msg.length = sizeof(request_header_t) + size;
    memcpy(msg.data, &execute_cmd, msg.length);

    transport_->Send(msg);

    // Create Request
    requests_lock.lock();
    request_queue_.push_back({execute_cmd.header.address, 2000});
    auto& request = request_queue_.back();
    requests_lock.unlock();

    auto start_time = std::chrono::high_resolution_clock::now();
    register_reply_t register_reply;
    bool status = request.Get(register_reply);

    auto time_now = std::chrono::high_resolution_clock::now();
    auto total_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    //std::cout << "Duration: " << total_elapsed << "us" << std::endl;

    if(status)
    {
        memcpy(return_data, register_reply.cmd_data, register_reply.header.length);
        return true;
    }
    return false;
    
    // Wait for Reply
    //std::future<register_reply_t> future_reply = promise_[execute_cmd.header.address].get_future();
    
    
    //future_reply = promise_[execute_cmd.header.address].get_future();
    // auto status = future_reply.wait_for(std::chrono::milliseconds(100));
    // if (status == std::future_status::ready)
    // {
    //     // TODO: Should we keep up with data sizes?  For now just mem copy all 60 cmd data bytes
    //     register_reply_t register_reply = future_reply.get();
    //     memcpy(return_data, register_reply.cmd_data, register_reply.header.length);
    //     return true;
    // }
    return false;
}

bool NomadBLDC::SetTransport(CANDevice *dev)
{
    transport_ = dev;
    if(transport_ == nullptr)
        return false;

    using namespace std::placeholders;
    transport_->RegisterListenerCB(std::bind(&NomadBLDC::ReceiveMessage, this, _1));
    return true;
}

void NomadBLDC::ReceiveMessage(CANDevice::CAN_msg_t &msg)
{
    // Check Sender ID is linked to this servo object
    register_reply_t *reply = (register_reply_t *)msg.data;

    if(reply->header.sender_id != servo_id_)
        return;
    
    requests_lock.lock();
    for (int i = request_queue_.size() - 1; i >= 0; i--)
    {
        if (request_queue_[i].GetAddress() == reply->header.address)
        {
            request_queue_[i].Set(*reply);
            request_queue_.erase(request_queue_.begin() + i);
        }
    }
    requests_lock.unlock();
    // TODO: This should be better linked with some sort of packet id?  I think we are safe for now
    // TODO: Use high level request object?
    // Update Promise Token
}

void NomadBLDC::PrintState()
{
    std::cout << std::endl << "-----------------------------------------" << std::endl;
    std::cout << "Servo: " << GetName() << " : ID: " << GetServoId() << std::endl;
    std::cout << "Pos: " << GetPosition() << " Vel: " << GetVelocity() << " Tau: " << GetTorque() << std::endl;;
    std::cout << "-----------------------------------------" << std::endl;
}