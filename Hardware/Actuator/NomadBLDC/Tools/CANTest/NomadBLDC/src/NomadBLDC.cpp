// C System Files
#include <unistd.h>

// C++ System Files
#include <cstring>
#include <iostream>
#include <future>

// Project Includes
#include <NomadBLDC/NomadBLDC.h>
#include <CAN/Registers.h>


auto start_time = std::chrono::high_resolution_clock::now();

NomadBLDC::NomadBLDC() :  master_id_(-1), servo_id_(-1), transport_(nullptr), connected_(false), control_mode_(0)
{
    memset(&joint_state_,0,sizeof(JointState_t));

    // Allocate Requester & Callback
    using namespace std::placeholders;
    requester_ = new Requester(transport_, servo_id_, master_id_);
    requester_->RegisterCompleteCB(std::bind(&NomadBLDC::UpdateRegisters, this, _1));
    
    // Setup Register Mappings
    SetupRegisterMap();
}
NomadBLDC::NomadBLDC(int master_id, int servo_id, CANDevice *transport) : master_id_(master_id), servo_id_(servo_id), transport_(transport), connected_(false), control_mode_(0)
{

    using namespace std::placeholders;
    // Allocate Requester & Callback
    requester_ = new Requester(transport_, servo_id_, master_id_);
    requester_->RegisterCompleteCB(std::bind(&NomadBLDC::UpdateRegisters, this, _1));


    if(transport_ != nullptr)
    {
        transport_->RegisterListenerCB(std::bind(&NomadBLDC::ReceiveMessage, this, _1));
    }
    memset(&joint_state_,0,sizeof(JointState_t));


    // Setup Register Mappings
    SetupRegisterMap();

}

void NomadBLDC::SetupRegisterMap()
{
    register_map_.reserve(kMaxRegisters);
    register_map_[DeviceRegisters_e::DeviceStatusRegister1] = (uint8_t *)&dsr1_;
    register_map_[DeviceRegisters_e::DeviceStatusRegister2] = (uint8_t *)&dsr2_;
}
bool NomadBLDC::Connect()
{
    // Try to read device status
    bool status = ReadRegisters({DeviceRegisters_e::DeviceStatusRegister1, DeviceRegisters_e::DeviceStatusRegister2});

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

bool NomadBLDC::ReadRegisters(std::vector<uint32_t> addresses)
{
    if(transport_ == nullptr)
        return false;

    // Create Request
    auto& request = requester_->CreateRequest(addresses, 0, 2000);

    // // TODO: Requester::CreateRequest(Address, timeout)
    // requests_lock.lock();
    // std::vector<CANDevice::CAN_msg_t> request_msgs;
    // for(uint32_t address : addresses)
    // {
    //     register_command_t read_cmd;
    //     read_cmd.header.rwx = 0; // 0 Read, 1 Write, 2 Executure
    //     read_cmd.header.address = address;
    //     read_cmd.header.data_type = 1; // TODO: Everything is 32-bit for now...
    //     read_cmd.header.sender_id = master_id_;
    //     read_cmd.header.length = 0;

    //     CANDevice::CAN_msg_t msg;
    //     msg.id = servo_id_;
    //     msg.length = sizeof(request_header_t);
    //     memcpy(msg.data, &read_cmd, msg.length);
    //     request_msgs.push_back(msg);
    // }
    // request_queue_.push_back({addresses, 2000});
    // auto& request = request_queue_.back();
    // requests_lock.unlock();
    // 

    // Requester::SendRequest()
    //auto sys_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    //std::cout << "Send Time1: " << sys_time << "us" << std::endl;
    // for(CANDevice::CAN_msg_t &msg : request_msgs)
    // {
    //     transport_->Send(msg);
    // }

    auto start_time = std::chrono::high_resolution_clock::now();

    // TODO: If synced we wait... Pass Function Param for Async vs Not?
    bool status = request.Wait(5000);

    auto total_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    std::cout << "Duration: " << total_elapsed << "us" << std::endl;

    if (status) { return true; }

    return false;
}
// TODO: ReadRegisterAsync, WriteRegisterAsync, ExecuteRegisterAsync
bool NomadBLDC::ReadRegister(uint32_t address, uint8_t *data)
{
    if(transport_ == nullptr)
        return false;

    register_command_t read_cmd;
    read_cmd.header.rwx = 0; // 0 Read, 1 Write, 2 Executure
    read_cmd.header.address = address;
    read_cmd.header.data_type = 1; // TODO: Everything is 32-bit for now...
    read_cmd.header.sender_id = master_id_;
    read_cmd.header.length = 0;

    CANDevice::CAN_msg_t msg;
    msg.id = servo_id_;
    msg.length = sizeof(request_header_t);
    memcpy(msg.data, &read_cmd, msg.length);

    // Create Request
    // TODO: CreateRequest(Address, timeout)
    requests_lock.lock();
    request_queue_.push_back({address, 2000});
    auto& request = request_queue_.back();
    requests_lock.unlock();

    auto sys_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    std::cout << "Send Time1: " << sys_time << "us" << std::endl;
    transport_->Send(msg);

    // register_command_t read_cmd2;
    // read_cmd2.header.rwx = 0; // 0 Read, 1 Write, 2 Executure
    // read_cmd2.header.address = DeviceRegisters_e::DeviceStatusRegister2 ;
    // read_cmd2.header.data_type = 1; // TODO: Everything is 32-bit for now...
    // read_cmd2.header.sender_id = master_id_;
    // read_cmd2.header.length = 0;

    // msg.id = servo_id_;
    // msg.length = sizeof(request_header_t);
    // memcpy(msg.data, &read_cmd2, msg.length);


    // transport_->Send(msg);
    //sys_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    //std::cout << "Send Time2: " << sys_time << "us" << std::endl;

    auto start_time = std::chrono::high_resolution_clock::now();

    // TODO: If synced we wait... Pass Function Param for Async vs Not?
    bool status = request.Wait(5000);

    auto total_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    std::cout << "Duration: " << total_elapsed << "us" << std::endl;

    if (status)
    {
        return true;
    }
    return false;
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
    std::vector<register_reply_t> replies;
    bool status = request.Get(replies);

    auto time_now = std::chrono::high_resolution_clock::now();
    auto total_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    //std::cout << "Duration: " << total_elapsed << "us" << std::endl;

    if(status)
    {
        // TODO: Loop and Copy Data.  Actually need this to be a callback for Update Registers
        //memcpy(return_data, register_reply.cmd_data, register_reply.header.length);
        return true;
    }
    return false;
}

void NomadBLDC::UpdateRegisters(RequestReply &request)
{
    std::vector<register_reply_t> replies;
    bool status = request.Get(replies);

    // TODO: Change this to lock guard etc
    update_lock.lock();
    for(auto& reply : replies)
    {
        memcpy(register_map_[reply.header.address], reply.cmd_data, reply.header.length);
    }

    request.Sync();
    update_lock.unlock();
}

bool NomadBLDC::SetTransport(CANDevice *dev)
{
    transport_ = dev;
    
    requester_->SetTransport(transport_);

    if(transport_ == nullptr)
        return false;

    using namespace std::placeholders;
    transport_->RegisterListenerCB(std::bind(&NomadBLDC::ReceiveMessage, this, _1));
    return true;
}

void NomadBLDC::ReceiveMessage(CANDevice::CAN_msg_t &msg)
{
    // TODO: This should be better linked with some sort of packet id?  I think we are safe for now
    // Check Sender ID is linked to this servo object
    register_reply_t *reply = (register_reply_t *)msg.data;

    if(reply->header.sender_id != servo_id_)
        return;
    
    //auto total_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    //std::cout << "Receive Time: " << total_elapsed << "us" << std::endl;
    
    //std::vector<RequestReply> &request_queue = requester_->GetRequests();
    requester_->ProcessReply(reply);
   
}

void NomadBLDC::PrintState()
{
    std::cout << std::endl << "-----------------------------------------" << std::endl;
    std::cout << "Servo: " << GetName() << " : ID: " << GetServoId() << std::endl;
    std::cout << "Pos: " << GetPosition() << " Vel: " << GetVelocity() << " Tau: " << GetTorque() << std::endl;;
    std::cout << "-----------------------------------------" << std::endl;
}