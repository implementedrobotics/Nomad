// C System Files

// C++ System Files
#include <cstring>
#include <iostream>
#include <future>

// Project Includes
#include <NomadBLDC/NomadBLDC.h>
#include <CAN/Registers.h>

NomadBLDC::NomadBLDC(int master_id, int servo_id, CANDevice *transport) : master_id_(master_id), servo_id_(servo_id), transport_(transport), connected_(false)
{
    using namespace std::placeholders;
    if(transport_ != nullptr)
    {
        transport_->RegisterListenerCB(std::bind(&NomadBLDC::ReceiveMessage, this, _1));
    }
}

bool NomadBLDC::Connect()
{

    // Try to read device status
    register_reply_t reply;
    bool status = ReadRegister(DeviceRegisters_e::DeviceStatusRegister1, reply);

    if(!status)
        return false;

    memcpy(&dsr1_, reply.cmd_data, sizeof(DeviceStatusRegister1_t));

    // Copy Register Information
    status = ReadRegister(DeviceRegisters_e::DeviceStatusRegister2, reply);

    if(!status)
        return false;

    DeviceStatusRegister2_t *dsr = (DeviceStatusRegister2_t *)reply.cmd_data;
    memcpy(&dsr2_, reply.cmd_data, sizeof(DeviceStatusRegister2_t));

    connected_ = true;
    return true;
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

bool NomadBLDC::ReadRegister(uint32_t address, register_reply_t &register_reply)
{
    if(transport_ == nullptr)
        return false;

    register_command_t read_cmd;
    read_cmd.header.rwx = 0; // 0 Read, 1 Write, 2 Executure
    read_cmd.header.address = address ;
    read_cmd.header.data_type = 1;
    read_cmd.header.sender_id = master_id_;
    read_cmd.header.length = 0;

    CANDevice::CAN_msg_t msg;
    msg.id = servo_id_;
    msg.length = sizeof(request_header_t);
    memcpy(msg.data, &read_cmd, msg.length);

    transport_->Send(msg);

    // Wait for Reply
    std::future<register_reply_t> future_reply = promise_[read_cmd.header.address].get_future();

    auto status = future_reply.wait_for(std::chrono::milliseconds(100));
    if (status == std::future_status::ready)
    {
        register_reply = future_reply.get();
        return true;
    }
    else
    {
        return false;
    }

    return true;
}

void NomadBLDC::ReceiveMessage(CANDevice::CAN_msg_t &msg)
{
    // Check Sender ID is linked to this servo object
    register_reply_t *reply = (register_reply_t *)msg.data;

    if(reply->header.sender_id != servo_id_)
        return;
    
    // Update Promise Token
    promise_[reply->header.address].set_value(*reply);
}