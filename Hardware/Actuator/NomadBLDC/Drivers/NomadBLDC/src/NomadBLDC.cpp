// C System Files
#include <unistd.h>

// C++ System Files
#include <future>

// Project Includes
#include <NomadBLDC/NomadBLDC.h>
#include <NomadBLDC/Registers.h>


auto start_time = std::chrono::high_resolution_clock::now();

NomadBLDC::NomadBLDC() :  master_id_(-1), servo_id_(-1), transport_(nullptr), connected_(false), control_mode_(0), torque_limit_(0.0f), current_limit_(11.0f)
{
    memset(&joint_state_,0,sizeof(JointState_t));

    // Allocate Requester & Callback
    using namespace std::placeholders;
    requester_ = new Requester(transport_, servo_id_, master_id_);
    requester_->RegisterCompleteCB(std::bind(&NomadBLDC::UpdateRegisters, this, _1));

    // Setup Register Mappings
    SetupRegisterMap();
}
NomadBLDC::NomadBLDC(int master_id, int servo_id, CANDevice *transport) : master_id_(master_id), servo_id_(servo_id), transport_(transport), connected_(false), control_mode_(0), torque_limit_(0.0f), current_limit_(11.0f)
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

    register_map_[DeviceRegisters_e::DeviceStatusRegister1] = {DeviceRegisters_e::DeviceStatusRegister1, sizeof(dsr1_), (uint8_t *)&dsr1_};
    register_map_[DeviceRegisters_e::DeviceStatusRegister2] = {DeviceRegisters_e::DeviceStatusRegister2, sizeof(dsr2_), (uint8_t *)&dsr2_};

    register_map_[ControllerCommandRegisters_e::ClosedLoopTorqueCommand] = {ControllerCommandRegisters_e::ClosedLoopTorqueCommand, sizeof(tcmr_), (uint8_t *)&tcmr_};
    register_map_[ControllerCommandRegisters_e::JointStateRegister] = {ControllerCommandRegisters_e::JointStateRegister, sizeof(joint_state_), (uint8_t *)&joint_state_};

    
    register_map_[ControllerStateRegisters_e::ControlMode] = {ControllerStateRegisters_e::ControlMode, sizeof(control_mode_), (uint8_t *)&control_mode_};

    register_map_[ControllerConfigRegisters_e::CurrentLimit] = {ControllerConfigRegisters_e::CurrentLimit, sizeof(current_limit_), (uint8_t *)&current_limit_};
    register_map_[ControllerConfigRegisters_e::TorqueLimit] = {ControllerConfigRegisters_e::TorqueLimit, sizeof(torque_limit_), (uint8_t *)&torque_limit_};


    register_map_[MotorConfigRegisters_e::ZeroOutputOffset] = {MotorConfigRegisters_e::ZeroOutputOffset, 0, nullptr};
    register_map_[DeviceRegisters_e::DeviceSaveConfig] = {DeviceRegisters_e::DeviceSaveConfig, 0, nullptr};
}

bool NomadBLDC::Connect()
{
    // Try to read device status
    bool status = ReadRegisters({register_map_[DeviceRegisters_e::DeviceStatusRegister1],
                                 register_map_[DeviceRegisters_e::DeviceStatusRegister2]}, 10000);

    if (!status) // Failed
        return false;

    connected_ = true; // We have a valid device connection
    return true;
}

bool NomadBLDC::Disconnect()
{
    connected_ = false; // We have a valid device connection

    if(transport_ == nullptr)
        return false;

    return transport_->Close();

}

bool NomadBLDC::SetMaxTorque(float tau_max)
{
    torque_limit_ = tau_max;
    bool status = WriteRegisters({register_map_[ControllerConfigRegisters_e::TorqueLimit]});
    return status;
}

float NomadBLDC::GetMaxTorque()
{
    bool status = ReadRegisters({register_map_[ControllerConfigRegisters_e::TorqueLimit]}, 5000);

    if (!status) // Failed
        return -1.0f;
    
    return torque_limit_;
}

bool NomadBLDC::SetMaxCurrent(float current)
{
    current_limit_ = current;
    bool status = WriteRegisters({register_map_[ControllerConfigRegisters_e::CurrentLimit]});
    return status;
}

float NomadBLDC::GetMaxCurrent()
{
    bool status = ReadRegisters({register_map_[ControllerConfigRegisters_e::CurrentLimit]}, 5000);

    if (!status) // Failed
        return -1.0f;
    
    return current_limit_;
}

bool NomadBLDC::ZeroOutput()
{
    bool status = ExecuteRegisters({register_map_[MotorConfigRegisters_e::ZeroOutputOffset]}, {}, 5000);
    return status;
}

bool NomadBLDC::SaveConfig()
{
    bool status = ExecuteRegisters({register_map_[DeviceRegisters_e::DeviceSaveConfig]}, {}, -1);
    return status;
}


bool NomadBLDC::SetControlMode(uint32_t control_mode)
{
    // TODO: State Checking Here? i.e. make sure you only transition from idle->other modes etc
    control_mode_ = control_mode;
    bool status = WriteRegisters({register_map_[ControllerStateRegisters_e::ControlMode]});
    return status;
}


bool NomadBLDC::ClosedLoopTorqueCommand(float k_p, float k_d, float pos_ref, float vel_ref, float torque_ff)
{
    // TODO: Support a local maximum torque?  Or just on the motor controller side?
    tcmr_.K_p = k_p;
    tcmr_.K_d = k_d;
    tcmr_.Pos_ref = pos_ref;
    tcmr_.Vel_ref = vel_ref;
    tcmr_.T_ff = torque_ff;

    bool status = ExecuteRegisters({register_map_[ControllerCommandRegisters_e::ClosedLoopTorqueCommand]},
                                   {register_map_[ControllerCommandRegisters_e::JointStateRegister]}, 5000);

    return true;
}

// TODO: ReadRegisterAsync, WriteRegisterAsync, ExecuteRegisterAsync
// Or bool "DoASync etc"
bool NomadBLDC::ReadRegisters(std::vector<Register> registers, uint32_t timeout)
{
    if(transport_ == nullptr)
        return false;

    // Create Request
    auto& request = requester_->CreateRequest(registers, RequestType_e::Read, timeout);

    auto start_time = std::chrono::high_resolution_clock::now();
    // TODO: If synced we wait... Pass Function Param for Async vs Not?
    bool status = request.Wait(timeout);
    auto total_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
   // std::cout << "Duration: " << total_elapsed << "us" << std::endl;

   // std::cout << "Status: " << status << std::endl;

    if (status) { return true; }

    return false;
}

bool NomadBLDC::WriteRegisters(std::vector<Register> registers, uint32_t timeout)
{
    if(transport_ == nullptr)
        return false;

    // TODO: Pass in if we need an ack/reply?
    // Create Request
    //auto& request = 
    requester_->CreateRequest(registers, RequestType_e::Write, timeout);

    // TODO: Sync/Reply?
    //request.Wait();

    return true;
}

bool NomadBLDC::ExecuteRegisters(std::vector<Register> param_registers, std::vector<Register> return_registers, uint32_t timeout)
{
    if(transport_ == nullptr)
        return false;

    // TODO: Pass in if we need an ack/reply?
    
    // Create Request
    auto& request = requester_->CreateExecuteRequest(param_registers, return_registers, RequestType_e::Execute, timeout);

    // TODO: Sync/Reply?
    //bool status = request.Wait(5000);
    return true;
}

void NomadBLDC::UpdateRegisters(RequestReply &request)
{
    std::vector<register_reply_t> replies;
    bool status = request.Get(replies);

    std::lock_guard<std::mutex> lock(update_lock); // Lock for updates.
    for(auto& reply : replies)
    {
        // Copy Data
        Register reg = register_map_[reply.header.address];
        memcpy(reg.data, reply.cmd_data, reg.size);
    }

    // Update any waiters that we are now synced
    request.Sync();
}

bool NomadBLDC::SetTransport(CANDevice *dev)
{
    // Update Transport Pointer
    transport_ = dev;
    
    // Pass to requester interface
    requester_->SetTransport(dev);

    // If null then return
    if(transport_ == nullptr)
        return false;

    // Update our listener callback to lower level can device
    using namespace std::placeholders;
    transport_->RegisterListenerCB(std::bind(&NomadBLDC::ReceiveMessage, this, _1));

    // Success
    return true;
}

void NomadBLDC::ReceiveMessage(CANDevice::CAN_msg_t &msg)
{
    // TODO: This should be better linked with some sort of packet id?  I think we are safe for now
    // Check Sender ID is linked to this servo object
    register_reply_t *reply = (register_reply_t *)msg.data.data();

    //std::cout << "Receiving Message!" << std::endl;
    if(reply->header.sender_id != servo_id_)
        return;
    
    // Send to requester
    requester_->ProcessReply(reply);
   
}

void NomadBLDC::PrintState()
{
    std::cout << std::endl << "-----------------------------------------" << std::endl;
    std::cout << "Servo: " << GetName() << " : ID: " << GetServoId() << std::endl;
    std::cout << "Pos: " << GetPosition() << " Vel: " << GetVelocity() << " Tau: " << GetTorque() << std::endl;;
    std::cout << "-----------------------------------------" << std::endl;
}