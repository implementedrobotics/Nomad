#include <CAN/SocketCANDevice.h>
#include <CAN/Registers.h>
#include <CAN/RealTimeTask.hpp>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <math.h>



SocketCANDevice can;
uint32_t can_tx_id = 0x10;
uint32_t can_tx_id2 = 0x110;

float pos1 = 0.0f;
float pos2 = 0.0f;
float vel1 = 0.0f;
float vel2 = 0.0f;

float kp = .050f;
float kd = .010f;
float tau1 = 0.0f;
float tau2 = 0.0f;

class CANTestNode : public Realtime::RealTimeTaskNode
{

public:
    // Block Diagram Class For Systems Task Node
    // name = Task Name
    // T_s = Sample Time (-1 for inherit)
    CANTestNode(const std::string &name, const double T_s = -1);

    virtual void Exit();

protected:
    // Overriden Run Function
    virtual void Run();

    // Pre-Run Setup Routine.  Setup any one time initialization here.
    virtual void Setup();

    double time_;


};

CANTestNode::CANTestNode(const std::string &name, const double T_s) : Realtime::RealTimeTaskNode(name, T_s, Realtime::Priority::MEDIUM, -1, PTHREAD_STACK_MIN), time_(0.0)
{
}

void CANTestNode::Run()
{
    return;
   // auto start_time = std::chrono::high_resolution_clock::now();

    tau1 = (kp*150*(pos2 - pos1) + kd*15*(vel2-vel1))*.05;
    tau2 = (kp*150*(pos1 - pos2) + kd*15*(vel1-vel2));

    //tau1 = 0;
    //tau2 = 0;
    time_ += dt_actual_;

    //std::cout << "DT: " << time_ << std::endl;
    //std::cout << "Run Diagram: " << task_name_ << std::endl;
    //TODO: Do CAN Things HEre...

    register_command_t test;
    test.header.rwx = 2;
    test.header.address = ControllerCommandRegisters_e::ClosedLoopTorqueCommand;
    test.header.data_type = 1;
    test.header.sender_id = 0x001;
    test.header.length = 4;

    TorqueControlModeRegister_t tcmr;

    tcmr.K_d = 0.0f;
    tcmr.K_p = 0.0f;
    tcmr.Pos_ref = 0.0f;
    tcmr.Vel_ref = 0.0f;
    tcmr.T_ff = 0.0f;

    //float pos = 0.0f;
    //float freq = 3.0f;

    //pos = 2.0 * sin(2 * M_PI * freq * time_);
    tcmr.T_ff = tau1;
    memcpy(&test.cmd_data, (uint8_t *)&tcmr, sizeof(TorqueControlModeRegister_t));
    
    CANDevice::CAN_msg_t msg;
    msg.id = can_tx_id;
    msg.length = sizeof(request_header_t) + sizeof(TorqueControlModeRegister_t);
    memcpy(msg.data, &test, msg.length);

    can.Send(msg);

    int i = 0;
    while (!can.Receive(msg))
    {
        if (i++ > 10000)
            break;

       // std::cout << "WAITING1: " << std::endl;
    }

    register_reply_t *reponse = (register_reply_t *)msg.data;

    JointState_t *joint_state = (JointState_t *)reponse->cmd_data;
    pos1 = joint_state->Pos;
    vel1 = joint_state->Vel;

    tcmr.T_ff = tau2;
    memcpy(&test.cmd_data, (uint8_t *)&tcmr, sizeof(TorqueControlModeRegister_t));

    msg.id = can_tx_id2;
    msg.length = sizeof(request_header_t) + sizeof(TorqueControlModeRegister_t);
    memcpy(msg.data, &test, msg.length);
    can.Send(msg);

    i = 0;
    while (!can.Receive(msg))
    {
        if (i++ > 10000)
            break;

       //  std::cout << "WAITING2: " << std::endl;
    }

    reponse = (register_reply_t *)msg.data;

    joint_state = (JointState_t *)reponse->cmd_data;
    pos2 = joint_state->Pos;
    vel2 = joint_state->Vel;


   std::cout << "Tau 2: " << tau2 << " : " << tau1 <<  std::endl;

  //  auto time_now = std::chrono::high_resolution_clock::now();
  // auto total_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
   //std::cout << "Duration: " << total_elapsed << "us" << std::endl;

  //  std::cout << "Receive Message: " << reponse->header.address << " : " << pos1 <<  std::endl;
}
void CANTestNode::Setup()
{
    // Bind any active output ports
    std::cout << "Setup Run! " << std::endl;

    CANDevice::Config_t config;
    config.bitrate = 1e6; //1mbps
    config.d_bitrate = 2e6; //2mbps
    config.sample_point = .875; //87.5% 
    config.d_sample_point = 0.6; //60%
    config.clock_freq = 80e6; // 80mhz // Read from driver?  
    config.mode_fd = 1; // FD Mode


    if(!can.Open("can0", config))
    {
        std::cout << "Unable to open CAN Device" << std::endl;
        return;
    }

    std::cout << "Enabling BLDC." << std::endl;

    register_command_t enable;
    enable.header.rwx = 1;
    enable.header.address = ControllerStateRegisters_e::ControlMode;
    enable.header.data_type = 1;
    enable.header.sender_id = 0x001;
    enable.header.length = 4;


    // register_command_t enable;
    // enable.header.rwx = 2;
    // enable.header.address = MotorConfigRegisters_e::ZeroOutputOffset;
    // enable.header.data_type = 1;
    // enable.header.sender_id = 0x001;
    // enable.header.length = 4;

    uint32_t new_mode = 10;
    memcpy(&enable.cmd_data, &new_mode, sizeof(uint32_t));

    CANDevice::CAN_msg_t msg;
    msg.id = can_tx_id;

    msg.length = sizeof(request_header_t) + sizeof(uint32_t);
    memcpy(msg.data, &enable, msg.length);

    can.Send(msg);

    usleep(1000000);

    msg.id = can_tx_id2;
    can.Send(msg);
    usleep(1000000);
}

void CANTestNode::Exit()
{
    std::cout << "Exiting!" << std::endl;

    register_command_t enable;
    enable.header.rwx = 1;
    enable.header.address = ControllerStateRegisters_e::ControlMode;
    enable.header.data_type = 1;
    enable.header.sender_id = 0x001;
    enable.header.length = 4;

    uint32_t new_mode = 1;
    memcpy(&enable.cmd_data, &new_mode, sizeof(uint32_t));

    CANDevice::CAN_msg_t msg;
    msg.id = can_tx_id;

    msg.length = sizeof(request_header_t) + sizeof(uint32_t);
    memcpy(msg.data, &enable, msg.length);

    can.Send(msg);

    msg.id = can_tx_id2;
    can.Send(msg);

}

int main(int argc, char *argv[])
{
    Realtime::RealTimeTaskManager::Instance();

    if (!Realtime::RealTimeTaskManager::EnableRTMemory(500 * 1024 * 1024)) // 500MB
    {
        // exit(-2);
        std::cout << "Error configuring Realtime Memory Requiremets!  Realtime Execution NOT guaranteed." << std::endl;
    }
    else
    {
        std::cout << "Real Time Memory Enabled!" << std::endl;
    }

    CANTestNode nomad_can("Test", 2e-3); //10hz
    nomad_can.SetStackSize(1024 * 1024);
    nomad_can.SetTaskPriority(Realtime::Priority::HIGHEST);
    nomad_can.SetCoreAffinity(2);

    nomad_can.Start();

    getchar();

    nomad_can.Exit();
    nomad_can.Stop();

    getchar();
}
