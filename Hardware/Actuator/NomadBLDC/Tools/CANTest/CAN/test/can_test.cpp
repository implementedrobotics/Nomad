#include <CAN/CANDevice.h>
#include <CAN/Registers.h>
#include <CAN/RealTimeTask.hpp>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <math.h>



CANDevice can;
uint32_t can_tx_id = 0x123;// 0x10;

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
    auto start_time = std::chrono::high_resolution_clock::now();


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

    tcmr.K_d = 0.05f;
    tcmr.K_p = 0.0f;
    tcmr.Pos_ref = 2.0f;
    tcmr.Vel_ref = 0.0f;
    tcmr.T_ff = 0.0f;

    //memcpy(&test.cmd_data, (uint8_t *)&tcmr, sizeof(TorqueControlModeRegister_t));

    float pos = 0.0f;
    float freq = 3.0f;

    pos = 2.0 * sin(2 * M_PI * freq * time_);
    tcmr.T_ff = pos;
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

     //   std::cout << "WAITING2: " << std::endl;
    }

    msg.id = 0x10;
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


    register_reply_t *reponse = (register_reply_t *)msg.data;

    JointState_t *joint_state = (JointState_t *)reponse->cmd_data;

    auto time_now = std::chrono::high_resolution_clock::now();
    auto total_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
   // std::cout << "Duration: " << total_elapsed << "us" << std::endl;

   // std::cout << "Receive Message: " << reponse->header.address << " : " << joint_state->Pos <<  std::endl;
}
void CANTestNode::Setup()
{
    // Bind any active output ports
    std::cout << "Setup Run! " << std::endl;

    if(!can.Open("can0", 1))
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
    msg.id = 0x10;
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

    msg.id = 0x10;
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
