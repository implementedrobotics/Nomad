#include <CAN/CANDevice.h>
#include <CAN/Registers.h>
#include <CAN/RealTimeTask.hpp>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <math.h>



CANDevice can;

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
    time_ += dt_actual_;
    //std::cout << "DT: " << time_ << std::endl;
    //std::cout << "Run Diagram: " << task_name_ << std::endl;
    //TODO: Do CAN Things HEre...


    register_command_t test;
    test.header.rwx = 1;
    test.header.address = ControllerStateRegisters_e::TorqueControlModeRegister;
    test.header.data_type = 1;
    test.header.sender_id = 0x001;
    test.header.length = 4;

    TorqueControlModeRegister_t tcmr;

    tcmr.K_d = 0.05f;
    tcmr.K_p = 0.0f;
    tcmr.Pos_ref = 0.0f;
    tcmr.Vel_ref = 0.0f;
    tcmr.T_ff = 0.0f;

    memcpy(&test.cmd_data, (uint8_t *)&tcmr, sizeof(TorqueControlModeRegister_t));

    float pos = 0.0f;
    float freq = 1.0f;

    pos = 1.0 * sin(2 * M_PI * freq * time_);
    tcmr.Pos_ref = pos;
    memcpy(&test.cmd_data, (uint8_t *)&tcmr, sizeof(TorqueControlModeRegister_t));
    
    CANDevice::CAN_msg_t msg;
    msg.id = 0x10;
    msg.length = sizeof(request_header_t) + sizeof(TorqueControlModeRegister_t);
    memcpy(msg.data, &test, msg.length);

    can.Send(msg);

    int i = 0;
    while (!can.Receive(msg))
    {
        if (i++ > 10000)
            break;

        std::cout << "WAITING: " << std::endl;
    }

    register_reply_t *reponse = (register_reply_t *)msg.data;
    std::cout << "Receive Message: " << reponse->header.address << std::endl;
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

    uint32_t new_mode = 10;
    memcpy(&enable.cmd_data, &new_mode, sizeof(uint32_t));

    CANDevice::CAN_msg_t msg;
    msg.id = 0x10;

    msg.length = sizeof(request_header_t) + sizeof(uint32_t);
    memcpy(msg.data, &enable, msg.length);

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
    msg.id = 0x10;

    msg.length = sizeof(request_header_t) + sizeof(uint32_t);
    memcpy(msg.data, &enable, msg.length);

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

/*


int main()
{
    CANDevice can;
    if(!can.Open("can0", 1))
    {
        std::cout << "Unable to open CAN Device" << std::endl;
        return -1;
    }


    register_command_t test;
    test.header.rwx = 1;
    test.header.address = ControllerStateRegisters_e::TorqueControlModeRegister;
    test.header.data_type = 1;
    test.header.sender_id = 0x001;
    test.header.length = 4;

    TorqueControlModeRegister_t tcmr;

    tcmr.K_d = 0.05f;
    tcmr.K_p = 10.0f;
    tcmr.Pos_ref = 0.0f;
    tcmr.Vel_ref = 0.0f;
    tcmr.T_ff = 0.0f;

    
    memcpy(&enable.cmd_data, &new_mode, sizeof(uint32_t));
    memcpy(&test.cmd_data, (uint8_t *)&tcmr, sizeof(TorqueControlModeRegister_t));

   // memcpy(&test.cmd_data, (uint8_t *)&test_me, sizeof(Test_Struct));
   
    std::cout << "Command Size: " << sizeof(request_header_t) << std::endl;


usleep(1000000);

float pos = 0.0f;

for (int j = 0; j < 50000; j++)
{
    pos = sin(2*3.14*.0035*j);
    tcmr.Pos_ref = pos;
    memcpy(&test.cmd_data, (uint8_t *)&tcmr, sizeof(TorqueControlModeRegister_t));
    msg.length = sizeof(request_header_t) + sizeof(TorqueControlModeRegister_t);
    memcpy(msg.data, &test, msg.length);

    can.Send(msg);

    usleep(1000);
}

        // int i = 0;
        // while (!can.Receive(msg))
        // {
        //     if (i++ > 10000)
        //         break;

        //     std::cout << "WAITING: " << std::endl;
        // }

//        register_reply_t *reponse = (register_reply_t *)msg.data;
  //      std::cout << "Receive Message: " << *(uint16_t *)reponse->cmd_data << std::endl;

      //  usleep(10000);
  //  }
    std::cout << "OUT!" << std::endl;
    bool b_close = can.Close();
}

*/