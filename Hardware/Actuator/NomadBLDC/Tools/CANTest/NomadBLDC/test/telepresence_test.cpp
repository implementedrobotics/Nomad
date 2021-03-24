#include <CAN/PCANDevice.h>
#include <CAN/Registers.h>
#include <CAN/RealTimeTask.hpp>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <math.h>

#include <NomadBLDC/NomadBLDC.h>
#define DEVICE "/dev/pcanusbfd32"


PCANDevice can;

class TelepresenceTest : public Realtime::RealTimeTaskNode
{

public:
    // Block Diagram Class For Systems Task Node
    // name = Task Name
    // T_s = Sample Time (-1 for inherit)
    TelepresenceTest(const std::string &name, const double T_s = -1);

    virtual void Exit();

protected:
    // Overriden Run Function
    virtual void Run();

    // Pre-Run Setup Routine.  Setup any one time initialization here.
    virtual void Setup();

    double time_;

    NomadBLDC *servo1;
    NomadBLDC *servo2;

};

TelepresenceTest::TelepresenceTest(const std::string &name, const double T_s) : Realtime::RealTimeTaskNode(name, T_s, Realtime::Priority::HIGH, -1, PTHREAD_STACK_MIN), time_(0.0)
{
}

void TelepresenceTest::Run()
{
    return;
     auto start_time = std::chrono::high_resolution_clock::now();

    float tau1 = 0.0f;
    float tau2 = 0.0f;

    // tau1 = (kp*30*(pos2 - pos1) + kd*1*(vel2-vel1));
    // tau2 = (kp*30*(pos1 - pos2) + kd*1*(vel1-vel2));

    //std::cout << "TAU HERE: " << tau1 << " " << tau2 << std::endl;
    time_ += dt_actual_;

    servo1->ClosedLoopTorqueCommand(0.0f, 0.0f, 0.0f, 0.0f, tau1);
    float pos1 = servo1->GetPosition();
    float vel1 = servo1->GetVelocity();

    servo1->ClosedLoopTorqueCommand(0.0f, 0.0f, 0.0f, 0.0f, tau1);
    pos1 = servo1->GetPosition();
    vel1 = servo1->GetVelocity();

    // servo2.ClosedLoopTorqueCommand(0.0f, 0.0f, 0.0f, 0.0f, tau2);
    // float pos2 = servo2.GetPosition();
    // float vel2 = servo2.GetVelocity();

    // std::cout << "Tau 2: " << tau1 << " : " << tau2 <<  std::endl;

   // servo1->PrintState();
   // servo2.PrintState();

     auto time_now = std::chrono::high_resolution_clock::now();
     auto total_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
     std::cout << "Duration: " << total_elapsed << "us" << std::endl;

    //  std::cout << "Receive Message: " << reponse->header.address << " : " << pos1 <<  std::endl;
}
void TelepresenceTest::Setup()
{
    // Bind any active output ports
    std::cout << "Setup Run! " << std::endl;
    
    CANDevice::Config_t config;
    config.bitrate = 1e6; //1mbps
    config.d_bitrate = 5e6; //2mbps
    config.sample_point = .80; //87.5% 
    config.d_sample_point = 0.625; //60%
    config.clock_freq = 80e6; // 80mhz // Read from driver?  
    config.mode_fd = 1; // FD Mode

    if(!can.Open(DEVICE, config, false))
    {
        std::cout << "Unable to open CAN Device" << std::endl;
        return;
    }

    // Setup Filters
   // can.ClearFilters(); // Clear Existing/Reset.  Filters are saved on the device hardware.  Must make sure to clear
   // can.AddFilter(1, 12); // Only Listen to messages on id 1.  

    CANDevice::CAN_msg_t msg;


    register_command_t test;
    test.header.rwx = 0;
    test.header.address = DeviceRegisters_e::DeviceStatusRegister1;
    test.header.data_type = 1;
    test.header.sender_id = 0x001;
    test.header.length = 4;

    msg.id = 0x10;
    msg.length = sizeof(request_header_t);
    memcpy(msg.data, &test, msg.length);


    can.Send(msg);

//usleep(500);

can.Send(msg);

auto start_time = std::chrono::high_resolution_clock::now();
int i = 0;
    while (!can.Receive(msg))
    {
        if (i++ > 10000)
            break;

        std::cout << "WAITING10: " << std::endl;
    }

        auto time_now = std::chrono::high_resolution_clock::now();
    auto total_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    std::cout << "Duration2: " << total_elapsed << "us" << std::endl;



    start_time = std::chrono::high_resolution_clock::now();
    i = 0;
    while (!can.Receive(msg))
    {
        if (i++ > 10000)
            break;

        std::cout << "WAITING10: " << std::endl;
    }

    std::cout << msg.length << std::endl;
    time_now = std::chrono::high_resolution_clock::now();
    total_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    std::cout << "Duration2: " << total_elapsed << "us" << std::endl;

    return;
    servo1 = new NomadBLDC(1, 0x10, &can);
    servo1->SetName("INPUT");
    if(!servo1->Connect())
    {
        std::cout << "[ERROR]: Unable to connect to Nomad Servo!" << std::endl;
        return;
    }



     std::cout << "Nomad Servo: " << "[" << servo1->GetName() << "] : " << servo1->GetServoId() << " Connected!" << std::endl;
     return;
    // servo2 = NomadBLDC(1, 0x11, &can);
    // servo2.SetName("OUTPUT");
    // if(!servo2.Connect())
    // {
    //     std::cout << "[ERROR]: Unable to connect to Nomad Servo!" << std::endl;
    //     return;
    // }

  //  std::cout << "Nomad Servo: " << "[" << servo2.GetName() << "]" << servo2.GetServoId() << " Connected!" << std::endl;
}

void TelepresenceTest::Exit()
{
    return;
    std::cout << "Exiting!" << std::endl;

    // Set back to idle.  In theory when no commands are sent it should auto back to idle or edamp?
    servo1->SetControlMode(1);
   // servo2->SetControlMode(1);
}


int main(int argc, char *argv[])
{
    Realtime::RealTimeTaskManager::Instance();

    if (!Realtime::RealTimeTaskManager::EnableRTMemory(500 * 1024 * 1024)) // 500MB
    {
        std::cout << "Error configuring Realtime Memory Requiremets!  Realtime Execution NOT guaranteed." << std::endl;
    }
    else
    {
        std::cout << "Real Time Memory Enabled!" << std::endl;
    }

    TelepresenceTest telepresenceNode("Test", 1/100.0f); //500hz
    telepresenceNode.SetStackSize(1024 * 1024);
    telepresenceNode.SetTaskPriority(Realtime::Priority::HIGHEST);
    telepresenceNode.SetCoreAffinity(2);
    telepresenceNode.Start();

    getchar();

    telepresenceNode.Exit();
    telepresenceNode.Stop();

    getchar();

}
