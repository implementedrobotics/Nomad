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

static int i = 0;
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

    // servo1->ClosedLoopTorqueCommand(0.0f, 0.0f, 0.0f, 0.0f, tau1);
    // pos1 = servo1->GetPosition();
    // vel1 = servo1->GetVelocity();

    // servo2.ClosedLoopTorqueCommand(0.0f, 0.0f, 0.0f, 0.0f, tau2);
    // float pos2 = servo2.GetPosition();
    // float vel2 = servo2.GetVelocity();

    std::cout << "Tau 2: " << pos1 << " : " << vel1 <<  std::endl;

   // servo1->PrintState();
   // servo2.PrintState();

     auto time_now = std::chrono::high_resolution_clock::now();
     auto total_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    // std::cout << "Duration: " << total_elapsed << "us" << std::endl;

    if(i++ % 10000 == 0)
        can.Status();
    //  std::cout << "Receive Message: " << reponse->header.address << " : " << pos1 <<  std::endl;
}
void TelepresenceTest::Setup()
{
    CANDevice::Config_t config;
    config.bitrate = 1e6; //1mbps
    config.d_bitrate = 5e6; //2mbps
    config.sample_point = 0.80f; //87.5% 
    config.d_sample_point = 0.625f; //60%
    config.clock_freq = 80e6; // 80mhz // Read from driver?  
    config.mode_fd = 1; // FD Mode

    if(!can.Open(DEVICE, config, true))
    {
        std::cout << "Unable to open CAN Device" << std::endl;
        return;
    }

    // Setup Filters
    can.ClearFilters(); // Clear Existing/Reset.  Filters are saved on the device hardware.  Must make sure to clear
    can.AddFilter(1, 2); // Only Listen to messages on id 1.  

    servo1 = new NomadBLDC(1, 0x10, &can);
    servo1->SetName("INPUT");
    if(!servo1->Connect())
    {
        std::cout << "[ERROR]: Unable to connect to Nomad Servo!" << std::endl;
        return;
    }

     std::cout << "Nomad Servo: " << "[" << servo1->GetName() << "] : " << servo1->GetServoId() << " Connected!" << std::endl;
    // return;
    // servo2 = NomadBLDC(1, 0x11, &can);
    // servo2.SetName("OUTPUT");
    // if(!servo2.Connect())
    // {
    //     std::cout << "[ERROR]: Unable to connect to Nomad Servo!" << std::endl;
    //     return;
    // }

  //  std::cout << "Nomad Servo: " << "[" << servo2.GetName() << "]" << servo2.GetServoId() << " Connected!" << std::endl;

    // Start Motor Control Mode
    usleep(1000000);
    servo1->SetControlMode(10);
}

void TelepresenceTest::Exit()
{
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

    TelepresenceTest telepresenceNode("Test", 1/200.0f); //500hz
    telepresenceNode.SetStackSize(1024 * 1024);
    telepresenceNode.SetTaskPriority(Realtime::Priority::HIGHEST);
    telepresenceNode.SetCoreAffinity(2);
    telepresenceNode.Start();

    getchar();

    telepresenceNode.Exit();
    telepresenceNode.Stop();

    getchar();

}
