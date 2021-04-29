#include <CAN/PCANDevice.h>
#include <CAN/Registers.h>
#include <CAN/RealTimeTask.hpp>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <cmath>

#include <NomadBLDC/NomadBLDC.h>
#include <nlohmann/json.hpp>
#include <zmq.hpp>

#define DEVICE "/dev/pcanusbfd32"

PCANDevice can;

class PlotJugglerTest : public Realtime::RealTimeTaskNode
{

public:
    // Block Diagram Class For Systems Task Node
    // name = Task Name
    // T_s = Sample Time (-1 for inherit)
    PlotJugglerTest(const std::string &name, const double T_s = -1);

    virtual void Exit();

protected:
    // Overriden Run Function
    virtual void Run();

    // Pre-Run Setup Routine.  Setup any one time initialization here.
    virtual void Setup();

    double time_;

    NomadBLDC *servo1;


    zmq::context_t ctx;
    zmq::socket_t *publisher;
};

PlotJugglerTest::PlotJugglerTest(const std::string &name, const double T_s) : Realtime::RealTimeTaskNode(name, T_s, Realtime::Priority::HIGH, -1, PTHREAD_STACK_MIN), time_(0.0)
{
}

void PlotJugglerTest::Run()
{
    auto start_time = std::chrono::high_resolution_clock::now();
    time_ += dt_actual_;

    float freq = 1.0f; // 2 hz

    float sin_val = std::sin(2 * M_PI * freq * time_);
    float cos_val = std::cos(2 * M_PI * freq * time_);
    
    //std::cout << "Time: " << time_ << " : " << sin_val << std::endl;
    //return;

    float pos = 1.0f * sin_val;
    servo1->ClosedLoopTorqueCommand(1.0f, 0.0f, pos, 0.0f, 0.0f);
    float pos1 = servo1->GetPosition();
    float vel1 = servo1->GetVelocity();

    nlohmann::json test = {
        {"timestamp", time_},
        {"motor", {{"pos", pos1}, {"pos_ref", pos},{"vel", vel1}}}};

    auto data = nlohmann::json::to_msgpack(test);
    zmq::message_t message(data.size());
    memcpy(message.data(), &data[0], data.size());
    publisher->send(message, zmq::send_flags::dontwait);

    auto time_now = std::chrono::high_resolution_clock::now();
    auto total_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
     //std::cout << "Duration: " << total_elapsed << "us" << std::endl;
}
void PlotJugglerTest::Setup()
{
    // TODO: We need some sort of exceptions here if setup fails
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

    servo1 = new NomadBLDC(1, 0x11, &can);
    servo1->SetName("INPUT");
    if(!servo1->Connect())
    {
        std::cout << "[ERROR]: Unable to connect to Nomad Servo!" << std::endl;
        exit(1);
    }

    std::cout << "Nomad Servo: " << "[" << servo1->GetName() << "] : " << servo1->GetServoId() << " Connected!" << std::endl;

    // Setup ZMQ
    publisher = new zmq::socket_t(ctx, ZMQ_PUB);
    std::string transport("tcp://*:9872");
    publisher->bind(transport);

    std::cout << "Please Zero Actuator.  Enter to Continue." << std::endl;
    getchar();

    servo1->ZeroOutput();
    
    // Start Motor Control Mode
    usleep(1000000);
    servo1->SetControlMode(PD_MODE);
}

void PlotJugglerTest::Exit()
{
    std::cout << "Exiting!" << std::endl;

    // Set back to idle.  In theory when no commands are sent it should auto back to idle or edamp?
    servo1->SetControlMode(IDLE_MODE);
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

    PlotJugglerTest pj_Node("Test", 1/500.0f); //500hz
    pj_Node.SetStackSize(1024 * 1024);
    pj_Node.SetTaskPriority(Realtime::Priority::HIGHEST);
    pj_Node.SetCoreAffinity(2);
    pj_Node.Start();

    char input;
    std::cin >> input;
    if(input == 'q')
    {
        std::cout << "Got Q:" << std::endl;
    }

    pj_Node.Exit();
    pj_Node.Stop();

    //getchar();
}
