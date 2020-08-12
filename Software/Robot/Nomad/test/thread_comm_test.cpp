#include <Realtime/RealTimeTask.hpp>
#include <Communications/Port.hpp>
#include <Common/Time.hpp>
#include <Nomad/NomadRobot.hpp>
#include <Nomad/Interface/StandController.hpp>
#include <TaskWrite.hpp>
#include <TaskRead.hpp>
#include <memory>

#include <unistd.h>
#include <sys/mman.h>

using Communications::Port;
using Communications::PortManager;
using Realtime::RealTimeTaskManager;
using Realtime::RealTimeTaskNode;

using Robot::Nomad::NomadRobot;
using Robot::Nomad::Interface::TaskWrite;
using Robot::Nomad::Interface::TaskRead;

int main(int argc, char *argv[])
{
    // Task Periods.
    // int freq1 = 1;
    int hi_freq = 5;
    // int freq2 = 100;

    // Create Manager Class Instance Singleton.
    // Must make sure this is done before any thread tries to access.
    // And thus tries to allocate memory inside the thread heap.
    RealTimeTaskManager::Instance();
    PortManager::Instance();

    if (!RealTimeTaskManager::EnableRTMemory(500 * 1024 * 1024)) // 500MB
    {
        // exit(-2);
        std::cout << "Error configuring Realtime Memory Requiremets!  Realtime Execution NOT guaranteed." << std::endl;
    }
    // // Plant Inputs
    // const std::string sim_url = "udpm://239.255.76.67:7667?ttl=0";

    // std::shared_ptr<Port> SIM_IMU = Port::CreateOutput("SIM_IMU", 10);
    // SIM_IMU->SetTransport(Port::TransportType::IPC, sim_url, "nomad.sim.imu_state");

    // std::shared_ptr<Port> JOINT_STATE = Port::CreateOutput("SIM_JOINT_STATE", 10);
    // JOINT_STATE->SetTransport(Port::TransportType::IPC, sim_url, "nomad.sim.joint_state");

    // std::shared_ptr<Port> COM_STATE = Port::CreateOutput("SIM_COM_STATE", 10);
    // COM_STATE->SetTransport(Port::TransportType::IPC, sim_url, "nomad.sim.com_state");

    // Write Task
    TaskWrite task_write("Task Write");
    task_write.SetStackSize(1024 * 1024); // 1MB
    task_write.SetTaskPriority(Realtime::Priority::MEDIUM);
    task_write.SetTaskFrequency(25); // 50 HZ
    task_write.SetCoreAffinity(-1);
    task_write.SetPortOutput(TaskWrite::JOINT_CONTROL_CMD_OUT,
                                             Communications::Port::TransportType::NATIVE, "native", "nomad.sim.joint_cmd");

    TaskRead task_read("Task Read");
    task_read.SetStackSize(1024 * 1024); // 1MB
    task_read.SetTaskPriority(Realtime::Priority::MEDIUM);
    task_read.SetTaskFrequency(25); // 50 HZ
    task_read.SetCoreAffinity(-1);
    // task_read.SetPortOutput(TaskWrite::JOINT_CONTROL_CMD_OUT,
    //                                          Communications::Port::TransportType::NATIVE, "native", "nomad.sim.joint_cmd");

    Port::Map(task_read.GetInputPort(TaskRead::InputPort::JOINT_CONTROL_CMD_IN),
               task_write.GetOutputPort(TaskRead::OutputPort::JOINT_CONTROL_CMD_OUT));


    task_read.Start();
    task_write.Start();
    

    // Print Threads
    RealTimeTaskManager::Instance()->PrintActiveTasks();

    // Start Inproc Context Process Thread
    PortManager::Instance()->GetInprocContext()->start();


    getchar();

}