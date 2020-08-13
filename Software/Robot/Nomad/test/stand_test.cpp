#include <Realtime/RealTimeTask.hpp>
#include <Communications/Port.hpp>
#include <Common/Time.hpp>
#include <Nomad/NomadRobot.hpp>
#include <Nomad/Interface/StandController.hpp>
#include <memory>

#include <unistd.h>
#include <sys/mman.h>

using Communications::Port;
using Communications::PortManager;
using Realtime::RealTimeTaskManager;
using Realtime::RealTimeTaskNode;

using Robot::Nomad::NomadRobot;
using Robot::Nomad::Interface::StandController;

int main(int argc, char *argv[])
{

    // Task Periods.
    int freq1 = 1;
    int hi_freq = 1000;
    // int freq2 = 100;

    // Create Manager Class Instance Singleton.
    // Must make sure this is done before any thread tries to access.
    // And thus tries to allocate memory inside the thread heap.
    RealTimeTaskManager::Instance();
    PortManager::Instance();

    if (!RealTimeTaskManager::EnableRTMemory(500 * 1024 * 1024)) // 1000MB
    {
        // exit(-2);
        std::cout << "Error configuring Realtime Memory Requiremets!  Realtime Execution NOT guaranteed." << std::endl;
    }
    // Plant Inputs
    const std::string sim_url = "udpm://239.255.76.67:7667?ttl=0";

    std::shared_ptr<Port> SIM_IMU = Port::CreateOutput("SIM_IMU", 10);
    SIM_IMU->SetTransport(Port::TransportType::UDP, sim_url, "nomad.sim.imu_state");

    std::shared_ptr<Port> JOINT_STATE = Port::CreateOutput("SIM_JOINT_STATE", 10);
    JOINT_STATE->SetTransport(Port::TransportType::UDP, sim_url, "nomad.sim.joint_state");

    std::shared_ptr<Port> COM_STATE = Port::CreateOutput("SIM_COM_STATE", 10);
    COM_STATE->SetTransport(Port::TransportType::UDP, sim_url, "nomad.sim.com_state");

    // Simulator Interface Task Setup
    StandController nomad_simulation_interface("Simulation Interface");
    nomad_simulation_interface.SetStackSize(1024 * 1024); // 1MB
    nomad_simulation_interface.SetTaskPriority(Realtime::Priority::MEDIUM);
    nomad_simulation_interface.SetTaskFrequency(hi_freq); // 50 HZ
    nomad_simulation_interface.SetCoreAffinity(-1);
    nomad_simulation_interface.SetPortOutput(StandController::JOINT_CONTROL_CMD_OUT,
                                             Communications::Port::TransportType::UDP, sim_url, "nomad.sim.joint_cmd");

    Port::Map(nomad_simulation_interface.GetInputPort(StandController::InputPort::IMU_STATE_IN),
              SIM_IMU);

    Port::Map(nomad_simulation_interface.GetInputPort(StandController::InputPort::JOINT_STATE_IN),
               JOINT_STATE);

    Port::Map(nomad_simulation_interface.GetInputPort(StandController::InputPort::COM_STATE_IN),
              COM_STATE);

    nomad_simulation_interface.Start();

    // Print Threads
    RealTimeTaskManager::Instance()->PrintActiveTasks();

    // Start Inproc Context Process Thread
    PortManager::Instance()->GetInprocContext()->start();

    // // Run for 10 Seconds
    // int j = 0;
    // while (j < 2)
    // {

    //     usleep(1e6);
    //     j++;
    // }

    getchar();

    //nomad.Stop();
    // scope.Stop();
    // scope2.Stop();
    //  ref_generator_node.Stop();
    // convex_mpc_node.Stop();
    // estimator_node.Stop();
    // teleop_node.Stop();

    //  scope.DumpCSV("test.csv");
    //scope2.DumpCSV("test2.csv");
    // scope.RenderPlot();
    //scope2.RenderPlot();
}