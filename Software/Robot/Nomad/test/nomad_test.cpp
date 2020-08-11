#include <Realtime/RealTimeTask.hpp>
#include <Communications/Port.hpp>
#include <Controllers/LegController.hpp>
#include <Nomad/NomadControl.hpp>
#include <Nomad/OperatorInterface/RemoteTeleop.hpp>
#include <Common/Time.hpp>
#include <Nomad/NomadRobot.hpp>
#include <Nomad/FSM/NomadControlFSM.hpp>
#include <Nomad/NomadDynamics.hpp>
#include <Controllers/StateEstimator.hpp>
#include <Nomad/Interface/SimulationInterface.hpp>
#include <Nomad/Estimators/FusedLegKinematicsStateEstimator.hpp>
#include <memory>

#include <unistd.h>
#include <sys/mman.h>

using Communications::Port;
using Communications::PortManager;
using Realtime::RealTimeTaskManager;
using Realtime::RealTimeTaskNode;

using Robot::Nomad::NomadRobot;
using Robot::Nomad::Controllers::NomadControl;
using Robot::Nomad::Dynamics::NomadDynamics;
using Robot::Nomad::Interface::SimulationInterface;

using Controllers::Locomotion::LegController;

using Robot::Nomad::Estimators::FusedLegKinematicsStateEstimator;

using OperatorInterface::Teleop::RemoteTeleop;

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
    SIM_IMU->SetTransport(Port::TransportType::IPC, "ipc", "nomad.sim.imu_state");

    std::shared_ptr<Port> JOINT_STATE = Port::CreateOutput("SIM_JOINT_STATE", 10);
    JOINT_STATE->SetTransport(Port::TransportType::IPC, "ipc", "nomad.sim.joint_state");

    std::shared_ptr<Port> COM_STATE = Port::CreateOutput("SIM_COM_STATE", 10);
    COM_STATE->SetTransport(Port::TransportType::IPC, "ipc", "nomad.sim.com_state");

    // Simulator Interface Task Setup
    SimulationInterface nomad_simulation_interface("Simulation Interface");
    nomad_simulation_interface.SetStackSize(1024 * 1024); // 1MB
    nomad_simulation_interface.SetTaskPriority(Realtime::Priority::HIGHEST);
    nomad_simulation_interface.SetTaskFrequency(hi_freq); // 50 HZ
    nomad_simulation_interface.SetCoreAffinity(2);
    nomad_simulation_interface.SetPortOutput(SimulationInterface::IMU_STATE_OUT,
                                             Port::TransportType::INPROC, "inproc", "nomad.imu");

    nomad_simulation_interface.SetPortOutput(SimulationInterface::JOINT_STATE_OUT,
                                             Port::TransportType::INPROC, "inproc", "nomad.joint_state");

    nomad_simulation_interface.SetPortOutput(SimulationInterface::COM_STATE_OUT,
                                             Port::TransportType::INPROC, "inproc", "nomad.com_state");
    nomad_simulation_interface.SetPortOutput(SimulationInterface::JOINT_CONTROL_CMD_OUT,
                                             Communications::Port::TransportType::IPC, "ipc", "nomad.sim.joint_cmd2");

    Port::Map(nomad_simulation_interface.GetInputPort(SimulationInterface::InputPort::IMU_STATE_IN),
              SIM_IMU);

    Port::Map(nomad_simulation_interface.GetInputPort(SimulationInterface::InputPort::JOINT_STATE_IN),
               JOINT_STATE);

    Port::Map(nomad_simulation_interface.GetInputPort(SimulationInterface::InputPort::COM_STATE_IN),
              COM_STATE);

    // Start Dynamics
    // Nomad Dynamics Computation Task
    NomadDynamics nomad_dynamics_node("Nomad_Dynamics_Handler");

    // Load DART from URDF
    std::string urdf = std::getenv("NOMAD_RESOURCE_PATH");
    urdf.append("/Robot/Nomad.urdf");

    //std::cout << "Load: " << urdf << std::endl;
    dart::dynamics::SkeletonPtr robot = NomadRobot::Load(urdf);

    nomad_dynamics_node.SetRobotSkeleton(robot->cloneSkeleton());
    nomad_dynamics_node.SetStackSize(1024 * 1024); // 1MB
    nomad_dynamics_node.SetTaskPriority(Realtime::Priority::MEDIUM);
    nomad_dynamics_node.SetTaskFrequency(hi_freq); // 50 HZ
    //nomad_dynamics_node.SetCoreAffinity(-1);
    Port::Map(nomad_dynamics_node.GetInputPort(NomadDynamics::InputPort::JOINT_STATE),
              nomad_simulation_interface.GetOutputPort(SimulationInterface::OutputPort::JOINT_STATE_OUT));

    Port::Map(nomad_dynamics_node.GetInputPort(NomadDynamics::InputPort::BODY_STATE_HAT),
              nomad_simulation_interface.GetOutputPort(SimulationInterface::OutputPort::COM_STATE_OUT));

    nomad_dynamics_node.SetPortOutput(NomadDynamics::OutputPort::FULL_STATE,
                                      Port::TransportType::INPROC, "inproc", "nomad.robot.state");

                                      

    nomad_dynamics_node.Start();

    // State Estimator Task
    FusedLegKinematicsStateEstimator estimator_node("Estimator_Task");
    estimator_node.SetStackSize(1024 * 1024); // 1MB
    estimator_node.SetTaskPriority(Realtime::Priority::MEDIUM);
    estimator_node.SetTaskFrequency(hi_freq); // 1000 HZ
    //estimator_node.SetCoreAffinity(1);
    estimator_node.SetPortOutput(FusedLegKinematicsStateEstimator::OutputPort::BODY_STATE_HAT,
                                 Port::TransportType::INPROC, "inproc", "nomad.com.state");

    estimator_node.SetPortOutput(FusedLegKinematicsStateEstimator::OutputPort::BODY_STATE_ACTUAL,
                                 Port::TransportType::INPROC, "inproc", "nomad.com.state2");

    Port::Map(estimator_node.GetInputPort(FusedLegKinematicsStateEstimator::InputPort::FOOT_STATE),
              nomad_dynamics_node.GetOutputPort(NomadDynamics::OutputPort::FULL_STATE));

    Port::Map(estimator_node.GetInputPort(FusedLegKinematicsStateEstimator::InputPort::IMU_DATA),
              nomad_simulation_interface.GetOutputPort(SimulationInterface::OutputPort::IMU_STATE_OUT));

    Port::Map(estimator_node.GetInputPort(FusedLegKinematicsStateEstimator::InputPort::JOINT_STATE),
              nomad_simulation_interface.GetOutputPort(SimulationInterface::OutputPort::JOINT_STATE_OUT));

    Port::Map(estimator_node.GetInputPort(FusedLegKinematicsStateEstimator::InputPort::COM_STATE),
              nomad_simulation_interface.GetOutputPort(SimulationInterface::OutputPort::COM_STATE_OUT));

    estimator_node.Start();

    // Remote Teleop Task
    RemoteTeleop teleop_node("Remote_Teleop");
    teleop_node.SetStackSize(1024 * 1024); // 1MB
    teleop_node.SetTaskPriority(Realtime::Priority::MEDIUM);
    teleop_node.SetTaskFrequency(freq1); // 50 HZ
    //teleop_node.SetCoreAffinity(-1);
    teleop_node.SetPortOutput(RemoteTeleop::OutputPort::TELEOP_DATA,
                              Port::TransportType::INPROC, "inproc", "nomad.teleop.data");

    teleop_node.Start();

    // FSM Task
    NomadControl nomad_controller_node("Nomad_Controller");

    nomad_controller_node.SetStackSize(1024 * 1024); // 1MB
    nomad_controller_node.SetTaskPriority(Realtime::Priority::MEDIUM);
    nomad_controller_node.SetTaskFrequency(hi_freq); // 50 HZ
    //nomad_controller_node.SetCoreAffinity(-1);
    nomad_controller_node.SetPortOutput(NomadControl::OutputPort::LEG_COMMAND,
                                        Communications::Port::TransportType::INPROC, "inproc", "nomad.control.fsm.leg_cmd");

    Port::Map(nomad_controller_node.GetInputPort(NomadControl::InputPort::TELEOP_DATA),
              teleop_node.GetOutputPort(RemoteTeleop::OutputPort::TELEOP_DATA));

    Port::Map(nomad_controller_node.GetInputPort(NomadControl::InputPort::FULL_STATE),
              nomad_dynamics_node.GetOutputPort(NomadDynamics::OutputPort::FULL_STATE));

    nomad_controller_node.Start();

    // Port Mappings
    //Port::Map(nomad_dynamics_node.GetInputPort(NomadDynamics::InputPort::BODY_STATE_HAT),
    //          estimator_node.GetOutputPort(FusedLegKinematicsStateEstimator::OutputPort::BODY_STATE_HAT));

    // Leg Controller Task
    LegController leg_controller_node("Leg_Controller");

    leg_controller_node.SetStackSize(1024 * 1024); // 1MB
    leg_controller_node.SetTaskPriority(Realtime::Priority::MEDIUM);
    leg_controller_node.SetTaskFrequency(hi_freq); // 50 HZ
    //leg_controller_node.SetCoreAffinity(-1);
    leg_controller_node.SetPortOutput(LegController::OutputPort::SERVO_COMMAND,
                                      Communications::Port::TransportType::INPROC, "inproc", "nomad.control.servo_cmd");

    Port::Map(leg_controller_node.GetInputPort(LegController::InputPort::LEG_COMMAND),
              nomad_controller_node.GetOutputPort(NomadControl::OutputPort::LEG_COMMAND));

    Port::Map(nomad_simulation_interface.GetInputPort(SimulationInterface::InputPort::JOINT_CONTROL_CMD_IN),
              leg_controller_node.GetOutputPort(LegController::OutputPort::SERVO_COMMAND));

    leg_controller_node.Start();

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