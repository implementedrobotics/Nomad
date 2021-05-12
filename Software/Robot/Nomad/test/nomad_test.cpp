
#include <Communications/Messages/msg_helpers.hpp>
#include <Realtime/RealTimeTask.hpp>
#include <Communications/Port.hpp>
#include <Systems/BlockDiagram.hpp>
#include <Systems/SystemBlock.hpp>
#include <Nomad/Interface/SimulationInterface.hpp>
#include <Nomad/Estimators/FusedLegKinematicsStateEstimator.hpp>
#include <Controllers/LegController.hpp>
#include <Nomad/NomadControl.hpp>
#include <Nomad/NomadDynamics.hpp>
#include <Nomad/NomadRobot.hpp>
#include <Nomad/OperatorInterface/RemoteTeleop.hpp>
#include <Common/Time.hpp>
#include <memory>

#include <Nomad/MessageTypes/full_state_t.hpp>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>

#include <unistd.h>
#include <sys/mman.h>

using namespace Core::Systems;

using Communications::PortInterface;
// using Communications::PortManager;
// using Realtime::RealTimeTaskManager;
// using Realtime::RealTimeTaskNode;

using Robot::Nomad::NomadRobot;
using Robot::Nomad::Controllers::NomadControl;
using Robot::Nomad::Dynamics::NomadDynamics;
using Robot::Nomad::Interface::SimulationInterface;

using Controllers::Locomotion::LegController;

using Robot::Nomad::Estimators::FusedLegKinematicsStateEstimator;

using OperatorInterface::Teleop::RemoteTeleop;



// template <typename T>
// struct conv;

// template<>
// struct conv<Hello> {
//   static void encode() {
//       std::cout << "YELLO2" << std::endl;
//   }
// };



// template <typename  T>
// struct PortConverter
// {
//     typedef T TestTypel;

//     T me;

//   void Receive() {
//       std::cout << "YELLO3" << std::endl;
//       me.Print();
//   }

// };
// namespace Communications
// {
//     template <>
//     struct conv<leg_controller_cmd_t>
//     {
//         static void Demux()
//         {
//             std::cout << "DVT" << std::endl;
//         }
//     };

//     template <>
//     struct conv<full_state_t>
//     {
//         static void Demux()
//         {
//             std::cout << "DVT" << std::endl;
//         }
//     };

//         template <>
//     struct conv<teleop_data_t>
//     {
//         static void Demux()
//         {
//             std::cout << "DVT" << std::endl;
//         }
//     };

//             template <>
//     struct conv<joint_state_t>
//     {
//         static void Demux()
//         {
//             std::cout << "DVT" << std::endl;
//         }
//     };

// }

template <typename T = int>
class Foo{

    public:
    T a;
};



int main(int argc, char *argv[])
{


    // std::vector<Foo<>*> vec;

    // Foo<int> me;

    // vec.push_back(&me);
    
    // std::cout << sizeof(me) << std::endl;
    // return true;
    // PortConverter<Hello> converter;
    // PortConverter<double> converter2;

    // converter.Receive();

    // conv<decltype(converter)::TestTypel>::encode();
    // conv<decltype(converter2)::TestTypel>::encode();
    // TODO: Validate proper path/config file.  And Error
    // std::string nomad_config = std::getenv("NOMAD_CONFIG_PATH");
    // nomad_config.append("/robot-config.yaml");

    // YAML::Node config = YAML::LoadFile(nomad_config);

    // double test = config["mass"].as<double>();

    // std::cout << "Read: " << test << std::endl;

    //return 0;
    // Create Manager Class Instance Singleton.
    // Must make sure this is done before any thread tries to access.
    // And thus tries to allocate memory inside the thread heap.
    Realtime::RealTimeTaskManager::Instance();
    Communications::PortManager::Instance();

    if (!Realtime::RealTimeTaskManager::EnableRTMemory(500 * 1024 * 1024)) // 500MB
    {
        exit(-2);
        std::cout << "Error configuring Realtime Memory Requiremets!  Realtime Execution NOT guaranteed." << std::endl;
    }

    // Create Block Diagram
    BlockDiagram diagram("NOMAD", 0.001); //1khz
    diagram.SetStackSize(16 * 1024 * 1024);
    diagram.SetTaskPriority(Realtime::Priority::HIGHEST);
    diagram.SetCoreAffinity(-1);

    // Sim Interface
    std::shared_ptr<SimulationInterface> sim = std::make_shared<SimulationInterface>(0.001);
    sim->SetPortOutput(SimulationInterface::COM_STATE, PortInterface::TransportType::NATIVE, "native", "nomad.com_state");
    sim->SetPortOutput(SimulationInterface::IMU_DATA, PortInterface::TransportType::NATIVE, "native", "nomad.imu");
    sim->SetPortOutput(SimulationInterface::JOINT_STATE, PortInterface::TransportType::NATIVE, "native", "nomad.joint_state");


    std::shared_ptr<Core::Systems::Demux<com_state_t>> demux = std::make_shared<Core::Systems::Demux<com_state_t>>(0.001);

    sim->GetOutputPort(SimulationInterface::OutputPort::COM_STATE)->Demux();


    return 0;

   // diagram.AddSystem(sim);

    // // Estimator
    // std::shared_ptr<FusedLegKinematicsStateEstimator> estimate = std::make_shared<FusedLegKinematicsStateEstimator>(0.001);
    // estimate->SetPortOutput(FusedLegKinematicsStateEstimator::OutputPort::BODY_STATE_HAT, PortInterface::TransportType::NATIVE, "native", "nomad.body.state");
    // diagram.AddSystem(estimate);

    // // Nomad Dynamics Computation Task
    // std::shared_ptr<NomadDynamics> dynamics = std::make_shared<NomadDynamics>(0.001);

    // // Load DART from URDF
    // std::string urdf = std::getenv("NOMAD_RESOURCE_PATH");
    // urdf.append("/Robot_V2/NOMAD.urdf");

    // dart::dynamics::SkeletonPtr robot = NomadRobot::Load(urdf);

    // dynamics->SetRobotSkeleton(robot->cloneSkeleton());
    // dynamics->SetPortOutput(NomadDynamics::OutputPort::FULL_STATE, PortInterface::TransportType::NATIVE, "native", "nomad.robot.state");
    // diagram.AddSystem(dynamics);
    
    // // Port::Map(nomad_dynamics_node.GetInputPort(NomadDynamics::InputPort::JOINT_STATE),
    // //           nomad_simulation_interface.GetOutputPort(SimulationInterface::OutputPort::JOINT_STATE));

    // // Port::Map(nomad_dynamics_node.GetInputPort(NomadDynamics::InputPort::BODY_STATE_HAT),
    // //           nomad_simulation_interface.GetOutputPort(SimulationInterface::OutputPort::COM_STATE));
    

    // // Control FSM Task
    // std::shared_ptr<NomadControl> control = std::make_shared<NomadControl>(0.001);
    //0 control->SetPortOutput(NomadControl::OutputPort::LEG_COMMAND, PortInterface::TransportType::NATIVE, "native", "nomad.control.fsm.leg_cmd");
    // diagram.AddSystem(control);

    // // Leg Controller
    // std::shared_ptr<LegController> leg_controller = std::make_shared<LegController>(0.001);
    // leg_controller->SetPortOutput(LegController::OutputPort::SERVO_COMMAND, PortInterface::TransportType::NATIVE, "native", "nomad.control.servo_cmd");
    // diagram.AddSystem(leg_controller);

    // std::shared_ptr<RemoteTeleop> teleop = std::make_shared<RemoteTeleop>(0.1);
    // teleop->SetPortOutput(RemoteTeleop::OutputPort::TELEOP_DATA, PortInterface::TransportType::NATIVE, "native", "nomad.teleop.data");
    // diagram.AddSystem(teleop);

    // // Add some debug port converter
    // // TODO: Move this logic to scope node
    // // "Connect<leg_command_t> to Port Blah.  This creates an inline converter so we don't need an additional system block?
    // // std::shared_ptr<PortConverter<leg_controller_cmd_t>> converter = std::make_shared<PortConverter<leg_controller_cmd_t>>(0.001);
    // // diagram.AddSystem(converter);

    // //diagram.Connect(control->GetOutputPort(NomadControl::OutputPort::LEG_COMMAND), converter->GetInputPort(0));

    // // Connect the graph
    // diagram.Connect(sim->GetOutputPort(SimulationInterface::OutputPort::COM_STATE), estimate->GetInputPort(FusedLegKinematicsStateEstimator::InputPort::COM_STATE));
    // diagram.Connect(sim->GetOutputPort(SimulationInterface::OutputPort::JOINT_STATE), estimate->GetInputPort(FusedLegKinematicsStateEstimator::InputPort::JOINT_STATE));
    // diagram.Connect(sim->GetOutputPort(SimulationInterface::OutputPort::JOINT_STATE), dynamics->GetInputPort(NomadDynamics::InputPort::JOINT_STATE));
    // diagram.Connect(estimate->GetOutputPort(FusedLegKinematicsStateEstimator::OutputPort::BODY_STATE_HAT), dynamics->GetInputPort(NomadDynamics::BODY_STATE_HAT));

    // diagram.Connect(dynamics->GetOutputPort(NomadDynamics::OutputPort::FULL_STATE), control->GetInputPort(NomadControl::InputPort::FULL_STATE));
    
    // diagram.Connect(control->GetOutputPort(NomadControl::OutputPort::LEG_COMMAND), leg_controller->GetInputPort(LegController::InputPort::LEG_COMMAND));
    // diagram.Connect(leg_controller->GetOutputPort(LegController::OutputPort::SERVO_COMMAND), sim->GetInputPort(SimulationInterface::InputPort::JOINT_CONTROL_CMD_IN));
    
    // diagram.Connect(teleop->GetOutputPort(RemoteTeleop::OutputPort::TELEOP_DATA), control->GetInputPort(NomadControl::InputPort::TELEOP_DATA));


    // // Start Run
    // diagram.Start();

    // // Print Threads
    // Realtime::RealTimeTaskManager::Instance()->PrintActiveTasks();

    // // Start Inproc Context Process Thread
    // Communications::PortManager::Instance()->GetInprocContext()->start();

    // getchar();

    // diagram.Stop();


    // // nomad_simulation_interface.SetPortOutput(SimulationInterface::IMU_STATE_OUT,
    // //                                          Port::TransportType::NATIVE, "native", "nomad.imu");

    // nomad_simulation_interface.SetPortOutput(SimulationInterface::JOINT_STATE,
    //                                          Port::TransportType::NATIVE, "native", "nomad.joint_state");

    // nomad_simulation_interface.SetPortOutput(SimulationInterface::COM_STATE,
    //                                           Port::TransportType::NATIVE, "native", "nomad.com_state");

    // nomad_simulation_interface.SetPortOutput(SimulationInterface::JOINT_CONTROL_CMD_OUT,
    //                                          Communications::Port::TransportType::UDP, sim_url, "nomad.sim.joint_cmd");

    

    // nomad_dynamics_node.SetRobotSkeleton(robot->cloneSkeleton());

    // Port::Map(nomad_dynamics_node.GetInputPort(NomadDynamics::InputPort::JOINT_STATE),
    //           nomad_simulation_interface.GetOutputPort(SimulationInterface::OutputPort::JOINT_STATE));

    // Port::Map(nomad_dynamics_node.GetInputPort(NomadDynamics::InputPort::BODY_STATE_HAT),
    //           nomad_simulation_interface.GetOutputPort(SimulationInterface::OutputPort::COM_STATE));

    // nomad_dynamics_node.SetPortOutput(NomadDynamics::OutputPort::FULL_STATE,
    //                                   Port::TransportType::NATIVE, "native", "nomad.robot.state");



    // Port::Map(estimator_node.GetInputPort(FusedLegKinematicsStateEstimator::InputPort::FOOT_STATE),
    //           nomad_dynamics_node.GetOutputPort(NomadDynamics::OutputPort::FULL_STATE));

    // Port::Map(estimator_node.GetInputPort(FusedLegKinematicsStateEstimator::InputPort::IMU_DATA),
    //           nomad_simulation_interface.GetOutputPort(SimulationInterface::OutputPort::IMU_STATE));

    // Port::Map(estimator_node.GetInputPort(FusedLegKinematicsStateEstimator::InputPort::JOINT_STATE),
    //           nomad_simulation_interface.GetOutputPort(SimulationInterface::OutputPort::JOINT_STATE));

    // Port::Map(estimator_node.GetInputPort(FusedLegKinematicsStateEstimator::InputPort::COM_STATE),
    //           nomad_simulation_interface.GetOutputPort(SimulationInterface::OutputPort::COM_STATE));


    // Port::Map(nomad_controller_node.GetInputPort(NomadControl::InputPort::FULL_STATE),
    //           nomad_dynamics_node.GetOutputPort(NomadDynamics::OutputPort::FULL_STATE));
}