#include <Realtime/RealTimeTask.hpp>
#include <Realtime/Port.hpp>
#include <Controllers/StateEstimator.hpp>
#include <Controllers/ConvexMPC.hpp>
#include <Controllers/GaitScheduler.hpp>
#include <Controllers/ReferenceTrajectoryGen.hpp>
#include <OperatorInterface/RemoteTeleop.hpp>
#include <Plotting/PlotterTaskNode.hpp>
#include <Systems/NomadPlant.hpp>
#include <Systems/Time.hpp>

#include <unistd.h>

int main(int argc, char *argv[])
{

//    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
//    {
//       printf("mlockall failed: %m\n");
//        exit(-2);
//    }


    
    int freq1 = 20;
    int freq2 = 20;
    std::cout << EIGEN_WORLD_VERSION << EIGEN_MAJOR_VERSION << EIGEN_MINOR_VERSION << std::endl;
    const int N = 16;
    const double T = 1.5;
    const double T_s = T / N;

    // Create Manager Class Instance Singleton.  Must make sure this is done before any thread tries to access.  And thus tries to allocate memory inside the thread heap.
    Realtime::RealTimeTaskManager::Instance();
    Realtime::PortManager::Instance();

    // Remote Teleop Task
    OperatorInterface::Teleop::RemoteTeleop teleop_node("Remote_Teleop");
    teleop_node.SetStackSize(100000);
    teleop_node.SetTaskPriority(Realtime::Priority::MEDIUM);
    teleop_node.SetTaskFrequency(freq1); // 50 HZ
    teleop_node.SetCoreAffinity(-1);
    teleop_node.SetPortOutput(OperatorInterface::Teleop::RemoteTeleop::OutputPort::SETPOINT, Realtime::Port::TransportType::INPROC, "inproc", "nomad.setpoint");
    teleop_node.Start();

    // Delay
    //usleep(100000);

    // State Estimator
    Controllers::Estimators::StateEstimator estimator_node("Estimator_Task");
    estimator_node.SetStackSize(100000);
    estimator_node.SetTaskPriority(Realtime::Priority::MEDIUM);
    estimator_node.SetTaskFrequency(freq2); // 1000 HZ
    estimator_node.SetCoreAffinity(1);
    estimator_node.SetPortOutput(Controllers::Estimators::StateEstimator::OutputPort::STATE_HAT, Realtime::Port::TransportType::INPROC, "inproc", "nomad.state");


    // Delay
    //usleep(100000);

    //Reference Trajectory Generator
    Controllers::Locomotion::ReferenceTrajectoryGenerator ref_generator_node("Reference_Trajectory_Task", N, T);
    ref_generator_node.SetStackSize(100000);
    ref_generator_node.SetTaskPriority(Realtime::Priority::MEDIUM);
    ref_generator_node.SetTaskFrequency(freq1); // 50 HZ
    ref_generator_node.SetCoreAffinity(-1);
    ref_generator_node.SetPortOutput(Controllers::Locomotion::ReferenceTrajectoryGenerator::OutputPort::REFERENCE, Realtime::Port::TransportType::INPROC, "inproc", "nomad.reference");

    // Map State Estimator Output to Trajectory Reference Input
    Realtime::Port::Map(ref_generator_node.GetInputPort(Controllers::Locomotion::ReferenceTrajectoryGenerator::InputPort::STATE_HAT), 
    estimator_node.GetOutputPort(Controllers::Estimators::StateEstimator::OutputPort::STATE_HAT));
    
    // Map Setpoint Output to Trajectory Reference Generator Input
    Realtime::Port::Map(ref_generator_node.GetInputPort(Controllers::Locomotion::ReferenceTrajectoryGenerator::InputPort::SETPOINT), 
    teleop_node.GetOutputPort(OperatorInterface::Teleop::RemoteTeleop::OutputPort::SETPOINT));
    ref_generator_node.Start();

    // Delay
    //usleep(100000);

    // Convex Model Predicive Controller for Locomotion
    Controllers::Locomotion::ConvexMPC convex_mpc_node("Convex_MPC_Task", N, T);
    convex_mpc_node.SetStackSize(100000);
    convex_mpc_node.SetTaskPriority(Realtime::Priority::HIGH);
    convex_mpc_node.SetTaskFrequency(freq1); // 50 HZ
    convex_mpc_node.SetCoreAffinity(2);
    convex_mpc_node.SetPortOutput(Controllers::Locomotion::ConvexMPC::OutputPort::FORCES, Realtime::Port::TransportType::INPROC, "inproc", "nomad.forces");

    // Map State Estimator Output to Trajectory Reference Input
    Realtime::Port::Map(convex_mpc_node.GetInputPort(Controllers::Locomotion::ConvexMPC::InputPort::STATE_HAT), 
    estimator_node.GetOutputPort(Controllers::Estimators::StateEstimator::OutputPort::STATE_HAT));

    // Map Reference Trajectory Output to Trajectory Reference Input of MPC
    Realtime::Port::Map(convex_mpc_node.GetInputPort(Controllers::Locomotion::ConvexMPC::InputPort::REFERENCE_TRAJECTORY), 
    ref_generator_node.GetOutputPort(Controllers::Locomotion::ReferenceTrajectoryGenerator::OutputPort::REFERENCE));

    convex_mpc_node.Start();

    //usleep(100000);

    // Plotter Task Node
    Plotting::PlotterTaskNode scope("State");
    scope.SetStackSize(100000);
    scope.SetTaskPriority(Realtime::Priority::MEDIUM);
    scope.SetTaskFrequency(100); // 50 HZ
    scope.SetCoreAffinity(-1);
    //scope.ConnectInput(Plotting::PlotterTaskNode::PORT_1, convex_mpc_node.GetOutputPort(Controllers::Locomotion::ConvexMPC::OutputPort::FORCES));
    scope.ConnectInput(Plotting::PlotterTaskNode::PORT_1, estimator_node.GetOutputPort(Controllers::Estimators::StateEstimator::OutputPort::STATE_HAT));
    //scope.ConnectInput(Plotting::PlotterTaskNode::PORT_2, teleop_node.GetOutputPort(OperatorInterface::Teleop::RemoteTeleop::OutputPort::SETPOINT));
    scope.AddPlotVariable(Plotting::PlotterTaskNode::PORT_1, Controllers::Estimators::StateEstimator::X);
    scope.Start();

    Plotting::PlotterTaskNode scope2("State2");
    scope2.SetStackSize(100000);
    scope2.SetTaskPriority(Realtime::Priority::MEDIUM);
    scope2.SetTaskFrequency(100); // 50 HZ
    scope2.SetCoreAffinity(-1);
    scope2.ConnectInput(Plotting::PlotterTaskNode::PORT_1, convex_mpc_node.GetOutputPort(Controllers::Locomotion::ConvexMPC::OutputPort::FORCES));
    //scope2.ConnectInput(Plotting::PlotterTaskNode::PORT_1, estimator_node.GetOutputPort(Controllers::Estimators::StateEstimator::OutputPort::STATE_HAT));
    //scope.ConnectInput(Plotting::PlotterTaskNode::PORT_2, teleop_node.GetOutputPort(OperatorInterface::Teleop::RemoteTeleop::OutputPort::SETPOINT));
    scope2.AddPlotVariable(Plotting::PlotterTaskNode::PORT_1, Controllers::Estimators::StateEstimator::X);
    scope2.Start();


    // Gait Scheduler
    // Controllers::Locomotion::GaitScheduler gait_scheduler_node("Gait_Scheduler_Task");
    // gait_scheduler_node.SetStackSize(100000);
    // gait_scheduler_node.SetTaskPriority(Controllers::RealTimeControl::Priority::MEDIUM);
    // gait_scheduler_node.SetTaskFrequency(10); // 1000 HZ
    // gait_scheduler_node.SetCoreAffinity(2);
    // gait_scheduler_node.SetPortOutput(Controllers::Locomotion::GaitScheduler::OutputPort::CONTACT_STATE, "nomad/gait_contacts");
    // gait_scheduler_node.Start();

    // Plant Node

    Systems::Nomad::NomadPlant nomad("Nomad_Plant", 0.01);
    nomad.SetStackSize(100000);
    nomad.SetTaskPriority(Realtime::Priority::HIGH);
    nomad.SetTaskFrequency(100); // 1000 HZ
    nomad.SetCoreAffinity(1);
    nomad.SetPortOutput(Systems::Nomad::NomadPlant::STATE, Realtime::Port::TransportType::INPROC, "inproc", "nomad.imu");

    Realtime::Port::Map(nomad.GetInputPort(Systems::Nomad::NomadPlant::InputPort::FORCES), 
    convex_mpc_node.GetOutputPort(Controllers::Locomotion::ConvexMPC::OutputPort::FORCES));

    Realtime::Port::Map(estimator_node.GetInputPort(Controllers::Estimators::StateEstimator::InputPort::IMU), 
    nomad.GetOutputPort(Systems::Nomad::NomadPlant::STATE));

    nomad.Start();

    estimator_node.Start();



    // Print Threads
    Realtime::RealTimeTaskManager::Instance()->PrintActiveTasks();

    // Start Inproc Context Process Thread
    Realtime::PortManager::Instance()->GetInprocContext()->start();

    std::cout << "BLAH: " << Systems::Time::GetTime() / 1e6<< std::endl;

    int j = 0;
    while (j <  5)
    {
        usleep(1e6);
        j++;
    }
    nomad.Stop();
    scope.Stop();
    scope2.Stop();
    ref_generator_node.Stop();
    convex_mpc_node.Stop();
    estimator_node.Stop();
    teleop_node.Stop();

    scope.DumpCSV("test.csv");
    scope2.DumpCSV("test2.csv");
    //scope2.RenderPlot();
}