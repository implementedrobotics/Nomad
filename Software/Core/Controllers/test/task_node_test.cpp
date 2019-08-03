#include <Realtime/RealTimeTask.hpp>
#include <Realtime/Port.hpp>
#include <Controllers/StateEstimator.hpp>
#include <Controllers/ConvexMPC.hpp>
#include <Controllers/GaitScheduler.hpp>
#include <Controllers/ReferenceTrajectoryGen.hpp>
#include <OperatorInterface/RemoteTeleop.hpp>
#include <Plotting/PlotterTaskNode.hpp>
#include <unistd.h>

int main(int argc, char *argv[])
{

    std::cout << EIGEN_WORLD_VERSION << EIGEN_MAJOR_VERSION << EIGEN_MINOR_VERSION << std::endl;
    const int N = 10;
    const double T = 1.0;

    // Create Manager Class Instance Singleton.  Must make sure this is done before any thread tries to access.  And thus tries to allocate memory inside the thread heap.
    Realtime::RealTimeTaskManager::Instance();
    Realtime::PortManager::Instance();

    // Remote Teleop Task
    OperatorInterface::Teleop::RemoteTeleop teleop_node("Remote_Teleop");
    teleop_node.SetStackSize(100000);
    teleop_node.SetTaskPriority(Realtime::Priority::MEDIUM);
    teleop_node.SetTaskFrequency(2); // 50 HZ
    teleop_node.SetCoreAffinity(-1);
    teleop_node.SetPortOutput(OperatorInterface::Teleop::RemoteTeleop::OutputPort::SETPOINT, Realtime::Port::TransportType::INPROC, "inproc", "nomad.setpoint");
    teleop_node.Start();

    // Delay
    usleep(100000);

    // State Estimator
    Controllers::Estimators::StateEstimator estimator_node("Estimator_Task");
    estimator_node.SetStackSize(100000);
    estimator_node.SetTaskPriority(Realtime::Priority::MEDIUM);
    estimator_node.SetTaskFrequency(10); // 1000 HZ
    estimator_node.SetCoreAffinity(1);
    estimator_node.SetPortOutput(Controllers::Estimators::StateEstimator::OutputPort::STATE_HAT, Realtime::Port::TransportType::INPROC, "inproc", "nomad.state");
    estimator_node.Start();

    // Delay
    usleep(100000);

    //Reference Trajectory Generator
    Controllers::Locomotion::ReferenceTrajectoryGenerator ref_generator_node("Reference_Trajectory_Task", N, T);
    ref_generator_node.SetStackSize(100000);
    ref_generator_node.SetTaskPriority(Realtime::Priority::MEDIUM);
    ref_generator_node.SetTaskFrequency(2); // 50 HZ
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
    usleep(100000);

    // Convex Model Predicive Controller for Locomotion
    Controllers::Locomotion::ConvexMPC convex_mpc_node("Convex_MPC_Task", N, T);
    convex_mpc_node.SetStackSize(100000);
    convex_mpc_node.SetTaskPriority(Realtime::Priority::HIGH);
    convex_mpc_node.SetTaskFrequency(2); // 50 HZ
    convex_mpc_node.SetCoreAffinity(2);
    convex_mpc_node.SetPortOutput(Controllers::Locomotion::ConvexMPC::OutputPort::FORCES, Realtime::Port::TransportType::INPROC, "inproc", "nomad.forces");

    // Map State Estimator Output to Trajectory Reference Input
    Realtime::Port::Map(convex_mpc_node.GetInputPort(Controllers::Locomotion::ConvexMPC::InputPort::STATE_HAT), 
    estimator_node.GetOutputPort(Controllers::Estimators::StateEstimator::OutputPort::STATE_HAT));

    // Map Reference Trajectory Output to Trajectory Reference Input of MPC
    Realtime::Port::Map(convex_mpc_node.GetInputPort(Controllers::Locomotion::ConvexMPC::InputPort::REFERENCE_TRAJECTORY), 
    ref_generator_node.GetOutputPort(Controllers::Locomotion::ReferenceTrajectoryGenerator::OutputPort::REFERENCE));

    convex_mpc_node.Start();

    usleep(100000);

    // Plotter Task Node
    Plotting::PlotterTaskNode scope("State");
    scope.SetStackSize(100000);
    scope.SetTaskPriority(Realtime::Priority::MEDIUM);
    scope.SetTaskFrequency(2); // 50 HZ
    scope.SetCoreAffinity(-1);
    scope.ConnectInput(Plotting::PlotterTaskNode::PORT_1, estimator_node.GetOutputPort(Controllers::Estimators::StateEstimator::OutputPort::STATE_HAT));
    scope.ConnectInput(Plotting::PlotterTaskNode::PORT_2, teleop_node.GetOutputPort(OperatorInterface::Teleop::RemoteTeleop::OutputPort::SETPOINT));
    scope.Start();


    // Gait Scheduler
    // Controllers::Locomotion::GaitScheduler gait_scheduler_node("Gait_Scheduler_Task");
    // gait_scheduler_node.SetStackSize(100000);
    // gait_scheduler_node.SetTaskPriority(Controllers::RealTimeControl::Priority::MEDIUM);
    // gait_scheduler_node.SetTaskFrequency(10); // 1000 HZ
    // gait_scheduler_node.SetCoreAffinity(2);
    // gait_scheduler_node.SetPortOutput(Controllers::Locomotion::GaitScheduler::OutputPort::CONTACT_STATE, "nomad/gait_contacts");
    // gait_scheduler_node.Start();

    // Start Inproc Context Process Thread
    Realtime::PortManager::Instance()->GetInprocContext()->start();

    // Print Threads
    Realtime::RealTimeTaskManager::Instance()->PrintActiveTasks();

    int j = 0;
    while (j <  5)
    {
        //printf("[TASK_NODE_TEST]: IDLE TASK\n");
        usleep(1000000);
        j++;
        //estimator_node.Stop();
    }

    scope.Stop();
    ref_generator_node.Stop();
    convex_mpc_node.Stop();
    estimator_node.Stop();
    teleop_node.Stop();
    scope.RenderPlot();
    std::cout << "OUT!" << std::endl;
}