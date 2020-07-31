#include <Realtime/RealTimeTask.hpp>
#include <Communications/Port.hpp>
#include <OperatorInterface/RemoteTeleop.hpp>
#include <Systems/NomadPlant.hpp>
#include <Common/Time.hpp>

#include <memory>

#include <unistd.h>
#include <sys/mman.h>


int main(int argc, char *argv[])
{

    //https://rt.wiki.kernel.org/index.php/Threaded_RT-application_with_memory_locking_and_stack_handling_example
    // if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    // {
    //     printf("mlockall failed: %m\n");
    //     exit(-2);
    // }

    // Task Periods.
    int freq1 = 50;
    // int freq2 = 100;

    // // Horizons
    // const int N = 16;
    // const double T = 1.5;
    // const double T_s = T / N;

    // const std::string gazebo_url = "udpm://239.255.76.67:7667?ttl=0";

    // Create Manager Class Instance Singleton.
    // Must make sure this is done before any thread tries to access.
    // And thus tries to allocate memory inside the thread heap.
    Realtime::RealTimeTaskManager::Instance();
    Communications::PortManager::Instance();

    // std::shared_ptr<Communications::Port> GAZEBO_IMU = std::make_shared<Communications::Port>("GAZEBO_IMU", Communications::Port::Direction::OUTPUT, Communications::Port::DataType::DOUBLE, 2, 10);
    // GAZEBO_IMU->SetTransport(Communications::Port::TransportType::UDP, gazebo_url, "nomad.imu");

    // Remote Teleop Task
    OperatorInterface::Teleop::RemoteTeleop teleop_node("Remote_Teleop");
    teleop_node.SetStackSize(1024 * 1024); // 1MB
    teleop_node.SetTaskPriority(Realtime::Priority::MEDIUM);
    teleop_node.SetTaskFrequency(freq1); // 50 HZ
    //teleop_node.SetCoreAffinity(-1);
    teleop_node.SetPortOutput(OperatorInterface::Teleop::RemoteTeleop::OutputPort::SETPOINT,
                              Communications::Port::TransportType::INPROC, "inproc", "nomad.setpoint");
    teleop_node.Start();

    // // State Estimator
    // Controllers::Estimators::StateEstimator estimator_node("Estimator_Task");
    // estimator_node.SetStackSize(1024 * 1024); // 1MB
    // estimator_node.SetTaskPriority(Realtime::Priority::MEDIUM);
    // estimator_node.SetTaskFrequency(freq2); // 1000 HZ
    // estimator_node.SetCoreAffinity(1);
    // estimator_node.SetPortOutput(Controllers::Estimators::StateEstimator::OutputPort::STATE_HAT,
    //                              Communications::Port::TransportType::INPROC, "inproc", "nomad.state");

    // //Reference Trajectory Generator
    // Controllers::Locomotion::ReferenceTrajectoryGenerator ref_generator_node("Reference_Trajectory_Task", N, T);
    // ref_generator_node.SetStackSize(1024 * 1024); // 1MB
    // ref_generator_node.SetTaskPriority(Realtime::Priority::MEDIUM);
    // ref_generator_node.SetTaskFrequency(freq1); // 50 HZ
    // ref_generator_node.SetCoreAffinity(-1);
    // ref_generator_node.SetPortOutput(Controllers::Locomotion::ReferenceTrajectoryGenerator::OutputPort::REFERENCE,
    //                                  Communications::Port::TransportType::INPROC, "inproc", "nomad.reference");

    // // Map State Estimator Output to Trajectory Reference Input
    // Communications::Port::Map(ref_generator_node.GetInputPort(Controllers::Locomotion::ReferenceTrajectoryGenerator::InputPort::STATE_HAT),
    //                     estimator_node.GetOutputPort(Controllers::Estimators::StateEstimator::OutputPort::STATE_HAT));

    // // Map Setpoint Output to Trajectory Reference Generator Input
    // Communications::Port::Map(ref_generator_node.GetInputPort(Controllers::Locomotion::ReferenceTrajectoryGenerator::InputPort::SETPOINT),
    //                     teleop_node.GetOutputPort(OperatorInterface::Teleop::RemoteTeleop::OutputPort::SETPOINT));
    // ref_generator_node.Start();


    // // Convex Model Predicive Controller for Locomotion
    // Controllers::Locomotion::ConvexMPC convex_mpc_node("Convex_MPC_Task", N, T);
    // convex_mpc_node.SetStackSize(8192 * 1024); // 8MB
    // convex_mpc_node.SetTaskPriority(Realtime::Priority::HIGH);
    // convex_mpc_node.SetTaskFrequency(freq1); // 50 HZ
    // convex_mpc_node.SetCoreAffinity(2);
    // convex_mpc_node.SetPortOutput(Controllers::Locomotion::ConvexMPC::OutputPort::FORCES, Communications::Port::TransportType::UDP, gazebo_url, "nomad.forces");

    // // Map State Estimator Output to Trajectory Reference Input
    // Communications::Port::Map(convex_mpc_node.GetInputPort(Controllers::Locomotion::ConvexMPC::InputPort::STATE_HAT),
    //                     estimator_node.GetOutputPort(Controllers::Estimators::StateEstimator::OutputPort::STATE_HAT));

    // // Map Reference Trajectory Output to Trajectory Reference Input of MPC
    // Communications::Port::Map(convex_mpc_node.GetInputPort(Controllers::Locomotion::ConvexMPC::InputPort::REFERENCE_TRAJECTORY),
    //                     ref_generator_node.GetOutputPort(Controllers::Locomotion::ReferenceTrajectoryGenerator::OutputPort::REFERENCE));

    // convex_mpc_node.Start();

    // // Plotter Task Node
    // Plotting::PlotterTaskNode scope("Forces");
    // scope.SetStackSize(8192 * 1024);
    // scope.SetTaskPriority(Realtime::Priority::LOWEST);
    // scope.SetTaskFrequency(freq1); // 50 HZ
    // scope.SetCoreAffinity(2);
    // scope.ConnectInput(Plotting::PlotterTaskNode::PORT_1, convex_mpc_node.GetOutputPort(Controllers::Locomotion::ConvexMPC::OutputPort::FORCES));
    // scope.AddPlotVariable(Plotting::PlotterTaskNode::PORT_1, Controllers::Locomotion::ConvexMPC::U);
    // scope.Start();

    // Plotting::PlotterTaskNode scope2("State");
    // scope2.SetStackSize(8192 * 1024);
    // scope2.SetTaskPriority(Realtime::Priority::LOWEST);
    // scope2.SetTaskFrequency(freq1); // 50 HZ
    // scope2.SetCoreAffinity(2);
    // scope2.ConnectInput(Plotting::PlotterTaskNode::PORT_1, estimator_node.GetOutputPort(Controllers::Estimators::StateEstimator::OutputPort::STATE_HAT));
    // scope2.AddPlotVariable(Plotting::PlotterTaskNode::PORT_1, Controllers::Estimators::StateEstimator::X);
    // scope2.AddPlotVariable(Plotting::PlotterTaskNode::PORT_1, Controllers::Estimators::StateEstimator::X_DOT);
    // scope2.Start();

    // Gait Scheduler
    // Controllers::Locomotion::GaitScheduler gait_scheduler_node("Gait_Scheduler_Task");
    // gait_scheduler_node.SetStackSize(100000);
    // gait_scheduler_node.SetTaskPriority(Controllers::RealTimeControl::Priority::MEDIUM);
    // gait_scheduler_node.SetTaskFrequency(10); // 1000 HZ
    // gait_scheduler_node.SetCoreAffinity(2);
    // gait_scheduler_node.SetPortOutput(Controllers::Locomotion::GaitScheduler::OutputPort::CONTACT_STATE, "nomad/gait_contacts");
    // gait_scheduler_node.Start();

    // Plant Node
    // Systems::Nomad::NomadPlant nomad("Nomad_Plant", 1.0/freq2);
    // nomad.SetStackSize(8192 * 1024); // 8 MB
    // nomad.SetTaskPriority(Realtime::Priority::HIGH);
    // nomad.SetTaskFrequency(freq2); // 1000 HZ
    // nomad.SetCoreAffinity(3);
    // nomad.SetPortOutput(Systems::Nomad::NomadPlant::STATE, Communications::Port::TransportType::UDP, "udpm://239.255.76.67:7667?ttl=0", "nomad.imu");

    //Communications::Port::Map(nomad.GetInputPort(Systems::Nomad::NomadPlant::InputPort::FORCES),
    // convex_mpc_node.GetOutputPort(Controllers::Locomotion::ConvexMPC::OutputPort::FORCES));

    // Communications::Port::Map(estimator_node.GetInputPort(Controllers::Estimators::StateEstimator::InputPort::IMU),
    //                     GAZEBO_IMU);

    //nomad.Start();

    //estimator_node.Start();

    // Print Threads
    Realtime::RealTimeTaskManager::Instance()->PrintActiveTasks();

    // Start Inproc Context Process Thread
    Communications::PortManager::Instance()->GetInprocContext()->start();

    // Run for 10 Seconds
    int j = 0;
    while (j < 50)
    {    

        usleep(1e6);
        j++;
    }
    //nomad.Stop();
   // scope.Stop();
   // scope2.Stop();
  //  ref_generator_node.Stop();
   // convex_mpc_node.Stop();
   // estimator_node.Stop();
    teleop_node.Stop();

  //  scope.DumpCSV("test.csv");
    //scope2.DumpCSV("test2.csv");
   // scope.RenderPlot();
    //scope2.RenderPlot();
}