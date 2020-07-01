#include <Realtime/RealTimeTask.hpp>
#include <Communications/Port.hpp>
#include <Controllers/LegController.hpp>
#include <Nomad/NomadControl.hpp>
#include <OperatorInterface/RemoteTeleop.hpp>
#include <Common/Time.hpp>
#include <Nomad/NomadRobot.hpp>
#include <Nomad/FSM/NomadControlFSM.hpp>

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

    // Create Manager Class Instance Singleton.
    // Must make sure this is done before any thread tries to access.
    // And thus tries to allocate memory inside the thread heap.
    Realtime::RealTimeTaskManager::Instance();
    Realtime::PortManager::Instance();

    // Remote Teleop Task
    OperatorInterface::Teleop::RemoteTeleop teleop_node("Remote_Teleop");
    teleop_node.SetStackSize(1024 * 1024); // 1MB
    teleop_node.SetTaskPriority(Realtime::Priority::MEDIUM);
    teleop_node.SetTaskFrequency(freq1); // 50 HZ
    //teleop_node.SetCoreAffinity(-1);
    teleop_node.SetPortOutput(OperatorInterface::Teleop::RemoteTeleop::OutputPort::MODE,
                              Realtime::Port::TransportType::INPROC, "inproc", "nomad.teleop.mode");
    teleop_node.SetPortOutput(OperatorInterface::Teleop::RemoteTeleop::OutputPort::SETPOINT,
                              Realtime::Port::TransportType::INPROC, "inproc", "nomad.teleop.setpoint");
                              
    teleop_node.Start();

    std::shared_ptr<Robot::Nomad::FSM::NomadControlFSM> test = std::make_shared<Robot::Nomad::FSM::NomadControlFSM >();

    // // FSM Task
    // Robot::Nomad::Controllers::NomadControl nomad_controller_node("Nomad_Controller");

    // nomad_controller_node.SetStackSize(1024 * 1024); // 1MB   
    // nomad_controller_node.SetTaskPriority(Realtime::Priority::MEDIUM);
    // nomad_controller_node.SetTaskFrequency(freq1); // 50 HZ
    // //nomad_controller_node.SetCoreAffinity(-1);
    // nomad_controller_node.SetPortOutput(Robot::Nomad::Controllers::NomadControl::OutputPort::LEG_COMMAND,
    //                                   Realtime::Port::TransportType::INPROC, "inproc", "nomad.control.fsm.leg_cmd");

    // Realtime::Port::Map(nomad_controller_node.GetInputPort(Robot::Nomad::Controllers::NomadControl::InputPort::CONTROL_MODE),
    //                     teleop_node.GetOutputPort(OperatorInterface::Teleop::RemoteTeleop::OutputPort::MODE));

    // nomad_controller_node.Start();


    // Print Threads
    Realtime::RealTimeTaskManager::Instance()->PrintActiveTasks();

    // Start Inproc Context Process Thread
    Realtime::PortManager::Instance()->GetInprocContext()->start();

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