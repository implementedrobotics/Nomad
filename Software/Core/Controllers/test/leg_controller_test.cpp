#include <Realtime/RealTimeTask.hpp>
#include <Communications/Port.hpp>
#include <Controllers/LegController.hpp>
#include <Controllers/PrimaryControl.hpp>
#include <Systems/NomadPlant.hpp>
#include <Common/Time.hpp>
#include <Nomad/NomadRobot.hpp>

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

    // FSM Task
    Controllers::FSM::PrimaryControl primary_controller_node("Primary_Controller");

    primary_controller_node.SetStackSize(1024 * 1024); // 1MB   
    primary_controller_node.SetTaskPriority(Realtime::Priority::MEDIUM);
    primary_controller_node.SetTaskFrequency(freq1); // 50 HZ
    //leg_controller_node.SetCoreAffinity(-1);
    primary_controller_node.SetPortOutput(Controllers::FSM::PrimaryControl::OutputPort::LEG_COMMAND,
                                      Realtime::Port::TransportType::INPROC, "inproc", "nomad.controllers.fsm.leg_cmd");
    primary_controller_node.Start();


    // Leg Controller Task
    Controllers::Locomotion::LegController leg_controller_node("Leg_Controller");

    // Load DART from URDF
    std::string urdf = std::getenv("NOMAD_RESOURCE_PATH");
    urdf.append("/Robot/Nomad.urdf");

    std::cout << "Load: " << urdf << std::endl;
    dart::dynamics::SkeletonPtr robot = Robot::Nomad::NomadRobot::Load(urdf);

    leg_controller_node.SetRobotSkeleton(robot->cloneSkeleton());
    leg_controller_node.SetStackSize(1024 * 1024); // 1MB   
    leg_controller_node.SetTaskPriority(Realtime::Priority::MEDIUM);
    leg_controller_node.SetTaskFrequency(freq1); // 50 HZ
    //leg_controller_node.SetCoreAffinity(-1);
    leg_controller_node.SetPortOutput(Controllers::Locomotion::LegController::OutputPort::SERVO_COMMAND,
                                      Realtime::Port::TransportType::INPROC, "inproc", "nomad.controllers.leg.servo_cmd");


    Realtime::Port::Map(leg_controller_node.GetInputPort(Controllers::Locomotion::LegController::InputPort::LEG_COMMAND),
                        primary_controller_node.GetOutputPort(Controllers::FSM::PrimaryControl::OutputPort::LEG_COMMAND));
    leg_controller_node.Start();


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

   //s leg_controller_node.Stop();
}