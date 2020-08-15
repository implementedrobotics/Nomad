#include <Realtime/RealTimeTask.hpp>
#include <Communications/Port.hpp>
#include <Controllers/BlockDiagram.hpp>
#include <Controllers/SystemBlock.hpp>
#include <memory>

#include <unistd.h>
#include <sys/mman.h>

using namespace Controllers::Systems;

int main(int argc, char *argv[])
{
    // Create Manager Class Instance Singleton.
    // Must make sure this is done before any thread tries to access.
    // And thus tries to allocate memory inside the thread heap.
    Realtime::RealTimeTaskManager::Instance();
    Communications::PortManager::Instance();

    // Create Block Diagram
    BlockDiagram diagram("Test", 1); //10hz
    diagram.SetStackSize(1024 * 1024);
    diagram.SetTaskPriority(Realtime::Priority::MEDIUM);
    diagram.SetCoreAffinity(2);
    

    Eigen::Vector3d vec = Eigen::Vector3d::Ones();
    //std::shared_ptr<ConstantBlock> cb = std::make_shared<ConstantBlock>(vec);
    //cb->SetPortOutput(0, Communications::Port::TransportType::IPC, "ipc", "system.somehash");

    //diagram.AddSystem(cb);

    std::shared_ptr<AddBlock> ab = std::make_shared<AddBlock>();
    ab->SetPortOutput(0, Communications::Port::TransportType::IPC, "ipc", "system.somehash");
    diagram.AddSystem(ab);


    // Start Run
    diagram.Start();



    // Print Threads
    Realtime::RealTimeTaskManager::Instance()->PrintActiveTasks();

    // Start Inproc Context Process Thread
    Communications::PortManager::Instance()->GetInprocContext()->start();

    getchar();

    diagram.Stop();
}