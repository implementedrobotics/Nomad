#include <Realtime/RealTimeTask.hpp>
#include <Communications/Port.hpp>
#include <Systems/BlockDiagram.hpp>
#include <Systems/SystemBlock.hpp>
#include <Common/Time.hpp>
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

    // if (!Realtime::RealTimeTaskManager::EnableRTMemory(500 * 1024 * 1024)) // 500MB
    // {
    //     // exit(-2);
    //     std::cout << "Error configuring Realtime Memory Requiremets!  Realtime Execution NOT guaranteed." << std::endl;
    // }


    // Create Block Diagram
    BlockDiagram diagram("Test", 0.002); //10hz
    diagram.SetStackSize(1024 * 1024);
    diagram.SetTaskPriority(Realtime::Priority::HIGH);
    diagram.SetCoreAffinity(2);

    Eigen::Vector3d vec = Eigen::Vector3d::Ones();
    std::shared_ptr<ConstantBlock> cb = std::make_shared<ConstantBlock>(vec);
    cb->SetPortOutput(0, Communications::Port::TransportType::NATIVE, "native", "system.A");
    diagram.AddSystem(cb);

    std::shared_ptr<ConstantBlock> cb2 = std::make_shared<ConstantBlock>(vec*2);
    cb2->SetPortOutput(0, Communications::Port::TransportType::NATIVE, "native", "system.B");
    diagram.AddSystem(cb2);

    std::shared_ptr<AddBlock> ab = std::make_shared<AddBlock>();
    ab->SetPortOutput(0, Communications::Port::TransportType::NATIVE, "native", "system.C");
    diagram.AddSystem(ab);

    ab->AddInput(AddBlock::ADD, 3);
    ab->AddInput(AddBlock::MINUS, 3);
    ab->AddInput(AddBlock::ADD, 3);

    diagram.Connect(cb->GetOutputPort(0), ab->GetInputPort(0));
    diagram.Connect(cb2->GetOutputPort(0), ab->GetInputPort(1));
    diagram.Connect(cb2->GetOutputPort(0), ab->GetInputPort(2));

    // diagram.Connect(ab->GetOutputPort(0), ab2->GetInputPort(0));
    // diagram.Connect(cb2->GetOutputPort(0), ab2->GetInputPort(1));

    // Start Run
    diagram.Start();

    // Print Threads
    Realtime::RealTimeTaskManager::Instance()->PrintActiveTasks();

    // Start Inproc Context Process Thread
    Communications::PortManager::Instance()->GetInprocContext()->start();

    getchar();

    diagram.Stop();
}