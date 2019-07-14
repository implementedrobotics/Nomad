#include <Controllers/RealTimeTask.hpp>
#include <Controllers/StateEstimator.hpp>
#include <unistd.h>

int main(int argc, char *argv[])
{
    // Create Task Manager Instance Singleton.  Must make sure this is done before any thread tries to access.  And thus tries to allocate memory inside the thread heap.
    Controllers::RealTimeControl::RealTimeTaskManager::Instance();

    Controllers::Estimators::StateEstimator estimator_node("Estimator_Task_1");
    estimator_node.SetStackSize(100000);
    estimator_node.SetTaskPriority(Controllers::RealTimeControl::Priority::MEDIUM);
    estimator_node.SetTaskFrequency(2); // 100 HZ
    estimator_node.SetCoreAffinity(1);
    estimator_node.Start();


    Controllers::Estimators::StateEstimator *estimator_node2 = new Controllers::Estimators::StateEstimator("Estimator_Task_2");
    estimator_node2->SetStackSize(100000);
    estimator_node2->SetTaskPriority(Controllers::RealTimeControl::Priority::MEDIUM);
    estimator_node2->SetTaskFrequency(1); // 100 HZ
    estimator_node2->SetCoreAffinity(2);
    estimator_node2->Start();

    usleep(1000000);

    delete estimator_node2;

    Controllers::RealTimeControl::RealTimeTaskManager::Instance()->PrintActiveTasks();


    while (1)
    {
        printf("IDLING\n");
        usleep(1000000);
        //estimator_node.Stop();
    }
}