#include <Controllers/StateEstimator.hpp>
#include <unistd.h>

int main(int argc, char *argv[])
{
    Controllers::Estimators::StateEstimator estimator_node("Estimator_Task");
    estimator_node.SetStackSize(100000);
    estimator_node.SetTaskPriority(Controllers::RealTimeControl::Priority::MEDIUM);
    estimator_node.SetTaskFrequency(100); // 100 HZ
    estimator_node.SetCoreAffinity(1);
    estimator_node.Start();


    Controllers::Estimators::StateEstimator estimator_node2("Estimator_Task");
    estimator_node2.SetStackSize(100000);
    estimator_node2.SetTaskPriority(Controllers::RealTimeControl::Priority::MEDIUM);
    estimator_node2.SetTaskFrequency(1); // 100 HZ
    estimator_node2.SetCoreAffinity(1);
    estimator_node2.Start();


    while (1)
    {
        printf("IDLING!\n");
        usleep(1000000);
        //estimator_node.Stop();
    }
}