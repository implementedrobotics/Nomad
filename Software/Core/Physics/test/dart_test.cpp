//#include <dart/dart.hpp>
//#include <dart/gui/gui.hpp>
#include "dart/simulation/simulation.hpp"

using namespace dart::dynamics;
using namespace dart::simulation;

int main(int argc, char *argv[])
{
    // Create an empty Skeleton with the name "pendulum"
    SkeletonPtr pendulum = Skeleton::create("pendulum");

    // Create a world and add the pendulum to the world
    WorldPtr world(new World);
    world->step();
}
