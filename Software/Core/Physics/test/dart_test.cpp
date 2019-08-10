//#include <dart/dart.hpp>
//#include <dart/gui/gui.hpp>
//#include "dart/simulation/simulation.hpp"
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/simulation/World.hpp>

//using namespace dart::dynamics;
//using namespace dart::simulation;

int main(int argc, char *argv[])
{
    // Create a block
    dart::dynamics::SkeletonPtr rigid_skel = dart::dynamics::Skeleton::create("body");
    dart::dynamics::BodyNodePtr rigid_body = rigid_skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(nullptr).second;

    // Set the shape
    double boxWidth = 1.0;
    double boxDepth = 1.0;
    double boxHeight = 0.5;


    dart::dynamics::ShapePtr box_shape(new dart::dynamics::BoxShape(Eigen::Vector3d(boxWidth, boxDepth, boxHeight)));

    //auto box_shape = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(boxWidth, boxDepth, boxHeight));

   rigid_body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(box_shape);



    rigid_body->setMass(2.0);
    //
    // Create an empty Skeleton with the name "pendulum"
    //dart::dynamics::SkeletonPtr pendulum = dart::dynamics::Skeleton::create("pendulum");


    // Create a world and add the pendulum to the world
    dart::simulation::WorldPtr world(new dart::simulation::World);
    world->addSkeleton(rigid_skel);
    std::cout << "Gravity: " << world->getGravity() << std::endl;
    constexpr double END_TIME = 1.0;
    
    while (world->getTime() < END_TIME) {

        rigid_skel->getBodyNode(0)->clearExternalForces();
        rigid_skel->getBodyNode(0)->addExtForce(Eigen::Vector3d::UnitZ() * 2.0*9.81, rigid_skel->getBodyNode(0)->getCOM(), false, true);
        std::cout << world->getTime() << " : ";
        std::cout << "X: " << rigid_skel->getPosition(3) << "\tY: " << rigid_skel->getPosition(4) << "\tZ: " << rigid_skel->getPosition(5) << std::endl;
        std::cout << std::endl;

        world->step(false);
    }

}