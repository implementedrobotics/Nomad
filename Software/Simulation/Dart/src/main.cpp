
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/simulation/World.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include <dart/gui/osg/osg.hpp>
#include <dart/gui/osg/TrackballManipulator.hpp>

#include <vector>

#include <Communications/Messages/double_vec_t.hpp>
//#include <Simulation/Dart/NomadRobot.h>
#include "../include/NomadRobot.h"

// Third Party Includes
#include <zcm/zcm-cpp.hpp>

using namespace dart::dynamics;
using namespace dart::simulation;

WorldPtr g_world;

std::shared_ptr<NomadRobot> g_nomad;

// Robot Class
// Dynamic State Vector
// Floating Base Class

class NomadSimWorldNode : public dart::gui::osg::RealTimeWorldNode
{

public:
  explicit NomadSimWorldNode(WorldPtr world, const std::shared_ptr<NomadRobot> nomad)
      : dart::gui::osg::RealTimeWorldNode(world), nomad_(nomad), world_(world)
  {
    // World gets saved as -> mWorld if ever needed.  Also ->getWorld()
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    //  step_iter = 0;

    // TODO: Pass IP Address as a parameter, in the .model file?
    context_ = std::make_unique<zcm::ZCM>("udpm://239.255.76.67:7667?ttl=0");

    std::cout << "Nomad DART Sim connecting." << std::endl;
    context_->subscribe("nomad.forces", &NomadSimWorldNode::OnMsg, this);
    context_->start();

    // TODO: Publish state back
    pub_context_ = std::make_unique<zcm::ZCM>("udpm://239.255.76.67:7667?ttl=0");

    sequence_num_ = 0;
  }

  void customPreStep()
  {

    // std::cout << leg->getJoint("leg_to_world")->getPosition(0) << std::endl;
    //  if (step_iter < 5)
    //    return;

    // Reset Leg Controller
    // nomad_->ProcessInputs();
    // nomad_->Run(world_->getTimeStep());
    //nomad_->SendOutputs();

    // Post Setup
  }

  void customPostStep()
  {
    // nomad_->UpdateState();
    PublishState();
    step_iter++;
  }

  void OnMsg(const zcm::ReceiveBuffer *rbuf, const std::string &chan, const double_vec_t *msg)
  {
    std::cout << "Received Message on Channel: " << chan << std::endl;
  }

  void PublishState()
  {
    double_vec_t imu_msg;
    imu_msg.length = 10;
    imu_msg.data.resize(imu_msg.length);

    uint64_t time_now = sequence_num_++;

    //
    imu_msg.timestamp = time_now;
    imu_msg.sequence_num = sequence_num_++;
    Eigen::Quaterniond body_orientation = nomad_->GetBodyOrientation();

    imu_msg.data[0] = body_orientation.x();
    imu_msg.data[1] = body_orientation.y();
    imu_msg.data[2] = body_orientation.z();
    imu_msg.data[3] = body_orientation.w();

    Eigen::Vector3d accel = nomad_->GetLinearAcceleration();
    Eigen::Vector3d angular = nomad_->GetAngularAcceleration();
    imu_msg.data[4] = accel[0];
    imu_msg.data[5] = accel[1];
    imu_msg.data[6] = accel[2];

    imu_msg.data[7] = angular[0];
    imu_msg.data[8] = angular[1];
    imu_msg.data[9] = angular[2];

    //std::cout << "Got :" << accel << std::endl;
    //std::cout << "Got Angular :" << angular << std::endl;

    //std::cout << "RPY: " << mat.eulerAngles(0,1,2) << std::endl;

    // Publish
    int rc = pub_context_->publish("nomad.imu", &imu_msg);

    // TODO: Publish Pose
    // TODO: Publish Joint Config
  }

protected:
  std::shared_ptr<NomadRobot> nomad_;
  WorldPtr world_;

  std::unique_ptr<zcm::ZCM> context_;

  std::unique_ptr<zcm::ZCM> pub_context_;

  int step_iter;

  uint64_t sequence_num_;

};

SkeletonPtr CreateGround()
{
  SkeletonPtr ground = Skeleton::create("ground");

  // Give the ground a body
  BodyNodePtr body = ground->createJointAndBodyNodePair<WeldJoint>(nullptr).second;
  double thickness = 0.1;

  ShapePtr groundShape = std::make_shared<BoxShape>(Eigen::Vector3d(4, 4, thickness));
  auto shapeNode = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(groundShape);

  shapeNode->getVisualAspect()->setColor(dart::Color::L    std::cout << "GOT WOrld : " << base_link->getLinearAcceleration() << std::endl;ightGray());

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0, 0, -thickness / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return ground;
}

int main(int argc, char *argv[])
{

  // Create dart simulation world
  g_world = World::create();

  // Update Gravity vector to Z down
  Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  g_world->setGravity(gravity);

  // Create and add ground to world
  SkeletonPtr ground = CreateGround();
  g_world->addSkeleton(ground);

  g_nomad = std::make_shared<NomadRobot>(g_world);

  std::string urdf = std::getenv("NOMAD_RESOURCE_PATH");
  urdf.append("/Robot/Nomad.urdf");

  g_nomad->LoadFromURDF(urdf);
  g_nomad->SetInitialPose();

  // Create osg world node
  ::osg::ref_ptr<NomadSimWorldNode> node = new NomadSimWorldNode(g_world, g_nomad);

  // Create osg Viewer
  dart::gui::osg::Viewer viewer = dart::gui::osg::Viewer();
  viewer.addWorldNode(node);

  // Default with Headlights
  viewer.switchHeadlights(false);

  //Add Grid Attachment
  ::osg::ref_ptr<dart::gui::osg::GridVisual> grid = new dart::gui::osg::GridVisual();

  viewer.addAttachment(grid);

  // Print out instructions for the viewer
  std::cout << viewer.getInstructions() << std::endl;

  viewer.setUpViewInWindow(0, 0, 1280, 1024);
  viewer.getCameraManipulator()->setHomePosition(::osg::Vec3(0.0f, -5.0f, 0.0f),
                                                 ::osg::Vec3(0.0f, 0.0f, 0.0f),
                                                 ::osg::Vec3(0.0f, 0.707f, 0.707f), false);

  viewer.setCameraManipulator(viewer.getCameraManipulator());

  viewer.run();

  return 0;
}
