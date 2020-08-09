#include <Eigen/Dense>
//#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/simulation/World.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/gui/osg/TrackballManipulator.hpp>

#include <vector>

#include <Communications/Messages/double_vec_t.hpp>
#include <Nomad/MessageTypes/imu_data_t.hpp>
#include <Nomad/MessageTypes/joint_control_cmd_t.hpp>
#include <Nomad/MessageTypes/joint_state_t.hpp>
#include <Nomad/MessageTypes/com_state_t.hpp>
//#include <Simulation/Dart/NomadRobot.h>
#include "../include/NomadRobot.h"

// Third Party Includes
#include <zcm/zcm-cpp.hpp>

using namespace dart::dynamics;
using namespace dart::simulation;

WorldPtr g_world;

std::shared_ptr<NomadRobot> g_nomad;

Eigen::VectorXd joint_torques = Eigen::VectorXd::Zero(18);

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
    context_->subscribe("nomad.sim.joint_cmd", &NomadSimWorldNode::OnJointControlMsg, this);
    context_->start();

    // TODO: Publish state back
    pub_context_ = std::make_unique<zcm::ZCM>("udpm://239.255.76.67:7667?ttl=0");

    sequence_num_ = 0;
  }

  void customPreStep()
  {

     // std::cout << "Step!" << std::endl;
    // std::cout << leg->getJoint("leg_to_world")->getPosition(0) << std::endl;
    //  if (step_iter < 5)
    //    return;

    // Reset Leg Controller
    // nomad_->ProcessInputs();
    // nomad_->Run(world_->getTimeStep());
    //nomad_->SendOutputs();
    std::cout << "SETTING" << std::endl;
    nomad_->Skeleton()->setForces(joint_torques);

    // Post Setup
  }

  void customPostStep()
  {
    // nomad_->UpdateState();
    PublishState();

    //joint_torques = Eigen::VectorXd::Zero(18);
    step_iter++;
  }

  void OnJointControlMsg(const zcm::ReceiveBuffer *rbuf, const std::string &chan, const joint_control_cmd_t *msg)
  {
    //std::cout << "Received Message on Channel: " << chan << std::endl;
    std::cout << "Got :" << msg->sequence_num << std::endl;

    Eigen::VectorXd tau_cmd(12);// = Eigen::VectorXd::Zero(12);

    tau_cmd[0] = msg->tau_ff[0];
    tau_cmd[1] = msg->tau_ff[1];
    tau_cmd[2] = msg->tau_ff[2];
    tau_cmd[3] = msg->tau_ff[3];
    tau_cmd[4] = msg->tau_ff[4];
    tau_cmd[5] = msg->tau_ff[5];
    tau_cmd[6] = msg->tau_ff[6];
    tau_cmd[7] = msg->tau_ff[7];
    tau_cmd[8] = msg->tau_ff[8];
    tau_cmd[9] = msg->tau_ff[9];
    tau_cmd[10] = msg->tau_ff[10];
    tau_cmd[11] = msg->tau_ff[11];

    //std::cout << " GOT: " << tau_cmd << std::endl;

    //Eigen::VectorXd tau_output = Eigen::Map<Eigen::VectorXd>(msg->tau_ff, 12);
    joint_torques.tail(12) = tau_cmd;
  }

  void PublishState()
  {

    imu_data_t imu_data;
    com_state_t com_state;
    joint_state_t joint_state;

    uint64_t time_now = sequence_num_;

    // Get Relevant State Data
    Eigen::Quaterniond body_orientation = nomad_->GetBodyOrientation();
    Eigen::Vector3d accel = nomad_->GetLinearAcceleration();
    Eigen::Vector3d angular = nomad_->GetAngularVelocity();

    // Cheater Orientation
    com_state.timestamp = time_now;
    com_state.sequence_num = sequence_num_;
    com_state.theta[0] = nomad_->Skeleton()->getDof(0)->getPosition();
    com_state.theta[1] = nomad_->Skeleton()->getDof(1)->getPosition();
    com_state.theta[2] = nomad_->Skeleton()->getDof(2)->getPosition();
    com_state.pos[0] = nomad_->Skeleton()->getDof(3)->getPosition();
    com_state.pos[1] = nomad_->Skeleton()->getDof(4)->getPosition();
    com_state.pos[2] = nomad_->Skeleton()->getDof(5)->getPosition();
    com_state.vel[0] = nomad_->Skeleton()->getDof(3)->getVelocity();
    com_state.vel[1] = nomad_->Skeleton()->getDof(4)->getVelocity();
    com_state.vel[2] = nomad_->Skeleton()->getDof(5)->getVelocity();

    com_state.orientation[0] = body_orientation.x();
    com_state.orientation[1] = body_orientation.y();
    com_state.orientation[2] = body_orientation.z();
    com_state.orientation[3] = body_orientation.w();

    com_state.omega[0] = angular[0];
    com_state.omega[1] = angular[1];
    com_state.omega[2] = angular[2];

    //
    imu_data.timestamp = time_now;
    imu_data.sequence_num = sequence_num_;
    

    // Orientation
    imu_data.orientation[0] = body_orientation.x();
    imu_data.orientation[1] = body_orientation.y();
    imu_data.orientation[2] = body_orientation.z();
    imu_data.orientation[3] = body_orientation.w();

    // Accelerometer
    imu_data.accel[0] = accel[0];
    imu_data.accel[1] = accel[1];
    imu_data.accel[2] = accel[2];

    // Gyro
    imu_data.omega[0] = angular[0];
    imu_data.omega[1] = angular[1];
    imu_data.omega[2] = angular[2];

    Eigen::VectorXd joint_pos = nomad_->Skeleton()->getPositions();
    Eigen::VectorXd joint_vel = nomad_->Skeleton()->getVelocities();
    Eigen::VectorXd joint_tau = nomad_->Skeleton()->getForces();

    joint_state.timestamp = time_now;
    joint_state.sequence_num = sequence_num_;

    joint_state.q[0] = joint_pos[6];
    joint_state.q[1] = joint_pos[7];
    joint_state.q[2] = joint_pos[8];
    joint_state.q[3] = joint_pos[9];
    joint_state.q[4] = joint_pos[10];
    joint_state.q[5] = joint_pos[11];
    joint_state.q[6] = joint_pos[12];
    joint_state.q[7] = joint_pos[13];
    joint_state.q[8] = joint_pos[14];
    joint_state.q[9] = joint_pos[15];
    joint_state.q[10] = joint_pos[16];
    joint_state.q[11] = joint_pos[17];

    joint_state.q_dot[0] = joint_vel[6];
    joint_state.q_dot[1] = joint_vel[7];
    joint_state.q_dot[2] = joint_vel[8];
    joint_state.q_dot[3] = joint_vel[9];
    joint_state.q_dot[4] = joint_vel[10];
    joint_state.q_dot[5] = joint_vel[11];
    joint_state.q_dot[6] = joint_vel[12];
    joint_state.q_dot[7] = joint_vel[13];
    joint_state.q_dot[8] = joint_vel[14];
    joint_state.q_dot[9] = joint_vel[15];
    joint_state.q_dot[10] = joint_vel[16];
    joint_state.q_dot[11] = joint_vel[17];

    joint_state.tau[0] = joint_tau[6];
    joint_state.tau[1] = joint_tau[7];
    joint_state.tau[2] = joint_tau[8];
    joint_state.tau[3] = joint_tau[9];
    joint_state.tau[4] = joint_tau[10];
    joint_state.tau[5] = joint_tau[11];
    joint_state.tau[6] = joint_tau[12];
    joint_state.tau[7] = joint_tau[13];
    joint_state.tau[8] = joint_tau[14];
    joint_state.tau[9] = joint_tau[15];
    joint_state.tau[10] = joint_tau[16];
    joint_state.tau[11] = joint_tau[17];

    sequence_num_++;
    //std::cout << "Got :" << accel << std::endl;
    //std::cout << "Got Angular :" << angular << std::endl;

    //std::cout << "RPY: " << mat.eulerAngles(0,1,2) << std::endl;

    // Publish
    int rc = pub_context_->publish("nomad.sim.imu_state", &imu_data);

    rc = pub_context_->publish("nomad.sim.joint_state", &joint_state);

    rc = pub_context_->publish("nomad.sim.com_state", &com_state);
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

  shapeNode->getVisualAspect()->setColor(dart::Color::LightGray());

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
  //Eigen::Vector3d gravity(0.0, 0.0, 0.0);
  g_world->setGravity(gravity);

  // Create and add ground to world
  SkeletonPtr ground = CreateGround();
  g_world->addSkeleton(ground);

  // Update Collision
  //g_world->getConstraintSolver()->setCollisionDetector(dart::collision::BulletCollisionDetector::create());

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
  viewer.simulate(true);
  viewer.run();

  return 0;
}
