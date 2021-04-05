#include <CAN/PCANDevice.h>
#include <CAN/Registers.h>
#include <CAN/RealTimeTask.hpp>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <cmath>

#include <NomadBLDC/NomadBLDC.h>
#include <nlohmann/json.hpp>
#include <zmq.hpp>

// Dart
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/dynamics/BallJoint.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/simulation/World.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include <dart/gui/osg/osg.hpp>


#define DEVICE "/dev/pcanusbfd32"

PCANDevice can;

float q1_ref = 0.6f;
float q2_ref = -0.7f;
float frequency = 1.0f;
float diameter = 0.05f;
double world_time = 0.0f;
    float home = 0.0308f;
    float home2 = -0.312458f;
Eigen::Vector2d foot_pos_des_;
Eigen::Vector2d foot_vel_des_;


class LegTest : public Realtime::RealTimeTaskNode
{

public:
    // Block Diagram Class For Systems Task Node
    // name = Task Name
    // T_s = Sample Time (-1 for inherit)
    LegTest(const std::string &name, const double T_s = -1);

    virtual void Exit();

protected:
    // Overriden Run Function
    virtual void Run();

    // Pre-Run Setup Routine.  Setup any one time initialization here.
    virtual void Setup();

    // TODO: Load Dart URDF
    void SetupDART();

    double time_;

    NomadBLDC *servo1;
    NomadBLDC *servo2;

    zmq::context_t ctx;
    zmq::socket_t *publisher;

    // Dart Pointers
    dart::dynamics::SkeletonPtr robot_;

    dart::dynamics::DegreeOfFreedomPtr hfe;
    dart::dynamics::DegreeOfFreedomPtr kfe;

    dart::dynamics::BodyNodePtr hip_body_;
    dart::dynamics::BodyNodePtr foot_body_;

    Eigen::Vector2d foot_pos_;
    Eigen::Vector2d foot_vel_;

    Eigen::Vector2d q_;
    Eigen::Vector2d qd_;
    Eigen::Vector2d qdd_;

    Eigen::MatrixXd J_; // Leg Jacobian

    Eigen::Matrix2d k_P_cartesian_;
    Eigen::Matrix2d k_D_cartesian_;
};

LegTest::LegTest(const std::string &name, const double T_s) : Realtime::RealTimeTaskNode(name, T_s, Realtime::Priority::HIGH, -1, PTHREAD_STACK_MIN), time_(0.0)
{
}
void LegTest::SetupDART()
{
    // Load URDF
    dart::utils::DartLoader loader;
    std::string urdf = std::getenv("NOMAD_RESOURCE_PATH");
    urdf.append("/LegTest/Leg.urdf");

    robot_= loader.parseSkeleton(urdf);
    robot_->getDof("j_hfe")->setPosition(q1_ref);
    robot_->getDof("j_kfe")->setPosition(q2_ref);

    //nomad->getJoint("j_kfe")->setPositionLimitEnforced(true);
    //robot_->getDof("j_kfe")->setDampingCoefficient(1.5f);
    //robot_->getDof("j_hfe")->setDampingCoefficient(1.5f);
    //nomad->getDof("j_kfe")->setPositionLimits(0.0f, 2.0);

    robot_->getDof("slider")->setPositionLimits(-0.0, 0.0);
    robot_->getJoint("slider")->setPositionLimitEnforced(true);


    hfe = robot_->getDof("j_hfe");
    kfe = robot_->getDof("j_kfe");

    hip_body_ = robot_->getBodyNode("HFE_Actuator1");
    foot_body_ = robot_->getBodyNode("Foot1");

    // Fix our base link (the stand)
    robot_->getRootBodyNode()->moveTo<dart::dynamics::WeldJoint>(nullptr);

    robot_->computeForwardKinematics();
    robot_->computeForwardDynamics();

    // Set Friction
    //std::cout << nomad->getBodyNode("foot")->getFrictionCoeff() << std::endl;
    //nomad->getBodyNode("foot")->setFrictionCoeff(10.8);
    //return nomad;
}
void LegTest::Run()
{
    auto start_time = std::chrono::high_resolution_clock::now();
    time_ += dt_actual_;

    q_[0] = servo1->GetPosition() + q1_ref;
    qd_[0] = servo1->GetVelocity();
    qdd_[0]= servo1->GetTorque();

    q_[1] = servo2->GetPosition() + q2_ref;
    qd_[1] = servo2->GetVelocity();
    qdd_[1]= servo2->GetTorque();


    robot_->setPosition(0, 0);
    robot_->setVelocity(0, 0);
    robot_->setForce(0, 0);

    robot_->setPosition(1, q_[0]);
    robot_->setVelocity(1, qd_[0]);
    robot_->setForce(1, qdd_[0]);

    robot_->setPosition(2, q_[1]);
    robot_->setVelocity(2, qd_[1]);
    robot_->setForce(2, qdd_[1]);

    robot_->computeForwardKinematics();
    robot_->computeForwardDynamics();

    J_ = robot_->getLinearJacobian(foot_body_, hip_body_);
    J_ = J_.bottomRows(2);
    
    foot_pos_ = foot_body_->getTransform(hip_body_).translation().tail(2);

    foot_vel_ = (J_ * robot_->getVelocities()).tail(2);

    foot_pos_des_[0] = home + diameter * std::cos(2 * M_PI * frequency * time_);
    foot_pos_des_[1] = home2 + diameter * std::sin(2 * M_PI * frequency * time_);


   // std::cout << "FP: " << foot_pos_des_[0] << " , " << foot_pos_des_[1] << std::endl;
    Eigen::Vector2d force_output = Eigen::Vector2d::Zero();
    Eigen::Vector2d tau_output;

    Eigen::Vector2d k_P(160, 160);
    Eigen::Vector2d k_D(2, 2);

    k_P_cartesian_ = k_P.asDiagonal();
    k_D_cartesian_ = k_D.asDiagonal();



    //tau_output = tau_feedforward_;
    //force_output = force_feedforward_;
    force_output += k_P_cartesian_ * (foot_pos_des_ - foot_pos_);
    force_output += k_D_cartesian_ * (foot_vel_des_ - foot_vel_);

    //std::cout << "FORCE: " << std::endl << force_output << std::endl;
    //std::cout << "ANGLE: " << std::endl << q_[0] << std::endl;    

    Eigen::VectorXd torque = J_.transpose() * force_output;


    //std::cout << " TAU: " << std::endl << torque << std::endl;
  //  std::cout << "Qd1: " << q_[0] << std::endl;
  //  std::cout << "Qd2: " << q_[1] << std::endl;




   // float freq = 2.0f; // 2 hz

  //  float sin_val = std::sin(2 * M_PI * freq * time_);
   // float cos_val = std::cos(2 * M_PI * freq * time_);


   // float pos = 1.1f * sin_val;
    servo1->ClosedLoopTorqueCommand(0.0f, 0.0f, 0.0f, 0.0f, torque[1]);
    servo2->ClosedLoopTorqueCommand(0.0f, 0.0f, 0.0f, 0.0f, torque[2]);

    // Update DART Here
    nlohmann::json test = {
        {"timestamp", time_},
        {"motor",
            {{"Foot_Pos_1", foot_pos_[0]},
            {"tau_ref_1", torque[1]},
            {"Foot_Des_1",foot_pos_des_[0]},
            {"tau_1", qdd_[0]},
            {"Foot_Pos_2", foot_pos_[1]}, 
            {"tau_ref_2", torque[2]},
            {"Foot_Des_2", foot_pos_des_[1]},
            {"tau_2", qdd_[1]},
            {"Q_1", q_[0]},
            {"Q_2", q_[1]},}
        }
    };

    auto data = nlohmann::json::to_msgpack(test);
    zmq::message_t message(data.size());
    memcpy(message.data(), &data[0], data.size());
    publisher->send(message, zmq::send_flags::dontwait);

    auto time_now = std::chrono::high_resolution_clock::now();
    auto total_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
}

void LegTest::Setup()
{
    // TODO: We need some sort of exceptions here if setup fails
    CANDevice::Config_t config;
    config.bitrate = 1e6; //1mbps
    config.d_bitrate = 5e6; //2mbps
    config.sample_point = 0.80f; //87.5% 
    config.d_sample_point = 0.625f; //60%
    config.clock_freq = 80e6; // 80mhz // Read from driver?  
    config.mode_fd = 1; // FD Mode

    if(!can.Open(DEVICE, config, true))
    {
        std::cout << "Unable to open CAN Device" << std::endl;
        return;
    }

    // Setup Filters
    can.ClearFilters(); // Clear Existing/Reset.  Filters are saved on the device hardware.  Must make sure to clear
    can.AddFilter(1, 2); // Only Listen to messages on id 1.  

    servo1 = new NomadBLDC(1, 0x10, &can);
    servo1->SetName("INPUT");
    if(!servo1->Connect())
    {
        std::cout << "[ERROR]: Unable to connect to Nomad Servo!" << std::endl;
        exit(1);
    }

    std::cout << "Nomad Servo: " << "[" << servo1->GetName() << "] : " << servo1->GetServoId() << " Connected!" << std::endl;

    servo2 = new NomadBLDC(2, 0x11, &can);
    servo2->SetName("INPUT2");
    if(!servo2->Connect())
    {
        std::cout << "[ERROR]: Unable to connect to Nomad Servo!" << std::endl;
        exit(1);
    }

    std::cout << "Nomad Servo: " << "[" << servo2->GetName() << "] : " << servo2->GetServoId() << " Connected!" << std::endl;

    // Setup ZMQ
    publisher = new zmq::socket_t(ctx, ZMQ_PUB);
    std::string transport("tcp://*:9872");
    publisher->bind(transport);

    std::cout << "Please Zero Actuator.  Enter to Continue." << std::endl;
    getchar();

    // Load DART
    SetupDART();

    std::cout << "DART LOADED!" << std::endl;

    servo1->ZeroOutput();
    usleep(30000);
    servo2->ZeroOutput();
    usleep(30000);

    // Start Motor Control Mode
    usleep(1000000);
    servo1->SetControlMode(PD_MODE);
    servo2->SetControlMode(PD_MODE);


    servo1->ClosedLoopTorqueCommand(0.0, 0.0f, 0.0f, 0.0f, 0.0f);
    servo2->ClosedLoopTorqueCommand(0.0, 0.0f, 0.0f, 0.0f, 0.0f);

    usleep(30000);

    // Get Initial Values
    q_[0] = servo1->GetPosition()  + q1_ref;
    qd_[0] = servo1->GetVelocity();
    qdd_[0]= servo1->GetTorque();

    q_[1] = servo2->GetPosition()  + q2_ref;
    qd_[1] = servo2->GetVelocity();
    qdd_[1]= servo2->GetTorque();

    robot_->setPosition(0, 0);
    robot_->setVelocity(0, 0);
    robot_->setForce(0, 0);

    robot_->setPosition(1, q_[0]);
    robot_->setVelocity(1, qd_[0]);
    robot_->setForce(1, qdd_[0]);

    robot_->setPosition(2, q_[1]);
    robot_->setVelocity(2, qd_[1]);
    robot_->setForce(2, qdd_[1]);

    robot_->computeForwardKinematics();
    robot_->computeForwardDynamics();

    J_ = robot_->getLinearJacobian(foot_body_, hip_body_);
    J_ = J_.bottomRows(2);

    foot_pos_ = foot_body_->getTransform(hip_body_).translation().tail(2);
    foot_vel_ = (J_ * robot_->getVelocities()).tail(2);
    foot_pos_des_ = foot_pos_;

    std::cout << "Foot Pos Here: " << foot_pos_[0] << ", " << foot_pos_[1] << std::endl;



    std::cout << "Qd: " << q_[0] << std::endl;
}

void LegTest::Exit()
{
    std::cout << "Exiting!" << std::endl;

    // Set back to idle.  In theory when no commands are sent it should auto back to idle or edamp?
    servo1->SetControlMode(IDLE_MODE);
    servo2->SetControlMode(IDLE_MODE);
   // servo2->SetControlMode(1);
}


int main(int argc, char *argv[])
{
    Realtime::RealTimeTaskManager::Instance();

    if (!Realtime::RealTimeTaskManager::EnableRTMemory(500 * 1024 * 1024)) // 500MB
    {
        std::cout << "Error configuring Realtime Memory Requiremets!  Realtime Execution NOT guaranteed." << std::endl;
    }
    else
    {
        std::cout << "Real Time Memory Enabled!" << std::endl;
    }

    LegTest pj_Node("Test", 1/500.0f); //500hz
    pj_Node.SetStackSize(1024 * 1024);
    pj_Node.SetTaskPriority(Realtime::Priority::HIGHEST);
    pj_Node.SetCoreAffinity(2);
    pj_Node.Start();

    char input;
    std::cin >> input;
    if(input == 'q')
    {
        std::cout << "Got Q:" << std::endl;
    }

    pj_Node.Exit();
    pj_Node.Stop();

    //getchar();
}
