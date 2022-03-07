// #include <CAN/PCANDevice.h>
// #include <NomadBLDC/Registers.h>
// #include <NomadBLDC/RealTimeTask.hpp>
// #include <string.h>
// #include <iostream>
// #include <unistd.h>
// #include <cmath>

// #include <NomadBLDC/NomadBLDC.h>
// #include <nlohmann/json.hpp>
// #include <zmq.hpp>

// // Dart
// #include <dart/dynamics/Skeleton.hpp>
// #include <dart/dynamics/FreeJoint.hpp>
// #include <dart/dynamics/WeldJoint.hpp>
// #include <dart/dynamics/BallJoint.hpp>
// #include <dart/dynamics/BoxShape.hpp>
// #include <dart/simulation/World.hpp>
// #include <dart/utils/urdf/DartLoader.hpp>

// #include <dart/gui/osg/osg.hpp>


// #define DEVICE "/dev/pcanusbfd32"

// PCANDevice can;

// double q1_ref = 0.0f;
// double q2_ref = 0.0f;
// double frequency = 1.0f;
// double diameter = 0.05f;

// double home = 0.0308f;
// double home2 = -0.312458f;

// Eigen::Vector2d foot_pos_des_;
// Eigen::Vector2d foot_vel_des_;

// Eigen::Vector2d k_P(2500, 2500);
// Eigen::Vector2d k_D(5, 5);


// typedef enum
// {
//     Zero = 0,
//     Homing = 1,
//     PreStand = 2,
//     Stand = 3,
//     Crouch = 4
// } FSMState_e;


// FSMState_e current_state_ = Zero;
// int debounce = 0;

// bool AlmostEquals(double A, double B, double epsilon = 0.005f)
// {
//     return (fabs(A - B) < epsilon);
// }

// #include <Eigen/Dense>

// // Project Include Files
// namespace Common
// {
//     class CubicPolynomialTrajectory
//     {

//     public:
//         CubicPolynomialTrajectory(double q_f, double t_f);
//         CubicPolynomialTrajectory(double q_0, double q_f, double v_0, double v_f, double t_0, double t_f);
//         CubicPolynomialTrajectory(); // Empty Trajectory

//         void Generate(double q_f, double t_f);
//         void Generate(double q_0, double q_f, double v_0, double v_f, double t_0, double t_f);

//         // TODO: Check for valid t between 0<->t_f
//         double Position(double t);
//         double Velocity(double t);
//         double Acceleration(double t);

//     protected:
//         void ComputeCoeffs();

//         Eigen::Vector4d a_; // Coefficients

//         double q_0_;
//         double v_0_;
//         double t_0_;

//         double q_f_;
//         double v_f_;
//         double t_f_;
//     };
// } // namespace Common

// namespace Common
// {
//     CubicPolynomialTrajectory::CubicPolynomialTrajectory(double q_f, double t_f)
//         : q_0_(0.0), q_f_(q_f), v_0_(0.0), v_f_(0.0), t_0_(0.0), t_f_(t_f)
//     {
//         // Compute Coefficients
//         ComputeCoeffs();
//     }
//     CubicPolynomialTrajectory::CubicPolynomialTrajectory(double q_0, double q_f, double v_0, double v_f, double t_0, double t_f)
//         : q_0_(q_0), q_f_(q_f), v_0_(v_0), v_f_(v_f), t_0_(t_0), t_f_(t_f)
//     {
//         // Compute Coefficients
//         ComputeCoeffs();
//     }

//     CubicPolynomialTrajectory::CubicPolynomialTrajectory() // Empty Trajectory
//         : q_0_(0.0), q_f_(0.0), v_0_(0.0), v_f_(0.0), t_0_(0.0), t_f_(0.0)
//     {
//         // Compute Coefficients
//         ComputeCoeffs();
//     }

//     void CubicPolynomialTrajectory::Generate(double q_f, double t_f)
//     {
//         q_0_ = 0.0;
//         q_f_ = q_f;
//         v_0_ = 0.0;
//         v_f_ = 0.0;
//         t_0_ = 0.0;
//         t_f_ = t_f;

//         ComputeCoeffs();
//     }

//     void CubicPolynomialTrajectory::Generate(double q_0, double q_f, double v_0, double v_f, double t_0, double t_f)
//     {
//         q_0_ = q_0;
//         q_f_ = q_f;
//         v_0_ = v_0;
//         v_f_ = v_f;
//         t_0_ = t_0;
//         t_f_ = t_f;

//         ComputeCoeffs();
//     }

//     // TODO: Check for valid t between 0<->t_f
//     double CubicPolynomialTrajectory::Position(double t)
//     {
//         // Trim Position
//         // TODO: Function for this trimming/mapping
//         double t_eval = std::min(t, t_f_);
//         t_eval = std::max(t_eval, t_0_);
//         return a_(0) + a_(1) * t_eval + a_(2) * t_eval * t_eval + a_(3) * t_eval * t_eval * t_eval;
//     }
//     double CubicPolynomialTrajectory::Velocity(double t)
//     {
//         // Trim Position
//         // TODO: Function for this trimming/mapping
//         double t_eval = std::min(t, t_f_);
//         t_eval = std::max(t_eval, t_0_);
//         return a_(1) + 2 * a_(2) * t_eval + 3 * a_(3) * t_eval * t_eval;
//     }
//     double CubicPolynomialTrajectory::Acceleration(double t)
//     {
//         // Trim Position
//         // TODO: Function for this trimming/mapping
//         double t_eval = std::min(t, t_f_);
//         t_eval = std::max(t_eval, t_0_);
//         return 2 * a_(2) + 6 * a_(3) * t_eval;
//     }

//     void CubicPolynomialTrajectory::ComputeCoeffs()
//     {

//         Eigen::Matrix4d C; // Constraints
//         C << 1, t_0_, t_0_ * t_0_, t_0_ * t_0_ * t_0_,
//             0, 1, 2 * t_0_, 3 * t_0_ * t_0_,
//             1, t_f_, t_f_ * t_f_, t_f_ * t_f_ * t_f_,
//             0, 1, 2 * t_f_, 3 * t_f_ * t_f_;

//         Eigen::Vector4d b;
//         b << q_0_, v_0_, q_f_, v_f_;

//         // Solve for Coefficients
//         a_ = C.lu().solve(b);
//     }
// } // namespace Common



//  Common::CubicPolynomialTrajectory com_traj_;


// class LegTest : public Realtime::RealTimeTaskNode
// {

// public:
//     // Block Diagram Class For Systems Task Node
//     // name = Task Name
//     // T_s = Sample Time (-1 for inherit)
//     LegTest(const std::string &name, const double T_s = -1);
//     virtual void Exit();

// protected:
//     // Overriden Run Function
//     virtual void Run();

//     // Pre-Run Setup Routine.  Setup any one time initialization here.
//     virtual void Setup();

//     // TODO: Load Dart URDF
//     void SetupDART();

//     // Time
//     double time_;

//     // NOMAD Servos
//     NomadBLDC *servo1;
//     NomadBLDC *servo2;

//     // Zero MQ Context for PlotJuggler
//     zmq::context_t ctx;
//     zmq::socket_t *publisher;

//     // DART Objects
//     dart::dynamics::SkeletonPtr robot_;

//     dart::dynamics::DegreeOfFreedomPtr hfe;
//     dart::dynamics::DegreeOfFreedomPtr kfe;

//     dart::dynamics::BodyNodePtr hip_body_;
//     dart::dynamics::BodyNodePtr foot_body_;

//     Eigen::Vector2d foot_pos_;
//     Eigen::Vector2d foot_vel_;

//     Eigen::Vector2d q_;
//     Eigen::Vector2d qd_;
//     Eigen::Vector2d qdd_;

//     Eigen::MatrixXd J_; // Leg Jacobian

//     Eigen::Matrix2d k_P_cartesian_; // Cartesian P Gains
//     Eigen::Matrix2d k_D_cartesian_; // Cartesian D Gains

//     Eigen::Vector2d force_output = Eigen::Vector2d::Zero();

//     Eigen::VectorXd torque_output;
// };

// LegTest::LegTest(const std::string &name, const double T_s) : Realtime::RealTimeTaskNode(name, T_s, Realtime::Priority::HIGH, -1, PTHREAD_STACK_MIN), time_(0.0)
// {
// }

// void LegTest::SetupDART()
// {
//     // Load URDF
//     dart::utils::DartLoader loader;
//     std::string urdf = std::getenv("NOMAD_RESOURCE_PATH");
//     urdf.append("/LegTest/Leg.urdf");

//     robot_= loader.parseSkeleton(urdf);
//     robot_->getDof("j_hfe")->setPosition(0.0f);
//     robot_->getDof("j_kfe")->setPosition(0.0f);

//     //nomad->getJoint("j_kfe")->setPositionLimitEnforced(true);
//     //robot_->getDof("j_kfe")->setDampingCoefficient(1.5f);
//     //robot_->getDof("j_hfe")->setDampingCoefficient(1.5f);
//     //nomad->getDof("j_kfe")->setPositionLimits(0.0f, 2.0);

//     robot_->getDof("slider")->setPositionLimits(-0.0, 0.0);
//     robot_->getJoint("slider")->setPositionLimitEnforced(true);

//     hfe = robot_->getDof("j_hfe");
//     kfe = robot_->getDof("j_kfe");

//     hip_body_ = robot_->getBodyNode("HFE_Actuator1");
//     foot_body_ = robot_->getBodyNode("Foot1");

//     // Fix our base link (the stand)
//     robot_->getRootBodyNode()->moveTo<dart::dynamics::WeldJoint>(nullptr);

//     robot_->computeForwardKinematics();
//     robot_->computeForwardDynamics();

//     // Set Friction
//     //std::cout << nomad->getBodyNode("foot")->getFrictionCoeff() << std::endl;
//     //nomad->getBodyNode("foot")->setFrictionCoeff(10.8);
// }
// void LegTest::Run()
// {
//     auto start_time = std::chrono::high_resolution_clock::now();
//     time_ += dt_actual_;

//     q_[0] = servo1->GetPosition();// + q1_ref;
//     qd_[0] = servo1->GetVelocity();
//     qdd_[0]= servo1->GetTorque();

//     q_[1] = servo2->GetPosition();// + q2_ref;
//     qd_[1] = servo2->GetVelocity();
//     qdd_[1]= servo2->GetTorque();

//     robot_->setPosition(0, 0);
//     robot_->setVelocity(0, 0);
//     robot_->setForce(0, 0);

//     robot_->setPosition(1, q_[0]);
//     robot_->setVelocity(1, qd_[0]);
//     robot_->setForce(1, qdd_[0]);

//     robot_->setPosition(2, q_[1]);
//     robot_->setVelocity(2, qd_[1]);
//     robot_->setForce(2, qdd_[1]);

//     robot_->computeForwardKinematics();
//     robot_->computeForwardDynamics();

//     J_ = robot_->getLinearJacobian(foot_body_, hip_body_);
//     J_ = J_.bottomRows(2);
    
//     foot_pos_ = foot_body_->getTransform(hip_body_).translation().tail(2);
//     foot_vel_ = (J_ * robot_->getVelocities()).tail(2);

//     Eigen::Vector3d torque;
//     double home_target = 0.2f;

//     switch (current_state_)
//     {
//     case FSMState_e::Homing:
//        // std::cout << " STATE HOMING " << std::endl;
//             servo1->ClosedLoopTorqueCommand(20.0f, 0.75f, home_target, 0.0f, 0.0f);
//             servo2->ClosedLoopTorqueCommand(20.0f, 0.75f, 0.0f, 0.0f, 0.0f);

//             if(AlmostEquals(q_[0], home_target, 0.1f))
//             {
//                // std::cout << "Arrived!" << std::endl;
//                 debounce++;

//                 if(debounce > 1000)
//                 {
//                 //std::cout << "NEXT STATE!" << std::endl;
//                    time_ = 0.0f;
//                    com_traj_.Generate(foot_pos_[1], -0.42f, 0.0, 0.0, 0.0, 0.8f);
//                    current_state_ = FSMState_e::Stand;
//                 }
//             }
//             else
//             {
//                 //std::cout << "Test: " << q_[0] << std::endl;
//                 debounce = 0;
//             }
//             // Check Position then goto PRESTAND
//         break;
//     case FSMState_e::Stand:

//         foot_pos_des_[1] = com_traj_.Position(time_);
//         foot_vel_des_[1] = com_traj_.Velocity(time_);

//         foot_pos_des_[0] = 0.01f;
//         foot_vel_des_[0] = 0.0f;

//         k_P_cartesian_ = k_P.asDiagonal();
//         k_D_cartesian_ = k_D.asDiagonal();
//         force_output = Eigen::Vector2d::Zero();
//         force_output += k_P_cartesian_ * (foot_pos_des_ - foot_pos_);
//         force_output += k_D_cartesian_ * (foot_vel_des_ - foot_vel_);

//         torque_output = J_.transpose() * force_output + robot_->getGravityForces() * 1.1f;

//         servo1->ClosedLoopTorqueCommand(0.0f, 0.0f, 0.0f, 0.0f, torque_output[1]);
//         servo2->ClosedLoopTorqueCommand(0.0f, 0.0f, 0.0f, 0.0f, torque_output[2]);

//         if (time_ > 1.0f)
//         {
//             //time_ = 0.0f;
//             //com_traj_.Generate(foot_pos_[1], -0.27f, 0.0, 0.0, 0.0, 1.0f);
//             //current_state_ = FSMState_e::Crouch;

//            // std::cout << "GO TO CROUCH" << std::endl;
//         }

//         break;
//     case FSMState_e::Crouch:

//         foot_pos_des_[0] = 0.01f;
//         foot_vel_des_[0] = 0.0f;

//         foot_pos_des_[1] = com_traj_.Position(time_);
//         foot_vel_des_[1] = com_traj_.Velocity(time_);


//         k_P_cartesian_ = k_P.asDiagonal();
//         k_D_cartesian_ = k_D.asDiagonal();
//         force_output = Eigen::Vector2d::Zero();
//         force_output += k_P_cartesian_ * (foot_pos_des_ - foot_pos_);
//         force_output += k_D_cartesian_ * (foot_vel_des_ - foot_vel_);

//         torque_output = J_.transpose() * force_output + robot_->getGravityForces() * 1.1f;

//         servo1->ClosedLoopTorqueCommand(0.0f, 0.0f, 0.0f, 0.0f, torque_output[1]);
//         servo2->ClosedLoopTorqueCommand(0.0f, 0.0f, 0.0f, 0.0f, torque_output[2]);

//         if (time_ > 0.6f)
//         {
//             time_ = 0.0f;
//             com_traj_.Generate(foot_pos_[1], -0.42f, 0.0, 0.0, 0.0, 0.27f);
//             current_state_ = FSMState_e::Stand;

//             std::cout << "GO TO STAND" << std::endl;
//         }


//         break;
//     default:
//         std::cout << "DEFAULT" << std::endl;
//         break;
//     }

//    // foot_pos_des_[0] = home;// + diameter * std::cos(2 * M_PI * frequency * time_);
//    // foot_pos_des_[1] = home2;// + diameter * std::sin(2 * M_PI * frequency * time_);

//   /*  Eigen::Vector2d force_output = Eigen::Vector2d::Zero();
//     Eigen::Vector2d tau_output;

//     Eigen::Vector2d k_P(600, 600);
//     Eigen::Vector2d k_D(2, 2);

//     k_P_cartesian_ = k_P.asDiagonal();
//     k_D_cartesian_ = k_D.asDiagonal();

//     force_output += k_P_cartesian_ * (foot_pos_des_ - foot_pos_);
//     force_output += k_D_cartesian_ * (foot_vel_des_ - foot_vel_);

//     Eigen::VectorXd torque = J_.transpose() * force_output + robot_->getGravityForces()*1.3f;

//     servo1->ClosedLoopTorqueCommand(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);//torque[1]);
//     servo2->ClosedLoopTorqueCommand(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);//torque[2]);*/

//     // Update DART Here
//     nlohmann::json test = {
//         {"timestamp", time_},
//         {"motor",
//             {{"Foot_Pos_1", foot_pos_[0]},
//             {"tau_ref_1", torque[1]},
//             {"Foot_Des_1",foot_pos_des_[0]},
//             {"tau_1", qdd_[0]},
//             {"Foot_Pos_2", foot_pos_[1]}, 
//             {"tau_ref_2", torque[2]},
//             {"Foot_Des_2", foot_pos_des_[1]},
//             {"tau_2", qdd_[1]},
//             {"Q_1", q_[0]},
//             {"Q_2", q_[1]},}
//         }
//     };

//     auto data = nlohmann::json::to_msgpack(test);
//     zmq::message_t message(data.size());
//     memcpy(message.data(), &data[0], data.size());
//     publisher->send(message, zmq::send_flags::dontwait);

//     auto time_now = std::chrono::high_resolution_clock::now();
//     auto total_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
// }

// void LegTest::Setup()
// {
//     // TODO: We need some sort of exceptions here if setup fails
//     CANDevice::Config_t config;
//     config.bitrate = 1e6; //1mbps
//     config.d_bitrate = 5e6; //5mbps
//     config.sample_point = 0.80f; //80.0% 
//     config.d_sample_point = 0.625f; //62.5%
//     config.clock_freq = 80e6; // 80mhz // Read from driver?  
//     config.mode_fd = 1; // FD Mode

//     if(!can.Open(DEVICE, config, true))
//     {
//         std::cout << "Unable to open CAN Device" << std::endl;
//         return;
//     }

//     // Setup Filters
//     can.ClearFilters(); // Clear Existing/Reset.  Filters are saved on the device hardware.  Must make sure to clear
//     can.AddFilter(1, 2); // Only Listen to messages on id 1.  

//     servo1 = new NomadBLDC(1, 0x10, &can);
//     servo1->SetName("INPUT");
//     if(!servo1->Connect())
//     {
//         std::cout << "[ERROR]: Unable to connect to Nomad Servo!" << std::endl;
//         exit(1);
//     }

//     std::cout << "Nomad Servo: " << "[" << servo1->GetName() << "] : " << servo1->GetServoId() << " Connected!" << std::endl;

//     servo2 = new NomadBLDC(2, 0x11, &can);
//     servo2->SetName("INPUT2");
//     if(!servo2->Connect())
//     {
//         std::cout << "[ERROR]: Unable to connect to Nomad Servo!" << std::endl;
//         exit(1);
//     }

//     std::cout << "Nomad Servo: " << "[" << servo2->GetName() << "] : " << servo2->GetServoId() << " Connected!" << std::endl;

//     // Setup ZMQ
//     publisher = new zmq::socket_t(ctx, ZMQ_PUB);
//     std::string transport("tcp://*:9872");
//     publisher->bind(transport);

//     std::cout << "Please Zero Actuator.  Enter to Continue." << std::endl;
//     getchar();

//     // Load DART
//     SetupDART();

//     servo1->ZeroOutput();
//     usleep(30000);
//     servo2->ZeroOutput();
//     usleep(30000);

//     // Start Motor Control Mode
//     usleep(8000000);
//     servo1->SetControlMode(PD_MODE);
//     servo2->SetControlMode(PD_MODE);
//     usleep(30000);

//     servo1->ClosedLoopTorqueCommand(0.0, 0.0f, 0.0f, 0.0f, 0.0f);
//     servo2->ClosedLoopTorqueCommand(0.0, 0.0f, 0.0f, 0.0f, 0.0f);

//     usleep(30000);

//     // Get Initial Values
//     q_[0] = servo1->GetPosition();
//     qd_[0] = servo1->GetVelocity();
//     qdd_[0]= servo1->GetTorque();

//     q_[1] = servo2->GetPosition();
//     qd_[1] = servo2->GetVelocity();
//     qdd_[1]= servo2->GetTorque();

//     robot_->setPosition(0, 0);
//     robot_->setVelocity(0, 0);
//     robot_->setForce(0, 0);

//     robot_->setPosition(1, q_[0]);
//     robot_->setVelocity(1, qd_[0]);
//     robot_->setForce(1, qdd_[0]);

//     robot_->setPosition(2, q_[1]);
//     robot_->setVelocity(2, qd_[1]);
//     robot_->setForce(2, qdd_[1]);

//     robot_->computeForwardKinematics();
//     robot_->computeForwardDynamics();

//     J_ = robot_->getLinearJacobian(foot_body_, hip_body_);
//     J_ = J_.bottomRows(2);

//     foot_pos_ = foot_body_->getTransform(hip_body_).translation().tail(2);
//     foot_vel_ = (J_ * robot_->getVelocities()).tail(2);
//     foot_pos_des_ = foot_pos_;

//     //std::cout << "Press Key to Start Control." << std::endl;
//     //usleep(5000000);
//     current_state_ = FSMState_e::Homing;
//     std::cout << "Foot Pos Here: " << foot_pos_[0] << ", " << foot_pos_[1] << std::endl;
// }

// void LegTest::Exit()
// {
//     std::cout << "Exiting!" << std::endl;

//     // Set back to idle.  In theory when no commands are sent it should auto back to idle or edamp?
//     servo1->SetControlMode(IDLE_MODE);
//     servo2->SetControlMode(IDLE_MODE);
// }


// int main(int argc, char *argv[])
// {
//     Realtime::RealTimeTaskManager::Instance();
//     if (!Realtime::RealTimeTaskManager::EnableRTMemory(500 * 1024 * 1024)) { // 500MB
//         std::cout << "Error configuring Realtime Memory Requiremets!  Realtime Execution NOT guaranteed." << std::endl;
//     }
//     else {
//         std::cout << "Real Time Memory Enabled!" << std::endl;
//     }

//     LegTest pj_Node("Test", 1/1000.0f); //1000hz
//     pj_Node.SetStackSize(1024 * 1024);
//     pj_Node.SetTaskPriority(Realtime::Priority::HIGHEST);
//     pj_Node.SetCoreAffinity(2);
//     pj_Node.Start();

//     char input;
//     std::cin >> input;
//     if(input == 'q') {
//         std::cout << "Got Q:" << std::endl;
//     }

//     pj_Node.Exit();
//     pj_Node.Stop();
// }
