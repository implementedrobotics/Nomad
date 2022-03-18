// #include <iostream>
// #include <typeinfo>  //for 'typeid' to work  

// #include <rbdl/rbdl.h>
// #include <ascent/Ascent.h>

// using namespace RigidBodyDynamics;
// using namespace RigidBodyDynamics::Math;

// using namespace asc;

// class RigidBodySystem {

// protected:
//     Model* model;

//     VectorNd Q;
//     VectorNd QDot; 
//     VectorNd Tau; 
//     VectorNd QDDot;

// public:

//     // x, y, z, xd, yd, zd
//     state_t x = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
//     RigidBodySystem() {
//         rbdl_check_api_version (RBDL_API_VERSION);
//         model = new Model();

//         unsigned int body_a_id;
//         Body body_a;
//         Joint joint_a;

//         model->gravity = Vector3d (0., 0., -10.0);

//         body_a = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
//         joint_a = Joint(JointTypeFloatingBase);
        
//         body_a_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

//         Q = VectorNd::Zero (model->dof_count);
//         QDot = VectorNd::Ones (model->dof_count);
//         Tau = VectorNd::Zero (model->dof_count);
//         QDDot = VectorNd::Zero (model->dof_count);

//         //std::cout << Q.size() << std::endl;
//         std::cout << sizeof(Q.data()[3]) << std::endl;
//         //std::cout << sizeof(double) << std::endl;
//     }

//     ~RigidBodySystem() {
//         delete model;
//     }
    
//     void operator()(const state_t& x, state_t& xd, const double t) {

//         Q[0] = x[0];
//         Q[1] = x[1];
//         Q[2] = x[2];

//         QDot[0] = xd[0] = x[3];
//         QDot[1] = xd[1] = x[4];
//         QDot[2] = xd[2] = x[5];

//         ForwardDynamics (*model, Q, QDot, Tau, QDDot);
        
//         xd[3] = QDDot[0];
//         xd[4] = QDDot[1];
//         xd[5] = QDDot[2];
//     }
// };



// class PendulumSystem {

// protected:
//     Model* model;

//     VectorNd q;
//     VectorNd qd; 
//     VectorNd tau; 
//     VectorNd qdd;

// public:

//     // x, y, z, xd, yd, zd
//     state_t x = { 0.5, 0.1 };
//     PendulumSystem() {
//         rbdl_check_api_version (RBDL_API_VERSION);
//         model = new Model();

//         unsigned int body_a_id;
//         Body body_a;
//         Joint joint_a;

//         model->gravity = Vector3d (0., 0., -10.0);

//         body_a = Body (1., Vector3d (0.0, 0.0, -0.5), Vector3d (1., 1., 1.));
//         joint_a = Joint(JointTypeRevoluteY);
        
//         body_a_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

//         q = VectorNd::Zero (model->dof_count);
//         qd = VectorNd::Zero (model->dof_count);
//         tau = VectorNd::Zero (model->dof_count);
//         qdd = VectorNd::Zero (model->dof_count);

//         q[0] = 0.5;
//         qd[0] = 0.1;
//         std::cout << model->dof_count << std::endl;

//         //std::cout << Q.size() << std::endl;
//         //std::cout << sizeof(Q.data()[3]) << std::endl;
//         //std::cout << sizeof(double) << std::endl;
//     }

//     ~PendulumSystem() {
//         delete model;
//     }
    
//     void operator()(const state_t& x, state_t& xd, const double t) {

//         q[0] = x[0];
//         qd[0] = xd[0] = x[1];

//         ForwardDynamics (*model, q, qd, tau, qdd);
        
//         xd[1] = qdd[0];

//  //       std::cout << q << std::endl;
//    //     std::cout << qdd << std::endl;
//     }
// };

// int main (int argc, char* argv[]) {

//     double t = 0.0;
//     double dt = 0.05;
//     double t_end = 20.0;

//     Euler integrator;
//     Recorder recorder;

//     PendulumSystem system;
//     while(t < t_end) {
//         recorder({t, system.x[0], system.x[1]});
//         integrator(system, system.x, t, dt);
//     }
//     recorder.csv("rbdl_test", { "t", "theta", "omega" });

//  	return 0;
// }