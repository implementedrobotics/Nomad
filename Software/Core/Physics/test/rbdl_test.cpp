#include <iostream>

#include <rbdl/rbdl.h>
#include <ascent/Ascent.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using namespace asc;

class RigidBodySystem
{

protected:
    Model* model;

    VectorNd Q;
    VectorNd QDot; 
    VectorNd Tau; 
    VectorNd QDDot;

public:

    RigidBodySystem() {
        rbdl_check_api_version (RBDL_API_VERSION);
        model = new Model();

        unsigned int body_a_id;
        Body body_a;
        Joint joint_a;

        model->gravity = Vector3d (0., 0., -10.0);

        body_a = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
        joint_a = Joint(JointTypeFloatingBase);
        
        body_a_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

        Q = VectorNd::Zero (model->dof_count);
        QDot = VectorNd::Zero (model->dof_count);
        Tau = VectorNd::Zero (model->dof_count);
        QDDot = VectorNd::Zero (model->dof_count);

    }
    ~RigidBodySystem() {
        delete model;
    }
    void operator()(const state_t& x, state_t& xd, const double t) {
        //xd[0] = cos(t);
        std::cout << t << std::endl;
        ForwardDynamics (*model, Q, QDot, Tau, QDDot);
        std::cout << QDDot.transpose() << std::endl;

        
        //xd[0] = 
    }
};


int main (int argc, char* argv[]) {

    state_t x = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    double t = 0.0;
    double dt = 0.1;
    double t_end = 10.0;

    Euler integrator;
    Recorder recorder;

    RigidBodySystem system;
    while(t < t_end) {
        std::cout << "Step: " << std::endl;
        recorder({t, x[0], x[1], x[2]});
        integrator(system, x, t, dt);
    }

    recorder.csv("rbdl_test", { "t", "x0", "x1", "x2" });
	



 	return 0;
}