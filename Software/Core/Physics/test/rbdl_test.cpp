#include <iostream>

#include <rbdl/rbdl.h>
#include <ascent/Ascent.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using namespace asc;

Model* model = new Model();

	VectorNd Q = VectorNd::Zero (model->dof_count);
	VectorNd QDot = VectorNd::Zero (model->dof_count);
	VectorNd Tau = VectorNd::Zero (model->dof_count);
	VectorNd QDDot = VectorNd::Zero (model->dof_count);


struct DynamicSystem
{
   void operator()(const state_t& x, state_t& xd, const double t)
   {
      //xd[0] = cos(t);

      ForwardDynamics (*model, Q, QDot, Tau, QDDot);
      std::cout << QDDot.transpose() << std::endl;

      
      //xd[0] = 
   }
};


int main (int argc, char* argv[]) {


    rbdl_check_api_version (RBDL_API_VERSION);

	unsigned int body_a_id;
	Body body_a;
	Joint joint_a;

	//model = new Model();

	model->gravity = Vector3d (0., 0., -10.0);

	body_a = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
	joint_a = Joint(JointTypeFloatingBase);
	
	body_a_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);


    Q = VectorNd::Zero (model->dof_count);
	QDot = VectorNd::Zero (model->dof_count);
	Tau = VectorNd::Zero (model->dof_count);
	QDDot = VectorNd::Zero (model->dof_count);

    state_t x = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    double t = 0.0;
    double dt = 0.1;
    double t_end = 10.0;

    RK4 integrator;
    Recorder recorder;

    while(t < t_end) {
        recorder({t, x[0], x[1], x[2]});
        integrator(DynamicSystem(), x, t, dt);
    }

    recorder.csv("rbdl_test", { "t", "x0", "x1", "x2" });
	

	delete model;

 	return 0;
}