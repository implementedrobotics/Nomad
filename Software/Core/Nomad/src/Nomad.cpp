#include <Nomad/Nomad.hpp>
#include <plotty/matplotlibcpp.hpp>
#include <OptimalControl/OptimalControlProblem.hpp>
#include <OptimalControl/LinearCondensedOCP.hpp>
#include <iostream>

RigidBlock1D::RigidBlock1D(const double &mass,
                           const Eigen::Vector3d &box_shape,
                           const double &T_s /* = 1e-1*/) : LinearDynamicalSystem(2, 1, T_s),
                                                            mass(mass)
{
    std::cout << "Num States: " << num_states << std::endl;
    length = box_shape[0];
    width = box_shape[1];
    height = box_shape[2];

    // Setup initial states
    SetState(Eigen::VectorXd::Zero(num_states));

    // Setup System Matrices
    _A << 0, 1,
          0, 0;

    // Setup Input Matrix
    _B << 0,
        1.0 / mass;

    std::cout << "A: " << std::endl
              << _A << std::endl;
    std::cout << "B: " << std::endl
              << _B << std::endl;

    // Cache Discrete Time Variant
    ControlsLibrary::ContinuousToDiscrete(_A, _B, T_s, _A_d, _B_d);
}

void RigidBlock1D::Step(const Eigen::VectorXd &u)
{
    _x = _A_d * _x + _B_d * u;
}

void RigidBlock1D::Update()
{
    
}

using namespace OptimalControl::LinearOptimalControl;

int main()
{

    LinearCondensedOCP ocp = LinearCondensedOCP(10, 1.0, 4,4,false);

    Eigen::VectorXd Q(2);
    Q[0] = 1.0;
    Q[1] = 2.0;

    Eigen::VectorXd R(1);
    R[0] = 0.01;
    ocp.SetWeights(Q, R);
    RigidBlock1D block = RigidBlock1D(1.0, Eigen::Vector3d(1.0, 0.5, 0.25));

    Eigen::VectorXd initial_state(2);
    initial_state[0] = 0.0;
    initial_state[1] = 0.0;

    int num_steps = 100;
    block.SetState(initial_state);
    Eigen::VectorXd input(1);
    Eigen::MatrixXd plot_me(2, num_steps);
    input[0] = 10.5;
    for (int i = 0; i < num_steps; i++)
    {
        block.Step(input);
        plot_me.col(i) = block.GetState();
    }
    

    plotty::labelPlot("pos", plot_me.row(0));
    plotty::labelPlot("vel", plot_me.row(1));
    plotty::legend();
    plotty::show();
}
