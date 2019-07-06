
#include <Nomad/Nomad.hpp>
#include <plotty/matplotlibcpp.hpp>

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

int main()
{
    RigidBlock1D block = RigidBlock1D(1.0, Eigen::Vector3d(1.0, 0.5, 0.25));

    Eigen::VectorXd initial_state(2);
    initial_state[0] = 10.0;
    initial_state[1] = 0.0;

    block.SetState(initial_state);
    Eigen::VectorXd input(1);

    input[0] = 1.5;
    for (int i = 0; i < 10; i++)
    {
        block.Step(input);

        std::cout << "State: " << block.GetState() << std::endl;
    }
    std::vector<double> v({1, 2, 3, 4});
    plotty::plot(v);
    plotty::show();
}
