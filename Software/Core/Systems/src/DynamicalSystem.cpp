#include <Systems/DynamicalSystem.hpp>

DynamicalSystem::DynamicalSystem(const int& num_states, const double& T_s /* = 1e-1*/) : T_s(T_s), T(0), num_states(num_states)
{
    _x = Eigen::VectorXd(num_states);
}


LinearDynamicalSystem::LinearDynamicalSystem(const int& num_states, const int& num_inputs, const double& T_s /* = 1e-1*/) : DynamicalSystem(num_states, T_s)
{
    _A = Eigen::MatrixXd(num_states, num_states);
    _A_d = Eigen::MatrixXd(num_states, num_states);

    _B = Eigen::MatrixXd(num_states, num_inputs);
    _B_d = Eigen::MatrixXd(num_states, num_inputs);

}


LinearTimeVaryingDynamicalSystem::LinearTimeVaryingDynamicalSystem(const int& num_states, const int& num_inputs, const double& T_s /* = 1e-1*/) : LinearDynamicalSystem(num_states, num_inputs, T_s)
{
    _x = Eigen::VectorXd(num_states);
    
}

