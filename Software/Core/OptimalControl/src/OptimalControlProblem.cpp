#include <OptimalControl/OptimalControlProblem.hpp>

namespace OptimalControl
{

OptimalControlProblem::OptimalControlProblem(const int &N, const double &T, const int &num_states, const int &num_inputs) : T(T), N(N), num_states(num_states), num_inputs(num_inputs)
{
    _x_0 = Eigen::VectorXd(num_states);
    _X_ref = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_states, N);

    _X = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_states, N);
    _U = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_inputs, N-1);

    _Q = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_states, num_states);
    _R = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_inputs, num_inputs);

    T_s = T / N;
}
} // namespace OptimalControl



namespace OptimalControl
{
namespace LinearOptimalControl
{
LinearOptimalControlProblem::LinearOptimalControlProblem(const int &N, const double &T, const int &num_states, const int &num_inputs) : OptimalControlProblem(N, T, num_states, num_inputs)
{
    _A = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_states, num_states);
    _B = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_states, num_inputs);
}
void LinearOptimalControlProblem::Solve()
{

}
} // namespace LinearOptimalControl
} // namespace OptimalControl