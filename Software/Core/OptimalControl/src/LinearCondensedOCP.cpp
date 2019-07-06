#include <OptimalControl/LinearCondensedOCP.hpp>


namespace OptimalControl
{
namespace LinearOptimalControl
{
LinearCondensedOCP::LinearCondensedOCP(const int &N, const double &T, const int &num_states, const int &num_inputs, const bool time_varying) : LinearOptimalControlProblem(N, T, num_states, num_inputs)
{

}

void LinearCondensedOCP::Solve()
{

}

} // namespace LinearOptimalControl
} // namespace OptimalControl

