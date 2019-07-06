#include <OptimalControl/OptimalControlProblem.hpp>
#include <vector>

#ifndef NOMAD_CORE_OPTIMALCONTROL_LINEARCONDENSEDOCP_H_
#define NOMAD_CORE_OPTIMALCONTROL_LINEARCONDENSEDOCP_H_

namespace OptimalControl
{
namespace LinearOptimalControl
{
class LinearCondensedOCP : public LinearOptimalControlProblem
{
public:
    // Base Class Linear Optimal Control Problem
    // N = Prediction Steps
    // T = Horizon Length
    // num_states = Number of States of OCP
    // num_inputs = Number of Inputs of OCP
    LinearCondensedOCP(const int &N, const double &T, const int &num_states, const int &num_inputs, const bool time_varying = false);

    // Set Model Matrices (Time Invariant)
    void SetModelMatrices(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B)
    {
        // TODO: Check for Time Varying Here
        // TODO: Check Dimensions and make sure they match num_states/inputs
        _A[0] = A;
        _B[0] = B;
    }

    // Set Model Matrices (Time Varying)
    void SetModelMatrices(const std::vector<Eigen::MatrixXd> &A, const std::vector<Eigen::MatrixXd> &B)
    {
        _A = A;
        _B = B;
    }

    // Solve
    virtual void Solve();

protected:
    std::vector<Eigen::MatrixXd> _A; // System State Transition Matrix (Vector List for Time Varying)
    std::vector<Eigen::MatrixXd> _B; // Input Matrix (Vector List for Time Varying)

    bool _time_varying;
};
} // namespace OptimalControl
} // namespace LinearOptimalControl

#endif // NOMAD_CORE_OPTIMALCONTROL_LINEARCONDENSEDOCP_H_