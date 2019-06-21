#include <OptimalControl/ControlsLibrary.hpp>


void ControlsLibrary::ContinuousToDiscrete(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, double Ts, Eigen::MatrixXd& A_d, Eigen::MatrixXd& B_d) 
{
    int num_states = A.cols(); // Get number of states
    int num_inputs = B.cols(); // Get number of inputs

    // Create placeholder for operations on matrices to convert to discrete time
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(num_states + num_inputs, num_states + num_inputs);
    M.block(0, 0, num_states, num_states) = A * Ts;
    M.block(0, num_states, num_states, num_inputs) = B * Ts;

    // Compute Matrix Exponential
    Eigen::MatrixXd M_d = M.exp();

    // Extract the discrete time dynamics matrices
    A_d = M_d.block(0, 0, num_states, num_states);
    B_d = M_d.block(0, num_states, num_states, num_inputs);

}