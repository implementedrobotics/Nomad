#include <iostream>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>


namespace ControlsLibrary {

    /**
     * Converts the linear continuous time dynamical systems model to disrete time
     * using the Zero Order Hold method
     *
     * @param[in] A Continuous time system(state) transition matrix
     * @param[in] B Continuous time input matrix
     * @param[in] Ts Sampling time
     * @param[out] A_d Discrete time system(state) transition matrix
     * @param[out] B_d Discrete time input matrix
     * @param[in] data the data to analyze
     */
    void ContinuousToDiscrete(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, double Ts, Eigen::MatrixXd& A_d, Eigen::MatrixXd& B_d);

}