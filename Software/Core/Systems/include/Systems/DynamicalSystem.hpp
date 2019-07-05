
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>

class DynamicalSystem
{

public:
    DynamicalSystem(const int& num_states, const double& Ts = 1e-1);

    // Step System
    virtual void Step() = 0;

    // Update
    virtual void Update() = 0;

protected:

    Eigen::VectorXd _x;  // State Vector

    int num_states; // Number of System States

    double T_s; // Sample Time
    double T; // Current Time
};


class NonLinearDynamicalSystem : public DynamicalSystem
{
    
protected:
    virtual void f(const Eigen::MatrixXd& x, const Eigen::MatrixXd& u) = 0;

};

class LinearDynamicalSystem : public DynamicalSystem
{

public:

    LinearDynamicalSystem(const int& num_states, const int& num_inputs, const double& T_s = 1e-1);

    // Model Matrices
    Eigen::MatrixXd A() { return _A; };
    Eigen::MatrixXd B() { return _B; };

    // Discrete Time Model Matrices
    Eigen::MatrixXd A_d() { return _A_d; };
    Eigen::MatrixXd B_d() { return _B_d; };

    // Get Current State
    Eigen::VectorXd GetState() { return _x; }

protected:
    int num_inputs;     // Number of inputs
    Eigen::MatrixXd _A;  // State Transition Mat rix
    Eigen::MatrixXd _B;  // Input Matrix

    Eigen::MatrixXd _A_d;  // Discrete Time State Transition Mat rix
    Eigen::MatrixXd _B_d;  // Discrete Time Input Matrix

};

class LinearTimeVaryingDynamicalSystem : public LinearDynamicalSystem
{

public:
    LinearTimeVaryingDynamicalSystem(const int& num_states, const int& num_inputs, const double& T_s = 1e-1);

    // Get the Time-Varying Matrices from Model
    virtual void GetModelMatrices(const int& N, Eigen::MatrixXd& A, Eigen::MatrixXd& B) = 0;


    // Model Matrices (Time Varying)
    std::vector<Eigen::MatrixXd> A_TV() { return _A_tv; };
    std::vector<Eigen::MatrixXd> B_TV() { return _B_tv; };

    // Discrete Time Model Matrices (Time Varying)
    std::vector<Eigen::MatrixXd> A_d_TV() { return _A_d_tv; };
    std::vector<Eigen::MatrixXd>B_d_TV() { return _B_d_tv; };

protected:

    // Time Varying Matrices
    std::vector<Eigen::MatrixXd> _A_tv;
    std::vector<Eigen::MatrixXd> _B_tv;

    std::vector<Eigen::MatrixXd> _A_d_tv;
    std::vector<Eigen::MatrixXd> _B_d_tv;
};
