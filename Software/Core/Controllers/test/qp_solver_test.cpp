#include <qpOASES.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
//#include <OptimalControl/ControlsLibrary.hpp>

using namespace qpOASES;


using Eigen::MatrixXd;


int main()
{
    /*
    MatrixXd A(2,2);
    MatrixXd B(2,1);
    A(0,0) = 1;
    A(0,1) = 1;
    A(1,1) = 1;
    A(1,0) = 1;
    B(0,0) = 10;
    B(1,0) = 10;

    MatrixXd A_d, B_d;
    
    ControlsLibrary::ContinuousToDiscrete(A, B, 0.1, A_d, B_d);
    
    cout <<A_d << endl << B_d << endl; */


    // QProblem qp(2,1);

    // MatrixXd H(2,2);
    // MatrixXd A(1,2);
    // MatrixXd g(2,1);
    // MatrixXd lbA(1,1);

    // H = 2*MatrixXd::Identity(2,2);
    // A = MatrixXd::Ones(1,2);
    // g = MatrixXd::Zero(2,1);

    // lbA << 10;
    // /*

    // H << 0.4, 0,
    //      0, 1;
    // H = 2 * H;
    // cout << H << endl;
    // A << 1, -1,
    //      -0.3, -1;
    // ubA << -2, -8;
    // g << -5, -6;
    // lb << 0, 0;
    // ub << 10, 10; */


    // cout << H << endl;
    // cout << A << endl;
    // cout << lbA << endl;
    // cout << g << endl;

    // int nWSR = 10;
    // qp.setPrintLevel(qpOASES::PL_MEDIUM);
    // qp.init(H.data(), g.data(), A.data(), NULL, NULL, lbA.data(), NULL, nWSR);
    // MatrixXd x_out(2,1);

    // qp.getPrimalSolution(x_out.data());

    // cout << x_out << endl;
    //cout << qp.getObjVal() << endl;



    QProblem qp(2,2);

     Eigen::Matrix<double, 2, 2, Eigen::RowMajor> H;
     Eigen::Matrix<double, 2, 2, Eigen::RowMajor> A;

   // MatrixXd H(2,2);
   // MatrixXd A(2,2);
    MatrixXd g(2,1);
    MatrixXd ubA(2,1);
    //MatrixXd lbX(2,1);
    //MatrixXd ubX(2,1);

    H << 0.4, 0.0,
         0.0, 1.0;

    H = 2 * H;

    A << 1.0, -1.0,
         -0.3, -1.0;

    ubA << -2, -8;

    g << -5, -6;
    //lbX << 0, 0;
    //ubX << 10, 10;

    std::cout << H << std::endl;
    std::cout << H.data()[0] << std::endl;
    std::cout << H.data()[1] << std::endl;
    std::cout << H.data()[2] << std::endl;
    std::cout << H.data()[3] << std::endl;
    std::cout << A << std::endl;

    std::cout << A.data()[0] << std::endl;
    std::cout << A.data()[1] << std::endl;
    std::cout << A.data()[2] << std::endl;
    std::cout << A.data()[3] << std::endl;

    std::cout << g << std::endl;
 //   cout << ubA << endl;
    std::cout << ubA << std::endl;
    //cout << lbX << endl;
    //cout << ubX << endl;

     qpOASES::Options myOptions;
     //myOptions.enableRamping = BT_FALSE;
     //myOptions.maxPrimalJump = 1;
     //myOptions.setToMPC();
    int64_t nWSR = 10;
    qp.setPrintLevel(qpOASES::PL_HIGH);
    qp.setOptions(myOptions);
    qp.init(H.data(), g.data(), A.data(), NULL, NULL, NULL, ubA.data(), nWSR);
    MatrixXd x_out(2,1);

    qp.getPrimalSolution(x_out.data());
    // qp.printOptions();
    std::cout << x_out << std::endl;
}   