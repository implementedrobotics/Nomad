
/*
 * ConvecMatrixEquationSolverQP.hpp
 *
 *  Created on: August 20, 2020
 *      Author: Quincy Jones
 *
 * Copyright (c) <2020> <Quincy Jones - quincy@implementedrobotics.com/>
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef NOMAD_CORE_OPTIMALCONTROL_CONVEXMATRIXEQUATIONSOLVERQP_H_
#define NOMAD_CORE_OPTIMALCONTROL_CONVEXMATRIXEQUATIONSOLVERQP_H_

// C System Files

// C++ System Files
#include <iostream>

// Third Party Includes
#include <Eigen/Dense>
#include <qpOASES.hpp>

// Project Includes
#include <Common/Math/MathUtils.hpp>


// Helper class to solve a system of linear equations of the form: A * x = b
// In general you would use this is there is reduncancy in the system (Number of equations/Number of unknowns)
// Therefore this is only solved in a somewhat "least squares sense" by exploiting variable weighting and inequality constraints to handle
// the reduncancy in the sysytem.  This is good because the redundancy in the system allows you to meet secondary objectives.

// x* = min(Ax-b)^T * S * (A*x-b) + alpha*x^T*W1*x + beta*(x-x_prev)^T * W2 * (x-x_prev))
// s.t. lb < C*x < ub

// Maps to a standard QP Formulation using qpOases:
//
// min x      1/2*x^T*H*x + x^t*g
// s.t. lb < A_qp*x > ub // Inequality Contraints

// x* = optimal solution (Num Equations)
// S  = Relative priority of decision variables (Num Equations x Num Equations)
// alpha = decision variable minimization weight (scalar)
// W1   = Relative priority of weight minimization on decision variables (Num Variables x Num Variables)
// beta = solution filtering from a previous solution.  Helps with smoothing from a previous solution (scalar)
// W2   = Relative priority of weight smoothing on decision variables (Num Variables x Num Variables)
// C    = Constraint matrix for solution (Num inequality constraints x Num Variables)

namespace Core::OptimalControl
{
    class ConvexLinearSystemSolverQP
    {

    public:
        // Base Class ConvexLinearSystemSolverQP
        // num_eq = Number of equations of OCP
        // num_vars = Number of veriables of OCP
        ConvexLinearSystemSolverQP(const unsigned int num_eq, const unsigned int num_vars, const unsigned int num_constraints);

        // Solve
        virtual void Solve();

        // Return Current Solution X
        Eigen::VectorXd X() const { return x_star_; }

        double GetSolverTime() const { return solver_time_; }

        void EnableQPDebug(bool enable);
        void PrintDebug();

    protected:

        qpOASES::SQProblem qp_;        // qpOases Solver Object

        qpOASES::int_t max_iterations_;    // Max Iterations for Solver
        qpOASES::int_t solver_iterations_; // Total number of Solver iterations for solution

        Eigen::MatrixXd A_;           // Coefficient Matrix
        Eigen::VectorXd x_star_;      // Solution Vector
        Eigen::VectorXd x_star_prev_; // Previous Solutions Vector
        Eigen::VectorXd b_;           // Constant Vector

        Eigen::MatrixXd S_;           // Relative Priority Weighting of Values
        Eigen::MatrixXd W_1_;         // Relative Priority Weighting of Solution Minimization
        Eigen::MatrixXd W_2_;         // Relative Priority Weighting of Solution Filtering Minimization
        //Eigen::MatrixXd C_;         // Inequality Constraint Matrix

        Eigen::VectorXd lbA_; // Lower Bound of Inequality Constraint Matrix
        Eigen::VectorXd ubA_; // Upper Bound of Inequality Constraint Matrix

        Eigen::MatrixXd H_qp_; // QP Hessian
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_qp_; // Inequality Constraint Matrix QP
        Eigen::VectorXd g_qp_; // Linear

        double alpha_; // Influence of Solution Minimization
        double beta_;  // Influece of Solution Filtering

        int num_equations_;   // Number of System Equations
        int num_variables_;   // Number of System Variables
        int num_constraints_; // Number of Inequality Constraints

        double solver_time_; // Total time for Solver to compute a solution

        bool solved_; // Valid Solution?

        bool is_hot_; // We have a warm QP and can use hotstarting

        // TODO:
        // Infeasible, BlahBlah
    };
} // namespace Core::OptimalControl

#endif // NOMAD_CORE_OPTIMALCONTROL_CONVEXMATRIXEQUATIONSOLVERQP_H_
