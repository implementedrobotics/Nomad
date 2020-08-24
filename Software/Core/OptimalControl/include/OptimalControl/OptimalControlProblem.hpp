
/*
 * OptimalControlProblem.hpp
 *
 *  Created on: July 7, 2019
 *      Author: Quincy Jones
 *
 * Copyright (c) <2019> <Quincy Jones - quincy@implementedrobotics.com/>
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
#ifndef NOMAD_CORE_OPTIMALCONTROL_OPTIMALCONTROLPROBLEM_H_
#define NOMAD_CORE_OPTIMALCONTROL_OPTIMALCONTROLPROBLEM_H_

#include <iostream>
#include <Eigen/Dense>
#include <qpOASES.hpp>

namespace OptimalControl
{

    class OptimalControlProblem
    {

    public:
        // Base Class Optimal Control Problem
        // N = Prediction Steps
        // T = Horizon Length
        // num_states = Number of States of OCP
        // num_inputs = Number of Inputs of OCP
        OptimalControlProblem(const unsigned int N, const double T, const unsigned int num_states, const unsigned int num_inputs, const unsigned int max_iterations = 1000);

        // Solve
        virtual void Solve() = 0;

        // Get System State Trajectory Vector
        // TODO: Compute this depending on U solutions.  X_ = Ax+Bu
        // TODO: Make pure virtual
        virtual Eigen::MatrixXd X() const { return X_; }

        //Get Current Input Sequence Solution
        Eigen::MatrixXd U() const { return U_; }

        // Set Weight Matrices
        virtual void SetWeights(const Eigen::VectorXd &Q, const Eigen::VectorXd &R)
        {
            // TODO: Verify Vector Size Matches correct state and inputs
            Q_ = Q.matrix().asDiagonal().toDenseMatrix();
            R_ = R.matrix().asDiagonal().toDenseMatrix();
        }

        // Set Problem Initial Condition
        void SetInitialCondition(const Eigen::VectorXd &x_0) { x_0_ = x_0; }

        // Set Reference Trajectory
        void SetReference(const Eigen::MatrixXd &X_ref) { X_ref_ = X_ref; }

        // Time step sample time
        double SampleTime() const { return T_s_; }

        // Prediction Horizon Steps
        int N() const { return N_; }

    protected:
        Eigen::VectorXd x_0_;   // Current State/Initial Condition
        Eigen::MatrixXd X_ref_; // Reference Trajectory

        Eigen::MatrixXd X_; // System State Trajectory
        Eigen::MatrixXd U_; // Optimal Control Input Sequence Solution

        Eigen::MatrixXd Q_; // State Weights
        Eigen::MatrixXd R_; // Input Weights

        int num_states_; // Number of System States
        int num_inputs_; // Number of System Inputs

        int N_; // Number of Prediction Steps

        double T_s_; // Sample Time
        double T_;   // Horizon Length

        qpOASES::int_t max_iterations_;    // Max Iterations for Solver
        qpOASES::int_t solver_iterations_; // Total number of Solver iterations for solution
        double solver_time_;    // Total time for Solver to compute a solution

        bool solved_;
        // TODO:
        // Infeasible, BlahBlah
    };
} // namespace OptimalControl

namespace OptimalControl
{
    namespace LinearOptimalControl
    {
        class LinearOptimalControlProblem : public OptimalControlProblem
        {
        public:
            // Base Class Linear Optimal Control Problem
            // N = Prediction Steps
            // T = Horizon Length
            // num_states = Number of States of OCP
            // num_inputs = Number of Inputs of OCP
            // max_iterations = Maximum number of solver iterations
            LinearOptimalControlProblem(const unsigned int N,
                                        const double T,
                                        const unsigned int num_states,
                                        const unsigned int num_inputs,
                                        const unsigned int max_iterations = 1000);

            // Set Model Matrices
            void SetModelMatrices(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B)
            {
                A_ = A;
                B_ = B;
            }

            // Solve
            virtual void Solve();

        protected:
            Eigen::MatrixXd A_; // System State Transition Matrix
            Eigen::MatrixXd B_; // Control Input Matrix
        };
    } // namespace LinearOptimalControl
} // namespace OptimalControl

#endif // NOMAD_CORE_OPTIMALCONTROL_OPTIMALCONTROLPROBLEM_H_
