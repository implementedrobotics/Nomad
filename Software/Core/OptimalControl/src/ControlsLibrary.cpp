
/*
 * ControlsLibrary.cpp
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

#include <OptimalControl/ControlsLibrary.hpp>

void ControlsLibrary::ContinuousToDiscrete(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, double Ts, Eigen::MatrixXd &A_d, Eigen::MatrixXd &B_d)
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
namespace ControlsLibrary
{
    namespace EigenHelpers
    {

        // TODO: Change this to a template so we can pass constexpr to get more efficient eigen arrays
        BlockMatrixXd::BlockMatrixXd(const unsigned int Rows,
                                     const unsigned int Cols,
                                     const unsigned int BlockHeight,
                                     const unsigned int BlockWidth,
                                     const int init_val) : Rows_(Rows),
                                                           Cols_(Cols),
                                                           BlockHeight_(BlockHeight),
                                                           BlockWidth_(BlockWidth)
        {
            assert(Rows > 0 && Cols > 0);
            assert(BlockHeight > 0 && BlockWidth > 0);
            Matrix_ = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(Rows_ * BlockHeight, Cols_ * BlockWidth);
            Matrix_ = Eigen::MatrixXd::Constant(Matrix_.rows(), Matrix_.cols(), init_val);
        }

        BlockMatrixXd::BlockMatrixXd(const unsigned int Rows,
                                     const unsigned int Cols,
                                     const unsigned int BlockHeight,
                                     const unsigned int BlockWidth) : Rows_(Rows),
                                                           Cols_(Cols),
                                                           BlockHeight_(BlockHeight),
                                                           BlockWidth_(BlockWidth)
        {
            assert(Rows > 0 && Cols > 0);
            assert(BlockHeight > 0 && BlockWidth > 0);
            Matrix_ = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(Rows_ * BlockHeight, Cols_ * BlockWidth);
        }


        BlockMatrixXd::BlockMatrixXd(const unsigned int Rows,
                                     const unsigned int Cols,
                                     const unsigned int BlockHeight,
                                     const unsigned int BlockWidth,
                                     Eigen::MatrixXd &matrix) : Rows_(Rows),
                                                           Cols_(Cols),
                                                           BlockHeight_(BlockHeight),
                                                           BlockWidth_(BlockWidth)
        {
            assert(Rows > 0 && Cols > 0);
            assert(BlockHeight > 0 && BlockWidth > 0);
            Matrix_ = matrix;
        }


        // Operator Overload:
        void BlockMatrixXd::operator()(const unsigned int Row, const unsigned int Col, const Eigen::MatrixXd &block_val)
        {
            assert(Row < Rows_ && Col < Cols_);
            Matrix_.block(Row * BlockHeight_, Col * BlockWidth_, BlockHeight_, BlockWidth_) = block_val;
        }

        Eigen::Block<Eigen::MatrixXd, -1, -1, false> BlockMatrixXd::operator()(const unsigned int Row, const unsigned int Col)
        {
            assert(Row < Rows_ && Col < Cols_);
            return Matrix_.block(Row * BlockHeight_, Col * BlockWidth_, BlockHeight_, BlockWidth_);
        }
        
        void BlockMatrixXd::SetBlock(const unsigned int Row, const unsigned int Col, const Eigen::MatrixXd &block_val)
        {
            assert(Row < Rows_ && Col < Cols_);
            Matrix_.block(Row * BlockHeight_, Col * BlockWidth_, BlockHeight_, BlockWidth_) = block_val;
        }
        void BlockMatrixXd::FillDiagonal(const Eigen::MatrixXd &block_val, const int k)
        {
            // Handle Super and Sub Diagonals (k)
            int row_offset = 0;
            int col_offset = 0;

            if (k < 0)
                row_offset = -k;
            else if (k > 0)
                col_offset = k;

            for (int i = 0; (i + row_offset) < Rows_ && (i + col_offset) < Cols_; i++)
            {
                Matrix_.block((i + row_offset) * BlockHeight_, (i + col_offset) * BlockWidth_, BlockHeight_, BlockWidth_) = block_val;
            }
        }

    } // namespace EigenHelpers
} // namespace ControlsLibrary