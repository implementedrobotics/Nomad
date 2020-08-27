/*
 * MathUtils.cpp
 *
 *  Created on: August 22, 2020
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

// C System Files

// C++ System Files
#include <iostream>

// Third Party Includes

// Project Include Files
#include <Common/Math/MathUtils.hpp>

namespace Common::Math
{
    // https://en.wikipedia.org/wiki/Skew-symmetric_matrix#Cross_product
    Eigen::Matrix3d SkewSymmetricCrossProduct(const Eigen::Vector3d& a)
    {
        Eigen::Matrix3d skew_matrix;
        skew_matrix << 0, -a(2), a(1),
                       a(2), 0, -a(0),
                       -a(1), a(0), 0;

        return skew_matrix;
    }

    // Convert Euler RPY -> Rotation Matrix.  TODO: Add support for rotation order
    Eigen::Matrix3d EulerToRotationMatrix(const Eigen::Vector3d &euler)
    {
        Eigen::Matrix3d rotation;
        rotation = Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitX());

        return rotation;
    }

    // Convert Rotation -> Euler RPY
    Eigen::Vector3d RotationMatrixToEuler(const Eigen::Matrix3d& R)
    {
        //Eigen::Vector3d euler;
        // Reverse to order RPY
        return R.eulerAngles(2,1,0).reverse();
    }

    // Convert Euler RPY -> Quaternion.  TODO: Add support for rotation order
    Eigen::Quaterniond EulerToQuaternion(const Eigen::Vector3d& euler)
    {
        return Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitZ()) *
               Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitX());
    }

    // Convert Euler RPY -> Quaternion.  TODO: Add support for rotation order
    Eigen::Vector3d QuaterionToEuler(const Eigen::Quaterniond& q)
    {
        return q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
    }



    // Compute Orientation Error between theta_1 -> theta_2 Euler orientations
    Eigen::Vector3d ComputeOrientationError(const Eigen::Vector3d& theta_1, const Eigen::Vector3d& theta_2)
    {
        Eigen::Quaterniond q_1 = Eigen::AngleAxisd(theta_1(2), Eigen::Vector3d::UnitZ()) *
                                         Eigen::AngleAxisd(theta_1(1), Eigen::Vector3d::UnitY()) *
                                         Eigen::AngleAxisd(theta_1(0), Eigen::Vector3d::UnitX());

        Eigen::Quaterniond q_2 = Eigen::AngleAxisd(theta_2(2), Eigen::Vector3d::UnitZ()) *
                                                 Eigen::AngleAxisd(theta_2(1), Eigen::Vector3d::UnitY()) *
                                                 Eigen::AngleAxisd(theta_2(0), Eigen::Vector3d::UnitX());

        // Compute Orientation Error
        Eigen::Quaterniond q_error = q_2 * q_1.conjugate();

        // Return shortest path orientation error.  The sgn * q_error_W prevents long path tranversals
        // See https://studywolf.wordpress.com/2018/12/03/force-control-of-task-space-orientation/
        return q_error.vec() * Common::Math::sgn(q_error.w());
    }

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
    
} // namespace Common