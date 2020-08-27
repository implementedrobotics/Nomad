/*
 * MathUtils.hpp
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

#ifndef NOMAD_COMMON_MATHUTILS_H
#define NOMAD_COMMON_MATHUTILS_H

// C System Files
#include <math.h>

// C++ System Files

// Third Party Includes
#include <Eigen/Dense>
// Project Include Files

namespace Common::Math
{
    template <typename T>
    int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }

    // https://en.wikipedia.org/wiki/Skew-symmetric_matrix#Cross_product
    Eigen::Matrix3d SkewSymmetricCrossProduct(const Eigen::Vector3d& a);

    void rpyToR(Eigen::Matrix3d &R, double *rpy_in);

        // Convert Rotation -> Euler RPY
    Eigen::Vector3d RotationMatrixToEuler(const Eigen::Matrix3d& R);

    // Convert Euler RPY -> Rotation Matrix.  TODO: Add support for rotation order
    Eigen::Matrix3d EulerToRotationMatrix(const Eigen::Vector3d& euler);
    
    // Convert Euler RPY -> Quaternion.  TODO: Add support for rotation order
    Eigen::Quaterniond EulerToQuaternion(const Eigen::Vector3d& euler);

    // Convert Euler RPY -> Quaternion.  TODO: Add support for rotation order
    Eigen::Vector3d QuaterionToEuler(const Eigen::Quaterniond& q);

    // Compute Orientation Error between 2 Euler orientations
    Eigen::Vector3d ComputeOrientationError(const Eigen::Vector3d& theta_1, const Eigen::Vector3d& theta_2);
    
    Eigen::Matrix3d RotationX(double rad);
    Eigen::Matrix3d RotationY(double rad);
    Eigen::Matrix3d RotationZ(double rad);

    namespace EigenHelpers
    {

        // Typedefs
        typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMatrixXd;
        // TODO: Make this more generic
        // Block Matrix Class
        class BlockMatrixXd
        {
        public:
            BlockMatrixXd() {}
            BlockMatrixXd(const unsigned int Rows, const unsigned int Cols, const unsigned int BlockHeight, const unsigned int BlockWidth, const int init_val);
            BlockMatrixXd(const unsigned int Rows, const unsigned int Cols, const unsigned int BlockHeight, const unsigned int BlockWidth, Eigen::MatrixXd &matrix);
            BlockMatrixXd(const unsigned int Rows, const unsigned int Cols, const unsigned int BlockHeight, const unsigned int BlockWidth);

            friend std::ostream &operator<<(std::ostream &os, BlockMatrixXd const &bm) { return os << bm.Matrix_; }
            // Operator Overload:
            void operator()(const unsigned int Row, const unsigned int Col, const Eigen::MatrixXd &block_val);

            // TODO: Fix Templated Hack so we can set this directly... i.e. matrix(1,1) = block
            Eigen::Block<Eigen::MatrixXd, -1, -1, false> operator()(const unsigned int Row, const unsigned int Col);

            // Cast Overload
            operator Eigen::MatrixXd() { return Matrix_; }

            Eigen::MatrixXd MatrixXd() const { return Matrix_; }
            // Set Block Matrix Value
            // TODO: Should be Matrix(1,1) = value
            void SetBlock(const unsigned int Row, const unsigned int Col, const Eigen::MatrixXd &block_val);

            // Fill Diagonal with a block
            void FillDiagonal(const Eigen::MatrixXd &block_val, const int k = 0);

        protected:
            unsigned int Rows_;
            unsigned int Cols_;
            unsigned int BlockWidth_;
            unsigned int BlockHeight_;

            Eigen::MatrixXd Matrix_;

        };
    } // namespace EigenHelpers
    
    
} // namespace Common::Math
#endif // NOMAD_COMMON_STATISTICS_H