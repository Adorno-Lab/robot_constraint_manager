/*
#    Copyright (c) 2024 Adorno-Lab
#
#    robot_constraint_manager is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    robot_constraint_manager is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with robot_constraint_manager.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana (email: juanjose.quirozomana@manchester.ac.uk)
#
# ################################################################
*/

#pragma once
#include <dqrobotics_extensions/robot_constraint_manager/checkers.hpp>
#include <dqrobotics_extensions/robot_constraint_manager/conversions.hpp>

#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

using namespace Eigen;

namespace DQ_robotics_extensions {

class Numpy
{
public:
    static MatrixXd vstack(const MatrixXd& A, const MatrixXd& B);
    static MatrixXd hstack(const MatrixXd& A, const MatrixXd& B);
    static MatrixXd block_diag(const std::vector<MatrixXd>& A);
    static MatrixXd block_diag(const std::vector<double>& A);
    static MatrixXd resize(const MatrixXd& A, const int& rows, const int& cols);
    static VectorXd linspace(const double& start, const double& stop, const int& size);
    static MatrixXd linspace(const VectorXd& start, const VectorXd& stop, const int& size);
    static double round(const double& value, const int& digits);
};
}


