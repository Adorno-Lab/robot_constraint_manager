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
#include <dqrobotics_extensions/robot_constraint_manager/numpy.hpp>


#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

using namespace Eigen;

namespace DQ_robotics_extensions {


class ConstraintsManager
{
protected:
    int dim_configuration_;
    MatrixXd equality_constraint_matrix_ = MatrixXd::Zero(0,0);
    VectorXd equality_constraint_vector_ = VectorXd::Zero(0);
    MatrixXd inequality_constraint_matrix_ = MatrixXd::Zero(0,0);
    VectorXd inequality_constraint_vector_ = VectorXd::Zero(0);

    void _reset_equality_constraints();
    void _reset_inequality_constraints();

public:
    ConstraintsManager() = delete;
    ConstraintsManager(const int& dim_configuration);

    void add_equality_constraint(const MatrixXd& Aeq, const VectorXd& beq);
    void add_inequality_constraint(const MatrixXd& A, const VectorXd& b);

    std::tuple<MatrixXd, VectorXd> get_equality_constraints(const bool& delete_equality_constraints = true);
    std::tuple<MatrixXd, VectorXd> get_inequality_constraints(const bool& delete_inequality_constraints = true);


};

}

