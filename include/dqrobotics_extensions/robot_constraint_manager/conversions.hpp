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
#include <vector>
#include <dqrobotics_extensions/robot_constraint_manager/numpy.hpp>
#include <dqrobotics/utils/DQ_Constants.h>

#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

using namespace Eigen;
using namespace DQ_robotics;
namespace DQ_robotics_extensions{

class Conversions
{
public:
    static VectorXd std_vector_double_to_vectorxd(std::vector<double> &std_vector);
    static VectorXd vectorxd_to_std_vector_double(std::vector<double> &std_vector);
    static VectorXd std_vector_vectorxd_to_vectorxd(std::vector<VectorXd>& std_vectorxd);
    static VectorXd double2vector(const double& value, const int& size);

};
}

