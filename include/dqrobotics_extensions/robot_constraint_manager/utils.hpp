/*
#    Copyright (c) 2024 Juan Jose Quiroz Omana
#
#    Capybara_toolkit is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    Capybara_toolkit is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with Capybara_toolkit.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana, (email: juanjqogm@gmail.com)
#
# ################################################################
*/

#pragma once
#include <dqrobotics_extensions/robot_constraint_manager/conversions.hpp>
#include <dqrobotics_extensions/robot_constraint_manager/checkers.hpp>
#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif
#include <thread>
#include <iostream>

using namespace Eigen;

namespace DQ_robotics_extensions {

VectorXd CVectorXd(const std::vector<double>& vec);
MatrixXd CMatrixXd(const std::vector<std::vector<double>>& mat);

void millidelay(const int& milliseconds);
void microdelay(const int& microseconds);
void delay(const int& seconds);

}

