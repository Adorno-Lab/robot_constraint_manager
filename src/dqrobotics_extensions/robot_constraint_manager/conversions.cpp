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

#include <dqrobotics_extensions/robot_constraint_manager/conversions.hpp>

namespace DQ_robotics_extensions {

/**
 * @brief Conversions::std_vector_double_to_vectorxd converts a vector to an Eigen vector
 * @param std_vector
 * @return The desired Eigen vector.
 */
Eigen::VectorXd Conversions::std_vector_double_to_vectorxd(std::vector<double> &std_vector)
{
    return Eigen::Map<VectorXd>(std_vector.data(), std_vector.size());
}

/**
 * @brief Conversions::std_vector_vectorxd_to_vectorxd converts a vector of vectors to an Eigen vector
 * @param std_vectorxd
 * @return
 */
VectorXd Conversions::std_vector_vectorxd_to_vectorxd(std::vector<VectorXd> &std_vectorxd)
{
    VectorXd q;
    for (auto &vec : std_vectorxd)
        q = DQ_robotics_extensions::Numpy::vstack(q, vec);
    return q;
}

/**
 * @brief Conversions::double2vector creates a vector of the specified size with all their elements
 *                                   initialized in a specific value.
 * @param value The value to initialize all vector elements.
 * @param size The desired size of the vector
 * @return The desired Eigen vector.
 */
VectorXd Conversions::double2vector(const double &value, const int &size)
{
    std::vector<double> aux(size, value);
    return std_vector_double_to_vectorxd(aux);
}

}
