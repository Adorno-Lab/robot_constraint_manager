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
#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif


using namespace Eigen;

namespace DQ_robotics_extensions{

class Checkers
{
public:
    enum class MODE{PANIC, DO_NOT_PANIC};
    static bool check_column_matrix_sizes(const MatrixXd& A,
                                          const MatrixXd& B,
                                          const Checkers::MODE& mode = Checkers::MODE::PANIC);

    static bool check_row_matrix_sizes(const MatrixXd& A,
                                       const MatrixXd& B,
                                       const MODE& mode = Checkers::MODE::PANIC);


    static bool check_constraint_sizes(const MatrixXd& A,
                                       const VectorXd& b,
                                       const double& optimization_vector_size,
                                       const MODE& mode = Checkers::MODE::PANIC);

    /**
     * @brief is_string checks if the input is a std::string
     * @param input
     * @return True if the input is a std::string. False otherwise.
     */
    template<typename T>
    static bool is_string(const T& input){
        const std::string msg{};
        return typeid(input) == typeid(msg);
    }


    template<typename T>
    static bool check_equal_elements(const T& vector,
                                     const MODE& mode = Checkers::MODE::PANIC)
    {
        if (std::all_of(vector.cbegin(), vector.cend(), [vector](int i) { return i == vector.at(0); }))
        {
            return true;
        }else{
            if (mode == Checkers::MODE::PANIC)
            {
                throw std::runtime_error("Panic with Capybara::check_for_equal_elements(vector). The vector containts different elements. ");
            }
            return false;
        }
    }

    template<typename T, typename U>
    static bool check_equal_sizes(const T &v1,
                                  const U &v2,
                                  const MODE& mode = Checkers::MODE::PANIC,
                                  const std::string& error_message = "")
    {
        std::size_t s1 = static_cast<std::size_t>(v1.size());
        std::size_t s2 = static_cast<std::size_t>(v2.size());
        if (s1 != s2)
        {
            if (mode == Checkers::MODE::PANIC)
                throw std::runtime_error("Panic with DQ_robotics::Checkers::check_equal_sizes(v1, v2). Both containers have diferent sizes. "
                                         + error_message);
            return false;
        }
        return true;
    }


    template<typename T, typename U, typename V>
    static bool check_equal_sizes(const T &v1,
                                  const U &v2,
                                  const V &v3,
                                  const MODE& mode = Checkers::MODE::PANIC)
    {
        bool s1 = Checkers::check_equal_sizes(v1, v2, mode);
        bool s2 = Checkers::check_equal_sizes(v2, v3, mode);
        if (s1 != s2)
        {
            return false;
        }
        return true;
    }





};


}


