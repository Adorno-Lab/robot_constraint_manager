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

#include <dqrobotics_extensions/robot_constraint_manager/utils.hpp>

namespace DQ_robotics_extensions{

/**
 * @brief Capybara::CVectorXd
 * @param vec
 * @return
 */
VectorXd CVectorXd(const std::vector<double> &vec)
{
    std::vector<double> myvec = vec;
    return DQ_robotics_extensions::Conversions::std_vector_double_to_vectorxd(myvec);
}

/**
 * @brief Capybara::CMatrixXd
 * @param mat
 * @return
 */
MatrixXd CMatrixXd(const std::vector<std::vector<double>>& mat)
{
    int rows = mat.size();
    std::vector<int> sizes(rows, 0);
    for (auto i=0;i<rows;i++)
        sizes.at(i) = mat.at(i).size();

    if (!DQ_robotics_extensions::Checkers::check_equal_elements(sizes, DQ_robotics_extensions::Checkers::MODE::DO_NOT_PANIC))
        throw std::runtime_error("Panic with CMatrixXd(). Wrong number of elements. "
                                 "All vector must have the same dimension.  ");

    int cols = sizes.at(0);
    MatrixXd output = MatrixXd(rows,cols);
    auto mymat = mat;
    for (auto i=0;i<rows;i++)
        output.row(i) = DQ_robotics_extensions::Conversions::std_vector_double_to_vectorxd(mymat.at(i));

    return output;
}

void delay(const int &seconds)
{
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
}

void microdelay(const int &microseconds)
{
    std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
}

void millidelay(const int &milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

/**
 * @brief get_planar_joint_configuration_from_pose given a unit dual quaternion that represents
 *              the pose of a planar joint, this method returns the planar joint configuration
 *              q = [x, y, phi]. This method assumes that the z-axis of the frame represented by
 *              the pose is collinear with the z-axis of the planar joint. Furthermore, the planar joint
 *              can rotate around its z-axis and translate in the XY plane.
 * @param pose A unit dual quaternion
 * @return the desired configuration
 */
VectorXd get_planar_joint_configuration_from_pose(const DQ &pose)
{
    DQ x = pose;
    VectorXd axis = x.rotation_axis().vec4();
    // To ensure that the z-axis is always positive.
    if (axis(3)<0)
        x = -x;
    VectorXd p = x.translation().vec3();
    double rangle = x.P().rotation_angle();
    return (VectorXd(3)<< p(0), p(1), rangle).finished();
}


/**
 * @brief get_planar_joint_configuration_velocities_at_body_frame returns the planar joint configuration velocities expressed in
 *                          the body frame.
 * @param body_frame_pose The unit dual quaternion that represents the pose of the body frame.
 * @param planar_joint_velocities_at_inertial_frame The planar joint configuration velocities wrt to the inertial frame.
 * @return The desired planar joint configuration velocities.
 */
VectorXd get_planar_joint_configuration_velocities_at_body_frame(const DQ &body_frame_pose,
                                                   const VectorXd &planar_joint_velocities_at_inertial_frame)
{
    const VectorXd& ua = planar_joint_velocities_at_inertial_frame; // x_dot, y_dot, phi_dot
    const DQ p_dot_a_ab = ua(0)*i_ + ua(1)*j_;
    const DQ w_a_ab = ua(2)*k_;
    // Build the Twist_a, which represents the velocities in the inertial frame
    const DQ twist_a = w_a_ab + E_*(p_dot_a_ab + DQ_robotics::cross(body_frame_pose.translation(), w_a_ab));

    // Twist_a expressed in the body frame is given as
    const DQ twist_b = Ad(body_frame_pose.conj(), twist_a);
    const VectorXd twist_b_vec = twist_b.vec6(); // [0 0 wb xb_dot yb_dot 0]
                                                 //[xb_dot         yb_dot        wb]
    return DQ_robotics_extensions::CVectorXd({twist_b_vec(3), twist_b_vec(4), twist_b_vec(2)});
}

}
