/*
#    Copyright (c) 2024-2025 Adorno-Lab
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
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobot.h>
#include <dqrobotics_extensions/robot_constraint_manager/vfi_manager.hpp>
#include <dqrobotics/utils/DQ_Math.h>
#include <memory>
#include <unordered_map>

using namespace Eigen;
using namespace DQ_robotics;

namespace DQ_robotics_extensions
{


class RobotConstraintManager
{
private:
    class Impl;
    std::shared_ptr<Impl> impl_;

protected:
    struct VFI_BUILD_DATA{
        VFI_Framework::VFI_MODE vfi_mode;
        VFI_Framework::VFI_TYPE vfi_type;
        VFI_Framework::DIRECTION direction;
        double safe_distance;
        double vfi_gain;
        int joint_index_one;
        int joint_index_two;
        DQ primitive_offset_one;
        DQ primitive_offset_two;
        DQ robot_attached_direction;
        DQ environment_attached_direction;
        DQ workspace_derivative;
        DQ cs_entity_environment_pose;
        std::string tag;
    };
    std::vector<VFI_BUILD_DATA> vfi_build_data_list_;
    std::unordered_map<std::string, VFI_BUILD_DATA> vfi_build_data_map_;

protected:
    std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> cs_;
    std::string config_path_;
    VFI_Framework::LEVEL level_;
    std::shared_ptr<DQ_Kinematics> robot_;
    std::shared_ptr<DQ_CoppeliaSimRobot> coppelia_robot_;
    std::shared_ptr<DQ_robotics_extensions::VFI_manager> VFI_M_;
    double configuration_limit_constraint_gain_;

    VectorXd initial_robot_configuration_;
    int number_of_constraints_;

    bool verbosity_{true};
    DQ _get_robot_primitive_offset_from_coppeliasim(const std::string& object_name, const int& joint_index);
    void _initial_settings();
    void _set_vfi_configuration_constraints_gain(const double& vfi_position_constraints_gain);

    void _check_unit(const std::string& unit);
public:
    RobotConstraintManager(const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ>& coppelia_interface,
                           const std::shared_ptr<DQ_CoppeliaSimRobot>& coppeliasim_robot,
                           const std::shared_ptr<DQ_Kinematics>& robot,
                           const std::string &yaml_file_path,
                           const bool& verbosity = false,
                           const VFI_manager::LEVEL& level = VFI_manager::LEVEL::VELOCITIES);


    int get_number_of_vfi_constraints() const;


    std::tuple<MatrixXd, VectorXd> get_inequality_constraints(const VectorXd& q);

    std::tuple<double, double, double, double, double, std::string> get_vfi_log_data(const std::string &tag) const;
    std::tuple<int, DQ, int, DQ> get_primitive_index_and_offset(const std::string& tag) const;

    std::vector<std::string> get_vfi_tags() const;
    double get_vfi_distance_error(const std::string& tag) const;
    double get_line_to_line_angle(const std::string& tag) const;
    void show_vfi_build_data(const std::string& tag) const;
    std::tuple<VectorXd, VectorXd> get_configuration_limits() const;
    std::tuple<VectorXd, VectorXd> get_configuration_velocity_limits() const;
    void set_configuration_limits(const std::tuple<VectorXd, VectorXd>& configuration_limits);
    void set_configuration_velocity_limits(const std::tuple<VectorXd, VectorXd> &configuration_velocity_limits);

};
}
