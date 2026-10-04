/*
#    Copyright (c) 2024-2026 Adorno-Lab
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
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobot.h>
#include <dqrobotics_extensions/robot_constraint_manager/vfi_configuration_file_v3_generator.hpp>
#include <memory>

namespace DQ_robotics_extensions
{

/**
 * @brief The VFICoppeliaSimScene class provides the poses of the objects and the configuration of the robot of a
 *        CoppeliaSim scene to the VFIConfigurationFileV3Generator.
 */
class VFICoppeliaSimScene : public VFISceneInterface
{
protected:
    std::shared_ptr<DQ_robotics::DQ_CoppeliaSimInterface> cs_;
    std::shared_ptr<DQ_robotics::DQ_CoppeliaSimRobot> robot_;

public:
    VFICoppeliaSimScene(const std::shared_ptr<DQ_robotics::DQ_CoppeliaSimInterface>& coppelia_interface,
                        const std::shared_ptr<DQ_robotics::DQ_CoppeliaSimRobot>& coppeliasim_robot);

    DQ_robotics::DQ get_object_pose(const std::string& object_name) override;
    Eigen::VectorXd get_configuration() override;
};

}
