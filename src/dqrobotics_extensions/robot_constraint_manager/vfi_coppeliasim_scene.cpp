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

#include <dqrobotics_extensions/robot_constraint_manager/vfi_coppeliasim_scene.hpp>
#include <stdexcept>

using namespace DQ_robotics;
using namespace Eigen;

namespace DQ_robotics_extensions
{

/**
 * @brief VFICoppeliaSimScene::VFICoppeliaSimScene constructor of the class.
 * @param coppelia_interface The interface connected to CoppeliaSim.
 * @param coppeliasim_robot The robot of the scene.
 */
VFICoppeliaSimScene::VFICoppeliaSimScene(const std::shared_ptr<DQ_CoppeliaSimInterface> &coppelia_interface,
                                         const std::shared_ptr<DQ_CoppeliaSimRobot> &coppeliasim_robot)
    :cs_{coppelia_interface}, robot_{coppeliasim_robot}
{
    if (!cs_)
        throw std::runtime_error("VFICoppeliaSimScene: The CoppeliaSim interface cannot be a null pointer.");
    if (!robot_)
        throw std::runtime_error("VFICoppeliaSimScene: The CoppeliaSim robot cannot be a null pointer.");
}

/**
 * @brief VFICoppeliaSimScene::get_object_pose returns the pose of a CoppeliaSim object.
 * @param object_name The name of the object.
 * @return The pose of the object, expressed in the absolute frame of CoppeliaSim.
 */
DQ VFICoppeliaSimScene::get_object_pose(const std::string &object_name)
{
    return cs_->get_object_pose(object_name);
}

/**
 * @brief VFICoppeliaSimScene::get_configuration returns the current configuration of the CoppeliaSim robot.
 */
VectorXd VFICoppeliaSimScene::get_configuration()
{
    return robot_->get_configuration();
}

}
