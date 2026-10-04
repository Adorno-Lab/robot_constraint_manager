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
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file.hpp>
#include <memory>
#include <string>

namespace DQ_robotics_extensions
{

/**
 * @brief The VFISceneInterface class provides the data of a scene (e.g., a CoppeliaSim scene) required by the
 *        VFIConfigurationFileV3Generator.
 */
class VFISceneInterface
{
public:
    virtual ~VFISceneInterface() = default;

    /**
     * @brief get_object_pose returns the pose of an object of the scene.
     * @param object_name The name of the object.
     * @return The pose of the object, expressed in the reference frame of the scene.
     * @throws std::runtime_error if the object does not exist.
     */
    virtual DQ_robotics::DQ get_object_pose(const std::string& object_name) = 0;

    /**
     * @brief get_configuration returns the current configuration of the robot in the scene.
     */
    virtual Eigen::VectorXd get_configuration() = 0;
};


/**
 * @brief The VFIConfigurationFileV3Generator class creates version 3 configuration files using the poses of the
 *        objects of a scene. The pose of each environment entity is the pose of the scene object with the same name,
 *        and the offset of each robot entity attached to the joint j is
 *
 *                     offset = robot->fkm(q, j).conj() * x_object,
 *
 *        where q is the current configuration of the robot in the scene. See Section 10.1 of the version 3
 *        specification.
 *
 *        The kinematic model must be the same model (including its end effector) used at runtime.
 */
class VFIConfigurationFileV3Generator
{
protected:
    std::shared_ptr<DQ_robotics::DQ_Kinematics> robot_;
    std::shared_ptr<VFISceneInterface> scene_;
    std::string reference_frame_object_name_;

    DQ_robotics::DQ _get_object_pose(const std::string& object_name) const;
    VFIConfigurationFile::POSE _get_environment_entity_pose(const std::string& object_name) const;
    VFIConfigurationFile::POSE _get_robot_entity_offset(const std::string& object_name,
                                                        const int& joint_index,
                                                        const bool& zero_indexed,
                                                        const Eigen::VectorXd& q) const;
    Eigen::VectorXd _get_configuration() const;

public:
    VFIConfigurationFileV3Generator(const std::shared_ptr<DQ_robotics::DQ_Kinematics>& robot,
                                    const std::shared_ptr<VFISceneInterface>& scene,
                                    const std::string& reference_frame_object_name = "");

    VFIConfigurationFile::DOCUMENT_V3 create_from_v2(const VFIConfigurationFile::DOCUMENT_V2& document,
                                                     const std::string& robot_name) const;

    VFIConfigurationFile::DOCUMENT_V3 update_from_scene(const VFIConfigurationFile::DOCUMENT_V3& document) const;
};

}
