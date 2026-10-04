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

#include <dqrobotics_extensions/robot_constraint_manager/vfi_configuration_file_v3_generator.hpp>
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_v3.hpp>
#include <map>
#include <set>
#include <stdexcept>
#include <unordered_map>

using namespace DQ_robotics;
using namespace Eigen;

namespace DQ_robotics_extensions
{

/**
 * @brief VFIConfigurationFileV3Generator::VFIConfigurationFileV3Generator constructor of the class.
 * @param robot The kinematic model used at runtime (including its end effector).
 * @param scene The scene that contains the objects and the robot.
 * @param reference_frame_object_name The name of the scene object whose frame is the frame of the output of
 *        robot->fkm(). If it is empty (default), the poses are expressed in the reference frame of the scene.
 *        For instance, the models returned by the CoppeliaSim robot classes (e.g.,
 *        FrankaEmikaPandaCoppeliaSimZMQRobot::kinematics()) are expressed in the reference frame of the scene.
 */
VFIConfigurationFileV3Generator::VFIConfigurationFileV3Generator(const std::shared_ptr<DQ_Kinematics> &robot,
                                                                 const std::shared_ptr<VFISceneInterface> &scene,
                                                                 const std::string &reference_frame_object_name)
    :robot_{robot}, scene_{scene}, reference_frame_object_name_{reference_frame_object_name}
{
    if (!robot_)
        throw std::runtime_error("VFIConfigurationFileV3Generator: The robot cannot be a null pointer.");
    if (!scene_)
        throw std::runtime_error("VFIConfigurationFileV3Generator: The scene cannot be a null pointer.");
}

/**
 * @brief VFIConfigurationFileV3Generator::_get_object_pose returns the pose of a scene object, expressed in the
 *        frame of the output of robot->fkm().
 * @param object_name The name of the object.
 * @return The normalized pose of the object.
 */
DQ VFIConfigurationFileV3Generator::_get_object_pose(const std::string &object_name) const
{
    DQ x;
    try {
        x = scene_->get_object_pose(object_name);
        if (!reference_frame_object_name_.empty())
            x = scene_->get_object_pose(reference_frame_object_name_).conj()*x;
    } catch (const std::exception& e) {
        throw std::runtime_error("VFIConfigurationFileV3Generator: Cannot obtain the pose of the object '" + object_name +
                                 "' from the scene. " + e.what());
    }
    return normalize(x);
}

/**
 * @brief VFIConfigurationFileV3Generator::_get_environment_entity_pose returns the pose of an environment entity.
 * @param object_name The name of the scene object.
 * @return The pose of the entity.
 */
VFIConfigurationFile::POSE VFIConfigurationFileV3Generator::_get_environment_entity_pose(const std::string &object_name) const
{
    return VFIConfigurationFileV3::dq_to_pose(_get_object_pose(object_name));
}

/**
 * @brief VFIConfigurationFileV3Generator::_get_robot_entity_offset returns the offset of a robot entity with respect
 *        to the frame returned by robot->fkm(q, j).
 * @param object_name The name of the scene object.
 * @param joint_index The joint to which the entity is attached, using the index convention of the file.
 * @param zero_indexed The index convention of the file.
 * @param q The configuration of the robot in the scene.
 * @return The offset of the entity.
 */
VFIConfigurationFile::POSE VFIConfigurationFileV3Generator::_get_robot_entity_offset(const std::string &object_name,
                                                                                    const int &joint_index,
                                                                                    const bool &zero_indexed,
                                                                                    const VectorXd &q) const
{
    const int j = zero_indexed ? joint_index : joint_index - 1;
    const DQ offset = robot_->fkm(q, j).conj()*_get_object_pose(object_name);
    return VFIConfigurationFileV3::dq_to_pose(normalize(offset));
}

/**
 * @brief VFIConfigurationFileV3Generator::_get_configuration returns the configuration of the robot in the scene.
 */
VectorXd VFIConfigurationFileV3Generator::_get_configuration() const
{
    const VectorXd q = scene_->get_configuration();
    if (q.size() != robot_->get_dim_configuration_space())
        throw std::runtime_error("VFIConfigurationFileV3Generator: The robot in the scene has " + std::to_string(q.size()) +
                                 " DoF, but the kinematic model has " +
                                 std::to_string(robot_->get_dim_configuration_space()) + " DoF.");
    return q;
}

/**
 * @brief VFIConfigurationFileV3Generator::create_from_v2 converts a version 2 configuration file into a version 3
 *        configuration file (see Section 10.1 of the version 3 specification).
 *        Each CoppeliaSim object of the version 2 file becomes an entity whose name is the name of the object.
 *        If an object is attached to different joints in different VFIs, one robot entity is created per joint,
 *        and the name of the additional entities is "<object>_joint<joint_index>".
 *        The attached directions are "k_", which is the behavior of version 2.
 * @param document The version 2 configuration file.
 * @param robot_name The name of the robot (informational).
 * @return The version 3 configuration file. It is validated before it is returned.
 */
VFIConfigurationFile::DOCUMENT_V3 VFIConfigurationFileV3Generator::create_from_v2(const VFIConfigurationFile::DOCUMENT_V2 &document,
                                                                                const std::string &robot_name) const
{
    using VCF = VFIConfigurationFile;
    const bool zero_indexed = document.zero_indexed;
    const int first_index = zero_indexed ? 0 : 1;
    const VectorXd q = _get_configuration();

    VCF::DOCUMENT_V3 output;
    output.zero_indexed = zero_indexed;
    output.metadata.generated_by = "robot_constraint_manager (VFIConfigurationFileV3Generator)";
    output.metadata.source = "Migrated from a version 2 configuration file";
    output.robots = {{first_index, robot_name, robot_->get_dim_configuration_space()}};

    // Entity names are unique across both tables.
    std::set<std::string> used_names;
    auto get_unique_name = [&used_names](const std::string& name, const std::string& suffix)
    {
        std::string unique_name = name;
        if (used_names.count(unique_name))
            unique_name = name + suffix;
        for (int i = 2; used_names.count(unique_name); i++)
            unique_name = name + suffix + "_" + std::to_string(i);
        used_names.insert(unique_name);
        return unique_name;
    };

    auto check_robot_index = [first_index](const int& robot_index, const std::string& tag)
    {
        if (robot_index != first_index)
            throw std::runtime_error("VFIConfigurationFileV3Generator::create_from_v2: The VFI " + tag + " uses the robot_index " +
                                     std::to_string(robot_index) + ". Version 3 supports a single robot, whose robot_index is " +
                                     std::to_string(first_index) + ".");
    };

    std::unordered_map<std::string, std::string> environment_entity_names; // object -> entity
    auto add_environment_entities = [&](const std::vector<std::string>& objects)
    {
        std::vector<std::string> names;
        for (const auto& object : objects)
        {
            auto search = environment_entity_names.find(object);
            if (search == environment_entity_names.end())
            {
                const std::string name = get_unique_name(object, "_environment");
                output.environment_entities.push_back({name, _get_environment_entity_pose(object), "k_"});
                search = environment_entity_names.emplace(object, name).first;
            }
            names.push_back(search->second);
        }
        return names;
    };

    std::map<std::pair<std::string, int>, std::string> robot_entity_names; // (object, joint_index) -> entity
    auto add_robot_entities = [&](const std::vector<std::string>& objects, const int& joint_index)
    {
        std::vector<std::string> names;
        for (const auto& object : objects)
        {
            const auto key = std::make_pair(object, joint_index);
            auto search = robot_entity_names.find(key);
            if (search == robot_entity_names.end())
            {
                const std::string name = get_unique_name(object, "_joint" + std::to_string(joint_index));
                output.robot_entities.push_back({name, first_index, joint_index,
                                                 _get_robot_entity_offset(object, joint_index, zero_indexed, q), "k_"});
                search = robot_entity_names.emplace(key, name).first;
            }
            names.push_back(search->second);
        }
        return names;
    };

    for (const auto& data_item : document.vfi_array)
    {
        std::visit([&](const auto& arg){
            using T = std::decay_t<decltype(arg)>;
            if constexpr (std::is_same_v<T, VCF::ENVIRONMENT_TO_ROBOT_DATA>) {
                check_robot_index(arg.robot_index, arg.tag);
                VCF::ENVIRONMENT_TO_ROBOT_DATA_V3 vfi;
                static_cast<VCF::BASE_DATA&>(vfi) = arg;
                vfi.entity_environment = add_environment_entities(arg.cs_entity_environment);
                vfi.entity_robot = add_robot_entities(arg.cs_entity_robot, arg.joint_index);
                vfi.entity_environment_primitive_type = arg.entity_environment_primitive_type;
                vfi.entity_robot_primitive_type = arg.entity_robot_primitive_type;
                output.vfi_array.push_back(vfi);
            } else {
                check_robot_index(arg.robot_index_one, arg.tag);
                check_robot_index(arg.robot_index_two, arg.tag);
                VCF::ROBOT_TO_ROBOT_DATA_V3 vfi;
                static_cast<VCF::BASE_DATA&>(vfi) = arg;
                vfi.entity_one = add_robot_entities(arg.cs_entity_one, arg.joint_index_one);
                vfi.entity_two = add_robot_entities(arg.cs_entity_two, arg.joint_index_two);
                vfi.entity_one_primitive_type = arg.entity_one_primitive_type;
                vfi.entity_two_primitive_type = arg.entity_two_primitive_type;
                output.vfi_array.push_back(vfi);
            }
        }, data_item);
    }

    VFIConfigurationFileV3::validate(output);
    return output;
}

/**
 * @brief VFIConfigurationFileV3Generator::update_from_scene updates the pose of every environment entity and the
 *        offset of every robot entity of a version 3 configuration file, using the scene objects with the same names.
 *        The rest of the file (e.g., the joint indexes, the attached directions, and the VFIs) is not modified,
 *        except the field generated_by of the metadata.
 *        This method can be used to create configuration files from a template, in which the poses and offsets are
 *        placeholders.
 * @param document The version 3 configuration file.
 * @return The updated configuration file. It is validated before it is returned.
 */
VFIConfigurationFile::DOCUMENT_V3 VFIConfigurationFileV3Generator::update_from_scene(const VFIConfigurationFile::DOCUMENT_V3 &document) const
{
    VFIConfigurationFileV3::validate(document);
    const int dim_configuration = document.robots.at(0).dim_configuration;
    if (dim_configuration != robot_->get_dim_configuration_space())
        throw std::runtime_error("VFIConfigurationFileV3Generator::update_from_scene: The configuration file defines a robot with " +
                                 std::to_string(dim_configuration) + " DoF, but the kinematic model has " +
                                 std::to_string(robot_->get_dim_configuration_space()) + " DoF.");

    const VectorXd q = _get_configuration();
    VFIConfigurationFile::DOCUMENT_V3 output = document;
    output.metadata.generated_by = "robot_constraint_manager (VFIConfigurationFileV3Generator)";
    for (auto& entity : output.environment_entities)
        entity.pose = _get_environment_entity_pose(entity.name);
    for (auto& entity : output.robot_entities)
        entity.offset = _get_robot_entity_offset(entity.name, entity.joint_index, output.zero_indexed, q);

    VFIConfigurationFileV3::validate(output);
    return output;
}

}
