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

// Unit tests for the VFIConfigurationFileV3Generator. A fake scene is used; CoppeliaSim is not required.

#include <gtest/gtest.h>
#include <dqrobotics/DQ.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics_extensions/robot_constraint_manager/robot_constraint_manager.hpp>
#include <dqrobotics_extensions/robot_constraint_manager/vfi_configuration_file_v3_generator.hpp>
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_yaml.hpp>
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_v3.hpp>
#include <filesystem>
#include <unordered_map>

using namespace DQ_robotics;
using namespace DQ_robotics_extensions;
using namespace Eigen;

namespace {

using VCF = VFIConfigurationFile;

class FakeScene : public VFISceneInterface
{
public:
    std::unordered_map<std::string, DQ> poses;
    VectorXd q;

    DQ get_object_pose(const std::string& object_name) override
    {
        const auto search = poses.find(object_name);
        if (search == poses.end())
            throw std::runtime_error("Object '" + object_name + "' not found.");
        return search->second;
    }

    VectorXd get_configuration() override
    {
        return q;
    }
};

class VFIConfigurationFileV3GeneratorTest : public testing::Test {
protected:
    std::shared_ptr<DQ_SerialManipulatorMDH> robot_ =
        std::make_shared<DQ_SerialManipulatorMDH>(FrankaEmikaPandaRobot::kinematics());
    std::shared_ptr<FakeScene> scene_ = std::make_shared<FakeScene>();

    const VectorXd q_ = (VectorXd(7) << 0.1, -0.6, 0.2, -2.2, 0.1, 1.7, 0.5).finished();
    const VectorXd q_other_ = (VectorXd(7) << -0.4, 0.3, -0.2, -1.6, 0.4, 2.1, -0.3).finished();

    // Offsets of the robot objects with respect to their joints. They are the values the generator must recover.
    const DQ rotation_ = cos(0.3) + sin(0.3)*i_;
    const DQ rsphere_offset_ = rotation_ + 0.5*E_*(0.05*k_)*rotation_;
    const DQ r_base_sphere_offset_ = 1 + 0.5*E_*(-0.1*k_);
    const DQ obs_sphere_pose_ = 1 + 0.5*E_*(0.4*i_ + 0.2*j_ + 0.6*k_);
    const DQ plane_pose_ = 1 + 0.5*E_*(0.1*k_);

    void SetUp() override
    {
        scene_->q = q_;
        scene_->poses = {
            {"Plane", plane_pose_},
            {"obs_sphere", obs_sphere_pose_},
            {"rsphere", robot_->fkm(q_, 6)*rsphere_offset_},
            {"r_base_sphere", robot_->fkm(q_, 0)*r_base_sphere_offset_},
        };
    }

    static VCF::ENVIRONMENT_TO_ROBOT_DATA environment_to_robot(const std::string& tag,
                                                               const std::string& environment_object,
                                                               const std::string& environment_primitive,
                                                               const std::string& robot_object,
                                                               const int& joint_index)
    {
        VCF::ENVIRONMENT_TO_ROBOT_DATA data;
        data.vfi_type = "ENVIRONMENT_TO_ROBOT";
        data.safe_distance = 0.1;
        data.buffer = 0.01;
        data.vfi_gain = 2.0;
        data.direction = "RESTRICTED_ZONE";
        data.tag = tag;
        data.cs_entity_environment = {environment_object};
        data.cs_entity_robot = {robot_object};
        data.entity_environment_primitive_type = environment_primitive;
        data.entity_robot_primitive_type = "POINT";
        data.robot_index = 1;
        data.joint_index = joint_index;
        return data;
    }

    static VCF::DOCUMENT_V2 make_v2_document()
    {
        VCF::ROBOT_TO_ROBOT_DATA robot_to_robot;
        robot_to_robot.vfi_type = "ROBOT_TO_ROBOT";
        robot_to_robot.safe_distance = 0.3;
        robot_to_robot.vfi_gain = 1.0;
        robot_to_robot.direction = "RESTRICTED_ZONE";
        robot_to_robot.tag = "C3";
        robot_to_robot.cs_entity_one = {"r_base_sphere"};
        robot_to_robot.cs_entity_two = {"rsphere"};
        robot_to_robot.entity_one_primitive_type = "POINT";
        robot_to_robot.entity_two_primitive_type = "POINT";
        robot_to_robot.robot_index_one = 1;
        robot_to_robot.robot_index_two = 1;
        robot_to_robot.joint_index_one = 1;
        robot_to_robot.joint_index_two = 7;

        VCF::DOCUMENT_V2 document;
        document.zero_indexed = false;
        document.vfi_array = {
            environment_to_robot("C1", "Plane", "PLANE", "rsphere", 7),
            environment_to_robot("C2", "obs_sphere", "POINT", "rsphere", 7),
            robot_to_robot,
            // The same object attached to a different joint
            environment_to_robot("C4", "obs_sphere", "POINT", "rsphere", 6),
        };
        return document;
    }

    static const VCF::ENVIRONMENT_ENTITY& find_environment_entity(const VCF::DOCUMENT_V3& document, const std::string& name)
    {
        for (const auto& entity : document.environment_entities)
            if (entity.name == name)
                return entity;
        throw std::runtime_error("Environment entity '" + name + "' not found.");
    }

    static const VCF::ROBOT_ENTITY& find_robot_entity(const VCF::DOCUMENT_V3& document, const std::string& name)
    {
        for (const auto& entity : document.robot_entities)
            if (entity.name == name)
                return entity;
        throw std::runtime_error("Robot entity '" + name + "' not found.");
    }

    static std::string save(const VCF::DOCUMENT_V3& document, const std::string& file_name)
    {
        const std::string path = (std::filesystem::temp_directory_path() / file_name).string();
        VFIConfigurationFileYaml().save_document(document, path);
        return path;
    }
};

//----------------------Constructor----------------------------------------

TEST_F(VFIConfigurationFileV3GeneratorTest, RejectsNullPointers)
{
    EXPECT_THROW(VFIConfigurationFileV3Generator(nullptr, scene_), std::runtime_error);
    EXPECT_THROW(VFIConfigurationFileV3Generator(robot_, nullptr), std::runtime_error);
}

//----------------------create_from_v2----------------------------------------

TEST_F(VFIConfigurationFileV3GeneratorTest, CreateFromV2BuildsEntityTables)
{
    const auto document = VFIConfigurationFileV3Generator(robot_, scene_).create_from_v2(make_v2_document(), "Franka");

    EXPECT_FALSE(document.zero_indexed);
    ASSERT_EQ(document.robots.size(), 1u);
    EXPECT_EQ(document.robots.at(0).robot_index, 1);
    EXPECT_EQ(document.robots.at(0).name, "Franka");
    EXPECT_EQ(document.robots.at(0).dim_configuration, 7);

    ASSERT_EQ(document.environment_entities.size(), 2u);
    EXPECT_EQ(VFIConfigurationFileV3::pose_to_dq(find_environment_entity(document, "Plane").pose), plane_pose_);
    EXPECT_EQ(VFIConfigurationFileV3::pose_to_dq(find_environment_entity(document, "obs_sphere").pose), obs_sphere_pose_);

    ASSERT_EQ(document.robot_entities.size(), 3u);
    const auto rsphere = find_robot_entity(document, "rsphere");
    EXPECT_EQ(rsphere.joint_index, 7);
    EXPECT_EQ(rsphere.robot_index, 1);
    EXPECT_EQ(rsphere.attached_direction, "k_");
    EXPECT_EQ(VFIConfigurationFileV3::pose_to_dq(rsphere.offset), rsphere_offset_);

    const auto r_base_sphere = find_robot_entity(document, "r_base_sphere");
    EXPECT_EQ(r_base_sphere.joint_index, 1);
    EXPECT_EQ(VFIConfigurationFileV3::pose_to_dq(r_base_sphere.offset), r_base_sphere_offset_);

    // The same object attached to the joint 6 becomes another entity
    const auto rsphere_joint6 = find_robot_entity(document, "rsphere_joint6");
    EXPECT_EQ(rsphere_joint6.joint_index, 6);
    EXPECT_EQ(VFIConfigurationFileV3::pose_to_dq(rsphere_joint6.offset),
              robot_->fkm(q_, 5).conj()*scene_->poses.at("rsphere"));
}

TEST_F(VFIConfigurationFileV3GeneratorTest, CreateFromV2MapsVfis)
{
    const auto document = VFIConfigurationFileV3Generator(robot_, scene_).create_from_v2(make_v2_document(), "Franka");
    ASSERT_EQ(document.vfi_array.size(), 4u);

    const auto& c1 = std::get<VCF::ENVIRONMENT_TO_ROBOT_DATA_V3>(document.vfi_array.at(0));
    EXPECT_EQ(c1.tag, "C1");
    EXPECT_EQ(c1.entity_environment, std::vector<std::string>{"Plane"});
    EXPECT_EQ(c1.entity_robot, std::vector<std::string>{"rsphere"});
    EXPECT_EQ(c1.entity_environment_primitive_type, "PLANE");
    EXPECT_DOUBLE_EQ(c1.safe_distance, 0.1);
    EXPECT_DOUBLE_EQ(c1.buffer, 0.01);
    EXPECT_DOUBLE_EQ(c1.vfi_gain, 2.0);
    EXPECT_EQ(c1.direction, "RESTRICTED_ZONE");

    const auto& c3 = std::get<VCF::ROBOT_TO_ROBOT_DATA_V3>(document.vfi_array.at(2));
    EXPECT_EQ(c3.entity_one, std::vector<std::string>{"r_base_sphere"});
    EXPECT_EQ(c3.entity_two, std::vector<std::string>{"rsphere"});

    const auto& c4 = std::get<VCF::ENVIRONMENT_TO_ROBOT_DATA_V3>(document.vfi_array.at(3));
    EXPECT_EQ(c4.entity_robot, std::vector<std::string>{"rsphere_joint6"});
}

TEST_F(VFIConfigurationFileV3GeneratorTest, CreateFromV2FileIsValidAtOtherConfigurations)
{
    // End-to-end: the generated file, loaded by the RobotConstraintManager, describes the scene objects at any
    // configuration, not only at the configuration used to compute the offsets.
    const auto document = VFIConfigurationFileV3Generator(robot_, scene_).create_from_v2(make_v2_document(), "Franka");
    RobotConstraintManager rcm(robot_, std::make_shared<VFIConfigurationFileYaml>(), save(document, "rcm_generator_v2.yaml"));
    rcm.get_inequality_constraints(q_other_, false, false);

    const DQ p_rsphere = (robot_->fkm(q_other_, 6)*rsphere_offset_).translation();
    const DQ p_r_base_sphere = (robot_->fkm(q_other_, 0)*r_base_sphere_offset_).translation();
    EXPECT_NEAR(rcm.get_vfi_distance_error("C2"), vec3(p_rsphere - obs_sphere_pose_.translation()).norm() - 0.1, 1e-12);
    EXPECT_NEAR(rcm.get_vfi_distance_error("C3"), vec3(p_rsphere - p_r_base_sphere).norm() - 0.3, 1e-12);
}

TEST_F(VFIConfigurationFileV3GeneratorTest, CreateFromV2RejectsMultipleRobots)
{
    auto document = make_v2_document();
    std::get<VCF::ENVIRONMENT_TO_ROBOT_DATA>(document.vfi_array.at(0)).robot_index = 2;
    EXPECT_THROW(VFIConfigurationFileV3Generator(robot_, scene_).create_from_v2(document, "Franka"), std::runtime_error);
}

TEST_F(VFIConfigurationFileV3GeneratorTest, CreateFromV2RejectsMissingObject)
{
    scene_->poses.erase("obs_sphere");
    try {
        VFIConfigurationFileV3Generator(robot_, scene_).create_from_v2(make_v2_document(), "Franka");
        FAIL() << "No exception was thrown.";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("obs_sphere"), std::string::npos);
    }
}

TEST_F(VFIConfigurationFileV3GeneratorTest, RejectsConfigurationSizeMismatch)
{
    scene_->q = VectorXd::Zero(6);
    EXPECT_THROW(VFIConfigurationFileV3Generator(robot_, scene_).create_from_v2(make_v2_document(), "Franka"),
                 std::runtime_error);
}

//----------------------Reference frame object----------------------------------------

TEST_F(VFIConfigurationFileV3GeneratorTest, ReferenceFrameObject)
{
    // The robot base is not at the origin of the scene, and the kinematic model is expressed in the base frame.
    const DQ base_rotation = cos(0.4) + sin(0.4)*k_;
    const DQ base = base_rotation + 0.5*E_*(0.2*i_ - 0.1*j_)*base_rotation;
    scene_->poses = {
        {"base", base},
        {"Plane", base*plane_pose_},
        {"obs_sphere", base*obs_sphere_pose_},
        {"rsphere", base*robot_->fkm(q_, 6)*rsphere_offset_},
        {"r_base_sphere", base*robot_->fkm(q_, 0)*r_base_sphere_offset_},
    };

    const auto document = VFIConfigurationFileV3Generator(robot_, scene_, "base").create_from_v2(make_v2_document(), "Franka");
    EXPECT_EQ(VFIConfigurationFileV3::pose_to_dq(find_environment_entity(document, "Plane").pose), plane_pose_);
    EXPECT_EQ(VFIConfigurationFileV3::pose_to_dq(find_robot_entity(document, "rsphere").offset), rsphere_offset_);
}

//----------------------update_from_scene----------------------------------------

TEST_F(VFIConfigurationFileV3GeneratorTest, UpdateFromSceneFillsTemplate)
{
    // The template is the configuration file of the unit tests. Its poses and offsets are replaced.
    VFIConfigurationFileYaml reader;
    reader.load_data(V3_TEST_CONFIG_FILE);
    const auto template_document = std::get<VCF::DOCUMENT_V3>(reader.get_document());

    const DQ cylinder_pose = cos(0.2) + sin(0.2)*j_;
    scene_->poses.emplace("x_inertial", DQ(1));
    scene_->poses.emplace("Cylinder", cylinder_pose);
    scene_->poses.emplace("rline", robot_->fkm(q_, 6));

    const auto document = VFIConfigurationFileV3Generator(robot_, scene_).update_from_scene(template_document);

    EXPECT_EQ(VFIConfigurationFileV3::pose_to_dq(find_environment_entity(document, "Cylinder").pose), cylinder_pose);
    EXPECT_EQ(VFIConfigurationFileV3::pose_to_dq(find_environment_entity(document, "obs_sphere").pose), obs_sphere_pose_);
    EXPECT_EQ(VFIConfigurationFileV3::pose_to_dq(find_robot_entity(document, "rsphere").offset), rsphere_offset_);
    EXPECT_EQ(VFIConfigurationFileV3::pose_to_dq(find_robot_entity(document, "r_base_sphere").offset), r_base_sphere_offset_);
    EXPECT_EQ(VFIConfigurationFileV3::pose_to_dq(find_robot_entity(document, "rline").offset), DQ(1));

    // The rest of the file is not modified
    EXPECT_EQ(find_environment_entity(document, "Cylinder").attached_direction, "j_");
    EXPECT_EQ(find_robot_entity(document, "rsphere").joint_index, 7);
    EXPECT_EQ(document.vfi_array.size(), template_document.vfi_array.size());
    EXPECT_EQ(document.metadata.description, template_document.metadata.description);
    EXPECT_EQ(document.metadata.generated_by, "robot_constraint_manager (VFIConfigurationFileV3Generator)");
}

TEST_F(VFIConfigurationFileV3GeneratorTest, UpdateFromSceneRejectsMissingObject)
{
    VFIConfigurationFileYaml reader;
    reader.load_data(V3_TEST_CONFIG_FILE);
    const auto template_document = std::get<VCF::DOCUMENT_V3>(reader.get_document());
    // x_inertial, Cylinder, and rline are not in the scene
    EXPECT_THROW(VFIConfigurationFileV3Generator(robot_, scene_).update_from_scene(template_document), std::runtime_error);
}

TEST_F(VFIConfigurationFileV3GeneratorTest, UpdateFromSceneRejectsDimConfigurationMismatch)
{
    VFIConfigurationFileYaml reader;
    reader.load_data(V3_TEST_CONFIG_FILE);
    auto template_document = std::get<VCF::DOCUMENT_V3>(reader.get_document());
    template_document.robots.at(0).dim_configuration = 8;
    EXPECT_THROW(VFIConfigurationFileV3Generator(robot_, scene_).update_from_scene(template_document), std::runtime_error);
}

} // namespace
