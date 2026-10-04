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

// Unit tests for version 3 configuration files. CoppeliaSim is not required.

#include <gtest/gtest.h>
#include <dqrobotics/DQ.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics_extensions/robot_constraint_manager/robot_constraint_manager.hpp>
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_yaml.hpp>
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_v3.hpp>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>

using namespace DQ_robotics;
using namespace DQ_robotics_extensions;
using namespace Eigen;

namespace {

using VCF = VFIConfigurationFile;

class RobotConstraintManagerV3Test : public testing::Test {
protected:
    const std::string config_file_ = V3_TEST_CONFIG_FILE;
    std::shared_ptr<DQ_SerialManipulatorMDH> robot_ =
        std::make_shared<DQ_SerialManipulatorMDH>(FrankaEmikaPandaRobot::kinematics());
    const VectorXd q_home_ = (VectorXd(7) << 0, -M_PI_4, 0, -3*M_PI_4, 0, M_PI_2, M_PI_4).finished();

    // Values of the configuration file
    const DQ rsphere_offset_ = 1 + 0.5*E_*(0.05*k_);
    const DQ r_base_sphere_offset_ = 1 + 0.5*E_*(-0.1*k_);
    const DQ obs_sphere_position_ = 0.4*i_ + 0.2*j_ + 0.6*k_;

    std::shared_ptr<RobotConstraintManager> make_rcm(const std::string& config_file) const
    {
        return std::make_shared<RobotConstraintManager>(robot_, std::make_shared<VFIConfigurationFileYaml>(),
                                                        config_file);
    }

    std::shared_ptr<RobotConstraintManager> make_rcm() const
    {
        return make_rcm(config_file_);
    }

    // Writes a copy of the configuration file, in which the text old_text is replaced by new_text.
    std::string write_modified_config(const std::string& file_name,
                                      const std::string& old_text,
                                      const std::string& new_text) const
    {
        std::ifstream in(config_file_);
        std::stringstream buffer;
        buffer << in.rdbuf();
        std::string content = buffer.str();
        const auto pos = content.find(old_text);
        if (pos == std::string::npos)
            throw std::runtime_error("write_modified_config: '" + old_text + "' not found.");
        content.replace(pos, old_text.size(), new_text);
        return write_file(file_name, content);
    }

    static std::string write_file(const std::string& file_name, const std::string& content)
    {
        const auto path = std::filesystem::temp_directory_path() / file_name;
        std::ofstream(path) << content;
        return path.string();
    }

    DQ rsphere_position(const VectorXd& q) const
    {
        return (robot_->fkm(q, 6)*rsphere_offset_).translation();
    }

    DQ r_base_sphere_position(const VectorXd& q) const
    {
        return (robot_->fkm(q, 0)*r_base_sphere_offset_).translation();
    }

    static int count_occurrences(const std::string& text, const std::string& pattern)
    {
        int count = 0;
        for (auto pos = text.find(pattern); pos != std::string::npos; pos = text.find(pattern, pos + 1))
            count++;
        return count;
    }
};

//----------------------Constructor----------------------------------------

TEST_F(RobotConstraintManagerV3Test, LoadsVersion3File)
{
    const auto rcm = make_rcm();
    EXPECT_EQ(rcm->get_number_of_vfi_constraints(), 6);

    auto tags = rcm->get_vfi_tags();
    std::sort(tags.begin(), tags.end());
    EXPECT_EQ(tags, (std::vector<std::string>{"C1", "C2", "C3", "C4", "C5", "C6"}));
}

TEST_F(RobotConstraintManagerV3Test, RejectsVersion2File)
{
    const std::string v2_file = write_file("rcm_v3_test_v2.yaml",
                                           "vfi_file_version: 2\n"
                                           "zero_indexed: false\n"
                                           "vfi_array:\n"
                                           "    -\n"
                                           "        vfi_type: \"ENVIRONMENT_TO_ROBOT\"\n"
                                           "        cs_entity_environment: [\"Plane\"]\n"
                                           "        cs_entity_robot: [\"rsphere\"]\n"
                                           "        entity_environment_primitive_type: \"PLANE\"\n"
                                           "        entity_robot_primitive_type: \"POINT\"\n"
                                           "        robot_index: 1\n"
                                           "        joint_index: 7\n"
                                           "        safe_distance: 0.05\n"
                                           "        vfi_gain: 1.0\n"
                                           "        direction: \"RESTRICTED_ZONE\"\n"
                                           "        tag: \"C1\"\n");
    EXPECT_THROW(make_rcm(v2_file), std::runtime_error);
}

TEST_F(RobotConstraintManagerV3Test, RejectsDimConfigurationMismatch)
{
    const std::string file = write_modified_config("rcm_v3_test_dim.yaml",
                                                   "dim_configuration: 7", "dim_configuration: 8");
    EXPECT_THROW(make_rcm(file), std::runtime_error);
}

TEST_F(RobotConstraintManagerV3Test, RejectsInvalidDocument)
{
    const std::string file = write_modified_config("rcm_v3_test_invalid.yaml",
                                                   "entity_robot: [\"rline\"]", "entity_robot: [\"missing\"]");
    EXPECT_THROW(make_rcm(file), std::runtime_error);
}

TEST_F(RobotConstraintManagerV3Test, RejectsNullPointers)
{
    EXPECT_THROW(RobotConstraintManager(nullptr, std::make_shared<VFIConfigurationFileYaml>(), config_file_),
                 std::runtime_error);
    EXPECT_THROW(RobotConstraintManager(robot_, nullptr, config_file_), std::runtime_error);
}

//----------------------Build data----------------------------------------

TEST_F(RobotConstraintManagerV3Test, BuildsEnvironmentToRobotData)
{
    const auto rcm = make_rcm();

    const auto c5 = rcm->get_vfi_build_data("C5");
    EXPECT_EQ(c5.vfi_type, VFI_Framework::VFI_TYPE::ENVIRONMENT_TO_ROBOT);
    EXPECT_EQ(c5.vfi_class, VFI_Framework::VFI_CLASS::RPOINT_TO_POINT);
    EXPECT_EQ(c5.direction, VFI_Framework::DIRECTION::RESTRICTED_ZONE);
    EXPECT_EQ(c5.joint_index_one, 6);  // joint_index 7 in a one-indexed file
    EXPECT_EQ(c5.robot_index_one, 0);
    EXPECT_EQ(c5.primitive_offsets_one.at(0), rsphere_offset_);
    // obs_sphere is written as a unit dual quaternion (8 coefficients)
    EXPECT_EQ(c5.environment_poses.at(0), 1 + 0.5*E_*obs_sphere_position_);

    // Attached directions are taken from the entities
    EXPECT_EQ(rcm->get_vfi_build_data("C4").environment_attached_direction, j_);
    EXPECT_EQ(rcm->get_vfi_build_data("C3").environment_attached_direction, k_);
    EXPECT_EQ(rcm->get_vfi_build_data("C1").vfi_class, VFI_Framework::VFI_CLASS::RLINE_TO_LINE_ANGLE);
}

TEST_F(RobotConstraintManagerV3Test, BuildsRobotToRobotData)
{
    const auto c2 = make_rcm()->get_vfi_build_data("C2");
    EXPECT_EQ(c2.vfi_type, VFI_Framework::VFI_TYPE::ROBOT_TO_ROBOT);
    EXPECT_EQ(c2.joint_index_one, 0);
    EXPECT_EQ(c2.joint_index_two, 6);
    EXPECT_EQ(c2.primitive_offsets_one.at(0), r_base_sphere_offset_);
    EXPECT_EQ(c2.primitive_offsets_two.at(0), rsphere_offset_);
}

TEST_F(RobotConstraintManagerV3Test, DistanceErrorsMatchManualComputation)
{
    const auto rcm = make_rcm();
    const auto [A, b] = rcm->get_inequality_constraints(q_home_, false, false);
    EXPECT_EQ(A.rows(), 6);
    EXPECT_EQ(b.size(), 6);

    const DQ p = rsphere_position(q_home_);
    const DQ p_base = r_base_sphere_position(q_home_);
    const Vector4d pv = vec4(p);

    // C2: robot point to robot point
    EXPECT_NEAR(rcm->get_vfi_distance_error("C2"), vec3(p - p_base).norm() - 0.3, 1e-12);
    // C3: point to the plane z = 0.1, whose normal is k_
    EXPECT_NEAR(rcm->get_vfi_distance_error("C3"), (pv(3) - 0.1) - 0.05, 1e-12);
    // C4: point to the line through (0.5, 0, 0) along j_
    EXPECT_NEAR(rcm->get_vfi_distance_error("C4"), std::hypot(pv(1) - 0.5, pv(3)) - 0.1, 1e-12);
    // C5: point to point
    EXPECT_NEAR(rcm->get_vfi_distance_error("C5"), vec3(p - obs_sphere_position_).norm() - 0.1, 1e-12);
}

//----------------------Getters----------------------------------------

TEST_F(RobotConstraintManagerV3Test, SharedFieldGetters)
{
    const auto rcm = make_rcm();
    EXPECT_EQ(rcm->get_vfi_type("C2"), "ROBOT_TO_ROBOT");
    EXPECT_EQ(rcm->get_vfi_direction("C1"), "SAFE_ZONE");
    EXPECT_DOUBLE_EQ(rcm->get_safe_distance("C4"), 0.1);
    EXPECT_DOUBLE_EQ(rcm->get_buffer("C4"), 0.01);
    EXPECT_DOUBLE_EQ(rcm->get_vfi_gain("C4"), 2.0);
}

TEST_F(RobotConstraintManagerV3Test, EntityNameGetters)
{
    const auto rcm = make_rcm();
    EXPECT_EQ(rcm->get_entity_one_or_entity_environment_names("C3"), std::vector<std::string>{"Plane"});
    EXPECT_EQ(rcm->get_entity_two_or_entity_robot_names("C3"), std::vector<std::string>{"rsphere"});
    EXPECT_EQ(rcm->get_entity_one_or_entity_environment_names("C2"), std::vector<std::string>{"r_base_sphere"});
    EXPECT_EQ(rcm->get_entity_two_or_entity_robot_names("C2"), std::vector<std::string>{"rsphere"});

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    EXPECT_THROW(rcm->get_coppeliasim_entity_one_or_entity_environment_names("C3"), std::runtime_error);
    EXPECT_THROW(rcm->get_coppeliasim_entity_two_or_entity_robot_names("C3"), std::runtime_error);
#pragma GCC diagnostic pop
}

TEST_F(RobotConstraintManagerV3Test, DataAndDocumentGetters)
{
    const auto rcm = make_rcm();
    EXPECT_TRUE(std::holds_alternative<VCF::ENVIRONMENT_TO_ROBOT_DATA_V3>(rcm->get_data_v3("C3")));
    EXPECT_TRUE(std::holds_alternative<VCF::ROBOT_TO_ROBOT_DATA_V3>(rcm->get_data_v3("C2")));
    EXPECT_THROW(rcm->get_data("C3"), std::runtime_error);

    const auto document = rcm->get_document();
    ASSERT_TRUE(std::holds_alternative<VCF::DOCUMENT_V3>(document));
    EXPECT_EQ(std::get<VCF::DOCUMENT_V3>(document).vfi_array.size(), 6u);
}

TEST_F(RobotConstraintManagerV3Test, UnknownTagThrows)
{
    const auto rcm = make_rcm();
    EXPECT_THROW(rcm->get_safe_distance("missing"), std::exception);
    EXPECT_THROW(rcm->set_vfi_status("missing", false), std::runtime_error);
    EXPECT_THROW(rcm->update_vfi_workspace_pose("missing", DQ(1)), std::runtime_error);
}

//----------------------Entity-level updates----------------------------------------

TEST_F(RobotConstraintManagerV3Test, EntityPoseUpdateReachesEveryVfiIncludingDisabled)
{
    const auto rcm = make_rcm();
    rcm->disable_vfi("C6");

    const DQ x = 1 + 0.5*E_*(0.3*i_ + 0.1*j_ + 0.5*k_);
    rcm->update_environment_entity_pose("obs_sphere", x);
    EXPECT_EQ(rcm->get_vfi_build_data("C5").environment_poses.at(0), x);
    EXPECT_EQ(rcm->get_vfi_build_data("C6").environment_poses.at(0), x);
    // Other entities are not modified
    EXPECT_EQ(rcm->get_vfi_build_data("C3").environment_poses.at(0), 1 + 0.5*E_*(0.1*k_));
}

TEST_F(RobotConstraintManagerV3Test, EntityPoseUpdateKeepsLoadedDocument)
{
    const auto rcm = make_rcm();
    rcm->update_environment_entity_pose("obs_sphere", 1 + 0.5*E_*(0.3*i_));

    const auto document = std::get<VCF::DOCUMENT_V3>(rcm->get_document());
    for (const auto& entity : document.environment_entities)
    {
        if (entity.name == "obs_sphere")
        {
            EXPECT_EQ(VFIConfigurationFileV3::pose_to_dq(entity.pose), 1 + 0.5*E_*obs_sphere_position_);
        }
    }
}

TEST_F(RobotConstraintManagerV3Test, EntityPoseUpdateRejectsInvalidArguments)
{
    const auto rcm = make_rcm();
    const DQ x = 1 + 0.5*E_*(0.3*i_);
    EXPECT_THROW(rcm->update_environment_entity_pose("rsphere", x), std::runtime_error);  // robot entity
    EXPECT_THROW(rcm->update_environment_entity_pose("missing", x), std::runtime_error);
    EXPECT_THROW(rcm->update_environment_entity_pose("obs_sphere", 2*x), std::runtime_error);  // not unit
}

TEST_F(RobotConstraintManagerV3Test, AttachedDirectionFollowsUpdatedPose)
{
    const auto rcm = make_rcm();
    // Rotate the plane 90 degrees about i_. Its attached direction k_ becomes -j_.
    const DQ r = cos(M_PI_4) + i_*sin(M_PI_4);
    const DQ plane_position = 0.1*k_;
    rcm->update_environment_entity_pose("Plane", r + 0.5*E_*plane_position*r);
    rcm->get_inequality_constraints(q_home_, false, false);

    const Vector4d pv = vec4(rsphere_position(q_home_) - plane_position);
    EXPECT_NEAR(rcm->get_vfi_distance_error("C3"), -pv(2) - 0.05, 1e-12);
}

TEST_F(RobotConstraintManagerV3Test, EntityDerivativeUpdateReachesEveryVfi)
{
    const auto rcm = make_rcm();
    rcm->update_environment_entity_derivative("obs_sphere", 0.2*k_);
    EXPECT_EQ(rcm->get_vfi_build_data("C5").workspace_derivative, 0.2*k_);
    EXPECT_EQ(rcm->get_vfi_build_data("C6").workspace_derivative, 0.2*k_);
    EXPECT_THROW(rcm->update_environment_entity_derivative("missing", 0.2*k_), std::runtime_error);
}

TEST_F(RobotConstraintManagerV3Test, PerTagUpdateWarnsOnceForSharedEntity)
{
    const auto rcm = make_rcm();
    const DQ x = 1 + 0.5*E_*(0.3*i_);

    testing::internal::CaptureStderr();
    for (int i = 0; i < 3; i++)
        rcm->update_vfi_workspace_pose("C5", x);    // obs_sphere is shared with C6
    rcm->update_vfi_workspace_derivative("C5", 0.1*k_);
    rcm->update_vfi_workspace_pose("C3", x);        // Plane is used only by C3
    const std::string output = testing::internal::GetCapturedStderr();

    EXPECT_EQ(count_occurrences(output, "Warning"), 1);
    EXPECT_EQ(count_occurrences(output, "VFI C5"), 1);
    EXPECT_EQ(count_occurrences(output, "VFI C3"), 0);
}

//----------------------Enable and disable----------------------------------------

TEST_F(RobotConstraintManagerV3Test, EnableAndDisableVfi)
{
    const auto rcm = make_rcm();
    auto rows = [&](){ return std::get<0>(rcm->get_inequality_constraints(q_home_, false, false)).rows(); };

    EXPECT_EQ(rows(), 6);
    rcm->disable_vfi("C3");
    rcm->disable_vfi("C5");
    EXPECT_EQ(rows(), 4);
    rcm->enable_vfi("C5");
    EXPECT_EQ(rows(), 5);
    EXPECT_THROW(rcm->enable_vfi("missing"), std::runtime_error);
}

TEST_F(RobotConstraintManagerV3Test, TogglingVfisMatchesFreshManager)
{
    // Regression test: enabling or disabling a VFI between calls must not shift the stack position of the others.
    const auto rcm = make_rcm();
    const auto tags = rcm->get_vfi_tags();
    std::unordered_map<std::string, bool> enabled;
    for (const auto& tag : tags)
        enabled[tag] = true;

    std::srand(3);
    for (int k = 0; k < 50; k++)
    {
        const std::string& tag = tags.at(std::rand() % tags.size());
        enabled.at(tag) = std::rand() % 2;
        rcm->set_vfi_status(tag, enabled.at(tag));

        const VectorXd q = VectorXd::Random(7);
        const auto [A, b] = rcm->get_inequality_constraints(q, false, false);

        const auto reference = make_rcm();
        for (const auto& [t, status] : enabled)
            reference->set_vfi_status(t, status);
        const auto [A_ref, b_ref] = reference->get_inequality_constraints(q, false, false);

        ASSERT_EQ(A.rows(), A_ref.rows());
        if (A.rows() > 0)
        {
            EXPECT_EQ(A, A_ref);
            EXPECT_EQ(b, b_ref);
        }
    }
}

} // namespace
