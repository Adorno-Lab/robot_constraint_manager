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

#include <dqrobotics_extensions/robot_constraint_manager/robot_constraint_manager.hpp>
#include <dqrobotics_extensions/robot_constraint_editor/utils.hpp>
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_v3.hpp>
#include <yaml-cpp/yaml.h>


namespace DQ_robotics_extensions{


class RobotConstraintManager::Impl
{
public:
    //YAML::Node config = YAML::LoadFile(config_path_);
    YAML::Node config_;
    Impl()
    {

    };
};

// * @brief RobotConstraintManager::RobotConstraintManager constructor of the class
// @param coppelia_interface The smartpointer of the DQ_CoppeliaSimInterfaceZMQ object

/**
 * @brief RobotConstraintManager::RobotConstraintManager
 * @param coppelia_interface
 * @param coppeliasim_robot
 * @param robot
 * @param yaml_file_path
 * @param configuration_limits
 * @param configuration_velocity_limits
 * @param level
 */
RobotConstraintManager::RobotConstraintManager(const std::shared_ptr<DQ_CoppeliaSimInterface> &coppelia_interface,
                                               const std::shared_ptr<DQ_CoppeliaSimRobot> &coppeliasim_robot,
                                               const std::shared_ptr<DQ_Kinematics> &robot,
                                               const std::string &yaml_file_path,
                                               const bool &verbosity,
                                               const VFI_Framework::LEVEL &level)
    :cs_{coppelia_interface}, config_path_{yaml_file_path}, level_{level},
    robot_{robot}, coppelia_robot_{coppeliasim_robot},
    rce_compatible_{false},
    verbosity_{verbosity}
{
    impl_ = std::make_shared<RobotConstraintManager::Impl>();

    VFI_M_ = std::make_shared<DQ_robotics_extensions::VFI_manager>(robot->get_dim_configuration_space());
    _initial_settings();
}


/**
 * @brief RobotConstraintManager::RobotConstraintManager
 * @param coppelia_interface
 * @param coppeliasim_robot
 * @param robot
 * @param config_file_reader
 * @param yaml_file_path
 * @param verbosity
 * @param level
 */
RobotConstraintManager::RobotConstraintManager(const std::shared_ptr<DQ_CoppeliaSimInterface> &coppelia_interface,
                                               const std::shared_ptr<DQ_CoppeliaSimRobot> &coppeliasim_robot,
                                               const std::shared_ptr<DQ_Kinematics> &robot,
                                               const std::shared_ptr<VFIConfigurationFile> &config_file_reader,
                                               const std::string &yaml_file_path,
                                               const bool &verbosity,
                                               const VFI_Framework::LEVEL &level)
    :cs_{coppelia_interface},
    config_path_{yaml_file_path},
    level_{level},
    robot_{robot},
    coppelia_robot_{coppeliasim_robot},
    config_file_reader_{config_file_reader},
    rce_compatible_{true},
    configuration_limit_constraint_gain_{1},
    verbosity_{verbosity}
{
    VFI_M_ = std::make_shared<DQ_robotics_extensions::VFI_manager>(robot->get_dim_configuration_space());
    try {
        config_file_reader_->load_data(config_path_);
    } catch (const std::exception& e) {
        throw std::runtime_error(e.what());
    }

    if (config_file_reader_->get_vfi_file_version() == 3)
        throw std::runtime_error("RobotConstraintManager: The configuration file " + config_path_ + " uses the version 3. "
                                 "Use the constructor that does not require CoppeliaSim.");

    if (!config_file_reader_->is_zero_indexed())
        robot_index_convention_ = 1;
    else
        robot_index_convention_ = 0;

    data_list_ = config_file_reader_->get_data();
    number_of_constraints_ = data_list_.size();
    vfi_file_version_ =  config_file_reader_->get_vfi_file_version();
    vfi_zero_indexed_ =  config_file_reader_->is_zero_indexed();
    for (auto& data_item : data_list_)
    {
        std::string tag;
        std::visit([&tag](const auto& d){tag = d.tag;}, data_item);
        data_map_.try_emplace(tag, data_item);
    }
    _create_build_data_v2();

}

/**
 * @brief RobotConstraintManager::RobotConstraintManager constructor of the class. It requires a version 3
 *        configuration file, which contains the poses and offsets of the entities. Therefore, CoppeliaSim is not required.
 * @param robot The kinematic model used to build the constraints.
 * @param config_file_reader The object used to read the configuration file.
 * @param yaml_file_path The path of the version 3 configuration file.
 * @param verbosity
 * @param level
 */
RobotConstraintManager::RobotConstraintManager(const std::shared_ptr<DQ_Kinematics> &robot,
                                               const std::shared_ptr<VFIConfigurationFile> &config_file_reader,
                                               const std::string &yaml_file_path,
                                               const bool &verbosity,
                                               const VFI_Framework::LEVEL &level)
    :config_path_{yaml_file_path},
    level_{level},
    robot_{robot},
    config_file_reader_{config_file_reader},
    rce_compatible_{true},
    configuration_limit_constraint_gain_{1},
    verbosity_{verbosity}
{
    if (!robot_)
        throw std::runtime_error("RobotConstraintManager: The robot cannot be a null pointer.");
    if (!config_file_reader_)
        throw std::runtime_error("RobotConstraintManager: The config_file_reader cannot be a null pointer.");

    VFI_M_ = std::make_shared<DQ_robotics_extensions::VFI_manager>(robot_->get_dim_configuration_space());
    try {
        config_file_reader_->load_data(config_path_);
    } catch (const std::exception& e) {
        throw std::runtime_error(e.what());
    }

    vfi_file_version_ = config_file_reader_->get_vfi_file_version();
    if (vfi_file_version_ != 3)
        throw std::runtime_error("RobotConstraintManager: The configuration file " + config_path_ + " uses the version "
                                 + std::to_string(vfi_file_version_) + ". This constructor requires the version 3. "
                                 "Use the constructor that requires CoppeliaSim instead.");

    document_v3_ = std::get<VFIConfigurationFile::DOCUMENT_V3>(config_file_reader_->get_document());

    // The reader is not required to validate the document. Rule 2 is completed below.
    try {
        VFIConfigurationFileV3::validate(document_v3_);
    } catch (const std::exception& e) {
        throw std::runtime_error("RobotConstraintManager: Invalid configuration file " + config_path_ + ". " + e.what());
    }

    const int dim_configuration = document_v3_.robots.at(0).dim_configuration;
    if (dim_configuration != robot_->get_dim_configuration_space())
        throw std::runtime_error("RobotConstraintManager: The configuration file " + config_path_ + " defines a robot with "
                                 + std::to_string(dim_configuration) + " DoF, but the kinematic model has "
                                 + std::to_string(robot_->get_dim_configuration_space()) + " DoF.");

    vfi_zero_indexed_ = document_v3_.zero_indexed;
    robot_index_convention_ = vfi_zero_indexed_ ? 0 : 1;
    number_of_constraints_ = document_v3_.vfi_array.size();
    for (const auto& data_item : document_v3_.vfi_array)
    {
        std::string tag;
        std::visit([&tag](const auto& d){tag = d.tag;}, data_item);
        data_v3_map_.try_emplace(tag, data_item);
    }
    _create_build_data_v3();
}

/**
 * @brief RobotConstraintManager::_add_build_data stores the build data of a VFI and enables it.
 * @param vfi_data The build data of the VFI.
 */
void RobotConstraintManager::_add_build_data(const VFI_manager::VFI_BUILD_DATA &vfi_data)
{
    vfi_build_data_map_.try_emplace(vfi_data.tag, vfi_data);
    vfi_enable_status_map_.try_emplace(vfi_data.tag, true);
    if (verbosity_)
        show_vfi_build_data(vfi_data.tag);
}

/**
 * @brief RobotConstraintManager::_create_build_data_v2 creates the build data of the VFIs defined in a
 *        version 2 configuration file. The primitive offsets and the workspace poses are obtained from CoppeliaSim.
 */
void RobotConstraintManager::_create_build_data_v2()
{
    if (!rce_compatible_)
        throw std::runtime_error("Invalid call. This private method requires the version 2 of the configuration File Specification");

    //const int n = data_map_.size();
    //std::vector<VFI_manager::VFI_BUILD_DATA> build_data;
    //build_data.reserve(n);
    for (auto& data_item : data_list_)
    {
        std::visit([this](auto&& arg){
            using T = std::decay_t<decltype(arg)>;
            if constexpr (std::is_same_v<T, VFIConfigurationFile::ENVIRONMENT_TO_ROBOT_DATA>) {
                VFI_manager::VFI_BUILD_DATA vfi_data;
                vfi_data.vfi_type  = VFI_manager::VFI_TYPE::ENVIRONMENT_TO_ROBOT;
                vfi_data.vfi_class = VFI_Framework::map_strings_to_vfiClass(arg.entity_robot_primitive_type,
                                                                            arg.entity_environment_primitive_type);
                vfi_data.direction = VFI_Framework::map_string_to_vfiDirection(arg.direction);
                vfi_data.safe_distance = arg.safe_distance;
                vfi_data.buffer = arg.buffer;
                vfi_data.vfi_gain = arg.vfi_gain;
                vfi_data.robot_index_one = arg.robot_index-robot_index_convention_;
                vfi_data.robot_index_two = -1;
                vfi_data.joint_index_one = arg.joint_index-robot_index_convention_;
                vfi_data.joint_index_two = -1;
                vfi_data.primitive_offsets_one =  _get_coppeliasim_offsets(arg.cs_entity_robot,  vfi_data.robot_index_one, vfi_data.joint_index_one);
                vfi_data.primitive_offsets_two = {DQ(-1)};
                vfi_data.robot_attached_direction = k_;
                vfi_data.environment_attached_direction = k_;
                vfi_data.workspace_derivative = DQ(0);
                vfi_data.environment_poses = _get_workspace_poses(arg.cs_entity_environment);
                vfi_data.tag = arg.tag;
                _add_build_data(vfi_data);

            }else if constexpr (std::is_same_v<T, VFIConfigurationFile::ROBOT_TO_ROBOT_DATA>){
                VFI_manager::VFI_BUILD_DATA vfi_data;
                vfi_data.vfi_type  = VFI_manager::VFI_TYPE::ROBOT_TO_ROBOT;
                vfi_data.vfi_class = VFI_Framework::map_strings_to_vfiClass(arg.entity_one_primitive_type,
                                                                            arg.entity_two_primitive_type);
                vfi_data.direction = VFI_Framework::DIRECTION::RESTRICTED_ZONE;
                vfi_data.safe_distance = arg.safe_distance;
                vfi_data.buffer = arg.buffer;
                vfi_data.vfi_gain = arg.vfi_gain;
                vfi_data.robot_index_one = arg.robot_index_one-robot_index_convention_;
                vfi_data.robot_index_two = arg.robot_index_two-robot_index_convention_;
                vfi_data.joint_index_one = arg.joint_index_one-robot_index_convention_;
                vfi_data.joint_index_two = arg.joint_index_two-robot_index_convention_;
                vfi_data.primitive_offsets_one = _get_coppeliasim_offsets(arg.cs_entity_one, vfi_data.robot_index_one, vfi_data.joint_index_one);
                vfi_data.primitive_offsets_two = _get_coppeliasim_offsets(arg.cs_entity_two, vfi_data.robot_index_two, vfi_data.joint_index_two);
                vfi_data.robot_attached_direction = DQ(-1);
                vfi_data.environment_attached_direction = DQ(-1);

                vfi_data.workspace_derivative = DQ(0);
                vfi_data.environment_poses = {DQ(-1)};
                vfi_data.tag = arg.tag;
                _add_build_data(vfi_data);
            }else {
                throw std::runtime_error("Unsupported VFI TYPE!");
            }
        }, data_item);

    }


}


/**
 * @brief RobotConstraintManager::_create_build_data_v3 creates the build data of the VFIs defined in a
 *        version 3 configuration file. The primitive offsets, the workspace poses, and the attached directions
 *        are obtained from the entities defined in the file.
 */
void RobotConstraintManager::_create_build_data_v3()
{
    std::unordered_map<std::string, const VFIConfigurationFile::ENVIRONMENT_ENTITY*> environment_entities;
    for (const auto& entity : document_v3_.environment_entities)
    {
        environment_entities.try_emplace(entity.name, &entity);
        environment_entity_usage_.try_emplace(entity.name);
    }

    std::unordered_map<std::string, const VFIConfigurationFile::ROBOT_ENTITY*> robot_entities;
    for (const auto& entity : document_v3_.robot_entities)
        robot_entities.try_emplace(entity.name, &entity);

    // The document is validated in the constructor. Therefore, every entity name exists in its table, and the
    // entities of a LINESEGMENT have the same robot_index and joint_index.
    auto get_offsets = [&robot_entities](const std::vector<std::string>& names)
    {
        std::vector<DQ> offsets;
        offsets.reserve(names.size());
        for (const auto& name : names)
            offsets.emplace_back(VFIConfigurationFileV3::pose_to_dq(robot_entities.at(name)->offset));
        return offsets;
    };
    auto get_poses = [&environment_entities](const std::vector<std::string>& names)
    {
        std::vector<DQ> poses;
        poses.reserve(names.size());
        for (const auto& name : names)
            poses.emplace_back(VFIConfigurationFileV3::pose_to_dq(environment_entities.at(name)->pose));
        return poses;
    };

    for (const auto& data_item : document_v3_.vfi_array)
    {
        std::visit([&](const auto& arg){
            using T = std::decay_t<decltype(arg)>;
            if constexpr (std::is_same_v<T, VFIConfigurationFile::ENVIRONMENT_TO_ROBOT_DATA_V3>) {
                const auto& robot_entity = *robot_entities.at(arg.entity_robot.at(0));
                const auto& environment_entity = *environment_entities.at(arg.entity_environment.at(0));

                VFI_manager::VFI_BUILD_DATA vfi_data;
                vfi_data.vfi_type  = VFI_manager::VFI_TYPE::ENVIRONMENT_TO_ROBOT;
                vfi_data.vfi_class = VFI_Framework::map_strings_to_vfiClass(arg.entity_robot_primitive_type,
                                                                            arg.entity_environment_primitive_type);
                vfi_data.direction = VFI_Framework::map_string_to_vfiDirection(arg.direction);
                vfi_data.safe_distance = arg.safe_distance;
                vfi_data.buffer = arg.buffer;
                vfi_data.vfi_gain = arg.vfi_gain;
                vfi_data.robot_index_one = robot_entity.robot_index-robot_index_convention_;
                vfi_data.robot_index_two = -1;
                vfi_data.joint_index_one = robot_entity.joint_index-robot_index_convention_;
                vfi_data.joint_index_two = -1;
                vfi_data.primitive_offsets_one = get_offsets(arg.entity_robot);
                vfi_data.primitive_offsets_two = {DQ(-1)};
                vfi_data.robot_attached_direction =
                    VFI_Framework::map_attached_direction_string_to_dq(robot_entity.attached_direction);
                vfi_data.environment_attached_direction =
                    VFI_Framework::map_attached_direction_string_to_dq(environment_entity.attached_direction);
                vfi_data.workspace_derivative = DQ(0);
                vfi_data.environment_poses = get_poses(arg.entity_environment);
                for (std::size_t i = 0; i < arg.entity_environment.size(); i++)
                    environment_entity_usage_.at(arg.entity_environment.at(i)).emplace_back(arg.tag, i);
                vfi_data.tag = arg.tag;
                _add_build_data(vfi_data);

            }else if constexpr (std::is_same_v<T, VFIConfigurationFile::ROBOT_TO_ROBOT_DATA_V3>){
                const auto& robot_entity_one = *robot_entities.at(arg.entity_one.at(0));
                const auto& robot_entity_two = *robot_entities.at(arg.entity_two.at(0));

                VFI_manager::VFI_BUILD_DATA vfi_data;
                vfi_data.vfi_type  = VFI_manager::VFI_TYPE::ROBOT_TO_ROBOT;
                vfi_data.vfi_class = VFI_Framework::map_strings_to_vfiClass(arg.entity_one_primitive_type,
                                                                            arg.entity_two_primitive_type);
                // As in version 2, the direction and the attached directions are not used yet.
                vfi_data.direction = VFI_Framework::DIRECTION::RESTRICTED_ZONE;
                vfi_data.safe_distance = arg.safe_distance;
                vfi_data.buffer = arg.buffer;
                vfi_data.vfi_gain = arg.vfi_gain;
                vfi_data.robot_index_one = robot_entity_one.robot_index-robot_index_convention_;
                vfi_data.robot_index_two = robot_entity_two.robot_index-robot_index_convention_;
                vfi_data.joint_index_one = robot_entity_one.joint_index-robot_index_convention_;
                vfi_data.joint_index_two = robot_entity_two.joint_index-robot_index_convention_;
                vfi_data.primitive_offsets_one = get_offsets(arg.entity_one);
                vfi_data.primitive_offsets_two = get_offsets(arg.entity_two);
                vfi_data.robot_attached_direction = DQ(-1);
                vfi_data.environment_attached_direction = DQ(-1);
                vfi_data.workspace_derivative = DQ(0);
                vfi_data.environment_poses = {DQ(-1)};
                vfi_data.tag = arg.tag;
                _add_build_data(vfi_data);
            }else {
                throw std::runtime_error("Unsupported VFI TYPE!");
            }
        }, data_item);
    }
}


/**
 * @brief RobotConstraintManager::get_number_of_vfi_constraints gets the number of VFI constraints set in the config file.
 *                  This number only counts the constraints that require tags. Therefore, the configuration limits, and the
 *                  configuration velocity limits are not taken into account.
 * @return The VFI constraints set in the config file.
 */
int RobotConstraintManager::get_number_of_vfi_constraints() const
{
    return number_of_constraints_;
}

/**
 * @brief RobotConstraintManager::add_inequality_constraint
 * @param A
 * @param b
 */
void RobotConstraintManager::add_inequality_constraint(const MatrixXd &A, const VectorXd &b)
{
    VFI_M_->add_inequality_constraint(A, b);
}


/**
 * @brief RobotConstraintManager::_check_unit throws an exception if the input string is not
 */
void RobotConstraintManager::_check_unit(const std::string &unit)
{
    if (unit != std::string("DEG") && unit !=std::string("RAD"))
        throw std::runtime_error("RobotConstraintManager: Bad argument. You used "+unit+" in the config file. "
                                 "Use DEG for degrees or RAD for radians.");
}


/**
 * @brief RobotConstraintManager::get_inequality_constraints creates and returns the VFIs inequalities. This set of constraints
 *                     include configuration and configuration velocity limits using the
 *                     inequality constraints (10) and (7), defined in
 *                     Adaptive Constrained Kinematic Control using Partial or Complete Task-Space Measurements.
 *                     Marinho, M. M. & Adorno, B. V.
 *                     IEEE Transactions on Robotics (T-RO),
 *                     38(6):3498–3513, December, 2022. Presented at ICRA'23.
 *
 * @param q The robot configuration.
 * @param include_configuration_constraints
 * @param include_configuration_velocity_constraints
 * @return A tuple containing the  desired VFIs constraints. For instance, given the constraints A*x <= b, this method returns {A,b}.
 */
std::tuple<MatrixXd, VectorXd> RobotConstraintManager::get_inequality_constraints(const VectorXd &q,
                                                                                  const bool &include_configuration_constraints,
                                                                                  const bool &include_configuration_velocity_constraints)
{
    const int n = vfi_build_data_map_.size();
    //const int robot_dim = robot_->get_dim_configuration_space();
    std::vector<VFI_manager::VFI_BUILD_DATA> vfi_build_data_list;
    vfi_build_data_list.reserve(n);

    for (auto& pair : vfi_build_data_map_)
    {
        auto data = pair.second;
        if (vfi_enable_status_map_.at(data.tag))
            vfi_build_data_list.push_back(pair.second);
    }



    if (include_configuration_constraints)
        VFI_M_->add_configuration_limits(configuration_limit_constraint_gain_, q);
    if (include_configuration_velocity_constraints)
        VFI_M_->add_configuration_velocity_limits();

    for (size_t i = 0; i<vfi_build_data_list.size(); i++)
        VFI_M_->add_vfi_constraint(vfi_build_data_list.at(i),i,robot_,q,robot_,q);
    /*
    for (int i = 0; i<n; i++)
    {
        VFI_M_->add_vfi_constraint(vfi_build_data_list.at(i),i,robot_,q,robot_,q);


        if (vfi_build_data_list.at(i).vfi_type == VFI_manager::VFI_TYPE::ENVIRONMENT_TO_ROBOT)
        {
            const int index = vfi_build_data_list.at(i).joint_index_one;
            const DQ offset = vfi_build_data_list.at(i).primitive_offsets_one.at(0);
            DQ x = (robot_->fkm(q, index))*offset;
            MatrixXd J = haminus8(offset)*robot_->pose_jacobian(q, index);
            if (J.cols() != robot_dim)
                J = DQ_robotics_extensions::Numpy::resize(J, J.rows(), robot_dim);


            VFI_M_->add_vfi_constraint(vfi_build_data_list.at(i).tag,
                                       i,
                                       vfi_build_data_list.at(i).direction,
                                       vfi_build_data_list.at(i).vfi_class,
                                       vfi_build_data_list.at(i).safe_distance,
                                       vfi_build_data_list.at(i).vfi_gain,
                                       J,
                                       x,
                                       vfi_build_data_list.at(i).robot_attached_direction,
                                       vfi_build_data_list.at(i).environment_poses.at(0), // x_workspace
                                       vfi_build_data_list.at(i).environment_attached_direction,
                                       vfi_build_data_list.at(i).workspace_derivative);
        }
        else{ //vfi_mode_list_.at(i) == VFI_manager::VFI_MODE::ROBOT_TO_ROBOT
            const int index_1 = vfi_build_data_list.at(i).joint_index_one;
            const DQ offset_1 = vfi_build_data_list.at(i).primitive_offsets_one.at(0);

            DQ x1 =  (robot_->fkm(q, index_1))*offset_1;
            MatrixXd J1 = haminus8(offset_1)*robot_->pose_jacobian(q, index_1);

            const int index_2 = vfi_build_data_list.at(i).joint_index_two;
            const DQ offset_2 = vfi_build_data_list.at(i).primitive_offsets_two.at(0);

            DQ x2 =  (robot_->fkm(q, index_2))*offset_2;
            MatrixXd J2 = haminus8(offset_2)*robot_->pose_jacobian(q, index_2);

            VFI_M_->add_vfi_constraint(vfi_build_data_list.at(i),i,robot_,q,robot_,q);


            VFI_M_->add_vfi_rpoint_to_rpoint(vfi_build_data_list.at(i).tag,
                                             i,
                                             vfi_build_data_list.at(i).safe_distance,
                                             vfi_build_data_list.at(i).vfi_gain,
                                             {J1, x1},
                                             {J2, x2});

        }
    }*/
    return VFI_M_->get_inequality_constraints();
}


/**
 * @brief RobotConstraintManager::get_vfi_log_data returns a tuple containing the vfi log data. This is useful for debugging.
 * @param tag The tag of the constraint.
 * @return A tuple containing the vfi log data
 *      {distance, square_distance, distance_error, square_distance_error, line_to_line_angle_rad, vfi_type}
 */
std::tuple<double, double, double, double, double, std::string> RobotConstraintManager::get_vfi_log_data(const std::string &tag) const
{
    return VFI_M_->get_vfi_log_data(tag);
}

/**
 * @brief RobotConstraintManager::get_primitive_index_and_offset returns the index and offset of the primitive related to the constraint defined
 *              by the specific tag.
 * @param The tag of the constraint.
 * @return {joint_index_one, primitive_offset_one, joint_index_two, primitive_offset_two}
 */
std::tuple<int, DQ, int, DQ> RobotConstraintManager::get_primitive_index_and_offset(const std::string &tag) const
{
    const auto data = vfi_build_data_map_.at(tag);
    return {data.joint_index_one, data.primitive_offsets_one.at(0), data.joint_index_two, data.primitive_offsets_two.at(0)};
}

/**
 * @brief RobotConstraintManager::get_vfi_build_data returns a custom struct containing the data used to build
 *              the VFIs
 * @param tag The tag of the constraint.
 * @return A VFI_BUILD_DATA struct.
 */
VFI_manager::VFI_BUILD_DATA RobotConstraintManager::get_vfi_build_data(const std::string &tag) const
{
    return vfi_build_data_map_.at(tag);
}

/**
 * @brief RobotConstraintManager::get_raw_data returns a custom struct containing the raw data from the YAML file.
 * @param tag The tag of the constraint.
 * @return A YAML_RAW_DATA struct.
 */
RobotConstraintManager::YAML_RAW_DATA RobotConstraintManager::get_raw_yaml_data(const std::string &tag) const
{
    if (rce_compatible_)
        throw std::runtime_error("Invalid call. This method is not available for versions 2 and 3.");

    return yaml_raw_data_map_.at(tag);
}

/**
 * @brief RobotConstraintManager::get_data returns the data from a version 2 configuration file.
 *        For version 3 files, use get_data_v3().
 * @param tag The tag of the constraint.
 * @return the data of the constraint.
 */
VFIConfigurationFile::Data RobotConstraintManager::get_data(const std::string& tag) const
{
    if (vfi_file_version_ == 3)
        throw std::runtime_error("Invalid call. get_data() supports only version 2 files. Use get_data_v3() instead.");

    try {
        return data_map_.at(tag);
    }catch (const std::exception& e){
        std::cerr<<"Tag "+tag+" not found!"<<std::endl;
        throw std::runtime_error(e.what());
    }
}

/**
 * @brief RobotConstraintManager::get_data_v3 returns the data from a version 3 configuration file.
 *        For version 2 files, use get_data().
 * @param tag The tag of the constraint.
 * @return the data of the constraint.
 */
VFIConfigurationFile::DataV3 RobotConstraintManager::get_data_v3(const std::string& tag) const
{
    if (vfi_file_version_ != 3)
        throw std::runtime_error("Invalid call. get_data_v3() supports only version 3 files. Use get_data() instead.");

    try {
        return data_v3_map_.at(tag);
    }catch (const std::exception& e){
        std::cerr<<"Tag "+tag+" not found!"<<std::endl;
        throw std::runtime_error(e.what());
    }
}

/**
 * @brief RobotConstraintManager::get_document returns the complete content of the configuration file.
 * @return A DOCUMENT_V2 or a DOCUMENT_V3, depending on the file version.
 */
VFIConfigurationFile::Document RobotConstraintManager::get_document() const
{
    if (!rce_compatible_)
        throw std::runtime_error("Invalid call. This method requires the version 2 or 3 of the configuration File Specification");

    if (vfi_file_version_ == 3)
        return document_v3_;
    return VFIConfigurationFile::DOCUMENT_V2{vfi_zero_indexed_, data_list_};
}

/**
 * @brief RobotConstraintManager::_get_base_data returns the data shared by every VFI type and file version.
 * @param tag The tag of the constraint.
 * @return The BASE_DATA of the constraint.
 */
VFIConfigurationFile::BASE_DATA RobotConstraintManager::_get_base_data(const std::string &tag) const
{
    auto to_base_data = [](const auto& d) -> VFIConfigurationFile::BASE_DATA { return d; };
    if (vfi_file_version_ == 3)
        return std::visit(to_base_data, get_data_v3(tag));
    return std::visit(to_base_data, get_data(tag));
}

/**
 * @brief RobotConstraintManager::get_buffer
 * @param tag The tag of the constraint.
 * @return the buffer
 */
double RobotConstraintManager::get_buffer(const std::string &tag) const
{
    try {
        return _get_base_data(tag).buffer;
    }catch (const std::exception& e) {
        throw std::runtime_error(e.what());
    }
}

/**
 * @brief RobotConstraintManager::get_safe_distance
 * @param tag The tag of the constraint.
 * @return The safe distance
 */
double RobotConstraintManager::get_safe_distance(const std::string &tag) const
{
    try {
        return _get_base_data(tag).safe_distance;
    }catch (const std::exception& e) {
        throw std::runtime_error(e.what());
    }
}

/**
 * @brief RobotConstraintManager::get_vfi_gain
 * @param tag The tag of the constraint.
 * @return The VFI gain
 */
double RobotConstraintManager::get_vfi_gain(const std::string &tag) const
{
    try {
        return _get_base_data(tag).vfi_gain;
    }catch (const std::exception& e) {
        throw std::runtime_error(e.what());
    }
}

/**
 * @brief RobotConstraintManager::get_vfi_direction
 * @param tag The tag of the constraint.
 * @return The vfi direction
 */
std::string RobotConstraintManager::get_vfi_direction(const std::string &tag) const
{
    try {
        return _get_base_data(tag).direction;
    }catch (const std::exception& e) {
        throw std::runtime_error(e.what());
    }
}

/**
 * @brief RobotConstraintManager::get_vfi_type
 * @param tag The tag of the constraint.
 * @return The vfi type
 */
std::string RobotConstraintManager::get_vfi_type(const std::string &tag) const
{
    try {
        return _get_base_data(tag).vfi_type;
    }catch (const std::exception& e) {
        throw std::runtime_error(e.what());
    }
}

/**
 * @brief RobotConstraintManager::get_coppeliasim_entity_one_or_entity_environment_names
 *        This method is deprecated. Use get_entity_one_or_entity_environment_names() instead.
 * @param tag The tag of the constraint.
 * @return A vector of strings containing the names of the entity one or entity environment names(according to the VFI type)
 */
std::vector<std::string> RobotConstraintManager::get_coppeliasim_entity_one_or_entity_environment_names(const std::string &tag) const
{
    if (vfi_file_version_ == 3)
        throw std::runtime_error("Invalid call. Version 3 files do not use CoppeliaSim names. "
                                 "Use get_entity_one_or_entity_environment_names() instead.");
    return get_entity_one_or_entity_environment_names(tag);
}

/**
 * @brief RobotConstraintManager::get_coppeliasim_entity_two_or_entity_robot_names
 *        This method is deprecated. Use get_entity_two_or_entity_robot_names() instead.
 * @param tag The tag of the constraint.
 * @return A vector of strings containing the names of the entity two or entity robot names(according to the VFI type)
 */
std::vector<std::string> RobotConstraintManager::get_coppeliasim_entity_two_or_entity_robot_names(const std::string &tag) const
{
    if (vfi_file_version_ == 3)
        throw std::runtime_error("Invalid call. Version 3 files do not use CoppeliaSim names. "
                                 "Use get_entity_two_or_entity_robot_names() instead.");
    return get_entity_two_or_entity_robot_names(tag);
}

/**
 * @brief RobotConstraintManager::get_entity_one_or_entity_environment_names
 * @param tag The tag of the constraint.
 * @return A vector of strings containing the names of the entity one or entity environment names (according to the VFI type).
 *         These are CoppeliaSim object names in version 2 files, and entity names in version 3 files.
 */
std::vector<std::string> RobotConstraintManager::get_entity_one_or_entity_environment_names(const std::string &tag) const
{
    auto get_names = [](const auto& d) -> std::vector<std::string> {
        using T = std::decay_t<decltype(d)>;
        if constexpr (std::is_same_v<T, VFIConfigurationFile::ENVIRONMENT_TO_ROBOT_DATA>) {
            return d.cs_entity_environment;
        } else if constexpr (std::is_same_v<T, VFIConfigurationFile::ROBOT_TO_ROBOT_DATA>) {
            return d.cs_entity_one;
        } else if constexpr (std::is_same_v<T, VFIConfigurationFile::ENVIRONMENT_TO_ROBOT_DATA_V3>) {
            return d.entity_environment;
        } else {
            return d.entity_one;
        }
    };
    try {
        if (vfi_file_version_ == 3)
            return std::visit(get_names, get_data_v3(tag));
        return std::visit(get_names, get_data(tag));
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("Failed to get entities for tag '") + tag + "': " + e.what());
    }
}

/**
 * @brief RobotConstraintManager::get_entity_two_or_entity_robot_names
 * @param tag The tag of the constraint.
 * @return A vector of strings containing the names of the entity two or entity robot names (according to the VFI type).
 *         These are CoppeliaSim object names in version 2 files, and entity names in version 3 files.
 */
std::vector<std::string> RobotConstraintManager::get_entity_two_or_entity_robot_names(const std::string &tag) const
{
    auto get_names = [](const auto& d) -> std::vector<std::string> {
        using T = std::decay_t<decltype(d)>;
        if constexpr (std::is_same_v<T, VFIConfigurationFile::ENVIRONMENT_TO_ROBOT_DATA>) {
            return d.cs_entity_robot;
        } else if constexpr (std::is_same_v<T, VFIConfigurationFile::ROBOT_TO_ROBOT_DATA>) {
            return d.cs_entity_two;
        } else if constexpr (std::is_same_v<T, VFIConfigurationFile::ENVIRONMENT_TO_ROBOT_DATA_V3>) {
            return d.entity_robot;
        } else {
            return d.entity_two;
        }
    };
    try {
        if (vfi_file_version_ == 3)
            return std::visit(get_names, get_data_v3(tag));
        return std::visit(get_names, get_data(tag));
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("Failed to get entities for tag '") + tag + "': " + e.what());
    }
}


/**
 * @brief RobotConstraintManager::get_vfi_tags returns all tags used in the configuration file
 * @return A vector containing all tags
 */
std::vector<std::string> RobotConstraintManager::get_vfi_tags() const
{
    std::vector<std::string> tags;
    tags.reserve(vfi_build_data_map_.size());
    for (auto& pair : vfi_build_data_map_)
        tags.push_back(pair.first);
    return tags;
}


/**
 * @brief RobotConstraintManager::get_vfi_distance_error gets the distance error computed in the tag-specified VFI. Some VFIs are implemented
 *                      using the square distance error, which is computed as
 *                              square_distance_error = square_d - square_safe_distance.
 *                      In such cases however, this method is going to return
 *                      distance_error = sqrt(square_d) - sqrt(square_safe_distance).
 *
 * @param tag The tag of the constraint.
 * @return The desired distance error.
 */
double RobotConstraintManager::get_vfi_distance_error(const std::string &tag) const
{
    return VFI_M_->get_vfi_distance_error(tag);
}


/**
 * @brief RobotConstraintManager::::get_line_to_line_angle gets the angle between the two Plücker line orientations when the VFI used is RLINE_TO_LINE_ANGLE.
 *              For other VFI types, an exception is thrown.
 *              Note that the safe angle is not taken into account. If you want to include the safe angle, consider using
 *              ferror = get_vfi_distance_error(tag), which will return
 *                          ferror = f-fsafe,
 *              where f = 2-2*cos(phi) and fsafe = 2-2*cos(safe_angle).
 *
 * @param tag The tag of the constraint.
 * @return the two Plücker line orientations.
 */
double RobotConstraintManager::get_line_to_line_angle(const std::string &tag) const
{
    return VFI_M_->get_line_to_line_angle(tag);
}


/**
 * @brief RobotConstraintManager::show_vfi_build_data shows the data extracted from the config yaml file.
 * @param tag The tag of the constraint.
 */
void RobotConstraintManager::show_vfi_build_data(const std::string &tag) const
{
    try {
        auto data = vfi_build_data_map_.at(tag);
        std::cout<<"---------------------------------------------"<<std::endl;
        std::cout<<"TAG:                             "<<tag<<std::endl;
        std::cout<<"VFI type:                        "<<VFI_Framework::map_vfiType_to_string(data.vfi_type)<<std::endl;
        std::cout<<"VFI class:                       "<<VFI_Framework::map_vfiClass_to_string(data.vfi_class)<<std::endl;
        std::cout<<"Direction:                       "<<VFI_Framework::map_vfiDirection_to_string(data.direction)<<std::endl;
        std::cout<<"Safe distance:                   "<<data.safe_distance<<std::endl;
        std::cout<<"buffer:                          "<<data.buffer<<std::endl;
        std::cout<<"VFI gain:                        "<<data.vfi_gain<<std::endl;
        std::cout<<"Joint index one:                 "<<data.joint_index_one<<std::endl;
        std::cout<<"Joint index two:                 "<<data.joint_index_two<<std::endl;
        std::cout<<"primitive_offset_one:            "<<data.primitive_offsets_one.at(0)<<std::endl;
        std::cout<<"primitive_offset_two:            "<<data.primitive_offsets_two.at(0)<<std::endl;
        std::cout<<"robot_attached_direction:        "<<data.robot_attached_direction<<std::endl;
        std::cout<<"environment_attached_direction:  "<<data.environment_attached_direction<<std::endl;
        std::cout<<"workspace derivative:            "<<data.workspace_derivative<<std::endl;
        std::cout<<"cs_entity_environment_pose:      "<<data.environment_poses.at(0)<<std::endl;
        std::cout<<"---------------------------------------------"<<std::endl;
    } catch (const std::exception& e) {
        std::cerr<<e.what()<<std::endl;
        throw std::runtime_error("RobotConstraintManager::show_vfi_build_data: VFI TAG not found!");
}
}


/**
 * @brief RobotConstraintManager::update_vfi_workspace_derivative
 * @param tag
 * @param workspace_derivative
 */
void RobotConstraintManager::update_vfi_workspace_derivative(const std::string &tag, const DQ &workspace_derivative)
{
    _warn_if_shared_environment_entity(tag, "update_vfi_workspace_derivative");
    try{
        VFI_manager::VFI_BUILD_DATA data = vfi_build_data_map_.at(tag);
        data.workspace_derivative = workspace_derivative;
        vfi_build_data_map_.insert_or_assign(tag,data);
    } catch (const std::exception& e) {
        std::cerr<<e.what()<<std::endl;
        throw std::runtime_error("RobotConstraintManager::update_vfi_workspace_derivative: Fail to update the VFI data!");
    }
}

/**
 * @brief RobotConstraintManager::update_vfi_workspace_pose
 * @param tag
 * @param workspace_pose
 */
void RobotConstraintManager::update_vfi_workspace_pose(const std::string &tag, const DQ &workspace_pose)
{
    _warn_if_shared_environment_entity(tag, "update_vfi_workspace_pose");
    try{
        VFI_manager::VFI_BUILD_DATA data = vfi_build_data_map_.at(tag);
        data.environment_poses.at(0) = workspace_pose;
        vfi_build_data_map_.insert_or_assign(tag,data);
    } catch (const std::exception& e) {
        std::cerr<<e.what()<<std::endl;
        throw std::runtime_error("RobotConstraintManager::update_vfi_workspace: Fail to update the VFI data!");
    }
}

/**
 * @brief RobotConstraintManager::update_environment_entity_pose updates the pose of an environment entity
 *        in every VFI that uses it, including the disabled ones. The attached direction of the entity is constant
 *        and expressed in the entity frame. Therefore, it is applied to the updated pose.
 *        The loaded configuration file and the document returned by get_document() are not modified.
 *        This method requires a version 3 configuration file.
 * @param name The name of the environment entity, as defined in the configuration file.
 * @param pose The new pose of the entity, expressed in the same frame as DQ_Kinematics::fkm().
 */
void RobotConstraintManager::update_environment_entity_pose(const std::string &name, const DQ &pose)
{
    if (vfi_file_version_ != 3)
        throw std::runtime_error("RobotConstraintManager::update_environment_entity_pose: This method requires a "
                                 "version 3 configuration file. Use update_vfi_workspace_pose() instead.");

    const auto usage = environment_entity_usage_.find(name);
    if (usage == environment_entity_usage_.end())
        throw std::runtime_error("RobotConstraintManager::update_environment_entity_pose: '" + name +
                                 "' is not an environment entity.");

    if (!is_unit(pose))
        throw std::runtime_error("RobotConstraintManager::update_environment_entity_pose: The pose of '" + name +
                                 "' must be a unit dual quaternion.");

    for (const auto& [tag, index] : usage->second)
        vfi_build_data_map_.at(tag).environment_poses.at(index) = pose;
}

/**
 * @brief RobotConstraintManager::_warn_if_shared_environment_entity shows a warning, once per tag, if the
 *        first environment entity of the VFI is used by other VFIs. In that case, a per-tag update does
 *        not update the other VFIs. This check applies only to version 3 configuration files.
 * @param tag The tag of the constraint.
 * @param method_name The name of the per-tag method, used in the warning.
 */
void RobotConstraintManager::_warn_if_shared_environment_entity(const std::string &tag, const std::string &method_name)
{
    if (vfi_file_version_ != 3 || shared_entity_warned_tags_.count(tag))
        return;

    const auto data = data_v3_map_.find(tag);
    if (data == data_v3_map_.end())
        return;
    const auto* env_data = std::get_if<VFIConfigurationFile::ENVIRONMENT_TO_ROBOT_DATA_V3>(&data->second);
    if (!env_data)
        return;

    const std::string& name = env_data->entity_environment.at(0);
    std::vector<std::string> other_tags;
    for (const auto& [other_tag, index] : environment_entity_usage_.at(name))
        if (other_tag != tag)
            other_tags.push_back(other_tag);
    if (other_tags.empty())
        return;

    shared_entity_warned_tags_.insert(tag);
    std::cerr<<"Warning: RobotConstraintManager::"<<method_name<<": The environment entity '"<<name
             <<"' of the VFI "<<tag<<" is also used by other VFIs ("<<join_vector(other_tags)<<"), which are not updated. "
             <<"Use the update_environment_entity_* methods to update all of them. "
             <<"This warning is shown once per tag."<<std::endl;
}

/**
 * @brief RobotConstraintManager::update_environment_entity_derivative updates the time derivative of the pose of an
 *        environment entity in every VFI that uses it, including the disabled ones.
 *        The loaded configuration file and the document returned by get_document() are not modified.
 *        This method requires a version 3 configuration file.
 * @param name The name of the environment entity, as defined in the configuration file.
 * @param derivative The new derivative of the entity.
 */
void RobotConstraintManager::update_environment_entity_derivative(const std::string &name, const DQ &derivative)
{
    if (vfi_file_version_ != 3)
        throw std::runtime_error("RobotConstraintManager::update_environment_entity_derivative: This method requires a "
                                 "version 3 configuration file. Use update_vfi_workspace_derivative() instead.");

    const auto usage = environment_entity_usage_.find(name);
    if (usage == environment_entity_usage_.end())
        throw std::runtime_error("RobotConstraintManager::update_environment_entity_derivative: '" + name +
                                 "' is not an environment entity.");

    // Each VFI stores a single workspace derivative, which corresponds to its first environment entity.
    for (const auto& [tag, index] : usage->second)
        if (index != 0)
            throw std::runtime_error("RobotConstraintManager::update_environment_entity_derivative: '" + name +
                                     "' is not the first environment entity of the VFI " + tag +
                                     ". Its derivative is not supported.");

    for (const auto& [tag, index] : usage->second)
        vfi_build_data_map_.at(tag).workspace_derivative = derivative;
}

/**
 * @brief RobotConstraintManager::update_vfi_buffer updates the buffer parameter for the desired VFI
 *              constraint.
 * @param tag
 * @param buffer
 */
void RobotConstraintManager::update_vfi_buffer(const std::string& tag, const double& buffer)
{
    try{
        VFI_manager::VFI_BUILD_DATA data = vfi_build_data_map_.at(tag);
        data.buffer = buffer;
        vfi_build_data_map_.insert_or_assign(tag, data);
    } catch (const std::exception& e) {
        std::cerr<<e.what()<<std::endl;
        throw std::runtime_error("RobotConstraintManager::update_vfi_buffer: Fail to update the VFI data!");
    }
}

/**
 * @brief RobotConstraintManager::set_vfi_status Enable or disable a VFI constraint by its tag
 *
 * @param tag   VFI constraint identifier (must exist in the system)
 * @param status true to enable, false to disable
 * @throws std::runtime_error if tag doesn't exist
 */
void RobotConstraintManager::set_vfi_status(const std::string& tag, const bool &status)
{
    try{
        vfi_enable_status_map_.at(tag) = status;
    } catch (const std::exception& e) {
        std::cerr<<e.what()<<std::endl;
        throw std::runtime_error("RobotConstraintManager::set_vfi_status: Fail to update the VFI data!");
    }
}

/**
 * @brief RobotConstraintManager::enable_vfi enables a VFI constraint by its tag. It is equivalent to
 *        set_vfi_status(tag, true).
 * @param tag VFI constraint identifier (must exist in the system)
 * @throws std::runtime_error if tag doesn't exist
 */
void RobotConstraintManager::enable_vfi(const std::string &tag)
{
    set_vfi_status(tag, true);
}

/**
 * @brief RobotConstraintManager::disable_vfi disables a VFI constraint by its tag. It is equivalent to
 *        set_vfi_status(tag, false).
 * @param tag VFI constraint identifier (must exist in the system)
 * @throws std::runtime_error if tag doesn't exist
 */
void RobotConstraintManager::disable_vfi(const std::string &tag)
{
    set_vfi_status(tag, false);
}


/**
 * @brief RobotConstraintManager::get_configuration_limits returns the configuration limits.
 * @return The configuration limits: limits {q_lower_bound, q_upper_bound}
 */
std::tuple<VectorXd, VectorXd> RobotConstraintManager::get_configuration_limits() const
{
    return VFI_M_->get_configuration_limits();
}


/**
 * @brief RobotConstraintManager::get_configuration_velocity_limits returns the configuration velocity limits.
 * @return The configuration velocity limits: {q_dot_lower_bound, q_dot_upper_bound}
 */
std::tuple<VectorXd, VectorXd> RobotConstraintManager::get_configuration_velocity_limits() const
{
    return VFI_M_->get_configuration_velocity_limits();
}


/**
 * @brief RobotConstraintManager::set_configuration_limits sets the configuration limits
 * @param configuration_limits A tuple containing the configuration limits. Example: {q_lower_bound, q_upper_bound}
 */
void RobotConstraintManager::set_configuration_limits(const std::tuple<VectorXd, VectorXd> &configuration_limits)
{
    VFI_M_->set_configuration_limits(configuration_limits);
}

/**
* @brief RobotConstraintManager::set_configuration_velocity_limits sets the configuration velocity limits
* @param configuration_velocity_limits. A tuple containing the configuration velocity limits.
*                      Example: {q_dot_lower_bound, q_dot_upper_bound}
*/
void RobotConstraintManager::set_configuration_velocity_limits(const std::tuple<VectorXd, VectorXd> &configuration_velocity_limits)
{
    VFI_M_->set_configuration_velocity_limits(configuration_velocity_limits);
}


/**
 * @brief RobotConstraintManager::set_configuration_limits_gain sets the gain for the configuration constraints.
 * @param configuration_limits_gain
 */
void RobotConstraintManager::set_configuration_limits_gain(const double& configuration_limits_gain)
{
    configuration_limit_constraint_gain_ = configuration_limits_gain;
}


/**
 * @brief RobotConstraintManager::_get_robot_primitive_offset_from_coppeliasim computes the primitive offsets
 * @param object_name The object name on CoppeliaSim
 * @param joint_index The joint index in which the primitive is kinematically attached.
 * @return The desired offset.
 */
DQ RobotConstraintManager::_get_robot_primitive_offset_from_coppeliasim(const std::string &object_name, const int &joint_index)
{
    DQ x;
    DQ x_offset;
    DQ xprimitive;
    VectorXd q;

    // In some versions of CoppeliaSim, the first simulation step could
    // return invalid data. I read the data five times just in case.
    for (int i=0;i<2;i++)    // Read the data from CoppeliaSim two times.
    {
        q = coppelia_robot_->get_configuration();
        xprimitive = cs_->get_object_pose(object_name);
        x = robot_->fkm(q, joint_index);
        x_offset =  x.conj()*xprimitive;
    }
    return x_offset;
}

std::vector<DQ> RobotConstraintManager::_get_coppeliasim_offsets(const std::vector<std::string>& primitives,
                                                                 [[maybe_unused]] const int& robot_index,
                                                                 const int& joint_index)
{
    const int n = primitives.size();
    std::vector<DQ> offsets;
    offsets.reserve(n);
    for (int i=0;i<n;i++)
    {
        offsets.emplace_back(_get_robot_primitive_offset_from_coppeliasim(primitives.at(i), joint_index));
    }
    return offsets;
}

std::vector<DQ> RobotConstraintManager::_get_workspace_poses(const std::vector<std::string>& entity_environment_primitives)
{
    const int n = entity_environment_primitives.size();
    std::vector<DQ> poses;
    poses.reserve(n);
    for (int i=0;i<n;i++)
        poses.emplace_back(cs_->get_object_pose(entity_environment_primitives.at(i)));
    return poses;
}

//This is used by an old constructor of the class. It will be deprecated
/**
 * @brief RobotConstraintManager::_initial_settings reads the yaml file used to build the VFIs.
 */
void RobotConstraintManager::_initial_settings()
{
    try {
        impl_->config_ = YAML::LoadFile(config_path_);

        if (verbosity_)
        {
            std::cout << "----------------------------------------------" <<std::endl;
            std::cout << "Config file path: " << config_path_ <<std::endl;
            std::cout << "Constraints found in the config file: " << impl_->config_.size() <<std::endl;
        }
        number_of_constraints_ =  impl_->config_.size() ;

        [[maybe_unused]] int i = 0;
        // The outer element is an array
        for(auto dict : impl_->config_) {
            auto name = dict["Description"];
            auto rect = dict["Parameters"];

            for(auto pos : rect) {
                auto raw_vfi_mode = pos["vfi_mode"].as<std::string>();

                if (raw_vfi_mode == "ENVIRONMENT_TO_ROBOT")
                {
                    auto raw_cs_entity_environment = pos["cs_entity_environment"].as<std::string>();
                    auto raw_cs_entity_robot = pos["cs_entity_robot"].as<std::string>() ;
                    auto raw_entity_environment_primitive_type =  pos["entity_environment_primitive_type"].as<std::string>();
                    auto raw_entity_robot_primitive_type = pos["entity_robot_primitive_type"].as<std::string>();
                    //auto raw_robot_index = pos["robot_index"].as<double>();

                    // C++ uses zero-index for the first element. However, the user specifies the first joint with index 1.
                    auto raw_joint_index =  pos["joint_index"].as<double>() - 1;
                    auto raw_safe_distance = pos["safe_distance"].as<double>();
                    auto raw_vfi_gain = pos["vfi_gain"].as<double>();
                    auto raw_direction =  pos["direction"].as<std::string>();
                    auto raw_entity_robot_attached_direction = pos["entity_robot_attached_direction"].as<std::string>();
                    auto raw_entity_environment_attached_direction = pos["entity_environment_attached_direction"].as<std::string>();
                    auto raw_tag = pos["tag"].as<std::string>();

                    YAML_RAW_DATA yaml_raw_data;
                    yaml_raw_data.vfi_mode = "ENVIRONMENT_TO_ROBOT";
                    yaml_raw_data.cs_entity_one_or_environment             = raw_cs_entity_environment;
                    yaml_raw_data.cs_entity_two_or_robot                   = raw_cs_entity_robot;
                    yaml_raw_data.entity_one_primitive_type_or_environment = raw_entity_environment_primitive_type;
                    yaml_raw_data.entity_two_primitive_type_or_robot       = raw_entity_robot_primitive_type;
                    yaml_raw_data.joint_index_one_or_joint_index           = raw_joint_index;
                    yaml_raw_data.joint_index_two                          = -1;
                    yaml_raw_data.safe_distance                            = raw_safe_distance;
                    yaml_raw_data.vfi_gain                                 = raw_vfi_gain;
                    yaml_raw_data.direction                                = raw_direction;
                    yaml_raw_data.entity_robot_attached_direction          = raw_entity_robot_attached_direction;
                    yaml_raw_data.entity_environment_attached_direction    = raw_entity_environment_attached_direction;
                    yaml_raw_data.tag                                      = raw_tag;

                    yaml_raw_data_list_.push_back(yaml_raw_data);
                    yaml_raw_data_map_.try_emplace(yaml_raw_data.tag, yaml_raw_data);


                    VFI_manager::VFI_BUILD_DATA vfi_data;
                    vfi_data.vfi_type = VFI_manager::VFI_TYPE::ENVIRONMENT_TO_ROBOT;
                    vfi_data.vfi_class = VFI_Framework::map_strings_to_vfiClass(raw_entity_robot_primitive_type,
                                                                              raw_entity_environment_primitive_type);
                    vfi_data.direction = VFI_Framework::map_string_to_vfiDirection(raw_direction);
                    vfi_data.safe_distance = raw_safe_distance;
                    vfi_data.vfi_gain = raw_vfi_gain;
                    vfi_data.buffer = 0.0;
                    vfi_data.joint_index_one = raw_joint_index;
                    vfi_data.joint_index_two = -1;
                    vfi_data.primitive_offsets_one = {_get_robot_primitive_offset_from_coppeliasim(raw_cs_entity_robot,
                                                                                                 raw_joint_index)};
                    vfi_data.primitive_offsets_two = {DQ(-1)};
                    vfi_data.robot_attached_direction = VFI_Framework::map_attached_direction_string_to_dq(raw_entity_robot_attached_direction);
                    vfi_data.environment_attached_direction = VFI_Framework::map_attached_direction_string_to_dq(raw_entity_environment_attached_direction);

                    vfi_data.workspace_derivative = DQ(0);
                    vfi_data.environment_poses = {cs_->get_object_pose(raw_cs_entity_environment)};
                    vfi_data.tag = raw_tag;

                    //vfi_build_data_list_.push_back(vfi_data);
                    vfi_build_data_map_.try_emplace(vfi_data.tag, vfi_data);
                    if (verbosity_)
                        show_vfi_build_data(vfi_data.tag);

                    vfi_enable_status_map_.try_emplace(vfi_data.tag, true);

                }else if (raw_vfi_mode == "ROBOT_TO_ROBOT"){

                    auto raw_cs_entity_one = pos["cs_entity_one"].as<std::string>();
                    auto raw_cs_entity_two = pos["cs_entity_two"].as<std::string>();

                    auto raw_entity_one_primitive_type =  pos["entity_one_primitive_type"].as<std::string>();
                    auto raw_entity_two_primitive_type =   pos["entity_two_primitive_type"].as<std::string>();

                    // C++ uses zero-index for the first element. However, the user specifies the first joint with index 1.
                    auto raw_joint_index_one =  pos["joint_index_one"].as<double>()-1;
                    auto raw_joint_index_two =  pos["joint_index_two"].as<double>()-1;

                    auto raw_safe_distance = pos["safe_distance"].as<double>();
                    auto raw_vfi_gain = pos["vfi_gain"].as<double>();
                    auto raw_tag = pos["tag"].as<std::string>();


                    YAML_RAW_DATA yaml_raw_data;
                    yaml_raw_data.vfi_mode = "ENVIRONMENT_TO_ROBOT";
                    yaml_raw_data.cs_entity_one_or_environment             = raw_cs_entity_one;
                    yaml_raw_data.cs_entity_two_or_robot                   = raw_cs_entity_two;
                    yaml_raw_data.entity_one_primitive_type_or_environment = raw_entity_one_primitive_type;
                    yaml_raw_data.entity_two_primitive_type_or_robot       = raw_entity_two_primitive_type;
                    yaml_raw_data.joint_index_one_or_joint_index           = raw_joint_index_one;
                    yaml_raw_data.joint_index_two                          = raw_joint_index_two;
                    yaml_raw_data.safe_distance                            = raw_safe_distance;
                    yaml_raw_data.vfi_gain                                 = raw_vfi_gain;
                    yaml_raw_data.direction                                = "NONE";
                    yaml_raw_data.entity_robot_attached_direction          = "NONE";
                    yaml_raw_data.entity_environment_attached_direction    = "NONE";
                    yaml_raw_data.tag                                      = raw_tag;

                    yaml_raw_data_list_.push_back(yaml_raw_data);
                    yaml_raw_data_map_.try_emplace(yaml_raw_data.tag, yaml_raw_data);


                    VFI_manager::VFI_BUILD_DATA vfi_data;
                    vfi_data.vfi_type = VFI_manager::VFI_TYPE::ROBOT_TO_ROBOT;
                    vfi_data.vfi_class = VFI_Framework::map_strings_to_vfiClass(raw_entity_one_primitive_type,
                                                                              raw_entity_two_primitive_type);
                    vfi_data.direction = VFI_Framework::DIRECTION::RESTRICTED_ZONE;
                    vfi_data.safe_distance = raw_safe_distance;
                    vfi_data.buffer = 0.0;
                    vfi_data.vfi_gain = raw_vfi_gain;
                    vfi_data.joint_index_one = raw_joint_index_one;
                    vfi_data.joint_index_two = raw_joint_index_two;
                    vfi_data.primitive_offsets_one = {_get_robot_primitive_offset_from_coppeliasim(raw_cs_entity_one,
                                                                                                 raw_joint_index_one)};
                    vfi_data.primitive_offsets_two = {_get_robot_primitive_offset_from_coppeliasim(raw_cs_entity_two,
                                                                                                 raw_joint_index_two)};
                    vfi_data.robot_attached_direction = DQ(-1);
                    vfi_data.environment_attached_direction = DQ(-1);

                    vfi_data.workspace_derivative = DQ(0);
                    vfi_data.environment_poses = {DQ(-1)};
                    vfi_data.tag = raw_tag;

                    //vfi_build_data_list_.push_back(vfi_data);
                    vfi_build_data_map_.try_emplace(vfi_data.tag, vfi_data);
                    if (verbosity_)
                        show_vfi_build_data(vfi_data.tag);
                    vfi_enable_status_map_.try_emplace(vfi_data.tag, true);


                }else if(raw_vfi_mode =="CONFIGURATION_LIMITS"){
                    auto q_min_raw = pos["q_min"].as<std::vector<double>>();
                    auto q_max_raw = pos["q_max"].as<std::vector<double>>();
                    auto unit_raw  = pos["unit"].as<std::string>();
                    _check_unit(unit_raw);
                    auto vfi_gain  = pos["vfi_gain"].as<double>();

                    VectorXd q_min = DQ_robotics_extensions::Conversions::std_vector_double_to_vectorxd(q_min_raw);
                    VectorXd q_max = DQ_robotics_extensions::Conversions::std_vector_double_to_vectorxd(q_max_raw);
                    if (unit_raw == "DEG")
                    {
                        q_min = DQ_robotics::deg2rad(q_min);
                        q_max = DQ_robotics::deg2rad(q_max);
                    }
                    VFI_M_->set_configuration_limits({q_min, q_max});
                    set_configuration_limits_gain(vfi_gain);
                    if (verbosity_)
                    {
                        std::cout<<"---------------------------------------------"<<std::endl;
                        std::cout<<"Configuration limits  (Radians)              "<<std::endl;
                        std::cout<<"q_min:    "<<q_min.transpose()<<std::endl;
                        std::cout<<"q_max:    "<<q_max.transpose()<<std::endl;
                        std::cout<<"vfi_gain: "<<vfi_gain<<std::endl;
                        std::cout<<"---------------------------------------------"<<std::endl;
                    }

                }else if(raw_vfi_mode =="CONFIGURATION_VELOCITY_LIMITS"){
                    auto q_dot_min_raw = pos["q_dot_min"].as<std::vector<double>>();
                    auto q_dot_max_raw = pos["q_dot_max"].as<std::vector<double>>();
                    auto unit_raw      = pos["unit"].as<std::string>();
                    _check_unit(unit_raw);

                    VectorXd q_dot_min = DQ_robotics_extensions::Conversions::std_vector_double_to_vectorxd(q_dot_min_raw);
                    VectorXd q_dot_max = DQ_robotics_extensions::Conversions::std_vector_double_to_vectorxd(q_dot_max_raw);
                    if (unit_raw == "DEG")
                    {
                        q_dot_min = DQ_robotics::deg2rad(q_dot_min);
                        q_dot_max = DQ_robotics::deg2rad(q_dot_max);
                    }
                    VFI_M_->set_configuration_velocity_limits({q_dot_min, q_dot_max});
                    if (verbosity_)
                    {
                        std::cout<<"---------------------------------------------"<<std::endl;
                        std::cout<<"Configuration velocity limits  (Rad/s)    "<<std::endl;
                        std::cout<<"q_dot_min:    "<<q_dot_min.transpose()<<std::endl;
                        std::cout<<"q_dot_max:    "<<q_dot_max.transpose()<<std::endl;
                        std::cout<<"---------------------------------------------"<<std::endl;
                    }
                }
                else{
                    throw std::runtime_error("Wrong vfi mode. USE ENVIRONMENT_TO_ROBOT, ROBOT_TO_ROBOT, CONFIGURATION_LIMITS or"
                                             "CONFIGURATION_VELOCITY_LIMITS");
                }
                i++;
            }
        }

        std::cout << "----------------------------------------------" <<std::endl;

    } catch(const YAML::BadFile& e) {
        std::cerr << e.msg << std::endl;
        throw std::runtime_error(e.msg);
        //return 1;
    } catch(const YAML::ParserException& e) {
        std::cerr << e.msg << std::endl;
        throw std::runtime_error(e.msg);
        //return 1;
    }

}


}
