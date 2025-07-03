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

#include <dqrobotics_extensions/robot_constraint_manager/robot_constraint_manager.hpp>
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
RobotConstraintManager::RobotConstraintManager(const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> &coppelia_interface,
                                               const std::shared_ptr<DQ_CoppeliaSimRobot> &coppeliasim_robot,
                                               const std::shared_ptr<DQ_Kinematics> &robot,
                                               const std::string &yaml_file_path, const bool &verbosity,
                                               const VFI_Framework::LEVEL &level)
    :cs_{coppelia_interface}, config_path_{yaml_file_path}, level_{level},
    robot_{robot}, coppelia_robot_{coppeliasim_robot},
    verbosity_{verbosity}
{
    impl_ = std::make_shared<RobotConstraintManager::Impl>();

    VFI_M_ = std::make_shared<DQ_robotics_extensions::VFI_manager>(robot->get_dim_configuration_space());
    _initial_settings();
}


/**
 * @brief RobotConstraintManager::get_number_vfi_constraints gets the number of VFI constraints set in the config file.
 *                  This number only counts the constraints that require tags. Therefore, the configuration limits, and the
 *                  configuration velocity limits are not taken into account.
 * @return The VFI constraints set in the config file.
 */
int RobotConstraintManager::get_number_vfi_constraints() const
{
    return number_of_constraints_;
}


/**
 * @brief RobotConstraintManager::set_vfi_configuration_constraints_gain sets the gain for the configuration constraints.
 * @param vfi_position_constraints_gain
 */
void RobotConstraintManager::_set_vfi_configuration_constraints_gain(const double &vfi_position_constraints_gain)
{
    configuration_limit_constraint_gain_ = vfi_position_constraints_gain;
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
 * @return A tuple containing the  desired VFIs constraints. For instance, given the constraints A*x <= b, this method returns {A,b}.
 */
std::tuple<MatrixXd, VectorXd> RobotConstraintManager::get_inequality_constraints(const VectorXd &q)
{
    const int n = vfi_build_data_list_.size();
    const int robot_dim = robot_->get_dim_configuration_space();

    VFI_M_->add_configuration_limits(configuration_limit_constraint_gain_, q);
    VFI_M_->add_configuration_velocity_limits();

    for (int i = 0; i<n; i++)
    {
        if (vfi_build_data_list_.at(i).vfi_mode == VFI_manager::VFI_MODE::ENVIRONMENT_TO_ROBOT)
        {
            const int index = vfi_build_data_list_.at(i).joint_index_one;
            const DQ offset = vfi_build_data_list_.at(i).primitive_offset_one;
            DQ x = (robot_->fkm(q, index))*offset;
            MatrixXd J = haminus8(offset)*robot_->pose_jacobian(q, index);
            if (J.cols() != robot_dim)
                J = DQ_robotics_extensions::Numpy::resize(J, J.rows(), robot_dim);

            VFI_M_->add_vfi_constraint(vfi_build_data_list_.at(i).tag,
                                       i,
                                       vfi_build_data_list_.at(i).direction,
                                       vfi_build_data_list_.at(i).vfi_type,
                                       vfi_build_data_list_.at(i).safe_distance,
                                       vfi_build_data_list_.at(i).vfi_gain,
                                       J,
                                       x,
                                       vfi_build_data_list_.at(i).robot_attached_direction,
                                       vfi_build_data_list_.at(i).cs_entity_environment_pose, // x_workspace
                                       vfi_build_data_list_.at(i).environment_attached_direction,
                                       vfi_build_data_list_.at(i).workspace_derivative);


        }
        else{ //vfi_mode_list_.at(i) == VFI_manager::VFI_MODE::ROBOT_TO_ROBOT
            const int index_1 = vfi_build_data_list_.at(i).joint_index_one;
            const DQ offset_1 = vfi_build_data_list_.at(i).primitive_offset_one;

            DQ x1 =  (robot_->fkm(q, index_1))*offset_1;
            MatrixXd J1 = haminus8(offset_1)*robot_->pose_jacobian(q, index_1);

            const int index_2 = vfi_build_data_list_.at(i).joint_index_two;
            const DQ offset_2 = vfi_build_data_list_.at(i).primitive_offset_two;

            DQ x2 =  (robot_->fkm(q, index_2))*offset_2;
            MatrixXd J2 = haminus8(offset_2)*robot_->pose_jacobian(q, index_2);


            VFI_M_->add_vfi_rpoint_to_rpoint(vfi_build_data_list_.at(i).tag,
                                             i,
                                             vfi_build_data_list_.at(i).safe_distance,
                                             vfi_build_data_list_.at(i).vfi_gain,
                                             {J1, x1},
                                             {J2, x2});
        }

    }
    return VFI_M_->get_inequality_constraints();
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
        std::cout<<"VFI mode:                        "<<VFI_Framework::map_vfiMode_to_string(data.vfi_mode)<<std::endl;
        std::cout<<"VFI type:                        "<<VFI_Framework::map_vfiType_to_string(data.vfi_type)<<std::endl;
        std::cout<<"Direction:                       "<<VFI_Framework::map_vfiDirection_to_string(data.direction)<<std::endl;
        std::cout<<"Safe distance:                   "<<data.safe_distance<<std::endl;
        std::cout<<"VFI gain:                        "<<data.vfi_gain<<std::endl;
        std::cout<<"Joint index one:                 "<<data.joint_index_one<<std::endl;
        std::cout<<"Joint index two:                 "<<data.joint_index_two<<std::endl;
        std::cout<<"primitive_offset_one:            "<<data.primitive_offset_one<<std::endl;
        std::cout<<"primitive_offset_two:            "<<data.primitive_offset_two<<std::endl;
        std::cout<<"robot_attached_direction:        "<<data.robot_attached_direction<<std::endl;
        std::cout<<"environment_attached_direction:  "<<data.environment_attached_direction<<std::endl;
        std::cout<<"workspace derivative:            "<<data.workspace_derivative<<std::endl;
        std::cout<<"cs_entity_environment_pose:      "<<data.cs_entity_environment_pose<<std::endl;


        std::cout<<"---------------------------------------------"<<std::endl;
    } catch (const std::runtime_error& e) {
        std::cerr<<e.what()<<std::endl;
        throw std::runtime_error("RobotConstraintManager::show_vfi_build_data: VFI TAG not found!");
    }

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
    for (int i=0;i<5;i++)    // Read the data from CoppeliaSim five times.
    {
        q = coppelia_robot_->get_configuration();
        xprimitive = cs_->get_object_pose(object_name);
        x = robot_->fkm(q, joint_index);
        x_offset =  x.conj()*xprimitive;
    }
    return x_offset;
}

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

        int i = 0;
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

                    VFI_BUILD_DATA vfi_data;
                    vfi_data.vfi_mode = VFI_manager::VFI_MODE::ENVIRONMENT_TO_ROBOT;
                    vfi_data.vfi_type = VFI_Framework::map_strings_to_vfiType(raw_entity_robot_primitive_type,
                                                                              raw_entity_environment_primitive_type);
                    vfi_data.direction = VFI_Framework::map_string_to_vfiDirection(raw_direction);
                    vfi_data.safe_distance = raw_safe_distance;
                    vfi_data.vfi_gain = raw_vfi_gain;
                    vfi_data.joint_index_one = raw_joint_index;
                    vfi_data.joint_index_two = -1;
                    vfi_data.primitive_offset_one = _get_robot_primitive_offset_from_coppeliasim(raw_cs_entity_robot,
                                                                                                 raw_joint_index);
                    vfi_data.primitive_offset_two = DQ(-1);
                    vfi_data.robot_attached_direction = VFI_Framework::map_attached_direction_string_to_dq(raw_entity_robot_attached_direction);
                    vfi_data.environment_attached_direction = VFI_Framework::map_attached_direction_string_to_dq(raw_entity_environment_attached_direction);

                    vfi_data.workspace_derivative = DQ(0);
                    vfi_data.cs_entity_environment_pose = cs_->get_object_pose(raw_cs_entity_environment);
                    vfi_data.tag = raw_tag;

                    vfi_build_data_list_.push_back(vfi_data);
                    vfi_build_data_map_.try_emplace(vfi_data.tag, vfi_data);
                    if (verbosity_)
                        show_vfi_build_data(vfi_data.tag);

                }else if (raw_vfi_mode == "ROBOT_TO_ROBOT"){

                    auto raw_cs_entity_one = pos["cs_entity_one"].as<std::string>();
                    auto raw_cs_entity_two = pos["cs_entity_two"].as<std::string>();

                    auto raw_entity_one_primitive_type =  pos["entity_one_primitive_type"].as<std::string>();
                    auto raw_entity_two_primitive_type=   pos["entity_two_primitive_type"].as<std::string>();

                    // C++ uses zero-index for the first element. However, the user specifies the first joint with index 1.
                    auto raw_joint_index_one =  pos["joint_index_one"].as<double>()-1;
                    auto raw_joint_index_two =  pos["joint_index_two"].as<double>()-1;

                    auto raw_safe_distance = pos["safe_distance"].as<double>();
                    auto raw_vfi_gain = pos["vfi_gain"].as<double>();
                    auto raw_tag = pos["tag"].as<std::string>();


                    VFI_BUILD_DATA vfi_data;
                    vfi_data.vfi_mode = VFI_manager::VFI_MODE::ROBOT_TO_ROBOT;
                    vfi_data.vfi_type = VFI_Framework::map_strings_to_vfiType(raw_entity_one_primitive_type,
                                                                              raw_entity_two_primitive_type);
                    vfi_data.direction = VFI_Framework::DIRECTION::KEEP_ROBOT_OUTSIDE;
                    vfi_data.safe_distance = raw_safe_distance;
                    vfi_data.vfi_gain = raw_vfi_gain;
                    vfi_data.joint_index_one = raw_joint_index_one;
                    vfi_data.joint_index_two = raw_joint_index_two;
                    vfi_data.primitive_offset_one = _get_robot_primitive_offset_from_coppeliasim(raw_cs_entity_one,
                                                                                                 raw_joint_index_one);
                    vfi_data.primitive_offset_two = _get_robot_primitive_offset_from_coppeliasim(raw_cs_entity_two,
                                                                                                 raw_joint_index_two);
                    vfi_data.robot_attached_direction = DQ(-1);
                    vfi_data.environment_attached_direction = DQ(-1);

                    vfi_data.workspace_derivative = DQ(0);
                    vfi_data.cs_entity_environment_pose = DQ(-1);
                    vfi_data.tag = raw_tag;

                    vfi_build_data_list_.push_back(vfi_data);
                    vfi_build_data_map_.try_emplace(vfi_data.tag, vfi_data);
                    if (verbosity_)
                        show_vfi_build_data(vfi_data.tag);


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
                    _set_vfi_configuration_constraints_gain(vfi_gain);
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
