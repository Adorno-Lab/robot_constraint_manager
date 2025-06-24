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
                                               const std::string &yaml_file_path,
                                               const std::tuple<VectorXd, VectorXd> &configuration_limits,
                                               const std::tuple<VectorXd, VectorXd> &configuration_velocity_limits,
                                               const double& configuration_limit_constraint_gain,
                                               const VFI_Framework::LEVEL &level)
    :coppelia_robot_{coppeliasim_robot}, cs_{coppelia_interface}, config_path_{yaml_file_path},
    level_{level}, robot_{robot}, configuration_limit_constraint_gain_{configuration_limit_constraint_gain}
{
    impl_ = std::make_shared<RobotConstraintManager::Impl>();

    VFI_M_ = std::make_shared<DQ_robotics_extensions::VFI_manager>(robot->get_dim_configuration_space(),
                                                                   configuration_limits,
                                                                   configuration_velocity_limits);
}

void RobotConstraintManager::set_vfi_position_constraints_gain(const double &vfi_position_constraints_gain)
{
    configuration_limit_constraint_gain_ = vfi_position_constraints_gain;
}

std::vector<std::tuple<double, double> > RobotConstraintManager::get_distances_and_error_distances() const
{
    return distances_and_error_distances_;
}

/**
 * @brief RobotConstraintManager::get_inequality_constraints
 * @param q
 * @return
 */
std::tuple<MatrixXd, VectorXd> RobotConstraintManager::get_inequality_constraints(const VectorXd &q)
{
    const int n = vfi_mode_list_.size();
    const int robot_dim = robot_->get_dim_configuration_space();

    VFI_M_->add_configuration_limits(configuration_limit_constraint_gain_, q);
    VFI_M_->add_configuration_velocity_limits();

    std::vector<std::tuple<double, double>> distances_and_error_distances;

    for (int i = 0; i<n; i++)
    {
        if (vfi_mode_list_.at(i) == VFI_manager::VFI_MODE::ENVIRONMENT_TO_ROBOT)
        {
            DQ x = (robot_->fkm(q, joint_index_list_one_.at(i)))*dq_offset_list_one_.at(i);
            MatrixXd J = haminus8(dq_offset_list_one_.at(i))*robot_->pose_jacobian(q, joint_index_list_one_.at(i));
            if (J.cols() != robot_dim)
                J = DQ_robotics_extensions::Numpy::resize(J, J.rows(), robot_dim);
            DQ x_workspace = cs_entity_environment_DQ_list_.at(i);

            distances_and_error_distances.push_back(

                VFI_M_->add_vfi_constraint(direction_list_.at(i),
                                           vfi_type_list_.at(i),
                                           safe_distance_list_.at(i),
                                           vfi_gain_list_.at(i),
                                           J,
                                           x,
                                           robot_attached_dir_list_.at(i),
                                           x_workspace,
                                           envir_attached_dir_list_.at(i),
                                           workspace_derivative_list_.at(i))
                );

        }
        else{ //vfi_mode_list_.at(i) == VFI_manager::VFI_MODE::ROBOT_TO_ROBOT
            DQ x1 =  (robot_->fkm(q, joint_index_list_one_.at(i)))*dq_offset_list_one_.at(i);
            MatrixXd J1 = haminus8(dq_offset_list_one_.at(i))*robot_->pose_jacobian(q, joint_index_list_one_.at(i));
            DQ x2 =  (robot_->fkm(q, joint_index_list_two_.at(i)))*dq_offset_list_two_.at(i);
            MatrixXd J2 = haminus8(dq_offset_list_two_.at(i))*robot_->pose_jacobian(q, joint_index_list_two_.at(i));

            distances_and_error_distances.push_back(
                VFI_M_->add_vfi_rpoint_to_rpoint(safe_distance_list_.at(i), vfi_gain_list_.at(i), {J1, x1}, {J2, x2})
                );


        }
        distances_and_error_distances_ = distances_and_error_distances;
    }
    return VFI_M_->get_inequality_constraints();
}


/*
std::vector<std::tuple<double, double> > RobotConstraintManager::get_distance_and_error_distance(const std::string &tag)
{

}*/

DQ RobotConstraintManager::_get_robot_primitive_offset_from_coppeliasim(const std::string &object_name, const int &joint_index)
{
    DQ x;
    DQ x_offset;
    DQ xprimitive;
    VectorXd q;
    for (int i=0;i<5;i++)
    {
        q = cs_->get_joint_positions(robot_jointnames_);
        xprimitive = cs_->get_object_pose(object_name);
        x = robot_->fkm(q, joint_index);
        x_offset =  x.conj()*xprimitive;
    }
    return x_offset;
}

void RobotConstraintManager::_initial_settings()
{
    try {
        impl_->config_ = YAML::LoadFile(config_path_);
        //const int size = config.size();
        std::cout << "----------------------------------------------" <<std::endl;
        std::cout << "Config file path: " << config_path_ <<std::endl;
        std::cout << "Constraints found in the config file: " << impl_->config_.size() <<std::endl;
        number_of_constraints_ =  impl_->config_.size() ;
        std::cout << "----------------------------------------------" <<std::endl;
        //----------------------------------------------------------------------------------
        std::cout<<"----------data----------------"<<std::endl;
        std::cout<<"      VFI MODE     "<<"   CS entity env/one  "<<" CS entity robot/two "<<" env/one type "<<
            " entv/two type " << " joint index 1 "<<" joint index 2 "<<" safe dist "<< "dir" <<" ent dir1 "<<" ent dir2 "<<std::endl;

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
                    auto raw_joint_index =  pos["joint_index"].as<double>();
                    auto raw_safe_distance = pos["safe_distance"].as<double>();
                    auto raw_vfi_gain = pos["vfi_gain"].as<double>();
                    auto raw_direction =  pos["direction"].as<std::string>();
                    auto raw_entity_robot_attached_direction = pos["entity_robot_attached_direction"].as<std::string>();
                    auto raw_entity_environment_attached_direction = pos["entity_environment_attached_direction"].as<std::string>();

                    vfi_mode_list_.push_back(VFI_manager::VFI_MODE::ENVIRONMENT_TO_ROBOT);
                    vfi_type_list_.     push_back(VFI_Framework::map_strings_to_vfiType(raw_entity_robot_primitive_type,
                                                                                   raw_entity_environment_primitive_type));
                    direction_list_.    push_back(VFI_Framework::map_string_to_direction(raw_direction));
                    safe_distance_list_.push_back(raw_safe_distance);
                    vfi_gain_list_.push_back(raw_vfi_gain);
                    joint_index_list_one_.push_back(raw_joint_index);
                    joint_index_list_two_.push_back(-1);

                    dq_offset_list_one_.    push_back(_get_robot_primitive_offset_from_coppeliasim(raw_cs_entity_robot,
                                                                                               raw_joint_index));
                    dq_offset_list_two_.push_back(DQ(-1));

                    robot_attached_dir_list_.push_back(VFI_Framework::map_attached_direction_string_to_dq(raw_entity_robot_attached_direction));
                    envir_attached_dir_list_.push_back(VFI_Framework::map_attached_direction_string_to_dq(raw_entity_environment_attached_direction));
                    workspace_derivative_list_.push_back(DQ(0));
                    cs_entity_environment_DQ_list_.push_back(cs_->get_object_pose(raw_cs_entity_environment));


                    std::cout << raw_vfi_mode << ",\t"
                              << raw_cs_entity_environment  << ",\t"
                              << raw_cs_entity_robot << ",\t"
                              << raw_entity_robot_primitive_type<< ",\t"
                              << raw_entity_environment_primitive_type <<  ",     \t"
                              << joint_index_list_one_.at(i) << ",    \t"
                              << joint_index_list_two_.at(i) << ",     \t"
                              << raw_safe_distance << ",\t"
                              << robot_attached_dir_list_.at(i) << ",\t"
                              << envir_attached_dir_list_.at(i) <<
                        std::endl;

                }else if (raw_vfi_mode == "ROBOT_TO_ROBOT"){

                    auto raw_cs_entity_one = pos["cs_entity_one"].as<std::string>();
                    auto raw_cs_entity_two = pos["cs_entity_two"].as<std::string>();

                    auto raw_entity_one_primitive_type =  pos["entity_one_primitive_type"].as<std::string>();
                    auto raw_entity_two_primitive_type=   pos["entity_two_primitive_type"].as<std::string>();

                    auto raw_joint_index_one =  pos["joint_index_one"].as<double>();
                    auto raw_joint_index_two =  pos["joint_index_two"].as<double>();

                    auto raw_safe_distance = pos["safe_distance"].as<double>();
                    auto raw_vfi_gain = pos["vfi_gain"].as<double>();

                    vfi_mode_list_.push_back(VFI_manager::VFI_MODE::ROBOT_TO_ROBOT);
                    vfi_type_list_.push_back(VFI_Framework::map_strings_to_vfiType(raw_entity_one_primitive_type,
                                                                                   raw_entity_two_primitive_type));
                    //vfi_type_list_.push_back(VFI_Framework::VFI_TYPE::RPOINT_TO_POINT); // This is the only one supported

                    direction_list_.push_back(VFI_Framework::DIRECTION::KEEP_ROBOT_OUTSIDE);
                    safe_distance_list_.push_back(raw_safe_distance);
                    vfi_gain_list_.push_back(raw_vfi_gain);

                    joint_index_list_one_.push_back(raw_joint_index_one);
                    joint_index_list_two_.push_back(raw_joint_index_two);

                    dq_offset_list_one_.push_back(_get_robot_primitive_offset_from_coppeliasim(raw_cs_entity_one,
                                                                                               raw_joint_index_one));
                    dq_offset_list_two_.push_back(_get_robot_primitive_offset_from_coppeliasim(raw_cs_entity_two,
                                                                                               raw_joint_index_two));

                    workspace_derivative_list_.push_back(DQ(0));


                    //The following variables are not used in this mode.
                    //However I want to keep all information in the same index.
                    //--------------------------------------------------
                    robot_attached_dir_list_.push_back(DQ(-1));
                    envir_attached_dir_list_.push_back(DQ(-1));
                    cs_entity_environment_DQ_list_.push_back(DQ(-1));
                    //--------------------------------------------------

                    std::cout << raw_vfi_mode << ",\t"
                              << raw_cs_entity_one  << ",\t"
                              << raw_cs_entity_two << ",  \t"
                              << raw_entity_one_primitive_type<< ",\t"
                              << raw_entity_two_primitive_type << ",     \t"
                              << joint_index_list_one_.at(i) << ",    \t"
                              << joint_index_list_two_.at(i) <<  ",        \t"
                              << raw_safe_distance << ",\t"
                              << robot_attached_dir_list_.at(i) << ",\t"
                              << envir_attached_dir_list_.at(i) <<
                        std::endl;


                }else{
                    throw std::runtime_error("Wrong vfi mode. USE ENVIRONMENT_TO_ROBOT or ROBOT_TO_ROBOT");
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
