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
                                               const double& configuration_limit_constraint_gain, const bool &verbosity,
                                               const VFI_Framework::LEVEL &level)
    :coppelia_robot_{coppeliasim_robot}, cs_{coppelia_interface}, config_path_{yaml_file_path},
    level_{level}, robot_{robot}, configuration_limit_constraint_gain_{configuration_limit_constraint_gain},
    verbosity_{verbosity}
{
    impl_ = std::make_shared<RobotConstraintManager::Impl>();

    VFI_M_ = std::make_shared<DQ_robotics_extensions::VFI_manager>(robot->get_dim_configuration_space(),
                                                                   configuration_limits,
                                                                   configuration_velocity_limits);
    _initial_settings();
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
    const int n = vfi_data_list_.size();
    const int robot_dim = robot_->get_dim_configuration_space();

    VFI_M_->add_configuration_limits(configuration_limit_constraint_gain_, q);
    VFI_M_->add_configuration_velocity_limits();

    std::vector<std::tuple<double, double>> distances_and_error_distances;

    for (int i = 0; i<n; i++)
    {
        if (vfi_data_list_.at(i).vfi_mode == VFI_manager::VFI_MODE::ENVIRONMENT_TO_ROBOT)
        {
            const int index = vfi_data_list_.at(i).joint_index_one;
            const DQ offset = vfi_data_list_.at(i).primitive_offset_one;
            DQ x = (robot_->fkm(q, index))*offset;
            MatrixXd J = haminus8(offset)*robot_->pose_jacobian(q, index);
            if (J.cols() != robot_dim)
                J = DQ_robotics_extensions::Numpy::resize(J, J.rows(), robot_dim);


            distances_and_error_distances.push_back(
                VFI_M_->add_vfi_constraint(vfi_data_list_.at(i).tag,
                                           vfi_data_list_.at(i).direction,
                                           vfi_data_list_.at(i).vfi_type,
                                           vfi_data_list_.at(i).safe_distance,
                                           vfi_data_list_.at(i).vfi_gain,
                                           J,
                                           x,
                                           vfi_data_list_.at(i).robot_attached_direction,
                                           vfi_data_list_.at(i).cs_entity_environment_pose, // x_workspace
                                           vfi_data_list_.at(i).environment_attached_direction,
                                           vfi_data_list_.at(i).workspace_derivative)
                );

        }
        else{ //vfi_mode_list_.at(i) == VFI_manager::VFI_MODE::ROBOT_TO_ROBOT
            const int index_1 = vfi_data_list_.at(i).joint_index_one;
            const DQ offset_1 = vfi_data_list_.at(i).primitive_offset_one;

            DQ x1 =  (robot_->fkm(q, index_1))*offset_1;
            MatrixXd J1 = haminus8(offset_1)*robot_->pose_jacobian(q, index_1);

            const int index_2 = vfi_data_list_.at(i).joint_index_two;
            const DQ offset_2 = vfi_data_list_.at(i).primitive_offset_two;

            DQ x2 =  (robot_->fkm(q, index_2))*offset_2;
            MatrixXd J2 = haminus8(offset_2)*robot_->pose_jacobian(q, index_2);

            distances_and_error_distances.push_back(
                VFI_M_->add_vfi_rpoint_to_rpoint(vfi_data_list_.at(i).tag,
                                                 vfi_data_list_.at(i).safe_distance,
                                                 vfi_data_list_.at(i).vfi_gain,
                                                 {J1, x1},
                                                 {J2, x2})
                );


        }
        distances_and_error_distances_ = distances_and_error_distances;
    }
    return VFI_M_->get_inequality_constraints();
}

double RobotConstraintManager::get_vfi_distance_error(const std::string &tag)
{
    return VFI_M_->get_vfi_distance_error(tag);
}


/*
std::vector<std::tuple<double, double> > RobotConstraintManager::get_distance_and_error_distance(const std::string &tag)
{

}*/

void RobotConstraintManager::_show_constraints()
{
/*
 *                     std::cout << raw_vfi_mode << ",\t"
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
 */
}

DQ RobotConstraintManager::_get_robot_primitive_offset_from_coppeliasim(const std::string &object_name, const int &joint_index)
{
    DQ x;
    DQ x_offset;
    DQ xprimitive;
    VectorXd q;
    for (int i=0;i<5;i++)
    {
        q = coppelia_robot_->get_configuration();
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

        if (verbosity_)
        {
        std::cout << "----------------------------------------------" <<std::endl;
        std::cout << "Config file path: " << config_path_ <<std::endl;
        std::cout << "Constraints found in the config file: " << impl_->config_.size() <<std::endl;
        number_of_constraints_ =  impl_->config_.size() ;
        std::cout << "----------------------------------------------" <<std::endl;
        //----------------------------------------------------------------------------------
        std::cout<<"----------data----------------"<<std::endl;
        std::cout<<"      VFI MODE     "<<"   CS entity env/one  "<<" CS entity robot/two "<<" env/one type "<<
            " entv/two type " << " joint index 1 "<<" joint index 2 "<<" safe dist "<< "dir" <<" ent dir1 "<<" ent dir2 "<<std::endl;
        }


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
                    vfi_data.direction = VFI_Framework::map_string_to_direction(raw_direction);
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

                    vfi_data_list_.push_back(vfi_data);


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

                    vfi_data_list_.push_back(vfi_data);



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
