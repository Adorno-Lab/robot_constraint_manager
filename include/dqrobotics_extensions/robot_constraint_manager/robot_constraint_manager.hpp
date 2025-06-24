#pragma once
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobot.h>
#include <dqrobotics_extensions/robot_constraint_manager/vfi_manager.hpp>
#include <memory>

using namespace Eigen;
using namespace DQ_robotics;

namespace DQ_robotics_extensions
{
class RobotConstraintManager
{
private:
    class Impl;
    std::shared_ptr<Impl> impl_;
protected:
    std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> cs_;
    std::string config_path_;
    VFI_manager::LEVEL level_;
    std::shared_ptr<DQ_Kinematics> robot_;
    std::shared_ptr<DQ_CoppeliaSimRobot> coppelia_robot_;
    std::shared_ptr<DQ_robotics_extensions::VFI_manager> VFI_M_;
    double configuration_limit_constraint_gain_;

    std::vector<std::string> robot_jointnames_;


    VectorXd initial_robot_configuration_;
    int number_of_constraints_;


    std::vector<VFI_manager::VFI_MODE> vfi_mode_list_;
    std::vector<VFI_manager::VFI_TYPE> vfi_type_list_;
    std::vector<VFI_manager::DIRECTION> direction_list_;
    std::vector<double> safe_distance_list_;
    std::vector<DQ> robot_attached_dir_list_;
    std::vector<DQ> envir_attached_dir_list_;
    std::vector<DQ> workspace_derivative_list_;
    std::vector<DQ> cs_entity_environment_DQ_list_;

    std::vector<double> vfi_gain_list_;
    std::vector<int> joint_index_list_one_;
    std::vector<int> joint_index_list_two_;

    std::vector<DQ> dq_offset_list_one_;
    std::vector<DQ> dq_offset_list_two_;

    std::vector<std::tuple<double, double>> distances_and_error_distances_;

    DQ _get_robot_primitive_offset_from_coppeliasim(const std::string& object_name, const int& joint_index);
    void _initial_settings();
public:
    RobotConstraintManager(const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ>& coppelia_interface,
                           const std::shared_ptr<DQ_CoppeliaSimRobot>& coppeliasim_robot,
                           const std::shared_ptr<DQ_Kinematics>& robot,
                           const std::string &yaml_file_path,
                           const std::tuple<VectorXd, VectorXd>& configuration_limits,
                           const std::tuple<VectorXd, VectorXd>& configuration_velocity_limits,
                           const double&  configuration_limit_constraint_gain,
                           const VFI_manager::LEVEL& level = VFI_manager::LEVEL::VELOCITIES);

    void set_vfi_position_constraints_gain(const double& vfi_position_constraints_gain);

    std::vector<std::tuple<double, double>> get_distances_and_error_distances() const;
    //std::vector<std::tuple<double, double>> get_distance_and_error_distance(const std::string& tag);

    std::tuple<MatrixXd, VectorXd> get_inequality_constraints(const VectorXd& q);
};
}
