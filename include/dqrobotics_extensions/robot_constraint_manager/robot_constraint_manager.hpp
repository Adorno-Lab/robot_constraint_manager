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
    struct VFI_BUILD_DATA{
        VFI_Framework::VFI_MODE vfi_mode;
        VFI_Framework::VFI_TYPE vfi_type;
        VFI_Framework::DIRECTION direction;
        double safe_distance;
        double vfi_gain;
        int joint_index_one;
        int joint_index_two;
        DQ primitive_offset_one;
        DQ primitive_offset_two;
        DQ robot_attached_direction;
        DQ environment_attached_direction;
        DQ workspace_derivative;
        DQ cs_entity_environment_pose;
        std::string tag;
    };
    std::vector<VFI_BUILD_DATA> vfi_data_list_;
protected:
    std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> cs_;
    std::string config_path_;
    VFI_Framework::LEVEL level_;
    std::shared_ptr<DQ_Kinematics> robot_;
    std::shared_ptr<DQ_CoppeliaSimRobot> coppelia_robot_;
    std::shared_ptr<DQ_robotics_extensions::VFI_manager> VFI_M_;
    double configuration_limit_constraint_gain_;



    VectorXd initial_robot_configuration_;
    int number_of_constraints_;

    std::vector<std::tuple<double, double>> distances_and_error_distances_;
    bool verbosity_{true};

    void _show_constraints();

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
                           const bool& verbosity = false,
                           const VFI_manager::LEVEL& level = VFI_manager::LEVEL::VELOCITIES);

    void set_vfi_position_constraints_gain(const double& vfi_position_constraints_gain);

    std::vector<std::tuple<double, double>> get_distances_and_error_distances() const;
    //std::vector<std::tuple<double, double>> get_distance_and_error_distance(const std::string& tag);

    std::tuple<MatrixXd, VectorXd> get_inequality_constraints(const VectorXd& q);

    double get_vfi_distance_error(const std::string& tag);
};
}
