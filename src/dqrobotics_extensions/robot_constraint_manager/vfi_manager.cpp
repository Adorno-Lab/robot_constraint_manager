#include <dqrobotics_extensions/robot_constraint_manager/vfi_manager.hpp>

namespace DQ_robotics_extensions  {


/**
 * @brief VFI_manager::VFI_manager constructor of the class.
 * @param dim_configuration The dimension of the configuration space
 * @param configuration_limits The configuration limits: {q_lower_bound, q_upper_bound}
 * @param configuration_velocity_limits The configuration velocity limits: {q_dot_lower_bound, q_dot_upper_bound}
 * @param level The desired level. Use VELOCITIES for first order kinematics, or
                                   ACCELERATIONS for second order kinematics.
 */
VFI_manager::VFI_manager(const int &dim_configuration,
                         const std::tuple<VectorXd, VectorXd> &configuration_limits,
                         const std::tuple<VectorXd, VectorXd> &configuration_velocity_limits,
                         const LEVEL &level)
    :dim_configuration_(dim_configuration),level_(level)
{
    constraint_manager_ = std::make_shared<DQ_robotics_extensions::ConstraintsManager>(dim_configuration_);
    I_ = MatrixXd::Identity(dim_configuration_, dim_configuration_);

    set_configuration_limits(configuration_limits);
    set_configuration_velocity_limits(configuration_velocity_limits);

    if (level_ == VFI_Framework::LEVEL::ACCELERATIONS)
        throw std::runtime_error("VFI_manager::VFI_manager Accelerations are unsupported!");
}

/**
 * @brief VFI_manager::_update_map
 * @param tag
 * @param vfi_parameters
 */
void VFI_manager::_update_vfi_parameters_map(const std::string &tag, const VFI_LOG_DATA &vfi_parameters)
{
    auto search = vfi_parameters_map_.find(tag);
    if (search != vfi_parameters_map_.end())
    { // tag found in map. Updated the map with
        vfi_parameters_map_.at(tag) = vfi_parameters;
    }
    else
    {   // tag not found in map. Add the gag
        vfi_parameters_map_.try_emplace(tag, vfi_parameters);
    }
}

/**
 * @brief VFI_manager::_get_data_from_vfi_parameters_map
 * @param tag
 * @return
 */
VFI_manager::VFI_LOG_DATA VFI_manager::_get_data_from_vfi_parameters_map(const std::string tag)
{
    try {
        return vfi_parameters_map_.at(tag);
    } catch (const std::runtime_error& e) {
        std::cerr<<e.what()<<std::endl;
        throw std::runtime_error("VFI TAG not found!");
    }

}

/**
 * @brief VFI_manager::_add_vfi_constraint adds the inequality constraint in the stack
 * @param Jd The distance Jacobian
 * @param b  The vector term in the vfi inequality. For instance b = vfi_gain*(error) + residual
 * @param direction Use KEEP_ROBOT_OUTSIDE, or  KEEP_ROBOT_INSIDE to define the VFI behaviour.
 */
void VFI_manager::_add_vfi_constraint(const MatrixXd &Jd, const VectorXd &b, const DIRECTION &direction)
{
    if(direction == DIRECTION::KEEP_ROBOT_OUTSIDE)
        constraint_manager_->add_inequality_constraint(-Jd, b);
    else
        constraint_manager_->add_inequality_constraint(Jd, -b);
}

/**
 * @brief VFI_manager::_check_vector_initialization throws an exception with a custom message if
 *                     the given vector is not initialized.
 * @param q The vector to be checked.
 * @param msg The custom message to be displayed.
 */
void VFI_manager::_check_vector_initialization(const VectorXd &q, const std::string &msg)
{
    if (q.size() == 0)
        throw std::runtime_error(msg);
}

/**
 * @brief VFI_manager::add_vfi_joint_position_constraints adds configuration limits using the
 *                     inequality constraints (10) defined in
 *                     Adaptive Constrained Kinematic Control using Partial or Complete Task-Space Measurements.
 *                     Marinho, M. M. & Adorno, B. V.
 *                     IEEE Transactions on Robotics (T-RO),
 *                     38(6):3498–3513, December, 2022. Presented at ICRA'23.
 *
 * @param gain
 * @param configuration
 */
void VFI_manager::add_configuration_limits(const double &gain, const VectorXd &configuration)
{
    _check_vector_initialization(q_min_, std::string("The configuration limits were not defined."));
    constraint_manager_->add_inequality_constraint(-I_,   gain*(configuration - q_min_));
    constraint_manager_->add_inequality_constraint( I_,  -gain*(configuration - q_max_));

}

/**
 * @brief VFI_manager::add_configuration_velocity_limits adds the configuration velocity limits using the
 *                     inequality constraints (7) defined in
 *                     Adaptive Constrained Kinematic Control using Partial or Complete Task-Space Measurements.
 *                     Marinho, M. M. & Adorno, B. V.
 *                     IEEE Transactions on Robotics (T-RO),
 *                     38(6):3498–3513, December, 2022. Presented at ICRA'23.
 */
void VFI_manager::add_configuration_velocity_limits()
{
    _check_vector_initialization(q_dot_min_, std::string("The joint velocity limits were not defined."));
    constraint_manager_->add_inequality_constraint(-I_, -q_dot_min_);
    constraint_manager_->add_inequality_constraint( I_,  q_dot_max_);
}


/**
 * @brief VFI_manager::add_vfi_rpoint_to_rpoint adds the rpoint-to-rpoint VFI constraint to the stack.
 * @param safe_distance The safe distance
 * @param vfi_gain  The vfi gain
 * @param robot_pose_jacobian_and_pose_one The tuple containing the pose Jacobian and pose of the first point attached to the robot
 * @param robot_pose_jacobian_and_pose_two The tuple containing the pose Jacobian and pose of the second point attached to the robot
 * @return A tuple containing the square distance and the square error. {square_d , square_error}
 */
void VFI_manager::add_vfi_rpoint_to_rpoint(const std::string &tag,
                                                                 const double &safe_distance,
                                                                 const double &vfi_gain,
                                                                 const std::tuple<MatrixXd, DQ> &robot_pose_jacobian_and_pose_one,
                                                                 const std::tuple<MatrixXd, DQ> &robot_pose_jacobian_and_pose_two)
{
    const double square_safe_distance = pow(safe_distance, 2);
    const MatrixXd robot_pose_jacobian_one = std::get<0>(robot_pose_jacobian_and_pose_one);
    const DQ robot_pose_one = std::get<1>(robot_pose_jacobian_and_pose_one);
    const MatrixXd robot_pose_jacobian_two = std::get<0>(robot_pose_jacobian_and_pose_two);
    const DQ robot_pose_two = std::get<1>(robot_pose_jacobian_and_pose_two);

    const DQ& x1 = robot_pose_one;
    const DQ& x2 = robot_pose_two;
    const MatrixXd& J1 = robot_pose_jacobian_one;
    const MatrixXd& J2 = robot_pose_jacobian_two;
    const DQ t1 = x1.translation();
    const DQ t2 = x2.translation();
    const MatrixXd Jt1 = DQ_robotics_extensions::Numpy::resize(DQ_Kinematics::translation_jacobian(J1, x1), 4, dim_configuration_);
    const MatrixXd Jt2 = DQ_robotics_extensions::Numpy::resize(DQ_Kinematics::translation_jacobian(J2, x2), 4, dim_configuration_);

    const MatrixXd Jd = 2*vec4(x1.translation()-x2.translation()).transpose()*(Jt1-Jt2);

    const double square_d = DQ_Geometry::point_to_point_squared_distance(t1, t2);
    const double residual = 0;
    const double square_error = square_d- square_safe_distance;
    VectorXd b = DQ_robotics_extensions::CVectorXd({vfi_gain*(square_error) + residual});
    _add_vfi_constraint(Jd, b, VFI_Framework::DIRECTION::KEEP_ROBOT_OUTSIDE);
    //#############-log data-###############
    const double d = std::sqrt(square_d);
    VFI_LOG_DATA data;
    data.vfi_type = VFI_TYPE::RPOINT_TO_POINT;
    data.distance = d;
    data.square_distance = square_d;
    data.distance_error = d-safe_distance;
    data.square_distance_error = square_error;
    data.line_to_line_angle_rad = -1;
    _update_vfi_parameters_map(tag, data);
    //######################################
}

/**
 * @brief VFI_manager::add_vfi_constraint
 * @param tag
 * @param direction
 * @param vfi_type
 * @param safe_distance
 * @param vfi_gain
 * @param robot_pose_jacobian
 * @param robot_pose
 * @param robot_attached_direction
 * @param workspace_pose
 * @param workspace_attached_direction
 * @param workspace_derivative
 */
void VFI_manager::add_vfi_constraint(const std::string &tag,
                                                           const DIRECTION &direction,
                                                           const VFI_TYPE &vfi_type,
                                                           const double &safe_distance,
                                                           const double &vfi_gain,
                                                           const MatrixXd &robot_pose_jacobian,
                                                           const DQ &robot_pose,
                                                           const DQ &robot_attached_direction,
                                                           const DQ &workspace_pose,
                                                           const DQ &workspace_attached_direction,
                                                           const DQ &workspace_derivative)
{
    switch(vfi_type)
    {

    case VFI_TYPE::RPOINT_TO_POINT:
    {
        const double square_safe_distance = pow(safe_distance, 2);
        const DQ& x = robot_pose;
        const DQ& x_ = workspace_pose;
        const DQ p = x.translation();
        const DQ p_ = x_.translation();
        const MatrixXd Jt     = DQ_Kinematics::translation_jacobian(robot_pose_jacobian, robot_pose);
        const MatrixXd Jd     = DQ_Kinematics::point_to_point_distance_jacobian(Jt, p, p_);
        const double square_d = DQ_Geometry::point_to_point_squared_distance(p, p_);
        const double residual = DQ_Kinematics::point_to_point_residual(p, p_, workspace_derivative);
        const double square_error = square_d- square_safe_distance;
        VectorXd b = DQ_robotics_extensions::CVectorXd({vfi_gain*(square_error) + residual});
        _add_vfi_constraint(Jd, b, direction);
        //#############-log data-###############
        const double d = std::sqrt(square_d);
        VFI_LOG_DATA data;
        data.vfi_type = VFI_TYPE::RPOINT_TO_POINT;
        data.distance = d;
        data.square_distance = square_d;
        data.distance_error = d-safe_distance;
        data.square_distance_error = square_error;
        data.line_to_line_angle_rad = -1;
        _update_vfi_parameters_map(tag, data);
        //######################################
        break;
    }
    case VFI_TYPE::RPOINT_TO_PLANE:
    {
        const DQ p = robot_pose.translation();
        const DQ& x_ = workspace_pose;
        const DQ plane_normal = x_.P()*workspace_attached_direction*x_.P().conj();
        const DQ plane_point =  x_.translation();
        const DQ workspace_plane  = plane_normal + E_*dot(plane_point, plane_normal);
        const MatrixXd Jt     =  DQ_Kinematics::translation_jacobian(robot_pose_jacobian, robot_pose);
        const MatrixXd Jd     =  DQ_Kinematics::point_to_plane_distance_jacobian(Jt, p, workspace_plane);
        const double residual =  DQ_Kinematics::point_to_plane_residual(p, workspace_derivative);
        const double d = DQ_Geometry::point_to_plane_distance(p, workspace_plane);
        const double error = d - safe_distance;
        VectorXd b = DQ_robotics_extensions::CVectorXd({vfi_gain*(error) + residual});
        _add_vfi_constraint(Jd, b, direction);
        //#############-log data-###############
        VFI_LOG_DATA data;
        data.vfi_type = VFI_TYPE::RPOINT_TO_PLANE;
        data.distance = d;
        data.square_distance = d*d;
        data.distance_error = error;
        data.square_distance_error = -1;
        data.line_to_line_angle_rad = -1;
        _update_vfi_parameters_map(tag, data);
        //######################################
        break;
    }

    case VFI_TYPE::RPOINT_TO_LINE:
    {
        const DQ& x= workspace_pose;
        const DQ l_= (x.P())*workspace_attached_direction*(x.P().conj());
        const DQ p_= x.translation();
        const DQ workspace_line = l_ + E_*cross(p_, l_);
        const double square_safe_distance = pow(safe_distance, 2);
        const DQ p = robot_pose.translation();
        const MatrixXd Jt =  DQ_Kinematics::translation_jacobian(robot_pose_jacobian, robot_pose);
        const MatrixXd Jd = DQ_Kinematics::point_to_line_distance_jacobian(Jt, p, workspace_line);
        const double residual = DQ_Kinematics::point_to_line_residual(p, workspace_line, workspace_derivative);
        const double square_d = DQ_Geometry::point_to_line_squared_distance(p, workspace_line);
        const double square_error = square_d - square_safe_distance;
        VectorXd b = DQ_robotics_extensions::CVectorXd({vfi_gain*(square_error) + residual});
        _add_vfi_constraint(Jd, b, direction);
        //#############-log data-###############
        const double d = std::sqrt(square_d);
        VFI_LOG_DATA data;
        data.vfi_type = VFI_TYPE::RPOINT_TO_POINT;
        data.distance = d;
        data.square_distance = square_d;
        data.distance_error = d-safe_distance;
        data.square_distance_error = square_error;
        data.line_to_line_angle_rad = -1;
        _update_vfi_parameters_map(tag, data);
        //######################################
        break;
    }
    case VFI_TYPE::RLINE_TO_LINE_ANGLE:
    {
        const DQ workspace_line = (workspace_pose.P())*workspace_attached_direction*(workspace_pose.P().conj());
        const double safe_angle = safe_distance*(pi/180);  //Convert to radians
        const DQ& robot_line_direction = robot_attached_direction;
        const MatrixXd Jl = DQ_Kinematics::line_jacobian(robot_pose_jacobian, robot_pose,robot_line_direction);
        const DQ r = robot_pose.P();
        const DQ robot_line = r*(robot_line_direction)*r.conj();
        const MatrixXd Jfphi = DQ_Kinematics::line_to_line_angle_jacobian(Jl,robot_line,workspace_line);
        const double fsafe = 2-2*cos(safe_angle);
        const double phi = DQ_Geometry::line_to_line_angle(robot_line, workspace_line);
        const double f = 2-2*cos(phi);
        const double ferror = f-fsafe;
        const double residual = DQ_Kinematics::line_to_line_angle_residual(robot_line,workspace_line,-workspace_derivative);
        VectorXd b = DQ_robotics_extensions::CVectorXd({vfi_gain*(ferror) + residual});
        _add_vfi_constraint(Jfphi, b, direction);
        //#############-log data-###############
        VFI_LOG_DATA data;
        data.vfi_type = VFI_TYPE::RLINE_TO_LINE_ANGLE;
        data.distance = f;
        data.square_distance = -1;
        data.distance_error = ferror;
        data.square_distance_error = -1;
        data.line_to_line_angle_rad = phi;
        _update_vfi_parameters_map(tag, data);
        //######################################
        break;
    }
    case VFI_TYPE::RLINE_TO_LINE:
    {
        throw std::runtime_error("VFI_TYPE::RLINE_TO_LINE is unsupported");
        break;
    }

    case VFI_TYPE::RLINE_TO_POINT:
    {
        throw std::runtime_error("VFI_TYPE::RLINE_TO_POINT is unsupported");
        break;
    }
    }

}


/**
 * @brief VFI_manager::set_configuration_limits sets the configuration limits
 * @param configuration_limits A tuple containing the configuration limits. Example: {q_lower_bound, q_upper_bound}
 */
void VFI_manager::set_configuration_limits(const std::tuple<VectorXd, VectorXd>& configuration_limits)
{
    const VectorXd q_lower_bound = std::get<0>(configuration_limits);
    const VectorXd q_upper_bound = std::get<1>(configuration_limits);
    DQ_robotics_extensions::Checkers::check_equal_sizes(q_lower_bound, q_upper_bound, DQ_robotics_extensions::Checkers::MODE::PANIC,
                              std::string("The sizes are incompatibles. q_lower_bound has size ") + std::to_string(q_lower_bound.size())
                            + std::string(" and q_upper_bound has size ") + std::to_string(q_upper_bound.size()));
    DQ_robotics_extensions::Checkers::check_equal_sizes(q_lower_bound, VectorXd::Zero(dim_configuration_),DQ_robotics_extensions::Checkers::MODE::PANIC,
                        std::string("The sizes are incompatibles. The joint limits have size ") + std::to_string(q_lower_bound.size())
                            + std::string(" and the robot configuration has size ") + std::to_string(dim_configuration_));
    q_min_ = q_lower_bound;
    q_max_ = q_upper_bound;

}

/**
 * @brief VFI_manager::set_configuration_velocity_limits sets the configuration velocity limits
 * @param configuration_velocity_limits. A tuple containing the configuration velocity limits.
 *                      Example: {q_dot_lower_bound, q_dot_upper_bound}
 */
void VFI_manager::set_configuration_velocity_limits(const std::tuple<VectorXd, VectorXd> &configuration_velocity_limits)
{
    const VectorXd q_dot_lower_bound = std::get<0>(configuration_velocity_limits);
    const VectorXd q_dot_upper_bound = std::get<1>(configuration_velocity_limits);
    DQ_robotics_extensions::Checkers::check_equal_sizes(q_dot_lower_bound, q_dot_upper_bound, DQ_robotics_extensions::Checkers::MODE::PANIC,
                        std::string("The sizes are incompatibles. q_dot_lower_bound has size ") + std::to_string(q_dot_lower_bound.size())
                            + std::string(" and q_dot_upper_bound has size ") + std::to_string(q_dot_upper_bound.size()));
    DQ_robotics_extensions::Checkers::check_equal_sizes(q_dot_lower_bound, VectorXd::Zero(dim_configuration_), DQ_robotics_extensions::Checkers::MODE::PANIC,
                        std::string("The sizes are incompatibles. The joint limits have size ") + std::to_string(q_dot_lower_bound.size())
                            + std::string(" and the robot configuration has size ") + std::to_string(dim_configuration_));
    q_dot_min_ = q_dot_lower_bound;
    q_dot_max_ = q_dot_upper_bound;

}

/**
 * @brief VFI_manager::get_inequality_constraints returns a tuple with the inequality constraints given by
 *                                                 Ax <= b
 * @return A tuple containing the inequality constraints {A,b}.
 */
std::tuple<MatrixXd, VectorXd> VFI_manager::get_inequality_constraints()
{
    return constraint_manager_->get_inequality_constraints();
}

double VFI_manager::get_vfi_distance_error(const std::string &tag)
{
    return _get_data_from_vfi_parameters_map(tag).distance_error;
}

double VFI_manager::get_line_to_line_angle(const std::string &tag)
{
    auto data = _get_data_from_vfi_parameters_map(tag);
    switch (data.vfi_type) {
    case VFI_Framework::VFI_TYPE::RLINE_TO_LINE_ANGLE:
        return _get_data_from_vfi_parameters_map(tag).line_to_line_angle_rad;
    case VFI_Framework::VFI_TYPE::RLINE_TO_LINE:
        throw std::runtime_error("VFI_manager::get_line_to_line_angle: not supported for RLINE_TO_LINE. "
                                 "However, your constraint TAG=\""+tag+"\" is "+map_vfyType_to_string(data.vfi_type));
    default:
        throw std::runtime_error("VFI_manager::get_line_to_line_angle is available for RLINE_TO_LINE_ANGLE only. "
                                 "However, your constraint TAG=\""+tag+"\" is "+map_vfyType_to_string(data.vfi_type));
    }

}

/*
std::tuple<MatrixXd, VectorXd> VFI_manager::get_equality_constraints()
{
    return constraint_manager_->get_equality_constraints();
}*/



}
