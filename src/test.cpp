
#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.h>
#include <dqrobotics/interfaces/coppeliasim/robots/FrankaEmikaPandaCoppeliaSimZMQRobot.h>
#include <dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/utils/DQ_Math.h>
#include <memory>
#include <dqrobotics/robot_control/DQ_ClassicQPController.h>
#include <dqrobotics/solvers/DQ_QPOASESSolver.h>
//#include <dqrobotics/solvers/DQ_PROXQPSolver.h>
//#include <dqrobotics/solvers/DQ_OSQPSolver.h>
#include <dqrobotics_extensions/robot_constraint_manager/robot_constraint_manager.hpp>

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int);
void sig_int_handler(int)
{
    kill_this_process = true;
}

int main()
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
        throw std::runtime_error("::Error setting the signal int handler.");


    auto cs = std::make_shared<DQ_CoppeliaSimInterfaceZMQ>();
    try {
        cs->connect("localhost", 23000, 1500);
        auto panda = std::make_shared<FrankaEmikaPandaCoppeliaSimZMQRobot>("Franka", cs);
        auto panda_model =  std::make_shared<DQ_SerialManipulatorMDH>(panda->kinematics());
        auto solver = std::make_shared<DQ_QPOASESSolver>();

        DQ_ClassicQPController controller(panda_model, solver);
        controller.set_control_objective(ControlObjective::Translation);
        controller.set_gain(1.0);
        controller.set_damping(0.01);

        std::string yaml_path = "/home/juanjqo/git/robot_constraint_manager/cfg/vfi_constraints.yaml";
        const VectorXd q_max = ((VectorXd(7) <<  2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094,  2.6895).finished());
        const VectorXd q_min = ((VectorXd(7) << -2.3093,-1.5133,-2.4937, -2.7478,-2.4800, 0.8521, -2.6895).finished());
        const VectorXd q_dot_min = ((VectorXd(7) << -2, -1, -1.5, -1.25, -3, -1.5, -3).finished());
        const VectorXd q_dot_max = ((VectorXd(7) <<  2,  1,  1.5,  1.25,  3,  1.5,  3).finished());

        MatrixXd A;
        VectorXd b;
        MatrixXd Aeq;
        VectorXd beq;
        DQ_robotics_extensions::RobotConstraintManager
            rcm{cs, panda, panda_model, yaml_path, {q_min, q_max}, {q_dot_min, q_dot_max}, 1.0, true};


        cs->start_simulation();

        std::cout<<"Starting teleoperation "<<std::endl;
        while ( not kill_this_process)
        {
            DQ xd = cs->get_object_pose("ReferenceFrame");
            auto q = panda->get_configuration();
            auto ineq_constraints = rcm.get_inequality_constraints(q);
            A = std::get<0>(ineq_constraints);
            b = std::get<1>(ineq_constraints);
            controller.set_inequality_constraint(A,b);
            auto u = controller.compute_setpoint_control_signal(q, xd.translation().vec4());
            panda->set_target_configuration_velocities(u);
            std::string tag = "C3";
            std::cout<<"Constraint tag="+tag+". distance_error: "<<rcm.get_vfi_distance_error(tag)<<std::endl;
        }
        std::cout<<"Teleoperation finished."<<std::endl;

        cs->stop_simulation();
    }
    catch (const std::runtime_error& e)
    {
        std::cerr<<e.what()<<std::endl;
    }
}


