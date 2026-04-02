
#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.h>
#include <dqrobotics/interfaces/coppeliasim/robots/FrankaEmikaPandaCoppeliaSimZMQRobot.h>
#include <dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/utils/DQ_Math.h>
#include <memory>
#include <dqrobotics/robot_control/DQ_ClassicQPController.h>
#include <dqrobotics/solvers/DQ_QPOASESSolver.h>
#include <dqrobotics_extensions/robot_constraint_manager/robot_constraint_manager.hpp>
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_yaml.hpp>


int main()
{

    auto cs = std::make_shared<DQ_CoppeliaSimInterfaceZMQ>();
    try {
        cs->connect("localhost", 23000, 2000);
        cs->set_stepping_mode(true);
        auto panda = std::make_shared<FrankaEmikaPandaCoppeliaSimZMQRobot>("Franka", cs);
        auto panda_model =  std::make_shared<DQ_SerialManipulatorMDH>(panda->kinematics());
        auto solver = std::make_shared<DQ_QPOASESSolver>();

        DQ_ClassicQPController controller(panda_model, solver);
        controller.set_control_objective(ControlObjective::Translation);
        controller.set_gain(1.0);
        controller.set_damping(0.01);



        auto vcr = std::make_shared<DQ_robotics_extensions::VFIConfigurationFileYaml>();
        DQ_robotics_extensions::RobotConstraintManager rcm{cs, panda, panda_model, vcr,
                                                            "vfi_constraints_2.yaml", true};

        // The new format does not include the configuration limits or configuration velocity limits.
        // However, we can still added in the code

        VectorXd q_min = (VectorXd(7) << -2.3093,-1.5133,-2.4937, -2.7478,-2.4800, 0.8521, -2.6895).finished();
        VectorXd q_max = (VectorXd(7) <<  2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094,  2.6895).finished();
        VectorXd q_dot_min = (VectorXd(7) <<-2, -1, -1.5, -1.25, -3, -1.5, -3).finished();
        VectorXd q_dot_max = (VectorXd(7) <<2,  1,  1.5,  1.25,  3,  1.5,  3).finished();
        rcm.set_configuration_limits({q_min, q_max});
        rcm.set_configuration_limits_gain(1.0);
        rcm.set_configuration_velocity_limits({q_dot_min, q_dot_max});
        cs->start_simulation();


        std::cout<<"Starting teleoperation "<<std::endl;

        int i=0;
        double T=0.01;
        double w=0.1;

        int ITERATIONS = 10000;
        std::string tag = "C5";
        int losses = 0;
        for (int i=0;i<ITERATIONS;i++)
        {

            DQ xd = cs->get_object_pose("ReferenceFrame");
            auto q = panda->get_configuration();
            auto [A,b] = rcm.get_inequality_constraints(q, false, false);

            controller.set_inequality_constraint(A,b);
            auto u = controller.compute_setpoint_control_signal(q, xd.translation().vec4());
            panda->set_target_configuration_velocities(u);


            double dist = rcm.get_vfi_distance_error(tag);
            std::cout<<"Constraint tag="+tag+". distance_error: "<<dist<<std::endl;
            if (dist < 0)
            {
                std::cerr<<"Collision!"<<std::endl;
                losses++;
            }

            // Varying-time VFI

            std::string ctag = "C5";
            DQ z = 0.6*i_ + 0.4*k_+ 0.1*sin(w*i*T)*k_;
            DQ z_dot = 0.1*w*cos(w*i*T)*k_;
            DQ xsphere = 1 + 0.5*E_*z;
            cs->set_object_pose("/obs_sphere", xsphere);

            rcm.update_vfi_workspace_pose(ctag, xsphere);
            rcm.update_vfi_workspace_derivative(ctag, z_dot);
            cs->trigger_next_simulation_step();
        }
        std::cout<<"Teleoperation finished."<<std::endl;

        cs->stop_simulation();
        std::cout<<"Losses: "<<losses<<std::endl;


    }
    catch (const std::runtime_error& e)
    {
        std::cerr<<e.what()<<std::endl;
    }
    return 0;
}


