/*
    Creates version 3 configuration files using the CoppeliaSim scene panda_example.ttt.

    Usage:
        panda_v3_generator migrate <v2_file> <v3_output_file> [port]
            Converts a version 2 configuration file into a version 3 configuration file.

        panda_v3_generator update <v3_template_file> <v3_output_file> [port]
            Fills the poses and offsets of a version 3 configuration file using the scene objects with the same
            names as the entities. The rest of the file is not modified.

    CoppeliaSim must be running with the scene panda_example.ttt (port 23000 by default). The simulation does not
    need to be running. The offsets are computed with the current configuration of the robot in the scene.
*/

#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.h>
#include <dqrobotics/interfaces/coppeliasim/robots/FrankaEmikaPandaCoppeliaSimZMQRobot.h>
#include <dqrobotics_extensions/robot_constraint_manager/robot_constraint_manager.hpp>
#include <dqrobotics_extensions/robot_constraint_manager/vfi_configuration_file_v3_generator.hpp>
#include <dqrobotics_extensions/robot_constraint_manager/vfi_coppeliasim_scene.hpp>
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_yaml.hpp>
#include <algorithm>
#include <iostream>
#include <memory>
#include <string>

using namespace DQ_robotics;
using namespace DQ_robotics_extensions;

namespace {

void print_usage()
{
    std::cout<<"Usage:"<<std::endl
             <<"    panda_v3_generator migrate <v2_file> <v3_output_file> [port]"<<std::endl
             <<"    panda_v3_generator update <v3_template_file> <v3_output_file> [port]"<<std::endl;
}

void print_entities(const VFIConfigurationFile::DOCUMENT_V3& document)
{
    std::cout<<"Environment entities:"<<std::endl;
    for (const auto& entity : document.environment_entities)
        std::cout<<"    "<<entity.name<<std::endl;
    std::cout<<"Robot entities:"<<std::endl;
    for (const auto& entity : document.robot_entities)
        std::cout<<"    "<<entity.name<<" (joint_index: "<<entity.joint_index<<")"<<std::endl;
}

/**
 * @brief compare_with_v2 compares the constraints built from the version 2 file (using CoppeliaSim) with the
 *        constraints built from the version 3 file (using only the kinematic model) at random configurations.
 * @return The maximum absolute difference between the constraints.
 */
double compare_with_v2(const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ>& cs,
                       const std::shared_ptr<FrankaEmikaPandaCoppeliaSimZMQRobot>& panda,
                       const std::shared_ptr<DQ_SerialManipulatorMDH>& model,
                       const std::string& v2_file,
                       const std::string& v3_file)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    RobotConstraintManager rcm_v2{cs, panda, model, std::make_shared<VFIConfigurationFileYaml>(), v2_file};
#pragma GCC diagnostic pop
    RobotConstraintManager rcm_v3{model, std::make_shared<VFIConfigurationFileYaml>(), v3_file};

    double max_difference = 0;
    for (int i = 0; i < 100; i++)
    {
        const VectorXd q = VectorXd::Random(model->get_dim_configuration_space());
        const auto [A_v2, b_v2] = rcm_v2.get_inequality_constraints(q, false, false);
        const auto [A_v3, b_v3] = rcm_v3.get_inequality_constraints(q, false, false);
        if (A_v2.rows() != A_v3.rows())
            throw std::runtime_error("The number of constraints of the version 2 and 3 files is different.");
        max_difference = std::max({max_difference, (A_v2 - A_v3).cwiseAbs().maxCoeff(), (b_v2 - b_v3).cwiseAbs().maxCoeff()});
    }
    return max_difference;
}

}

int main(int argc, char* argv[])
{
    if (argc < 4 || argc > 5)
    {
        print_usage();
        return 1;
    }
    const std::string mode = argv[1];
    const std::string input_file = argv[2];
    const std::string output_file = argv[3];
    const int port = (argc == 5) ? std::stoi(argv[4]) : 23000;
    if (mode != "migrate" && mode != "update")
    {
        print_usage();
        return 1;
    }

    try
    {
        auto cs = std::make_shared<DQ_CoppeliaSimInterfaceZMQ>();
        cs->connect("localhost", port, 2000);
        auto panda = std::make_shared<FrankaEmikaPandaCoppeliaSimZMQRobot>("Franka", cs);
        // The kinematic model must be the model used at runtime (see panda_example.cpp)
        auto model = std::make_shared<DQ_SerialManipulatorMDH>(panda->kinematics());

        VFIConfigurationFileV3Generator generator{model, std::make_shared<VFICoppeliaSimScene>(cs, panda)};

        VFIConfigurationFileYaml reader;
        reader.load_data(input_file);
        VFIConfigurationFile::DOCUMENT_V3 document;
        if (mode == "migrate")
        {
            if (reader.get_vfi_file_version() != 2)
                throw std::runtime_error(input_file + " is not a version 2 configuration file.");
            document = generator.create_from_v2(std::get<VFIConfigurationFile::DOCUMENT_V2>(reader.get_document()), "Franka");
            document.metadata.source = "panda_example.ttt (migrated from " + input_file + ")";
        }
        else
        {
            if (reader.get_vfi_file_version() != 3)
                throw std::runtime_error(input_file + " is not a version 3 configuration file.");
            document = generator.update_from_scene(std::get<VFIConfigurationFile::DOCUMENT_V3>(reader.get_document()));
        }

        VFIConfigurationFileYaml().save_document(document, output_file);
        print_entities(document);

        // Check that the file can be loaded
        RobotConstraintManager rcm{model, std::make_shared<VFIConfigurationFileYaml>(), output_file};
        std::cout<<output_file<<" contains "<<rcm.get_number_of_vfi_constraints()<<" VFIs."<<std::endl;

        if (mode == "migrate")
        {
            const double difference = compare_with_v2(cs, panda, model, input_file, output_file);
            std::cout<<"Maximum difference between the constraints of the version 2 and 3 files: "<<difference<<std::endl;
            if (difference > 1e-9)
            {
                std::cerr<<"The constraints of the version 2 and 3 files are different!"<<std::endl;
                return 1;
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr<<e.what()<<std::endl;
        return 1;
    }
    return 0;
}
