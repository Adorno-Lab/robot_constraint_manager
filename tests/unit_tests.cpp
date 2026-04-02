#include <gtest/gtest.h>
#include <dqrobotics/DQ.h>
#include <dqrobotics_extensions/robot_constraint_manager/robot_constraint_manager.hpp>
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_yaml.hpp>
#include <dqrobotics/interfaces/coppeliasim/robots/FrankaEmikaPandaCoppeliaSimZMQRobot.h>

using namespace DQ_robotics;
using namespace Eigen;

namespace My{
class InterfaceUnitTests : public testing::Test {

protected:

    std::shared_ptr<DQ_robotics_extensions::VFIConfigurationFileYaml> vcr_;
    std::shared_ptr<DQ_robotics_extensions::RobotConstraintManager> rcm_;
    std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> cs_;
    std::shared_ptr<FrankaEmikaPandaCoppeliaSimZMQRobot> panda_;
    std::shared_ptr<DQ_SerialManipulatorMDH> robot_model_;

    VectorXd q_min_ = (VectorXd(7) << -2.3093,-1.5133,-2.4937, -2.7478,-2.4800, 0.8521, -2.6895).finished();
    VectorXd q_max_ = (VectorXd(7) <<  2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094,  2.6895).finished();
    VectorXd q_dot_min_ = (VectorXd(7) <<-2, -1, -1.5, -1.25, -3, -1.5, -3).finished();
    VectorXd q_dot_max_ = (VectorXd(7) <<2,  1,  1.5,  1.25,  3,  1.5,  3).finished();

    VectorXd q_panda_home_ = (VectorXd(7)<<0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4).finished();
    InterfaceUnitTests() {
        cs_ = std::make_unique<DQ_CoppeliaSimInterfaceZMQ>();
        cs_->connect("localhost", 23000, 1000);
        panda_ = std::make_shared<FrankaEmikaPandaCoppeliaSimZMQRobot>("Franka", cs_);
        robot_model_ = std::make_shared<DQ_SerialManipulatorMDH>(panda_->kinematics());

        vcr_ = std::make_shared<DQ_robotics_extensions::VFIConfigurationFileYaml>();
        rcm_ = std::make_shared<DQ_robotics_extensions::RobotConstraintManager>(cs_, panda_, robot_model_, vcr_,
                                                                                "vfi_constraints_3.yaml", true);
    }

    ~InterfaceUnitTests() override {
        cs_->stop_simulation();
    }

    void delay(const int& time_ms = 100){
        std::this_thread::sleep_for(std::chrono::milliseconds(time_ms));
    }

    void SetUp() override {
        // Code here will be called immediately after the constructor (right
        // before each test).

    }

    void TearDown() override {
        // Code here will be called immediately after each test (right
        // before the destructor).
        cs_->stop_simulation();
    }

}; // class InterfaceUnitTests



//----------------------TESTS HERE----------------------------------------
TEST_F(InterfaceUnitTests, distances){
    auto q = panda_->get_configuration();
    auto [A,b] = rcm_->get_inequality_constraints(q, false, false);
    auto tags = rcm_->get_vfi_tags();
    for (auto& tag : tags)
    {
        double distance = rcm_->get_vfi_distance_error(tag);
        std::cout<<tag + " distance: "<<distance<<std::endl;
    }

};


//-----------------------------------------------------------------------

} // namespace My

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
