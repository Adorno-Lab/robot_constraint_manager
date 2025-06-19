#include <dqrobotics_extensions/robot_constraint_manager/numpy.hpp>


#include <iostream>

#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

using namespace Eigen;

int main()
{
    auto A = MatrixXd::Identity(3,3);
    auto B = MatrixXd(3,3);

    auto flag = DQ_robotics_extensions::Checkers::check_column_matrix_sizes(A,B, DQ_robotics_extensions::Checkers::MODE::PANIC);
    std::cout<<"flag: "<<flag<<std::endl;

    auto flag2 = DQ_robotics_extensions::Checkers::check_row_matrix_sizes(A,B, DQ_robotics_extensions::Checkers::MODE::PANIC);
    std::cout<<"flag2: "<<flag2<<std::endl;

    std::vector<MatrixXd> VM = {A,B};
    auto C = DQ_robotics_extensions::Numpy::block_diag({A,B});
    std::cout<<C<<std::endl;

    std::cout<<DQ_robotics_extensions::Numpy::linspace(0,10,5).transpose()<<std::endl;

    VectorXd v1 = (VectorXd(4)<<1,2,3,4).finished();
    VectorXd v2 = (VectorXd(4)<<10,20,30,40).finished();
    std::vector<VectorXd> VV = {v1,v2};

    std::cout<<DQ_robotics_extensions::Numpy::linspace(v1, v2, 5)<<std::endl;
    std::cout<<DQ_robotics_extensions::Conversions::std_vector_vectorxd_to_vectorxd(VV)<<std::endl;

    std::vector<std::string> names = {"joint1", "joint2", "joint3"};

    std::cout<<DQ_robotics_extensions::Checkers::check_equal_sizes(VV,VM, names, DQ_robotics_extensions::Checkers::MODE::DO_NOT_PANIC)<<std::endl;

    auto v3 = DQ_robotics_extensions::Numpy::vstack(v1, v2);
    std::cout<<"v3: "<<v3<<std::endl;

    return 0;
}
