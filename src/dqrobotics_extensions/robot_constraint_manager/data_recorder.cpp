#include <dqrobotics_extensions/robot_constraint_manager/data_recorder.hpp>
#include <stdexcept>
#include <string>


DQ_robotics_extensions::DataRecorder::DataRecorder(const TYPE &type)
:type_{type}
{

}


void DQ_robotics_extensions::DataRecorder::add_data(const VectorXd &data)
{
    switch (type_) {
        case TYPE::VECTORXD:
        {
            if (first_call_)
            {
                vector_size_ =  data.size();
                matrix_data_ = data;
                first_call_ = false;
            }
            matrix_data_ = Numpy::resize(matrix_data_, matrix_data_.rows(), matrix_data_.cols()+1);
            i_ = i_ + 1;
            matrix_data_.block(0,i_, vector_size_, 1) = data;
            //matrix_data_.block(0, matrix_data_.cols() - 1, vector_size_, 1) = data;
            break;
        };
        default:
            throw std::runtime_error("DataRecorder::add_data: Wrong type of data.");

    }
}

void DQ_robotics_extensions::DataRecorder::add_data(const double &data)
{
    Eigen::VectorXd v(1);
    v(0) = data;
    add_data(v);
}

void DQ_robotics_extensions::DataRecorder::save_data(const std::string &filename)
{
    data_logger_.open(filename + ".csv");
    int m = matrix_data_.rows();
    int n = matrix_data_.cols();
    for (int j=0; j<n;j++)
    {
        for (int i=0;i<m;i++)
        {
            if (i<m-1)
                data_logger_ << matrix_data_(i,j)<<",";
            else
                data_logger_ << matrix_data_(i,j)<<","<<'\n';
        }
    }
    data_logger_.close();
}

void DQ_robotics_extensions::DataRecorder::show_data()
{
   std::cout<<matrix_data_<<std::endl;
}
