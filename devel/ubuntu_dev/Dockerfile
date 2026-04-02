FROM juanjqo/ubuntu_24_dqrobotics_ros_jazzy
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"



# Install QT
RUN apt update && apt-get install qt6-base-dev qt6-tools-dev-tools -y


# Install robot_constraint_editor
RUN cd ~/git && git clone https://github.com/Adorno-Lab/robot_constraint_editor
RUN cd ~/git/robot_constraint_editor && mkdir -p build && cd build && cmake .. && make && make install


RUN ldconfig
RUN echo "source /etc/bash_env" >> ~/.bashrc


# Clone the robot_constraint_manager package 
RUN cd ~/git && git clone --branch main https://github.com/Adorno-Lab/robot_constraint_manager 
RUN cd ~/git/robot_constraint_manager && mkdir -p build && cd build && cmake .. && make

 
