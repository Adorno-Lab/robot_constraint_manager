cmake_minimum_required(VERSION 3.5...3.26)

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

project(robot_constraint_manager LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
if (APPLE)
    find_package(cppzmq REQUIRED)
    INCLUDE_DIRECTORIES(/usr/local/include/)
    LINK_DIRECTORIES(/usr/local/lib/)
endif()

include_directories(${PROJECT_NAME}  include)

add_library(${PROJECT_NAME} SHARED
    src/dqrobotics_extensions/robot_constraint_manager/robot_constraint_manager.cpp
    src/dqrobotics_extensions/robot_constraint_manager/checkers.cpp
    src/dqrobotics_extensions/robot_constraint_manager/conversions.cpp
    src/dqrobotics_extensions/robot_constraint_manager/numpy.cpp
    src/dqrobotics_extensions/robot_constraint_manager/vfi_framework.cpp
    src/dqrobotics_extensions/robot_constraint_manager/vfi_manager.cpp
    src/dqrobotics_extensions/robot_constraint_manager/constraints_manager.cpp
    src/dqrobotics_extensions/robot_constraint_manager/utils.cpp
)

target_link_libraries(${PROJECT_NAME}
        dqrobotics
        dqrobotics-interface-coppeliasim-zmq
        yaml-cpp
        #}
)

add_executable(test src/test.cpp)
target_link_libraries(test PRIVATE ${PROJECT_NAME})
