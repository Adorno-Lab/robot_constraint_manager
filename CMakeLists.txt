cmake_minimum_required(VERSION 3.5...3.26)

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

project(robot_constraint_manager LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



find_package(Eigen3 REQUIRED)
if (APPLE)
    INCLUDE_DIRECTORIES(
           /usr/local/include/
           /usr/local/include/eigen3
           # Most recent versions of brew install here
           /opt/homebrew/include/
           /opt/homebrew/include/eigen3
       )
   ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
   # The library is installed here when using the regular cmake ., make, sudo make install
   LINK_DIRECTORIES(
       /usr/local/lib/
       /opt/homebrew/lib
       )
endif()

#
#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
#add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})
add_custom_target(cfg ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})
#

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
target_link_libraries(test PRIVATE
    dqrobotics
    dqrobotics-interface-coppeliasim-zmq
    qpOASES
    yaml-cpp
    ${PROJECT_NAME})

set(ENV{DYLD_LIBRARY_PATH} /usr/local/lib/)

if (APPLE)
    set(qpOASES_LIBRARY32 /usr/local/lib/libqpOASES.3.2.dylib)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${qpOASES_LIBRARY32}
            $<TARGET_FILE_DIR:${PROJECT_NAME}>)
endif()
