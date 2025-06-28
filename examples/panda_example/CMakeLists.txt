cmake_minimum_required(VERSION 3.5...3.26)



project(panda_example LANGUAGES CXX)
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


add_executable(${PROJECT_NAME}
               ${PROJECT_NAME}.cpp)

target_link_libraries(${PROJECT_NAME}
    dqrobotics
    dqrobotics-interface-coppeliasim-zmq
    robot_constraint_manager
    Eigen3::Eigen
    qpOASES)


# COPY the Yaml config file to the build folder
set(YAML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cfg/vfi_constraints.yaml)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${YAML_FILE}
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)


if (APPLE)
    # For some reason, on my local tests running on my macOS, 
    # qpOASES is not found in /usr/local/lib/. Because of that, I am
    # copying the library from /usr/local/lib/ to the target directory
    set(qpOASES_LIBRARY32 /usr/local/lib/libqpOASES.3.2.dylib)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${qpOASES_LIBRARY32}
            $<TARGET_FILE_DIR:${PROJECT_NAME}>)
endif()
