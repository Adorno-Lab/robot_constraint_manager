cmake_minimum_required(VERSION 3.5...3.26)

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

project(robot_constraint_manager LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  #URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

find_package(Eigen3 REQUIRED)

if(UNIX AND NOT APPLE)
    FIND_PACKAGE(Eigen3 REQUIRED)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
    ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
endif()

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

target_include_directories(${PROJECT_NAME} PRIVATE
        include
)

target_link_libraries(${PROJECT_NAME}
        dqrobotics
        dqrobotics-interface-coppeliasim-zmq
        Eigen3::Eigen
        yaml-cpp
        robot_constraint_editor
)

SET_TARGET_PROPERTIES(${PROJECT_NAME}
    PROPERTIES PUBLIC_HEADER
    "include/dqrobotics_extensions/robot_constraint_manager/robot_constraint_manager.hpp"
    )

INSTALL(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION "lib"
    PUBLIC_HEADER DESTINATION "include/dqrobotics_extensions/robot_constraint_manager/"
    PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)


# Other Headers
INSTALL(FILES
    include/dqrobotics_extensions/robot_constraint_manager/checkers.hpp
    include/dqrobotics_extensions/robot_constraint_manager/constraints_manager.hpp
    include/dqrobotics_extensions/robot_constraint_manager/conversions.hpp
    include/dqrobotics_extensions/robot_constraint_manager/numpy.hpp
    include/dqrobotics_extensions/robot_constraint_manager/utils.hpp
    include/dqrobotics_extensions/robot_constraint_manager/vfi_framework.hpp
    include/dqrobotics_extensions/robot_constraint_manager/vfi_manager.hpp

    DESTINATION "include/dqrobotics_extensions/robot_constraint_manager")

# For unit tests

add_executable(unit_tests
    tests/unit_tests.cpp)

target_link_libraries(unit_tests
    dqrobotics
    dqrobotics-interface-coppeliasim-zmq
    gtest_main
    ${PROJECT_NAME}
     robot_constraint_editor
)

# COPY the Yaml config file to the build folder
set(YAML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/examples/panda_example/cfg/vfi_constraints_3.yaml)


add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${YAML_FILE}
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)
