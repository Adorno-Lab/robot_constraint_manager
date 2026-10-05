![Static Badge](https://img.shields.io/badge/Written_in-C%2B%2B17-blue)![GitHub License](https://img.shields.io/github/license/Adorno-Lab/robot_constraint_manager?color=orange)![Static Badge](https://img.shields.io/badge/status-experimental-red)[![CPP Build MacOS](https://github.com/Adorno-Lab/robot_constraint_manager/actions/workflows/cpp_build_macos.yml/badge.svg)](https://github.com/Adorno-Lab/robot_constraint_manager/actions/workflows/cpp_build_macos.yml)[![CPP Build Ubuntu](https://github.com/Adorno-Lab/robot_constraint_manager/actions/workflows/cpp_build.yml/badge.svg)](https://github.com/Adorno-Lab/robot_constraint_manager/actions/workflows/cpp_build.yml)[![Docs](https://img.shields.io/badge/docs-GitHub_Pages-green)](https://adorno-lab.github.io/robot_constraint_manager/)
# robot_constraint_manager

<img src="https://github.com/user-attachments/assets/810414b6-5dbc-4889-83c4-802c863807ea" alt="drawing" width="600"/>

This project implements the VFIs constraints using a YAML configuration file.
A version is described in this [publication](https://ieeexplore.ieee.org/document/10399868)

```bibtext
@Article{marinho2023multiarm,
  author       = {Marinho, M. M. and Quiroz-Omana, J. J. and Harada, K.},
  title        = {A Multi-Arm Robotic Platform for Scientific Exploration},
  journal      = {IEEE Robotics and Automation Magazine (RAM)}, 
  month        = dec,
  year         = {2024},
  pages        = {10--20},
  custom_type  = {1. Journal Paper},
  url          = {https://arxiv.org/abs/2210.11877},
  url_video    = {https://youtu.be/hnBuCpjLWzs},
  doi          = {10.1109/MRA.2023.3336472},
  volume       = {31},
  number       = {4}
}
```

The VFIs are described in this [publication](https://ieeexplore.ieee.org/document/8742769)

```bibtext
@Article{marinho2019dynamic,
  author       = {Marinho, Murilo M and Adorno, Bruno V and Harada, Kanako and Mitsuishi, Mamoru},
  title        = {Dynamic Active Constraints for Surgical Robots using Vector Field Inequalities},
  journal      = {IEEE Transactions on Robotics (T-RO)},
  year         = {2019},
  month        = oct,
  volume       = {35}, 
  number       = {5}, 
  pages        = {1166--1185},
  url          = {https://arxiv.org/pdf/1804.11270},
  url_video    = {https://youtu.be/tB6moMfeacs},
  doi          = {10.1109/TRO.2019.2920078},
  custom_type  = {1. Journal Paper},
}
```


## Configuration files

The VFIs are described in a YAML configuration file, which can be created and edited with the
[robot_constraint_editor](https://github.com/Adorno-Lab/robot_constraint_editor).

| Version | Geometric data | CoppeliaSim | Constructor |
|---|---|---|---|
| 3 (recommended) | Stored in the file (`environment_entities` and `robot_entities`) | Not required | `RobotConstraintManager(robot, config_file_reader, config_file)` |
| 2 | Obtained from a CoppeliaSim scene | Required | Deprecated |

See the [version 3 specification](https://github.com/Adorno-Lab/robot_constraint_editor/blob/main/design/specs_document/config_file_specification_v3.md).


# Install

> [!NOTE]
> Non-sudo privileges? Create a custom prefix folder (e.g. `~/opt`) to hold `lib/` and `include/` without needing root. See [this guide](https://ros2-tutorial.readthedocs.io/en/latest/cmake/cmake_packages_without_sudo.html) for background.

## Prerequisites

### DQ Robotics

Ubuntu:
```shell
sudo add-apt-repository ppa:dqrobotics-dev/development
sudo apt-get update && sudo apt-get install -y libdqrobotics libdqrobotics-interface-coppeliasim libdqrobotics-interface-coppeliasim-zmq libdqrobotics-interface-qpoases
```
macOS:

Clone and build the CMake projects manually.

### [yaml-cpp](https://github.com/jbeder/yaml-cpp)

Ubuntu:
```shell
cd ~/Downloads && git clone https://github.com/jbeder/yaml-cpp
cd ~/Downloads/yaml-cpp
```
```shell
mkdir -p build && cd build
cmake -DYAML_BUILD_SHARED_LIBS=on ..
make
sudo make install
```
macOS:
```shell
brew update
brew install yaml-cpp
```

### [robot_constraint_editor](https://github.com/Adorno-Lab/robot_constraint_editor)

```shell
cd ~/Downloads && git clone https://github.com/Adorno-Lab/robot_constraint_editor
cd ~/Downloads/robot_constraint_editor
```
```shell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
sudo cmake --install build
```

See its [install instructions](https://github.com/Adorno-Lab/robot_constraint_editor#install) for non-sudo users.

If you're installing any of the above without sudo, install them to the same custom prefix `~/opt`, and see the non-sudo instructions for `robot_constraint_manager` itself.

## Get the source code

```shell
git clone https://github.com/Adorno-Lab/robot_constraint_manager
cd robot_constraint_manager
```

## Sudo users

```shell

# 1. Configure: choose Release, and (optionally) where to install it.
#    Omit -DCMAKE_INSTALL_PREFIX to use the system default (/usr/local on Linux).
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local

# 2. Build the library and the unit tests.
cmake --build build -j$(nproc)

# 3. Install the headers and the library.
sudo cmake --install build
```

## Non-sudo users

```shell

# 1. Configure: choose Release, and install to your own prefix instead of a system path.
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=$HOME/opt

# 2. Build the library and the unit tests.
cmake --build build -j$(nproc)

# 3. Install the headers and the library. No sudo needed.
cmake --install build
```

> [!TIP]
> `robot_constraint_manager` finds `robot_constraint_editor` and `yaml-cpp` through the default search paths of the
> compiler and the linker. If they are installed in `$HOME/opt`, export `CPATH`, `LIBRARY_PATH`, and
> `LD_LIBRARY_PATH` in `~/.bashrc` (see [this guide](https://ros2-tutorial.readthedocs.io/en/latest/cmake/cmake_packages_without_sudo.html))
> before building it, or any project that uses it.

## Unit tests

The unit tests of version 3 files do not require CoppeliaSim:

```shell
./build/unit_tests_v3
```

The unit tests in `build/unit_tests` require CoppeliaSim with the scene of the [panda_example](https://github.com/Adorno-Lab/robot_constraint_manager/tree/main/examples/panda_example).


# Usage

```cmake
target_link_libraries(${YOUR_LIBRARY}
    robot_constraint_manager
    robot_constraint_editor
    dqrobotics
    dqrobotics-interface-coppeliasim-zmq)
```

```cpp
#include <dqrobotics_extensions/robot_constraint_manager/robot_constraint_manager.hpp>
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_yaml.hpp>
```

Build the constraints from a version 3 configuration file and the kinematic model of the robot:

```cpp
using namespace DQ_robotics_extensions;

auto robot = std::make_shared<DQ_SerialManipulatorMDH>(FrankaEmikaPandaRobot::kinematics());
RobotConstraintManager rcm{robot, std::make_shared<VFIConfigurationFileYaml>(), "vfi_constraints_v3.yaml"};

// The configuration file does not include the configuration limits or the configuration velocity limits.
rcm.set_configuration_limits({q_min, q_max});
rcm.set_configuration_velocity_limits({q_dot_min, q_dot_max});

auto [A, b] = rcm.get_inequality_constraints(q);    // A*q_dot <= b
```

Update the constraints at runtime:

```cpp
// Every VFI that uses the environment entity "obs_sphere" is updated, including the disabled ones.
// The attached direction is constant and expressed in the frame of the entity.
rcm.update_environment_entity_pose("obs_sphere", x_sphere);
rcm.update_environment_entity_derivative("obs_sphere", x_sphere_dot);

// Only the VFI with the tag "C5" is updated.
rcm.update_vfi_workspace_pose("C5", x_sphere);
rcm.update_vfi_workspace_derivative("C5", x_sphere_dot);

rcm.disable_vfi("C5");
rcm.enable_vfi("C5");
```

The configuration file loaded by the `RobotConstraintManager` is not modified by these methods, and `get_document()`
returns the configuration as it was loaded.

## Migrating from version 2

- The constructors that require CoppeliaSim are deprecated. Convert the configuration file to version 3
  (see Section 10.1 of the [version 3 specification](https://github.com/Adorno-Lab/robot_constraint_editor/blob/main/design/specs_document/config_file_specification_v3.md)),
  and use the constructor that requires only the kinematic model.
- `get_coppeliasim_entity_one_or_entity_environment_names()` and `get_coppeliasim_entity_two_or_entity_robot_names()`
  are deprecated. Use `get_entity_one_or_entity_environment_names()` and `get_entity_two_or_entity_robot_names()`,
  which support versions 2 and 3.
- For version 3 files, use `get_data_v3()` instead of `get_data()`.

# Examples

- [panda_example](https://github.com/Adorno-Lab/robot_constraint_manager/tree/main/examples/panda_example): version 3 configuration file. CoppeliaSim is used only to simulate the robot.
- [panda_example_old_format](https://github.com/Adorno-Lab/robot_constraint_manager/tree/main/examples/panda_example_old_format): legacy configuration file (deprecated).
