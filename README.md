![Static Badge](https://img.shields.io/badge/Written_in-C%2B%2B17-blue)![GitHub License](https://img.shields.io/github/license/juanjqo/capybara_toolkit?color=orange)![Static Badge](https://img.shields.io/badge/status-experimental-red)[![CPP Build MacOS](https://github.com/Adorno-Lab/robot_constraint_manager/actions/workflows/cpp_build_macos.yml/badge.svg)](https://github.com/Adorno-Lab/robot_constraint_manager/actions/workflows/cpp_build_macos.yml)![Static Badge](https://img.shields.io/badge/Platform-Ubuntu_x64-orange)![Static Badge](https://img.shields.io/badge/tested-green)

# robot_constraint_manager


This project implements the VFIs constraints using a yaml configuration file.
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

### Install (UNIX)

```shell
git clone https://github.com/Adorno-Lab/robot_constraint_manager
cd robot_constraint_manager
mkdir -p build && build
cmake ..
make
sudo make install
```
