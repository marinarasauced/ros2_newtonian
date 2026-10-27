# newtonian

![Ubuntu](https://img.shields.io/badge/Ubuntu-26.04-E95420?logo=ubuntu&logoColor=white)
![ROS 2](https://img.shields.io/badge/ROS_2-Lyrical-22314E?logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.14-3776AB?logo=python&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-yellow)

ROS 2 package that runs [Newton](https://github.com/newton-physics/newton) physics simulations as ROS 2 nodes ; the scene (environments, inputs, robots, sensors, solvers) is described in one YAML parameter file, allowing the simulator to be built and rebuilt at runtime.

## License

This package is released under an [MIT license](https://github.com/marinarasauced/newtonian/blob/main/LICENSE).

Authors: [Marina Nelson](https://github.com/marinarasauced) <br/>
Maintainer: Marina Nelson, marinarasauced@outlook.com

Parts of this package (code review, documentation) were developed with the assistance of Claude Opus 5.5 and Claude Sonnet 5.5, AI models by Anthropic.
All AI contributions were reviewed, tested, and applied by the author.

## Contents

- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Docs](#docs)

## Requirements

| Component | Version tested |
|---|---|
| ROS 2 | Lyrical |
| Python | 3.14 |
| Newton | 1.7.0 |
| Warp | 1.18.0 |
| GPU (optional) | NVIDIA, CUDA 13 |

The Python dependencies are installed with pip into a virtual environment.
Alternatively, they may be installed directly to the site packages, but this is not recommended.
The ROS dependencies are installed with rosdep.

<!-- ## Architecture

```zsh
newtonian/

    node.py                  ROS 2 node

    modules/
        params.py            Parameters
        registry.py          Registry
        arrays.py
        transforms.py
        paths.py

        physics/
            simulator.py     NewtonianSimulator
            viewer.py        NewtonianViewer

        plugins/
            base.py          Plugin
            environments/    groundplane, track
            robots/          mjcf, urdf, usd
            sensors/         imu, lidar, camera, odometry, jointstate, contact
            inputs/          racecar, jointcommand, markers
            solvers/         mujoco, featherstone, kamino, xpbd

config/                      parameter files
descriptions/                robot files
launch/                      sim.launch.py
``` -->

## Installation

Clone the package into your workspace.

```zsh
cd ~/ros2_ws/src
git clone https://github.com/marinarasauced/ros2_newton newtonian
```

Install all Python dependencies into a virtual environment.

```zsh
python3 -m venv ~/.virtualenvs/newtonian
source ~/.virtualenvs/newtonian/bin/activate
pip install -r ~/ros2_ws/src/newtonian/newtonian/requirements.txt
```

Install all ROS dependencies.

```zsh
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

Build the package.

```zsh
python3 -m colcon build --symlink-install --packages-select newtonian
source install/setup.zsh
```

With `--symlink-install`, changes to the existing Python code and to the `config/*.yaml` files take effect without rebuilding the package.

## Usage

Start the example scene of a racecar on a pear-shaped track.

```zsh
ros2 launch newtonian sim.launch.py
```

Start your own scene.

```zsh
ros2 launch newtonian sim.launch.py config:=/path/to/your/scene.yaml
```

## Docs

AI generated documentation from Claude Opus 5.5.

| Page | Contents |
|---|---|
| [Configuration](docs/configuration.md) | all YAML keys, with defaults and units |
| [Development](docs/development.md) | code layout, how to add a plugin |
| [Runtime](docs/runtime.md) | topics, services, viewer controls, rebuild |
| [Troubleshooting](docs/troubleshooting.md) | errors and their fixes |