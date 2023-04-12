# Long-Term Dynamic Window Approach for Kinodynamic Local Planning in Static and Crowd Environments

This repo is the code for the paper "Long-Term Dynamic Window Approach for Kinodynamic Local Planning in Static and Crowd Environments".

# Requirements

For running code:

- Ubuntu20.04

- python3.9

- ros-noetic (http://wiki.ros.org/ROS/Installation)

- ros-navigation (sudo apt install ros-noetic-navigation)

- ros-movebase (sudo apt install ros-noetic-move-base)

- pcl-ros (sudo apt install ros-noetic-pcl-ros)

- opencv4.2 (sudo apt install libopencv-dev)

- g2o (sudo apt install ros-noetic-libg2o)

- suitesparse (sudo apt install libsuitesparse-dev)

- yaml-cpp (sudo apt install libyaml-cpp-dev)

# Install

1. create a root directory

```
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
catkin_init_workspace
```

2. clone the code

```
git clone https://github.com/flztiii/LT_DWA.git
```

3. install

```
catkin_make -DCMAKE_BUILD_TYPE=Release
```

# How to use

## ORCA crowd env test

1. launch the program

```
cd catkin_ws
source devel/setup.bash
roslaunch local_planner start_orca.launch
```

2. press "Enter" key in the terminal to start the program

## STU dataset env test

1. change the parameters

```
cd catkin_ws
gedit src/local_planner/config/planning.config
```

change the parameter "simulator" from "orca" to "crowd".

2. launch the program

```
cd catkin_ws
source devel/setup.bash
roslaunch local_planner start_crowd.launch
```

3. press "Enter" key in the terminal to start the program

## Static env test

1. change the parameters

```
cd catkin_ws
gedit src/local_planner/config/planning.config
```

change the parameter "simulator" from "orca" to "static", and change the parameter "nav_method" from "none" to "astar".

2. launch the program

```
cd catkin_ws
source devel/setup.bash
roslaunch local_planner start_static.launch
```

3. press "Enter" key in the terminal to start the program

# License

LT-DWA code is distributed under MIT License.

# Citing

If you use the code in your research, please use the following BibTeX entry.

```BibTeX
@ARTICLE{jian2023long,
  author={Jian, Zhiqiang and Zhang, Songyi and Sun, Lingfeng and Zhan, Wei and Zheng, Nanning and Tomizuka, Masayoshi},
  journal={IEEE Robotics and Automation Letters}, 
  title={Long-Term Dynamic Window Approach for Kinodynamic Local Planning in Static and Crowd Environments}, 
  year={2023},
  volume={},
  number={},
  pages={1-8},
  doi={10.1109/LRA.2023.3266664}
}
```