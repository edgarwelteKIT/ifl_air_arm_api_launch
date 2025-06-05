# ifl_air_arm_api_launch

This ROS2 package contains a example launch configuration and description to run arm_api with and without cumotion support.
See installation instructions for cumotion below.


## Usage

### Without cumotion

Start driver for robot and gripper:

```bash
ros2 launch ifl_air_arm_api_launch robot_ur_robotiq.launch.py
```


Start MoveIt! and arm_api2:

```bash
ros2 launch ifl_air_arm_api_launch moveit_ur_robotiq.launch.py
```


### With cumotion

1. Start driver for robot and gripper:

```bash
ros2 launch ifl_air_arm_api_launch robot_ur_robotiq.launch.py
```

2. Start MoveIt!, arm_api2 and cumotion:

```bash
ros2 launch ifl_air_arm_api_launch launch/moveit_ur_robotiq_cumotion.launch.py
```




# cuMotion 

# About cuMotion

CuRobo is a CUDA accelerated library containing a suite of robotics algorithms that run significantly faster than existing implementations leveraging parallel compute. cuRobo currently provides the following algorithms: 

(1) forward and inverse kinematics 

(2) collision checking between robot and world, with the world represented as Cuboids, Meshes, and Depth images 

(3) numerical optimization with gradient descent, L-BFGS, and MPPI

(4) geometric planning 

(5) trajectory optimization 

(6) motion generation that combines inverse kinematics, geometric planning, and trajectory optimization to generate global motions within 30ms.

CuMotion is a collision-free planner using the cuRobo’s core and backend, and it can be used as a plugin for Moveit2.

## Requirements and Installation

Required python packages: `torch`, `warp-lang`, `yourdfpy`, `trimesh`

```bash 
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
pip3 install warp-lang yourdfpy trimesh
```

Required ros packages: `ros-humble-isaac-ros-cumotion`, `ros-humble-isaac-ros-cumotion-moveit`, which we can't directly install from ros source deb, we need to add the source key to apt following [instruction](https://nvidia-isaac-ros.github.io/getting_started/isaac_apt_repository.html)

```bash
wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add -
grep -qxF "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" /etc/apt/sources.list || \
echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" | sudo tee -a /etc/apt/sources.list
sudo apt-get update
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Then install the ros packages using:

```bash
sudo apt install ros-humble-isaac-ros-cumotion ros-humble-isaac-ros-cumotion-moveit
```


## Issues

### Missing some dependencies

If you meet such issue

```bash
[move_group-1] [ERROR] [1736787468.149232694] [moveit.ros_planning.planning_pipeline]: Exception while loading planner 'isaac_ros_cumotion_moveit/CumotionPlanner': Failed to load library /opt/ros/humble/lib/libisaac_ros_cumotion_moveit.so. Make sure that you are calling the PLUGINLIB_EXPORT_CLASS macro in the library code, and that names are consistent between this macro and your XML. Error string: Could not load library dlopen error: **libmoveit_planning_interface.so.2.5.5**: cannot open shared object file: No such file or directory, at ./src/shared_library.c:99Available plugins: chomp_interface/CHOMPPlanner, isaac_ros_cumotion_moveit/CumotionPlanner, ompl_interface/OMPLPlanner, pilz_industrial_motion_planner/CommandPlanner
```

Then it indicates that the library dependency **libmoveit_planning_interface.so.2.5.5** is requiered by the plugin. You can use command ```find /opt/ros/humble/ -name libmoveit_planning_interface.so*``` and command ```ldd /opt/ros/humble/lib/libisaac_ros_cumotion_moveit.so``` to check if any are missing.

Once you found there are files with slightly different version, you can create a symlink to solve the Error (tested in following case)

```bash
sudo ln -s /opt/ros/humble/lib/libmoveit_planning_interface.so.2.5.6 /opt/ros/humble/lib/libmoveit_planning_interface.so.2.5.5
sudo ln -s /opt/ros/humble/lib/libmoveit_planning_scene.so.2.5.6 /opt/ros/humble/lib/libmoveit_planning_scene.so.2.5.5
sudo ln -s /opt/ros/humble/lib/libmoveit_robot_trajectory.so.2.5.6 /opt/ros/humble/lib/libmoveit_robot_trajectory.so.2.5.5
sudo ln -s /opt/ros/humble/lib/libmoveit_robot_state.so.2.5.6 /opt/ros/humble/lib/libmoveit_robot_state.so.2.5.5
```

### isaac_ros_cumotion doesn't detect scene objects or the planner plans to a strange pose

**The reason is:** in the official cumotion tutorial, robot 'base_link' frame and 'world' frame is the same. If this is not case (e.g. robot is on table, world is on ground level) the cumotion planner doesn't know the actual robot position in the environment, it will plan as default.

**Solution is:** Use and create our own urdf file and xrdf file. And in xrdf file, we should set the base frame as 'world' frame, so it will recognize the objects and poses based on 'world' frame instead of 'base_link' frame.
