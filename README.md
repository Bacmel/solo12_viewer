# Solo12 Viewer

> *ROS2 package made for visualizing [odri](https://github.com/open-dynamic-robot-initiative) & [pinocchio](https://stack-of-tasks.github.io/pinocchio/) datas using ROS2 tools.*

> Tested with ROS2 Galactic :stars:

## Installation

### Python3 dependencies

**Solo12 Viewer** uses **ODRI Control Interface** and **Pinocchio** libraries.
The installation's instructions can be find [here (ODRI)](https://github.com/open-dynamic-robot-initiative/odri_control_interface) and [here (Pinocchio)](https://github.com/stack-of-tasks/pinocchio).

**Solo12 Viewer** needs also to have access to the description file of solo12, available [here (`example_robot_data`)](https://github.com/Gepetto/example-robot-data).

### Packages dependencies

Inside the *src* directory of your *ros2\_ws*, you need the following packages:

``` shell
# ODRI ROS2 MSG
path/to/ros2_ws/src$ git clone https://github.com/hidro-iri/odri_ros2.git
# VRPN CLIENT ROS
path/to/ros2_ws/src$ git clone https://github.com/hidro-iri/vrpn_client_ros.git
# Solo12 Viewer
path/to/ros2_ws/src$ git clone https://github.com/Bacmel/solo12_viewer.git
```

:warning: A package named `example_robot_data` with the *robots* directory inside must also be added to your workspace for **Rviz2**.
> ⚠️ In construction 👷

## Usage

After source ROS2 and your workspace :

``` shell
# Launch
ros2 launch solo12_viewer solo12_viewer.launch.py <debug_mode:="true"/"false">
# debug_mode = true stop RVIZ
```

### Build Command

> ⚠️ In construction 👷
