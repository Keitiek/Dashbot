## We are using ROS2 Humble distribution.

### Basic ROS commands:

- _colcon info_ - get basic workspace, package and dependency information.
- _colcon build_ - build the project (in the root)
- _source [file]_ - source the workspace, e.g.: _source ~/ros2_humble/install/setup.bash_
- _ros2 run [package] [node]_ - run the node, e.g.: _ros2 run teleop_twist_joy teleop_node_
- _rviz2_ - open RViz
- _ros2 run teleop_twist_keyboard teleop_twist_keyboard_ - running keyboard teleop
- _ros2 topic list_ - shows the list of topics running 

### Work History
To use keyboard teleop, we cloned teleop_twist_keyboard package: 


![ROS architecture draft](../assets/ros_architecture.png)
