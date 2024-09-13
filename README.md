# UR5 Workspace

This repository contains ROS packages and configurations to simulate, visualize, and control the UR5 robotic arm in Gazebo and RViz. The package also includes a custom joint dynamics calculator for computing the robot's dynamics using Computed Torque Control (CTC) and tools for setting the UR5 to its zero position.

## Features

- **UR5 Simulation**: Simulates the UR5 robotic arm in Gazebo.
- **Visualization**: Visualizes the robot in RViz.
- **Trajectory Control**: Implements effort-based joint trajectory control using ROS's `effort_controllers/JointTrajectoryController`.
- **Dynamics Calculation**: Computes joint torques using CTC (Computed Torque Control).

## Prerequisites

Before using this package, make sure you have installed the following:

- [ROS Noetic](http://wiki.ros.org/noetic/Installation) or higher
- [Universal Robots ROS Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
- [Gazebo](http://gazebosim.org/)
- [Pinocchio](https://stack-of-tasks.github.io/pinocchio/) (for feedforward dynamics)
- `effort_controllers` package (for joint trajectory control)
- `ur_description` and `ur_gazebo` packages for the UR5 robot

## Installation

1. Clone this repository into your catkin workspace:
   ```bash
   cd ~
   git clone https://github.com/yourusername/ur5_ws.git
   ```

2. Build your catkin workspace:
   ```bash
   cd ~/ur5_ws
   catkin_make
   ```

3. Source the workspace:
   ```bash
   source devel/setup.bash
   ```

## Usage

### Launch UR5 with Gazebo and RViz

To bring up the UR5 simulation in Gazebo and RViz, run the following:

```bash
roslaunch ur_gazebo ur5_bringup_with_rviz.launch
```

This will start Gazebo with the UR5 robot loaded, as well as RViz for visualizing the robot's state and environment.

### Calculate Joint Dynamics

To calculate the dynamics of the UR5 robot using Computed Torque Control (CTC), use the following command(You need to enable the joint_group_eff_controller and stop the eff_joint_traj_controller):

```bash
rosrun ur5_control joint_dynamics_calculator
```

Make sure that the Pinocchio library is properly configured and that the UR5 model is loaded for dynamic calculations.

### Set UR5 to Zero Position

To set the UR5 robot to its zero position using `effort_controllers/JointTrajectoryController`, run(You need to enable the eff_joint_traj_controller and stop the joint_group_eff_controller):

```bash
rosrun ur5_control ur5_set_zero_position
```

This will use the `JointTrajectoryController` to move the robot to its zero position by applying effort-based control on each joint.

## Dependencies

The following ROS packages are required:

- `ur_description`
- `ur_gazebo`
- `effort_controllers`
- `controller_manager`
- `ros_control`
- `roscpp`
- `rospy`
- `std_msgs`
- `sensor_msgs`
- `trajectory_msgs`

## Contributing

Contributions are welcome! Please fork this repository and submit a pull request with any improvements or new features. 

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
