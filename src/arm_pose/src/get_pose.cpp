#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "get_end_effector_pose");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP =
      "manipulator"; // 根据你的机械臂配置更改
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  while (ros::ok()) {
    geometry_msgs::PoseStamped end_effector_pose = move_group.getCurrentPose();
    ROS_INFO_STREAM("End effector pose: \n" << end_effector_pose);

    ros::Duration(1.0).sleep(); // 稍作延时以便观察输出
  }

  return 0;
}
