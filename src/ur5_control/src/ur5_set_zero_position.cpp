#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "ur5_zero_position");
  ros::NodeHandle nh;

  // 发布命令到关节轨迹控制器的发布器
  ros::Publisher joint_traj_pub =
      nh.advertise<trajectory_msgs::JointTrajectory>(
          "/eff_joint_traj_controller/command", 10);

  // 确保发布器有足够的时间连接
  ros::Duration(1.0).sleep();

  // 初始化关节轨迹消息
  trajectory_msgs::JointTrajectory traj;
  trajectory_msgs::JointTrajectoryPoint point;

  // 设置关节名称
  traj.joint_names.push_back("shoulder_pan_joint");
  traj.joint_names.push_back("shoulder_lift_joint");
  traj.joint_names.push_back("elbow_joint");
  traj.joint_names.push_back("wrist_1_joint");
  traj.joint_names.push_back("wrist_2_joint");
  traj.joint_names.push_back("wrist_3_joint");

  // 将所有关节目标位置设置为零
  point.positions = {0, 0, 0, 0, 0, 0};
  point.time_from_start = ros::Duration(3.0);  // 该点应达到的持续时间

  // 将此点添加到轨迹中
  traj.points.push_back(point);

  // 发布轨迹
  joint_traj_pub.publish(traj);

  ros::spinOnce();
  ros::Duration(1.0).sleep();  // 等待一段时间以确保消息被发送

  return 0;
}
