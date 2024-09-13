#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>

int main(int argc, char **argv) {
  // 初始化ROS节点
  ros::init(argc, argv, "ur5_effort_test_node");
  ros::NodeHandle nh;

  // 定义发布器，用于向 joint_group_eff_controller 发送扭矩命令
  ros::Publisher effort_pub = nh.advertise<std_msgs::Float64MultiArray>(
      "/joint_group_eff_controller/command", 10);

  // 设置循环频率
  ros::Rate loop_rate(125);  // 125 Hz

  // 定义用于存储各关节扭矩命令的数组
  std_msgs::Float64MultiArray effort_command;

  // UR5有6个关节，初始化扭矩命令为零
  effort_command.data.resize(6, 0.0);

  // 初始化一些控制参数，例如简单的正弦函数用于扭矩变化（可以根据需求调整）
  double torque_amplitude = 5.0;  // 最大扭矩幅值
  double frequency = 0.1;         // 频率（控制扭矩变化的速度）

  while (ros::ok()) {
    // 通过一个简单的公式（如正弦函数）生成扭矩命令
    for (size_t i = 0; i < 2; ++i) {
      effort_command.data[i] =
          torque_amplitude * sin(ros::Time::now().toSec() * frequency + i);
    }

    // 向 joint_group_eff_controller 发布扭矩命令
    effort_pub.publish(effort_command);

    // 打印当前的扭矩命令，便于调试
    ROS_INFO_STREAM("Sending torque command: ["
                    << effort_command.data[0] << ", " << effort_command.data[1]
                    << ", " << effort_command.data[2] << ", "
                    << effort_command.data[3] << ", " << effort_command.data[4]
                    << ", " << effort_command.data[5] << "]");

    // 确保循环按照设定的频率运行
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
