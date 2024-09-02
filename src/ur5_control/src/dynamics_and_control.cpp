#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

vector<double> joint_positions;
vector<double> joint_velocities;
vector<double> joint_accelerations;
Eigen::VectorXd joint_efforts;
bool received_state = false;

ros::Publisher joint_trajectory_pub;
ros::Publisher joint_effort_pub;

pinocchio::Model model;
pinocchio::Data data;

void publishJointEfforts(const vector<double> &efforts,
                         const vector<string> &joint_names) {
  if (efforts.size() != joint_names.size()) {
    ROS_ERROR("Size mismatch between efforts and joint names.");
    return;
  }
  trajectory_msgs::JointTrajectory traj_msg;
  trajectory_msgs::JointTrajectoryPoint traj_point;

  traj_msg.header.stamp = ros::Time::now();
  traj_msg.joint_names = joint_names;

  traj_point.effort = efforts;
  traj_point.positions.resize(joint_names.size(), 0.0);
  traj_point.velocities.resize(joint_names.size(), 0.0);
  traj_point.accelerations.resize(joint_names.size(), 0.0);
  traj_point.time_from_start =
      ros::Duration(1.0);  // 根据需要设置适当的持续时间
  traj_msg.points.push_back(traj_point);

  joint_trajectory_pub.publish(traj_msg);
}

void publishJointEfforts(const Eigen::VectorXd &efforts) {
  std_msgs::Float64MultiArray effort_msg;

  // Fill the effort message
  for (int i = 0; i < efforts.size(); ++i) {
    effort_msg.data.push_back(efforts[i]);
  }

  // Publish the effort message
  joint_effort_pub.publish(effort_msg);
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
  joint_positions.resize(msg->position.size());
  for (size_t i = 0; i < msg->position.size(); ++i) {
    joint_positions[i] = msg->position[i];
  }
  joint_velocities.resize(msg->velocity.size());
  for (size_t i = 0; i < msg->velocity.size(); ++i) {
    joint_velocities[i] = msg->velocity[i];
  }
  joint_accelerations.resize(msg->effort.size());
  for (size_t i = 0; i < msg->effort.size(); ++i) {
    joint_accelerations[i] = msg->effort[i];
  }
  received_state = true;
}

// Eigen::VectorXd computeCoefficients(double q0, double qf) {
//   Eigen::MatrixXd M(6, 6);
//   Eigen::VectorXd b(6);
//   Eigen::VectorXd a(6);

//   M << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 1, 1, 1, 1, 1,
//   1,
//       0, 1, 2, 3, 4, 5, 0, 0, 2, 6, 12, 20;
//   b << q0, 0, 0, qf, 0, 0;

//   a = M.colPivHouseholderQr().solve(b);
//   return a;
// }

void initializeModel(const string &urdf_path) {
  pinocchio::urdf::buildModel(urdf_path, model);
  data = pinocchio::Data(model);
  cout << "Model initialized" << endl;
}

void printModelDetails(const pinocchio::Model &model) {
  cout << "Model has " << model.njoints << " joints." << endl;
  for (size_t i = 0; i < model.njoints; ++i) {
    cout << "Joint " << i << ": " << model.names[i] << endl;
  }
  cout << "Model has " << model.nframes << " frames." << endl;
}

void computeDynamics(const pinocchio::Model &model, pinocchio::Data &data,
                     const Eigen::VectorXd &q, const Eigen::VectorXd &v,
                     const Eigen::VectorXd &a) {
  Eigen::VectorXd tau = pinocchio::rnea(model, data, q, v, a);

  joint_efforts.resize(tau.size());
  for (size_t i = 0; i < tau.size(); ++i) {
    joint_efforts[i] = tau[i] / 2;
  }

  pinocchio::crba(model, data, q);
  Eigen::MatrixXd &M = data.M;

  Eigen::VectorXd g = pinocchio::computeGeneralizedGravity(model, data, q);

  cout << "Torques: " << tau.transpose() << endl;
  //   cout << "Mass matrix: \n" << M << endl;
  //   cout << "Gravity vector: " << g.transpose() << endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "joint_state_listener");
  ros::NodeHandle nh;

  string urdf_path =
      "/home/prodefi/ur5_ws/src/universal_robot/ur_description/urdf/ur5.urdf";
  initializeModel(urdf_path);
  printModelDetails(model);

  joint_trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
      "/eff_joint_traj_controller/command", 10);
  joint_effort_pub = nh.advertise<std_msgs::Float64MultiArray>(
      "/joint_group_eff_controller/command", 10);
  ros::Subscriber sub = nh.subscribe("/joint_states", 10, jointStateCallback);
  ros::Rate rate(100);  // 10 Hz

  while (ros::ok() && !received_state) {
    ros::spinOnce();
    rate.sleep();
    cout << "Waiting for joint states..." << endl;
  }

  // // Compute coefficients after receiving state
  // vector<Eigen::VectorXd> coefficients;
  // Eigen::VectorXd desired_positions =
  //     Eigen::VectorXd::Zero(joint_positions.size());
  // Eigen::VectorXd desired_velocities =
  //     Eigen::VectorXd::Zero(joint_positions.size());

  // for (size_t i = 0; i < joint_positions.size(); ++i) {
  //   coefficients.push_back(
  //       computeCoefficients(joint_positions[i], desired_positions[i]));
  // }

  // // Interpolation loop
  // double t = 0.0;
  // double dt = 0.1;
  // while (t <= 1.0) {
  //   for (size_t i = 0; i < joint_positions.size(); ++i) {
  //     double position = 0.0;
  //     for (size_t j = 0; j < 6; ++j) {
  //       position += coefficients[i][j] * pow(t, j);
  //     }
  //     ROS_INFO("Joint %zu: Position = %f", i, position);
  //   }
  //   t += dt;
  // }

  // Eigen::VectorXd Kp(6);
  // Eigen::VectorXd Kd(6);
  // Kp << 0.1, 200, 10, 0.1, 0.1, 0.1;
  // Kd << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
  // while (ros::ok()) {
  //   // 使用Pinocchio计算动力学参数
  //   Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  //   Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  //   Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
  //   // 对q, v, a进行赋值
  //   q << joint_positions[0], joint_positions[1], joint_positions[2],
  //       joint_positions[3], joint_positions[4], joint_positions[5];
  //   v << joint_velocities[0], joint_velocities[1], joint_velocities[2],
  //       joint_velocities[3], joint_velocities[4], joint_velocities[5];
  //   // a << joint_accelerations[0], joint_accelerations[1],
  //   // joint_accelerations[2],
  //   //     joint_accelerations[3], joint_accelerations[4],
  //   //     joint_accelerations[5];

  //   computeDynamics(model, data, q, v, a);

  //   // 计算反馈力矩
  //   // 计算PD反馈力矩
  //   Eigen::VectorXd position_error = desired_positions - q;
  //   Eigen::VectorXd velocity_error = desired_velocities - v;
  //   // Eigen::VectorXd position_error = q - desired_positions;
  //   // Eigen::VectorXd velocity_error = v - desired_velocities;
  //   Eigen::VectorXd tau_fb =
  //       Kp.cwiseProduct(position_error) + Kd.cwiseProduct(velocity_error);
  //   // Publish joint efforts
  //   vector<string> joint_names;
  //   // Assuming 'model.names' is a vector<string> and contains at
  //   // least one element
  //   if (!model.names.empty()) {
  //     joint_names.assign(model.names.begin() + 1, model.names.end());
  //   }

  //   // publishJointEfforts(joint_efforts, joint_names);
  //   Eigen::VectorXd tau = tau_fb + joint_efforts;
  //   publishJointEfforts(tau);
  //   ros::spinOnce();
  //   rate.sleep();
  // }
  return 0;
}
