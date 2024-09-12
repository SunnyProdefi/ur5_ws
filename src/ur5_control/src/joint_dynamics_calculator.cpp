#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Dense>

using namespace std;

pinocchio::Model model;
pinocchio::Data data(model);
vector<vector<double>> tau;

vector<double> joint_positions;
vector<double> joint_velocities;
vector<double> joint_tau;

ros::Publisher joint_effort_pub;
bool received_state = false;

// 从 joint_states 到 Pinocchio 的映射
vector<int> ros_to_pinocchio_map = {2, 1, 0, 3, 4, 5};
// 从 Pinocchio 到 joint_states 的映射
vector<int> pinocchio_to_ros_map = {2, 1, 0, 3, 4, 5};

void initializeModel(const string &urdf_path) {
  pinocchio::urdf::buildModel(urdf_path, model);
  data = pinocchio::Data(model);
  cout << "Model initialized" << endl;
}

void printModelDataDetails(const pinocchio::Model &model,
                           const pinocchio::Data &data) {
  cout << "Model has " << model.njoints << " joints." << endl;
  for (size_t i = 0; i < model.njoints; ++i) {
    cout << "Joint " << i << ": " << model.names[i] << endl;
  }

  for (size_t i = 1; i < model.njoints; ++i) {
    cout << "Joint " << i << " (" << model.names[i] << ") position: ";
    cout << data.oMi[i].translation().transpose() << endl;
  }
}

void computeDynamics(const pinocchio::Model &model, pinocchio::Data &data,
                     const Eigen::VectorXd &q, const Eigen::VectorXd &v,
                     const Eigen::VectorXd &a) {
  Eigen::VectorXd calcTau = pinocchio::rnea(model, data, q, v, a);
  cout << "Torques: " << calcTau.transpose() << endl;
  vector<double> tau_i =
      vector<double>(calcTau.data(), calcTau.data() + calcTau.size());
  tau.push_back(tau_i);
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
  joint_positions.resize(msg->position.size());
  joint_velocities.resize(msg->velocity.size());
  joint_tau.resize(msg->effort.size());

  // 重排关节位置、速度和加速度
  // shoulder_pan_joint shoulder_lift_joint elbow_joint
  // wrist_1_joint wrist_2_joint wrist_3_joint
  for (size_t i = 0; i < 6; ++i) {
    joint_positions[ros_to_pinocchio_map[i]] = msg->position[i];
    joint_velocities[ros_to_pinocchio_map[i]] = msg->velocity[i];
    joint_tau[ros_to_pinocchio_map[i]] = msg->effort[i];
  }

  received_state = true;
}

void printVector(const string &name, const vector<double> &vec) {
  cout << name << ": ";
  for (size_t i = 0; i < vec.size(); ++i) {
    cout << vec[i] << " ";
  }
  cout << endl;
}

void printJointData(const vector<vector<double>> &q_vecs,
                    const vector<vector<double>> &v_vecs,
                    const vector<double> &joint_positions,
                    const vector<double> &joint_velocities, int i) {
  cout << "q_des: ";
  for (int j = 0; j < 6; ++j) {
    cout << q_vecs[j][i] << " ";
  }
  cout << endl;

  printVector("joint_positions", joint_positions);

  cout << "v_des: ";
  for (int j = 0; j < 6; ++j) {
    cout << v_vecs[j][i] << " ";
  }
  cout << endl;

  printVector("joint_velocities", joint_velocities);
}

// 封装读取 CSV 文件的函数
void readTrajectoryFromFile(const string &filename, vector<double> &q_vec,
                            vector<double> &v_vec, vector<double> &a_vec,
                            int num) {
  ifstream file(filename);
  string line, cell;

  if (file.is_open()) {
    getline(file, line);  // 读取并跳过标题行
    int i = 0;
    while (getline(file, line) && i < num) {
      stringstream lineStream(line);
      getline(lineStream, cell, ',');  // 跳过时间列
      getline(lineStream, cell, ',');
      q_vec[i] = stod(cell);
      getline(lineStream, cell, ',');
      v_vec[i] = stod(cell);
      getline(lineStream, cell, ',');
      a_vec[i] = stod(cell);
      i++;
    }
    file.close();
  } else {
    cerr << "Failed to open file: " << filename << endl;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "joint_dynamics_calculator");
  ros::NodeHandle nh;
  joint_effort_pub = nh.advertise<std_msgs::Float64MultiArray>(
      "/joint_group_eff_controller/command", 10);

  ros::Subscriber sub = nh.subscribe("/joint_states", 1, jointStateCallback);
  string urdf_path =
      "/home/prodefi/ur5_ws/src/universal_robot/ur_description/urdf/ur5.urdf";

  initializeModel(urdf_path);
  Eigen::VectorXd q_des = Eigen::VectorXd::Zero(model.nq);
  Eigen::VectorXd v_des = Eigen::VectorXd::Zero(model.nq);
  Eigen::VectorXd a_des = Eigen::VectorXd::Zero(model.nq);
  pinocchio::forwardKinematics(model, data, q_des);
  printModelDataDetails(model, data);

  vector<string> files = {
      "/home/prodefi/ur5_ws/shoulder_pan_joint_trajectory.csv",
      "/home/prodefi/ur5_ws/shoulder_lift_joint_trajectory.csv",
      "/home/prodefi/ur5_ws/elbow_joint_trajectory.csv",
      "/home/prodefi/ur5_ws/wrist_1_joint_trajectory.csv",
      "/home/prodefi/ur5_ws/wrist_2_joint_trajectory.csv",
      "/home/prodefi/ur5_ws/wrist_3_joint_trajectory.csv"};

  int hz = 50;
  int time = 3;
  int num = hz * time;
  vector<vector<double>> q_vecs(6, vector<double>(num)),
      v_vecs(6, vector<double>(num)), a_vecs(6, vector<double>(num));

  // 调用封装的函数读取每个文件
  for (int f = 0; f < files.size(); ++f) {
    readTrajectoryFromFile(files[f], q_vecs[f], v_vecs[f], a_vecs[f], num);
  }

  // 打印q_des,v_des,a_des
  cout << "q_des: " << q_des.transpose() << endl;
  cout << "v_des: " << v_des.transpose() << endl;
  cout << "a_des: " << a_des.transpose() << endl;

  for (int i = 0; i < num; ++i) {
    for (int j = 0; j < 6; ++j) {
      q_des[j] = q_vecs[j][i];
      v_des[j] = v_vecs[j][i];
      a_des[j] = a_vecs[j][i];
    }
    cout << "---------------------------" << endl;
    cout << "---------------------------" << endl;
    cout << "q_des: " << q_des.transpose() << endl;
    cout << "v_des: " << v_des.transpose() << endl;
    cout << "a_des: " << a_des.transpose() << endl;
    pinocchio::forwardKinematics(model, data, q_des, v_des, a_des);
    computeDynamics(model, data, q_des, v_des, a_des);
  }
  ros::Rate rate(50);
  while (ros::ok() && !received_state) {
    ros::spinOnce();
    rate.sleep();
    cout << "Waiting for joint states..." << endl;
  }

  int i = 0;
  vector<double> Kp(6);
  vector<double> Kd(6);
  // shoulder_pan_joint
  // elbow_joint
  // shoulder_lift_joint
  // wrist_1_joint
  // wrist_2_joint
  // wrist_3_joint
  nh.getParam("/Kp", Kp);
  nh.getParam("/Kd", Kd);
  vector<double> position_error(6), velocity_error(6);

  // 插值到tau_ff的起始点,避免第一次下发的力矩突变,导致机械臂抖动,插值时间为0.5s
  vector<double> tau_ff_start = joint_tau;
  vector<double> tau_ff_end = tau[0];
  vector<double> tau_ff(6);
  int k = 0;
  while (ros::ok() && k < 25) {
    for (int j = 0; j < 6; ++j) {
      tau_ff[j] = tau_ff_start[j] + (tau_ff_end[j] - tau_ff_start[j]) * k / 25;
    }
    k++;
    std_msgs::Float64MultiArray effort_msg;
    cout << "tau_ff: ";
    effort_msg.data.clear();  // 确保每次下发时都是清空状态
    for (int j = 0; j < tau_ff.size(); ++j) {
      effort_msg.data.push_back(tau_ff[j]);
      cout << effort_msg.data[j] << " ";
    }
    cout << endl;
    joint_effort_pub.publish(effort_msg);
    ros::spinOnce();
    rate.sleep();
  }

  while (ros::ok()) {
    std_msgs::Float64MultiArray effort_msg;
    cout << "----------------------i----------------------: " << i << endl;
    cout << "----------------------i----------------------: " << i << endl;
    if (i > 149) i = 149;
    // 计算反馈力矩 tau_fb = Kp * (q_des - q) + Kd * (v_des - v)
    // 打印q_des、joint_positions、v_des、joint_velocities
    printJointData(q_vecs, v_vecs, joint_positions, joint_velocities, i);
    printVector("joint_tau", joint_tau);
    for (int j = 0; j < 6; ++j) {
      position_error[j] = q_vecs[j][i] - joint_positions[j];
      velocity_error[j] = v_vecs[j][i] - joint_velocities[j];
    }
    // 打印position_error、velocity_error
    printVector("position_error", position_error);
    printVector("velocity_error", velocity_error);

    // 计算反馈力矩
    vector<double> tau_fb(6);
    for (int j = 0; j < 6; ++j) {
      tau_fb[j] = Kp[j] * position_error[j] + Kd[j] * velocity_error[j];
    }
    printVector("tau_fb", tau_fb);
    printVector("tau_ff", tau[i]);

    // 使用映射表将 tau 重新排序
    cout << "tau: ";
    effort_msg.data.clear();  // 确保每次下发时都是清空状态
    for (int j = 0; j < tau[i].size(); ++j) {
      effort_msg.data.push_back(tau[i][j] + tau_fb[j]);
      cout << effort_msg.data[j] << " ";
    }
    cout << endl;
    i++;
    joint_effort_pub.publish(effort_msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
