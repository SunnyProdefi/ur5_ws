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
vector<double> joint_accelerations;

ros::Publisher joint_effort_pub;
bool received_state = false;

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
  double temp = tau_i[0];
  tau_i[0] = tau_i[2];
  tau_i[2] = temp;
  tau.push_back(tau_i);
  //   pinocchio::crba(model, data, q);
  //   Eigen::MatrixXd &M = data.M;
  //   cout << "Mass matrix:\n" << M << endl;
  //   Eigen::VectorXd c = pinocchio::nonLinearEffects(model, data, q, v);
  //   cout << "Coriolis and centrifugal forces: " << c.transpose() << endl;
  //   Eigen::VectorXd g = pinocchio::computeGeneralizedGravity(model, data, q);
  //   cout << "Gravity vector: " << g.transpose() << endl;
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

  for (int f = 0; f < files.size(); ++f) {
    ifstream file(files[f]);
    string line, cell;

    if (file.is_open()) {
      getline(file, line);  // 读取并跳过标题行
      int i = 0;
      while (getline(file, line) && i < num) {
        stringstream lineStream(line);
        getline(lineStream, cell, ',');  // 跳过时间列
        getline(lineStream, cell, ',');
        q_vecs[f][i] = stod(cell);
        getline(lineStream, cell, ',');
        v_vecs[f][i] = stod(cell);
        getline(lineStream, cell, ',');
        a_vecs[f][i] = stod(cell);
        i++;
      }
      file.close();
    } else {
      cerr << "Failed to open file: " << files[f] << endl;
    }
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
    // printModelDataDetails(model, data);
    computeDynamics(model, data, q_des, v_des, a_des);
  }
  ros::Rate rate(50);
  while (ros::ok() && !received_state) {
    ros::spinOnce();
    rate.sleep();
    cout << "Waiting for joint states..." << endl;
  }

  for (int i = 0; i < num; i++) {
    for (int j = 0; j < 2; ++j) {
      if (j == 0) tau[i][j] = tau[i][j] / 1.2;
      if (j == 1) tau[i][j] = tau[i][j] / 2.6;
    }
  }

  int i = 0;
  vector<double> Kp(6);
  vector<double> Kd(6);
  // elbow_joint
  // shoulder_lift_joint
  // shoulder_pan_joint
  // wrist_1_joint
  // wrist_2_joint
  // wrist_3_joint
  Kp = {10, 10, 0.1, 0.1, 0.1, 0.1};
  Kd = {0.1, 0.1, 0.01, 0.01, 0.01, 0.01};
  vector<double> position_error(6), velocity_error(6);
  while (ros::ok()) {
    std_msgs::Float64MultiArray effort_msg;
    cout << "----------------------i----------------------: " << i << endl;
    cout << "----------------------i----------------------: " << i << endl;
    if (i > 149) i = 149;
    // 计算反馈力矩 tau_fb = Kp * (q_des - q) + Kd * (v_des - v)
    // 打印q_des、joint_positions、v_des、joint_velocities
    cout << "q_des: " << q_vecs[2][i] << " " << q_vecs[1][i] << " "
         << q_vecs[0][i] << " " << q_vecs[3][i] << " " << q_vecs[4][i] << " "
         << q_vecs[5][i] << endl;
    cout << "joint_positions: ";
    for (int j = 0; j < 6; ++j) {
      cout << joint_positions[j] << " ";
    }
    cout << "\nv_des: " << v_vecs[2][i] << " " << v_vecs[1][i] << " "
         << v_vecs[0][i] << " " << v_vecs[3][i] << " " << v_vecs[4][i] << " "
         << v_vecs[5][i] << endl;
    cout << "joint_velocities: ";
    for (int j = 0; j < 6; ++j) {
      cout << joint_velocities[j] << " ";
    }

    position_error[0] = q_vecs[2][i] - joint_positions[0];
    velocity_error[0] = v_vecs[2][i] - joint_velocities[0];
    position_error[2] = q_vecs[0][i] - joint_positions[2];
    velocity_error[2] = v_vecs[0][i] - joint_velocities[2];
    for (int j = 0; j < 6; ++j) {
      if (j == 0 || j == 2) continue;
      position_error[j] = q_vecs[j][i] - joint_positions[j];
      velocity_error[j] = v_vecs[j][i] - joint_velocities[j];
    }
    // 打印position_error、velocity_error
    cout << "\nposition_error: ";
    for (int j = 0; j < 6; ++j) {
      cout << position_error[j] << " ";
    }
    cout << "\nvelocity_error: ";
    for (int j = 0; j < 6; ++j) {
      cout << velocity_error[j] << " ";
    }
    vector<double> tau_fb(6);
    cout << "\ntau_fb: ";
    for (int j = 0; j < 6; ++j) {
      tau_fb[j] = Kp[j] * position_error[j] + Kd[j] * velocity_error[j];
      cout << tau_fb[j] << " ";
    }
    cout << "\ntau_ff: " << "i: " << i << " ";
    for (int j = 0; j < 6; ++j) {
      cout << tau[i][j] << " ";
    }
    cout << "\ntau: ";
    for (int j = 0; j < tau[i].size(); ++j) {
      effort_msg.data.push_back(tau[i][j] + tau_fb[j]);
      //   effort_msg.data.push_back(tau[i][j]);
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
