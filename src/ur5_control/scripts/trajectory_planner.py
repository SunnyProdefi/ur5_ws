import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline
import pandas as pd

# 初始化全局变量
data_received = False  # 用于指示是否已接收到数据
joint_data = None  # 用于存储接收到的JointState数据
subscriber = None  # 用于存储订阅者对象

def plot_joint_trajectories(joint_trajectories, t_fine):
    # 创建一个3行1列的图形，用于显示位置、速度和加速度
    fig, axs = plt.subplots(3, 1, figsize=(10, 15))  # figsize可以调整图形的大小

    # 绘制关节位置
    for trajectory in joint_trajectories:
        axs[0].plot(t_fine, trajectory["positions"], label=f"{trajectory['joint_name']} position")
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel("Position (rad)")
    axs[0].legend()
    axs[0].set_title("Joint Positions")

    # 绘制关节速度
    for trajectory in joint_trajectories:
        axs[1].plot(t_fine, trajectory["velocities"], label=f"{trajectory['joint_name']} velocity")
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel("Velocity (rad/s)")
    axs[1].legend()
    axs[1].set_title("Joint Velocities")

    # 绘制关节加速度
    for trajectory in joint_trajectories:
        axs[2].plot(t_fine, trajectory["accelerations"], label=f"{trajectory['joint_name']} acceleration")
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel("Acceleration (rad/s²)")
    axs[2].legend()
    axs[2].set_title("Joint Accelerations")

    # 自动调整子图布局，避免重叠
    plt.tight_layout()

    # 保存和显示图形
    plt.savefig("joint_trajectories.png")
    plt.show()

def process_joint_data(joint_data):
    # 确保joint_data不是None
    if joint_data is None:
        rospy.logwarn("No joint data to process.")
        return

    joint_names = joint_data.name
    joint_positions = joint_data.position

    # 定义初始和结束时间点
    t = np.array([0, 3])  # 从时间0到时间3
    k = 5  # 样条的阶数
    # 边界条件：两端的一阶和二阶导数都设置为0（代表位置、速度、加速度）
    bc_type = ((1, 0.0), (2, 0.0)), ((1, 0.0), (2, 0.0))
    t_fine = np.linspace(t.min(), t.max(), 150)  # 在0到3之间生成150个时间点,频率：50Hz

    joint_trajectories = []

    # 循环遍历每个关节
    for i, joint_name in enumerate(joint_names):
        joint_positions_desired = 0
        y = np.array([joint_positions[i], joint_positions_desired])
        # 创建样条曲线
        tck = make_interp_spline(t, y, k=k, bc_type=bc_type)
        # 计算细分时间点上的位置、速度和加速度
        positions = tck(t_fine)
        velocities = tck(t_fine, nu=1)  # 一阶导数
        accelerations = tck(t_fine, nu=2)  # 二阶导数

        # 将计算结果存储到列表中
        joint_trajectories.append({
            "joint_name": joint_name,
            "positions": positions,
            "velocities": velocities,
            "accelerations": accelerations
        })

    # 写入CSV文件
    for trajectory in joint_trajectories:
        df = pd.DataFrame({
            "time": t_fine,
            "position": trajectory["positions"],
            "velocity": trajectory["velocities"],
            "acceleration": trajectory["accelerations"]
        })
        # 为每个关节生成一个CSV文件
        df.to_csv(f"{trajectory['joint_name']}_trajectory.csv", index=False)

    rospy.loginfo("Trajectories written to CSV files.")

    # 绘制关节轨迹
    plot_joint_trajectories(joint_trajectories, t_fine)
    rospy.loginfo("Trajectories plotted.")

def callback(data):
    global data_received, joint_data
    joint_data = data  # 存储接收到的数据
    data_received = True  # 设置标志

def handle_trajectory_planner(req):
    global subscriber, data_received

    if subscriber is None:  # 确保只初始化一次
        subscriber = rospy.Subscriber('/joint_states', JointState, callback)  # 订阅话题
    
    data_received = False  # 重置标志
    # 等待数据接收完成
    while not data_received:
        rospy.sleep(0.1)

    # 处理接收到的关节数据
    process_joint_data(joint_data)

    return TriggerResponse(success=True, message="Trajectory planning completed successfully")

def trajectory_planner_server():
    rospy.init_node('trajectory_planner_server')
    rospy.Service('/trajectory_planner', Trigger, handle_trajectory_planner)
    rospy.spin()

if __name__ == '__main__':
    trajectory_planner_server()
