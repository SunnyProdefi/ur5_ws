import rospy
from std_srvs.srv import Trigger

def call_trajectory_planner():
    rospy.init_node('trajectory_planner_client')
    rospy.wait_for_service('/trajectory_planner')
    try:
        trajectory_planner = rospy.ServiceProxy('/trajectory_planner', Trigger)
        response = trajectory_planner()
        rospy.loginfo(f"Service call successful: {response.success}, {response.message}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    call_trajectory_planner()
