import rospy
import tf
from tf.transformations import euler_from_quaternion

def listen_to_transforms():
    rospy.init_node('tf_listener')
    listener = tf.TransformListener()

    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        try:
            # 确保在请求之前，变换已经被发布一段时间
            now = rospy.Time(0)  # 请求最新的变换
            listener.waitForTransform('/base', '/wrist_3_link', now, rospy.Duration(4.0))
            (trans, rot) = listener.lookupTransform('/base', '/wrist_3_link', now)

            print("Translation: ", trans)
            print("Rotation(xyzw): ", rot)
            # 将四元数转换为欧拉角
            euler = euler_from_quaternion(rot)
            print("Rotation (RPY): ", euler)  # 输出为 (roll, pitch, yaw)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            continue

        rate.sleep()

if __name__ == '__main__':
    listen_to_transforms()