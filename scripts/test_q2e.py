#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf


imuMsg = Imu()
#roll, pitch, yaw = 0;


def imucallback(data):
    quaternion = (
    data.orientation.x,
    data.orientation.y,
    data.orientation.z,
    data.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    rospy.loginfo("roll %f pitch %f yaw %f", roll,pitch, yaw)
def posecallback(data):
    quaternion = (
    data.pose.pose.orientation.x,
    data.pose.pose.orientation.y,
    data.pose.pose.orientation.z,
    data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    rospy.loginfo("roll %f pitch %f yaw %f", roll,pitch, yaw)  
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=False)

    #rospy.Subscriber("/mavros/imu/data", Imu, imucallback)
    rospy.Subscriber("/xxx", PoseWithCovarianceStamped, posecallback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
