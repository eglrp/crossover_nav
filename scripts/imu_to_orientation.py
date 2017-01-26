#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
# from geometry_msgs.msg import PoseWithCovarianceStamped
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Quaternion
import tf
import sys


orientation_ = Quaternion()
#roll, pitch, yaw = 0;
rospy.init_node('odo_to_pose', anonymous=True)

def imucallback(data):

    orientation_.x= data.orientation.x
    orientation_.y= data.orientation.y
    orientation_.z= data.orientation.z
    orientation_.w= data.orientation.w

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

def main():

    quater = rospy.Publisher('/pub_orientation', Quaternion, queue_size=10)
    freq = int(sys.argv[1])
    rate = rospy.Rate(freq) # 120 hz

    rospy.Subscriber("/sub_imu", Imu, imucallback)
    while not rospy.is_shutdown():
        t_now = rospy.Time.now()
        quater.publish(orientation_) 
        rate.sleep()
        

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass