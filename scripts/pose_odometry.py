#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf
import sys

odotoposeMsg = PoseWithCovarianceStamped()
#roll, pitch, yaw = 0;
rospy.init_node('odo_to_pose', anonymous=True)

def imucallback(data):
    # quaternion = (
    # data.orientation.x,
    # data.orientation.y,
    # data.orientation.z,
    # data.orientation.w)
    # euler = tf.transformations.euler_from_quaternion(quaternion)
    # roll = euler[0]
    # pitch = euler[1]
    # yaw = euler[2]
    # rospy.loginfo("roll %f pitch %f yaw %f", roll,pitch, yaw)
    odotoposeMsg.pose.pose.orientation = data.orientation


def odocalback(data):
    odotoposeMsg.header = data.header
    odotoposeMsg.header.frame_id = str(sys.argv[2])
    odotoposeMsg.pose.pose.position = data.pose.pose.position
    odotoposeMsg.pose.pose.orientation = data.pose.pose.orientation
    odotoposeMsg.pose.covariance = data.pose.covariance
    


    # rospy.loginfo("x %f y %f z %f", odotoposeMsg.pose.pose.position.x ,odotoposeMsg.pose.pose.position.y , odotoposeMsg.pose.pose.position.z)

def main():

    pose = rospy.Publisher('/pub_pose', PoseWithCovarianceStamped, queue_size=10)
    freq = int(sys.argv[1])
    rate = rospy.Rate(freq) # 120 hz

    rospy.Subscriber("/sub_odo", Odometry, odocalback)
    if str(sys.argv[3])=="include_orientation" :
        rospy.Subscriber("/imu_max", Imu, imucallback)
    while not rospy.is_shutdown():
        t_now = rospy.Time.now()
        pose.publish(odotoposeMsg) 
        rate.sleep()
        

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass