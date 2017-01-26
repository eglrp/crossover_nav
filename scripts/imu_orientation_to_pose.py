#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
# from nav_msgs.msg import Odometry
import tf
import sys

imutoposeMsg = PoseWithCovarianceStamped()
#roll, pitch, yaw = 0;
rospy.init_node('odo_to_pose', anonymous=True)

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
    imutoposeMsg.pose.pose.orientation.x = data.orientation.x
    imutoposeMsg.pose.pose.orientation.y = data.orientation.y
    imutoposeMsg.pose.pose.orientation.z = data.orientation.z
    imutoposeMsg.pose.pose.orientation.w = data.orientation.w

    #xyz
    imutoposeMsg.header.stamp = rospy.Time.now()
    imutoposeMsg.pose.covariance[0]=10
    imutoposeMsg.pose.covariance[7]=10
    imutoposeMsg.pose.covariance[14]=10
    #rpy 
    imutoposeMsg.pose.covariance[21]=0.000001
    imutoposeMsg.pose.covariance[28]=0.000001
    imutoposeMsg.pose.covariance[35]=0.000001
# def odocalback(data):
#     odotoposeMsg.header = data.header
#     odotoposeMsg.header.frame_id = str(sys.argv[2])
#     odotoposeMsg.pose = data.pose 
#     odotoposeMsg.pose.covariance = data.pose.covariance

    # rospy.loginfo("x %f y %f z %f", odotoposeMsg.pose.pose.position.x ,odotoposeMsg.pose.pose.position.y , odotoposeMsg.pose.pose.position.z)

def main():

    pose = rospy.Publisher('/pub_pose', PoseWithCovarianceStamped, queue_size=10)
    freq = int(sys.argv[1])
    rate = rospy.Rate(freq) # 120 hz

    rospy.Subscriber("/sub_imu", Imu, imucallback)
    while not rospy.is_shutdown():
        t_now = rospy.Time.now()
        pose.publish(imutoposeMsg) 
        rate.sleep()
        

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass