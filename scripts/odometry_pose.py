#!/usr/bin/env python
import rospy
from std_msgs.msg import String
#from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped,TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf
import sys

posetoodoMsg = Odometry()
#roll, pitch, yaw = 0;
rospy.init_node('posetwist_to_odom', anonymous=True)

# def imucallback(data):
#     quaternion = (
#     data.orientation.x,
#     data.orientation.y,
#     data.orientation.z,
#     data.orientation.w)
#     euler = tf.transformations.euler_from_quaternion(quaternion)
#     roll = euler[0]
#     pitch = euler[1]
#     yaw = euler[2]
#     rospy.loginfo("roll %f pitch %f yaw %f", roll,pitch, yaw)
def odocalback(data):
    posetoodoMsg.header = data.header
    posetoodoMsg.pose = data.pose 
    posetoodoMsg.pose.covariance = data.pose.covariance
    # rospy.loginfo("x %f y %f z %f", posetoodoMsg.pose.pose.position.x ,posetoodoMsg.pose.pose.position.y , posetoodoMsg.pose.pose.position.z)
def odocalback2(data):
    posetoodoMsg.header = data.header
    posetoodoMsg.twist = data.twist 
    posetoodoMsg.twist.covariance = data.twist.covariance
    # rospy.loginfo("vx %f vy %f vz %f", posetoodoMsg.twist.twist.linear.x ,posetoodoMsg.twist.twist.linear.y , posetoodoMsg.twist.twist.linear.z)

def main():

    pose = rospy.Publisher('/pub_odo', Odometry, queue_size=10)
    freq = int(sys.argv[1])
    rate = rospy.Rate(freq) # 120 hz

    rospy.Subscriber("/sub_pose", PoseWithCovarianceStamped, odocalback)
    rospy.Subscriber("/sub_twist", TwistWithCovarianceStamped, odocalback2)
    while not rospy.is_shutdown():
        posetoodoMsg.header.frame_id = str(sys.argv[2])
        posetoodoMsg.child_frame_id = str(sys.argv[3])

        t_now = rospy.Time.now()
        pose.publish(posetoodoMsg) 
        rate.sleep()
        

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass