#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic


import rospy


from std_msgs.msg import String
# Input messages type
from sensor_msgs.msg import Imu , MagneticField , Joy , Temperature, FluidPressure, JointState ,NavSatFix, Range
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped ,TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from crossover_nav.msg import Ack ,Info, Navdata
#from svo.msg import Info
import time
import serial
#from ctypes import *
rospy.init_node('imu_node', anonymous=True)
import ulink
from datetime import date
import sys
import tf

import roslib;roslib.load_manifest("using_markers") #import cfg from this package
import dynamic_reconfigure.client
#print 'Argument List:', str(sys.argv[1])
#ser = serial.Serial('/dev/ttyUSB0',115200)
ser = serial.Serial(str(sys.argv[1]),sys.argv[2])
ser.setDTR(True)

u=ulink.Ulink(ser)
time.sleep(4)
imuMsgval   = u.imuMsg
poseVal     = u.poseMsg
posenavVal  = u.pose_navMsg 
compassVal  = u.compassMsg
remoteVal   = u.remoteMsg
tempVal     = u.tempMsg
baroVal     = u.baroMsg
gpsrawVal   = u.gpsrawMsg
navvelVal   = u.navvelMsg
ackVal      = u.ackMsg
sonarVal    = u.sonarMsg
Navval      = u.NavMsg
altval      = u.altMsg

imuMsgval.header.frame_id  = 'base_link'
posenavVal.header.frame_id = 'odom'
posenavVal.child_frame_id  = 'base_link'
poseVal.header.frame_id    = 'base_link'
gpsrawVal.header.frame_id  = 'base_link'
navvelVal.header.frame_id  = 'base_link'
altval.header.frame_id     = 'base_link'

vision = PoseWithCovarianceStamped()
vision_quality = Info()
vision_quality_ok = False
vision_pose_ok = False

# def talker():
#     pub = rospy.Publisher('/imu_max', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()
def posedemandcallback(data): #use x y z for position and w for yaw angle
    u.send_position_desire(data.pose.position.x, data.pose.position.y, data.pose.position.z, data.pose.orientation.w)
    #rospy.loginfo(data.pose)
def flowcallback(data): #use x y z for position and w for yaw angle
    u.send_flow(data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)   #flow x flow y tracknumber
    #rospy.loginfo(data.pose.pose.position.x)
def visioncallback(data):
    global vision_pose_ok
    global vision
    vision = data
    vision_pose_ok = True
    #rospy.loginfo(vision.pose.pose.position.x)    
def visionqcallback(data):
    global vision_quality_ok
    global vision_quality 
    vision_quality = data
    vision_quality_ok = True
    #rospy.loginfo(vision_quality.tracking_quality)    
def callback(data):
    u.send_motor_desire(data.velocity[0],data.velocity[1],data.velocity[2],data.velocity[3])
    rospy.loginfo(data.velocity)
def CFGcallback(config):
    a=0
    #rospy.loginfo("gc")
    # rospy.loginfo("""Reconfigure Request: {wv_param}, {wp_param},\ 
    #       {wbias_param}, {wv_param}, {delaygps_param}""".format(**config))
def CFGcallback2(config):
    #rospy.loginfo("pid")
    rospy.loginfo("PIDSERVER")
    d1=[config.KplevelRoll_param,config.KprateRoll_param, config.KirateRoll_param,config.KdrateRoll_param, config.KplevelPitch_param, config.KpratePitch_param]
    d2=[config.KiratePitch_param,config.KdratePitch_param,config.KplevelYaw_param,config.KprateYaw_param, config.KirateYaw_param, config.KdrateYaw_param]
    d3=[config.lambdaalt_param  ,config.KpAltitude_param, config.KiAltitude_param,config.KdAltitude_param,config.KaAltitude_param]
    d4=[config.KpVel_param      ,config.KiVel_param,      config.KdVel_param,     config.KpPos_param,config.KiPos_param]

    # rospy.loginfo(ackVal.ACK_PENDING)
    while ackVal.ACK_PENDING ==1 :
        u.send_pid1_param(d1)
        u.send_pid2_param(d2)
        u.send_pid_alt_param(d3)
        u.send_pid_nav_param(d4)
        time.sleep(2.0)
    # rospy.loginfo("All config was sent")
    if ackVal.ACK_PID_RPY1 ==0 and config.sending_pid1 ==1 :
        u.send_pid1_param(d1)
        rospy.loginfo("pid1 sent")
        time.sleep(1.0)
    
    if ackVal.ACK_PID_RPY2 ==0 and config.sending_pid2 ==1 :
        u.send_pid2_param(d2)
        rospy.loginfo("pid2 sent")
        time.sleep(1.0)
    
    if ackVal.ACK_PID_ALT ==0 and config.sending_pid_alt ==1 :
        u.send_pid_alt_param(d3)
        rospy.loginfo("pid3 sent")
        time.sleep(1.0)
    
    if ackVal.ACK_PID_NAV ==0 and config.sending_pid_NAV ==1 :
        u.send_pid_nav_param(d4)
        rospy.loginfo("pid4 sent")
        time.sleep(1.0)
    

def imutalk():
    imu = rospy.Publisher('/imu_max', Imu, queue_size=10)
    pose = rospy.Publisher('/imu_max/pose', PoseWithCovarianceStamped, queue_size=2)
    posenav = rospy.Publisher('/imu_max/pose_nav', Odometry, queue_size=10)
    alt     = rospy.Publisher('/imu_max/alt_odometry', Odometry, queue_size=10)
    #mag = rospy.Publisher('/imu_max/Mag', MagneticField, queue_size=10)
    #remote = rospy.Publisher('/imu_max/Remote', Joy, queue_size=10)
    #temp = rospy.Publisher('/imu_max/Temp', Temperature, queue_size=10)
    #baro = rospy.Publisher('/imu_max/Baro', FluidPressure, queue_size=10)
    gpsraw = rospy.Publisher('/imu_max/Gpsraw', NavSatFix, queue_size=2)
    navvel = rospy.Publisher('/imu_max/Navvel', TwistWithCovarianceStamped,queue_size=10)
    sonar  = rospy.Publisher('/sonar', Range, queue_size=10)
    Nav    = rospy.Publisher('/imu_max/Navdata',Navdata, queue_size=10)

    time.sleep(3.0)
    client1 = dynamic_reconfigure.client.Client("general_config", timeout=15, config_callback=CFGcallback)
    client2 = dynamic_reconfigure.client.Client("PID_TUNE", timeout=15, config_callback=CFGcallback2)

    rate = rospy.Rate(200) # 120 hz

    #write logfile
    d = date.today()
    #logfile = open('logfile-%s.txt'%d.isoformat(),'w')

    # rospy.Subscriber("/motor_out", JointState, callback)
    global vision
    global vision_quality 
    global vision_quality_ok
    global vision_pose_ok

    rospy.Subscriber("/pose_demand", PoseStamped, posedemandcallback)
    rospy.Subscriber("/svo/pose", PoseWithCovarianceStamped, visioncallback)
    rospy.Subscriber("/svo/info", Info, visionqcallback)
    rospy.Subscriber("/optical_flow", PoseWithCovarianceStamped, flowcallback)

    timeout_100 = rospy.Duration.from_sec(0.005)
    timeout_50 = rospy.Duration.from_sec(0.01)
    timeout_33 = rospy.Duration.from_sec(0.04)
    timeout_15 = rospy.Duration.from_sec(0.1)
    while not rospy.is_shutdown():
        #imuMsgval.linear_acceleration.x = 55.5
        t_now = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        
        if t_now-imuMsgval.header.stamp< timeout_100 : 
            imu.publish(imuMsgval)
            Nav.publish(Navval) 
        if t_now-poseVal.header.stamp< timeout_100 : 
            pose.publish(poseVal) 
        if t_now-posenavVal.header.stamp< timeout_100 :
            posenav.publish(posenavVal)  
        #mag.publish(compassVal)
        #remote.publish(remoteVal)
        #temp.publish(tempVal)
        #baro.publish(baroVal)
        if t_now-gpsrawVal.header.stamp< timeout_100 :
            gpsraw.publish(gpsrawVal)    
        if t_now-navvelVal.header.stamp< timeout_100 :
            navvel.publish(navvelVal)    
        if t_now-sonarVal.header.stamp< timeout_100:
            sonar.publish(sonarVal)  
        # if t_now- altval.header.stamp < timeout_100 :
        #     alt.publish(altval)
        alt.publish(altval)
        br = tf.TransformBroadcaster()
        br.sendTransform((posenavVal.pose.pose.position.x,posenavVal.pose.pose.position.y,posenavVal.pose.pose.position.z),(imuMsgval.orientation.x,imuMsgval.orientation.y,imuMsgval.orientation.z,imuMsgval.orientation.w),t_now,
                     "base_link",
                     "odom")
        #rospy.loginfo(vision_pose_ok)
        #rospy.loginfo(vision_quality_ok)
        #rospy.loginfo("-----------")
        if vision_pose_ok and vision_quality_ok :
            #rospy.loginfo("vision sent")
            data=[vision.pose.pose.position.x, vision.pose.pose.position.y, vision.pose.pose.position.z, vision_quality.tracking_quality]
            u.send_vision_data(data)
            vision_pose_ok = False
            vision_quality_ok = False
             
        # if ackVal.ACK_PID_RPY1 == 1 :
        #     client2.update_configuration({"sending_pid1":0})
        # if ackVal.ACK_PID_RPY2 == 1 :
        #     client2.update_configuration({"sending_pid2":0})
        # if ackVal.ACK_PID_ALT == 1 :
        #     client2.update_configuration({"sending_pid_alt":0})
        # if ackVal.ACK_PID_NAV == 1 :
        #     client2.update_configuration({"sending_pid_NAV":0})
        
        #write logfile 1-8 can use variable
        #logfile.write('%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n'%(t_now,1,2,3,4,5,6,7,8))
        #rospy.spinOnce()
        #rospy.spin()
        rate.sleep()
    #write logfile
    # logfile.close()
if __name__ == '__main__':
    
    try:
        imutalk()
    except rospy.ROSInterruptException:
        pass
