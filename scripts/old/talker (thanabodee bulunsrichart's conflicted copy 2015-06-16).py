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
from sensor_msgs.msg import Imu , MagneticField , Joy , Temperature, FluidPressure
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import time
import serial
#from ctypes import *
rospy.init_node('imu_node', anonymous=True)
import ulink
from datetime import date


ser = serial.Serial('/dev/ttyUSB1',115200)
u=ulink.Ulink(ser)
time.sleep(4)
imuMsgval=ulink.Ulink.imuMsg
imuMsgval.header.frame_id = 'world'
poseVal = ulink.Ulink.poseMsg
posenavVal = ulink.Ulink.pose_navMsg 
compassVal = ulink.Ulink.compassMsg
remoteVal = ulink.Ulink.remoteMsg
tempVal = ulink.Ulink.tempMsg
baroVal = ulink.Ulink.baroMsg
# def talker():
#     pub = rospy.Publisher('/imu_max', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()
def imutalk():
    imu = rospy.Publisher('/imu_max', Imu, queue_size=10)
    pose = rospy.Publisher('/imu_max/pose', PoseWithCovarianceStamped, queue_size=10)
    posenav = rospy.Publisher('/imu_max/pose_nav', PoseWithCovarianceStamped, queue_size=10)
    mag = rospy.Publisher('/imu_max/Mag', MagneticField, queue_size=10)
    remote = rospy.Publisher('/imu_max/Remote', Joy, queue_size=10)
    temp = rospy.Publisher('/imu_max/Temp', Temperature, queue_size=10)
    baro = rospy.Publisher('/imu_max/Baro', FluidPressure, queue_size=10)
    rate = rospy.Rate(100) # 10hz

    #write logfile
    d = date.today()
    logfile = open('logfile-%s.txt'%d.isoformat(),'w')

    

    while not rospy.is_shutdown():
        #imuMsgval.linear_acceleration.x = 55.5
        imuMsgval.header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        imu.publish(imuMsgval)
        pose.publish(poseVal)
        posenav.publish(posenavVal)
        mag.publish(compassVal)
        remote.publish(remoteVal)
        temp.publish(tempVal)
        baro.publish(baroVal)

        #write logfile 1-8 can use variable
        #logfile.write('%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n'%(rospy.Time.now(),1,2,3,4,5,6,7,8))
        rate.sleep()
    #write logfile
    logfile.close()
if __name__ == '__main__':
    
    try:
        imutalk()
    except rospy.ROSInterruptException:
        pass
