#!/usr/bin/env python
import serial,thread,time,struct
from ctypes import *

import numpy as np
import cv2
from numpy import *

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu ,Range
import tf

rospy.init_node('optical_flow', anonymous=False)



poseMsg = PoseWithCovarianceStamped()
imuMsg = Imu()
rangeMsg = Range()


x=0.0
y=0.0
cap = cv2.VideoCapture(1)
R = [[0 for x in range(3)] for x in range(3)] 
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
    #rospy.loginfo("roll %f pitch %f yaw %f", roll,pitch, yaw)
    q0=data.orientation.w
    q1=data.orientation.x
    q2=data.orientation.y
    q3=data.orientation.z

  # //direction cosine matrix (DCM), Rotation matrix , Rotated Frame to Stationary Frame XYZ
  # //Quaternions_and_spatial_rotation
  # R[0][0]=DCM00 = 2*(0.5 - q2q2 - q3q3);//2*(0.5 - q2q2 - q3q3);//=q0q0 + q1q1 - q2q2 - q3q3
  # R[0][1]=DCM01 = 2*(q1q2 - q0q3);//2*(q0q1 + q2q3)
  # R[0][2]=DCM02 = 2*(q1q3 + q0q2);//2*(q1q3 - q0q2); 2*(q0q2 - q1q3)
  # R[1][0]=DCM10 = 2*(q1q2 + q0q3);
  # R[1][1]=DCM11 = 2*(0.5 - q1q1 - q3q3);//2*(0.5 - q1q1 - q3q3);//q0q0 - q1q1 + q2q2 - q3q3
  # R[1][2]=DCM12 = 2*(q2q3 - q0q1);
  # R[2][0]=DCM20 = 2*(q1q3 - q0q2);//-sin pitch
  # R[2][1]=DCM21 = 2*(q2q3 + q0q1);
  # R[2][2]=DCM22 = 2*(0.5 - q1q1 - q2q2);//2*(0.5 - q1q1 - q2q2);//=q0q0 - q1q1 - q2q2 + q3q3
   # R[0][0] = 2*(0.5-q2*q2-q3*q3)
  #  R[0][1] = 2*(q1*q2 - q0*q3)
   # R[0][2] = 2*(q1*q3 + q0*q2)
  #  R[1][0] = 2*(q1*q2 + q0*q3)
   # R[1][1] = 2*(0.5 - q1*q1 - q3*q3)
   # R[1][2] = 2*(q2*q3 - q0*q1)
   # R[2][0] = 2*(q1*q3 - q0*q2)
   # R[2][1] = 2*(q2*q3 + q0*q1)
   # R[2][2] = 2*(0.5 - q1*q1 - q2*q2)


    #x_nt = poseMsg.pose.pose.position.y + 25000*math.tan(pitch)
    #y_nt = poseMsg.pose.pose.position.y + 25000*math.tan(pitch)
    # x_nt = poseMsg.pose.pose.position.x*R[1][0]+poseMsg.pose.pose.position.x*R[1][1]+poseMsg.pose.pose.position.x*R[1][2]
    # y_nt = poseMsg.pose.pose.position.y*R[0][0]+poseMsg.pose.pose.position.x*R[0][1]+poseMsg.pose.pose.position.x*R[0][2]

    #rospy.loginfo("x %f y %f tan %f", x_nt,y_nt,25000*math.tan(pitch))
    #rospy.loginfo("roll %f pitch %f yaw %f R[2][2] %f ", roll,pitch, yaw,R[2][2])
#cap= cv2.VideoCapture('untitled2.mp4')
while not rospy.is_shutdown():
	
	# params for ShiTomasi corner detection
	feature_params = dict( maxCorners = 100,
	                       qualityLevel = 0.001,
	                       minDistance = 7,
	                       blockSize = 7 )

	# Parameters for lucas kanade optical flow
	lk_params = dict( winSize  = (15,15),
	                  maxLevel = 2,
	                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

	# Create some random colors
	color = np.random.randint(0,255,(100,3))

	# Take first frame and find corners in it
	ret, old_frame = cap.read()
	old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
	p0 = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)

	# Create a mask image for drawing purposes
	mask = np.zeros_like(old_frame)

	rate = rospy.Rate(100) # 100 hz
	time_start = rospy.Time.now()
	pose = rospy.Publisher('/optical_flow', PoseWithCovarianceStamped, queue_size=10)
	rospy.Subscriber("/imu_max", Imu, imucallback)
	# rospy.Subscriber("/svo/info", Info, visionqcallback)
	while not rospy.is_shutdown():
	    poseMsg.header.stamp = rospy.Time.now()-time_start
	    poseMsg.pose.pose.position.x = x  #pixel
	    poseMsg.pose.pose.position.y = y
	    poseMsg.pose.pose.position.z = len(p0) #track number
	    pose.publish(poseMsg)
	    if len(p0)<75:
		    ret,old_frame = cap.read()
		    old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
		    p0 = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)

	    ret,frame = cap.read()
	    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	    # calculate optical flow
	    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

	    # Select good points
	    try:
		    good_new = p1[st==1]
		    good_old = p0[st==1]
	    except TypeError:
	    	break


	    # draw the tracks
	    for i,(new,old) in enumerate(zip(good_new,good_old)):
	        a,b = new.ravel()
	        c,d = old.ravel()
	        #cv2.line(mask, (a,b),(c,d), color[i].tolist(), 2)
	        cv2.circle(frame,(a,b),5,color[i].tolist(),-1)
	        x=x+a-c
	        y=y+b-d
	        # if i == 99:
	            # break
	    # print x,y
	    X=str(x)
	    Y=str(y)
	    cv2.putText(frame,X+','+Y,(20,50),cv2.FONT_HERSHEY_SIMPLEX,1,color = (255,255,255),thickness = 2)
	    img = cv2.add(frame,mask)

	    cv2.imshow('frame',img)
	    k = cv2.waitKey(30) & 0xff
	    if k == 27:
	        break

	    # Now update the previous frame and previous points
	    old_gray = frame_gray.copy()
	    p0 = good_new.reshape(-1,1,2)
	    print len(p0)
	    rate.sleep()

	# cv2.destroyAllWindows()
	# cap.release()
