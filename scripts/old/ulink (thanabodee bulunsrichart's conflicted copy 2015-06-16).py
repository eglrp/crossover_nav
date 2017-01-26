#!/usr/bin/env python
import serial,thread,time,struct
from ctypes import *

import rospy
from std_msgs.msg import String
# Input messages type
from sensor_msgs.msg import Imu ,MagneticField , Joy , Temperature, FluidPressure
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
# from sensor_msgs.msg import MagneticField
import time
import serial
import tf
#from ctypes import *

#msg_id

ATTITUDE        = 1 #float 3
RAW_IMU         = 2 #float 6
RAW_BARO        = 3 #float 3
RAW_RADIO       = 4 #int 5
MOTOR_OUT       = 5 #int 4
ALTITUDE        = 6 #float 3
ACC_AXIS        = 7 #float 3
V_VAL           = 8 #float 2
CNT_VAL         = 9 #float 3 cnt roll,pitch,yaw
STATUS_CHK      = 10 #int 1
GPS             = 11 #long 2
DATA_REQUEST    = 12 #chr 1
CMD_TAKEOFF     = 13 #chr 1
UNKNOWN_TYPE    = 14 #unknown
UNKNOWN_FLOAT   = 15 
CONT_COMPASS    = 16 #int16 6
CONT_PID        = 17 #float 
NAV_DATA        = 18

#data Streams

DATA_ATT        = 1
DATA_ALL_RAW    = 2
DATA_RAW_IMU    = 3
DATA_RAW_BARO   = 4
DATA_RAW_RADIO  = 5
DATA_GPS        = 6
DATA_MOTOR      = 7
DATA_CNT_VAL    = 8
DATA_STATUS     = 9
DATA_ALLTITUDE  = 10
DATA_V_VAL      = 11
DATA_ACC_AXIS   = 12
DATA_QUATERNION = 13



class Ulink:
    msg=dict()
    CONT=dict()
    imuMsg = Imu()
    poseMsg = PoseWithCovarianceStamped()
    pose_navMsg = PoseWithCovarianceStamped()
    compassMsg = MagneticField()
    remoteMsg = Joy()
    baroMsg = FluidPressure()
    tempMsg = Temperature()
    #rospy.init_node('imu_node2', anonymous=True)
    time_start=rospy.Time.now()
    imuMsg.orientation_covariance = [0.005 , 0 , 0, 0, 0.005, 0, 0, 0, 0.005]
    imuMsg.angular_velocity_covariance = [1.2184696791468346e-7, 0 , 0, 0 , 1.2184696791468346e-7, 0, 0 , 0 , 1.2184696791468346e-7]
    imuMsg.linear_acceleration_covariance = [8.99999999999e-8 , 0 , 0, 0 , 8.99999999999e-8, 0, 0 , 0 , 8.99999999999e-8]

    
    #u = serial.Serial()
    def __init__(self,ser):
        try :
            self.s = ser
            print "Conecting.."
        except ValueError:
            print "Cannot connect the Port"
        if self.s.isOpen() :
            print "Conected"
            time.sleep(2)
##            self.u.write('X')
##            print 'wait start..'
##            while (ord(self.u.read(1)) != 0xa1) :
##                time.sleep(0.1)
##                'wait start cmd'
##            time.sleep(0.2)
            print "sending data request"
            self.send_data_request(self.s,DATA_ATT)
            self.send_data_request(self.s,DATA_GPS)
            print "Starting message handler thread"
            self.msg['STATUS'] = 'alive'
            thread.start_new_thread(self.MsgHandler, (self.s,))
            #thread.start_new_thread(self.cmd_send,())
            #self.send_cmd_takeoff(5)
##            for i in range (0,10) :
##                print self.u.write(chr(0xaa))

    def MsgHandler(self,s) :
        while s.isOpen():
            if s.inWaiting() > 0:
                ch=s.read(1)
                if ord(ch) == 0xb5 :
                    hdr=ord(ch)
                    lenp=ord(s.read(1))
                    idp=ord(s.read(1))
                    pay=s.read(lenp)
                    chk=struct.unpack('h',s.read(2))[0]
                    summ=0

                    for c in pay :
                        summ=summ+ord(c)
                    
                    if chk == hdr + lenp+ idp + summ :
                        #print 'chk ok'
                        #print idp,lenp
                        self.packet_decode(idp,pay,lenp)
                        

    def wait_alive(self,s) :
        'nothing'

    
    def send_data_request(self,s,d) :
        #hdr =0xb5 len =1 id = 6
        chk = 181+1+DATA_REQUEST+d
        data = struct.pack('cccch',chr(181),chr(1),chr(DATA_REQUEST),chr(d),chk)
        s.write( data )
    
    def send_cmd_takeoff(self,h) :
        chk = 181+1+CMD_TAKEOFF+h
        return self.s.write( struct.pack('cccch',chr(181),chr(1),chr(CMD_TAKEOFF),chr(h),chk))

    def send_position_desire(self,px,py,pz,yaw) :
         dlen = 16
         head = struct.pack('ccc',chr(181),chr(dlen),chr(DATA_MOTOR))
         data = struct.pack('ffff',px,py,pz,yaw)
         summ= 181+dlen+DATA_MOTOR
         for d in str(data):
             summ+=ord(d)
         chk = struct.pack('h',summ)
         tatal = len(head)+len(data)+len(chk)
         self.s.write( head )
         self.s.write( data )
         self.s.write( chk )
         return tatal

    
    def packet_decode(self,msgid,payload,lenp) :
        #print 'get ',msgid
        if msgid == ATTITUDE :
                #print 'id : ' ,idp
                d=struct.unpack('fff',payload)
                #d=struct.unpack('iii',payload)
                self.msg.update({'ATTITUDE': d})
                q = tf.transformations.quaternion_from_euler(d[0],d[1],d[2]+1.57079632679)  #roll pitch yaw   (addition for use in rviz +p/2)
                self.imuMsg.header.stamp = rospy.Time.now()-self.time_start
                self.imuMsg.orientation.x=q[0]
                self.imuMsg.orientation.y=q[1]
                self.imuMsg.orientation.z=q[2]
                self.imuMsg.orientation.w=q[3]
                self.poseMsg.header.stamp = rospy.Time.now()-self.time_start
                self.poseMsg.pose.pose.orientation.x=self.imuMsg.orientation.x
                self.poseMsg.pose.pose.orientation.y=self.imuMsg.orientation.y
                self.poseMsg.pose.pose.orientation.z=self.imuMsg.orientation.z
                self.poseMsg.pose.pose.orientation.w=self.imuMsg.orientation.w
                #self.imuMsg.linear_acceleration.x = d[0]
                #self.imuMsg.linear_acceleration.y = d[1]
                #self.imuMsg.linear_acceleration.z = d[2]
                #print 'ATT ',d[0],d[1],d[2]
		# print 'ATTITUDE roll', str(d[0]*180/3.14)[:5] ,' pitch :' , str(d[1]*180/3.14)[:5] ,' yaw:' , str(d[2]*180/3.14)[:6]

        elif msgid == RAW_IMU :
                d=struct.unpack('fffffffff',payload)
                self.msg.update({'RAW_IMU': d})
                self.imuMsg.linear_acceleration.x = d[0]*9.80655
                self.imuMsg.linear_acceleration.y = d[1]*9.80655
                self.imuMsg.linear_acceleration.z = d[2]*9.80655
                self.imuMsg.angular_velocity.x = d[3]
                self.imuMsg.angular_velocity.y = d[4]
                self.imuMsg.angular_velocity.z = d[5]
                self.compassMsg.header.stamp = rospy.Time.now()-self.time_start
                self.compassMsg.magnetic_field.x = d[6]
                self.compassMsg.magnetic_field.y = d[7]
                self.compassMsg.magnetic_field.z = d[8]
                
        elif msgid == ALTITUDE :
                d=struct.unpack('ff',payload)
                self.msg.update({'ALTITUDE': d})
                self.poseMsg.pose.pose.position.z = d[0]
                self.pose_navMsg.pose.pose.position.z = d[0]
                # print 'ALT  rel_ALT : ', str(d[0]*100)[:4] ,' vel_z :' , str(d[1]*10)[:4]
                
        elif msgid == RAW_BARO :
                d=struct.unpack('ff',payload)  #pressure , temperature
                self.msg.update({'RAW_BARO': d})
                self.baroMsg.fluid_pressure = d[0]
                self.tempMsg.temperature = d[1]

                
        elif msgid == RAW_RADIO :
                d=struct.unpack('HHHHHH',payload)
                self.msg.update({'RAW_RADIO': d})
                self.remoteMsg.header.stamp = rospy.Time.now()-self.time_start
                # self.remoteMsg.pose.position.x = d[0]
                # self.remoteMsg.pose.position.y = d[1]
                # self.remoteMsg.pose.position.z = d[2]
                # self.remoteMsg.pose.orientation.x = d[3]
                # self.remoteMsg.pose.orientation.y = d[4]
                # self.remoteMsg.pose.orientation.z = d[5]
                self.remoteMsg.axes= d[0:5]
                self.send_position_desire(d[2]/8,d[2]/8,d[2]/8,d[2]/8)
                #print 'r1:', str(d[0])[:4] ,'r2:' , str(d[1])[:4] ,'r3:' , str(d[2])[:4] ,'r4:' , str(d[3])[:4] ,'r5:' , str(d[4])[:4] ,'r6:' , str(d[5])[:4]
                        
        elif msgid == MOTOR_OUT :
                d=struct.unpack('hhhh',payload)
                self.msg.update({'MOTOR_OUT': d})

        elif msgid == CNT_VAL :
                d=struct.unpack('fff',payload)
                self.msg.update({'CNT_VAL': d})
                # print 'CONTROL  ROLL : ', str(d[0])[:4] ,' PITCH :' , str(d[1])[:4] ,' YAW :' , str(d[2])[:4]
                
        elif msgid == STATUS_CHK :
                d=struct.unpack('h',payload)
                self.msg.update({'STATUS_CHK': d})
                print 'NUM SEN : ', str(d[0])
                
        elif msgid == UNKNOWN_TYPE :
                print struct.unpack('%ds'%lenp,payload)
                #self.msg.update({'UNKOWN_TYPE': s})

        elif msgid == UNKNOWN_FLOAT :
                d=struct.unpack('%df'%(lenp/4),payload)
##                a=(d)
##                for i in range((lenp/4)) :
##                    a[i] = (round(a[i],5))
##                print a
##                #self.msg.update({'UNKOWN_TYPE': s})
                #print round(d[0]*100,5),'\t',round(d[1]*100,5),'\t',round(d[2]*100,5),'\t',round(d[3]*100,5),'\t',round(d[4]*100,5),'\t',round(d[5]*100,5),'\t',round(d[6]*100,5),'\t',round(d[7]*100,5),'\t',round(d[8],5)
                
        elif msgid == GPS :
                #print 'get gps'
                gps = struct.unpack('hh',payload)
                #self.msg.update({'GPS': (gps[0]/(1e7),gps[1]/(1e7))})
                self.msg.update({'GPS': (gps[0],gps[1])})
                #print 'GPS lat', gps[0] ,' lon :' , gps[1]

                self.poseMsg.pose.pose.position.x = gps[1]  #CM
                self.poseMsg.pose.pose.position.y = gps[0]
        elif msgid == NAV_DATA :
                d=struct.unpack('hh',payload)
                self.msg.update({'NAV_DATA': d})
                self.pose_navMsg.pose.pose.position.x = d[0]  #CM
                self.pose_navMsg.pose.pose.position.y = d[1]
                #print 'nav x:', d[0] ,'nav y:' , d[1]

        elif msgid ==  CONT_COMPASS:
                compass = struct.unpack('hhhhhh',payload)
                self.CONT.update({'COMPASS': compass})
                print 'Update Compass constance..'
	
        elif msgid ==  CONT_PID:
                pid = struct.unpack('HHHHHHHHHHHH',payload)
                self.CONT.update({'PID': pid})
                print 'Update PID constance..'

    def takeoff(self,h) :
        return self.send_cmd_takeoff(h)
    
    def msg_print(self) :
        print '---------------------------------------------'
        print self.msg
        print '---------------------------------------------'

    def get_msg(self) :
        return self.msg

    def get_cont(self) :
        return self.CONT

    def getATT(self) :
        if 'ATTITUDE' in self.msg :
            return self.msg['ATTITUDE'] #roll,pitch,yaw
        else :
            return None
    def getALT(self) :
        if 'ALTITUDE' in self.msg :
            return self.msg['ALTITUDE'] #rel_alt,vel_z
        else :
            return None

    def getCNT_VAL(self) :
        if 'CNT_VAL' in self.msg :
            return self.msg['CNT_VAL'] #control roll pitch yaw
        else :
            return None
    def getRAW_RADIO(self) :
        if 'RAW_RADIO' in self.msg :
            return self.msg['RAW_RADIO'] #control roll pitch yaw
        else :
            return None
            
    def getGPS(self) :
        if 'GPS' in self.msg :
            return self.msg['GPS']
        else :
            return None


