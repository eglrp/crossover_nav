#!/usr/bin/env python
import serial,thread,time,struct
from ctypes import *

import rospy
from std_msgs.msg import String
# Input messages type
from sensor_msgs.msg import Imu ,MagneticField , Joy , Temperature, FluidPressure, NavSatFix, Range
from geometry_msgs.msg import PoseWithCovarianceStamped,  PoseStamped,TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from crossover_nav.msg import Ack , Navdata

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
COMPASS         = 19
GPS_RAW_FIX     = 20
ACK             = 21
POSE_DEMAND     = 22
VISION_DATA     = 23
SONAR_DATA      = 24
FLOW_DATA       = 25
SONAR_DATA2_O   = 26

GCONFIG     =100 
PID_RPY1    =101 
PID_RPY2    =102 
PID_ALT     =103 
PID_NAV     =104 
PID_SPD     =105 
NAV_INER    =106 
MODE_CFG    =107 

def STABILIZE_MODE():
    return "STABILIZE_MODE"
def ALT_MODE():
    return "ALT_MODE"
def OTHER_MODE():
    return "OTHER_MODE"
def TAKE_OFF():
    return "TAKE_OFF"
def VEL_HOLD():
    return "VEL_HOLD"
def GUIDE_MODE():
    return "GUIDE_MODE"

modes = {0 : STABILIZE_MODE,
           1 : ALT_MODE,
           2 : OTHER_MODE,
           3 : TAKE_OFF,
           4 : VEL_HOLD,
           5 : GUIDE_MODE,
           }

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

baro_old = 0
temp_old = 0

class Ulink:
    msg=dict()
    CONT=dict()
    imuMsg = Imu()
    poseMsg = PoseWithCovarianceStamped()
    pose_navMsg = Odometry()
    compassMsg = MagneticField()
    remoteMsg = Joy()
    baroMsg = FluidPressure()
    tempMsg = Temperature()
    gpsrawMsg = NavSatFix()
    navvelMsg = TwistWithCovarianceStamped()
    ackMsg = Ack()
    sonarMsg = Range()
    NavMsg   = Navdata()
    #rospy.init_node('imu_node2', anonymous=True)
    time_start=rospy.Time.now()
    imuMsg.orientation_covariance = [1 , 0 , 0, 0, 1, 0, 0, 0, 1]
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
            #print "sending data request"
            #self.send_data_request(self.s,DATA_ATT)
            #self.send_data_request(self.s,DATA_GPS)
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
                #print ord(ch)
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
         head = struct.pack('ccc',chr(181),chr(dlen),chr(POSE_DEMAND))
         data = struct.pack('ffff',px,py,pz,yaw)
         summ= 181+dlen+POSE_DEMAND
         for d in str(data):
             summ+=ord(d)
         chk = struct.pack('h',summ)
         tatal = len(head)+len(data)+len(chk)
         self.s.write( head )
         self.s.write( data )
         self.s.write( chk )
         return tatal
    def send_flow(self,fx,fy,track) :
        dlen = 10
        head = struct.pack('ccc',chr(181),chr(dlen),chr(FLOW_DATA))
        data = struct.pack('ffH',fx,fy,track)
        summ= 181+dlen+FLOW_DATA
        for d in str(data):
            summ+=ord(d)
        chk = struct.pack('h',summ)
        tatal = len(head)+len(data)+len(chk)
        self.s.write( head )
        self.s.write( data )
        self.s.write( chk )
        return tatal
    def send_motor_desire(self,M1,M2,M3,M4) :
         dlen = 16
         head = struct.pack('ccc',chr(181),chr(dlen),chr(MOTOR_OUT))
         data = struct.pack('ffff',M1,M2,M3,M4)
         summ= 181+dlen+MOTOR_OUT
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
        t_now = rospy.Time.now()
        if msgid == ATTITUDE :
                #print 'id : ' ,idp
                d=struct.unpack('hhh',payload)
                #d=struct.unpack('iii',payload)
                self.msg.update({'ATTITUDE': d})

                self.NavMsg.tm   = 1000000*(t_now.secs-self.time_start.secs)+(t_now.nsecs-self.time_start.nsecs)/1000
                self.NavMsg.rotX = d[0]*0.0572957;
                self.NavMsg.rotY = d[1]*0.0572957;
                self.NavMsg.rotZ = d[2]*0.0572957;

                q = tf.transformations.quaternion_from_euler(d[0]*0.001,d[1]*0.001,d[2]*0.001)  #roll pitch yaw   (addition for use in rviz +p/2)
                self.imuMsg.header.stamp = t_now#-self.time_start
                self.imuMsg.orientation.x=q[0]
                self.imuMsg.orientation.y=q[1]
                self.imuMsg.orientation.z=q[2]
                self.imuMsg.orientation.w=q[3]
                
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
                d=struct.unpack('hhhhhh',payload)
                self.msg.update({'RAW_IMU': d})
                self.imuMsg.header.stamp = t_now
                self.imuMsg.linear_acceleration.x = d[0]*9.80655/1000.0
                self.imuMsg.linear_acceleration.y = d[1]*9.80655/1000.0
                self.imuMsg.linear_acceleration.z = d[2]*9.80655/1000.0
                self.imuMsg.angular_velocity.x = d[3]/1000.0
                self.imuMsg.angular_velocity.y = d[4]/1000.0
                self.imuMsg.angular_velocity.z = d[5]/1000.0
                self.compassMsg.header.stamp = t_now#-self.time_start
                # self.compassMsg.magnetic_field.x = d[6]
                # self.compassMsg.magnetic_field.y = d[7]
                # self.compassMsg.magnetic_field.z = d[8]
                
                self.NavMsg.ax = d[0]/1000.00;
                self.NavMsg.ay = d[1]/1000.00;
                self.NavMsg.az = d[2]/1000.00;
        
        elif msgid == V_VAL :
                d=struct.unpack('hh',payload)  #v_est_x v_est_y
                self.msg.update({'V_VAL': d})
                self.navvelMsg.twist.twist.linear.x = d[0]*0.01
                self.navvelMsg.twist.twist.linear.y = d[1]*0.01
                #print 'r1:', str(d[0])[:4] ,'r2:' , str(d[1])[:4] 

        elif msgid == RAW_BARO :
                d=struct.unpack('fff',payload)  #alt_bybaro , pressure , temperature
                global baro_old
                global temp_old
                self.msg.update({'RAW_BARO': d})
                if baro_old == 0 or temp_old ==0:
                    baro_old = d[1]
                    temp_old = d[2]
                else :
                    if abs(d[1]-baro_old)>2 or abs(d[2]-temp_old)>1:
                        self.poseMsg.pose.pose.position.x=1+self.poseMsg.pose.pose.position.x
                
                    else :
                        self.baroMsg.fluid_pressure = d[1]
                        self.tempMsg.temperature = d[2]
                        baro_old = d[1]
                        temp_old = d[2]
                  
        
                #print 'BARODATA : ', str(d[1])[:7] ,' temp :' , str(d[2])[:7] ,' alt :' , str(d[0])[:7]
                
        elif msgid == RAW_RADIO :
                d=struct.unpack('HHHHHH',payload)
                self.msg.update({'RAW_RADIO': d})
                self.remoteMsg.header.stamp = t_now#-self.time_start
                # self.remoteMsg.pose.position.x = d[0]
                # self.remoteMsg.pose.position.y = d[1]
                # self.remoteMsg.pose.position.z = d[2]
                # self.remoteMsg.pose.orientation.x = d[3]
                # self.remoteMsg.pose.orientation.y = d[4]
                # self.remoteMsg.pose.orientation.z = d[5]
                self.remoteMsg.axes= d[0:5]
                #self.send_motor_desire(d[2]/8,d[2]/8,d[2]/8,d[2]/8)
                #print 'r1:', str(d[0])[:4] ,'r2:' , str(d[1])[:4] ,'r3:' , str(d[2])[:4] ,'r4:' , str(d[3])[:4] ,'r5:' , str(d[4])[:4] ,'r6:' , str(d[5])[:4]
                        
        elif msgid == MOTOR_OUT :
                d=struct.unpack('hhhh',payload)
                self.msg.update({'MOTOR_OUT': d})

        elif msgid == CNT_VAL :
                d=struct.unpack('fff',payload)
                self.msg.update({'CNT_VAL': d})
                # print 'CONTROL  ROLL : ', str(d[0])[:4] ,' PITCH :' , str(d[1])[:4] ,' YAW :' , str(d[2])[:4]
        elif msgid == SONAR_DATA2_O :
                d=struct.unpack('f',payload)
                self.msg.update({'SONAR_DATA2_O': d})
                self.sonarMsg.range = d[0]
                print 'SONAR : ', str(d[0])[:4]      
        elif msgid == STATUS_CHK :
                d=struct.unpack('hhhh',payload)
                self.msg.update({'STATUS_CHK': d})
                print 'NUM SEN : ', str(d[0]) , '--ARM : ', bool(d[1]) , '--MODE : ' ,modes[d[2]](), '--RC : ' ,d[3]

                
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
                #print round(d[0]*100,5),'\t',round(d[1]*100,5),'\t',round(d[2]*100,5),'\t',round(d[3]*100,5)
        elif msgid == GPS :
                #print 'get gps'
                gps = struct.unpack('ff',payload)
                #self.msg.update({'GPS': (gps[0]/(1e7),gps[1]/(1e7))})
                self.msg.update({'GPS': (gps[0],gps[1])})
                #print 'GPS lat', gps[0] ,' lon :' , gps[1]
                self.poseMsg.header.stamp = t_now#-self.time_start
                self.poseMsg.pose.pose.position.x = gps[1]/100  #to m
                self.poseMsg.pose.pose.position.y = gps[0]/100
        elif msgid == GPS_RAW_FIX :  #lat,lng , numsat, hacc ,alt,fix_type,vacc
                #print 'get gps'
                gps_raw = struct.unpack('iiHHiHH',payload) 
                #self.msg.update({'GPS': (gps[0]/(1e7),gps[1]/(1e7))})
                self.msg.update({'GPS_RAW_FIX': (gps_raw[0],gps_raw[1],gps_raw[2],gps_raw[3],gps_raw[4],gps_raw[5],gps_raw[6])})  #lat lon numsat hacc alt fix vacc
                #print 'GPS lat', gps[0] ,' lon :' , gps[1]
                self.gpsrawMsg.header.stamp = t_now#-self.time_start
                self.gpsrawMsg.latitude = gps_raw[0]/10000000.00000000  #deg
                self.gpsrawMsg.longitude = gps_raw[1]/10000000.00000000
                self.gpsrawMsg.status.status = -1 if gps_raw[5] == 0 else 1
                self.gpsrawMsg.position_covariance=[gps_raw[3],0,0,0,gps_raw[3],0,0,0,gps_raw[6]]
                self.poseMsg.pose.covariance=[gps_raw[3], 0, 0, 0, 0, 0,
                                                0, gps_raw[3],0, 0, 0, 0,
                                                0, 0, gps_raw[6],0, 0, 0,
                                                0, 0, 0, 0.01, 0, 0,
                                                0, 0, 0, 0, 0.01, 0,
                                                0, 0, 0, 0, 0, 0.1]
                self.gpsrawMsg.status.service = gps_raw[2]
                self.gpsrawMsg.altitude = gps_raw[4]/1000.00000000
                self.gpsrawMsg.position_covariance_type = 2;

        elif msgid == NAV_DATA :
                d=struct.unpack('ff',payload)
                self.msg.update({'NAV_DATA': d})
                self.pose_navMsg.pose.pose.position.x = d[0]  #to m
                self.pose_navMsg.pose.pose.position.y = d[1]
                #print 'nav x:', d[0] ,'nav y:' , d[1]
        elif msgid == ALTITUDE :
                d=struct.unpack('fh',payload)
                self.msg.update({'ALTITUDE': d})
                self.poseMsg.pose.pose.position.z = d[0]
                self.pose_navMsg.pose.pose.position.z = d[0]
                self.navvelMsg.header.stamp = t_now#-self.time_start
                self.navvelMsg.twist.twist.linear.z = d[1]/100.000  #to m/s

                self.NavMsg.altd = int(d[0]*1000);  #report in mm
                # print 'ALT  rel_ALT : ', str(d[0]*100)[:4] ,' vel_z :' , str(d[1]*10)[:4]
        elif msgid ==  CONT_COMPASS:
                compass = struct.unpack('hhhhhh',payload)
                self.CONT.update({'COMPASS': compass})
                print 'Update Compass constance..'
    
        elif msgid ==  CONT_PID:
                pid = struct.unpack('HHHHHHHHHHHH',payload)
                self.CONT.update({'PID': pid})
                print 'Update PID constance..'

        elif msgid == ACK :
                d = struct.unpack('H',payload)[0]
                ack=[]
                for i in range(16) :
                    ack.append( bool(d  & (0x0001 << i)) )
                print ack
                self.ackMsg.ACK_PENDING  =   ack[0]
                self.ackMsg.ACK_GCONFIG  =   ack[1]
                self.ackMsg.ACK_PID_RPY1 =   ack[2]
                self.ackMsg.ACK_PID_RPY2 =   ack[3]
                self.ackMsg.ACK_PID_ALT  =   ack[4]
                self.ackMsg.ACK_PID_NAV  =   ack[5]
                self.ackMsg.ACK_SPD      =   ack[6]
                self.ackMsg.ACK_NAV_INER =   ack[7]
                self.ackMsg.ACK_MODE     =   ack[8]
                #del self.ack[:]
                print "....."

    def takeoff(self,h) :
        return self.send_cmd_takeoff(h)
    
    def msg_print(self) :
        print '---------------------------------------------'
        print self.msg
        print '---------------------------------------------'

    def send_pid1_param(self,data) : #(6 float)
         dlen = 24
         head = struct.pack('ccc',chr(181),chr(dlen),chr(PID_RPY1))
         data = struct.pack('ffffff',data[0],data[1],data[2],data[3],data[4],data[5])
         summ= 181+dlen+PID_RPY1
         for d in str(data):
             summ+=ord(d)
         chk = struct.pack('h',summ)
         tatal = len(head)+len(data)+len(chk)
         self.s.write( head )
         self.s.write( data )
         self.s.write( chk )
         return tatal
    def send_pid2_param(self,data) : #(6 float)
         dlen = 24
         head = struct.pack('ccc',chr(181),chr(dlen),chr(PID_RPY2))
         data = struct.pack('ffffff',data[0],data[1],data[2],data[3],data[4],data[5])
         summ= 181+dlen+PID_RPY2
         for d in str(data):
             summ+=ord(d)
         chk = struct.pack('h',summ)
         tatal = len(head)+len(data)+len(chk)
         self.s.write( head )
         self.s.write( data )
         self.s.write( chk )
         return tatal
    def send_pid_alt_param(self,data) : #(5 float)
         dlen = 20
         head = struct.pack('ccc',chr(181),chr(dlen),chr(PID_ALT))
         data = struct.pack('fffff',data[0],data[1],data[2],data[3],data[4])
         summ= 181+dlen+PID_ALT
         for d in str(data):
             summ+=ord(d)
         chk = struct.pack('h',summ)
         tatal = len(head)+len(data)+len(chk)
         self.s.write( head )
         self.s.write( data )
         self.s.write( chk )
         return tatal
    def send_pid_nav_param(self,data) : #(5 float)
         dlen = 20
         head = struct.pack('ccc',chr(181),chr(dlen),chr(PID_NAV))
         data = struct.pack('fffff',data[0],data[1],data[2],data[3],data[4])
         summ= 181+dlen+PID_NAV
         for d in str(data):
             summ+=ord(d)
         chk = struct.pack('h',summ)
         tatal = len(head)+len(data)+len(chk)
         self.s.write( head )
         self.s.write( data )
         self.s.write( chk )
         return tatal
    def send_spd_param(self,data) : #(2 float)
         dlen = 8
         head = struct.pack('ccc',chr(181),chr(dlen),chr(PID_SPD))
         data = struct.pack('ff',data[0],data[1])
         summ= 181+dlen+PID_SPD
         for d in str(data):
             summ+=ord(d)
         chk = struct.pack('h',summ)
         tatal = len(head)+len(data)+len(chk)
         self.s.write( head )
         self.s.write( data )
         self.s.write( chk )
         return tatal
    def send_nav_iner_param(self,data) : #(5 float)
         dlen = 20
         head = struct.pack('ccc',chr(181),chr(dlen),chr(NAV_INER))
         data = struct.pack('fffff',data[0],data[1],data[2],data[3],data[4])
         summ= 181+dlen+NAV_INER
         for d in str(data):
             summ+=ord(d)
         chk = struct.pack('h',summ)
         tatal = len(head)+len(data)+len(chk)
         self.s.write( head )
         self.s.write( data )
         self.s.write( chk )
         return tatal
    def send_mode_param(self,data) : #(6 float)
         dlen = 24
         head = struct.pack('ccc',chr(181),chr(dlen),chr(MODE_CFG))
         data = struct.pack('ffffff',data[0],data[1],data[2],data[3],data[4],data[5])
         summ= 181+dlen+MODE_CFG
         for d in str(data):
             summ+=ord(d)
         chk = struct.pack('h',summ)
         tatal = len(head)+len(data)+len(chk)
         self.s.write( head )
         self.s.write( data )
         self.s.write( chk )
         return tatal
    def send_vision_data(self,datav) : #(3 float 1 uint)
         dlen = 14
         head = struct.pack('ccc',chr(181),chr(dlen),chr(VISION_DATA))
         data = struct.pack('fffH',datav[0],datav[1],datav[2],datav[3])
         summ= 181+dlen+VISION_DATA
         for d in str(data):
             summ+=ord(d)
         chk = struct.pack('h',summ)
         tatal = len(head)+len(data)+len(chk)
         self.s.write( head )
         self.s.write( data )
         self.s.write( chk )
         print 'p x:', datav[0] ,'p y:' , datav[1]
         return tatal
