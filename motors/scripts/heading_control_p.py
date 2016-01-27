#!/usr/bin/env python
#import roslib; roslib.load_manifest('motors') #used on old rosbuild to hack python modules into PYTHONPATH
import motorcontdaisy, math
import rospy
from motors.msg import motorcmd
from geometry_msgs.msg import Twist
import threading
import os
#import time #only for test code below

class Headingcontrolp():
    #will send updated commands to the motors each time a cmd_vel subscription
    #comes in. Only runs motors if heartbeat on parameter server is alive
    #(set by rostoserial.py)
    def __init__(self):   
        self.motors_attached = True
        self.mylock = threading.Lock()
        try:
            motor_port = rospy.get_param("motor_ttl")
        except: pass
        try:
            self.p_gain = rospy.get_param("p_gain")
        except:
            self.p_gain = 1.0
        try:
            self.motors_attached = rospy.get_param("motors_attached")
        except: pass
        try: 
            self.maxthrust = rospy.get_param("max_thrust")
            self.minthrust = (-1)*self.maxthrust
        except:
            self.maxthrust = 90.0
            self.minthrust = -90.0    
        #rospy.loginfo("p_gain = " + str(self.p_gain))
        if self.motors_attached == True:  
            #rospy.loginfo("inside motors attached loop")
            self.mc = motorcontdaisy.MotorContDaisy(port=motor_port)
        self.count = 0
        self.lin_x_a = 0
        self.ang_z_a = 0
        self.lin_x_m = 0
        self.ang_z_m = 0
        self.hb = 'alive'
        self.mode = 'manual'
        self.leak = 'dry'
        self.userHome = os.path.expanduser('~')
        self.MotorDebugOut = open((self.userHome + '/tmp/Motorcont.txt'),'w')


        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, 
                                            self.callback_cmd_vel, queue_size=1)
        self.sub_cmd_vel_man = rospy.Subscriber('cmd_vel_man', Twist, 
                                            self.callback_cmd_vel_man, queue_size=1)                                            
        self.pub = rospy.Publisher('MotorCont', motorcmd, queue_size=1)
        self.msg = motorcmd()

    def run_cmd_vel(self):           
        with self.mylock:
            self.mode = rospy.get_param("mode")
            self.leak = rospy.get_param("leak")
            self.hb = rospy.get_param("heartbeat")
        if self.hb == 'alive': 
            if self.leak == 'dry':
                if self.mode == 'auto':
                    self.run_speed(self.lin_x_a,self.ang_z_a)
                else:
                    #rospy.loginfo("manual velocity cmd sent")
                    self.run_speed(self.lin_x_m,self.ang_z_m)
            else:
                rospy.loginfo("leak")
                self.run_speed(0,0)
        else:
            rospy.loginfo("lost heartbeat")
            self.run_speed(0,0)
     
    def callback_cmd_vel(self,data):
        #print "auto motor", data.linear.x, data.angular.z, self.mode, self.leak, self.hb
        self.lin_x_a = data.linear.x
        self.ang_z_a = data.angular.z
        self.run_cmd_vel()
                                    
    def callback_cmd_vel_man(self,data): 
        self.lin_x_m = data.linear.x
        self.ang_z_m = data.angular.z
        self.run_cmd_vel()
        
    def run_speed(self,speed,steer):
        #port_power, star_power = self.bestSteerConstThrust(speed,steer)
        port_power, star_power = self.bestSteerVarThrust(speed,steer)       
        #rospy.loginfo("port power = " + str(port_power) + " star power = " + str(star_power))
        if self.motors_attached == True:         
            #rospy.loginfo("portpow %s, starpow %s", port_power, star_power)
            self.mc.throttlePort(port_power)
            self.mc.throttleStar(star_power)
        mtcmdstr = str(rospy.Time.now()) + ',' + str(speed) + ',' + str(steer) + ',' + str(port_power) + ',' + str(star_power)
        self.MotorDebugOut.write(mtcmdstr)
        #publish motor comands for debugging purposes
        self.msg.header.seq = self.count
        self.msg.header.stamp = rospy.Time.now()
        self.msg.speed = speed
        self.msg.portpower = port_power
        self.msg.starpower = star_power
        self.pub.publish(self.msg)
        self.count += 1        
    
    def bestSteerVarThrust(self, speed, steer):
        #speed is percentage between -100 and 100
        #steer is heading adjust in radians
        #allow average thrust to be below directed thrust
        #pgain should be somewhere between 2 and 3
        # higher pgains lead to avg thrust dropping below directed speed sooner
        # directed speeds closer to 100 are impacted earlier
        # with speed 50 and pgain 3, avg thrust droops below directed at approx
        # .52 radians or 30 degrees steer angle. thrust drops to zero at around 
        # 1.57 radians or 90 degrees
        heading_chg = steer/math.pi
        speed = self.clamp(self.minthrust, self.maxthrust, speed)
        steering_range = 100 #this is where it happens. then clamped later
        steering_dif = self.p_gain*heading_chg*steering_range    
        port_power = self.clamp(-100, 100, speed - steering_dif)
        star_power = self.clamp(-100, 100, speed + steering_dif)  
        self.msg.steerdif =  steering_dif
        self.msg.headingerr = heading_chg                          
        return port_power, star_power
 
    def clamp(self, minimum, maximum, value):
        return max(minimum,min(value, maximum))
                   
    def close(self):
        self.run_speed(0,0)
        #try:
        self.mc.close()
        self.MotorDebugOut.close()
        #except: pass

if __name__ == '__main__':
  rospy.init_node('heading_control')
  Headingcontrolp()
  rospy.spin()
        

