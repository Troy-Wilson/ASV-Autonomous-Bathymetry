#!/usr/bin/env python
"""
Created on Thu Feb  6 11:39:47 2014

@author: troy
"""
import sys, httplib
import xbeetrans
#import roslib; roslib.load_manifest('xbee')
import rospy
import tf
from sensors.msg import gpsne, Leak, Battery, Temp, SBS, kfpose
from navigation.msg import navgoalActionGoal, navgoalActionFeedback#, navgoalActionResult
from actionlib_msgs.msg import GoalStatusArray 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from motors.msg import motorcmd
from navigation.msg import waypoints, navStatus, NextWayPointControl,DebugString
import threading
import time
import math 

#import math

class RosToSerial():
    def __init__(self,port):
        self.xbee = xbeetrans.XbeeTrans(port)        
        self.cvmsg = Twist()
        self.thrust = 0
        self.steer = 0
        self.mylock = threading.Lock()
        self.heartmonitor = time.time()      
        self.imuclock = time.time()      
        self.cvclock = time.time()      
        self.mtrclock = time.time() 
        self.nadj = 0
        self.eadj = 0             
        with self.mylock:        
            rospy.set_param('heartbeat','alive')
        try: 
            self.imu_adj = float(rospy.get_param("IMU_Adj"))
        except:
            self.imu_adj = 3.09
        try:
            self.simulation = rospy.get_param("sim_status")
        except:
            self.simulation = False



        #self.sub_gpsne = rospy.Subscriber('GPSDataNE',gpsne,self.callback_gps, queue_size=1)
        #self.sub_imu = rospy.Subscriber('IMUData',Imu,self.callback_imu, queue_size=1)
        self.sub_kfpose = rospy.Subscriber('KFPose',kfpose,self.callback_kfpose, queue_size=1)
        self.sub_mot = rospy.Subscriber('MotorCont',motorcmd,self.callback_mtr, queue_size=1)
        self.sub_cv = rospy.Subscriber('cmd_vel', Twist,self.callback_cv, queue_size=1)
        self.sub_path = rospy.Subscriber('path', waypoints,self.callback_path, queue_size=1)
        self.sub_navstatus = rospy.Subscriber('NavStatus', navStatus,self.callback_navStatus, queue_size=1)
        # self.sub_goal = rospy.Subscriber('/nav_ms/goal',
        #                                  navgoalActionGoal,self.callback_goal, queue_size=1)
        # self.sub_goal = rospy.Subscriber('/nav_ms/feedback',
        #                                  navgoalActionFeedback,
        #                                  self.callback_feedback, queue_size=1)
#       self.sub_goal = rospy.Subscriber('/nav_ms/result',
#                                         navgoalActionResult,
#                                         self.callback_result)
#         self.sub_goal = rospy.Subscriber('/nav_ms/status',
#                                          GoalStatusArray,self.callback_status, queue_size=1)
        self.sub_batt =  rospy.Subscriber('battData',Battery,self.callback_battery, queue_size=1)
        #self.sub_temp =  rospy.Subscriber('temp',temp,self.callback_temp)
        self.sub_leak = rospy.Subscriber('leakData',Leak,self.callback_leak, queue_size=1)
        self.sub_temp = rospy.Subscriber('Temperature',Temp,self.callback_temp, queue_size=1)
        self.sub_sbs = rospy.Subscriber('SBSData',SBS, self.callback_sbs, queue_size=1)
        self.sub_Debugstr = rospy.Subscriber('debugstr', DebugString,self.callback_debug, queue_size = 1)

        self.sub_NextWayPoint = rospy.Subscriber('NextWPCont', NextWayPointControl, self.callback_nwp, queue_size=1)

        self.pubcv = rospy.Publisher('cmd_vel_man', Twist, queue_size=1)

        t = threading.Timer(1, self.monitorrx)
        t.start()
        # This pause to is make sure that everything has stared up before the poly boundaries are tranmitted as they only
        # go once, so the other side needs to be listening
        time.sleep(1)
        self.transmit_polyBoundary()
        
    def monitorrx(self):
       # try:            
        data = self.xbee.read_data()

        if data != None:
            #rospy.loginfo("xbeeread " + str(data[0]) + " " + str(data[1]))
            if data[0] == 'md':
                try:                
                    #rospy.loginfo("received change of mode")
                    self.thrust = 0
                    self.steer = 0
                    with self.mylock:
                        if data[1]=='a':
                            rospy.set_param('mode','auto')
                        else:
                            rospy.set_param('mode','manual')
                        #print rospy.get_param('mode')
                except:
                    rospy.loginfo("issue in mode change in rostoserial.py")

            elif data[0] == 'hb':
                try:
                    beattime = time.time()-self.heartmonitor
                    #rospy.loginfo("beattime" + str(beattime))
                    if beattime > 2:
                        with self.mylock:
                            rospy.set_param('heartbeat','dead')
                                #rospy.loginfo("dead heartbeat")
                    else:
                        with self.mylock:
                            rospy.set_param('heartbeat','alive')                                  
                                #rospy.loginfo("live heartbeat")
                    self.heartmonitor = time.time()
                except httplib.CannotSendRequest:
                    pass #ignore param update problems. failing on multiple fast updates even with locking
                except Exception as e:
                    rospy.loginfo("hb status update failure, prob due to lock conflict") 
                    exc_type, exc_val, exc_tb = sys.exc_info()
                    rospy.loginfo(str(exc_type) + str(exc_val) + str(exc_tb))                           
            elif data[0] == 's':                
                #rospy.loginfo(data)                    
                try:                
                    #self.steer = int(data[1])/180.0*math.pi  #convert to radians
                    self.steer = float(data[1])                    
                    self.cvmsg.linear.x = self.thrust
                    self.cvmsg.angular.z = self.steer
                    #rospy.loginfo("steer" + str(self.steer))
                    self.pubcv.publish(self.cvmsg)       
                except Exception as e: 
                    rospy.loginfo("error on steering data read in rostoserial.py")
                    rospy.loginfo(e)
                    rospy.loginfo(data)
            elif data[0] == 't':
                try:                        
                    #rospy.loginfo(data)                    
                    self.thrust = int(data[1])
                    self.cvmsg.linear.x = self.thrust
                    self.cvmsg.angular.z = self.steer
                    #rospy.loginfo("thrust" + str(self.thrust))
                    self.pubcv.publish(self.cvmsg)
                except:
                    rospy.loginfo("error on thrust data read in rostoserial.py")
        else:
            if time.time() - self.heartmonitor > 2:
                with self.mylock:
                            rospy.set_param('heartbeat','dead')
        t = threading.Timer(0.01, self.monitorrx)
        t.start() #recursively calls itself continuously      
       # except Exception as e:
#            pass
      #      rospy.loginfo("exception in monitorrx in rostoserial, caught line 115")
#            exc_type, exc_val, exc_tb = sys.exc_info()
#            rospy.loginfo(str(exc_type) + str(exc_val) + str(exc_tb)) 
#            rospy.loginfo(data)
#            self.cvmsg.linear.x = 0
#            self.cvmsg.angular.z = 0
#            self.pubcv.publish(self.cvmsg)
#            with self.mylock:
#                rospy.set_param('mode','manual')
            
    # def callback_gps(self,data):
    #     try:
    #         self.nadj = rospy.get_param("northing_home")
    #         self.eadj = rospy.get_param("easting_home")
    #     except:
    #         pass
    #     n = round(float(data.northing)-float(self.nadj),3)
    #     e = round(float(data.easting)-float(self.eadj),3)
    #     gpsstring = str(n) + " " + str(e)
    #     #rospy.loginfo(gpsstring + " " + str(data.northing) + " " + str(data.easting))
    #     #gpsstring = str(data.northing-nadj) + " " + str(data.easting-eadj)
    #     gpsstatstr = (str(data.no_satellites) + " " + str(data.sol_status))
    #     navlooping = False
    #     try:
    #         navlooping = rospy.get_param("navlooping")
    #     except:
    #         pass
    #     if navlooping == True:
    #         self.xbee.transmit(("c",gpsstring))
    #     self.xbee.transmit(("st", gpsstatstr))
    #
    # def callback_imu(self,data):
    #     tmptime = time.time()
    #     if tmptime - self.imuclock >0.5:
    #         imustring =( str(data.angular_velocity.y) + " " + str(data.angular_velocity.x))
    #         self.xbee.transmit(("v",imustring))
    #         current_pose_euler = tf.transformations.euler_from_quaternion([
    #                                     data.orientation.x,
    #                                     data.orientation.y,
    #                                     data.orientation.z,
    #                                     data.orientation.w
    #                                     ])
    #         current_heading = self.wrapAngle(current_pose_euler[2] + self.imu_adj) #get yaw
    #         self.xbee.transmit(("h",round(current_heading,2)))
    #         self.imuclock = time.time()
    #     #man_auto = rospy.get_param("mode")
    #     #rospy.loginfo(man_auto)

    def callback_kfpose(self,data):
        tmptime = time.time()
        if tmptime - self.imuclock >1.0:
            self.xbee.transmit(("h",round(data.yaw,2)))
            n = round(float(data.northing),1)
            e = round(float(data.easting),1)
            gpsstring = str(e) + " " + str(n)
            gpsstatstr = (str(data.no_satellites) + " " + str(data.sol_status))
            velstring = round(float(data.bodyFrameVel),2)
            #print "in callback_kfpose n,e ", n, e
            self.xbee.transmit(("c",gpsstring))
            self.xbee.transmit(("st", gpsstatstr))
            self.xbee.transmit(("v",velstring))
            self.imuclock = time.time()

    def callback_debug(self,data):
        self.xbee.transmit(("db",data.debug))

    def callback_mtr(self,data):
        tmptime = time.time()
        if tmptime - self.mtrclock >0.5:
            thruststring = (str(round(data.portpower,2)) + " " + str(round(data.starpower,2)))
            self.xbee.transmit(("m",thruststring))
            self.mtcclock = time.time()
        
    def callback_cv(self,data):
        tmptime = time.time()
        if tmptime - self.cvclock >0.5:        
            self.xbee.transmit(("t",round(data.linear.x),2))
            self.xbee.transmit(("s",round(data.angular.z,2)))
            self.cvclock = time.time()

    def callback_nwp(self, data):
        #publish next waypoint
        waypntstr = (str(round(data.easting,2)) + " " + str(round(data.northing,2)) + " " +
        str(round(data.desHead,2)) + " " +
        str(round(data.headingAdj,2)) + " " +
        str(round(data.euclidDist,2)))

        self.xbee.transmit(("w",waypntstr))
        if self.simulation == True:
            print "nwp string", waypntstr

#-----------------------
           
#     def callback_feedback(self, data):
#         #triggered as each goal reached
#         #rospy.loginfo("feedback triggered")
#         #rospy.loginfo(data)
#         waypntstr =(str(data.feedback.currentWaypoint.y) +
#                     " " + str(data.feedback.currentWaypoint.x))
#         self.xbee.transmit(("w",waypntstr))
#         self.xbee.transmit(("wc",data.feedback.WaypointsReached))
#
#     def callback_goal(self, data):
#         #called when goal set
#         #need to loop through and get goal coordinates out and into a string
#         #maybe in format "x:4.5 y:5.6 x:7.6 y:8.4"
#         #rospy.loginfo(data)
#         rospy.loginfo("callback_goal")
#         #rospy.loginfo(data.goal.navgoal.poses[0])
#         #rospy.loginfo(data.goal.navgoal.poses[1])
#         #rospy.loginfo(data.goal.navgoal.poses[2])
#         waypnts = ""
#         for i in data.goal.navgoal.poses:
#             waypnts = waypnts + "y:" + str(i.pose.position.y)
#             waypnts = waypnts + " x:" + str(i.pose.position.x) + " "
#         rospy.loginfo("waypoints - rostoserial")
#         rospy.loginfo(waypnts)
#         self.xbee.transmit(("gw",waypnts))
#
# #    def callback_result(self,data):
#         #called when action server teminates, returns distance from goal
#         #rospy.loginfo("in result" + str(data.result))
#         #rospy.loginfo(data)
# #        self.xbee.transmit(("goalresult",data.result))
#
#     def callback_status(self, data):
#         #called each loop of the action server, returns goal status
#         try:
#             #rospy.loginfo(data.status_list[0].status)
#             self.xbee.transmit(("go",data.status_list[0].status))
#         except: pass

    def callback_path(self, data):
        #called when lawnmower path created
        #need to loop through and get goal coordinates out and into a string

        for i in range(len(data.easting)):
            waypnts = str(round(data.easting[i],1)) + " " + str(round(data.northing[i],1))
            self.xbee.transmit(("gw",waypnts))

        # print "msg read by callback_path inside rostoserial.py", data
        # waypnts = ""
        # for i in range(len(data.easting)):
        #     waypnts = waypnts + "y:" + str(round(data.northing[i],1))
        #     waypnts = waypnts + " x:" + str(round(data.easting[i],1)) + " "
        # rospy.loginfo("waypoints - rostoserial")
        # rospy.loginfo(waypnts)
        # print waypnts
        # self.xbee.transmit(("gw",waypnts))

    def transmit_polyBoundary(self):
        polyboundaries_E = rospy.get_param("poly_boundaries_E")
        polyboundaries_N = rospy.get_param("poly_boundaries_N")
        for i in range(len(polyboundaries_E)):
            polypnts = str(round(polyboundaries_E[i],1)) + " " + str(round(polyboundaries_N[i],1))
            self.xbee.transmit(("pb",polypnts))


    def callback_navStatus(self,data):
        #waypntstr =(str(round(data.wayPtEast,2)) +
        #             " " + str(round(data.wayPtNorth,2)))
        #self.xbee.transmit(("w",waypntstr))
        self.xbee.transmit(("go",data.goalStatus))
#------------------------
        
    def callback_battery(self,data):
        self.xbee.transmit(("z1", str(data.SMinToEmpty)))
        self.xbee.transmit(("z2", str(data.SPerCharge)))
        self.xbee.transmit(("y1",str(round(data.B1Voltage,2))))
        self.xbee.transmit(("x1",str(round(data.B2Voltage,2))))
        self.xbee.transmit(("y2",str(round(data.B1Current,2))))
        self.xbee.transmit(("x2",str(round(data.B2Current,2))))
        self.xbee.transmit(("y3",str(round(data.B1RemainingCapacity,2))))
        self.xbee.transmit(("x3",str(round(data.B2RemainingCapacity,2))))
        self.xbee.transmit(("y4",str(round(data.B1Temp,2))))
        self.xbee.transmit(("x4",str(round(data.B2Temp,2))))
        self.xbee.transmit(("y5",str(data.B1AvgTimeToEmpty)))
        self.xbee.transmit(("x5",str(data.B2AvgTimeToEmpty)))
        self.xbee.transmit(("y6",data.B1Status))
        self.xbee.transmit(("x6",data.B2Status))        
                        
#        
#    def callback_temp(self,data):
#        self.xbee.transmit(("temp",data.temp))
#        
    def callback_leak(self,data):
        #rospy.loginfo('in leak msg')
        leakmsg = data.status + " " + str(data.val)
        self.xbee.transmit(("l",leakmsg))

    def callback_temp(self,data):
        #rospy.loginfo('in temp msg')
        self.xbee.transmit(("t1",str(round(data.degrees,1))))

    def callback_sbs(self,data):
        #self.xbee.transmit(("wt",str(round(data.waterTemp,1))))
        self.xbee.transmit(("sb",str(round(data.range,2))))

        
    def wrapAngle(self, angle):
    #wrap into -pi to pi space
        angle = (angle+math.pi)%(math.pi*2)-math.pi
        return angle   

      
if __name__ == '__main__':
  rospy.init_node('heading_control')
  RosToSerial(rospy.get_param("xbee_tty"))
  rospy.spin()    
    

    
