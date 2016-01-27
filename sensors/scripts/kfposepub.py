#!/usr/bin/env python
__author__ = 'troy'

import rospy
import time
import tf
import gpsthread3
import imuthread3
from sensors.msg import kfpose
import ASV_KF
import numpy as np
import os
import Transformations
import math

def kfposePub():
    trans = Transformations.Tf()
    pub = rospy.Publisher('KFPose', kfpose, queue_size=1)
    # Initialize the node and name it.
    rospy.init_node('KFPosPub')
    print "kfposePub initialised"
    novatel = gpsthread3.Gpsthread(port1 = rospy.get_param("gps_port1"),
                                   port2 = rospy.get_param("gps_port2"),
                                   port3= rospy.get_param("gps_port3"))
    print "novatel initialised by kfposepub"
    vn = imuthread3.Imuthread(port=rospy.get_param("imu_port"), pause=0.005)
    print "imu initilised by kfposepub"
    novatel.start()
    vn.start()
    vn.set_qmr_mode()
    rospy.sleep(1)
    #vn.set_data_freq10()
    vn.set_data_freq50()
    KFrate = 50
    rospyKFRate = rospy.Rate(KFrate)  #10hz
    rospy.sleep(1)
    msg = kfpose()
    gpscount = 0
    count = 0
    imu_adj = float(rospy.get_param("IMU_Adj"))
    imu_adj_i = np.pi
    #note yaw = 0 when facing east

    #wait for GPS to lock on
    IMU_acc_adj = rospy.get_param("IMU_acc_adj") #x,y,z accel bias
    IMU_acc_var = rospy.get_param("IMU_acc_var") #x,y,z accel variance
    GPS_loc_var = rospy.get_param("GPS_loc_var") #x,y,z loc variance
    KF_model_var = rospy.get_param("KF_model_var")

    userHome = os.path.expanduser('~')
    KFDebugOut = open((userHome + '/tmp/KFDebug.txt'),'w')

    while novatel.sol_status < 0:
        print "novatel sync'ing solstatus, no. satellites", novatel.sol_status, novatel.satellites
        rospy.sleep(0.1) #check at 10hz
    #initalise XHat to current x,y,z position with zero velocity and acceleration
    XHAT_init = Xhat = np.zeros((9,1))
    XHAT_init[0,0]  = novatel.easting#x
    XHAT_init[3,0]  = novatel.northing#y
    XHAT_init[6,0]  = novatel.height#*(-1)#z
    print "XHAT_init", XHAT_init
    #initialise acceleration variance
    kf = ASV_KF.ASV_KF(XHAT_init,ri=IMU_acc_var, rg = GPS_loc_var, sigq = KF_model_var) #initial parameters and variance set at 0's and 1's respectively by default
    print "entering kf loop"
    while not rospy.is_shutdown():
        stamp = rospy.Time.now()
        if len(vn.lastreadings)>0: #we have IMU data
            if vn.lastreadings[0] =='VNQMR' and vn.lastreadings.shape[0]>13: #and it is valid
                # set quaternions
                stamp = rospy.Time.now()
                t = time.time()
                #these are in imu co-ordinate frame
                xddi = float(vn.lastreadings[8])- float(IMU_acc_adj[0])
                yddi = float(vn.lastreadings[9])- float(IMU_acc_adj[1])
                zddi = float(vn.lastreadings[10])- float(IMU_acc_adj[2])

                #these are in global co-ordinate frame
                q_x = vn.lastreadings[1]
                q_y = vn.lastreadings[2]
                q_z = vn.lastreadings[3]
                q_w = vn.lastreadings[4]
                current_pose_euler = tf.transformations.euler_from_quaternion([
                    q_x,q_y,q_z,q_w])

                yaw = current_pose_euler[2]
                yawadj = (yaw+imu_adj +np.pi)%(np.pi*2)-np.pi
                #print "yaw, yawadj, imuadj", yaw, yawadj, imu_adj
                rot1 = trans.rotz(yawadj)
                Al = np.array([[xddi, yddi, zddi]]).T
                A = np.dot(rot1,Al) #accelerations now in global frame
                roty = trans.roty(np.pi) #pitch 180 degrees
                A = np.dot(roty,A)
                xdd = A[0,0]
                ydd = A[1,0]
                zdd = A[2,0]

                kf.updateINS(np.array([[xdd],[ydd],[zdd]]),t)
                msg.easting = kf.kalmandt.Xhat[0,0]
                eastingVel = kf.kalmandt.Xhat[1,0]
                msg.northing = kf.kalmandt.Xhat[3,0]
                northingVel = kf.kalmandt.Xhat[4,0]
                msg.bodyFrameVel = np.sqrt(eastingVel*eastingVel + northingVel*northingVel)
                KFbodyHEading = math.atan2(northingVel, eastingVel) #Heading of the vessels momentum
                msg.height = kf.kalmandt.Xhat[6,0]
                msg.yaw = yawadj*(-1)#to get rotation direction AC as assumed by code
                #if msg.bodyFrameVel > 0.2: #Only print out when moving
                #    print "BodyHead, yaw", KFbodyHEading, msg.yaw
                msg.pitch = 0. #current_pose_euler[1] #not certain this is correct, not currently using it
                msg.roll = 0. #current_pose_euler[0] #not certain this is correct, not currently using it
                msg.header.seq = count
                msg.header.stamp = stamp
                msg.header.frame_id = 'kfpose'
                KFString ='kf,' + str(stamp) + ',' + str(msg.easting) + ',' + str(msg.northing) + ',' + str(msg.bodyFrameVel) + ',' + str(msg.yaw) + ',' + str(msg.pitch) + ',' + str(msg.roll) + '\n'
                KFDebugOut.write(KFString)
        if novatel.count > gpscount: #only publish when there is new info
            msg.no_satellites = novatel.satellites
            msg.sol_status = str(novatel.sol_status)
            if novatel.sol_status >= 0:
                #these are in global co-ordinate frame
                x = novatel.easting
                xvar = novatel.easting_sd**2
                y = novatel.northing
                yvar = novatel.northing_sd**2
                z = novatel.height#*(-1) #convert to z-down
                zvar = novatel.height_sd**2
                #print x, y, z, xvar, yvar, zvar
                kf.updateGPS(np.array([[x],[y],[z]]),np.array([[xvar],[yvar],[zvar]]))
                KFString = 'gps,' + str(stamp) + ',' + str(x) + ',' + str(y) + ',' + str(z) + ',' + str(xvar) + ',' + str(yvar) + ',' + str(zvar) + '\n'
                KFDebugOut.write(KFString)
            gpscount = novatel.count
        count += 1
        #print "inside KF", msg.easting, msg.northing, msg.height, msg.yaw, msg.pitch, msg.roll
        pub.publish(msg)
        rospyKFRate.sleep()
    vn.kill = True
    novatel.kill = True
    novatel.close()
    KFDebugOut.close()

if __name__ == '__main__':
    try:
        kfposePub()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()