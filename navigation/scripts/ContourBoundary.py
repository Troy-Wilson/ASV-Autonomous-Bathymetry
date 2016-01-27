#!/usr/bin/env python
from __future__ import division
__author__ = 'troy'

import rospy
from geometry_msgs.msg import Twist
import Bathymetry
from sensors.msg import kfpose, SBS
from navigation.msg import waypoints, navStatus, NextWayPointControl, DebugString, bath, hyperSE
import math
import numpy as np
import Controllers
import curvefit
import DMPP
import TSP
import SingleBeam
import Transformations as TF
import Lines
import os.path
import time
import OnlineGP
import threading

class contourBoundary():
    def __init__(self):
        rospy.init_node('ContourBoundary')
        try:
            self.maxThrust = rospy.get_param("max_thrust")
        except:
            self.maxThrust = 100
        try:
            #width of paths in meters. this should also really be a multiple of the velocity. will be hard to follow tracks that are tighter than 1 second travel
            self.den = rospy.get_param("den")
        except:
            self.den = 5.0
        try:
            self.CLrate = rospy.get_param("CLrate")
        except:
            self.CLrate = 5
        try:
            #meters (prob want as a multiple of velocity to give a target point more than 1 second away
            self.searchRadius = rospy.get_param("search_radius")
        except:
            self.searchRadius = 5.0
        try:
            self.targetDepth = float(rospy.get_param('target_depth')) #meters
        except:
            self.targetDepth = 5.0
        try:
            # don't search within this number of the last points for boundary closure,
            # where boundary points are minimum self.searchRadius/2.0 apart
            self.ptBuff = rospy.get_param("complete_radius_buffer")
        except:
            self.ptBuff = 40

        #parameters to set
        self.rospyCLRate = rospy.Rate(self.CLrate)
        self.initDuration = 5 #s
        # if search radius is too small it becomes really unstable. Need to be related to the radius of the basins you are looking at, maybe 10%?
        self.sweepAngle = 0 #direction of sweep for DMPP decompositon
        self.waypointEps = 2.0
        self.sensorAngleRad = rospy.get_param('sensor_angle_rad') #-math.pi/2 = straight down, 0 = straight forwards
        #expMA
        self.EMAHalfLife = 5.0 #seconds
        self.posestamp = rospy.Time.now()
        self.theta = None

        #other class variables to initialise
        #rospy.sleep(3)

        self.TSP = TSP.Tsp()
        self.tf = TF.Tf()
        self.LI = Lines.Intersections()
        self.Planes = curvefit.Planes()
        self.sbsf = SingleBeam.SensorFns(self.sensorAngleRad)

        polyboundaries_E = rospy.get_param("poly_boundaries_E")
        polyboundaries_N = rospy.get_param("poly_boundaries_N")

        # self.polyboundaries = np.empty([len(polyboundaries_N),2])
        # for i in range(len(polyboundaries_N)):
        #     self.polyboundaries[i, 0] = polyboundaries_E[i]
        #     self.polyboundaries[i, 1] = polyboundaries_N[i]

        #create points along the line spaced self.den apart between the polyboundaries_E and _N imported
        self.polyboundaries = self.genPolyWaypoints(polyboundaries_E, polyboundaries_N, self.den, self.waypointEps)

        print self.polyboundaries

        self.planes = curvefit.Planes()
        self.northing = None
        self.easting = None
        self.height = None
        self.yaw = None
        self.pitch = 0
        self.roll = 0
        self.foundCB = False
        self.foundpoint = None
        self.boundaryDone = False
        self.lawnmowerPath = None
        self.KFinit = False
        self.SBSinit = False
        self.missionDone = False

        self.scannedPointsCount = 0
        self.scannedPoints = np.empty([200,3]) #X,Y,Z
        self.travelledCount = 0
        self.travelledPoints = np.empty([200,4]) #northing,easting,height, psi
        self.boundaryCount = 0
        self.boundaryPoints = np.empty([200,2]) #northing,easting

        self.initialScanDone = False
        self.initialFitDone = False
        self.boundarydone = False
        self.Cont = Controllers.Controllers(self.polyboundaries, self.den)
        self.Cont.onlineGP = OnlineGP.GP()
        self.newTheta = True
        self.userHome = os.path.expanduser('~')
        self.SBSDebugOut = open((self.userHome + '/tmp/scannedPoints.txt'),'w')
        self.SBSDebugOut.close()
        self.debugout = open((self.userHome + '/tmp/debugout.txt'),'w')
        self.GPOpt2 = open((self.userHome + '/tmp/GPOpt2.txt'),'w')

        self.msg = Twist()
        self.wpmsg = waypoints()
        self.nextWPCmsg = NextWayPointControl()
        self.debugstr = DebugString()
        self.bathmsg = bath()
        self.nsmsg = navStatus()

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pubPath = rospy.Publisher('path', waypoints, queue_size=1)
        self.pubNavStatus = rospy.Publisher('NavStatus', navStatus, queue_size=1)
        self.pubNextWayPoint = rospy.Publisher('NextWPCont',NextWayPointControl, queue_size=1)
        self.pubDebugstr = rospy.Publisher('debugstr', DebugString, queue_size = 1)
        self.pubBath = rospy.Publisher('bath', bath, queue_size=1)
        self.subPose = rospy.Subscriber('KFPose', kfpose, self.callback_pose, queue_size=10)
        self.subHyper = rospy.Subscriber('hyper', hyperSE, self.callback_hyper, queue_size = 1)
        rospy.sleep(3) #allow enough time for SBS and FPFMC to start up

        #set inital cmd velocity
        self.msg.linear.x = 0 #speed
        self.msg.angular.z = 0
        self.pub.publish(self.msg) #publish cmd_vel message

        # wait for Kalman Filter  to initialise
        rospy.loginfo('pre KFinit')
        while self.KFinit == False:
            self.rospyCLRate.sleep()
        self.subSBS = rospy.Subscriber('SBSData', SBS, self.callback_sbs, queue_size=10)
        # wait for and SBS to initialise
        rospy.loginfo('post KFinit, pre-SBS init')
        while self.SBSinit == False:
            self.rospyCLRate.sleep()

        self.findFollowBoundary()

        self.planLawnmowerPath()

        self.followPath()

        print 'done'
        self.SBSDebugOut.close()
        self.GPOpt2.close()
        self.missionDone = True
        rospy.spin()

    def callback_hyper(self,msg):
        lock = threading.RLock()
        with lock:
            print "new hyper", msg.var_f, msg.charLen, msg.var_n
            self.theta = [msg.var_f, msg.charLen, msg.var_n]
            self.newTheta = True
            self.GPOpt2.write(str(msg.var_f) + ',' +  str(msg.charLen) + ',' + str(msg.var_n) + '\n')
            print "new hyper done"
        print "callback_hyper lock released"

    def callback_pose(self,msg):
        lock = threading.RLock()
        with lock:
            #subscriber to kalman filter estimated pose
            self.northing = msg.northing
            self.easting = msg.easting
            self.height = msg.height
            self.yaw = msg.yaw
            self.pitch = msg.pitch
            self.roll = msg.roll
            dt = (msg.header.stamp - self.posestamp).to_nsec()/1e09
            if self.Cont.EMAHeading == None:
                self.Cont.EMAHeading = self.yaw
            self.posestamp = msg.header.stamp
            if dt < 100: # seconds. Check for reasonable timestamp difference to update exponential heading
                self.EMAAlpha = -math.log(0.5)*dt/self.EMAHalfLife #note half life = ln(0.5)/(-self.EMAAlpha) in timesteps
                EMACosHeading = self.EMAAlpha*math.cos(self.yaw) + (1-self.EMAAlpha)*math.cos(self.Cont.EMAHeading)
                EMASinHeading = self.EMAAlpha*math.sin(self.yaw) + (1-self.EMAAlpha)*math.sin(self.Cont.EMAHeading)
                self.Cont.EMAHeading = math.atan2(EMASinHeading, EMACosHeading)
            if self.KFinit == False:
                self.KFinit = True

    def callback_sbs(self,msg):
        lock = threading.RLock()
        with lock:
            #subscriber to Sonar readings
            if self.SBSinit == False:
                self.SBSinit = True
            #convert range to x,y,z bathymetry
            pos = self.tf.posMatrix(self.easting, self.northing, self.height)
            rot = self.tf.rotz(self.yaw)
            bath = self.sbsf.convRangeToBathpos(msg.range,rot,pos).ravel()
            if self.KFinit == True and bath != -9999.9:
                self.scannedPoints, self.scannedPointsCount = self.append(self.scannedPoints, bath, self.scannedPointsCount)
                self.bathmsg.X = bath[0]
                self.bathmsg.Y = bath[1]
                self.bathmsg.Z = bath[2]
                self.bathmsg.initDone = self.initialScanDone
                self.pubBath.publish(self.bathmsg)
                if self.missionDone == False:
                    bathstr = str(bath[0]) + ',' + str(bath[1]) + ',' + str(bath[2]) + ',' + str(self.yaw)+ ',' + str(pos[0][0])+ ',' + str(pos[1][0]) +'\n'
                    self.SBSDebugOut = open((self.userHome + '/tmp/scannedPoints.txt'),'a')
                    self.SBSDebugOut.write(bathstr)
                    self.SBSDebugOut.close()

    def fitPoints(self):
        fpcount = 0
        while self.theta == None:
            time.sleep(0.2) #block until OptGP has run
            print fpcount, "fitPoints() awaiting initial HP's from OptGP.py"
            fpcount += 0.2
        if self.newTheta == True:
            X = self.scannedPoints[:self.scannedPointsCount, 0:2]
            Y = self.scannedPoints[:self.scannedPointsCount, 2]
            self.Cont.onlineGP.setThetaFitAll(X, Y, self.theta)
            self.newTheta = False
        else:
            X = self.scannedPoints[self.scannedPointsCount-1, 0:2][np.newaxis,:]
            Y = self.scannedPoints[self.scannedPointsCount-1, 2]
            self.Cont.onlineGP.addFitObs(X,Y)

    def append(self, arr, data, counter):
        # add a row the the bottom of an array. if the array is full, double the number of rows. No checking that the
        # new data is the correct size
        m = arr.shape[0]
        if counter >= m:
            print "extending array"
            arr = np.vstack((arr,np.empty([m,arr.shape[1]]))) #note this will double the array size each time
        arr[counter] = data
        counter += 1 #this is passed by value to to get the increment we need to return the counter
        return arr, counter

    def pubNextWPControl(self,easting, northing, eucliddist, thrust, headingAdj , desiredheading, currentheading):
        self.nextWPCmsg.easting = easting
        self.nextWPCmsg.northing = northing
        self.nextWPCmsg.euclidDist = eucliddist
        self.nextWPCmsg.thrust = thrust
        self.nextWPCmsg.headingAdj = headingAdj
        self.nextWPCmsg.desHead = desiredheading
        #print "target", easting, northing, eucliddist, thrust, headingAdj, desiredheading, currentheading
        self.pubNextWayPoint.publish(self.nextWPCmsg)

    def spin(self,secs):
        mode = rospy.get_param('mode')
        while mode == 'manual':
            self.nsmsg.goalStatus = "FFC - await auto"
            self.pubNavStatus.publish(self.nsmsg)
            mode = rospy.get_param('mode')
            time.sleep(1)
        t = time.time()
        tcount = t
        self.nsmsg.goalStatus = "FFC - Init Circle"
        self.pubNavStatus.publish(self.nsmsg)

        while (time.time() - t) < secs:
            # with p_gain at 4, this should lead to ~ 25 power to port and 75 power to starboard
            # spin AC
            self.msg.linear.x = 50.0
            self.msg.angular.z = 0.2
            self.pub.publish(self.msg) #publish cmd_vel message
            time.sleep(0.01)
            if time.time() -tcount > 1:
                tcount = time.time()
                print round(time.time() - t,0), "'s of ", secs

    def findFollowBoundary(self):
        rospy.loginfo('in findfollowboundary, initialize by rotation')
        self.spin(self.initDuration)

        self.Cont.EMAHeading = self.yaw
        self.initialScanDone = True
        self.fitPoints()

        rospy.loginfo('in findfollowboundary, now follow contour')
        #now follow contour

        self.debugout = open((self.userHome + '/tmp/debugout.txt'),'w')
        print "pre-ffb loop", self.targetDepth, self.yaw, self.easting, self.northing, self.searchRadius, self.foundCB
        print "new theta ", self.newTheta, self.theta
        while self.boundaryDone == False:
            looptimer = time.time()
            e_copy = self.easting #take local copies so these aren't updated whilst we are in the loop below
            n_copy = self.northing
            y_copy = self.yaw
            self.travelledPoints, self.travelledCount = self.append(self.travelledPoints,[e_copy, n_copy, self.height, self.yaw],self.travelledCount)
            pos = np.array([[e_copy], [n_copy], [self.height]])
            stamp = rospy.Time.now()
            #check if new SBS data, if so fit it (assuming here here is only 1 point, if more than 1 we will only grab the last point
            # but then I have bigger issues anyway as my controll loop is slower than 1Hz
            if self.Cont.onlineGP.X.shape[0] < self.scannedPointsCount:
                self.fitPoints()
                #print "data point added to GP in ffb loop"

            desHead, r, ztmp, self.foundCB, debugstr = self.Cont.findFollowContourBoundary(self.targetDepth,y_copy, pos,
                                                                                    self.searchRadius, self.foundCB,
                                                                                     self.searchRadius)
            if desHead == None:
                rospy.loginfo("vessel more than R outside of polygon. Velocity set = 0. Autonomous control ended")
                self.msg.linear.x = 0
                self.pub.publish(self.msg) #publish cmd_vel message
                self.nsmsg.goalStatus = "too far outside Polygon"
                self.pubNavStatus.publish(self.nsmsg)
                break

            if self.foundCB == True:
                FCBtxt = "True"
            else:
                FCBtxt = "False"
            self.debugstr.debug = ((debugstr) + "ztmp=" + str(round(ztmp, 2)) + " " + FCBtxt + " ")

            if self.foundCB == True:

                # we have found the contour or boundary, now start checking for loop completion
                if self.boundaryCount == 0: #first point
                    #print '*******************************************'
                    ptd = self.searchRadius +1
                else:

                    ptd = ((self.boundaryPoints[self.boundaryCount-1, 0]-e_copy)**2
                           + (self.boundaryPoints[self.boundaryCount-1, 1]-n_copy)**2)**0.5
                minptdi = 9999999999
                #only use points every quarter radius to keep data sparse

                if ptd > self.searchRadius/4.0:
                    self.nsmsg.goalStatus = "FFC - " + debugstr[:3]
                    self.pubNavStatus.publish(self.nsmsg)
                    boundaryCountMinBuff = self.boundaryCount - self.ptBuff
                    #if we have more points than the minimum buffer,
                    #loop through all but the last self.ptbuff points to check for loop closure
                    if boundaryCountMinBuff > 1:
                        for i in range(boundaryCountMinBuff):
                            ptdi = ((self.boundaryPoints[i, 0]-e_copy)**2
                                    + (self.boundaryPoints[i, 1]-n_copy)**2)**0.5
                            minptdi = min(ptdi,minptdi)
                            if self.boundaryDone == False:
                                if ptdi < self.searchRadius:
                                    #we have completed the boundary, but keep going to see if there is a closer point
                                    self.boundaryDone = True
                                    rejoinIdx = i
                            else:
                                if ptdi < self.searchRadius and ptdi < minptdi:
                                    rejoinIdx = i
                                else:
                                    print 'contour boundary complete'
                                    self.boundarydone = True
                                    break
                    self.boundaryPoints, self.boundaryCount = self.append(self.boundaryPoints,[e_copy, n_copy],self.boundaryCount)
            thrust = self.maxThrust
            headingErr = self.planes.clampAngle(desHead-y_copy)
            self.msg.linear.x = thrust
            self.msg.angular.z = headingErr
            self.pub.publish(self.msg) #publish cmd_vel message to motor
            debugoutwrite = str(stamp) + ',' + debugstr + ',' + str(thrust) + ',' + str(headingErr) + '\n'
            self.debugout.write(debugoutwrite)
            wpE = e_copy + math.cos(desHead)*r
            wpN = n_copy + math.sin(desHead)*r
            self.pubNextWPControl(wpE, wpN, r, thrust, headingErr, desHead, y_copy)
            self.debugstr.debug = self.debugstr.debug + str(round(time.time()-looptimer,4))
            self.pubDebugstr.publish(self.debugstr)
            self.rospyCLRate.sleep() #sleep such that sleep time + loop time >=KFRate
        self.debugout.close()
        #resize boundaryPoints array to be the loop that is the boundary
        self.boundaryPoints = self.boundaryPoints[rejoinIdx:self.boundaryCount,:]
        f = open((self.userHome + '/tmp/boundaryDataPH.txt'),'w')
        for points in self.boundaryPoints:
            tmp = '{:.32f},{:.32f}'.format(points[0],points[1]) + '\n'
            f.write(tmp)
        f.close()
        print "calculating external Hull"
        #note maybe pad the search radius out here if the hull is not solving, but it should.
        self.boundaryPoints = self.LI.externalHull(self.boundaryPoints,self.searchRadius)
        print "external hull done"
        self.boundaryCount = len(self.boundaryPoints)
        f = open((self.userHome + '/tmp/boundaryData.txt'),'w')
        for points in self.boundaryPoints:
            tmp = '{:.32f},{:.32f}'.format(points[0],points[1]) + '\n'
            f.write(tmp)
        f.close()
        #stop ASV
        self.msg.linear.x = 0
        self.msg.angular.z = 0
        for i in range(10):
            self.pub.publish(self.msg) #publish cmd_vel message

    def planLawnmowerPath(self):
        # #implement cellular decomposition, AStar and DMPP decomposition to fill contour
        lastPoint = [self.easting, self.northing]
        polyDir =  self.LI.rotDirection(self.boundaryPoints)
        crossingAr, crossingCount, tmppos = DMPP.sweep(self.boundaryPoints, self.sweepAngle, self.den)
        cells, neighbours = DMPP.createCells(crossingAr,crossingCount,self.den,self.sweepAngle)
        shrunkCells = DMPP.shrinkCells(cells,self.den)

        posinpoly = False
        for sc in shrunkCells:
            if self.LI.isPointInPolygon(lastPoint[0],lastPoint[1],sc) ==True:
                posinpoly = True
                break
        # else find nearest point that is
        if posinpoly == False:
            eas,nor = DMPP.crossingPointClosestCell(lastPoint[0],lastPoint[1],shrunkCells)
            lastPoint = [eas,nor]

        path = np.array([lastPoint])

        for i in range(cells.shape[0]):
            lastPoint = path[-1]
            entry, transit = self.TSP.greedyCellTSP2(shrunkCells, lastPoint, self.boundaryPoints, self.den)
            transit = np.asarray(transit)
            if transit != []:
                path = np.vstack([path,transit])
            cellDir = self.LI.rotDirection(cells[entry[0]])
            lp = DMPP.localPoly(cells[entry[0]],self.boundaryPoints, cellDir, polyDir)
            subPath = DMPP.genLawnmower(cells[entry[0]],lp,entry[1],self.sweepAngle,self.den)
            cells = np.delete(cells,entry[0],axis=0) #remove that cell from the list
            shrunkCells = np.delete(shrunkCells, entry[0],axis=0)
            path = np.vstack([path,subPath])

        self.lawnmowerPath = path
        bp = open((self.userHome + '/tmp/dmppPath.txt'),'w')
        for point in self.lawnmowerPath:
            row = str(round(point[0],2)) + ',' + str(round(point[1],2)) + '\n'
            bp.write(row)
        bp.close()

        self.wpmsg.easting = path[:,0]
        self.wpmsg.northing = path[:,1]
        self.pubPath.publish(self.wpmsg)

    def followPath(self):
        rospy.loginfo('in followPath')
        #implement loop to issue motor comands to follow the lawnmower path.
        for i in xrange(self.lawnmowerPath.shape[0]):
            wpE = round(self.lawnmowerPath[i, 0],2)
            wpN = round(self.lawnmowerPath[i, 1],2)
            self.pubNavStatus.publish(self.nsmsg)
            euclidDist = self.waypointEps + 1
            while euclidDist>self.waypointEps:
                deshead = math.atan2(wpN-self.northing, wpE-self.easting)
                adj_head = self.planes.clampAngle(deshead-self.yaw)
                print "DH, H, Diff", deshead, self.yaw, adj_head
                #set veolcity (which is a percentage) at min of max velocity, or if within twice the distance to goal slow up
                thrust = min(self.maxThrust,euclidDist/0.5*100) #we slow down when within 0.5m of target Note this assumes
                # we have no environmental forcing. It should really be controlled on velocity, not thrust.
                self.msg.linear.x = thrust
                self.msg.angular.z = adj_head
                self.pub.publish(self.msg) #publish cmd_vel message
                euclidDist = math.sqrt((self.easting-wpE)**2 + (self.northing-wpN)**2)
                self.pubNextWPControl(wpE, wpN, euclidDist, thrust, adj_head, deshead, self.yaw)
                self.rospyCLRate.sleep()

    def genPolyWaypoints(self, pE, pN, dEN, wpEps):
        m = len(pN)
        path = [pE[0], pN[0]]
        for i in range(m):
            j = (i+1)% m
            pathseg = DMPP.wayPointsOnLine(pE[i], pN[i],pE[j], pN[j],dEN )
            path = np.vstack((path,pathseg))
        print path.shape
        done = False
        i = 0
        while done == False:
            j = (i+1)% len(path)
            ED = np.sqrt((path[i,0]-path[j,0])**2 + (path[i,1]-path[j,1])**2)
            if ED < wpEps:
                path = np.delete(path,i,0)
            else:
                i += 1
            if i == len(path):
                done = True
        return path

if __name__ == '__main__':
    try:
        contourBoundary()
    except rospy.ROSInterruptException:
        pass