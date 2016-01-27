__author__ = 'troy'

import math
import numpy as np
import Transformations as tf

# using SNAME(1950) notation for marine vehicles as per Fossen 2011
# Body reference frame, positions/angles
# x = 0 direction is forward
# y = 0 direction is starboard
# z = 0 direction is down (distance from average sea surface to bottom)
# Phi = rotation about x axis (roll, heel)
# Theta = rotation about Y axis (pitch, trim)
# Psi = rotation about z axis (yaw)
# NED = North (x), East (y), Down (z)
# Angles rotate anti-clockwise around these axes

class SensorFns():
    def __init__(self,sensTheta = -math.pi/2):
        self.sensTheta = sensTheta
        self.Tf = tf.Tf()
        self.SensorT = self._getSensorTranform()

    def convRangeToBathpos(self,range, rot,pos):
        adjPos, adjTheta, adjPsi = self._getAdjPosAngles(rot,pos)
        if abs(adjTheta + math.pi/2) < 1e-06: #looking straight down grab, depth directly
            bathpos = self.Tf.posMatrix(adjPos[0], adjPos[1], range)
        else:
            adjThetaRot = adjTheta + math.pi/2
            bathpos = self._getxyz(adjPos,adjThetaRot, adjPsi,range)
        return bathpos

    def _getSensorTranform(self):
        #Place the sensor tip at the given co-ordinates relative to CG, and with the given rotation
        x = 0.2
        y = 0.0
        z = -0.1
        phi = 0
        theta = self.sensTheta
        psi = 0
        A = self.Tf.homogenousMatrix(self.Tf.rotFixedRPY(phi, theta, psi),self.Tf.posMatrix(x,y,z))
        return np.round(A,16)

    def _getAdjPosAngles(self,rot,pos):
        # input rot and pos for CG
        # return position of sensor tip and pitch and yaw angles to go into Fixed RPY to get orientation of sensor tip
        CGh = self.Tf.homogenousMatrix(rot,pos)
        Sh = np.dot(CGh,self.SensorT) #position and orientation of sensor tip
        angles = self.Tf.getAngles(Sh)
        adjTheta = angles[1]
        adjPsi = angles[2]
        pos = self.Tf.posFromH(Sh) #[x,y,z]
        return pos, adjTheta, adjPsi

    def _getxyz(self, pos, theta, psi, range):
        z = math.cos(theta)*range +pos[2]
        dxy = z*math.tan(theta)
        x = pos[0] + math.cos(psi)*dxy
        y = pos[1] + math.sin(psi)*dxy
        bathPos = self.Tf.posMatrix(x, y, z)
        return bathPos

class SingleBeam(object):
    def __init__(self, bath, sensTheta = -math.pi/2, errStdDev=0.0, eps=1.0e-6, shiftTheta = 2*math.pi/180, maxRange=200):
        #all angle in radians
        # *Fix variable are the location of the tip of the sensor
        self.bath = bath
        self.errStdDev = errStdDev
        self.eps = eps
        self.Tf = tf.Tf()
        self.shiftTheta = shiftTheta
        self.sensTheta = sensTheta
        self.SensorT = self._getSensorTranform()
        self.maxRange = maxRange

    def _getSensorTranform(self):
        #Place the sensor tip at the given co-ordinates relative to CG, and with the given rotation
        x = 0.2
        y = 0.0
        z = -0.1
        phi = 0
        theta = self.sensTheta
        psi = 0
        A = self.Tf.homogenousMatrix(self.Tf.rotFixedRPY(phi, theta, psi),self.Tf.posMatrix(x,y,z))
        return np.round(A,16)

    def _getAdjSensorAngles(self,CGh):
        #return angles of sensor tip in terms of Yaw then Pitch
        #probably a cleaner way to do it, but this works as a first go
        ST = self.SensorT
        #CGh is the homogenoous Matrix of the CG of the ASV in world terms
        #Sh is the homogenous Matrix of the sensor tip in world terms
        Sh = np.dot(CGh,ST)
        #zero Sensor tip position
        Ssh = self.Tf.homogenousMatrix(self.Tf.rotFromH(Sh),self.Tf.posMatrix(0,0,0))
        # translate along sensor beam(x axis in sensor frame) by 1
        Ssdxh = np.dot(Ssh,self.Tf.transxh(1))
        #rotate Sh back to world frame
        Swh = self.Tf.homogenousMatrix(self.Tf.rotFixedRPY(0,0,0),self.Tf.posFromH(Ssh))
        #translate along worldframe X axis by 1
        Swdxh = np.dot(Swh,self.Tf.transxh(1))
        #translate along worldframe Z axis by 1
        Swdzh = np.dot(Swh,self.Tf.transzh(1))
        Ssdxp = self.Tf.posFromH(Ssdxh)[:,0]
        Swdxp = self.Tf.posFromH(Swdxh)[:,0]
        Swdzp = self.Tf.posFromH(Swdzh)[:,0]
        theta = math.atan2(np.linalg.norm(np.cross(Ssdxp,Swdzp)),np.dot(Ssdxp,Swdzp)) #Angle between Z worldframe z axis and beam
        # now need to calculate the yaw
        #if I take my point Ssdxh and set the z value = 0, then I have a new point which is projected onto the X,Y world plane
        Ssdxz0p = np.copy(Ssdxp)
        Ssdxz0p[2] = 0
        psi = math.atan2(np.linalg.norm(np.cross(Ssdxz0p,Swdxp)),np.dot(Ssdxz0p,Swdxp)) #Angle between the 2 vectors
        #print "gamma \n", gamma, gamma*180/math.pi #angle from x around z)

        return theta, psi

    def _getbathline(self,x,y,psi,dxy):
        adjx = x + math.cos(psi)*dxy
        adjy = y + math.sin(psi)*dxy
        return self.bath.queryDepth(adjx, adjy)

    def _getxyz(self, pos, theta, psi, range):
        z = math.cos(theta)*range +pos[2]
        dxy = z*math.tan(theta)
        x = pos[0] + math.cos(psi)*dxy
        y = pos[1] + math.sin(psi)*dxy
        bathPos = self.Tf.posMatrix(x, y, z)
        return bathPos

    def _getAdjPosAngles(self,rot,pos):
        # input rot and pos for CG
        # return position of sensor tip and pitch and yaw angles to go into Fixed RPY to get orientation of sensor tip
        CGh = self.Tf.homogenousMatrix(rot,pos)
        Sh = np.dot(CGh,self.SensorT) #position and orientation of sensor tip
        angles = self.Tf.getAngles(Sh)
        adjTheta = angles[1]
        adjPsi = angles[2]
        pos = self.Tf.posFromH(Sh) #[x,y,z]
        return pos, adjTheta, adjPsi

    def _adderr(self,range):
        if self.errStdDev == 0:
            return range
        else:
            return np.random.normal(range, self.errStdDev)

    def getReadingsH(self,h):
        bathpos = self.getReadings(self.Tf.rotFromH(h), self.Tf.posFromH(h))
        return bathpos

    def getReadingsold(self ,rot ,pos):
        #input angles and position for ASV CG
        #solve for depth within a given angle error using bisection method
        # no overhangs assumed (or even slope steeper than beam angle?)
        adjPos, adjTheta, adjPsi = self._getAdjPosAngles(rot,pos)
        # adjTheta, adjPsi = self._getAdjSensorAngles(self.Tf.homogenousMatrix(rot,pos))
        # adjPos = pos + self.Tf.posFromH(self.SensorT)
        # print adjTheta, adjPsi
        if adjTheta == -math.pi/2: #looking straight down grab, depth directly
            depth = self.bath.queryDepth(adjPos[0], adjPos[1])
            bathpos = self.Tf.posMatrix(adjPos[0], adjPos[1], depth)
        else:
            maxz = self.bath.maxz - adjPos[2]
            adjThetaRot = adjTheta + math.pi/2
            guessdxy = math.tan(adjThetaRot)*(maxz) #need to add Pi/2 as adj theta is the angle from the x axis, not z axis
            shift = abs(math.tan(self.shiftTheta)*(maxz))
            lBound = guessdxy - shift
            uBound = guessdxy + shift
            #print adjTheta/math.pi*180, adjPsi/math.pi*180,
            mBound = ((uBound+lBound)/2.0)
            error = self.eps + 1  # set to force loop entry
            count = 0
            while error > self.eps:  # the loop executes till the desired level of accuracy is attained
                z=self._getbathline(adjPos[0],adjPos[1],adjPsi, mBound) - adjPos[2]
                thetam= math.atan(mBound/z)
                thetal = math.atan(lBound/z)
                thetau = math.atan(uBound/z)
                #range = z/math.cos(thetam)

                #if the actual beam lies between upper bound and the middle value, the lower bound is set to be the middle value
                if adjThetaRot>thetam:
                    lBound = mBound

                #if the actual beam lies between lower bound and the middle value, the upper bound is set to be the middle value
                elif adjThetaRot<=thetam:
                    uBound=mBound
                count += 1
                #this is needed as especially for large sensor angles, fitting of curves can make the actual max depth > self.maxZ
                if self.eps>math.fabs(thetau-thetal):
                    if adjThetaRot < thetal:
                        lBound = lBound - shift
                    else:
                        uBound = uBound + shift

                mBound = ((uBound+lBound)/2.0)
                error=math.fabs(adjThetaRot-thetam)

                range = z/(math.cos(adjThetaRot))
                if range > self.maxRange:
                    return None
                if count >2000:
                    print "\n failed to converge on sonar reading"
                    return None
                count += 1

            range = self._adderr(range)
            bathpos = self._getxyz(adjPos,adjThetaRot, adjPsi,range)
            #print adjPos[0,0], adjPos[1,0],adjPos[2,0], adjTheta/math.pi*180, adjPsi/math.pi*180, bathpos[0,0],bathpos[1,0],bathpos[2,0]
        return bathpos

    def getRange(self,rot ,pos):
        #input angles and position for ASV CG
        #solve for depth within a given angle error using bisection method
        # no overhangs assumed (or even slope steeper than beam angle?)
        adjPos, adjTheta, adjPsi = self._getAdjPosAngles(rot,pos)
        # adjTheta, adjPsi = self._getAdjSensorAngles(self.Tf.homogenousMatrix(rot,pos))
        # adjPos = pos + self.Tf.posFromH(self.SensorT)
        # print adjTheta, adjPsi
        if adjTheta == -math.pi/2: #looking straight down grab, depth directly
            range = self.bath.queryDepth(adjPos[0], adjPos[1])
        else:
            maxz = self.bath.maxz - adjPos[2]
            adjThetaRot = adjTheta + math.pi/2
            guessdxy = math.tan(adjThetaRot)*(maxz) #need to add Pi/2 as adj theta is the angle from the x axis, not z axis
            shift = abs(math.tan(self.shiftTheta)*(maxz))
            lBound = guessdxy - shift
            uBound = guessdxy + shift
            #print adjTheta/math.pi*180, adjPsi/math.pi*180,
            mBound = ((uBound+lBound)/2.0)
            error = self.eps + 1  # set to force loop entry
            count = 0
            while error > self.eps:  # the loop executes till the desired level of accuracy is attained
                z=self._getbathline(adjPos[0],adjPos[1],adjPsi, mBound) - adjPos[2]
                thetam= math.atan(mBound/z)
                thetal = math.atan(lBound/z)
                thetau = math.atan(uBound/z)
                #range = z/math.cos(thetam)

                #if the actual beam lies between upper bound and the middle value, the lower bound is set to be the middle value
                if adjThetaRot>thetam:
                    lBound = mBound

                #if the actual beam lies between lower bound and the middle value, the upper bound is set to be the middle value
                elif adjThetaRot<=thetam:
                    uBound=mBound
                count += 1
                #this is needed as especially for large sensor angles, fitting of curves can make the actual max depth > self.maxZ
                if self.eps>math.fabs(thetau-thetal):
                    if adjThetaRot < thetal:
                        lBound = lBound - shift
                    else:
                        uBound = uBound + shift

                mBound = ((uBound+lBound)/2.0)
                error=math.fabs(adjThetaRot-thetam)

                range = z/(math.cos(adjThetaRot))
                if range > self.maxRange:
                    return None
                if count >2000:
                    print "\n failed to converge on sonar reading"
                    return None
                count += 1
        return self._adderr(range)

    def getReadings(self ,rot ,pos):
        range = self.getRange(rot ,pos)
        adjPos, adjTheta, adjPsi = self._getAdjPosAngles(rot,pos)
        if adjTheta == -math.pi/2: #looking straight down grab, depth directly
            bathpos = self.Tf.posMatrix(adjPos[0], adjPos[1], range)
        else:
            adjThetaRot = adjTheta + math.pi/2
            bathpos = self._getxyz(adjPos,adjThetaRot, adjPsi,range)
        return bathpos

