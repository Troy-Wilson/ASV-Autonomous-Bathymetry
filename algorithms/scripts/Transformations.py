# -*- coding: utf-8 -*-
"""
Created on Mon Aug 18 10:50:17 2014

@author: troy
"""

import numpy as np
from math import sin, cos, pi, asin, sqrt, atan2, acos

class Tf(object):
    # using SNAME(1950) notation for marine vehicles as per Fossen 2011
    # Body reference frame, positions/angles
    # x = 0 direction is forward
    # y = 0 direction is starboard
    # z = 0 direction is down (distance from average sea surface to bottom)
    # Phi = rotation about x axis (roll, heel)
    # Theta = rotation about Y axis (pitch, trim)
    # Psi = rotation about z axis (yaw)
    def __init__(self,x=None,y=None,z=None,phi=None,theta=None,psi=None):
        if x == None:
            self.h = None
        else:
            self.h = self.homogenousMatrix(x,y,z,phi,theta,psi)
               
    # def rotFixedRPY(self,phi,theta,psi):
    #     #roll pitch yaw in world frame
    #     r = np.empty([3,3])
    #     r[0,0] = cos(psi)*cos(theta)
    #     r[0,1] = -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi)
    #     r[0,2] = sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta)
    #     r[1,0] = sin(psi)*cos(theta)
    #     r[1,1] = cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi)
    #     r[1,2] = -cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi)
    #     r[2,0] = -sin(theta)
    #     r[2,1] = cos(theta)*sin(phi)
    #     r[2,2] = cos(theta)*cos(phi)
    #     return np.round(r,16)

    def rotFixedRPY(self,phi,theta,psi,precision = 12):
        #roll pitch yaw in world frame
        cosphi = np.round(cos(phi),precision)
        costheta = np.round(cos(theta),precision)
        cospsi = np.round(cos(psi),precision)
        sinphi = np.round(sin(phi),precision)
        sintheta = np.round(sin(theta),precision)
        sinpsi = np.round(sin(psi),precision)
        r = np.empty([3,3])
        r[0,0] = cospsi*costheta
        r[0,1] = -sinpsi*cosphi+cospsi*sintheta*sinphi
        r[0,2] = sinpsi*sinphi+cospsi*cosphi*sintheta
        r[1,0] = sinpsi*costheta
        r[1,1] = cospsi*cosphi+sinphi*sintheta*sinpsi
        r[1,2] = -cospsi*sinphi+sintheta*sinpsi*cosphi
        r[2,0] = -sintheta
        r[2,1] = costheta*sinphi
        r[2,2] = costheta*cosphi
        return np.round(r,precision)

    def rotEulerYPR(self,psi,theta,phi,precision = 12):
        #yaw pitch roll in body frame

        #working on assumption x is direction of heading
        #from craig
        #alpha = psi [yaw - rotation around z axis]
        #beta = theta [pitch - rotation around y axis]
        #gamma = phi [roll - rotation around x axis]

        cosphi = np.round(cos(phi),precision)
        costheta = np.round(cos(theta),precision)
        cospsi = np.round(cos(psi),precision)
        sinphi = np.round(sin(phi),precision)
        sintheta = np.round(sin(theta),precision)
        sinpsi = np.round(sin(psi),precision)
        r = np.empty([3,3])
        r[0,0] = cospsi*costheta
        r[0,1] = cospsi*sintheta*sinphi - sinpsi*cosphi
        r[0,2] = cospsi*sintheta*cosphi + sinpsi*sinphi
        r[1,0] = sinpsi*costheta
        r[1,1] = sinpsi*sintheta*sinphi + cospsi*cosphi
        r[1,2] = sinpsi*sintheta*cosphi - cospsi*sinphi
        r[2,0] = -sintheta
        r[2,1] = costheta*sinphi
        r[2,2] = costheta*cosphi
        return np.round(r,precision)

    def rotx(self,phi):
        r = np.empty([3,3])
        r[0,0] = 1 
        r[0,1] = 0
        r[0,2] = 0
        r[1,0] = 0
        r[1,1] = cos(phi)
        r[1,2] = -sin(phi)
        r[2,0] = 0
        r[2,1] = sin(phi)
        r[2,2] = cos(phi)
        return np.round(r,15)
        
    def roty(self,theta):
        r = np.empty([3,3])
        r[0,0] = cos(theta)
        r[0,1] = 0
        r[0,2] = sin(theta)
        r[1,0] = 0
        r[1,1] = 1
        r[1,2] = 0
        r[2,0] = -sin(theta)
        r[2,1] = 0
        r[2,2] = cos(theta)
        return np.round(r,15)
        
    def rotz(self,psi):
        r = np.empty([3,3])
        r[0,0] = cos(psi)
        r[0,1] = -sin(psi)
        r[0,2] = 0
        r[1,0] = sin(psi)
        r[1,1] = cos(psi)
        r[1,2] = 0
        r[2,0] = 0
        r[2,1] = 0
        r[2,2] = 1
        return np.round(r,15)
        
    def transx(self,x):
        p = np.empty([3,1])
        p[0,0] = x
        p[1,0] = 0
        p[2,0] = 0
        return np.round(p,15)
    
    def transy(self,y):
        p = np.empty([3,1])
        p[0,0] = 0
        p[1,0] = y
        p[2,0] = 0
        return np.round(p,15)
        
    def transz(self,z):
        p = np.empty([3,1])
        p[0,0] = 0
        p[1,0] = 0
        p[2,0] = z
        return np.round(p,15)

    def transxh(self,x):
        I = np.identity(4)
        I[0,3] = x
        return np.round(I,15)

    def transyh(self,y):
        I = np.identity(4)
        I[1,3] = y
        return np.round(I,15)

    def transzh(self,z):
        I = np.identity(4)
        I[2,3] = z
        return np.round(I,15)

    def homogenousOrigin(self):
        I = np.identity(4)
        return I

    def rotxh(self,x):
        r = self.rotx(x)
        r = np.append(r,[[0,0,0]],0)
        r = np.append(r,[[0],[0],[0],[1]],1)
        return np.round(r,15)

    def rotyh(self,y):
        r = self.roty(y)
        r = np.append(r,[[0,0,0]],0)
        r = np.append(r,[[0],[0],[0],[1]],1)
        return np.round(r,15)

    def rotzh(self,z):
        r = self.rotz(z)
        r = np.append(r,[[0,0,0]],0)
        r = np.append(r,[[0],[0],[0],[1]],1)
        return np.round(r,15)

    def posMatrix(self,x,y,z):
        p = np.empty([3,1])
        p[0,0] = x
        p[1,0] = y
        p[2,0] = z
        return np.round(p,15)

    def transThenRot(self, h1, h2):
        p = self.posFromH(h1) + self.posFromH(h2)
        r = np.dot(self.rotFromH(h1),self.rotFromH(h2))
        h = self.homogenousMatrix(r,p)
        return np.round(h,15)

    def homogenousMatrix(self,r,p):
        #create homogenous matrix from Rot and pos matricies
        h = np.append(r,[[0.0,0.0,0.0]],0)        
        p = np.append(p,[[1.0]],0)
        h = np.append(h,p,1)
        return np.round(h,15)
        
    def rotFromH(self,h):
        h = h[0:3,0:3]
        return np.round(h,15)

    def posFromH(self,h):
        h=h[0:3,3:]
        return np.round(h,15)

    def getAngles(self,h):
        #works on either homogenous or rotation matrix as only using top left cnr
        #http://www.soi.city.ac.uk/~sbbh653/publications/euler.pdf, but it had phi and psi defined backwards
        theta = asin(self.clampAngle(-h[2,0]))
        if cos(theta) != 0:
            phi = atan2(h[2,1]/cos(theta),h[2,2]/cos(theta))
            psi = atan2(h[1,0]/cos(theta),h[0,0]/cos(theta))
        else:
            #Gimble Lock. An infinite number of possible solutions, lets set Phi = 0
            psi = 0
            if h[2,0] == -1:
                theta = pi/2
                phi = atan2(h[0,1],h[0,2])
            else:
                theta = -pi/2
                phi = atan2(-h[0,1],-h[0,2])

        #THIS WOULD ALSO BE A VALID TRIPLET
        # theta2 = pi-theta1
        # if cos(theta2) !=0:
        #     phi2 = atan2(h[2,1]/cos(theta2),h[2,2]/cos(theta2))
        #     psi2 = atan2(h[2,1]/cos(theta2),h[2,2]/cos(theta2))

        angles= np.empty([3])
        angles[0] = phi
        angles[1] = theta
        angles[2] = psi
        return angles

    def clampAngle(self, x):
        x = max(min(1,x),-1)
        return x

    def dh(self,d,theta,r,alpha):
        A = np.dot(self.rotzh(theta),self.transzh(d))
        A = np.dot(A, self.transxh(r))
        A = np.dot(A,self.rotxh(alpha))
        return A

    def invertH(self,h):
        #using eqn 2.45 from Craig 2005
        rt = np.transpose(self.rotFromH(h))
        post = np.dot(-rt,self.posFromH(h))
        ih = self.homogenousMatrix(rt,post)
        return ih

    def angleTwoVectors(self,A,B):
        return atan2(np.linalg.norm(np.cross(A,B)),np.dot(A,B))

    def getAngleAxis(self,v0,v1):
        #get axis perpendicular to the plane formed by the 2 vectors
        if (v0 == v1).all():
            angle = 0
            axis = v0
        else:
            axis = np.cross(v0,v1)
            #get angle between the 2 vectors
            angle = atan2(np.linalg.norm(axis),np.dot(v0,v1))
        return angle, axis

    def invertVByZ(self,v):
        vFlip = v*(-1)
        #done below with rotation matricies :)
        # vO = self.transz(1)[:,0]
        # #print v, vO
        # angle, axis = self.getAngleAxis(vO,v)
        # rot = self.angleAxisToRotMatrix(axis,angle)
        # #flip around Z
        # length = np.linalg.norm(v)
        # hv = self.homogenousMatrix(rot,self.posMatrix(v[0],v[1],v[2]))
        # hvFlip = np.dot(hv,self.transzh(-length*2))
        # vFlip = self.posFromH(hvFlip)[:,0]
        return vFlip

    def angleAxisToRotMatrix(self,axis,angle):
        #http://en.wikipedia.org/wiki/Euler%E2%80%93Rodrigues_formula
        axis = axis/sqrt(np.dot(axis,axis))
        a = cos(angle/2)
        b,c,d = -axis*sin(angle/2)
        return np.array([[a*a+b*b-c*c-d*d, 2*(b*c+a*d), 2*(b*d-a*c)],
                         [2*(b*c-a*d), a*a+c*c-b*b-d*d, 2*(c*d+a*b)],
                         [2*(b*d+a*c), 2*(c*d-a*b), a*a+d*d-b*b-c*c]])

if __name__ == '__main__':
    tf=Tf()

    #print tf.rotFixedRPY(np.pi,0.0,np.pi)
    #print tf.rotFixedRPY(0.0,np.pi,0.0)


    imuadji = 4.17
    print (imuadji+np.pi)%(np.pi*2)-np.pi
    rot = tf.rotFixedRPY(np.pi,0.0,imuadji,precision = 6)
    rot2 = tf.rotFixedRPY(0.0,0.0,imuadji,precision = 6)
    yaw = tf.getAngles(rot)
    yaw2 = tf.getAngles(rot2)

    print rot, yaw
    print rot2, yaw2


    #ASV in world frame (NED)
#     phicg= 1*pi/180
#     thetacg= 2*pi/180
#     psicg = 3*pi/180
#     xcg = 10.0
#     ycg = 5.0
#     zcg = 1.0
#     rcg = tf.rotFixedRPY(phicg, thetacg,psicg)
#     pcg = tf.posMatrix(xcg, ycg, zcg)
#     hcg = tf.homogenousMatrix(rcg,pcg)
#     print hcg
#     print tf.getAngles(hcg)/pi*180
#
#     #Sensor relative to CG of ASV
#     # DH table to get fixed arm from cg to sensor tip
#     # define
#     phis = 0
#     thetas = 15*pi/180
#     psis = 0
#     xs = 1
#     ys = .1
#     zs = -.2
#     rs = tf.rotFixedRPY(phis, thetas, psis)
#     ps = tf.posMatrix(xs, ys, zs)
#     hs = tf.homogenousMatrix(rs,ps)
#
#
#
#     #sensor in worldframe
#     rsw = np.dot(rcg,rs)
#     #psw =
#
#
# #    phi0 = 45*pi/180.0
#    theta0 = 45*pi/180.0     
#    psi0 = 45*pi/180.0
#    x0 = 1.0
#    y0 = 2.0
#    z0 = 3.0
#
#    tf = Transformations()
##    rot = tf.rotMatrix(phi,theta,psi)
##    pos = tf.posMatrix(x,y,z)
##    h = tf.homogenousMatrix(rot,pos)
#    
#    phi1 = 1*pi/180.0
#    theta1 = 2*pi/180.0
#    psi1 = 3*pi/180.0
#    x1 = 0.1
#    y1 = 0.1
#    z1 = 0.1
# 
#    rot0 = tf.rotFixedRPY(phi0, theta0, psi0)
#    pos0 = tf.posMatrix(x0,y0,z0)
#    h0 = tf.homogenousMatrix(rot0,pos0)
#    
#    rot1 = tf.rotFixedRPY(phi1, theta1, psi1)
#    pos1 = tf.posMatrix(x1,y1,z1)
#    h1 = tf.homogenousMatrix(rot1,pos1)
#    
#    h2 = np.dot(h0,h1)
#    print h2
#    print tf.getAngles(h2)*180/pi
#    
#    h2a = tf.transThenRot(h0,h1)
#    print h2a
#    print tf.getAngles(h2a)*180/pi
#
#    
# test a manual rotation
# import curvefit
# Tf = Tf()
# Cf = curvefit.Planes()
# #create a point translated by 1 along the z axis
# h0 = Tf.transzh(1)
# v0 = Tf.posFromH(h0)[:,0]
# print "h0\n", h0
# print "v0\n", v0
# v1 = np.array([1,1,1])
# v1 = Cf.normalise3dVector(v1)
# print "v1\n",v1, "\n"
#
# ang, axis = getAngleAxis(v0,v1)
# print ang
# print axis
#
# rot = angleAxisToRotMatrix(axis,ang)
# print "\n",  rot, "\n"
# print np.dot(rot,v0)