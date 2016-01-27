#!/usr/bin/env python
__author__ = 'troy'

# KF for GPS and IMU in 3D, but with non-syncronous updates
# Assume GPS update interval is a multiple of INS (in reality, INS so fast, just take closest as per Qi, Moore 2002)
# Constant acceleration model

# intended for this to be called by the publisher for filtered position data
# raw GPS and IMU data will not be published.

import numpy as np
import Kalman

class ASV_KF():
    def __init__(self,Xhat=None,P=None, ri=None, rg=None, sigq = None):
        #Xhat Nx1
        #P NxN
        #ri should be either a scalar or a
        #rg should be 1xN matrix or list

        self.dim = 9 #estimate kalman filter states = x,xd,xdd,y, yd, ydd, z, zd, zdd
        self.wait = False
        self.zgps = None
        self.zgpsvar = None
        self.t = None
        # define process and measurment variances
        if sigq == None:
            self.sigq = 0.1**2 #process variance
        else:
            self.sigq = sigq

        if ri == None:
            self.ri = .03**2 #measurement variance INS acceleration
        else:
            self.ri = ri
        #rg = .3**2 #measurement variance gps position
        if rg != None:
            #use static GPS variances
            self.rg = rg
            self.staticRG = True
            #generate static self.Rig
            if len(self.ri) == 3:
                self.Rig = np.array([[self.rg[0],0,0,0,0,0],#x obs
                        [0,self.ri[0],0,0,0,0], #xdd obs
                        [0,0,self.rg[1],0,0,0], #y obs
                        [0,0,0,self.ri[1],0,0], #ydd obs
                        [0,0,0,0,self.rg[2],0], #z obs
                        [0,0,0,0,0,self.ri[2]]]) #zdd obs
            else:
                self.Rig = np.array([[self.rg[0],0,0,0,0,0],#x obs
                        [0,self.ri,0,0,0,0], #xdd obs
                        [0,0,self.rg[1],0,0,0], #y obs
                        [0,0,0,self.ri,0,0], #ydd obs
                        [0,0,0,0,self.rg[2],0], #z obs
                        [0,0,0,0,0,self.ri]]) #zdd obs

        else:
            self.staticRG = False

        # set initial estimates and initialise KF
        if Xhat == None:
            Xhat = np.zeros((self.dim,1))
        if P == None:
            P = np.eye(self.dim)
        self.kalmandt = Kalman.Kalmandt(Xhat,P)

        #define static matricies
        self.Hi = np.array([[0,0,1,0,0,0,0,0,0], #xdd observed
                        [0,0,0,0,0,1,0,0,0], #ydd observed
                        [0,0,0,0,0,0,0,0,1]]) #zdd observed
        self.Hig = np.array([[1,0,0,0,0,0,0,0,0], #x obs
                         [0,0,1,0,0,0,0,0,0], #xdd obs
                         [0,0,0,1,0,0,0,0,0], #y obs
                         [0,0,0,0,0,1,0,0,0], #ydd obs
                         [0,0,0,0,0,0,1,0,0], #z obs
                         [0,0,0,0,0,0,0,0,1]]) #zdd obs
        self.Ri = np.eye(3)*self.ri #xdd, ydd, zdd obs
        # self.Rig = np.array([[rg,0,0,0,0,0],#x obs
        #                 [0,ri,0,0,0,0], #xdd obs
        #                 [0,0,rg,0,0,0], #y obs
        #                 [0,0,0,ri,0,0], #ydd obs
        #                 [0,0,0,0,rg,0], #z obs
        #                 [0,0,0,0,0,ri]]) #zdd obs

    def generateFQ(self,dt):
        F_ = np.array([[1,dt, dt**2/2],
                      [0,1,dt],
                      [0,0,1]])
        F = np.vstack((np.hstack((F_,np.zeros((3,3)),np.zeros((3,3)))),
                        np.hstack((np.zeros((3,3)),F_,np.zeros((3,3)))),
                        np.hstack((np.zeros((3,3)),np.zeros((3,3)),F_))))
        Q_ = np.array([[dt**5/20,dt**4/8,dt**3/6],
                        [dt**4/8,dt**3/3,dt**2/2],
                        [dt**3/6,dt**2/2,dt]])
        Q = np.vstack((np.hstack((Q_,np.zeros((3,3)),np.zeros((3,3)))),
                        np.hstack((np.zeros((3,3)),Q_,np.zeros((3,3)))),
                        np.hstack((np.zeros((3,3)),np.zeros((3,3)),Q_))))
        Q = Q*self.sigq
        return F, Q

    def generateRig(self,zvar):
        if self.staticRG == True:
            pass #use statically generated Rig
        else:
            if len(self.ri) == 3:
                self.Rig = np.array([[zvar[0,0],0,0,0,0,0],#x obs
                            [0,self.ri[0],0,0,0,0], #xdd obs
                            [0,0,zvar[1,0],0,0,0], #y obs
                            [0,0,0,self.ri[1],0,0], #ydd obs
                            [0,0,0,0,zvar[2,0],0], #z obs
                            [0,0,0,0,0,self.ri[2]]]) #zdd obs
            else:
                self.Rig = np.array([[zvar[0,0],0,0,0,0,0],#x obs
                            [0,self.ri,0,0,0,0], #xdd obs
                            [0,0,zvar[1,0],0,0,0], #y obs
                            [0,0,0,self.ri,0,0], #ydd obs
                            [0,0,0,0,zvar[2,0],0], #z obs
                            [0,0,0,0,0,self.ri]]) #zdd obs

    def pred(self,t):
        if self.t == None:
            self.t = t
        #if t == 0:
        #    dt = 1/50.0
        #    #self.t = t
        #else:
        dt = t-self.t
        self.t = t
        F,Q = self.generateFQ(dt)
        self.kalmandt.pred(F,Q)

    def updateINS(self,z,t):
        #dt = t-self.t
        #self.t = t
        self.pred(t)
        if self.wait == False:
            self.kalmandt.update(z,self.Hi,self.Ri)
            #print "INS update values", z
        else:
            self.zgpsins = np.array([[self.zgps[0,0]], [z[0,0]],
                                [self.zgps[1,0]], [z[1,0]],
                                [self.zgps[2,0]], [z[2,0]]])
            self._updateGPSINS()
            self.wait = False

    def updateGPS(self,z,zvar):
        #print "zvar", zvar
        self.wait = True
        # set trigger update GPS and INS when next INS data point arrives
        self.zgps = z
        self.generateRig(zvar)

    def _updateGPSINS(self):
        #don't call this function, call update GPS which then combines data and calls this
        self.kalmandt.update(self.zgpsins,self.Hig,self.Rig)
        #print "gps n,e, KF n,e", self.zgps[0,0], self.zgps[1,0], self.zgps[2,0],\
            #self.kalmandt.Xhat[0,0], self.kalmandt.Xhat[3,0], self.kalmandt.Xhat[6,0]
        #print self.Rig[0,0], self.Rig[2,2], self.Rig[4,4], self.Rig[1,1], self.Rig[3,3], self.Rig[5,5]










