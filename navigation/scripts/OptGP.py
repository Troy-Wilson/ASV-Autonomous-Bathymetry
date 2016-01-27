#!/usr/bin/env python

from __future__ import division
__author__ = 'troy'

# This node will listen to bath messages and periodically estimate the hyper-paramaters and publish them
# Probably too nasty in Python to pass messages of large covariance matricies, so I will just pass the hyper-params
# this may be slightly in-efficient as it will result in 1 extra gp.fit() call to get that covariance matrix in the
#  other thread


import OnlineGP
from navigation.msg import bath, hyperSE
import numpy as np
import rospy
import os.path
import time

class optGP():
    def __init__(self):
        rospy.init_node('OptGP')
        self.bathCount = 0
        self.bathPoints = np.empty([2000,3]) #X,Y,Z

        self.pubHyper = rospy.Publisher('hyper', hyperSE, queue_size = 1)
        self.subBath = rospy.Subscriber('bath', bath, self.callback_bath, queue_size=10)
        self.repeatOptTrigger = rospy.get_param("repeat_HP_opt")
        self.hyperMsg = hyperSE()

        self.onlineGP = OnlineGP.GP()
        self.initTheta = [1.0,1.0,1.0]
        self.bounds=[(0.000001,None),(0.000001,None),(0.000001,None)]
        self.onlineGP.setTheta(self.initTheta)
        self.initScanDone = False
        self.initFitDone = False

        self.userHome = os.path.expanduser('~')
        self.GPOpt = open((self.userHome + '/tmp/GPOpt.txt'),'w')
        self.keepAliveLoop()

    def keepAliveLoop(self):
        while True:
            time.sleep(0.5)

    def callback_bath(self,data):
        dataArr = [data.X, data.Y, data.Z]
        self.initScanDone = data.initDone
        self.bathPoints, self.bathCount = self.append(self.bathPoints,dataArr,self.bathCount)
        print "bath data received", self.bathCount
        if self.initScanDone == True:
            if self.initFitDone == False:
                print "init HP Fit"
                self.runOpt()
                self.initFitDone = True
                self.trigger = self.bathCount + self.repeatOptTrigger
            else:
                if self.bathCount >= self.trigger:
                    self.runOpt()
                    self.trigger += self.repeatOptTrigger

    def append(self, arr, data, counter):
        # add a row the the bottom of an array. if the array is full, double the number of rows. No checking that the
        # new data is the correct size
        m = arr.shape[0]
        if counter >= m:
            arr = np.vstack((arr,np.empty([m,arr.shape[1]]))) #note this will double the array size each time
        arr[counter] = data
        counter += 1 #this is passed by value to to get the increment we need to return the counter
        return arr, counter

    def runOpt(self):
        X = self.bathPoints[:self.bathCount, 0:2]
        Y = self.bathPoints[:self.bathCount, 2]
        self.onlineGP.setData(X, Y)
        theta = [self.onlineGP.var_f, self.onlineGP.charLen, self.onlineGP.var_n]
        opt = self.onlineGP.optimise(theta,self.bounds)

        theta = opt[0]
        GPOptstr = 'init,'+  str(theta[0])+ ','+  str(theta[1])+ ','+  str(theta[2])+ ','+ str(opt[1])  + '\n'
        self.GPOpt.write(GPOptstr)

        self.onlineGP.setTheta(theta)
        self.hyperMsg.var_f = theta[0]
        self.hyperMsg.charLen = theta[1]
        self.hyperMsg.var_n = theta[2]
        self.pubHyper.publish(self.hyperMsg)
        print "OptGP.py HP", self.hyperMsg.var_f, self.hyperMsg.charLen, self.hyperMsg.var_n, "X.shape ", X.shape

if __name__ == '__main__':
    try:
        optGP()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()