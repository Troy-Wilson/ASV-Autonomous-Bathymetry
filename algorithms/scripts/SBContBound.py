__author__ = 'troy'

import numpy as np
import matplotlib.pyplot as plt
import math
import time
import Transformations as tf
import CurveFit
import BathymCreate
import SingleBeam
import Controllers
from matplotlib import cm
import OnlineGP
import os.path
import numpy as np

class SBContBound():
    def __init__(self, fitmodel = "onlineGP"):
        self.Tf = tf.Tf()
        self.planes = CurveFit.Planes()
        self.userHome = os.path.expanduser('~')
        self.GPOpt = open((self.userHome + '/tmp/GPOpt.txt'),'w')

        #bath create
        xlen = 500
        ylen = 500
        xtrapoints = 0#10
        xedgez = 0
        maxz = 20
        resmult = .25
        res = xlen*resmult
        seed = 145#9768#36344#145# #75638 boustrophedon fails to sweep this correctly as there is a crossover...
        manPoints = np.array([[0,200,10],
                          [100,175,10],
                          [200,125,10],
                          [300,100,10],
                          [400,100,10],
                          [400,200,10],
                          [400,250,10],
                          [400,300,10],
                          [400,400,10],
                          [300,350,10],
                          [200,350,7.5],
                          [100,300,0],
                          [0,400,0],
                          [0,300,0],
                          [50,250,0],
                          [100,240,0],
                          [150,230,0],
                          [200,220,0],
                          [250,210,0],
                          [300,200,0],
                          [300,300,0]
                          ])
        self.bath = BathymCreate.bathymCreate(xlen, ylen, xtrapoints, xedgez, maxz, res, seed)
        self.bath.createBasin(manPoints=manPoints, edgePoints=7)
        #self.bath.print3D(invert=True)
        #vessel
        #length = 1.2
        #width = 0.8
        self.velocity = 1 #m/s
        self.sensorFreq = 1.0 #hz
        self.CL = 1 #hz control loop freq
        self.maxTurnSpeed = math.pi/4 #radians/s
        SingleBeamAngle = -math.pi/2 #-math.pi/2 = straight down, 0 = straight forwards
        maxRange = maxz*3
        #sea surface
        #fX = 9999999#0.51 # frequency X axis
        #fY = 9999999 #0.6 # frequency Y axis
        #AX = 0.0000001#0.2 # Amplitude X axis
        #AY = 0.0000001#.2 # Amplitude Y axis
        #if (AX+AY) > xedgez:
        #    print "Sea surface depression bigger than min depth"
        #    exit()
        #self.sea = ShortCrestIrregularSea.SCISSinWave(fX, fY, AX, AY, length, width)
        #control
        self.targetDepth = 4.5
        self.boundBuff = 5
        self.r = 2.5#5*self.velocity
        self.initScanRadius = 5
        self.polyBondaries = np.array([[0, 0], [xlen, 0], [xlen, ylen], [0, ylen]])
        self.window = 50 # initialisation points
        #self.rmax = (xlen+ylen)/20
        self.searchRange = 2000
        self.duration = 5000 #seconds

        #self.dt = 1/self.sensorFreq
        self.dt = 1./self.CL
        self.dxy = self.velocity*self.dt
        self.obsCount = int(self.duration/self.dt)
        x0 = 250#xlen/2.0
        y0 = 350#ylen/2.0
        z0 = 0
        self.psi = 0#-math.pi*3/4 # initial heading
        self.posCG = np.array([[x0], [y0], [z0]])
        self.rotCG = self.Tf.rotFixedRPY(0.0, 0.0, self.psi)
        self.CGh = self.Tf.homogenousMatrix(self.rotCG, self.posCG)
        self.sb = SingleBeam.SingleBeam(self.bath, sensTheta=SingleBeamAngle, maxRange=maxRange)
        self.Cont = Controllers.Controllers(self.polyBondaries, self.dxy)
        # arrays for scanned points
        self.scannedPoints = np.empty([self.obsCount, 3])
        self.travelledPoints = np.empty([self.obsCount, 4])
        self.targetPoints = np.empty([self.obsCount, 3])

        #expMA
        EMAHalfLife = 5 #seconds
        self.EMAAlpha = -math.log(0.5)*self.dt/EMAHalfLife #note half life = ln(0.5)/(-self.EMAAlpha) in timesteps

        #counters
        self.tcounter = 0
        self.headingAdj = 0
        self.scounter = 0
        self.tpcounter = 0
        self.t = 0
        self.ptcounter = 0

        #circum navigation
        self.radiusCoverage = self.r #m
        self.pathtraveled = np.empty([self.obsCount, 2])#([self.obsCount/self.radiusCoverage, 2])
        self.ptBuff = int(self.radiusCoverage*25)
        self.inittheta = [1.0,1.0,1.0]#[25.0, 10.0, 0.00002] #[varf, charlen, varn]
        self.fitmodel = fitmodel
        self.bounds=[(0.25,500),(0.25,500),(0.000001,2)]
        #thetares 22.703044 ,  10.398308 ,  0.000033 LML 6404.345035 thetaStart [ 1.553  1.45   1.471] secs 110.197999954

        # lawnmower
        self.pathSpacing = 5 # = turn diameter m
        semiCirc = math.pi*self.pathSpacing/2
        semiCircTime = semiCirc/self.velocity
        self.lawnTurnSpeed = min(math.pi/semiCircTime, self.maxTurnSpeed) #radians/s
        gpHyper = open('./npy/gpHyperSimR2_5.txt','w') #clear file for initial write
        gpHyper.close()

    def initialScan(self):
        #create a set of waypoints, tracing a circle of radius rad, in time secs
        psi = self.Tf.getAngles(self.CGh)[2]
        center = [self.CGh[0, 3],self.CGh[1, 3]]
        for i in range(int(self.window)):
            psi = psi + math.pi*2/self.window
            e = center[0] + self.initScanRadius*math.cos(psi)
            n = center[1] + self.initScanRadius*math.sin(psi)
            self.posCG = np.array([[e], [n], [0]])
            self.rotCG = self.Tf.rotFixedRPY(0.0, 0.0, psi)
            self.CGh = self.Tf.homogenousMatrix(self.rotCG, self.posCG)
            self.bathPos = self.sb.getReadingsH(self.CGh)
            self.travelledPoints[self.tcounter, 0] = self.CGh[0, 3]
            self.travelledPoints[self.tcounter, 1] = self.CGh[1, 3]
            self.travelledPoints[self.tcounter, 2] = self.CGh[2, 3]
            self.travelledPoints[self.tcounter, 3] = self.psi
            if self.bathPos is not None:
                self.scannedPoints[self.scounter, 0] = self.bathPos[0]
                self.scannedPoints[self.scounter, 1] = self.bathPos[1]
                self.scannedPoints[self.scounter, 2] = self.bathPos[2]
                self.scounter += 1
            self.tcounter += 1
            #do 1 full rotation
            self.headingAdj = math.pi*2/self.window

        self.Cont.EMAHeading = self.psi
        self.Cont.onlineGP = OnlineGP.GP()
        X = self.scannedPoints[:self.scounter, 0:2]
        Y = self.scannedPoints[:self.scounter, 2]
        self.Cont.onlineGP.setData(X,Y)
        GPOptstr = 'init' +','+  str(self.inittheta)+  '\n'
        self.GPOpt.write(GPOptstr)
        opt = self.Cont.onlineGP.optimise(self.inittheta,self.bounds)
        GPOptstr = str(self.tcounter) +','+  str(opt[0])+ ','+ str(opt[1])  + '\n'
        self.GPOpt.write(GPOptstr)
        self.Cont.onlineGP.setTheta(opt[0])
        print "gp theta on initial points", opt[0]
        gpHyper = open('./npy/gpHyperSimR2_5.txt','a')
        gpHyper.write(str(self.tcounter) + ',' + str(opt[0][0]) + ',' +  str(opt[0][1]) + ',' + str(opt[0][2]) + '\n')
        gpHyper.close()
        self.Cont.onlineGP.fit()

    def contBoundScan(self):
        breakout = False
        foundCB = False
        timer = time.time()
        gpcount = self.tcounter
        while breakout == False:
        #for i in xrange(self.tcounter, self.obsCount):
            #this should be done in a separate thread in real world
            # however care will need to be taken on the fit step as we could be trying to add new points whilst it is
            # running
            if (self.tcounter-gpcount)%100 == 0:
                timer = time.time()
                print "optimising GP",
                theta = [self.Cont.onlineGP.var_f, self.Cont.onlineGP.charLen,  self.Cont.onlineGP.var_n]
                opt = self.Cont.onlineGP.optimise(theta,self.bounds)
                GPOptstr = str(self.tcounter) +','+  str(opt[0])+ ','+ str(opt[1])  + '\n'
                self.GPOpt.write(GPOptstr)
                self.Cont.onlineGP.setTheta(opt[0])
                print "gp theta recalc ", opt[0], ' time = ', time.time()-timer
                gpHyper = open('./npy/gpHyperSimR2_5.txt','a')
                gpHyper.write(str(self.tcounter) + ',' + str(opt[0][0]) + ',' +  str(opt[0][1]) + ',' + str(opt[0][2]) + '\n')
                gpHyper.close()
                timer = time.time()
                self.Cont.onlineGP.fit() #need to recalculate the Covariance matrix from scratch as we are changing the hyper-parameters
                print "gp fit time = ", time.time()-timer

            self.t += self.dt
            print self.t,
            self.CGh = np.dot(self.CGh, self.Tf.rotzh(self.headingAdj)) #heading
            self.psi = self.Tf.getAngles(self.CGh)[2]
            # if self.spinScan != True:
            self.CGh = np.dot(self.CGh, self.Tf.transxh(self.dxy)) #velocity
            #self.CGh = self.sea.getCGhSS(self.CGh, self.t) #seasurface
            tmp = round(self.t,2) % round(1/self.sensorFreq,2)
            #print tmp,
            if abs(tmp) < 0.01:
                self.bathPos = self.sb.getReadingsH(self.CGh)
                #print "CGh x y ", self.CGh[0, 3], self.CGh[1, 3],
                self.travelledPoints[self.tcounter, 0] = self.CGh[0, 3]
                self.travelledPoints[self.tcounter, 1] = self.CGh[1, 3]
                self.travelledPoints[self.tcounter, 2] = self.CGh[2, 3]
                self.travelledPoints[self.tcounter, 3] = self.psi
                if self.bathPos is not None:
                    self.scannedPoints[self.scounter, 0] = self.bathPos[0]
                    self.scannedPoints[self.scounter, 1] = self.bathPos[1]
                    self.scannedPoints[self.scounter, 2] = self.bathPos[2]
                    #print "bath", self.bathPos[0],self.bathPos[1], self.bathPos[2],
                    self.scounter += 1
            self.tcounter += 1
            self.fitPoints()
            desHead, r, ztmp, foundCB ,debugstr= self.Cont.findFollowContourBoundary(self.targetDepth,
                                                                      self.psi, self.Tf.posFromH(self.CGh),
                                                                      self.r, foundCB, 1)
            print debugstr,
            print "loop time", time.time() - timer
            timer = time.time()
            #print 'desHead, self.psi',desHead, self.psi
            headingErr = self.planes.clampAngle(desHead-self.psi)
            self.headingAdj = max(min(headingErr, self.maxTurnSpeed*self.dt), -self.maxTurnSpeed*self.dt)
            tpx = self.CGh[0, 3] + r*math.cos(desHead)
            tpy = self.CGh[1, 3] + r*math.sin(desHead)
            self.targetPoints[self.tpcounter, 0] = tpx
            self.targetPoints[self.tpcounter, 1] = tpy
            self.targetPoints[self.tpcounter, 2] = -ztmp
            # if self.spinScan==False:
            EMACosHeading = self.EMAAlpha*math.cos(self.psi) + (1-self.EMAAlpha)*math.cos(self.Cont.EMAHeading)
            EMASinHeading = self.EMAAlpha*math.sin(self.psi) + (1-self.EMAAlpha)*math.sin(self.Cont.EMAHeading)
            self.Cont.EMAHeading = math.atan2(EMASinHeading, EMACosHeading)
            self.tpcounter += 1
            if foundCB == True: #only start watching the boundary once we have hit it

                #work though path travelled
                if self.ptcounter == 0: #first point
                    print "*********************"
                    ptd = self.r +1
                else:
                    ptd = ((self.pathtraveled[self.ptcounter-1, 0]-self.CGh[0, 3])**2
                           + (self.pathtraveled[self.ptcounter-1, 1]-self.CGh[1, 3])**2)**0.5
                #only add points every half radius to keep data sparse
                if ptd > self.r/4.0:
                    #print self.ptcounter, self.CGh[0, 3], self.CGh[1, 3]
                    ptcounterMinBuff = self.ptcounter - self.ptBuff
                    if ptcounterMinBuff > 1: #ie we have more points in here than the buffer required
                        for i in range(ptcounterMinBuff):
                            ptdi = ((self.pathtraveled[i, 0]-self.CGh[0, 3])**2
                                    + (self.pathtraveled[i, 1]-self.CGh[1, 3])**2)**0.5
                            if breakout == False:
                                if ptdi < self.radiusCoverage:
                                    #we have completed the boundary, but keep going to see if there is a closer point
                                    breakout = True
                                    rejoinIdx = i
                                    minptdi = ptdi
                            else:
                                if ptdi < self.radiusCoverage and ptdi < minptdi:
                                    rejoinIdx = i
                                else:
                                    #we have found the closest breakout point
                                    print "boundary circumnavigation complete"
                                    break


                    self.pathtraveled[self.ptcounter, 0] = self.CGh[0, 3]
                    self.pathtraveled[self.ptcounter, 1] = self.CGh[1, 3]
                    self.ptcounter += 1
        self.pathtraveled = self.pathtraveled[rejoinIdx:self.ptcounter,:]

    def remDuplicates(self, data):
        #remove rows where vals in 0 and 1 (ie x and y co-ordinates) have already occured, as this breaks the GP
        obscount = data.shape[0]
        dups = []
        for i in range(obscount):
            for j in range(obscount):
                if i != j:
                    if data[i,0] == data[j,0] and data[i,1] == data[j,1]:
                        if j not in dups:
                            dups.append(i)
        data = np.delete(data,dups,0)
        return data

    def fitPoints(self):

        scannedPointsLocal = self.scannedPoints[:self.scounter,:]
        if self.fitmodel == 'onlineGP':
            if self.Cont.onlineGP == None:                      #input initial data
                X = self.scannedPoints[:self.scounter, 0:2]
                Y = self.scannedPoints[:self.scounter, 2]
                sigf_2 = self.gpsigf2
                sign_2 = self.gpsign2
                length = self.gpLength
                theta = [sigf_2, length,  sign_2]
                gp = onlineGP.GP()
                gp.setTheta(theta)
                gp.setData(X, Y)
                gp.fit()
                self.Cont.onlineGP = gp
            else:
                X = self.scannedPoints[self.scounter-1, 0:2][np.newaxis,:]
                Y = self.scannedPoints[self.scounter-1, 2]
                self.Cont.onlineGP.addFitObs(X,Y)
        else:                                               #add aditional point
            print "choose fit model from 'onlineGP'"
            exit()

if __name__ == '__main__':
    # initialise and run module

    #sbcb = SBContBound(fitmodel="gp")
    sbcb = SBContBound(fitmodel="onlineGP")
    sbcb.initialScan()
    print 'init done'
    fig = plt.figure()
    print sbcb.scannedPoints.shape
    #plt.plot(sbcb.scannedPoints[:sbcb.tcounter,0],sbcb.scannedPoints[:sbcb.tcounter,1])
    #plt.show()

    sbcb.contBoundScan()

    # add travelled points to bathymtery surface plot
    plt.figure
    #plt.plot(sbcb.travelledPoints[sbcb.window:sbcb.tcounter, 0],
    #         sbcb.travelledPoints[sbcb.window:sbcb.tcounter, 1],
    #         sbcb.travelledPoints[sbcb.window:sbcb.tcounter, 2])

    # this gives us a reduced bounding polygon where we only take points that are sbcb.radiusCoverage*2 apart
    plt.plot(sbcb.pathtraveled[:, 0],
             sbcb.pathtraveled[:, 1])
    print "number of reduced bounding polygon points = ", sbcb.ptcounter

    #plt.plot(sbcb.targetPoints[:sbcb.tpcounter, 0],
    #         sbcb.targetPoints[:sbcb.tpcounter, 1],
    #         sbcb.targetPoints[:sbcb.tpcounter, 2])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(sbcb.scannedPoints[:sbcb.scounter, 0],
               sbcb.scannedPoints[:sbcb.scounter, 1],
               sbcb.scannedPoints[:sbcb.scounter, 2]*(-1), c='r', marker='.')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    plt.title("scanned points")
    plt.show()
    np.save('./npy/scannedPointsSimR2_5',sbcb.scannedPoints[:sbcb.scounter,:])


    #
    # X = sbcb.scannedPoints[:sbcb.scounter, 0:2]
    # Y = sbcb.scannedPoints[:sbcb.scounter, 2]
    # theta = [sbcb.Cont.onlineGP.var_f,sbcb.Cont.onlineGP.charLen,sbcb.Cont.onlineGP.var_n ]
    # sbcb.Cont.onlineGP.setTheta(theta)
    # sbcb.Cont.onlineGP.setData(X, Y)
    # print "optimised GP theta and LML",  sbcb.Cont.onlineGP.optimise(theta)
    # opt = sbcb.Cont.onlineGP.optimiseRandStart(num=10, messages=True)
    # print opt
    # sbcb.Cont.onlineGP.setTheta(opt[0])
    # sbcb.Cont.onlineGP.fit()
    # GPOptstr = 'full' +','+  str(opt[0])+ ','+ str(opt[1]) + '\n'
    # sbcb.GPOpt.write(GPOptstr)
    # sbcb.GPOpt.close()
    #
    # print "query model for estimated depth vs real"
    # resolution = 10
    # len = 500
    # obs = len/resolution
    # estSurface = np.empty([obs*obs,3])
    # errSurface = np.empty([obs*obs])
    # stdDevSurface = np.empty([obs,obs])
    # # populate grid
    # for i in range(obs):
    #     for j in range(obs):
    #         estSurface[i*obs+j, 0] = i*resolution
    #         estSurface[i*obs+j, 1] = j*resolution
    #         if sbcb.Cont.tck is not None:
    #             estSurface[i*obs+j,2] = sbcb.Cont.getDepth(i*resolution, j*resolution)
    #         elif sbcb.Cont.gp is not None:
    #             depth, MSE = sbcb.Cont.gp.predict([[i*resolution, j*resolution]], eval_MSE=True)
    #             estSurface[i*obs+j, 2] = depth
    #             stdDevSurface[i, j] = np.sqrt(MSE) #std deviation
    #         else:
    #             print i, np.array([[i*resolution, j*resolution]])
    #             depth, var = sbcb.Cont.onlineGP.pred(np.array([[i*resolution, j*resolution]]),varinfo = True)
    #             estSurface[i*obs+j, 2] = depth
    #             stdDevSurface[i, j] = np.sqrt(var) #std deviation
    #         errSurface[i*obs+j] = estSurface[i*obs+j, 2] - sbcb.bath.queryDepth(i*resolution, j*resolution)
    #
    # print "plot data"
    #
    # fig2 = plt.figure()
    # ax = fig2.add_subplot(111, projection='3d')
    # ax.scatter(estSurface[:,0],
    #            estSurface[:,1],
    #            estSurface[:,2]*(-1), c='r', marker='.')
    # ax.set_xlabel('x')
    # ax.set_ylabel('y')
    # plt.title("est points")
    #
    # fig3 = plt.figure()
    # ax = fig3.add_subplot(111, projection='3d')
    # ax.scatter(estSurface[:,0],
    #            estSurface[:,1],
    #            errSurface[:], c='r', marker='.')
    # ax.set_xlabel('x')
    # ax.set_ylabel('y')
    # plt.title("err")
    #
    # if sbcb.Cont.gp is not None:
    #     ch = Charts.charts()
    #     print "plot variances of GP"
    #     x = np.linspace(0, len, obs)
    #     y = np.linspace(0, len, obs)
    #     ch.surface3D(x,y,stdDevSurface,title="std dev", rstride=resolution, cstride=resolution)
    # plt.show()
    #
    #
    #
    #
    #
    #
    #
    #
    #
