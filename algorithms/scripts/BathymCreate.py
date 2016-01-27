__author__ = 'troy'

""" Returns a array (x,y) of depth values
"""
import numpy as np
from scipy.interpolate import bisplev
import CurveFit

class bathymCreate():
    def __init__(self,xlen,ylen,xtrapoints,minz,maxz, res, seed=None):
        self.xlen = xlen
        self.ylen = ylen
        self.xtrapoints = xtrapoints
        self.minz = minz
        self.maxz = maxz
        self.tck = None
        self.xp = None
        self.yp = None
        self.zp = None
        self.regbath = None
        self.res = res
        if seed != None:
            np.random.seed(seed)
        self.cf = None
        self.planes = None

    def createBasinPoints(self, flat=False, manPoints=[], edgePoints = 10):
        #creates a basin with sides fixed on the x axis at minz, and a spline over xtrapoints in the range
        # minz to max z
        mpCount = len(manPoints)
        #edgePoints = self.xlen
        edgeDisc = self.xlen/(edgePoints-1)
        pointCnt = edgePoints*2 + self.xtrapoints
        self.xp = np.empty(pointCnt)
        self.yp = np.empty(pointCnt)
        self.zp = np.empty(pointCnt)
        #boundary points on x edge
        for i in range(edgePoints):
            self.xp[i] = i*edgeDisc
            self.yp[i] = 0
            self.zp[i] = self.minz
            self.xp[i+edgePoints] = i*edgeDisc
            self.yp[i+edgePoints] = self.ylen#-1
            self.zp[i+edgePoints] = self.minz

        if flat == False and self.xtrapoints != 0:
            for i in range(self.xtrapoints):
                self.xp[i+edgePoints*2] = np.random.random_integers(1,self.xlen-1)
                self.yp[i+edgePoints*2] = np.random.random_integers(1,self.ylen-1)
                self.zp[i+edgePoints*2] = np.random.uniform(self.minz,self.maxz/2)
                #print self.xp[i+self.xlen*2], self.yp[i+self.xlen*2], self.zp[i+self.xlen*2]
            #else:
            #    self.xp[i+pCntedge] = np.random.random_integers(1,self.xlen-1)
            #    self.yp[i+pCntedge] = np.random.random_integers(1,self.ylen-1)
            #    self.zp[i+pCntedge] = self.minz

        #print np.vstack([self.xp[:,np.newaxis],  manPoints[:, 0][:,np.newaxis]])[0]

        if mpCount != 0:
            self.xp = np.vstack([self.xp[:,np.newaxis], manPoints[:, 0][:,np.newaxis]])[:, 0]
            self.yp = np.vstack([self.yp[:,np.newaxis], manPoints[:, 1][:,np.newaxis]])[:, 0]
            self.zp = np.vstack([self.zp[:,np.newaxis], manPoints[:, 2][:,np.newaxis]])[:, 0]
            print self.xp.shape


    def createBasinPointsPlane(self,p,v):
        import matplotlib.pyplot as plt
        self.planes =CurveFit.Planes()
        pointCnt = 200
        self.xp = np.empty(pointCnt)
        self.yp = np.empty(pointCnt)
        self.zp = np.empty(pointCnt)
        for i in range(pointCnt):
            self.xp[i] = np.random.random_integers(0,self.xlen)
            self.yp[i] = np.random.random_integers(0,self.ylen)
            self.zp[i] = self.planes.depth(p,v,self.xp[i],self.yp[i])
        self.cf = CurveFit.curveFit(self.xp, self.yp, self.zp, self.res)
        self.tck = self.cf.fitSurface()
        self.cf.createRegGrid()
        #self.cf.clamp(self.minz,self.maxz)
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # ax.scatter(self.xp, self.yp, self.zp, c='r', marker='.')
        # ax.set_xlabel('x')
        # ax.set_ylabel('y')

    def printPoints(self):
        print len(self.xp), len(self.yp), len(self.zp)
        for i in range(len(self.xp)):
            print self.xp[i], self.yp[i], self.zp[i]

    def queryDepth(self, xq, yq):
        #return 1 data point
        depth = bisplev(xq, yq, self.tck)
        if np.isnan(depth):
            print "NaN returned for depth"
        else:
            return max(self.minz,min(depth, self.maxz))

    def createBasin(self, flat = False, manPoints = [], edgePoints = 10):
        self.createBasinPoints(flat, manPoints, edgePoints)
        self.cf = CurveFit.curveFit(self.xp, self.yp, self.zp, self.res)
        self.tck = self.cf.fitSurface()
        self.cf.createRegGrid()
        self.cf.clamp(self.minz,self.maxz)

    def createBasinManual(self):
        self.createBasinPoints()
        # x = [294.0, 141.0, 252.0, 426.0, 178.0, 19.0, 499.0, 63.0, 321.0, 162.0]
        # y = [422.0, 408.0, 146.0, 264.0, 430.0, 19.0, 431.0, 284.0, 63.0, 56.0]
        # z = [3.81,  6.43,   6.00, 6.97,  4.4,   2.49, 0.1,  2.31,  4.98, 0]
        x = [29.40, 14.10, 25.20, 42.60, 17.80, 1.90, 49.90, 6.30, 32.10, 16.20]
        y = [42.20, 40.80, 14.60, 26.40, 43.00, 1.90, 43.10, 28.40, 6.30, 5.60]
        z = [3.81,  6.43,   6.00, 6.97,  4.4,   2.49, 0.1,  2.31,  4.98, 0]
        #fig3 = plt.figure()
        #ax = fig3.add_subplot(111, projection='3d')
        #ax.scatter(x[:],y[:],z[:], c='r', marker='.')
        for i in range(self.xtrapoints):
        #    print i, self.xp[i+self.xlen*2], self.yp[i+self.xlen*2], self.zp[i+self.xlen*2]
            self.xp[i+self.xlen*2] = x[i]
            self.yp[i+self.xlen*2] = y[i]
            self.zp[i+self.xlen*2] = z[i]
        #    print i, self.xp[i+self.xlen*2], self.yp[i+self.xlen*2], self.zp[i+self.xlen*2]
        self.cf = CurveFit.curveFit(self.xp, self.yp, self.zp, self.res)
        self.tck = self.cf.fitSurface()
        self.cf.createRegGrid()
        self.cf.clamp(self.minz,self.maxz)

    def print3D(self,invert=False):
        self.cf.print3D(invert)

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    xlen = 500
    ylen = 500
    xtrapoints = 0
    xedgez = 0
    maxz = 20
    resmult = .25
    res = xlen*resmult
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
    print manPoints[:,0].shape

    bath = bathymCreate(xlen, ylen, xtrapoints, xedgez, maxz, res, 145)#145#36344#987
    bath.createBasin(manPoints = manPoints, edgePoints = 7)
    #bath.createBasinManual()
    bath.print3D(invert=True)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(bath.xp,bath.yp,bath.zp*(-1))


    print "bathqueryres", "x", "y", "z"
    for i in range(5):
        xq = np.random.uniform(0,xlen)
        yq = np.random.uniform(0,ylen)
        print xq, yq, bath.queryDepth(xq,yq)
    print bath.queryDepth(25,25)
    plt.show()
