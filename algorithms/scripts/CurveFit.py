__author__ = 'troy'

from scipy.interpolate import bisplrep, bisplev
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import Transformations as tf
import math

class curveFit():
    def __init__(self, x, y, z, res, rstride = 5, cstride = 5):
        self.x = x
        self.y = y
        self.z = z
        self.res = res
        self.minx = min(x)
        self.maxx = max(x)
        self.miny = min(y)
        self.maxy = max(y)
        self.rstride = rstride
        self.cstride = cstride
        self.grid = None
        self.tck = None
        if len(self.x) == len(self.y) == len(self.z):
            self.pointCnt = len(self.x)
        else:
            print "inconsistent number of elements in x, y and z arrays"
            print len(self.x), len(self.y), len(self.z)

    def createRegGrid(self):
        xnew, ynew = np.mgrid[self.minx:self.maxx:complex(self.res), self.miny:self.maxy:complex(self.res)] #note the j represents complx number and in this instance means step 20 times with the stop inclusive
        self.grid = bisplev(xnew[:, 0], ynew[0, :], self.tck)

    def plotPoints(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for i in range(self.pointCnt):
            ax.scatter(self.xp[i], self.yp[i], self.zp[i], c='b')

        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        plt.show

    def fitSurface(self):
        self.tck = bisplrep(self.x, self.y, self.z)
        return self.tck

    def clamp(self, minz, maxz):
        self.grid[self.grid > maxz] = maxz
        self.grid[self.grid < minz] = minz


    def print3D(self, invert=False):
        #surface is rotated by 90 degrees somehow !!!!!!!!!!!!!!!!!
        print "self.minx, self.maxx, self.miny, self.maxy", self.minx, self.maxx, self.miny, self.maxy
        x = np.linspace(self.minx, self.maxx, self.res)
        y = np.linspace(self.miny, self.maxy, self.res)
        X, Y = np.meshgrid(x, y)
        fig=plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        if invert==False:
            ax.plot_surface(Y, X, self.grid, rstride=self.rstride, cstride=self.cstride, cmap=cm.cool)
        else:
            ax.plot_surface(Y, X, self.grid*(-1), rstride=self.rstride, cstride=self.cstride, cmap=cm.cool)
        ax.set_xlabel('Easting (m)')
        ax.set_ylabel('Northing (m)')
        ax.set_zlabel('Depth (m)')

        #ax.set_zlim3d(0.0,10.0)

class Planes():
    def __init__(self):
        self.Tf = tf.Tf()
    def fitPlane(self,points):
        #expects point in the following format, will work on any number of dimensions
        #points = np.array([[1,1,3], [2,1,3], [3,1,3], [1,-1,2], [2,-1,2], [3,-1,2]],float)  3D
        #points = np.array([[1,1], [-1,1], [1,-1], [-1,-1]],float) 2D
        # returns centroid of the points which is a point on the plane a and normal to that plane
        p = points.mean(axis=0)
        M = points-p
        u, s, vH = np.linalg.svd(M)
        #return p, v[:,-1]
        #note whilst I want to return the last column of v, it is actually v.H that is returned by np.linalg.svd
        #so rather than transpose and then take the last column I take the last row
        v = vH[-1,:]
        #flip to make Z axis positive
        sgn = np.sign(v[2])
        v = v*sgn
        return p, v

    def normalise3dVector(self,v):
        a = v[0]
        b = v[1]
        c = v[2]
        denom = math.sqrt(a*a+b*b+c*c)
        v = v/denom
        return v

    def getAngleCosine(self, v):
        #feed in a vector and return the angles to  x,y,z axes
        # http://www.geom.uiuc.edu/docs/reference/CRC-formulas/node52.html
        a = v[0]
        b = v[1]
        c = v[2]
        denom = np.sqrt(a*a+b*b+c*c)
        alpha_phi = math.acos(a/denom)
        beta_theta = math.acos(b/denom)
        gamma_psi = math.acos(c/denom)
        return np.array([alpha_phi, beta_theta, gamma_psi])

    def XYplaneFromH(self,h):
        p = self.Tf.posFromH(h)
        v = self.Tf.posFromH(np.dot(h,self.Tf.transzh(1)))[:,0]
        v = self.normalise3dVector(v)
        return p,v

    def getRotBetweenPlanes(self,v0,v1):
        # input 2 normals, return rotation matrix
        ang, axis = self.Tf.getAngleAxis(v0,v1)
        rot = self.Tf.angleAxisToRotMatrix(axis,ang)
        angles = self.Tf.getAngles(rot)
        # Yaw arbitrary thus set to zero
        rot2 = self.Tf.rotFixedRPY(angles[0],angles[1],0.0)
        #debugging code - proving yaw is arbitrary for this purpose
        #print angles[0]/math.pi*180,angles[1]/math.pi*180
        #h0 = self.Tf.homogenousMatrix(rot,self.Tf.posMatrix(0,0,0))
        #h1 = np.dot(h0,self.Tf.transzh(1))
        #p1 = self.Tf.posFromH(h1)
        #print p1[0,0], p1[1,0], p1[2,0],
        #h2 = np.dot(self.Tf.transzh(1),h0)
        #p2 = self.Tf.posFromH(h2)
        #print p2[0,0], p2[1,0], p2[2,0]

        #h2 = np.dot(self.Tf.transzh(1),self.Tf.homogenousMatrix(rot2,self.Tf.posMatrix(0,0,0)))
        #p2 = self.Tf.posFromH(h2)
        #print p2[0,0], p2[1,0], p2[2,0]

        return rot2

    def getRotFromFlat(self,v1):
        v0 = self.Tf.transz(1)[:,0]
        return self.getRotBetweenPlanes(v0,v1)

    def getSlopeDirectionofPlane(self,v1):
        #Get angle between normal of plane and normal of vertical = diff between horizontal planes
        v0 = self.Tf.transz(1)[:,0]
        slope = self.Tf.angleTwoVectors(v0,v1)
        # Project to XY and the point of the normal on XY will be the lowest(as Z defined +ve down)
        # point on a circle of diameter |XY| around the origin
        # Get angle of this 2D vector to the X axis and we have the direction of steepest slope, up direction (to smaller number)
        direction = math.atan2(v1[1],v1[0])
        return slope, direction

    def solveAngleForDepth(self,p,v,x,y,targetz,r,heading,upDirection):#, depthstats = False):
        #solves for angle to get to a given depth on the half circle currently facing
        # uses bisection method
        eps=1.0e-6
        downDirection = self.clampAngle(upDirection+math.pi)
        mindepth = self.depthTheta(p,v,x,y,r,upDirection)
        maxdepth = self.depthTheta(p,v,x,y,r,downDirection)

        #theta = self.clampAngle(heading + (np.random.binomial(1,0.5)-0.5)/30) # add -+ 0.95 degrees to the heading to randomly chose left or right when near centre
        ztmp = self.depthTheta(p,v,x,y,r,heading) #depth at distance R from current pos at heading theta
        downDepth = self.depthTheta(p,v,x,y,0.00001,heading)
        mtheta = heading
        if targetz>maxdepth:
            mtheta =  downDirection
            ztmp = self.depthTheta(p,v,x,y,r,mtheta)
        elif targetz<mindepth:
            mtheta = upDirection
            ztmp = self.depthTheta(p,v,x,y,r,mtheta)
        else:
            #set down direction to zero, search between zero and pi
            #dtheta = 0
            #utheta = math.pi
            #mtheta = theta-downDirection
            dtheta = downDirection
            utheta = upDirection
            #mtheta = self.clampAngle((dtheta +utheta)/2)
            err = eps +1
            while err > eps:
                if targetz == ztmp:
                    pass
                elif targetz < ztmp:
                    dtheta = mtheta
                    mtheta = (utheta+mtheta)/2
                else:
                    utheta = mtheta
                    mtheta = (dtheta+mtheta)/2
                #theta = mtheta+downDirection
                ztmp = self.depthTheta(p,v,x,y,r,mtheta)
                err = abs(targetz-ztmp)
            #check solved correct side of the circle, within 90 deg either side of current heading
            if mtheta <(heading-math.pi/2) or mtheta>(heading+math.pi/2):
                mtheta = self.circleAngleFlip(mtheta,upDirection)
                ztmp = self.depthTheta(p,v,x,y,r,mtheta)
                #print "flip it"
        #print "Plane est Depth at r,theta", ztmp,
        # if depthstats == False:
        #     return mtheta
        # else:
        #     return mtheta, ztmp, downDepth
        return mtheta, ztmp, downDepth

    def depthTheta(self,p,v,x, y, r,theta):
        #theta = self.clampAngle(theta-math.pi/2)
        xp = x + r*math.cos(theta)
        yp = y +r*math.sin(theta)
        return self.depth(p,v,xp,yp)

    def depth(self,p,v,x,y):
        #given parameters of plane and an x and y, return z
        return (np.dot(v,p)-v[0]*x-v[1]*y)/v[2]

    def clampAngle(self,theta):
        return (theta+math.pi)%(2*math.pi)-math.pi

    def circleAngleFlip(self,angle,pole):
        #return mirror image angle
        return self.clampAngle(angle-(angle-pole)*2)


if __name__ == '__main__':
    from math import acos, pi, sqrt
    Tf = tf.Tf()
    # test getRotationBetweenPlanes
    refX = 0*180/np.pi
    refY = 0*180/np.pi
    refZ = 0*180/np.pi
    h0 = Tf.homogenousMatrix(Tf.rotFixedRPY(refX,refY,refZ),Tf.posMatrix(0,0,0))
    v0 = Tf.posFromH(np.dot(h0,Tf.transzh(1)))[:,0]
    print v0, "v0"
    #points = np.array([[-1,1,3], [1,1,3], [-1,-1,7], [1,-1,2]],float)
    #points = np.array([[-1,1,.5], [0,1,.5], [1,1,.5], [-1,-1,-0.5], [0,-1,-0.5], [1,-1,-0.5]],float)
    points = np.array([[-1,1,.5], [0,1,.7], [1,1,1], [-1,-1,-0.5], [0,-1,-0.6], [1,-1,-1.5]],float)
    Planes = Planes()
    p1, v1 = Planes.fitPlane(points)
    #adj for flipping of normal vector, ie if Z co-ordinates are -ve as unless the boat flips they will always be positive
    # if v1[2] < 0:
    #     hv1 = np.dot(hv1,Tf.transzh(2))
    #     v1 = Tf.posFromH(hv1)[:,0]
    #     v1 = Planes.normalise3dVector(v1)
    #     print "************* Z FLIPPED ****************"
    rot = Planes.getRotBetweenPlanes(v0,v1)
    print rot, "rot"
    print v1, "original v1"
    print np.dot(rot,v0), "reprojected v1 from v0 and calculated rotation matrix"

    print "\n or using full homongenous matricies \n"
    print h0, "h0"
    h1r = Tf.homogenousMatrix(rot,Tf.posMatrix(0,0,0))
    h0p = np.dot(h0,h1r)
    print h0p, "h0p"
    h0pdz = np.dot(h0p,Tf.transzh(1))
    print h0pdz, "h0pdz"

