from __future__ import division

__author__ = 'Troy Wilson'

import math
import CurveFit
import numpy as np

class Intersections():
    def __init__(self):
        self.planes = CurveFit.Planes()
        self.eps = 1e-06

    def segmentsCrossCart(self,x1, y1, x2, y2, x3, y3, x4, y4, bool = False, pad = 0):
        # two point form
        # returns true if segments cross, and the crossing point of the (infinite length) lines
        # crossing point for parallel line is inf
        px1, py1, theta1, r1 = self.cartestianToPolar(x1,y1,x2,y2)
        px2, py2, theta2, r2 = self.cartestianToPolar(x3,y3,x4,y4)
        r1 += pad
        r2 += pad
        crossing, x,y = self.segmentsCrossPolar(px1,py1,theta1,r1,px2,py2,theta2,r2)
        if bool == False:
            return crossing, x,y
        else:
            return crossing

    def segmentsCrossPolar(self,x1,y1,theta1, r1, x2,y2,theta2, r2):
        # returns true if segments cross, and the crossing point of the (infinite length) lines
        # crossing point for parallel line is inf

        if abs(theta1 - theta2) < self.eps or abs(abs(self.planes.clampAngle(theta1+math.pi)) - theta2) < self.eps: # parallel lines
            # just return false, don't really care. proper solution would be to return the end points of the overlapping segments
            return False, np.inf, np.inf
        else:
            if abs(abs(theta1) - math.pi/2) < self.eps: #line is vertical and tantheta1 = +/-inf
                x = x1
                y = np.tan(theta2)*(x-x2)+y2
            elif abs(abs(theta2) - math.pi/2) < self.eps: #line is vertical and tantheta2 = +/-inf
                x = x2
                y = np.tan(theta1)*(x-x1)+y1
            elif abs(abs(theta2) - math.pi) < self.eps or abs(theta2)<self.eps: #line is horizontal and tantheta2 = +/-inf
                y = y2
                #print "Lines 45", np.tan(theta1), theta1, theta2
                x = (y-y1)/np.tan(theta1)+x1
            elif abs(abs(theta1) - math.pi) < self.eps or abs(theta1)<self.eps: #line is horizontal and tantheta1 = +/-inf
                y = y1
                x = (y-y2)/np.tan(theta2)+x2
            else:
                tanTheta1 = np.tan(theta1)
                tanTheta2 = np.tan(theta2)
                x = (y2-y1 + x1*tanTheta1 - x2*tanTheta2)/(tanTheta1 - tanTheta2)

                y = round((x-x1),8)*tanTheta1+y1
            # now check if these crossing points are within the line segments
            x1e = x1 + r1*np.cos(theta1)
            y1e = y1 + r1*np.sin(theta1)
            x2e = x2 + r2*np.cos(theta2)
            y2e = y2 + r2*np.sin(theta2)
            t1 = self.between(x,x1,x1e)
            t2 = self.between(y,y1,y1e)
            t3 = self.between(x,x2,x2e)
            t4 = self.between(y,y2,y2e)
            if (self.between(x,x1,x1e) and self.between(x,x2,x2e) and
                self.between(y,y1,y1e) and  self.between(y,y2,y2e)):
                crossing = True
            else:
                crossing = False
            return crossing, round(x,16), round(y,16)

    def segmentsCrossCartPolar(self,x1,y1,x2,y2,x3,y3,theta,r):
        x1, y1, theta1, r1 = self.cartestianToPolar(x1,y1,x2,y2)
        return self.segmentsCrossPolar(x1,y1,theta1,r1,x3,y3,theta,r)

    def coincedentSegmentsCross(self, x1,y1,x2,y2,x3,y3,x4,y4):
        theta1 = math.atan2(y2-y1,x2-x1)
        theta3 = math.atan2(y4-y3,x4-x3)
        #Check if parallel
        if abs(theta1)-abs(theta3) < self.eps:
            #test if end points of line 1 are on line 2
            if self.isPointOnLine(x3,y3,x4,y4,x1,y1):
                return True
            elif self.isPointOnLine(x3,y3,x4,y4,x2,y2):
                return True
            else:
                return False
        else:
            return False

    def segmentCrossRayPolar(self,x1,y1,theta1, r1, x2,y2,theta2,):
        tanTheta1 = np.tan(theta1)
        tanTheta2 = np.tan(theta2)
        if abs(tanTheta1) == abs(tanTheta2): # parallel lines
            # just return false, don't really care. proper solution would be to return the end points of the overlapping segments
            return False, np.inf, np.inf
        else:
            x = (y2-y1 + x1*tanTheta1 - x2*tanTheta2)/(tanTheta1 - tanTheta2)
            if abs(tanTheta1)<abs(tanTheta2):
                y = (x-x1)*tanTheta1+y1
            else:
                y = (x-x2)*tanTheta2+y2
            # now check if these crossing points are within the line segments
            x1e = x1 + r1*np.cos(theta1)
            y1e = y1 + r1*np.sin(theta1)
            if self.between(x,x1,x1e) and self.between(y,y1,y1e):
                crossing = True
            else:
                crossing = False
            return crossing, round(x,16), round(y,16)

    def segmentCrossRayCart(self,x1,y1,x2,y2,px,py,theta):
        # returns true if segments cross, and the crossing point of the (infinite length) lines
        # crossing point for parallel line is inf
        px0,py0,theta0,r0 = self.cartestianToPolar(x1,y1,x2,y2)
        crossing, x, y = self.segmentCrossRayPolar(px0,py0,theta0,r0,px,py,theta)
        return crossing, x, y

    def segmentPolarCrossPolyXY(self, poly, px, py, theta, r, bump=0):
        if bump!=0:
            px += r*bump*math.cos(theta)
            py += r*bump*math.sin(theta)
            r = r*(1-bump)
        #Return true if the segment crosses any edges of the polygon
        edgeCount = len(poly)
        crossing = False
        retX, retY = np.inf, np.inf
        minDist = 9e16
        for i in xrange(edgeCount):
            j = (i+1) % edgeCount
            x1 = poly[i,0]
            y1 = poly[i,1]
            x2 = poly[j,0]
            y2 = poly[j,1]
            c, x, y = self.segmentsCrossCartPolar(x1,y1,x2,y2,px,py,theta,r)
            if c == True:
                dist = math.sqrt((x-px)*(x-px)+(y-py)*(y-py))
                crossing = True
                if dist<minDist:
                    minDist = dist
                    retX = x
                    retY = y
        return crossing, retX, retY

    def segmentPolarCrossPoly(self, poly, px, py, theta, r,bump=0):
        crossing, x, y = self.segmentPolarCrossPolyXY(poly, px, py, theta, r, bump = bump)
        return crossing

    def segmentCartCrossPoly(self,poly, x1, y1, x2, y2,bump = False):
        x1, y1, theta, r = self.cartestianToPolar(x1, y1, x2, y2)
        if bump == True:
            x1 += math.cos(theta)*0.0001
            y1 += math.sin(theta)*0.0001
        crossing, x, y = self.segmentPolarCrossPolyXY(poly, x1, y1, theta, r)
        return crossing

    def segmentCartCrossPolyXY(self,poly, x1, y1, x2, y2,extend = 0):
        x1, y1, theta, r = self.cartestianToPolar(x1, y1, x2, y2)
        r += extend
        crossing, x, y = self.segmentPolarCrossPolyXY(poly, x1, y1, theta, r)
        return crossing, x, y

    def polyIdDistDir(self,poly,i,j):
        # making asumption here that we want to go the shortest direction around the polygon in testing the
        # intermediate segments
        n = poly.shape[0]
        jFwd = (j-i)%n
        if jFwd > n/2.:
            jBwd = (i-j)%n
            return jBwd + 1, -1
        else:
            return jFwd + 1, 1

    def segmentCartInOrOnPoly(self,poly,x1,y1,x2,y2):
        # test whether a segment is within on on a polygon
        # Note currently inefficent as looping through poly multiple times
        if not self.segmentCartCrossPoly(poly,x1,y1,x2,y2):
            return True
        else:
            p1Stat, p1Grad, p1i, p1j = self.isPointOnPolygonDetailed(x1,y1,poly)
            p2Stat, p2Grad, p2i, p2j = self.isPointOnPolygonDetailed(x2,y2,poly)
            if p1Stat and p2Stat: #both point on boundary
                if p1i == p2i and p1j == p2j: #both point on same segment
                    return True
                else:   # iterate through the segments and check we are on all of them. if we are off any of them,
                        # check the segment between where we went off and came back on again is inside the poly
                    m = len(poly)
                    n, n_iter = self.polyIdDistDir(poly,p1i,p2i)
                    segsStat = np.zeros([n,2])
                    if n_iter==-1:
                        iterList = reversed(range(n))
                    else:
                        iterList = range(n)
                    for k in iterList:
                        i = (p1i + k)%m
                        j = (p1i + k + 1*n_iter)%m
                        segSStat = self.isPointOnPolygon(x1,y1,np.array([[poly[i,0],poly[i,1]],[poly[j,0],poly[j,1]]]))
                        segEStat = self.isPointOnPolygon(x2,y2,np.array([[poly[i,0],poly[i,1]],[poly[j,0],poly[j,1]]]))
                        # if both points on the line then return true
                        segsStat[k,0] = int(segSStat) #cast bool to int
                        segsStat[k,1] = int(segEStat) #cast bool to int
                    if np.sum(segsStat) == 2*n: #They were all on the line
                        return True
                    else:
                        grad = math.atan2(y2-y1,x2-x1)
                        x1b = x1 + math.cos(grad)*self.eps*2 # need to bump this by eps*2 as the isPointInPolygon
                        y1b = y1 + math.sin(grad)*self.eps*2 # pads the point by eps already
                        x2b = x2 - math.cos(grad)*self.eps*2
                        y2b = y2 - math.sin(grad)*self.eps*2
                        psbStat = self.isPointInPolygon(x1b,y1b,poly)
                        pebStat = self.isPointInPolygon(x2b,y2b,poly)
                        if psbStat == False or pebStat == False:
                            return False
                        return True
            #if one point on poly and the other in the poly then srhink the point on the poly towards the other point
            # else shrink it away and check that segment doesn't cross the poly
            elif p1Stat == True:
                grad = math.atan2(y2-y1,x2-x1)
                if self.isPointInPolygon(x2,y2,poly):
                    x1b = x1 + math.cos(grad)*self.eps*2 # need to bump this by eps*2 as the isPointInPolygon
                    y1b = y1 + math.sin(grad)*self.eps*2 # pads the point by eps already
                else:
                    x1b = x1 - math.cos(grad)*self.eps*2 # need to bump this by eps*2 as the isPointInPolygon
                    y1b = y1 - math.sin(grad)*self.eps*2 # pads the point by eps already
                if not self.segmentCartCrossPoly(poly,x1b,y1b,x2,y2):
                    return True
            elif p2Stat == True:
                grad = math.atan2(y2-y1,x2-x1)
                if self.isPointInPolygon(x1,y1,poly):
                    x2b = x2 - math.cos(grad)*self.eps*2
                    y2b = y2 - math.sin(grad)*self.eps*2
                else:
                    x2b = x2 + math.cos(grad)*self.eps*2
                    y2b = y2 + math.sin(grad)*self.eps*2
                if not self.segmentCartCrossPoly(poly,x1,y1,x2b,y2b):
                    return True
        return False


    def rayCrossPoly(self, poly, px, py, theta):
        # returns the indicies of the verticies of the closest edge an infinite ray crosses
        # requires the verticies in poly to be ordered (direction irrelevant)
        edgeCount = len(poly)
        minD = 99999999
        v1 = None
        v2 = None
        for i in xrange(edgeCount):
            j = (i+1) % edgeCount
            x1 = poly[i,0]
            y1 = poly[i,1]
            x2 = poly[j,0]
            y2 = poly[j,1]
            crossing, x, y = self.segmentCrossRayCart(x1,y1,x2,y2,px,py,theta)
            #check the crossing point is not behind us
            if crossing == True:
                crossingHeading = math.atan2(y-py,x-px)
                if abs(theta-crossingHeading)<.01:
                    d = ((x-px)**2+(y-py)**2)**0.5
                    if d < minD:
                        minD = d
                        v1 = i
                        v2 = j
        return v1, v2

    def rayCrossPolyLoc(self, poly, px, py, theta, rp = None):
        # returns the crossing point of the closest edge it crosses (or would cross if r infinite)
        # requires the verticies in poly to be ordered (direction irrelevant)
        edgeCount = len(poly)
        minD = 99999999
        xret = None
        yret = None
        for i in xrange(edgeCount):
            j = (i+1) % edgeCount
            x1 = poly[i,0]
            y1 = poly[i,1]
            x2 = poly[j,0]
            y2 = poly[j,1]
            if rp == None:
                crossing, x, y = self.segmentCrossRayCart(x1,y1,x2,y2,px,py,theta)
            else:
                crossing, x, y = self.segmentsCrossCartPolar(x1,y1,x2,y2,px,py,theta,rp)
            if crossing == True:
                d = ((x-px)**2+(y-py)**2)**0.5
                if d < minD:
                    minD = d
                    xret = x
                    yret = y
        return xret, yret, minD

    def intersectionCircleSegment(self, x0, y0, r, x1, y1, x2, y2):
        # crop intersection points to segment
        icl = self.intersectionCircleLine(x0, y0, r, x1, y1, x2, y2)
        if icl.size == 4:
            for i in range(2):
                icl[i,0] = self.bound(icl[i,0],x1,x2)
                icl[i,1] = self.bound(icl[i,1],y1,y2)
        else:
            icl = np.array([])
        return icl

    def intersectionCirclePoly(self, poly, x0, y0, r):
        #assuming only 2 intersections
        #returns the two intersecting poitns ordered by the first point where adding to the angle would put you inside the poly
        #each point returns as [x, y, angle]
        icp = np.zeros([2,3])
        icpRet = np.zeros([2,3])
        smallRotInPoly = np.zeros([2])
        pCount = 0
        for i in range(len(poly)):
            j = (i+1) % len(poly)
            icl = self.intersectionCircleLine(x0, y0, r, poly[i,0], poly[i,1],poly[j,0], poly[j,1])
            if icl.size == 4: #!=4 means no an intersection with the infinite line
                for k in range(2):
                    if self.between(icl[k,0], poly[i,0] ,poly[j,0]) and self.between(icl[k,1], poly[i,1], poly[j,1]):
                        #ray intersects segment
                        angle = math.atan2(icl[k,1]-y0, icl[k,0]-x0)
                        icp[pCount] = [icl[k,0],icl[k,1],angle]
                        xt = x0 + r*math.cos(angle+.1)
                        yt = y0 + r*math.sin(angle+.1)
                        smallRotInPoly[pCount] = self.isPointInPolygon(xt,yt,poly)
                        print smallRotInPoly[pCount], icp[pCount]
                        pCount += 1
            if pCount == 2: break
        #now which is the first segment, and which the second in terms of rotating AC to be inside the poly
        if smallRotInPoly[0] == True:
            icpRet = icp
        else:
            icpRet[0] = icp[1]
            icpRet[1] = icp[0]
        return icpRet

    def intersectionCircleLine(self, x0, y0, r, x1, y1, x2, y2):
        # note is returning crossing of infinite line, not line segment
        # returns array of size 4 for 2 crossings, size 2 for tangent point, or size 0 for no points
        if abs(x2-x1) > abs(y2-y1):  # allows vertical and near vertical(which could cause floating point errors) lines
            m = (y2-y1)/(x2-x1)

            A = 1 + m**2
            B = -2*x0 - 2*y0*m - 2*m**2*x1 + 2*m*y1
            C = x0**2 + y0**2 - r**2 + 2*y0*m*x1 - 2*y0*y1 + m**2*x1**2 - 2*m*x1*y1 + y1**2

            discriminant = B**2 - 4*A*C
            if discriminant > 0: #2 intersections
                xm = (-B - math.sqrt(discriminant))/(2*A)
                xp = (-B + math.sqrt(discriminant))/(2*A)
                ym = m*(xm - x1) + y1
                yp = m*(xp - x1) + y1
                outarr = np.array([[xm, ym], [xp, yp]])
            elif discriminant == 0: #1 tangent point
                xt = -B/(2*A)
                yt = m*(xt - x1) + y1
                outarr = np.array([xt, yt])
            else: #circle does not cross line
                outarr = np.array([])
        else:
            m = (x2-x1)/(y2-y1)
            A = 1 + m**2
            B = -2*y0 - 2*x0*m - 2*m**2*y1 + 2*m*x1
            C = y0**2 + x0**2 - r**2 + 2*x0*m*y1 - 2*x0*x1 + m**2*y1**2 - 2*m*y1*x1 + x1**2
            discriminant = B**2 - 4*A*C
            if discriminant > 0: #2 intersections
                ym = (-B - math.sqrt(discriminant))/(2*A)
                yp = (-B + math.sqrt(discriminant))/(2*A)
                xm = m*(ym - y1) + x1
                xp = m*(yp - y1) + x1
                outarr = np.array([[xm, ym], [xp, yp]])
            elif discriminant == 0: #1 tangent point
                yt = -B/(2*A)
                xt = m*(yt - y1) + x1
                outarr = np.array([xt, yt])
            else: #circle does not cross line
                outarr = np.array([])
        return outarr

    def intersectionAngleCircleSegment(self, x0, y0, r, x1, y1, x2, y2):
        # returns an array of 2 angles if there are 2 crossing of the line with the circle. empty array for tangent point
        # or no crossing
        ics = self.intersectionCircleSegment(x0, y0, r, x1, y1, x2, y2)
        if ics.size == 4:
            a1 = math.atan2(ics[0,1]-y0,ics[0,0]-x0)
            a2 = math.atan2(ics[1,1]-y0,ics[1,0]-x0)
            outarr = np.array([[a1,a2]])
        else:
            outarr = np.array([])
        return outarr


    def segmentToRay(self,x1,y1,x2,y2):
        #Convert from 2 point from to polar format(required for vertical lines)
        theta = math.atan2(y2-y1,x2-x1)
        return x1,y1,theta

    def cartestianToPolar(self,x1,y1,x2,y2):
        #Convert from 2 point from to polar format(required for vertical lines)
        r = ((x2-x1)**2 + (y2-y1)**2)**0.5
        theta = math.atan2(y2-y1,x2-x1)
        return x1,y1,theta,r

    def polarToCartesian(self,x,y,theta,r):
        x2 = x + r*math.cos(theta)
        y2 = y + r*math.sin(theta)
        return x,y,x2,y2

    def perpDistLine(self, x0, y0, x1, y1, x2, y2):
        # return distance between point x0,y0 and the line defined by the 2 points x1, y1, x2, y2
        Dx = x2-x1
        Dy = y2-y1
        dist = abs(Dy*x0-Dx*y0-x1*y2+x2*y1)/math.sqrt(Dx**2 +Dy**2)
        return dist

    def shortDistSegment(self, x0, y0, x1, y1, x2, y2):
        L01 = math.sqrt((x0-x1)**2+ (y0-y1)**2)
        L02 = math.sqrt((x0-x2)**2+ (y0-y2)**2)
        P = self.perpDistLine(x0, y0, x1, y1, x2, y2)
        return min(L01, L02, P)

    def minDistSegment(self, px, py, x1, y1, x2, y2):
        # return the minimum distance from a point to a segment
        normAng = math.atan2((y2-y1),(x2-x1))+math.pi/2
        crossing,xc,yc = self.segmentCrossRayCart(x1,y1,x2,y2,px,py,normAng)
        if crossing == True:
            dist = self.perpDistLine(px,py,x1,y1,x2,y2)
        else:
            L01 = math.sqrt((px-x1)**2+ (py-y1)**2)
            L02 = math.sqrt((px-x2)**2+ (py-y2)**2)
            dist = min(L01,L02)
        return dist


    def bound(self, val, low, high):
        if low > high:
            l = high
            h = low
        else:
            l = low
            h = high
        return max(l,min(val,h))

    def between(self, val, low, high, decimalPrecision = 8):
        val = round(val,decimalPrecision)
        low = round(low,decimalPrecision)
        high = round(high,decimalPrecision)
        if low > high:
            l = high
            h = low
        else:
            l = low
            h = high
        if val >= l and val <= h:
            return True
        else:
            return False


    def isPointInPolygon(self, x, y, poly):
        # Odd-Even rule for determining if a point is in a polygon.
        # Based off code from http://en.wikipedia.org/wiki/Even%E2%80%93odd_rule
        # Corrected to include all points on the lines of the polygon
        # (within a given tolerance) as being in the polygon. Original code has
        # points on a vertical line on the right would be included, but all
        # others not
        # x, y -- x and y coordinates of point
        # a list of tuples [(x, y), (x, y), ...]
        num = len(poly)
        j = num - 1
        result = False
        if self.isPointOnPolygon(x,y,poly):
            return True
        else:
            for i in range(num):
                v1x = poly[i][0]
                v1y = poly[i][1]
                v2x = poly[j][0]
                v2y = poly[j][1]
                if ((v1y >y) != ( v2y > y)) and (x <= ( v2x - v1x ) * (y - v1y) / (v2y - v1y) + v1x):
                    result = not result
                j = i
            return result

    def isPointOnPolyPoint(self,x,y,poly):
        # check whether the point is actually on a poly point as opposed to a line segment
        # as this can give uncertain results as to which segement it is on
        for i in range(len(poly)):
            ed = math.sqrt((poly[i,0]-x)**2 + (poly[i,1]-y)**2)
            if ed < self.eps:
                return True
        return False

    def isPointOnPolygon(self, x, y, poly, eps='default', gradient = False):
        n = len(poly)

        for i in range(n):
            j = (i+1) % n
            x1 = poly[i][0]
            y1 = poly[i][1]
            x2 = poly[j][0]
            y2 = poly[j][1]
            if self.isPointOnLine(x1,y1,x2,y2,x,y, eps=eps):
                if gradient == False:
                    return True
                else:
                    return True, math.atan2(y2-y1,x2-x1)
        if gradient == False:
            return False
        else:
            return False, None

    def isPointOnPolygonDetailed(self, x, y, poly, eps='default'):
        # return status, gradient of segement, start and end point index of segment
        n = len(poly)
        for i in range(n):
            j = (i+1) % n
            x1 = poly[i][0]
            y1 = poly[i][1]
            x2 = poly[j][0]
            y2 = poly[j][1]
            if self.isPointOnLine(x1,y1,x2,y2,x,y, eps=eps):
                    return True, math.atan2(y2-y1,x2-x1), i, j
        return False, None, None, None

    def isPointOnLine(self,x1,y1,x2,y2,xp,yp,eps='default',bumpDir = 'default'):
        # create a segment around xp,yp, perpendicular to x1,y1,x2,y2 and 2*bump long
        # if these two segments then intersect we will say that the point is on the
        # line. then define any point on the line as in the polygon. wrap this into
        # isPointInPolygon
        if eps == 'default':
            bump = self.eps
        else:
            bump = eps
        if bumpDir == 'default':
            if x1 != x2:
                theta = math.atan2(y2-y1,x2-x1)
                thetap = self.planes.clampAngle(theta+math.pi/2)
            else:
                thetap = 0
        else:
            thetap = bumpDir
        #avoid floating pint errors on cos and sin functions
        if abs(abs(thetap) - math.pi/2) < self.eps:
            cosThetap = 0
        else:
            cosThetap = math.cos(thetap)
        if abs(thetap) < self.eps or abs(thetap-math.pi)< self.eps:
            sinThetap = 0
        else:
            sinThetap = math.sin(thetap)
        # note potential issue here if bumpDir is in left half of circle as extra self.eps adjustments will cancel rather than add
        xp1 = xp - cosThetap*bump# - self.eps
        xp2 = xp + cosThetap*bump# + self.eps
        yp1 = yp - sinThetap*bump# - self.eps
        yp2 = yp + sinThetap*bump# + self.eps
        #print "xp1, xp2, yp1, yp2, thetap, cosThetap, sinThetap", xp1, xp2, yp1, yp2, thetap, cosThetap, sinThetap
        return self.segmentsCrossCart(x1,y1,x2,y2,xp1,yp1,xp2,yp2,bool=True)

    def externalHull(self,path, r):
        # take a given path, staring at left most point(first found if tie, doesn't matter), proceed anticlockwise
        # around exterior searching for next best point in range r
        #find start point
        planes = CurveFit.Planes()
        eHPath = np.empty([len(path)*2,2])
        startIdx = np.argmin(path,0)[0] #lowest easting
        psi = -math.pi/2 #set straight down
        startPoint = path[startIdx]
        eHPath[0] = startPoint
        pointCount = 1
        loopComplete = False
        # now loop until we get back to start point
        point = np.copy(startPoint)
        oldPoint = np.copy(startPoint)
        oldPsi = psi
        pathLen = 0
        while loopComplete == False:
            locPoints = self.localPoly(point,path,r)
            rix = np.where((oldPoint[0]==locPoints[:,0]) & (oldPoint[1]==locPoints[:,1]))[0]
            locPoints = np.delete(locPoints,rix,0)
            angles = np.empty(len(locPoints))
            psiadj = max(math.pi/8.0,planes.clampAngle(psi-oldPsi)*(-1))
            for i in range(len(locPoints)):
                angles[i] = planes.clampAngle(math.atan2(locPoints[i,1]-point[1], locPoints[i,0]-point[0])-psi-psiadj)#math.pi/8.0)
            minIx = np.argmin(angles)
            oldPoint = point
            point = locPoints[minIx]
            #print point, psiadj, oldPsi, psi, angles[minIx]
            if point[0] == startPoint[0] and point[1] == startPoint[1]:
                loopComplete = True
            if pathLen > r*4:
                ED = math.sqrt((startPoint[0]-point[0])**2 + (startPoint[1]-point[1])**2 )
                #print ED, startPoint, point
                if ED < r:
                    loopComplete = True
                    break

            oldPsi = psi
            psi = math.atan2(point[1]-oldPoint[1],point[0]-oldPoint[0])
            if pointCount >= len(eHPath):
                eHPath = np.vstack((eHPath,np.empty([len(eHPath),2])))
            eHPath[pointCount] = point

            if pointCount >0:
                pathLen += math.sqrt((eHPath[pointCount,0]-eHPath[pointCount-1,0])**2 + (eHPath[pointCount,1]-eHPath[pointCount-1,1])**2 )
            pointCount += 1

        return eHPath[:pointCount,:]

    def localPoly(self,point,poly,r):
        #find local square (could then iterate to get a circle, but not realy worth the effort)

        xlow = point[0] - r
        ylow = point[1] - r
        xhigh = point[0] + r
        yhigh = point[1] + r
        ix = np.where((poly[:, 0] > xlow) &
                      (poly[:, 0] < xhigh) &
                      (poly[:, 1] > ylow) &
                      (poly[:, 1] < yhigh))[0]
        poly = poly[ix, :]
        #remove point
        rix = np.where((point[0]==poly[:,0]) & (point[1]==poly[:,1]))[0]
        poly = np.delete(poly,rix,0)
        return poly

    def rotDirection(self,poly):
        #reutrn 1 if points are clockwise, -1 if anti-clockwise
        #http://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order
        n = len(poly)
        sum = 0
        for i in range(n):
            j = (i+1) % n
            sum += (poly[j,0]-poly[i,0])*(poly[j,1]+poly[i,1])
        return np.sign(sum)

if __name__ == '__main__':
    LI = Intersections()

    import matplotlib.pyplot as plt
    import numpy as np

    poly = np.array([[0,0],[4,0],[4,1],[3,1],[3,1.5],[4,1.5],[4,6],[2,7],[2,4],[3,4],[3.1,4.5],[3.5,4.5],[3.5,3],[1.5,3],[1,7],[0,6]])
    p1 = np.array([[3.1, 1.0]])
    p2 = np.array([[3.0, 1.1]])


    path = np.vstack([p1,p2])

    fig = plt.figure()
    plt.plot(poly[:,0], poly[:,1])
    plt.plot(path[:,0], path[:,1])
    plt.show()

    print p1[0][0], p1[0][1]

    print LI.segmentCartInOrOnPoly(poly,p1[0][0],p1[0][1],p2[0][0],p2[0][1])