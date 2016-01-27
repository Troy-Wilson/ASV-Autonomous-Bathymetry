from __future__ import division
__author__ = 'Admin'

import CurveFit
import math
import numpy as np
import Lines
import OnlineGP

class Controllers():
    def __init__(self, poly, dxy):
        self.Planes = CurveFit.Planes()
        self.eps=1.0e-6
        self.boundIdx = None
        self.BoundFollow = False
        self.PrevDesHead = None
        self.poly = poly
        self.LI = Lines.Intersections()
        self.boundaryDirection = None
        self.tck = None
        self.gp = None
        self.dxy = dxy
        minPolyLen = 999999999999999
        for i in range(len(poly)):
            j = (i+1) % len(poly)
            polyLen = ((poly[i,0]-poly[j,0])**2 +(poly[i,1]-poly[j,1])**2)**0.5
            if polyLen < minPolyLen:
                minPolyLen = polyLen
        self.boundaryStepLen = minPolyLen/self.dxy
        self.lastVertex = None
        self.timeSinceBoundaryFollow = self.boundaryStepLen+1
        self.EMAHeading = None
        self.returnToPolyHeading = None
        self.onlineGP = None

    def findFollowContourBoundary(self, targetDepth, currentHeading, posCG ,r, foundCB, wpeps):
        if self.BoundFollow == False:
            desHead, ztmp, polytest = self.roseSolvePoly(targetDepth, self.EMAHeading, posCG, r)
        else:
            desHead = math.atan2(self.prevVY-posCG[1][0],self.prevVX-posCG[0][0])
        bStats, outAway = self.BoundaryStats(posCG,desHead)
        if outAway == True and self.BoundFollow == False: #we are outside of polygon and pointing away from it. we need to get back inside
            #only apply when we are not boundary following
            state =  6.0
            #solve for polygon boundary on r closest to current moving average heading
            # use half the normal search radius to make it run tighter to get back on course
            self.returnToPolyHeading = self.circleCrossPoly(posCG[0][0], posCG[1][0],r*2.0)
            desHead_r = self.returnToPolyHeading
            #we may be pointing back at poly but stil outside it. keep heading until we get back inside otherwise
        else:
            #Are we in contour or boundary following mode?
            if self.BoundFollow == False: #contour following mode
                #self.timeSinceBoundaryFollow += 1
                if polytest == True: # Contour point in boundary
                    desHead_r = desHead
                    state = 1.0
                else: # Contour point out of boundary. This will be our initial boundary following
                    upSlopeIdx, downSlopeIdx = self.BoundarySlope(posCG, desHead, r,bStats)
                    self.BoundFollow = True
                    idx = downSlopeIdx
                    if self.boundaryDirection == None: #we not yet followed the boundary, lets set the direction
                        #self.boundaryDirection = (downSlopeIdx-upSlopeIdx) % len(bStats)
                        if abs(upSlopeIdx-downSlopeIdx) ==1:
                            self.boundaryDirection = (downSlopeIdx-upSlopeIdx)
                        else:
                            self.boundaryDirection = np.sign(downSlopeIdx-upSlopeIdx)*(-1)
                    if bStats[idx][0]>r:
                        self.boundIdx = idx
                        state = 2.0
                    else:
                        self.boundIdx = (idx + self.boundaryDirection) % len(bStats)
                        state = 3.0
                    desHead_r = bStats[self.boundIdx][1]
                    self.prevVX = bStats[self.boundIdx][3]
                    self.prevVY = bStats[self.boundIdx][4]
            else: #we are in boundary following mode
                if foundCB == False: #we have found boundary
                    foundCB = True
                #Check if within r of next vertex
                if bStats[self.boundIdx][0] > wpeps:
                    state = 2.1
                else: #we have hit vertex, go to next vertex
                    self.boundIdx = (self.boundIdx + self.boundaryDirection) % len(bStats)
                    state = 3.1
                desHead_r = bStats[self.boundIdx][1]
                zBoundary = self.getDepthTheta(posCG[0],posCG[1],r,desHead_r)
                self.prevVX = bStats[self.boundIdx][3]
                self.prevVY = bStats[self.boundIdx][4]
                #if boundary following would take us too shallow, switch boundFollow flag, find best heading inside poly
                if zBoundary < targetDepth:
                    print '****** bouncing off boundary **********'
                    circleBounds = self.LI.intersectionCirclePoly(self.poly,posCG[0],posCG[1],r)
                    inPolyHeadings = self.roseSolveBatch(posCG[0],posCG[1],self.EMAHeading,r,targetDepth,circleBounds[0,2],circleBounds[1,2])
                    desHead_r = inPolyHeadings[0,0]
                    ztmp = inPolyHeadings[0,1]
                    self.BoundFollow = False
                    state = 1.1
                    self.EMAHeading = desHead_r
                    print 'exiting bounce'
                else:
                    ztmp = zBoundary
        if foundCB == False: #set flag for cont/bound folowing has started
            if abs(ztmp-targetDepth) < self.eps or self.BoundFollow == True:
                foundCB = True

        if desHead_r == None:
            desheadtxt = 99999
        else:
            desheadtxt = desHead_r
        debugstr = '{:.1f},{:.2f},{:.2f},{:.2f},{:.4f},{:.4f},{:.4f}'.format(state,posCG[0][0],posCG[1][0],ztmp,currentHeading,desheadtxt,self.EMAHeading)
        return desHead_r, r,ztmp, foundCB, debugstr


    def roseSolvePoly(self, targetDepth, currentHeading, posCG, r):
        res = self.roseSolveBatch(posCG[0],posCG[1],currentHeading,r,targetDepth)
        # res is a Xx4 array of the heading giving the best depth error in X segments.
        # values are heading, depth, absheadingerr, absdeptherr, sorted by abs depth err then abs heading err
        # Set return as first point
        desiredHeading = res[0,0]
        ztmp = res[0,1]
        # if there are multiple solutions to the target depth, return the first one that is inside the polygon
        # if there are none then the return result will be the first point as above
        # Reasoning here is that if we have multiple solutions, then we want to return the one that is in the polygon
        # if none are in the polygon then we will boundary follow outside this function
        i = 0
        #return opposite of segmentPolarCrossPoly function Ie lines cross = False => insidePoly = True
        insidePoly= not self.LI.segmentPolarCrossPoly(self.poly,posCG[0],posCG[1],res[i,0],r)
        while res[i,3] < self.eps and insidePoly == False:
            #check if point is in polygon
            if not self.LI.segmentPolarCrossPoly(self.poly,posCG[0],posCG[1],res[i,0],r):
                desiredHeading = res[i,0]
                ztmp = res[i,1]
                insidePoly = True
            i += 1
            if i==res.shape[0]:
                break
        #if desiredHeading != res[0,0]:
        #    print "not first rosesolve element"
        return desiredHeading, ztmp, insidePoly #return inside poly value so I dont check again later?? first check funtion works

    def roseSolveBatch(self,x0,y0,heading,r,targetDepth,lbound=None, ubound=None):
        #get depths data in a batch rather thasn interatively. this is faster for querying a GP
        #linearly interpolate between the results.
        #TODO: for better accuaracy for a moving robot with rotational velocity limits it would make
        #TODO: sense to have finer graduation around current heading rather than even splits.
        splits = 50
        if lbound == None:
            lbound = self.Planes.clampAngle(heading-5./(8)*np.pi)
            dtheta = (5./8*2*np.pi)/(splits-1)
        else:
            direction = np.sign(ubound-lbound)
            if direction == -1:
                arcLen = math.pi*2-direction*(ubound-lbound)
            else:
                arcLen = ubound-lbound
            dtheta = arcLen/(splits-1)
        psis =  self.Planes.clampAngle(np.arange(lbound,lbound+splits*dtheta,dtheta)[:,np.newaxis])
        Xstar = np.hstack([np.cos(psis)*r + x0,np.sin(psis)*r + y0])
        Ystar = self.getDepthBatch(Xstar)
        #now linearly interpolate
        ret = np.empty([splits-1,4])
        for i in range(splits-1):
            psiRange = self.Planes.clampAngle(psis[i+1]-psis[i])
            slope = psiRange/(Ystar[i+1]-Ystar[i])
            err = targetDepth-Ystar[i]
            slopeSign = np.sign(slope)
            if err*slopeSign < 0: #start range
                psi = psis[i]
                z = Ystar[i]
            else:
                dPsi =  err*slope
                if dPsi > psiRange: #end range
                    psi = psis[i+1]
                    z = Ystar[i+1]
                else: #interpolate
                    psi = self.Planes.clampAngle(psis[i] + dPsi)
                    z = Ystar[i] + dPsi*(1/slope)
            ret[i,0] = psi
            ret[i,1] = z
            ret[i,2] = abs(self.Planes.clampAngle(psi-heading))
            ret[i,3] = abs(targetDepth-z)
        ret.view('i8,i8,i8,i8').sort(order = ['f3','f2'], axis=0)
        # values are heading, depth, absheadingerr, absdeptherr, sorted by abs depth err then abs heading err
        return ret


    def getDepthTheta(self,x, y, r,theta):
        xp = x + r*math.cos(theta)
        yp = y + r*math.sin(theta)
        return self.getDepth(xp,yp)

    def getDepth(self, x, y):
        x = float(x)
        y = float(y)
        depth = self.onlineGP.pred(np.array([[x, y]]))[0][0]
        return depth

    def getDepthBatch(self,X):
        depth = self.onlineGP.pred(X)
        return depth

    def BoundaryStats(self,posCG,desHead):
        stats = np.zeros((len(self.poly),5))
        x = posCG[0]
        y = posCG[1]
        for i in range(len(self.poly)):
            xp = self.poly[i][0]
            yp = self.poly[i][1]
            stats[i][0] = ((xp-x)**2 +(yp-y)**2)**0.5 #distance
            #get heading
            stats[i][1] = math.atan2(yp-y,xp-x) #heading
            stats[i][2] = self.Planes.clampAngle(stats[i][1]-desHead)#headingDiff
            stats[i][3] = xp
            stats[i][4] = yp
        V1Idx,V2Idx  = self.LI.rayCrossPoly(self.poly, x, y,desHead)
        if V1Idx == None:
            #outside poly and facing away thus ray not crossing
            outAway = True
        else:
            outAway = False
        return stats, outAway

    def BoundarySlope(self,posCG,desHead,boundBuff,stats):
        x = posCG[0]
        y = posCG[1]
        V1Idx,V2Idx  = self.LI.rayCrossPoly(self.poly, x, y,desHead)

        if V1Idx == None: #outside polygon and heading away from an edge. these are irrelevant, set such that they don't crash the program
            #print "*outside polygon err*"#, self.poly, x, y,desHead

            VUpIdx = None
            VDownIdx = None
            #V1Idx = 0
            #V2Idx = 1
        else:
            #now determine which is sloping up, and which is sloping down
            V1H = stats[V1Idx][1]
            V2H = stats[V2Idx][1]
            V1xq = x + boundBuff*math.cos(V1H)
            V1yq = y + boundBuff*math.sin(V1H)
            #V1Hdepth = self.getDepth(V1xq, V1yq)
            V2xq = x + boundBuff*math.cos(V2H)
            V2yq = y + boundBuff*math.sin(V2H)
            #V2Hdepth = self.getDepth(V2xq, V2yq)

            v1Hv2Hdepth = self.getDepthBatch(np.array([[V1xq, V1yq],[V2xq, V2yq]]))
            V1Hdepth = v1Hv2Hdepth[0]
            V2Hdepth = v1Hv2Hdepth[1]

            if V1Hdepth < V2Hdepth:
                VUpIdx = V1Idx
                VDownIdx = V2Idx
            else:
                VUpIdx = V2Idx
                VDownIdx = V1Idx

        return VUpIdx, VDownIdx

    def circleCrossPoly(self, xp, yp, r):
        anglesCount = 0
        angles = np.zeros([self.poly.shape[0]*2])
        for i in range(self.poly.shape[0]):
            x1 = self.poly[i,0]
            y1 = self.poly[i,1]
            i2 = (i+1) % self.poly.shape[0]
            x2 = self.poly[i2,0]
            y2 = self.poly[i2,1]
            res = self.LI.intersectionAngleCircleSegment(xp,yp,r,x1,y1,x2,y2)
            if res == []: #try again on double radius(which will be normal search radius) before exiting program
                res = self.LI.intersectionAngleCircleSegment(xp,yp,r*2.0,x1,y1,x2,y2)
            if res != []:
                for j in range(res.shape[1]):
                    angles[anglesCount+j] = res[0,j]
                    #idxAr[anglesCount+j] = i
                anglesCount += res.shape[1]
        err = 7 #> than 2*pi
        newHeading = None
        if anglesCount > 0:
            for i in range(anglesCount):
                angDiff = abs(self.Planes.clampAngle(angles[i] - self.EMAHeading))
                if angDiff < err:
                    err = abs(self.Planes.clampAngle(angles[i] - self.EMAHeading))
                    newHeading = angles[i]
        else:
            print 'vessel more than R outside of bounding polygon. Control stopped for Safety'
            newHeading = None
        return newHeading



