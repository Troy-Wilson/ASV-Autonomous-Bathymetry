from __future__ import division
__author__ = 'Admin'

import numpy as np
import math
import matplotlib.pyplot as plt
import Lines
import CurveFit


eps = 1e-06
significance = 12
LI = Lines.Intersections()
planes = CurveFit.Planes()


def reduceCrossingList(crossings, crossingCount):
    #only required if we hit a vertex instead of an edge, as each vertex is in 2 edges
    if crossingCount <= 1:
        return crossings[:crossingCount,:], crossingCount
    else:
        crossingsR = np.empty([crossingCount,3])
        ccr = 0
        for i in range(crossingCount):
            duplicate = False
            for j in range(i+1,crossingCount):
                t1 = abs(crossings[i,0] - crossings[j,0])
                t2 = abs(crossings[i,1] - crossings[j,1])
                if t1 < 1e-06 and t2 < 1e-06:
                    duplicate = True
            if duplicate == False:
                crossingsR[ccr,:] = crossings[i,:]
                ccr += 1
        return crossingsR[:ccr,:], ccr

def sweep(points, angle, dxy):
    #sweep from bottom left corner, angle is the angle of the line the ray runs on must be between 0 and pi/2
    #dxy is the line spacing
    minx = min(points[:,0])
    miny = min(points[:,1])
    maxx = max(points[:,0])
    maxy = max(points[:,1])
    cosang = math.cos(angle)
    sinang = math.sin(angle)

    if abs(cosang) > eps:#1.0e-6:
        lenRayX = abs((maxx-minx)/cosang)
    else:
        lenRayX = np.inf
    if abs(sinang) > eps:#1.0e-6:
        lenRayY = abs((maxy-miny)/math.sin(angle))
    else:
        lenRayY = np.inf
    lenRay = min(lenRayX, lenRayY)
    steps = int(lenRay/dxy) + 2

    pointCnt = len(points[:,0])
    maxCrossings = int(math.sqrt((maxx-minx)**2+(maxy-miny)**2)/dxy)#steps
    #pos = [minx+dxy, miny+dxy]
    #pos = [minx+0.001*cosang, miny+0.001*sinang]
    pos = [minx, miny]
    #pos = [minx+eps*cosang, miny+eps*sinang]
    print pos
    normAngle = planes.clampAngle(angle-math.pi/2)
    crossingAr = np.empty([steps],dtype=object)
    crossingCount = np.zeros([steps],dtype=np.int_)
    for i in range(steps):
        crossings = np.empty([maxCrossings,3])

        for j in range(pointCnt):
            a = points[j, :]
            j1 = (j+1) % (pointCnt)
            b = points[j1, :]
            cross, x, y = LI.segmentCrossRayCart(a[0],a[1],b[0],b[1],pos[0],pos[1],normAngle)
            if cross == True:
                # work out which point is closest to the origin(ie on the bottom left side of the ray)
                adist = math.sqrt((a[0]-minx)*(a[0]-minx) + (a[1]-miny)*(a[1]-miny))
                bdist = math.sqrt((b[0]-minx)*(b[0]-minx) + (b[1]-miny)*(b[1]-miny))
                if adist < bdist:
                    gradient = math.atan2(b[1]-a[1],b[0]-a[0])
                else:
                    gradient = math.atan2(a[1]-b[1],a[0]-b[0])
                crossings[crossingCount[i],0] = x
                crossings[crossingCount[i],1] = y
                crossings[crossingCount[i],2] = gradient
                #print i, x, y, gradient
                crossingCount[i] += 1

        crossings, crossingCount[i] = reduceCrossingList(crossings,crossingCount[i])
        #sort array from bottom to top of slice
        if crossingCount[i] > 0:
            if abs(angle) < math.pi/4 or abs(angle) >3*math.pi/4 : #sort by y vlaues
                crossings = crossings[crossings[:,1].argsort()]
            else: # sort by x values
                crossings = crossings[crossings[:,0].argsort()]
        #print crossingCount[i], crossings
        crossingAr[i] = np.around(crossings,significance)
        pos[0] = pos[0] + dxy*math.cos(angle)
        pos[1] = pos[1] + dxy*math.sin(angle)
    crossingAr, crossingCount, validEdgeCount = checkEdgeLen(crossingAr, crossingCount,dxy)
    pos = [crossingAr[validEdgeCount-1][0][0],crossingAr[validEdgeCount-1][0][1]]
    return crossingAr, crossingCount, pos

def checkEdgeLen(crossingAr, crossingCount,dxy):
    #this is required to make sure that the edges are long enough. without it we can get spacing between tracks correct,
    # but along track lengths can be too short which then causes boustrophedon creation to fail.
    edgeCount = len(crossingAr)
    validEdgeCount = 0
    newCrossingAr = np.empty([edgeCount],dtype=object)
    newCrossingCount = np.zeros([edgeCount],dtype=np.int_)
    for i in range(edgeCount):
        if len(crossingAr[i]) > 1 :
            edgelen = math.sqrt((crossingAr[i][0][0]-crossingAr[i][1][0])**2 + (crossingAr[i][0][1]-crossingAr[i][1][1])**2)
            if edgelen > dxy:
                newCrossingAr[validEdgeCount] = crossingAr[i]
                newCrossingCount[validEdgeCount] = crossingCount[i]
                validEdgeCount += 1
    return newCrossingAr[:validEdgeCount], newCrossingCount[:validEdgeCount+1], validEdgeCount

def creatCellPolygons(cells, poly,dxy,sweepAngle):
    cellPolygons = []
    #direction = orderPointsPoly(poly)
    for cell in cells:
        cellDone = False
        #add first 2 points, these are the first 2 points that make 1 boundary on the side where the sweepline first
        # touches the polygon
        cellPolygon = [cell[0]] #bottom left point
        cellPolygon.append(cell[1]) #top left point
        #now find where cell[1] crosses the polygon

        for i in range(len(poly)):
            j = (i + 1) % len(poly)
            if LI.isPointOnLine(poly[i,0],poly[i,1],poly[j,0],poly[j,1],cell[1,0],cell[1,1],eps=dxy,bumpDir=sweepAngle):
                iang = math.atan2(poly[i,1]-cell[1,1],poly[i,0]-cell[1,0])
                jang = math.atan2(poly[j,1]-cell[1,1],poly[j,0]-cell[1,0])
                iangdiff = abs(planes.clampAngle(iang-sweepAngle))
                jangdiff = abs(planes.clampAngle(jang-sweepAngle))
                if iangdiff > jangdiff: #use jang
                    direction = 1
                    k = j
                else:
                    direction = -1
                    k = i
                break
        # check cell[2] is not on the same line segment
        if not LI.isPointOnLine(poly[i,0],poly[i,1],poly[j,0],poly[j,1],cell[2,0],cell[2,1],eps=dxy,bumpDir=sweepAngle):
            cellPolygon.append(poly[k])
            #Follow until we hit cell[2]
            i = k
            j = (i + direction) % len(poly)
            while not LI.isPointOnLine(poly[i,0],poly[i,1],poly[j,0],poly[j,1],cell[2,0],cell[2,1],eps=dxy,bumpDir=sweepAngle):
                cellPolygon.append(poly[j])
                i = (i + direction) % len(poly)
                j = (i + direction) % len(poly)
        cellPolygon.append(cell[2])
        cellPolygon.append(cell[3])
        #now find where cell[3] crosses the polygon

        for i in range(len(poly)): #i should get reset here
            j = (i + 1) % len(poly)
            if LI.isPointOnLine(poly[i,0],poly[i,1],poly[j,0],poly[j,1],cell[3,0],cell[3,1],eps=dxy,bumpDir=sweepAngle):
                # check cell[0] is not on the line as well though first
                if not LI.isPointOnLine(poly[i,0],poly[i,1],poly[j,0],poly[j,1],cell[0,0],cell[0,1],eps=dxy,bumpDir=sweepAngle):
                    iang = math.atan2(poly[i,1]-cell[3,1],poly[i,0]-cell[3,0])
                    jang = math.atan2(poly[j,1]-cell[3,1],poly[j,0]-cell[3,0])
                    iangdiff = abs(planes.clampAngle(iang-sweepAngle))
                    jangdiff = abs(planes.clampAngle(jang-sweepAngle))
                    if iangdiff > jangdiff: #use jang
                        direction = 1
                        k = j
                    else:
                        direction = -1
                        k = i
                    cellPolygon.append(poly[k])
                else:
                    # we have finished cell
                    cellDone = True
                break

        if cellDone == False:
            # Follow until we hit cell[0]
            i = k
            j = (i + direction) % len(poly)
            while not LI.isPointOnLine(poly[i,0],poly[i,1],poly[j,0],poly[j,1],cell[0,0],cell[0,1],eps=dxy,bumpDir=sweepAngle):
                cellPolygon.append(poly[k])
                i = (i + direction) % len(poly)
                j = (i + direction) % len(poly)
        cellPolygons.append(np.asarray(cellPolygon))
        #end loop
    return cellPolygons

def orderPointsPoly(poly):
    #calc centre
    count = len(poly[:,0])
    C = [sum(poly[:,0]/count),sum(poly[:,0]/count)]
    #take sign of the sum of the angle differences of consecutive points to this centre
    diffSum = 0
    for i in range(count):
        j = (i+1) % count
        ang1 = math.atan2(poly[i,1]-C[1],poly[i,0]-C[0])
        ang2 = math.atan2(poly[j,1]-C[1],poly[j,0]-C[0])
        diffSum += planes.clampAngle(ang2-ang1)
    return np.sign(diffSum)

def createCells(crossingAr, crossingCount, dxy, angle):
    # Currently only returns the 4 points where the sweep line crosses the polygon
    # That rectangle does not define the cell without also reference to the whole polygon
    # need to create the polygon boundary of cells that are created by the 2 edges straight aligned with the sweep
    # and the two potentially complex edges between them that follow the polygon
    # I could create this during this function or perhaps convolve the two polygons somehow?
    cellcount = 0
    cells = np.empty([50, 4, 2],dtype=object) #oversized array, returns 4 corners
    neighbours = []
    openCells = []
    prevCrossings = 0
    adjx = math.cos(angle)*dxy
    adjy = math.sin(angle)*dxy
    for i in range(len(crossingCount)):
        currentCrossings = crossingCount[i]
        diff = (currentCrossings - prevCrossings)/2
        prevOpenCells = []
        # keep track of cells through ordering along the slice. cells cannot cross.
        if diff != 0: #we have an event, close open cells and open new cells
            # close all open cells
            cc = 0
            while len(openCells) > 0:
                oc = openCells.pop(0)
                #order of points will be circular from point 1
                cells[oc, 3, :] = crossingAr[i-1][cc][:2]
                cells[oc, 2, :] = crossingAr[i-1][cc+1][:2]
                cc += 2
                prevOpenCells.append(oc)
            # open new cells
            for j in range(int(currentCrossings/2)):
                openCells.append(cellcount)
                cells[cellcount, 0, :] = crossingAr[i][j*2][:2]
                cells[cellcount, 1, :] = crossingAr[i][j*2+1][:2]
                #work out neighbours
                for id in prevOpenCells:
                    prevt = np.copy(cells[id, 2, :])
                    prevb = np.copy(cells[id, 3, :])
                    prevt[0] += adjx
                    prevt[1] += adjy
                    prevb[0] += adjx
                    prevb[1] += adjy
                    currt = np.copy(cells[cellcount, 0, :])
                    currb = np.copy(cells[cellcount, 1, :])
                    #now test if the segments prevt,prevb and currt, currb overlap. if so add to neighbour list
                    overlap = LI.coincedentSegmentsCross(prevt[0],prevt[1],prevb[0],prevb[1,],currt[0],currt[1],currb[0],currb[1])
                    if overlap:
                        neighbours.append([id,cellcount])
                cellcount += 1
        prevCrossings = currentCrossings
    cells = cells[:cellcount, :4]
    return cells, neighbours

def shrinkCells(cells, dxy, sweepAngle):
    sCells = np.copy(cells).astype(np.double)
    for i in range(sCells.shape[0]):
        #adjust the 0 and 1 corners
        ang = sweepAngle+math.pi/2
        xadj = dxy*math.cos(ang)
        yadj = dxy*math.sin(ang)
        sCells[i][0,0] += xadj
        sCells[i][0,1] += yadj
        sCells[i][1,0] -= xadj
        sCells[i][1,1] -= yadj
        #adjust the 2 and 3 corners
        ang = sweepAngle-math.pi/2
        xadj = dxy*math.cos(ang)
        yadj = dxy*math.sin(ang)
        sCells[i][2,0] += xadj
        sCells[i][2,1] += yadj
        sCells[i][3,0] -= xadj
        sCells[i][3,1] -= yadj
    sCells = np.round(sCells,significance)
    return sCells

def genLawnmower(corners, boundaryPoly, localPoly, initialIdx, sweepAngle, dxy):
    print dxy
    x = corners[initialIdx,0]
    y = corners[initialIdx,1]
    cellCovered = False
    path = []
    #calc direction of first line, and sweep
    opCnrListPL = [1,0,3,2]
    opCnrListPS = [0,0,1,1]
    opCnrPLIdx = opCnrListPL[initialIdx]
    opCnrPSAdj = opCnrListPS[initialIdx]
    pt01Y = corners[opCnrPLIdx,1]-corners[initialIdx,1]
    pt01X = corners[opCnrPLIdx,0]-corners[initialIdx,0]
    if pt01X == 0 and pt01Y == 0: #cover case where points 0 and 1 are the same
        pt2Idx = (initialIdx + 2) % 4
        pt3Idx = (initialIdx + 3) % 4
        pt23Y = corners[pt2Idx,1]-corners[pt3Idx,1]
        pt23X = corners[pt2Idx,0]-corners[pt3Idx,0]
        psiLine = math.atan2(pt23Y,pt23X)
    else:
        psiLine = math.atan2(corners[opCnrPLIdx,1]-corners[initialIdx,1], corners[opCnrPLIdx,0]-corners[initialIdx,0])
    psiSweepCell =  planes.clampAngle(sweepAngle + math.pi*opCnrPSAdj)
    print psiSweepCell

    j = 0
    while not cellCovered:
        # calc line
        isPOP = False
        xBump = x + dxy/100.*math.cos(psiLine) #bump along trackline
        yBump = y + dxy/100.*math.sin(psiLine) #bump along trackline
        if j == 0: #on first point bump along sweep direction to make sure poly test is not on boundary
            xBump += 2*eps*math.cos(psiSweepCell)
            yBump += 2*eps*math.sin(psiSweepCell)
        else:
            isPOP, polySegGrad = LI.isPointOnPolygon(x,y,localPoly,eps=dxy/100, gradient=True) #on last point bump along sweep direction to make sure poly test is not on boundary
            if isPOP:
                xBump += 2*eps*math.cos(psiSweepCell+math.pi)
                yBump += 2*eps*math.sin(psiSweepCell+math.pi)
        xe, ye, dist = LI.rayCrossPolyLoc(localPoly,xBump,yBump,psiLine,99999999)
        if isPOP:#now undo last point bump
                xBump -= 2*eps*math.cos(psiSweepCell+math.pi)
                yBump -= 2*eps*math.sin(psiSweepCell+math.pi)
        if j == 0: #now undo 1st point bump
            if xe != None:
                xe -= 2*eps*math.cos(psiSweepCell)
                ye -= 2*eps*math.sin(psiSweepCell)
            else:
                x = x + dxy*math.cos(psiSweepCell)
                y = y + dxy*math.sin(psiSweepCell)
                xe, ye, dist = LI.rayCrossPolyLoc(localPoly,x,y,psiLine,99999999)
        if xe == None:
            cellCovered = True
            break
        xe = xe - math.cos(psiLine)*dxy
        ye = ye - math.sin(psiLine)*dxy
        line = wayPointsOnLine(x,y,xe,ye,dxy)
        if path == []:
            path = line
        else:
            path = np.vstack([path,line])
        psiLine = planes.clampAngle(psiLine + math.pi)
        # turn
        x = path[-1,0] + dxy*math.cos(psiSweepCell)#*(0.999999) #fudge to avoid floating point issues on in polygon check
        y = path[-1,1] + dxy*math.sin(psiSweepCell)#*(0.999999)

        onSweepLine = False
        isPOP, polySegGrad = LI.isPointOnPolygon(x,y,boundaryPoly,eps=dxy/100, gradient=True)
        if isPOP:
            onSweepLine = True
            if LI.isPointOnPolyPoint(x,y,boundaryPoly):
                xBump = x + dxy/100.*math.cos(psiLine)
                yBump = y + dxy/100.*math.sin(psiLine)
                isPOP, polySegGrad = LI.isPointOnPolygon(xBump,yBump,boundaryPoly,eps=dxy/100, gradient=True)
                if not isPOP:
                    onSweepLine = False
        if onSweepLine:
            if abs(polySegGrad - psiLine) < eps or abs(abs(polySegGrad - psiLine) - math.pi) < eps:
                cellCovered = True
                break

        if LI.isPointInPolygon(x,y,localPoly):
            #cast ray back towards boundary

            xb, yb, dist = LI.rayCrossPolyLoc(localPoly,x,y,planes.clampAngle(psiLine + math.pi),99999999)
        else:
            #cast ray to boundary
            xb, yb, dist = LI.rayCrossPolyLoc(localPoly,x,y,psiLine,99999999)
        if xb == None:
            #ray doesn't cross boundary
            cellCovered = True
            break
        x = xb + dxy*math.cos(psiLine)
        y = yb + dxy*math.sin(psiLine)
        path = np.vstack([path,[x,y]])
        j += 1
    return path

def wayPointsOnLine(xs, ys, xe, ye, dxy):
    #print "waypointsonline", xs, ys, xe, ye, dxy
    #return list of waypoints from start to end point with points dxy apart between
    len = math.sqrt((xs-xe)**2 + ((ys-ye)**2))
    if len < dxy:
        path = np.array([[xs,ys],[xe,ye]])
    else:
        ang = math.atan2(ye-ys,xe-xs)
        n = int(len/dxy) + 1
        path = np.empty([n,2])
        x = xs
        y = ys
        for i in range(0,n-1):
            x = round(x+math.cos(ang)*dxy,2)
            y = round(y+math.sin(ang)*dxy,2)
            path[i] = [x,y]
        path[-1] = [xe,ye]
        #print "waypointsonline lem, n path ",len, n, path
        lastGap = math.sqrt((path[-1,0]-path[-2,0])**2+(path[-1,1]+path[-2,1])**2)
        if lastGap < dxy/4:
            #delete second last waypoint
            path = np.delete(path,path[-2],axis=0)
    return path

def crossingPointClosestCell(x,y,cellPolygons,circleSegments=16,sweepAngle=0):
    LI = Lines.Intersections()
    Planes = CurveFit.Planes()
    minDist = 9999999999999999999
    mindistX = None
    minDistY = None
    for i in range(circleSegments):
        angle = Planes.clampAngle(sweepAngle + float(i)/circleSegments*(np.pi*2))
        for cp in cellPolygons:
            xret, yret, dist = LI.rayCrossPolyLoc(cp,x,y,angle)
            if dist < minDist:
                minDist = dist
                mindistX = xret
                minDistY = yret
                mincp= cp
    #print "crossing is on cell ", mincp, mindistX, minDistY
    return mindistX,minDistY

def clockwise(poly, cell):
    polyDir =  LI.rotDirection(poly)
    cellDir = LI.rotDirection(cell)
    if polyDir == -1: #reverse order from 2nd element
        poly = poly[::-1]
        poly = np.roll(poly,1, axis=0)
    if cellDir == -1: #reverse order from 2nd element
        cell = cell[::-1]
        cell = np.roll(cell,1, axis=0)
    return poly, cell

def retRollSegment(arr,i,j):
    #return section of list, rolling over then end back to the start if required.
    if i<=j:
        return arr[i:j]
    else:
        a = arr[i:]
        b =  arr[:j]
        return np.vstack([a,b])

def localPoly(cell,poly):
    poly,cell = clockwise(poly,cell)
    #loop around cell corners
    n = 4
    m = poly.shape[0]
    segIdxCross = np.empty(4)
    for i in range(n):
        segList = []
        for j in range(m):
            k = (j+1) % m
            x1 = poly[j,0]
            y1 = poly[j,1]
            x2 = poly[k,0]
            y2 = poly[k,1]
            cross = LI.isPointOnLine(x1,y1,x2,y2,cell[i,0],cell[i,1])
            if cross == True:
                # not this will get overwritten until the last point clockise from the starting point is tested
                # need to handle the situtaion where the first cell point co-incides with a polygon point
                segIdxCross[i] = k

    localPoly = cell[0]
    localPoly = np.vstack([localPoly,cell[1]])
    topSeg = retRollSegment(poly,segIdxCross[1],segIdxCross[2])
    localPoly = np.vstack([localPoly,topSeg])
    localPoly = np.vstack([localPoly,cell[2]])
    localPoly = np.vstack([localPoly,cell[3]])
    botSeg = retRollSegment(poly,segIdxCross[3],segIdxCross[0])
    #handle degenerate situation where the 1st cell point is on the polygon point
    ed = np.sqrt((cell[0,0]-poly[0,0])**2 + (cell[0,1]-poly[0,1])**2)
    if ed < eps:
        botSeg = np.vstack([botSeg,poly[-1]])
    localPoly = np.vstack([localPoly,botSeg])
    return localPoly

if __name__ == '__main__':

    import AStar
    import TSP
    import os

    userHome = os.path.expanduser('~')
    dir = userHome + '/Dropbox/missions/ASV/2015nov/24/7/'
    userHome = 'D:/User/Admin'
    dir = userHome + '/Dropbox/missions/ASV/2015nov/24/9/'
    #dir = userHome + '/tmp/nov2015/24/7/'
    points = np.genfromtxt((dir + 'boundaryDataPH.txt'), delimiter=',') #this is external hull drawn around the points that made the loop
    scannedpoints = np.genfromtxt((dir + 'scannedPoints.txt'), delimiter=',')
    debugout = np.genfromtxt((dir + 'debugout.txt'), delimiter=',')
    debugoutrf = open((dir + 'debugoutrf.txt'),'w')
    j= 0
    txt = ''
    for i in range(len(debugout)):
        if j == 9:
            txt += '\n'
            j=0
        txt += str(debugout[i]) + ','
        j +=1
    debugoutrf.write(txt)

    debugoutrf.close()
    #points = np.array([[0,0],[4,0],[4,1],[3,1],[3,1.5],[4,2],[4,6],[2,7],[2,4],[3,4],[3.1,4.5],[3.5,4.5],[3.5,3],[1.5,3],[1,7],[0,6]])
    #points = np.genfromtxt('/home/troy/tmp/bound.csv', delimiter=',')
    #points = np.genfromtxt('/home/troy/tmp/boundaryData.txt', delimiter=',')
    #points = np.genfromtxt('/home/troy/Dropbox/Simulations/ASV/Contour_boust/201505/datafiles/f/boundaryData.txt', delimiter=',')
    #points = np.array([[0,0],[2,0],[2,4],[4,4],[4,2],[0,2]])
    #points = np.array([[0,0],[4,0],[4,4],[0,4]])
    #points = np.array([[0,0],[0.5,-0.5],[1,-1],[1.5,0],[2,0],[2,1],[1.8,1],[1,3],[0.5,1.5],[0,1]])

    LI = Lines.Intersections()

    sweepAngle = 0
    dxy = 2
    searchradius = 3.0
    points = scannedpoints[53:396,4:]

    fig = plt.figure()
    plt.plot(points[:,0],points[:,1], c = 'g')
    plt.show()
    #
    # points = LI.externalHull(points,searchradius)
    #
    # fig4 = plt.figure()
    # plt.plot(points[:,0],points[:,1], c = 'g')# External hull
    # plt.plot(scannedpoints[:,4],scannedpoints[:,5], c = 'r')# External hull
    # plt.show()
    #
    # #note cannot have a different pathwidth vs sweep size as otherwise we can get cells which are smaller than the distance
    # #between points
    # print "sweep"
    # crossingAr, crossingCount, tmppos = sweep(points, sweepAngle, dxy)
    # print "createcells"
    # originalCells, neighbours = createCells(crossingAr,crossingCount,dxy,sweepAngle)
    # print originalCells.shape#, originalCells
    # for cell in originalCells:
    #     plt.figure
    #     plt.plot(points[:,0], points[:,1])
    #     plt.plot(cell[:,0],cell[:,1])
    #     plt.show()
    # print "create polygons"
    # # cellPolygons = creatCellPolygons(originalCells,points,dxy,sweepAngle)
    # # print len(cellPolygons)#, cellPolygons
    #
    # # for cell in cellPolygons:
    # #     plt.figure
    # #     plt.plot(points[:,0], points[:,1])
    # #     plt.plot(cell[:,0],cell[:,1])
    # #     plt.show()
    #
    #
    # print "shrink cells"
    # shrunkCells = originalCells#shrinkCells(originalCells,dxy,sweepAngle)
    # openShrunkCells = np.copy(shrunkCells)
    #
    # # cdg = AStar.CellularDecompGraph()
    # # pos = points[-1,:]
    # # pos = [37.29138341603209028107812628149986,34.28674878739212772416067309677601]
    # # print 'pos',pos
    #
    # # plt.figure
    # # plt.plot(points[:,0], points[:,1]) #boundary
    # #for cp in cellPolygons:
    # #    plt.plot(cp[:,0], cp[:,1])
    # #plt.plot(shrunkCells[0][:,0], shrunkCells[0][:,1])
    # #plt.show()
    #
    # TSP = TSP.Tsp()
    # LI = Lines.Intersections()
    # Planes = CurveFit.Planes()
    #
    # # posinpoly = False
    # # for sc in shrunkCells:
    # #     if LI.isPointInPolygon(pos[0],pos[1],sc) ==True:
    # #         posinpoly = True
    # #         break
    # # # else find nearest point that is
    # # if posinpoly == False:
    # #     x,y = crossingPointClosestCell(pos[0],pos[1],shrunkCells)
    # #     pos = [x,y]
    # #
    # # print 'adj pos',pos
    # pos = [-5.3, 5.14] #points[-1,:]
    # #pos = [1, 1]
    # path = np.array([pos])
    # for i in range(originalCells.shape[0]):
    #     print i, openShrunkCells.shape[0]
    #     pos = path[-1]
    #     t = time.time()
    #     #entry, transit = TSP.greedyCellTSP2(openShrunkCells, shrunkCells, cellPolygons, neighbours, pos)
    #     entry, transit = TSP.greedyCellTSP3(openShrunkCells,pos)
    #     print("TSP calc time = %s seconds " % (time.time() - t)), entry, transit
    #     transit = np.asarray(transit)
    #     path = np.vstack([path,transit])
    #     tmpStr = openShrunkCells[entry[0]][entry[1]]
    #     subPath = genLawnmower(openShrunkCells[entry[0]],points,entry[1],sweepAngle,dxy)
    #     plt.figure
    #     plt.plot(points[:,0], points[:,1])
    #     plt.plot(path[:,0],path[:,1])
    #     plt.plot(subPath[:,0],subPath[:,1])
    #     plt.plot(transit[:,0],transit[:,1])
    #     plt.show()
    #     openShrunkCells = np.delete(openShrunkCells,entry[0],axis=0)#remove that cell from the list(entry[0] is the idx)
    #     path = np.vstack([path,subPath])
    #     print "cell ", i, " done", "entry = ", tmpStr , "exit = ", path[-1,:]
    # plt.figure()
    # plt.plot(points[:,0], points[:,1])
    # plt.plot(path[:,0],path[:,1])
    # print path.shape
    # plt.show()
    #
    # print 'done'
    #
    #
    #
    #
    #
    #
