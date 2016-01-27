__author__ = 'Admin'

import SBContBound
import matplotlib.pyplot as plt
import numpy as np
import DMPP
import TSP
import time
import Lines

timer = time.time()
LI = Lines.Intersections()
reSim = False

if reSim == True:
    sbcb = SBContBound.SBContBound(fitmodel="mygp")
    sbcb.initialScan()
    sbcb.contBoundScan()

    travelled = sbcb.pathtraveled
    np.save('travelled',travelled)
    scannedPoints = sbcb.scannedPoints[:sbcb.scounter,:]
    np.save('scannedPoints',scannedPoints)
    hull = LI.externalHull(travelled,sbcb.r)
    pos = [sbcb.CGh[0,3],sbcb.CGh[1,3]]
else:
    travelled = np.load('travelled.npy')
    scannedPoints = np.load('scannedPoints.npy')
    hull = LI.externalHull(travelled,5.0)
    boundingPolygon = hull
    pos = travelled[-1]
    # fig = plt.figure()
    # plt.plot(travelled[:, 0], travelled[:, 1], color = 'blue', linestyle = '-', label = 'Travelled')
    # plt.plot(boundingPolygon[:, 0], boundingPolygon[:, 1], color = '#A901DB', linestyle = '-', label = 'Hull')
    # plt.show()

boundingPolygon = hull
sweepAngle = 0
dxy = 10
polyDir =  LI.rotDirection(boundingPolygon)

crossingAr, crossingCount, tmppos = DMPP.sweep(boundingPolygon, sweepAngle, dxy)
openCells, neighbours = DMPP.createCells(crossingAr,crossingCount,dxy,sweepAngle)
openShrunkCells = DMPP.shrinkCells(openCells,dxy, sweepAngle)
TSP = TSP.Tsp()
path = np.array([pos])

subPaths = np.empty(4,dtype='object')
transits = np.empty(4,dtype='object')
lps = np.empty(4,dtype='object')

for i in range(openCells.shape[0]):
    lastPoint = path[-1]
    entry, transit = TSP.greedyCellTSP2(openShrunkCells, lastPoint, boundingPolygon, dxy, 5)
    # note due to cells being monotone to sweep direction, not to the normal of the transit direction, the transit path
    # plan can cross the polygon boundary
    # need to write function to hug boundary instead of crossing it
    # Done - A* handles this
    transit = np.asarray(transit)
    if transit != []:
        path = np.vstack([path,transit])
    #tmpStr = openCells[entry[0]][entry[1]]
    cellDir = LI.rotDirection(openCells[entry[0]])
    lp = DMPP.localPoly(openCells[entry[0]],boundingPolygon)#, cellDir, polyDir)
    subPath = DMPP.genLawnmower(openCells[entry[0]], boundingPolygon,lp,entry[1],sweepAngle,dxy)
    openCells = np.delete(openCells,entry[0],axis=0) #remove that cell from the list
    openShrunkCells = np.delete(openShrunkCells, entry[0],axis=0)
    path = np.vstack([path,subPath])
    #for plotting
    print i
    subPaths[i] = subPath
    transits[i] = transit
    lps[i] = lp





#Hull and 2D path
fig = plt.figure()
plt.plot(boundingPolygon[:, 0], boundingPolygon[:, 1], color = '#A901DB', linestyle = '-', label = 'Hull')
first = True
for t in transits:
    if first == True:
        #plt.plot(t[:,0], t[:,1], color = '#81F7F3', linestyle = '-', label = 'Transit')
        first = False
    else:
        plt.plot(t[:,0], t[:,1], color = '#81F7F3', linestyle = '-')
plt.plot(subPaths[0][:,0],subPaths[0][:,1],color = '#58D3F7', linestyle = '-', label = 'Path 1')
plt.plot(subPaths[1][:,0],subPaths[1][:,1],color = '#58ACFA', linestyle = '-', label = 'Path 2')
plt.plot(subPaths[2][:,0],subPaths[2][:,1],color = '#2E64FE', linestyle = '-', label = 'Path 3')
plt.plot(subPaths[3][:,0],subPaths[3][:,1],color = '#0000FF', linestyle = '-', label = 'Path 4')
plt.legend(loc=2)
plt.savefig('.\images\simHullSubPaths.png', bbox_inches='tight')

#batymetry, boundary and sonar points in 3D
if reSim == True:
    sbcb.bath.print3D(invert=True)
else:
    sbcb = SBContBound.SBContBound(fitmodel="mygp") #running this again to get the 3D print of the bath incase we are running from memory

plt.plot(scannedPoints[:,0], scannedPoints[:,1], scannedPoints[:,2]*(-1),
         label = 'Sonar Points', color = 'Maroon',  marker='.', linestyle='None', markersize = 3 )
plt.plot(travelled[:, 0], travelled[:, 1], label = 'Boundary', color = 'Violet',  marker='.', linestyle='None' , markersize = 1)
plt.legend()
plt.savefig('.\images\simBathPath.png', bbox_inches='tight')
plt.savefig('.\images\simBathPath.pdf', bbox_inches='tight')

fig5 = plt.figure()
plt.plot(path[2:,0], path[2:,1], label = 'Path', color = 'Crimson',  marker='None', linestyle='-',  markersize = 1)
plt.plot(hull[:, 0], hull[:, 1], label = 'Hull', color = 'Indigo',  marker='None', linestyle='-' , markersize = 1)
plt.xlabel('Easting (m)')
plt.ylabel('Northing (m)')
plt.legend()
plt.savefig('.\images\simHullPath.png', bbox_inches='tight')
plt.savefig('.\images\simHullPath.pdf', bbox_inches='tight')
print "tot time", timer-time.time()
plt.show()

