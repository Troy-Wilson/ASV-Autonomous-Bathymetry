__author__ = 'Admin'

import SBContBound
import matplotlib.pyplot as plt
import numpy as np
import DMPP
import TSP
import time
import Lines
from matplotlib.offsetbox import AnchoredOffsetbox, TextArea, DrawingArea, HPacker, VPacker
from matplotlib.patches import Rectangle, Circle, Polygon, Ellipse, Arrow, FancyArrow, ArrowStyle,PathPatch
from matplotlib import cm
from matplotlib.ticker import MaxNLocator
import Transformations as tf
import numpy as np

from matplotlib import rcParams
rcParams['xtick.direction'] = 'out'
rcParams['ytick.direction'] = 'out'

Tf = tf.Tf()
timer = time.time()
LI = Lines.Intersections()
reSim = True

if reSim == True:
    sbcb = SBContBound.SBContBound(fitmodel="onlineGP")
    sbcb.initialScan()
    sbcb.contBoundScan()

    travelled = sbcb.pathtraveled
    np.save('./npy/travelled',travelled)
    scannedPoints = sbcb.scannedPoints[:sbcb.scounter,:]
    np.save('./npy/scannedPoints',scannedPoints)
    hull = LI.externalHull(travelled,sbcb.r)
    pos = [sbcb.CGh[0,3],sbcb.CGh[1,3]]
else:
    travelled = np.load('./npy/travelled.npy')
    scannedPoints = np.load('./npy/scannedPoints.npy')
    hull = LI.externalHull(travelled,10)
    boundingPolygon = hull
    pos = travelled[-1]

#bump pos inside poly
pos[1] = pos[1]+ 0.1

boundingPolygon = hull
sweepAngle = 0
dxy = 10
reSim = False
if reSim == True:
    polyDir =  LI.rotDirection(boundingPolygon)
    crossingAr, crossingCount, tmppos = DMPP.sweep(boundingPolygon, sweepAngle, dxy)
    openCells, neighbours = DMPP.createCells(crossingAr,crossingCount,dxy,sweepAngle)
    openShrunkCells = DMPP.shrinkCells(openCells,dxy, sweepAngle)
    TSP = TSP.Tsp()
    path = np.array([pos])
    cellcount = len(openShrunkCells)

    subPaths = np.empty(cellcount,dtype='object')
    transits = np.empty(cellcount,dtype='object')
    lps = np.empty(cellcount,dtype='object')

    for i in range(openCells.shape[0]):
        lastPoint = path[-1]
        entry, transit = TSP.greedyCellTSP2(openShrunkCells, lastPoint, boundingPolygon, dxy, 1)
        transit = np.asarray(transit)
        if transit != []:
            path = np.vstack([path,transit])
        else:
            transit = np.vstack([lastPoint,openShrunkCells[entry[0]][entry[1]]])
        #tmpStr = openCells[entry[0]][entry[1]]
        cellDir = LI.rotDirection(openCells[entry[0]])
        lp = DMPP.localPoly(openCells[entry[0]],boundingPolygon)#, cellDir, polyDir)
        subPath = DMPP.genLawnmower(openCells[entry[0]], boundingPolygon,lp,entry[1],sweepAngle,dxy)
        openCells = np.delete(openCells,entry[0],axis=0) #remove that cell from the list
        openShrunkCells = np.delete(openShrunkCells, entry[0],axis=0)
        if subPath != []:
            path = np.vstack([path,subPath])
        #for plotting
        print i
        subPaths[i] = subPath
        transits[i] = transit
        lps[i] = lp

    np.save('./npy/path', path)
    np.save('./npy/sp', subPaths)
    np.save('./npy/tran', transits)
    np.save('./npy/lps', lps)
else:
    path = np.load('./npy/path.npy')
    subPaths = np.load('./npy/sp.npy')
    transits = np.load('./npy/tran.npy')
    lps = np.load('./npy/lps.npy')
cellcolours = ['#E0E0E0','#C8C8C8','#B0B0B0','#989898']

#load other paths
#pathr2_5 = np.load('./npy/scannedPointsSimR2_5.npy')
#pathr7_5 = np.load('./npy/scannedPointsSimR7_5.npy')

boundingPolygon = np.vstack([boundingPolygon, boundingPolygon[0,:]])

fig = plt.figure(figsize=(15,6), facecolor = 'white')
adj = plt.subplots_adjust(hspace=0.3,wspace=0.5, bottom=0.2)
ax3 = fig.add_subplot(122)
ax3.set_title('Contour/Boundary Path and Coverage (b)', fontsize=12, y=-.2)
#ax3.axes.get_xaxis().set_visible(False)
#ax3.axes.get_yaxis().set_visible(False)
ax3.set_xlim([-110, 510])
ax3.set_ylim([-10, 510])
ax3.set_ylabel('Northing (m)')
ax3.set_xlabel('Easting (m)')
start, = plt.plot(pos[0],pos[1], markersize = 10, marker = '.', color = '#6600FF', linewidth = 0, label= 'Start')
finish, = plt.plot(path[-1,0],path[-1,1], markersize = 5, marker = 'D', color = '#6600FF', linewidth = 0, markeredgecolor = 'none', label='Finish')
plt.plot(boundingPolygon[:,0], boundingPolygon[:,1], color = '#6600FF', linewidth = 0.5 , label='Path r=5')

#colour in the patches of each cell
poly0 = Polygon(lps[0],color=cellcolours[0])
poly1 = Polygon(lps[1],color=cellcolours[1])
poly2 = Polygon(lps[2],color=cellcolours[2])
poly3 = Polygon(lps[3],color=cellcolours[3])
ax3.add_patch(poly0)
ax3.add_patch(poly1)
ax3.add_patch(poly2)
ax3.add_patch(poly3)


path0, = plt.plot(subPaths[0][:,0], subPaths[0][:,1], color = '#9900FF', linewidth = 1.5, label='Path')
for i in range(1,subPaths.shape[0]):
    plt.plot(subPaths[i][:,0], subPaths[i][:,1], color = '#9900FF', linewidth = 1.5)

transit0, = plt.plot(transits[0][:,0], transits[0][:,1], color = '#9966FF', linestyle=':', linewidth = 3, label='Transit')
for i in range(1,transits.shape[0]):
    plt.plot(transits[i][:,0], transits[i][:,1], color = '#9966FF', linestyle=':', linewidth = 3)
plt.legend(numpoints=1, loc=2, fontsize = 12)
edPath1 = 0
edTransit1 = 0
for i in range(len(subPaths)):
    for j in range(subPaths[i].shape[0]-1):
        edPath1 += np.sqrt((subPaths[i][1,0]-subPaths[i][0,0])**2 + (subPaths[i][1,1]-subPaths[i][0,1])**2)

for i in range(len(transits)):
    for j in range(transits[i].shape[0]-1):
        edTransit1 += np.sqrt((transits[i][1,0]-transits[i][0,0])**2 + (transits[i][1,1]-transits[i][0,1])**2)



#batymetry, boundary and sonar points in 3D
sbcb = SBContBound.SBContBound(fitmodel="onlineGP")
xlen = 510
ylen = 510
count = 50
x = np.linspace(-110,xlen, count)
y = np.linspace(-10,ylen, count)
X, Y = np.meshgrid(y, x) #used for cf plot
depth = np.empty([count,count])
Xstar = np.empty([count*count,2])
k = 0
for i in range(count):
    for j in range(count):
        Xstar[k,0] = x[i]
        Xstar[k,1] = y[j]
        k+=1
k=0

for i in range(count):
    for j in range(count):
        #depth[i,j] = Ystar[k]
        posCG = np.array([[x[i]], [y[j]], [0]])
        rotCG = Tf.rotFixedRPY(0.0, 0.0, 0.0)
        CGh = Tf.homogenousMatrix(rotCG, posCG)
        #tmp = sbcb.sb.getReadingsH(CGh)
        depth[i,j] = sbcb.sb.getReadingsH(CGh)[2]
        k+=1

cmap = cm.gray_r
depth = depth
levels = MaxNLocator(nbins=30).tick_values(depth.min(), depth.max()) #flipped max and min as I am flipping depthlevels = MaxNLocator(nbins=30).tick_values(depth.min(), depth.max()) #flipped max and min as I am flipping depth
ax1 = fig.add_subplot(121)
ax1.set_title('Contour/Boundary Path and Bathymetry (a)', fontsize=12, y=-.2)
cf = plt.contourf(Y, X, depth, levels,
                 cmap=cm.get_cmap(cmap, len(levels) - 1),
                 )
ax1.set_xlim([-110, 510])
ax1.set_ylim([-10, 510])
ax1.set_ylabel('Northing (m)')
ax1.set_xlabel('Easting (m)')
#ax1.axes.get_xaxis().set_visible(False)
#ax1.axes.get_yaxis().set_visible(False)
start, = plt.plot(scannedPoints[0,0],scannedPoints[0,1], markersize = 10, marker = '.', color = '#6600FF', linewidth = 0, label= 'Start')
finish, = plt.plot(scannedPoints[-1,0],scannedPoints[-1,1], markersize = 5, marker = 'D', color = '#6600FF', linewidth = 0, markeredgecolor = 'none', label='Finish')
#plt.plot(pathr2_5[:,0], pathr2_5[:,1], color = '#6600FF', linewidth = 0.5 , label='Path r=2.5', linestyle=':')
plt.plot(scannedPoints[:,0], scannedPoints[:,1], label = 'Path r=5', color = '#9900FF',  marker='None', linestyle='-',  linewidth = 1)
#plt.plot(pathr7_5[:,0], pathr7_5[:,1], color = '#6600FF', linewidth = 0.5 , label='Path r=7.5', linestyle='--')
plt.plot([0,500,500,0,0],[0,0,500,500,0], label = 'Boundary', color = 'black',  marker='None', linestyle='--',  linewidth = 1)
plt.legend(numpoints=1, loc=2, fontsize = 12)
axColor = plt.axes([.445, .2, .03, .7]) #([from left, from bottom, width, height])
cb = fig.colorbar(cf, cax=axColor, label='Depth', ticks=[0,4,8,12,16,20])
cb.ax.invert_yaxis()

plt.savefig('./images/simulation.png', bbox_inches='tight')
plt.show()
