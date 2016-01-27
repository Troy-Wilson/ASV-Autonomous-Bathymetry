__author__ = 'troy'

# http://nbviewer.ipython.org/url/norvig.com/ipython/TSPv3.ipynb

import numpy as np
#import math
import AStar
import Lines
from matplotlib import pyplot as plt

class Tsp():
    def __init__(self):
        self.LI = Lines.Intersections()

    def greedyCellTSP(self, openCells, pos, poly, discretisation):
        # Find solution from position to next next nearest open cell corner, staying within polygons
        # First try straight line. i fthe shortest striaght line doesn't cross the polygon use it
        # Else iterate through A* solutions on a 4 point grid (8 point grid can cause path to cut corners
        # as only the start and end points are tested for vailidity, not the path). Note also if the discretisation of
        # the poly is smaller than the path discretisation, again we can end up with invalid path since only the
        # end points are tested. Start with the shortest path, then resort list based on distance,
        # Stop once the shortest distance path is feasible.

        cellCount = openCells.shape[0]
        minValidDist = 99999999
        closest = np.empty([2])
        s = AStar.Search()
        data = []
        for i in range(cellCount):
            for j in range(4):
                #try straight line first
                dist = np.sqrt((pos[0]-openCells[i,j,0])**2 + (pos[1]-openCells[i,j,1])**2 )
                validPath = not self.LI.segmentCartCrossPoly(poly,pos[0],pos[1],openCells[i,j,0],openCells[i,j,1],bump=True)
                cnrIdx = [i,j]
                path = np.array([pos,openCells[i,j]])
                data.append([dist,cnrIdx,validPath,path,0])
                if validPath == True:
                    if dist<minValidDist:
                        minValidDist = dist

        data.sort()
        while data[0][2] == False:
            dist, closestPoint = s.AStarTess(poly, discretisation,pos,openCells[data[0][1][0],data[0][1][1]], eight = False, minDist=minValidDist)
            path = s.getPath(closestPoint)
            data[0] = [dist,data[0][1],True,path,1]
            if dist<minValidDist:
                minValidDist = dist
            data.sort()
        return data[0][1], data[0][3]

    def greedyCellTSP2(self, openCells, pos, poly, discretisation,roughMult=5):
        # as above but larger step size used initially and then refined back down. Note this larger step size and
        # using an 8 gid on refinement can cause invalid paths as we are only testing end points.


        cellCount = openCells.shape[0]
        minValidDist = 99999999
        closest = np.empty([2])
        s = AStar.Search()
        data = []
        for i in range(cellCount):
            for j in range(4):
                #try straight line first
                dist = np.sqrt((pos[0]-openCells[i,j,0])**2 + (pos[1]-openCells[i,j,1])**2 )
                validPath = not self.LI.segmentCartCrossPoly(poly,pos[0],pos[1],openCells[i,j,0],openCells[i,j,1],bump=True)
                cnrIdx = [i,j]
                path = np.array([pos,openCells[i,j]])
                data.append([dist,cnrIdx,validPath,path,0])
                if validPath == True:
                    if dist<minValidDist:
                        minValidDist = dist

        data.sort()
        while data[0][2] == False:
            dist, closestPoint = s.AStarTess(poly, discretisation*roughMult,pos,openCells[data[0][1][0],data[0][1][1]], eight = False, minDist=minValidDist)
            path = s.getPath(closestPoint)
            data[0] = [dist,data[0][1],True,path,1]
            if dist<minValidDist:
                minValidDist = dist
            data.sort()
        #print "minValidDist", minValidDist
        #print "refining a* soltuion for start, goal, dxy", pos, openCells[data[0][1][0],data[0][1][1]], discretisation
        #if roughMult != 1:
        if data[0][4] == 1: #refine A* path with finer discretisation
            dist, closestPoint = s.AStarTess(poly, discretisation,pos,openCells[data[0][1][0],data[0][1][1]], eight = True, minDist=minValidDist)
            path = s.getPath(closestPoint)
            data[0] = [dist,data[0][1],True,path,1]
        return data[0][1], data[0][3]


if __name__ == "__main__":
    TSP = Tsp()
    openCells = None
    cells = None
    neighboursList = None
    pos = None
    gctsp2 = TSP.greedyCellTSP2(openCells, cells, neighboursList, pos)

