__author__ = 'Admin'

import Lines
import numpy as np
import math
from operator import itemgetter
import matplotlib.pyplot as plt

class TesselatedPolygonGraph():
    def __init__(self,poly, dxy, eight=False):
        self.poly = poly
        self.dxy = dxy
        self.Li = Lines.Intersections()
        self.eight = eight

    def neighbours(self,loc):
        x = loc[0]
        y = loc[1]
        results = []
        n = []
        n.append([round(x+self.dxy,8), y])
        n.append([round(x-self.dxy,8), y])
        n.append([x, round(y+self.dxy,8)])
        n.append([x,round(y-self.dxy,8)])
        if self.eight == True:
            n.append([round(x+self.dxy,8),round(y+self.dxy,8)])
            n.append([round(x+self.dxy,8),round(y-self.dxy,8)])
            n.append([round(x-self.dxy,8),round(y+self.dxy,8)])
            n.append([round(x-self.dxy,8),round(y-self.dxy,8)])
        for ni in n:
            if self.Li.segmentCartInOrOnPoly(self.poly,x,y,ni[0],ni[1]):
                results.append(ni)
        return results


class CellularDecompGraph():
    def __init__(self):
        pass

    def getFace(self,pos,cells):
        Li = Lines.Intersections()
        face = 0
        for cell in cells:
            cell = np.asarray(cell) #in case coming in as a list
            if Li.isPointInPolygon(pos[0],pos[1],cell,rnd=6):
                return face
            else:
                face +=1
        return None #pos not in any of the cells

    def neighbours(self, pos, face, shrunckCells, neighboursList):
        #return all corners in current and neigbouring faces
        #pos = [round(n,2) for n in pos]
        neighbouringFacesL = []
        neighbouringFacesR = []
        neighbouringCnrs = []
        neighboursIdxR = [i for i, x in enumerate(neighboursList) if x[0] == face]
        neighboursIdxL = [i for i, x in enumerate(neighboursList) if x[1] == face]
        for idx in neighboursIdxR:
            neighbouringFacesR.append(neighboursList[idx][1])
        for idx in neighboursIdxL:
            neighbouringFacesL.append(neighboursList[idx][0])
        #add corners of current face
        # note this can go across illegal space if the boundary is concave
        # could potentially remove any neighbours that cross the boundary and replace them with
        # the crossing point
        print 'face', face
        print 'pos', pos
        print 'shrunckCells',shrunckCells
        for cnrs in shrunckCells[face, :, :]:
            #for cnr in cnrs:
            #print 'cnrs from AStar.CellularDecompGraph.neighbours'
            print cnrs[0],  pos[0], cnrs[1], pos[1]
            if cnrs[0] != pos[0] or cnrs[1] != pos[1]:
                neighbouringCnrs.append(cnrs)
        #if in a corner, add the 2 corners from each cell joining on that side
        try:
            faceIdx = shrunckCells[face].tolist().index(pos)
            if faceIdx < 2:
                for faces in neighbouringFacesL:
                    for cnrs in shrunckCells[faces, 2:4, :]:
                        neighbouringCnrs.append(cnrs)
            elif faceIdx >=2:
                for faces in neighbouringFacesR:
                    for cnrs in shrunckCells[faces, :2, :]:
                        neighbouringCnrs.append(cnrs)
        except ValueError:
            pass #no current in a corner so don't add neighrbouring cells. this iteration will move to
            # a corner and then the next one can go into a neighbouring cell
        return neighbouringCnrs


class Search():
    def __init__(self):
        self.nodes = [] # f, g, h, loc, parent

        # Parent in the index of the parent node
        # loc is the x,y position of the node
        # f = cost of the node
        # g = cost to get to the node
        # h = hueristic cost from node to goal

    def heuristic(self, a, b):
        (x1, y1) = a
        (x2, y2) = b
        return math.sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2))

    def AStarTess(self,poly, dxy, start, goal, eight=False,minDist=9999999999):
        #based on psuedocode from http://web.mit.edu/eranki/www/tutorials/search/
        self.graph = TesselatedPolygonGraph(poly,dxy,eight)
        if type(start).__module__ == np.__name__:
            start = start.tolist()
        if type(goal).__module__ == np.__name__:
            goal = goal.tolist()
        self.openList = []
        self.closedList = []
        g = 0
        h = self.heuristic(start,goal)
        f = g + h
        #self.openList.append([f,g,h,start,None])

        if f == 0:
            self.closedList.append([f,g,h,start,None])
            return g, start
        elif f > minDist:
            #using this to stop searching if hueristic (which is straight line euclidian) is
            #further than the best we have found so far
            return f, None
        else:
            self.openList.append([f,g,h,start,None])

        while len(self.openList) > 0:
            #print "length of openList in AstarTess()", len(self.openList),len(self.closedList), dxy, eight
            self.openList.sort(key=itemgetter(0)) #sorts by f
            q = self.openList.pop(0) #get node with smallest f
            tmp = q[3]
            children = self.graph.neighbours(tmp)
            for child in children:
                # check if child in the open or closed list already, if so skip, else add to open list
                if self.isCellInList(child,self.openList) == False and self.isCellInList(child,self.closedList) == False:
                    g = q[1]+self.heuristic(q[3],child)
                    h = self.heuristic(child,goal)
                    f = g + h
                    #test if within dxy of the goal
                    if h < dxy:
                        self.openList = []
                        self.closedList.append(q)
                        self.closedList.append([f,g,h,child,q[3]])
                        return g, child
                    else:
                        self.openList.append([f,g,h,child,q[3]])
            self.closedList.append(q)
        return 9999999999, None

    def AStarCellOld(self, shrunkCells,CellPolygons ,neighboursList, start, goal,minDist=9999999999):
        self.graph = CellularDecompGraph()

        #if numpy array, convert to a list
        if type(start).__module__ == np.__name__:
            start = start.tolist()
        if type(goal).__module__ == np.__name__:
            goal = goal.tolist()
        self.openList = []
        self.closedList = []
        g = 0
        h = self.heuristic(start,goal)
        f = g + h
        if f == 0:
            self.closedList.append([f,g,h,start,None])
            return g
        elif f > minDist:
            #using this to stop searching if hueristic (which is straight line euclidian) is
            #further than the best we have found so far
            return None
        else:
            self.openList.append([f,g,h,start,None])

        while len(self.openList) > 0:
            self.openList.sort(key=itemgetter(0)) #sorts by f
            q = self.openList.pop(0) #get node with smallest f
            #tmp = [round(n,6) for n in q[3]]
            tmp = q[3]
            face = self.graph.getFace(tmp,CellPolygons)
            #print tmp, face, shrunkCells, neighboursList
            children = self.graph.neighbours(tmp, face, shrunkCells, neighboursList)
            for child in children:
                if type(child).__module__ == np.__name__:
                    child = child.tolist()
                # check if child in the open or closed list already, if so skip, else add to open list
                if self.isCellInList(child,self.openList) == False and self.isCellInList(child,self.closedList) == False:
                    g = q[1]+self.heuristic(q[3],child)
                    h = self.heuristic(child,goal)
                    f = g + h
                    #test if this is the goal
                    if child[0] == goal[0] and child[1] == goal[1]:
                        self.openList = []
                        self.closedList.append(q)
                        self.closedList.append([f,g,h,child,q[3]])
                        return g
                    else:
                        self.openList.append([f,g,h,child,q[3]])
            self.closedList.append(q)
        return None

    def AStarCell(self, shrunkCells, neighboursList, start, goal,minDist=9999999999):
        self.graph = CellularDecompGraph()

        #if numpy array, convert to a list
        if type(start).__module__ == np.__name__:
            start = start.tolist()
        if type(goal).__module__ == np.__name__:
            goal = goal.tolist()
        self.openList = []
        self.closedList = []
        g = 0
        h = self.heuristic(start,goal)
        f = g + h
        if f == 0:
            self.closedList.append([f,g,h,start,None])
            return g
        elif f > minDist:
            #using this to stop searching if hueristic (which is straight line euclidian) is
            #further than the best we have found so far
            return None
        else:
            self.openList.append([f,g,h,start,None])

        while len(self.openList) > 0:
            self.openList.sort(key=itemgetter(0)) #sorts by f
            q = self.openList.pop(0) #get node with smallest f
            #tmp = [round(n,6) for n in q[3]]
            tmp = q[3]
            face = self.graph.getFace(tmp,shrunkCells)
            #print tmp, face, shrunkCells, neighboursList
            children = self.graph.neighbours(tmp, face, shrunkCells, neighboursList)
            for child in children:
                if type(child).__module__ == np.__name__:
                    child = child.tolist()
                # check if child in the open or closed list already, if so skip, else add to open list
                # t1 = self.isCellInList(child,self.openList)
                # t2 = self.isCellInList(child,self.closedList)
                if self.isCellInList(child,self.openList) == False and self.isCellInList(child,self.closedList) == False:
                    g = q[1]+self.heuristic(q[3],child)
                    h = self.heuristic(child,goal)
                    f = g + h
                    #test if this is the goal
                    if child[0] == goal[0] and child[1] == goal[1]:
                        self.openList = []
                        self.closedList.append(q)
                        self.closedList.append([f,g,h,child,q[3]])
                        return g
                    else:
                        self.openList.append([f,g,h,child,q[3]])
            self.closedList.append(q)
        return None

    def getPath(self,goal):
        #step back from goal to start via parent mappings to get path
        if type(goal).__module__ == np.__name__:
            goal = goal.tolist()
        parent = goal
        self.path = []
        while parent is not None:
            tmp = [x[3]for x in self.closedList]
            idx = tmp.index(parent)
            node = self.closedList.pop(idx)
            parent = node[4]
            self.path.append(node[3])
        self.path.reverse()
        return self.path

    def isCellInList(self, cell,list):
        #cell = cell.tolist() #convert from numpy array to list
        try:
            tmp = [x[3] for x in list]
            res = tmp.index(cell)
            return True
        except ValueError:
            return False
