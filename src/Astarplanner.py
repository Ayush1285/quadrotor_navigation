#!/usr/bin/env python3
import numpy as np
import math
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
from matplotlib import colors
import numpy as np


class Astar:
    def __init__(self,resolution,robotradius):
        
        self.map = np.ones((120,120))*(0)
        self.resolution = resolution
        self.rr = robotradius
        self.Rd = robotradius + 0.2
        self.collisioncostmap = np.zeros((self.map.shape))
        self.krep = 20
        self.parent = np.zeros((120,120))    

    def updatecollisioncostmap(self):
        for i in range(self.map.shape[0]):
            for j in range(self.map.shape[1]):
                if self.map[i,j]==100:
                    for k in range(math.ceil(2*self.Rd/self.resolution)):
                        for l in range(math.ceil(2*self.Rd/self.resolution)):
                            currentobstaclecell = np.array([i,j])
                            othercell = np.array([min(max(0,i-math.ceil(self.Rd/self.resolution))+k,119),min(max(0,j-math.ceil(self.Rd/self.resolution))+l,119)])
                            distancebtwcells = np.linalg.norm(currentobstaclecell-othercell)
                            if distancebtwcells < self.rr/self.resolution:
                                self.collisioncostmap[othercell[0], othercell[1]] = max(self.collisioncostmap[othercell[0],\
                                othercell[1]],math.inf)
                            elif distancebtwcells < self.Rd/self.resolution:
                                self.collisioncostmap[othercell[0], othercell[1]] = max(self.collisioncostmap[othercell[0], othercell[1]],\
                                self.krep*((self.Rd/self.resolution - distancebtwcells)/(self.Rd/self.resolution - self.rr/self.resolution))**2)


    
    def astarplanner(self, start, goal, map):
        self.map = map
        self.updatecollisioncostmap()



        self.fcost = np.ones((self.map.shape))*math.inf  # Astar f costmap initialisation : f = g + h
        self.gcost = np.ones((self.map.shape))*math.inf  # Astar g costmap initialisation : g the distance from start to current node
        self.gcost[start[0],start[1]] = 0
        self.fcost[start[0],start[1]] = self.gcost[start[0],start[1]] + np.linalg.norm(np.array([start[0],start[1]])-goal)\
            +self.collisioncostmap[0,int(self.map.shape[0]/2)-1]
                                                                                            #euclidean distance heuristic

        while True:
            i,j = np.unravel_index(self.fcost.argmin(),self.fcost.shape)

            if self.resolution*np.linalg.norm(np.array([i,j])-goal) < 0.05 or self.fcost[i,j] == math.inf:
                break
            
            self.fcost[i,j] = math.inf

            if self.map[min(i+1,119),j] in [-1,0]:
                if self.gcost[min(i+1,119),j] > self.gcost[i,j] + 1 :
                    self.gcost[min(i+1,119),j] = self.gcost[i,j] + 1
                    self.fcost[min(i+1,119),j] = self.gcost[min(i+1,119),j] + \
                        np.linalg.norm(np.array([min(i+1,119),j])-goal) + self.collisioncostmap[min(i+1,119),j]

                    self.parent[min(i+1,119),j] = int(120*i+j)
            
            if self.map[min(i+1,119),min(j+1,119)] in [-1,0]:
                if self.gcost[min(i+1,119),min(j+1,119)] > self.gcost[i,j] + 1.414 :
                    self.gcost[min(i+1,119),min(j+1,119)] = self.gcost[i,j] + 1.414
                    self.fcost[min(i+1,119),min(j+1,119)] = self.gcost[min(i+1,119),min(j+1,119)] + \
                        np.linalg.norm(np.array([min(i+1,119),min(j+1,119)])-goal) + self.collisioncostmap[min(i+1,119),min(j+1,119)]

                    self.parent[min(i+1,119),min(j+1,119)] = int(120*i+j)

            if self.map[i,min(j+1,119)] in [-1,0]:
                if self.gcost[i,min(j+1,119)] > self.gcost[i,j] + 1 :
                    self.gcost[i,min(j+1,119)] = self.gcost[i,j] + 1
                    self.fcost[i,min(j+1,119)] = self.gcost[i,min(j+1,119)] + \
                        np.linalg.norm(np.array([i,min(j+1,119)])-goal) + self.collisioncostmap[i,min(j+1,119)]

                    self.parent[i,min(j+1,119)] = int(120*i+j)

            if self.map[max(i-1,0),min(j+1,119)] in [-1,0]:
                if self.gcost[max(i-1,0),min(j+1,119)] > self.gcost[i,j] + 1.414 :
                    self.gcost[max(i-1,0),min(j+1,119)] = self.gcost[i,j] + 1.414
                    self.fcost[max(i-1,0),min(j+1,119)] = self.gcost[max(i-1,0),min(j+1,119)] + \
                        np.linalg.norm(np.array([max(i-1,0),min(j+1,119)])-goal) + self.collisioncostmap[max(i-1,0),min(j+1,119)]

                    self.parent[max(i-1,0),min(j+1,119)] = int(120*i+j)

            if self.map[max(i-1,0),j] in [-1,0]:
                if self.gcost[max(i-1,0),j] > self.gcost[i,j] + 1 :
                    self.gcost[max(i-1,0),j] = self.gcost[i,j] + 1
                    self.fcost[max(i-1,0),j] = self.gcost[max(i-1,0),j] + \
                        np.linalg.norm(np.array([max(i-1,0),j])-goal) + self.collisioncostmap[max(i-1,0),j]

                    self.parent[max(i-1,0),j] = int(120*i+j)

            if self.map[max(i-1,0),max(j-1,0)] in [-1,0]:
                if self.gcost[max(i-1,0),max(j-1,0)] > self.gcost[i,j] + 1.414 :
                    self.gcost[max(i-1,0),max(j-1,0)] = self.gcost[i,j] + 1.414
                    self.fcost[max(i-1,0),max(j-1,0)] = self.gcost[max(i-1,0),max(j-1,0)] + \
                        np.linalg.norm(np.array([max(i-1,0),j])-goal) + self.collisioncostmap[max(i-1,0),max(j-1,0)]

                    self.parent[max(i-1,0),max(j-1,0)] = int(120*i+j)

            if self.map[i,max(j-1,0)] in [-1,0]:
                if self.gcost[i,max(j-1,0)] > self.gcost[i,j] + 1 :
                    self.gcost[i,max(j-1,0)] = self.gcost[i,j] + 1
                    self.fcost[i,max(j-1,0)] = self.gcost[i,max(j-1,0)] + \
                        np.linalg.norm(np.array([i,j])-goal) + self.collisioncostmap[i,max(j-1,0)]

                    self.parent[i,max(j-1,0)] = int(120*i+j)

            if self.map[min(i+1,119),max(j-1,0)] in [-1,0]:
                if self.gcost[min(i+1,119),max(j-1,0)] > self.gcost[i,j] + 1.414 :
                    self.gcost[min(i+1,119),max(j-1,0)] = self.gcost[i,j] + 1.414
                    self.fcost[min(i+1,119),max(j-1,0)] = self.gcost[min(i+1,119),max(j-1,0)] + \
                        np.linalg.norm(np.array([min(i+1,119),j])-goal) + self.collisioncostmap[min(i+1,119),max(j-1,0)]

                    self.parent[min(i+1,119),max(j-1,0)] = int(120*i+j)

        path = [self.parent[i,j]]
        while path[0] != 120*start[0]+start[1] :
            i,j = np.unravel_index(int(path[0]), self.map.shape, order='C')
            path = [self.parent[i,j]] + path

        return path



            

                    



            


