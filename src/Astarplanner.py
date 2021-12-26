#!/usr/bin/env python3
import numpy as np
import math
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
from matplotlib import colors
import scipy.signal as signal
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from math import sqrt
from rdp import runrdp


class Astar:
    def __init__(self,resolution,robotradius):
        
        self.map = np.ones((120,120))*(0)
        self.resolution = resolution
        self.rr = robotradius
        self.Rd = robotradius + 0.1
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

   
    def linesegment(self, p1, p2):

        length = sqrt((p1[1]-p2[1])**2 + (p1[0]-p2[0])**2)

        if p2[0] == p1[0]:
            py = np.linspace(p1[1],p2[1], int(length/0.05) )
            px = np.ones(len(py))*p1[0]
            return np.c_[px[1:],py[1:]]


        slope = ((p2[1]-p1[1])/(p2[0]-p1[0]))
        px = np.linspace(p1[0],p2[0],int(length/0.05))
        py = slope*(px - np.ones(len(px))*p1[0]) + np.ones(len(px))*p1[1]

        return np.c_[px[1:],py[1:]]

    def astarplanner(self, start, goal, map):
        self.map = map
        self.updatecollisioncostmap()
        #self.collisioncostmap = self.collisioncostmap*0



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
                if self.gcost[min(i+1,119),min(j+1,119)] > self.gcost[i,j] + 1.4 :
                    self.gcost[min(i+1,119),min(j+1,119)] = self.gcost[i,j] + 1.4
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
                if self.gcost[max(i-1,0),min(j+1,119)] > self.gcost[i,j] + 1.4 :
                    self.gcost[max(i-1,0),min(j+1,119)] = self.gcost[i,j] + 1.4
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
                if self.gcost[max(i-1,0),max(j-1,0)] > self.gcost[i,j] + 1.4 :
                    self.gcost[max(i-1,0),max(j-1,0)] = self.gcost[i,j] + 1.4
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
                if self.gcost[min(i+1,119),max(j-1,0)] > self.gcost[i,j] + 1.4 :
                    self.gcost[min(i+1,119),max(j-1,0)] = self.gcost[i,j] + 1.4
                    self.fcost[min(i+1,119),max(j-1,0)] = self.gcost[min(i+1,119),max(j-1,0)] + \
                        np.linalg.norm(np.array([min(i+1,119),j])-goal) + self.collisioncostmap[min(i+1,119),max(j-1,0)]

                    self.parent[min(i+1,119),max(j-1,0)] = int(120*i+j)

        pathros = Path()
        pathros.header.frame_id = "map"
        path = [int(self.parent[i,j])]
        ypos,xpos = np.unravel_index(path[0],self.map.shape)
        ypos = [self.resolution*ypos]
        xpos = [self.resolution*xpos]
        points = [[xpos[0],ypos[0]]]
        while path[0] != 120*start[0]+start[1] :
            #pose = PoseStamped()
            #pose.header.frame_id = 'map'
            i,j = np.unravel_index(int(path[0]), self.map.shape, order='C')
            path = [int(self.parent[i,j])] + path
            yp,xp = np.unravel_index(path[0],self.map.shape)
            xp = xp*self.resolution
            yp = yp*self.resolution
            points = [[xp,yp]]+points
            xpos = [xp]+xpos
            ypos = [yp]+ypos
            #pose.pose.position.x = self.resolution*xp
            #pose.pose.position.y = self.resolution*yp
            #pathros.poses.append(pose)

        lines = runrdp(points,0.1)
        lines = np.array(lines)
        
        for i in range(len(lines[:,0])-1):
            linesegment = self.linesegment(lines[i,:],lines[i+1,:])
            if i == 0:
                new_lines = np.reshape(lines[i,:],(1,2))
                print(new_lines.shape)
            new_lines = np.r_[new_lines,linesegment,lines[i+1,:].reshape((1,2))]




        z = np.polyfit(new_lines[:,0], new_lines[:,1], 8)
        f = np.poly1d(z)
        x_new = np.linspace(new_lines[0,0], new_lines[-1,0], len(new_lines[:,0]))
        y_new = f(x_new)
        for i in range(len(new_lines[:,0])):
        #for i in range(len(xpos)):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = new_lines[i,0]
            #pose.pose.position.x = x_new[i]
            #pose.pose.position.x = xpos[i]

            #pose.pose.position.y = ypos[i]
            #pose.pose.position.y = y_new[i]
            pose.pose.position.y = new_lines[i,1]
            pathros.poses.append(pose)
        return pathros



            

                    



            


