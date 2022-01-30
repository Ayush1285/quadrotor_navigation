#!/usr/bin/env python3
import numpy as np

from math import atan2,cos, hypot,sin
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import Pose,PoseStamped
#from scipy import interpolate
from Astarplanner import Astar


class Flight:
    def __init__(self, robotradius, localmapsize):
        self.rr = robotradius
        self.inner_radius = localmapsize/4
        self.globalmap = OccupancyGrid()
        self.pose = Pose()
        #self.pose.position.x = -1.369
        #self.pose.position.y = 10.7708
        self.globalgoal = PoseStamped()
        #self.globalgoal.pose.position.x = -7.617
        #self.globalgoal.pose.position.y = -10.5207
        self.localgoal = (0,0)
        self.resolution = 0.
        self.globalmaporigin = (0.,0.)
        self.localmaporigin = (0.,0.)
        self.maparray = []
        self.startpos = (0,0)
        self.despath = Path()
        self.desyaw = 0.

    def updateinitialpose(self, msg):
        print("start = ",msg.pose.pose.position)
        self.pose.position = msg.pose.pose.position
        self.pose.orientation = msg.pose.pose.orientation

    def updatepose(self, msg):
        #print(msg.position)
        self.pose.position = msg.position
        self.pose.orientation = msg.orientation

    def goalcb(self,msg):
        print("goal = ",msg.pose.position)
        self.desyaw = atan2(msg.pose.position.y-self.pose.position.y,msg.pose.position.x-self.pose.position.x)
        self.globalgoal = msg

    def getmap(self, msg):
        self.globalmap = msg
        width = msg.info.width
        height = msg.info.height
        self.resolution = msg.info.resolution
        self.globalmaporigin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.maparray = np.array(msg.data).reshape(height,width,order='C')

    def occupancycheck(self,goal_in_globfr,angle):
        angle1 = angle
        angle2 = angle
        if self.maparray[goal_in_globfr] == 100:
            while True:
                angle1 = angle1 - 0.17
                angle2 = angle2 + 0.17

                localgoal1 = (self.pose.position.x + cos(angle1)*3*self.inner_radius , self.pose.position.y + sin(angle1)*3*self.inner_radius)
                localgoal2 = (self.pose.position.x + cos(angle2)*3*self.inner_radius , self.pose.position.y + sin(angle2)*3*self.inner_radius)
                goal1_globalfr = self.getarrayindex(self.globalmaporigin,localgoal1)
                goal2_globalfr = self.getarrayindex(self.globalmaporigin,localgoal2)
                if self.maparray[goal1_globalfr] in [-1,0]:
                    angle1 = angle1 - 0.10
                    localgoal1 = (self.pose.position.x + cos(angle1)*3*self.inner_radius , self.pose.position.y + sin(angle1)*3*self.inner_radius)
                    goal1_globalfr = self.getarrayindex(self.globalmaporigin,localgoal1)
                    goal_in_globfr = goal1_globalfr
                    break
                elif self.maparray[goal2_globalfr] in [-1,0]:
                    angle2 = angle2 + 0.10
                    localgoal2 = (self.pose.position.x + cos(angle2)*3*self.inner_radius , self.pose.position.y + sin(angle2)*3*self.inner_radius)
                    goal2_globalfr = self.getarrayindex(self.globalmaporigin,localgoal2)
                    goal_in_globfr = goal2_globalfr
                    break
        #print("og value = ",self.maparray[goal_in_globfr])
            
        return (goal_in_globfr[1]*self.resolution+self.globalmaporigin[0],goal_in_globfr[0]*self.resolution+self.globalmaporigin[1])

    def getarrayindex(self,parent,child):

        return (int((child[1]-parent[1])/self.resolution),int((child[0]-parent[0])/self.resolution))

    def localregion(self,angle):
        localregion_center = (self.pose.position.x + cos(angle)*self.inner_radius , self.pose.position.y + sin(angle)*self.inner_radius)
        self.localmaporigin = (localregion_center[0]-self.inner_radius*2, localregion_center[1]-self.inner_radius*2)
        uprightcorner = (localregion_center[0]+self.inner_radius*2, localregion_center[1]+self.inner_radius*2)
        localgoal = (self.pose.position.x + cos(angle)*3*self.inner_radius , self.pose.position.y + sin(angle)*3*self.inner_radius)

        return uprightcorner,localgoal

    def getlocalmap(self):
        slopeangle = atan2(self.globalgoal.pose.position.y - self.pose.position.y , self.globalgoal.pose.position.x - self.pose.position.x)
        uprightcorner, localgoal = self.localregion(slopeangle)
        if hypot(self.globalgoal.pose.position.x-self.pose.position.x,self.globalgoal.pose.position.y-self.pose.position.y) < 3*self.inner_radius:
            localgoal = (self.globalgoal.pose.position.x,self.globalgoal.pose.position.y)
        # converting to array indexes
        #print("localgoal = ",localgoal)
        localmaporigin = self.getarrayindex(self.globalmaporigin,self.localmaporigin)
        uprightcorner = self.getarrayindex(self.globalmaporigin,uprightcorner)
        goalindex_in_globfr = self.getarrayindex(self.globalmaporigin,localgoal)
        self.startpos = self.getarrayindex(self.localmaporigin,(self.pose.position.x,self.pose.position.y))

        updated_goal = self.occupancycheck(goalindex_in_globfr,slopeangle)   # updated goal coordinates
    
        #self.localgoal = (updated_goal[0]-localmaporigin[0],updated_goal[1]-localmaporigin[1])

        slopeangle = atan2(updated_goal[1]-self.pose.position.y,updated_goal[0]-self.pose.position.x)
        uprightcorner, localgoal = self.localregion(slopeangle)
        self.localgoal = self.getarrayindex(self.localmaporigin,updated_goal)
        self.startpos = self.getarrayindex(self.localmaporigin,(self.pose.position.x,self.pose.position.y))
        localmaporigin = self.getarrayindex(self.globalmaporigin,self.localmaporigin)
        uprightcorner = self.getarrayindex(self.globalmaporigin,uprightcorner)
        #print(localmaporigin,uprightcorner)

        localmap = self.maparray[localmaporigin[0]-1:uprightcorner[0]+1,localmaporigin[1]-1:uprightcorner[1]+1]
        return localmap

    def runplanner(self):
        #print("pose",self.pose.position)
        localmap = self.getlocalmap()
        #print(localmap[self.startpos])
        planner = Astar(self.resolution,self.rr,localmap,self.localmaporigin[0],self.localmaporigin[1])
        #print(self.startpos,self.localgoal)
        #print("goal1 = ",(self.localgoal[1]*self.resolution+self.localmaporigin[0],self.localgoal[0]*self.resolution+self.localmaporigin[1]))
    
        self.despath = planner.astarplanner(self.startpos,self.localgoal,self.desyaw)
        #print(self.despath.poses[0])
    
    
        index = min(range(len(self.despath.poses)), key=lambda i:hypot(self.despath.poses[i].pose.position.x-self.pose.position.x,\
            self.despath.poses[i].pose.position.y-self.pose.position.y))
        self.despath.poses[0:index] = []
    
        
       
        return self.despath