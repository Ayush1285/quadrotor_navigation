#!/usr/bin/env python3
import numpy as np
import matplotlib as mpl
from matplotlib import pyplot
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from scipy import interpolate
from cubicspline import calc_2d_spline_interpolation
from genmap import generatemap
from Astarplanner import Astar
import rospy
from tf.transformations import quaternion_from_euler
import timeit

map = generatemap()
mapdata = map.flatten()
mapdata = mapdata.astype('int8')


def main():



    def startcb(data):
        global start
        start = np.array([int(data.pose.pose.position.y/0.08),int(data.pose.pose.position.x/0.08)])
        

    def goalcb(data):
        navgoal = np.array([int(data.pose.position.y/0.08),int(data.pose.position.x/0.08)])
        path = planner.astarplanner(start, navgoal, map)
        
        pathpub.publish(path)


    rospy.init_node('astar',anonymous=True)
    rate = rospy.Rate(20.0)
    mappub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
    rospy.Subscriber('/move_base_simple/goal',PoseStamped,goalcb)
    rospy.Subscriber('/initialpose',PoseWithCovarianceStamped,startcb)
    pathpub = rospy.Publisher('/path', Path, queue_size=1)
    mapmsg = OccupancyGrid()
    
    robotradius = 0.3
    resolution = 0.08
    planner = Astar(resolution,robotradius)

    mapmsg.header.frame_id = 'map'
    mapmsg.info.height = 120
    mapmsg.info.width = 120
    mapmsg.info.resolution = 0.08
    mapmsg.info.origin.position.x = 0
    mapmsg.info.origin.position.y = 0
    mapmsg.data = mapdata
    
    while not rospy.is_shutdown():
        mappub.publish(mapmsg)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()	
    except rospy.ROSInterruptException:
        pass
    

