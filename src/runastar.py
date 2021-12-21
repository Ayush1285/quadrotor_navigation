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


map = generatemap()
mapdata = map.flatten()
mapdata = mapdata.astype('int8')


def main():

    def goalcb(data):
        navgoal = np.array([int(data.pose.position.y/0.08),int(data.pose.position.x/0.08)])
        path = planner.astarplanner(start, navgoal, map)

        pathmessage = generatepath(path)
        pathpub.publish(pathmessage)

    def startcb(data):
        global start
        start = np.array([int(data.pose.pose.position.y/0.08),int(data.pose.pose.position.x/0.08)])

    def generatepath(path):
        pathmsg = Path()

        path = [int(path) for path in path]
        ypos,xpos = np.unravel_index(path,map.shape)
        xpos = xpos*resolution
        ypos = ypos*resolution
        r_x, r_y, r_yaw, r_k, travel = calc_2d_spline_interpolation(xpos, ypos, len(xpos))

        for i in range(len(xpos)):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = r_x[i]
            pose.pose.position.y = r_y[i]
            q = quaternion_from_euler(0, 0, r_yaw[i])
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            pathmsg.poses.append(pose)

        pathmsg.header.frame_id = 'map'
        return pathmsg

    rospy.init_node('astar',anonymous=True)
    rate = rospy.Rate(20.0)
    mappub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
    rospy.Subscriber('/move_base_simple/goal',PoseStamped,goalcb)
    rospy.Subscriber('/initialpose',PoseWithCovarianceStamped,startcb)
    pathpub = rospy.Publisher('/path', Path, queue_size=1)
    mapmsg = OccupancyGrid()
    

    robotradius = 0.2
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
    

