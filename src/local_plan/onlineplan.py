#!/usr/bin/env python3
from math import hypot
from time import time
from nav_msgs.msg import Path, OccupancyGrid,Odometry
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,Pose
from onlineflight import Flight
#from scipy import interpolate
import rospy




        
def main():

    robotradius = 0.8
    localmapsize = 10
    flight = Flight(robotradius,localmapsize)
    rospy.init_node('autonav',anonymous=True)
    rate = rospy.Rate(4)
    despath = Path()
    mapdata = rospy.wait_for_message('/map',OccupancyGrid)
    flight.getmap(mapdata)
    posedata = rospy.wait_for_message('initialpose',PoseWithCovarianceStamped)
    flight.updateinitialpose(posedata)
    #posedata = rospy.wait_for_message('/ground_truth/state',Odometry)
    #flight.updateinitialpose(posedata)
    goaldata = rospy.wait_for_message('/move_base_simple/goal',PoseStamped)
    flight.goalcb(goaldata)
    #rospy.Subscriber('/ground_truth/state',Odometry,flight.updateinitialpose)
    
    pathpub = rospy.Publisher('/path', Path, queue_size=5)
    
    despath = flight.runplanner()
    pathpub.publish(despath)
    pastpath = despath.poses
    #rospy.Subscriber('/map',OccupancyGrid,getmap)
    rospy.Subscriber('pose',Pose,flight.updatepose)
    

    while not rospy.is_shutdown():
        data = rospy.wait_for_message('/map',OccupancyGrid)
        flight.getmap(data)
        if hypot(flight.globalgoal.pose.position.x-flight.pose.position.x, flight.globalgoal.pose.position.y-flight.pose.position.y) < 0.2:
            goaldata = rospy.wait_for_message('/move_base_simple/goal',PoseStamped)
            flight.goalcb(goaldata)

        else:
            initialt = time()
            latestpath = flight.runplanner()
            
            if time()-initialt > 0.2:
                latestpath.poses = pastpath[4:-1]
                pathpub.publish(latestpath)
                pastpath = latestpath.poses
            else:
                latestpath.poses[0:3] = []
                pathpub.publish(latestpath)
                pastpath = latestpath.poses


            #print(len(latestpath.poses))
            

        rate.sleep()

if __name__ == "__main__":
    try:
        main()	
    except rospy.ROSInterruptException:
        pass