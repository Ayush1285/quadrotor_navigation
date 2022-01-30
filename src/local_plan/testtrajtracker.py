#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped,Pose
from nav_msgs.msg import Path


def main():

    def poseupdate(msg):
        print(msg.position)
        points = PoseStamped()
        points.header.frame_id = 'map'
        currstate = msg
        points.pose = msg
        coveredpath.poses.append(points)
        posepub.publish(currstate)
        pathpub.publish(coveredpath)
        rate.sleep()


    rospy.init_node("tracker",anonymous=True)
    rate = rospy.Rate(20)
    currstate = Pose()
    coveredpath = Path()
    coveredpath.header.frame_id = 'map'
    posepub = rospy.Publisher('pose',Pose,queue_size=1)
    pathpub = rospy.Publisher('coveredpath',Path,queue_size=1)
    data = rospy.wait_for_message('desired/trajectory',Pose)
    poseupdate(data)

    rospy.Subscriber('desired/trajectory',Pose,poseupdate)
    

    while not rospy.is_shutdown():
        continue





if __name__ == '__main__':
    try:
        main()	
    except rospy.ROSInterruptException:
        pass