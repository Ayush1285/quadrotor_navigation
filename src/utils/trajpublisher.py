#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from trajgen import trajgenlemniscate,trajcircle
from mavros_msgs.msg import State



def main():

    def pubpath(data):
        path = data.poses
        for k in range(min(len(path),4)):
            destaj.position.x = path[k].pose.position.x
            destaj.position.y = path[k].pose.position.y
            
            destaj.position.z = 2.0
            #print(destaj.position)
            destaj.orientation = path[k].pose.orientation

            sp_pub.publish(destaj)
            rate.sleep()



    rospy.init_node('trajpub', anonymous=True)
    rate = rospy.Rate(15.0)

    destaj = Pose()
    
    sp_pub = rospy.Publisher('desired/trajectory', Pose, queue_size=1)
    #pose_pub = rospy.Publisher('pose', Pose, queue_size=1)
    rospy.Subscriber('/path',Path,pubpath)

    while not rospy.is_shutdown():
        #msg = rospy.wait_for_message('/path',Path)
        #pubpath(msg)
        continue
    

if __name__ == '__main__':
    try:
        main()	
    except rospy.ROSInterruptException:
        pass


