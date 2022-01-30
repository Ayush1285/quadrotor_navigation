#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseStamped, Pose
from mavros_msgs.msg import State,PositionTarget
from mavros_msgs.srv import CommandBool,SetMode
from tf.transformations import euler_from_quaternion

class flightModes:
    def __init__(self):
        pass

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arm call failed")

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("Service disarm call failed")

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed")

class Controller:
    def __init__(self):
        self.state = State()
        self.sp = PositionTarget()

        self.sp.type_mask = int('001111111000', 2)
        self.sp.coordinate_frame = 1

        self.desired_height = 2.0
        
        self.sp.position.z = self.desired_height

        self.local_pos = Point(0.0, 0.0, 1.0)
        self.curr_yaw = 0
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0
        self.sp.yaw = 0.0
        self.sp.yaw_rate = 0.0

	
    def posCb(self, msg):
        self.local_pos.x = msg.pose.pose.position.x
        self.local_pos.y = msg.pose.pose.position.y
        self.local_pos.z = msg.pose.pose.position.z
        roll,pitch,yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        self.curr_yaw = yaw

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    def desSp(self, msg):
        self.sp.position.x = msg.position.x + 22
        self.sp.position.y = msg.position.y + 49
        self.sp.position.z = msg.position.z
        roll,pitch,yaw = euler_from_quaternion([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])
        self.sp.yaw = yaw


def main():
    rospy.init_node('px4_offboard', anonymous=True)

    modes = flightModes()
    control = Controller()

    rate = rospy.Rate(15.0)
    rospy.Subscriber('mavros/state', State, control.stateCb)
    rospy.Subscriber('desired/trajectory',Pose, control.desSp,queue_size=20)
    rospy.Subscriber('/ground_truth/state', Odometry, control.posCb)    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)


    while not control.state.armed:
        modes.setArm()
        rate.sleep()
    k=0
    while k<10:
        sp_pub.publish(control.sp)
        rate.sleep()
        k = k + 1

    modes.setOffboardMode()
    while not rospy.is_shutdown():
        sp_pub.publish(control.sp)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()	
    except rospy.ROSInterruptException:
        pass

  


