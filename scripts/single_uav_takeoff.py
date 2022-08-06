#!/usr/bin/env python3
import rospy 
import mavros

from geometry_msgs.msg import PoseStamped 
from mavros_msgs.msg import * 
from mavros_msgs.srv import * 
from swarm.srv import PoseCommand, PoseCommandResponse



pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 5

global state 

def position_command(req):
    pose.pose.position.x = req.x
    pose.pose.position.y = req.y
    pose.pose.position.z = req.z
    return PoseCommandResponse(True)


def state_callback(data):
    
    global state
    state = data
    return

def state_listener():
    rospy.Subscriber("/mavros/state", State, state_callback)
    rate = rospy.Rate(100)
    rate.sleep()



if __name__ == '__main__':
    
    rospy.init_node("takeoff", anonymous=True)
    rate = rospy.Rate(10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode") 
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    command_service = rospy.Service("PoseCommand", PoseCommand, position_command)

    pose_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    pose_pub.publish(pose)

    set_mode = SetMode()
    set_mode._response_class.custom_mode = "OFFBOARD"
    set_mode._response_class.base_mode = 0

    state_listener()

    for i in range(100):
        pose_pub.publish(pose)
        state_listener()

    print(state.connected) 

    last_request = rospy.Time.now()

    try:

        while not rospy.is_shutdown():
            if state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
                if set_mode_client(base_mode=0, custom_mode="OFFBOARD") and state.mode == "OFFBOARD":
                    print("OFFBOARD")
                    last_request = rospy.Time.now()
            else:
                if not state.armed and (rospy.Time.now()-last_request > rospy.Duration(5.0)):
                    arming_client(True)
                    last_request = rospy.Time.now()

            pose_pub.publish(pose)
            
            rate.sleep()

    except rospy.exceptions.ROSInterruptException:
        print("\nshutdown")