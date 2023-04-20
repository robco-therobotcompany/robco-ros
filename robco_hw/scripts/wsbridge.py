#!/usr/bin/env python
from websocket import create_connection
import json
import rospy
from robco_robot_interface.msg import RobcoJointAngles

def bridge():
    pub = rospy.Publisher('/websocket', RobcoJointAngles, queue_size=10)
    rospy.init_node('wsbridge', anonymous=True)
    rate = rospy.Rate(10)

    ws = create_connection("ws://sn23-1000360.fritz.box/api/v2.0/robot")

    while not rospy.is_shutdown():
        result = ws.recv()

        inMsg = json.loads(result)
        if inMsg['type'] != 'jointAngles':
            continue

        msg = RobcoJointAngles()
        msg.data = inMsg['data']

        #rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()
    
if __name__ == '__main__':
    try:
        bridge()
    except rospy.ROSInterruptException:
        pass
