#!/usr/bin/env python  
import roslib
import rospy
import tf

class SimulatorTFBroadcaster:
    def __init__(self):
        


if __name__ == '__main__':
    rospy.init_node('Simulator_Tf_Broadcaster')
    tf_caster = SimulatorTFBroadcaster()
    rospy.spin()