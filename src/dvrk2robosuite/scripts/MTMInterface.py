#!/usr/bin/env python
import rospy
import numpy as np
import copy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped as transform
import robosuite.utils.transform_utils as T


class MTMInterface:
    def __init__(self):
        # ======================= MTMR ==========================================
        # master Right EE translation
        self.MTMRCurrentTranslation = np.zeros(3)
        self.MTMRPreviousTranslation = np.zeros(3)
        self.MTMRDeltaTranslation = np.zeros(3)
        
        # master Right EE orientation
        self.MTMRCurrentEEOrien_quat = np.array([0.,0.,0.,1.])
        self.MTMRPreviousEEOrien_quat = np.array([0.,0.,0.,1.])
        self.MTMRDeltaEEOrien_qaut = np.array([0.,0.,0.,1.])
        
        # master Right wrist joint pos
        self.MTMRCurrentWristJointPos = np.zeros(3)
        self.MTMRPreviousWristJointPos = np.zeros(3)
        self.MTMRDeltaWristJointPos = np.zeros(3)
        
        # master Right gripper pos
        self.MTMRGripperPos = 0.0
        
        self.master_ee_pos_subscriber = rospy.Subscriber("/MTMR/measured_cp", transform, self.MTMRDeltaTransCallback)
        self.master_gripper_js_subscriber = rospy.Subscriber("/MTMR/gripper/measured_js", JointState, self.MTMRGrippercallback)
        self.master_js_subscriber = rospy.Subscriber("/MTMR/measured_js", JointState, self.MTMRDeltaWristCallback)
        
        self.R_trans_publisher = rospy.Publisher('/Tele/MTMR/DeltaTranslation', transform, queue_size=1)
        self.R_gripper_state_publisher = rospy.Publisher('/Tele/MTMR/GripperJointState', JointState, queue_size=1)
        self.R_wrist_roll_publisher = rospy.Publisher('/Tele/MTMR/DeltaWristJoint', JointState, queue_size=1)
        
        # ======================= MTML ==========================================
        # master Left gripper pos
        self.MTMLGripperPos = 0.0
        self.master_left_gripper_js_subscriber = rospy.Subscriber("/MTML/gripper/measured_js", JointState, self.MTMLGrippercallback)
        self.L_gripper_state_publisher = rospy.Publisher('/Tele/MTML/GripperJointState', JointState, queue_size=1)
        
        self.rate = rospy.Rate(5)
        
    def MTMRDeltaTransCallback(self, msg):
        # =============== msg.transform.translation.x y z
        self.MTMRCurrentTranslation[0] = msg.transform.translation.x
        self.MTMRCurrentTranslation[1] = msg.transform.translation.y
        self.MTMRCurrentTranslation[2] = msg.transform.translation.z

        # =============== msg.transform.rotation.x y z w
        self.MTMRCurrentEEOrien_quat[0] = msg. transform.rotation.x
        self.MTMRCurrentEEOrien_quat[1] = msg. transform.rotation.y
        self.MTMRCurrentEEOrien_quat[2] = msg. transform.rotation.z
        self.MTMRCurrentEEOrien_quat[3] = msg. transform.rotation.w
        
        # =============== Calculate the delta translation of MTMR EE
        self.MTMRDeltaTranslation = self.MTMRCurrentTranslation - self.MTMRPreviousTranslation
        self.MTMRPreviousTranslation = copy.deepcopy( self.MTMRCurrentTranslation )
        
        # =============== Calculate the delta orientation in quat of MTMR EE; delat_quat = q1 * inv(q2)
        self.MTMRDeltaEEOrien_qaut = T.quat_multiply(self.MTMRCurrentEEOrien_quat, T.quat_inverse(self.MTMRPreviousEEOrien_quat))
        self.MTMRPreviousEEOrien_quat = copy.deepcopy(self.MTMRCurrentEEOrien_quat)
        
        # publish the delta translation as well as the delta quat
        tDiff = transform()
        tDiff.transform.translation.x = self.MTMRDeltaTranslation[0]
        tDiff.transform.translation.y = self.MTMRDeltaTranslation[1]
        tDiff.transform.translation.z = self.MTMRDeltaTranslation[2]
        tDiff.transform.rotation.x = self.MTMRDeltaEEOrien_qaut[0]
        tDiff.transform.rotation.y = self.MTMRDeltaEEOrien_qaut[1]
        tDiff.transform.rotation.z = self.MTMRDeltaEEOrien_qaut[2]
        tDiff.transform.rotation.w = self.MTMRDeltaEEOrien_qaut[3]
        self.R_trans_publisher.publish(tDiff)
    
    
    def MTMRGrippercallback(self, msg):
        self.MTMRGripperPos = msg.position[0]
        
        # publish
        right_gripper_joint_state = JointState()
        right_gripper_joint_state.position = [self.MTMRGripperPos]
        self.R_gripper_state_publisher.publish(right_gripper_joint_state)
    
    
    def MTMRDeltaWristCallback(self, msg):
        self.MTMRCurrentWristJointPos[0] = msg.position[4]
        self.MTMRCurrentWristJointPos[1] = msg.position[5]
        self.MTMRCurrentWristJointPos[2] = msg.position[6]
        
        self.MTMRDeltaWristJointPos = self.MTMRCurrentWristJointPos - self.MTMRPreviousWristJointPos
        
        self.MTMRPreviousWristJointPos = copy.deepcopy( self.MTMRCurrentWristJointPos ) 
        
        # publish
        wristJointDiff = JointState()
        wristJointDiff.position = [self.MTMRDeltaWristJointPos[0], self.MTMRDeltaWristJointPos[1], self.MTMRDeltaWristJointPos[2]]
        self.R_wrist_roll_publisher.publish(wristJointDiff)
        
    def MTMLGrippercallback(self, msg):
        self.MTMLGripperPos = msg.position[0]
        # publish
        left_gripper_joint_state = JointState()
        left_gripper_joint_state.position = [self.MTMLGripperPos]
        self.L_gripper_state_publisher.publish(left_gripper_joint_state)
        
        
if __name__ == "__main__":
    rospy.init_node('Teleoperation_Interface', anonymous=False)
    mtm_interface = MTMInterface()
    mtm_interface.rate.sleep()
    rospy.spin()