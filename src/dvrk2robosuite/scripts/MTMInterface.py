#!/usr/bin/env python
import rospy
import numpy as np
import copy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped as transform
import robosuite.utils.transform_utils as T
import tf
import tf2_ros


class MTMInterface:
    def __init__(self):
        # ======================= MTMR ==========================================
        # Transformation matrix of the operator's reference frame measured from Base frame of MTM device
        self.RotationB2R = np.identity(3)
        
        # master Right EE translation
        self.MTMRCurrentTranslation = np.zeros(3)
        self.MTMRPreviousTranslation = np.zeros(3)
        self.MTMRDeltaTranslation = np.zeros(3)
        
        # master Right EE orientation
        self.MTMRCurrentEEOrien_quat = np.array([0.,0.,0.,1.])
        self.MTMRPreviousEEOrien_quat = np.array([0.,0.,0.,1.])
        self.MTMRDeltaEEOrien_qaut = np.array([0.,0.,0.,1.])
        
        # master Right EE rotation
        self.MTMRCurrentEEOrien_rotm = np.identity(3)
        
        # master Right wrist joint pos
        self.MTMRCurrentWristJointPos = np.zeros(3)
        self.MTMRPreviousWristJointPos = np.zeros(3)
        self.MTMRDeltaWristJointPos = np.zeros(3)
        
        # master Right gripper pos
        self.MTMRGripperPos = 0.0
        
        # master Right EE Initial translation
        self.MTMRInitialTranslation = np.zeros(3)
        
        self.master_ee_transform_subscriber = rospy.Subscriber("/MTMR/measured_cp", transform, self.MTMRTransformCallback)
        self.master_gripper_js_subscriber = rospy.Subscriber("/MTMR/gripper/measured_js", JointState, self.MTMRGrippercallback)
        
        # ======================= SimulatorR ==========================================
        self.SimulatorRInitialT = transform()
        self.SimulatorRInitialTranslation = np.zeros(3)
        self.SimulatorRInitialQuat = np.array([0.,0.,0.,1.])
        self.SimulatorRInitialRotm = np.identity(3)
        
        # subscriber of initial values
        self.init_subscriber = True
        self.R_trans_init_subscriber = rospy.Subscriber('/Tele/SimulatorR/InitialTranslation', transform, self.SimulatorRInitialTCallback)
        
        self.R_Dtrans_publisher = rospy.Publisher('/Tele/MTMR/DeltaTranslation', transform, queue_size=1)
        self.R_transform_publisher = rospy.Publisher('/Tele/MTMR/TransformBase2Refence', transform, queue_size=1)
        self.R_gripper_state_publisher = rospy.Publisher('/Tele/MTMR/GripperJointState', JointState, queue_size=1)
        self.R_wrist_roll_publisher = rospy.Publisher('/Tele/MTMR/DeltaWristJoint', JointState, queue_size=1)
        
        # ======================= MTML ==========================================
        # master Left gripper pos
        self.MTMLGripperPos = 0.0
        self.master_left_gripper_js_subscriber = rospy.Subscriber("/MTML/gripper/measured_js", JointState, self.MTMLGrippercallback)
        self.L_gripper_state_publisher = rospy.Publisher('/Tele/MTML/GripperJointState', JointState, queue_size=1)
        
        self.rate = rospy.Rate(20)
        
    def MTMRTransformCallback(self, msg):
        # =============== msg.transform.translation.x y z
        self.MTMRCurrentTranslation[0] = msg.transform.translation.x
        self.MTMRCurrentTranslation[1] = msg.transform.translation.y
        self.MTMRCurrentTranslation[2] = msg.transform.translation.z

        # =============== msg.transform.rotation.x y z w
        self.MTMRCurrentEEOrien_quat[0] = msg.transform.rotation.x
        self.MTMRCurrentEEOrien_quat[1] = msg.transform.rotation.y
        self.MTMRCurrentEEOrien_quat[2] = msg.transform.rotation.z
        self.MTMRCurrentEEOrien_quat[3] = msg.transform.rotation.w
        
        # =============== Calculate the translation of MTMR EE
        self.MTMRDeltaTranslation = self.MTMRCurrentTranslation - self.MTMRInitialTranslation
        self.MTMRDeltaTranslation = self.RotationB2R @ self.MTMRDeltaTranslation
        
        # =============== Calculate the rotation of MTMR EE
        self.MTMRCurrentEEOrien_rotm = T.quat2mat(self.MTMRCurrentEEOrien_quat)
        self.MTMRCurrentEEOrien_rotm = self.RotationB2R @ self.MTMRCurrentEEOrien_rotm
        
        # publish the delta translation as well as the delta quat
        reference_MTMR_pose = T.make_pose(self.MTMRDeltaTranslation, self.MTMRCurrentEEOrien_rotm)
        reference_MTMR_tarns, reference_MTMR_quat = T.mat2pose(reference_MTMR_pose) # need check for losing precision upon multiple conversions
        referenceT = transform()
        referenceT.transform.translation.x = reference_MTMR_tarns[0]
        referenceT.transform.translation.y = reference_MTMR_tarns[1]
        referenceT.transform.translation.z = reference_MTMR_tarns[2]
        referenceT.transform.rotation.x = reference_MTMR_quat[0]
        referenceT.transform.rotation.y = reference_MTMR_quat[1]
        referenceT.transform.rotation.z = reference_MTMR_quat[2]
        referenceT.transform.rotation.w = reference_MTMR_quat[3]
        self.R_transform_publisher.publish(referenceT)
    
    def MTMRGrippercallback(self, msg):
        self.MTMRGripperPos = msg.position[0]
        
        # publish
        right_gripper_joint_state = JointState()
        right_gripper_joint_state.position = [self.MTMRGripperPos]
        self.R_gripper_state_publisher.publish(right_gripper_joint_state)
        
    def MTMLGrippercallback(self, msg):
        self.MTMLGripperPos = msg.position[0]
        # publish
        left_gripper_joint_state = JointState()
        left_gripper_joint_state.position = [self.MTMLGripperPos]
        self.L_gripper_state_publisher.publish(left_gripper_joint_state)
        
    def SimulatorRInitialTCallback(self, msg):
        if self.init_subscriber:
            self.SimulatorRInitialT = msg
            self.SimulatorRInitialTranslation[0] = self.SimulatorRInitialT.transform.translation.x
            self.SimulatorRInitialTranslation[1] = self.SimulatorRInitialT.transform.translation.y
            self.SimulatorRInitialTranslation[2] = self.SimulatorRInitialT.transform.translation.z
            
            self.MTMRInitialTranslation[0] = self.MTMRCurrentTranslation[0]
            self.MTMRInitialTranslation[1] = self.MTMRCurrentTranslation[1]
            self.MTMRInitialTranslation[2] = self.MTMRCurrentTranslation[2]
            self.init_subscriber = False
        
if __name__ == "__main__":
    rospy.init_node('Teleoperation_Interface', anonymous=False)
    mtm_interface = MTMInterface()
    mtm_interface.rate.sleep()
    rospy.spin()