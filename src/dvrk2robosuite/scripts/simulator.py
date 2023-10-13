#!/usr/bin/env python
import rospy
import os, sys

import numpy as np
import robosuite as suite
from robosuite import load_controller_config
import xml.etree.ElementTree as ET
from robosuite.environments.manipulation import pick_place, lift

from geometry_msgs.msg import TransformStamped as transform
from sensor_msgs.msg import JointState

from robosuite.wrappers.visualization_wrapper import VisualizationWrapper
from robosuite.utils.transform_utils import rotation_matrix
import robosuite.utils.transform_utils as T
from robosuite.utils.camera_utils import CameraMover
from robosuite.wrappers import DataCollectionWrapper

# simulator settings
controller_setting_fpath = os.path.join( os.path.dirname( os.path.dirname( os.path.abspath(__file__) ) ), 'config/tele_osc.json')
controller_config = load_controller_config(custom_fpath=controller_setting_fpath)

config = {
    "env_name": "PickPlaceCan", #Lift, Stack, NutAssembly, NutAssemblySingle, NutAssemblySquare, NutAssemblyRound
    # PickPlace, PickPlaceSingle, PickPlaceMilk, PickPlaceBread, PickPlaceCereal, PickPlaceCan, Door
    "robots": "Panda",
    "controller_configs": controller_config,
}

class Simulator:
    
    def __init__(self, config):
        
        # ======================= env ==========================================

        CAMERA_NAME = "agentview" # agentview

        # initialize the simulator
        env = suite.make(**config,
                            has_renderer=True,
                            has_offscreen_renderer=False,
                            render_camera=CAMERA_NAME,
                            ignore_done=True,
                            use_camera_obs=False,
                            # reward_shaping=True,
                            control_freq=20,
                            # hard_reset=False,
                        )
        env.reset()
        
        # Wrap this environment in a visualization wrapper
        self._env = VisualizationWrapper(env, indicator_configs=None)
        
        # wrap the environment with data collection wrapper
        data_directory = 'home/tyx/dvrk2robotsuite_ws/data_collection'
        env = DataCollectionWrapper(env, data_directory)
        
        self._env.set_visualization_setting(setting="grippers", visible=True)
        
        # ======================= Camera ==========================================

        camera_mover = CameraMover(env=env, camera=CAMERA_NAME)
        camera_id = env.sim.model.camera_name2id(CAMERA_NAME)
        env.viewer.set_camera(camera_id=camera_id)
        
        self.cameraTranslation, self.cameraQuat = camera_mover.get_camera_pose()
        # self.cameraRotm = T.quat2mat(self.cameraQuat)
        self.cameraRotm = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]]) # need think again
        self.cameraTransformWorld2Camera = T.make_pose(self.cameraTranslation, self.cameraRotm)
        self.cameraTransformCamera2World = T.pose_inv(self.cameraTransformWorld2Camera)
        
        # cannot move camera
        
        # ======================= Lambda ==========================================
        # postion sensitivity
        self.lx = 4
        self.ly = 4
        self.lz = 2
        
        # ======================= Action ==========================================
        self.SimulatorRightAction = np.zeros(7)         # translation, rotation
        self.SimulatorRightGripperAction = 0.0          # gripper, axis-angle rotation
        
        self.SimulatorRightEETransform2World = np.identity(4)
        self.SimulatorRightEETranslation2World = np.zeros(3)
        self.SimulatorRightEEQuat2World = np.array([0.,0.,0.,1.])
        self.SimulatorRightEEAxisAngle2World = np.zeros(3)
        
        # ======================= SimulatorR ==========================================
        self.SimulatorRightEEInitialTransform = transform()   # Simulator Right Arm 's EE Initial Trasform
        self.SimulatorRightEEInitialTranslation = np.zeros(3)
        
        self.SimulatorRightEETranslation2Camera = np.zeros(3)
        self.SimulatorRightEERotm2Camera = np.identity(3)
        
        #  publisher of initial values
        self.init_publisher = True
        self.R_trans_init_publisher = rospy.Publisher('/Tele/SimulatorR/InitialTranslation', transform, queue_size=1)
        
        # ======================= Operator ==========================================
        self.ReferenceCurrentEETranslation = np.zeros(3)
        self.ReferenceCurrentEEOrien_quat = np.array([0.,0.,0.,1.])
        self.ReferenceCurrentEERotm = np.identity(3)

        self.operator_ee_transform_subscriber = rospy.Subscriber('/Tele/MTMR/TransformBase2Refence', transform, self.Reference2SimulatorCamera_TransformCallback)
        self.master_gripper_js_subscriber = rospy.Subscriber('/Tele/MTMR/GripperJointState', JointState, self.MTMR2Simulator_GripperCallback)
        
        # lef mtm gripper state (for clutch)
        self.SimulatorLeftGripperAction = 0.0
        self.master_left_gripper_js_subscriber = rospy.Subscriber('/Tele/MTML/GripperJointState', JointState, self.MTML2Simulator_GripperCallback)
        
        self.rate = rospy.Rate(20)
        
    def Reference2SimulatorCamera_TransformCallback(self, msg):
        # =============== translation align with the operator
        self.ReferenceCurrentEETranslation[0] = msg.transform.translation.x * self.lx
        self.ReferenceCurrentEETranslation[1] = msg.transform.translation.y * self.ly
        self.ReferenceCurrentEETranslation[2] = msg.transform.translation.z * self.lz

        # =============== rotation align with the operator
        self.ReferenceCurrentEEOrien_quat[0] = msg.transform.rotation.x
        self.ReferenceCurrentEEOrien_quat[1] = msg.transform.rotation.y
        self.ReferenceCurrentEEOrien_quat[2] = msg.transform.rotation.z
        self.ReferenceCurrentEEOrien_quat[3] = msg.transform.rotation.w

        self.SimulatorRightEETranslation2Camera = self.cameraTranslation + self.cameraRotm @ self.SimulatorRightEEInitialTranslation + self.ReferenceCurrentEETranslation
        
        self.ReferenceCurrentEERotm = T.quat2mat(self.ReferenceCurrentEEOrien_quat)
        self.SimulatorRightEERotm2Camera = self.ReferenceCurrentEERotm
        self.SimulatorRightEETransform2Camera = T.make_pose(self.SimulatorRightEETranslation2Camera, self.SimulatorRightEERotm2Camera)
        
        # Trasfrom from camera view to world view
        self.SimulatorRightEETransform2World = self.cameraTransformCamera2World @ self.SimulatorRightEETransform2Camera
        
    def MTMR2Simulator_GripperCallback(self, msg):
        self.SimulatorRightGripperAction = msg.position[0]
        
        
    def MTML2Simulator_GripperCallback(self, msg):
        self.SimulatorLeftGripperAction = msg.position[0]
        
        
    def construct_action(self):
        
        self.SimulatorRightEETranslation2World, self.SimulatorRightEEQuat2World = T.mat2pose(self.SimulatorRightEETransform2World)
        
        # translation
        self.SimulatorRightAction[0] = self.SimulatorRightEETranslation2World[0]
        self.SimulatorRightAction[1] = self.SimulatorRightEETranslation2World[1]
        self.SimulatorRightAction[2] = self.SimulatorRightEETranslation2World[2]

        # orientation
        self.SimulatorRightEEAxisAngle2World = T.quat2axisangle(self.SimulatorRightEEQuat2World)
        self.SimulatorRightAction[3] = self.SimulatorRightEEAxisAngle2World[0]
        self.SimulatorRightAction[4] = self.SimulatorRightEEAxisAngle2World[1]
        self.SimulatorRightAction[5] = self.SimulatorRightEEAxisAngle2World[2]
        
        # gripper
        self.SimulatorRightAction[6] = -self.SimulatorRightGripperAction
        
        # imitate cltuch functionality
        if self.SimulatorLeftGripperAction < 0.0:
            clutch_signal = 0.0
        else:
            clutch_signal = 1.0
        
        return self.SimulatorRightAction * clutch_signal
        
    def run(self):
        obs = self._env.reset()
        
        while not rospy.is_shutdown():
            
            # ================ get simulator state =======================
            if self.init_publisher:
                self.SimulatorRightEEInitialTransform.transform.translation.x = obs['robot0_eef_pos'][0]
                self.SimulatorRightEEInitialTransform.transform.translation.y = obs['robot0_eef_pos'][1]
                self.SimulatorRightEEInitialTransform.transform.translation.z = obs['robot0_eef_pos'][2]
                self.SimulatorRightEEInitialTransform.transform.rotation.x = obs['robot0_eef_quat'][0]
                self.SimulatorRightEEInitialTransform.transform.rotation.y = obs['robot0_eef_quat'][1]
                self.SimulatorRightEEInitialTransform.transform.rotation.z = obs['robot0_eef_quat'][2]
                self.SimulatorRightEEInitialTransform.transform.rotation.w = obs['robot0_eef_quat'][3]
                
                self.SimulatorRightEEInitialTranslation[0] = obs['robot0_eef_pos'][0]
                self.SimulatorRightEEInitialTranslation[1] = obs['robot0_eef_pos'][1]
                self.SimulatorRightEEInitialTranslation[2] = obs['robot0_eef_pos'][2]
                self.init_publisher = False
            
            # ================== action construction =====================
            action = self.construct_action()
            obs, _, _, _ = self._env.step(action)
            
            # =========== pub some simulator data here if necessary =======
            self.R_trans_init_publisher.publish(self.SimulatorRightEEInitialTransform)
            # =============================================================
            # do visualization
            self._env.render()
        
        
if __name__ == "__main__":
    
    rospy.init_node('Simulator', anonymous=False)
    
    simulator = Simulator(config)
    simulator.run()
    simulator.rate.sleep() # maybe problematic and need change to service instead
    rospy.spin()