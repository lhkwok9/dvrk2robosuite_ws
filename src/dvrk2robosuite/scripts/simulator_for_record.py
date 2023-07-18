#!/usr/bin/env python
"""Simulator For Data Recording"""
import rospy

import os, json

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

import time

# simulator settings
controller_setting_fpath = os.path.join( os.path.dirname( os.path.dirname( os.path.abspath(__file__) ) ), 'config/tele_osc.json')
controller_config = load_controller_config(custom_fpath=controller_setting_fpath)

config = {
    "env_name": "PickPlaceCan", #NutAssemblySingle PickPlace
    "robots": "Panda",
    "controller_configs": controller_config,
}

# ================================== add your info (operator ID) before tele-operation =============================================
tele_config = {
    "operator": "JK", # Xiangyu Chu
    "proficiency_level": "Non-expert", # [non-expert, mid, expert]
}

# data collection dir
data_directory = os.path.join( os.path.dirname( os.path.dirname( os.path.abspath(__file__) ) ), 'data_collection', config['env_name'])

class Simulator:
    
    def __init__(self, config):

        # initialize the simulator
        env = suite.make(
            **config,
            has_renderer=True,
            has_offscreen_renderer=True,
            render_camera="agentview",
            ignore_done=True,
            use_camera_obs=False,
            reward_shaping=True,
            control_freq=20,
            hard_reset=False,
            )
        
        # reset the camera position
        camera_mover = CameraMover(env)
        new_pos = camera_mover.get_camera_pose()[0]
        new_pos[2] += 0.5
        camera_mover.set_camera_pose(pos=new_pos)
        
        # Wrap this environment in a visualization wrapper
        self._env = VisualizationWrapper(env, indicator_configs=None)
        self._env.set_visualization_setting(setting="grippers", visible=True)
        
        # wrap the environment with data collection wrapper
        self._env  = DataCollectionWrapper(self._env , data_directory)
        
        # some hyperparameter
        # postion sensitivity
        self.lx = 30e2
        self.ly = 20e2
        self.lz = 10e2
        
        # full action command
        self.SimulatorRightAction = np.zeros(7)
        
        self.SimulatorRightTransAction =np.zeros(3)     # translation
        self.SimulatorRightWristAction = np.zeros(3)    # Orientation
        self.SimulatorRightGripperAction = 0.0          # gripper 
        
        self.ee_orien_quat = np.array([0.,0.,0.,1.]) # Simulator Right Arm 's EE Orientation in quat
        self.MTMRDeltaEEOrien_qaut = np.array([0.,0.,0.,1.]) # MTMR EE's Delta Orientation in quat
        
        self.master_ee_delta_trans_subscriber = rospy.Subscriber('/Tele/MTMR/DeltaTranslation', transform, self.MTMR2Simulator_DeltaTransCallback)
        self.master_gripper_js_subscriber = rospy.Subscriber('/Tele/MTMR/GripperJointState', JointState, self.MTMR2Simulator_GripperCallback)
        self.master_ee_delta_wrist_subscriber = rospy.Subscriber('/Tele/MTMR/DeltaWristJoint', JointState, self.MTMR2Simulator_DeltaWristCallback)
        
        # lef mtm gripper state (for clutch)
        self.SimulatorLeftGripperAction = 0.0
        self.master_left_gripper_js_subscriber = rospy.Subscriber('/Tele/MTML/GripperJointState', JointState, self.MTML2Simulator_GripperCallback)
        
    def MTMR2Simulator_DeltaTransCallback(self, msg):
        # translation: align with the operator
        self.SimulatorRightTransAction[0] = -msg.transform.translation.y * self.ly
        self.SimulatorRightTransAction[1] = msg.transform.translation.x * self.lx
        self.SimulatorRightTransAction[2] = msg.transform.translation.z * self.lz
        
        # delta orientation of MTMR EE
        self.MTMRDeltaEEOrien_qaut[0] = msg.transform.rotation.x
        self.MTMRDeltaEEOrien_qaut[1] = msg.transform.rotation.y
        self.MTMRDeltaEEOrien_qaut[2] = msg.transform.rotation.z
        self.MTMRDeltaEEOrien_qaut[3] = msg.transform.rotation.w
        # json.dumps(config)
    
    
    def MTMR2Simulator_DeltaWristCallback(self, msg):
        # convert RPY to an absolute orientation
        # double check the order
        pitch = msg.position[0]
        roll = msg.position[1]
        yaw = msg.position[2]        
        raw_drotation = np.array([roll, pitch, yaw])
        drotation = raw_drotation[[0, 1, 2]]
        drotation = T.mat2quat( T.euler2mat(drotation)  ) # master ee quat
        
        # ===================================== TODO: Orientation Mapping ==============================================
        # using quat
        # drotation = self.MTMRDeltaEEOrien_qaut
        
        drotation =  T.quat2mat(self.ee_orien_quat) @ T.quat2axisangle(drotation)
        # drotation *= 5.
        
        # orientation using quat
        # self.SimulatorRightWristAction[0] = 0*5*drotation[2] # roll
        # self.SimulatorRightWristAction[1] = -0*10*drotation[0] # pitch
        # self.SimulatorRightWristAction[2] =  30*drotation[1] # yaw
        
        # orientation only using pure yaw
        self.SimulatorRightWristAction[2] =  0*30*drotation[2] # yaw
        
        # orientation using all angles 
        # self.SimulatorRightWristAction[0] = 5*drotation[0] # roll
        # self.SimulatorRightWristAction[1] = 10*drotation[1] # pitch
        # self.SimulatorRightWristAction[2] =  30*drotation[2] # yaw
        # ============================================================================================
        
        
    def MTMR2Simulator_GripperCallback(self, msg):
        self.SimulatorRightGripperAction = msg.position[0]
        
        
    def MTML2Simulator_GripperCallback(self, msg):
        self.SimulatorLeftGripperAction = msg.position[0]
        
        
    def construct_action(self):
        # translation
        self.SimulatorRightAction[0] = self.SimulatorRightTransAction[0]
        self.SimulatorRightAction[1] = self.SimulatorRightTransAction[1]
        self.SimulatorRightAction[2] = self.SimulatorRightTransAction[2]

        # orientation
        self.SimulatorRightAction[3] = self.SimulatorRightWristAction[0]
        self.SimulatorRightAction[4] = self.SimulatorRightWristAction[1]
        self.SimulatorRightAction[5] = self.SimulatorRightWristAction[2]
        
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
        
        # call an empty step so that dirs to store data is created
        action = self.construct_action()
        self._env.step(action)
        
        # path to save result
        self._json_path = os.path.join(self._env.ep_directory, 'result.json')
        
        start_time = time.time()
        # rospy.loginfo()
        print('Start Time Count ...')
        count = 0
        while not rospy.is_shutdown():
            
            # ================ get simulator state =======================
            self.ee_orien_quat[0] = obs['robot0_eef_quat'][0]
            self.ee_orien_quat[1] = obs['robot0_eef_quat'][1]
            self.ee_orien_quat[2] = obs['robot0_eef_quat'][2]
            self.ee_orien_quat[3] = obs['robot0_eef_quat'][3]
            
            # ================== action construction =====================
            action = self.construct_action()
            obs, _, _, _ = self._env.step(action)
            # do visualization
            self._env.render()
            
            if self._env.successful:
                count += 1
                
            # check exit condition
            ep_time = time.time() - start_time
            
            if (self._env.successful and count >= 5) or ep_time > 120.:
                print(f'Exit with Elapsed_time: {ep_time} seconds.')
                json_object = json.dumps(
                    {
                    'operator': tele_config['operator'],
                    'level': tele_config['proficiency_level'],
                    'status': self._env.successful,
                    'elapsed time': ep_time,
                    'config': config
                    },
                )
                with open(self._json_path, "w") as outfile:
                    outfile.write(json_object)
                self._env.close()
                rospy.signal_shutdown(True)
                # exit()
                
            # =========== pub/log some simulator data here if necessary =======
            
            # =============================================================
            
        
        
if __name__ == "__main__":
    
    rospy.init_node('Simulator', anonymous=False)
    
    simulator = Simulator(config)
    simulator.run()
    rospy.spin()