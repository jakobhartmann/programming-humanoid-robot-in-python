'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))
import json

from numpy.matlib import matrix, identity, sin, cos, arcsin, arctan2
from angle_interpolation import AngleInterpolationAgent


class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       # YOUR CODE HERE
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'], # LWristYaw, LHand
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'] # RWristYaw, RHand
                       }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def translation(self, x, y, z):
        return matrix([[1, 0, 0, x],
                       [0, 1, 0, y],
                       [0, 0, 1, z],
                       [0, 0, 0, 1]])

    def rotation(self, axis, angle):
        s = sin(angle)
        c = cos(angle)

        R_x = matrix([[1, 0,  0, 0],
                      [0, c, -s, 0],
                      [0, s,  c, 0],
                      [0,  0, 0, 1]])

        R_y = matrix([[c,  0, s, 0],
                     [0,  1, 0, 0],
                     [-s, 0, c, 0],
                     [0,  0, 0, 1]])
 
        R_z = matrix([[c, -s, 0, 0],
                      [s,  c, 0, 0],
                      [0,  0, 1, 0],
                      [0,  0, 0, 1]])

        if axis == 'x':
            return R_x
        elif axis == 'y':
            return R_y
        elif axis == 'z':
            return R_z

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE

        # http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
        X = ['LElbowYaw', 'LHipRoll', 'LAnkleRoll', 'RHipRoll', 'RAnkleRoll', 'RElbowYaw']        
        Y = ['HeadPitch', 'LShoulderPitch', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RShoulderPitch']
        Z = ['HeadYaw', 'LShoulderRoll', 'LElbowRoll', 'RShoulderRoll', 'RElbowRoll']
        Y_Z45 = ['LHipYawPitch', 'RHipYawPitch']

        # http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
        # in meters to fit samples
        links = {'HeadYaw': [0, 0, 0.1265],
                 'HeadPitch': [0, 0, 0],
                 'LShoulderPitch': [0, 0.098, 0.1],
                 'LShoulderRoll': [0, 0, 0],
                 'LElbowYaw': [0.105, 0.015, 0],
                 'LElbowRoll': [0, 0, 0],
                 'LWristYaw': [0.05595, 0, 0], # not included
                 'LHipYawPitch': [0, 0.05, -0.085],
                 'LHipRoll': [0, 0, 0],
                 'LHipPitch': [0, 0, 0],
                 'LKneePitch': [0, 0, -0.1],
                 'LAnklePitch': [0, 0, -0.1029],
                 'LAnkleRoll': [0, 0, 0],
                 'RShoulderPitch': [0, -0.098, 0.1],
                 'RShoulderRoll': [0, 0, 0],
                 'RElbowYaw': [0.105, -0.015, 0],
                 'RElbowRoll': [0, 0, 0],
                 'RWristYaw': [0.05595, 0, 0], # not included
                 'RHipYawPitch': [0, -0.05, -0.085],
                 'RHipRoll': [0, 0, 0],
                 'RHipPitch': [0, 0, 0],
                 'RKneePitch': [0, 0, -0.1],
                 'RAnklePitch': [0, 0, -0.1029],
                 'RAnkleRoll': [0, 0, 0]}

        x = links[joint_name][0]
        y = links[joint_name][1]
        z = links[joint_name][2]

        if joint_name in X:
            T = self.translation(x, y, z) * self.rotation('x', joint_angle)
        elif joint_name in Y:
            T = self.translation(x, y, z) * self.rotation('y', joint_angle)
        elif joint_name in Z:
            T = self.translation(x, y, z) * self.rotation('z', joint_angle)
        elif joint_name in Y_Z45:
            T = self.translation(x, y, z) * self.rotation('y', joint_angle) * self.rotation('z', joint_angle)

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE

                T = T * Tl

                self.transforms[joint] = T

                # test forward kinematics with fk_samples.json
                if joint in ['HeadPitch', 'LElbowRoll', 'RElbowRoll', 'LAnkleRoll', 'RAnkleRoll']:
                    x = T.item(3)
                    y = T.item(7)
                    z = T.item(11)
                    
                    # source: B-Human Team Report and Code Release 2017 (section 8.3.4)
                    r21 = T.item(9)
                    angle_x = arcsin(r21)

                    r20 = T.item(8)
                    r22 = T.item(10)
                    angle_y = arctan2(-r20, r22)

                    r01 = T.item(1)
                    r11 = T.item(5)
                    angle_z = arctan2(-r01, r11)

                    position = [x, y, z, angle_x, angle_y, angle_z]
                    # print(joint, position)
    

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()

    with open('kinematics/fk_samples.json') as json_file:
        data = json.load(json_file)

    joint_names = data['joint_names']
    endEffector_names = data['names']
    moveInit_positions = data['moveInit']['positions']
    moveInit_angles = data['moveInit']['angles']
    init_positions = data['init']['positions']
    init_angles = data['init']['angles']
    rest_positions = data['rest']['positions']
    rest_angles = data['rest']['angles']

    names = joint_names
    times = [[0.0, 1.0]] * 22

    joint_angles = init_angles # define what keyframe to test: init, moveInit, rest
    keys = []
    handle = [3, 0.0, 0.0]
    for i in range(len(names)):
        keys.append([[agent.perception.joint[names[i]], handle, handle], [joint_angles[i], handle, handle]])

    agent.keyframes = (names, times, keys)

    agent.run()
