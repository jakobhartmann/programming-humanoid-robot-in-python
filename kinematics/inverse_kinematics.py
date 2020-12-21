'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity, pi, arccos, arctan2, arcsin, sqrt
import numpy as np


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE

        # source: B-Human Team Report and Code Release 2017 (section 8.3.4)
        self.forward_kinematics(self.perception.joint)
        if effector_name == 'LLeg':
            AnkeRoll = self.transforms['LAnkleRoll']
        elif effector_name == 'RLeg':
            AnkeRoll = self.transforms['RAnkleRoll']
        
        Foot2Torso = AnkeRoll * transform # target

        # different calculation than in B-Human Team Report (hip and torso have different heights, translation along z-axis is also necessary)
        if effector_name == 'LLeg':
            Foot2Hip = self.translation(0, -0.050, 0.085) * Foot2Torso
        elif effector_name == 'RLeg':
            Foot2Hip = self.translation(0, 0.050, 0.085) * Foot2Torso

        Foot2HipOrthogonal = self.rotation('x', pi / 4) * Foot2Hip
        HipOrthogonal2Foot = np.linalg.inv(Foot2HipOrthogonal)
        l_translation_HipOrthogonal2Foot = np.linalg.norm(HipOrthogonal2Foot[0:3,3])

        l_upperLeg = 0.100
        l_lowerLeg = 0.1029
        gamma = arccos((l_upperLeg**2 + l_lowerLeg**2 - l_translation_HipOrthogonal2Foot**2) / (2 * l_upperLeg * l_lowerLeg))
        angle_knee = pi - gamma

        angle_footPitch1 = arccos((l_lowerLeg**2 + l_translation_HipOrthogonal2Foot**2 - l_upperLeg**2) / (2 * l_lowerLeg * l_translation_HipOrthogonal2Foot))

        translation_Foot2HipOrthogonal = Foot2HipOrthogonal[0:3,3]
        x = translation_Foot2HipOrthogonal.item(0)
        y = translation_Foot2HipOrthogonal.item(1)
        z = translation_Foot2HipOrthogonal.item(2)

        angle_footPitch2 = arctan2(x, sqrt(y**2 + z**2))
        
        angle_footRoll = arctan2(y, z)

        angle_footPitch = angle_footPitch1 + angle_footPitch2

        Thigh2Foot = self.rotation('x', angle_footRoll) * self.rotation('y', angle_footPitch) * self.translation(0, 0, l_lowerLeg) * self.rotation('y', angle_knee) * self.translation(0, 0, l_upperLeg)
        HipOrthogonal2Thigh = np.linalg.inv(Thigh2Foot) * HipOrthogonal2Foot

        r21 = HipOrthogonal2Thigh.item(9)
        angle_x = arcsin(r21)

        r01 = HipOrthogonal2Thigh.item(1)
        r11 = HipOrthogonal2Thigh.item(5)
        angle_hipYaw = arctan2(-r01, r11)

        r20 = HipOrthogonal2Thigh.item(8)
        r22 = HipOrthogonal2Thigh.item(10)
        angle_hipPitch = arctan2(-r20, r22)

        angle_hipRoll = angle_x - (pi / 4)

        joint_angles = [angle_hipYaw, angle_hipRoll, angle_hipPitch, angle_knee, angle_footPitch, angle_footRoll]
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results'''
        
        # YOUR CODE HERE
        if np.array_equal(transform, identity(4)):
            return
        
        joint_angles = self.inverse_kinematics(effector_name, transform)

        if effector_name == 'LLeg':
            names = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll']
        elif effector_name == 'RLeg':
            names = ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']

        times = [[0.0, 5.0]] * 6

        handle = [3, 0.0, 0.0]
        keys = []
        for i in range(len(names)):
            keys.append([[self.perception.joint[names[i]], handle, handle], [joint_angles[i], handle, handle]])

        self.keyframes = (names, times, keys)  # the result joint angles have to fill in
     

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics (not sure what the desired behavior is)
    T = identity(4)
    # row major
    # T[-1, 1] = 0.05
    # T[-2, 1] = 0.26

    # column major
    T[1, -1] = 0.05
    T[2, -1] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
