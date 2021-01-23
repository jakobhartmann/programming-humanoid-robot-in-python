'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import leftBackToStand
from keyframes import leftBellyToStand
from keyframes import rightBackToStand
from keyframes import rightBellyToStand
from keyframes import hello
from keyframes import wipe_forehead
import pickle
import numpy as np


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = pickle.load(open('joint_control/robot_pose.pkl', 'rb'))  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE

        # necessary for distributed computing to work
        self.posture_classifier = pickle.load(open('joint_control/robot_pose.pkl', 'rb'))  # LOAD YOUR CLASSIFIER

        features = np.array(['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch'])
        classes = ['Crouch', 'Belly', 'Frog', 'Back', 'Left', 'Right', 'StandInit', 'Stand', 'Knee', 'Sit', 'HeadBack']
        data = np.array([])

        for feature in features:
            data = np.append(data, self.perception.joint.get(feature))

        data = np.append(data, self.perception.imu[0])
        data = np.append(data, self.perception.imu[1])

        predicted_class_index = self.posture_classifier.predict([data])[0]
        posture = classes[predicted_class_index]

        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
