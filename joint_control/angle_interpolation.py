'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''

import numpy as np
import operator
from functools import reduce

from pid import PIDAgent
from keyframes import leftBackToStand
from keyframes import leftBellyToStand
from keyframes import rightBackToStand
from keyframes import rightBellyToStand
from keyframes import hello
from keyframes import wipe_forehead


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        # https://isis.tu-berlin.de/mod/forum/discuss.php?d=285263
        self.elapsed_time_until_init = self.perception.time
        self.keyframes_execution_finished = False

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE

        # parameters for testing
        allow_repeat_after_first_init = True
        repeat_standing_up = True
        repeat_angle_interpolation = False

        if repeat_standing_up and keyframes == ([], [], []):
            self.elapsed_time_until_init = self.perception.time

        names = keyframes[0]
        times = keyframes[1]
        keys = keyframes[2]

        if allow_repeat_after_first_init:
            time = self.perception.time - self.elapsed_time_until_init
        else:
            time = self.perception.time

        for n in range(len(names)):
            if (time > times[n][-1]):
                break
            else:
                repeat_angle_interpolation = False

            for t in range(len(times[n]) - 1):
                if (times[n][t] <= time <= times[n][t+1]):
                    starttime = times[n][t]
                    endtime = times[n][t+1]

                    # https://isis.tu-berlin.de/mod/forum/discuss.php?d=288132
                    P_0 = keys[n][t][0]
                    P_1 = keys[n][t][2][2] + P_0
                    P_3 = keys[n][t+1][0]
                    P_2 = keys[n][t+1][1][2] + P_3

                    # https://medium.com/@adrian_cooney/bezier-interpolation-13b68563313a
                    i = (starttime - time) / (starttime - endtime)

                    # lecture 2, slide 15
                    B = pow((1 - i), 3) * P_0 + 3 * pow((1 - i), 2) * i * P_1 + 3 * (1 - i) * pow(i, 2) * P_2 + pow(i, 3) * P_3
                    target_joints[names[n]] = B
                    break

        if 'LHipYawPitch' in target_joints:
            target_joints['RHipYawPitch'] = target_joints.get('LHipYawPitch')

        if 'RHipYawPitch' in target_joints:
            target_joints['LHipYawPitch'] = target_joints.get('RHipYawPitch')

        if repeat_angle_interpolation:
            self.elapsed_time_until_init = self.perception.time
        
        if len(times) > 0:
            # https://stackoverflow.com/questions/952914/how-to-make-a-flat-list-out-of-list-of-lists
            times_flattend = reduce(operator.concat, times)
            times_flattend_max = np.max(times_flattend)
            if target_joints == {} and ((self.elapsed_time_until_init + times_flattend_max) < self.perception.time):
                self.keyframes_execution_finished = True

        return target_joints


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
