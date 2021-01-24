'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

import weakref
import xmlrpc.client
import numpy as np
import threading

from numpy.matlib import identity
from keyframes import leftBackToStand
from keyframes import leftBellyToStand
from keyframes import rightBackToStand
from keyframes import rightBellyToStand
from keyframes import hello
from keyframes import wipe_forehead

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        # parameter args with comma at the end must be passed to the function for parallel execution to work!!!
        # threading.Thread(target=self.proxy.execute_keyframes(keyframes)).start() # serial execution!
        threading.Thread(target=self.proxy.execute_keyframes, args=(keyframes, )).start() # parallel execution!

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        threading.Thread(target=self.proxy.set_transform, args=(effector_name, transform, )).start()


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)
        self.server = xmlrpc.client.ServerProxy('http://localhost:8000/')
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.server.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.server.set_angle(joint_name, angle)
        print(f"Target angle for {joint_name} successfully set.")

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.server.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.server.execute_keyframes(keyframes)
        print("Keyframes successfully executed.")

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return np.array(self.server.get_transform(name))

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.server.set_transform(effector_name, transform.tolist())
        print(f"Transformations successfully to {effector_name} applied.")


if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE

    # test get_angle
    print('Angle: ', agent.get_angle('HeadYaw'))

    # test set_angle
    # agent.set_angle('HeadYaw', 1.0)

    # test get_posture
    # print('Posture: ', agent.get_posture())

    # test execute_keyframes
    # agent.execute_keyframes(hello())

    # test get_transform
    # print('Transform:')
    # print(agent.get_transform('HeadYaw'))

    # test set_transform (should be tested independently of execute_keyframes)
    T = identity(4)
    T[1, -1] = 0.05
    T[2, -1] = 0.26
    # agent.set_transform('LLeg', T)

    # test execute_keyframes in PostHandler
    # print('Message before execute_keyframes is called.')
    # agent.post.execute_keyframes(hello())
    # print('Message while execute_keyframes is running.')

    # test set_transform in PostHandler (should be tested independently of execute_keyframes)
    # print('Message before set_transform is called.')
    # agent.post.set_transform('LLeg', T)
    # print('Message while set_transform is running.')

