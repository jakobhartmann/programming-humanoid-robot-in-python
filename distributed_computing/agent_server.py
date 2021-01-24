'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent
from xmlrpc.server import SimpleXMLRPCServer
import threading
import numpy as np


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    # https://docs.python.org/3/library/xmlrpc.client.html#module-xmlrpc.client
    def rpcserver(self):
        server = SimpleXMLRPCServer(('localhost', 8000), allow_none=True)
        print("Listening on port 8000...")
        server.register_instance(self)
        server.serve_forever()

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return agent.perception.joint[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        agent.target_joints[joint_name] = angle
        return

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return agent.recognize_posture(agent.perception)

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        agent.keyframes_execution_finished = False # allows repeatedly executing the keyframes, without restarting the AgentServer
        agent.elapsed_time_until_init = agent.perception.time # allows repeatedly executing the keyframes, without restarting the AgentServer

        agent.keyframes = keyframes
        while True:
            if agent.keyframes_execution_finished:
                return

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        # https://stackoverflow.com/questions/22993302/twisted-xmlrpc-and-numpy-float-64-exception
        return agent.transforms[name].tolist()

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        agent.set_transforms(effector_name, np.array(transform))
        while True:
            if agent.keyframes_execution_finished:
                return

if __name__ == '__main__':
    agent = ServerAgent()
    threading.Thread(target=agent.rpcserver).start()
    agent.run()

