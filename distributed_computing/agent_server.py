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
import xmlrpc.server
import threading
from time import sleep
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from inverse_kinematics import InverseKinematicsAgent


class RequestHandler(xmlrpc.server.SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2')


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self):
        super(ServerAgent, self).__init__()
        self.s = xmlrpc.server.SimpleXMLRPCServer(('localhost', 5500), logRequests=True)
        self.s.register_introspection_functions()
        self.s.register_multicall_functions()
        self.s.register_instance(self)
        self.thread = threading.Thread(target=self.s.serve_forever)
        self.thread.start()
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.perception.joint[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        if joint_name in self.perception.joint:
            self.target_joints[joint_name] = angle
        else:
            print(joint_name + " is not a joint")

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.recognize_posture(self.perception)

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.keyframes(keyframes)
        sleep(max(keyframes[1]))
        #print("done")

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.transforms[name].tolist()

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.set_transform(effector_name, transform)

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

