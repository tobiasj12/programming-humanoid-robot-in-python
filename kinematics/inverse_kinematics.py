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
from numpy.matlib import identity
import numpy as np
from scipy.linalg import pinv


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def from_trans(self, T):
        '''
        :param transform: 4x4 transform matrix
        :return: x, y, z, theta
        '''
        #lista = [T[3,0], T[3,1], T[3,2]]
        if T[0,0] == 1:
            return [T[3,0], T[3,1], T[3,2], np.arctan2(T[2,1],T[1,1])]
        elif T[1,1] == 1:
            return [T[3,0], T[3,1], T[3,2], np.arctan2(T[0,2], T[0,0])]
        elif T[2,2] == 1:
            return [T[3,0], T[3,1], T[3,2], np.arctan2(T[1,0], T[0,0])]
        else:
            return [T[3,0], T[3,1], T[3,2],0]
    
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = {}
        # YOUR CODE HERE
        lambda_ = 1
        max_step = 0.1
        for chain in self.chains:
            for joint_name in self.chains[chain]:
                joint_angles[joint_name] = self.perception.joint[joint_name]
            
        target = np.array([self.from_trans(transform)]).T
        
        for i in range(1000):
            Ts = self.forward_kinematics(joint_angles)
            Te = np.array([self.from_trans(Ts[-1])]).T
            print(Te)
            print('_____')
            print(target)
            e = target - Te
            e[e > max_step] = max_step
            e[e < -max_step] = -max_step
            T = np.array([self.from_trans(i) for i in Ts[1:-1]]).T
            
            J = Te - T
            dT = Te - T
            J[0, :] = -dT[1, :]
            J[1, :] = dT[0, :]
            J[-1, :] = 1
            d_theta = lambda_ * pinv(J) * e
            for i, joint_name in enumerate(self.chains[effector_name]):
                joint_angles[joint_name] += np.asarray(d_theta.T)[0][i]
            if np.linalg.norm(d_theta) < 1e-4:
                break
            
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.keyframes = ([], [], [])  # the result joint angles have to fill in
        joint_angles = self.inverse_kinematics(effector_name, transform)
        
        for name in self.chains[effector_name]:
            self.keyframes[0].append(name)
            self.keyframes[1].append([[0,4]])
            self.keyframes[2].append([[self.perception.joint[name], [3, 0, 0]], [joint_angles[name], [3, 0, 0]]])
        

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
