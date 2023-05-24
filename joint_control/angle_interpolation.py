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


from pid import PIDAgent
from keyframes import hello
import numpy as np


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        t = perception.time
        for joint_iter in range(len(keyframes[0])):
            joint_name = keyframes[0][joint_iter]
            joint_timestamps = keyframes[1][joint_iter]
            joint_keyframes = keyframes[2][joint_iter]
            
            interval_indicator = 0
            
            for j in range(len(joint_timestamps)):
                if joint_timestamps[j] > (t + 0.01) % 10:
                    interval_indicator = j
                    break
                    
            if interval_indicator == 0:
                target_joints[joint_name] = joint_keyframes[j][0]
            else:
                stime = joint_timestamps[interval_indicator -1]
                etime = joint_timestamps[interval_indicator]
                interval_etime = etime - stime
                
                if interval_indicator == 0:
                    point0 = np.array([0.0,0.0])
                    point1 = np.array([0.0,0.0])
                else:
                    point0 = np.array([joint_keyframes[interval_indicator -1][0], 0.0])
                    point1 = np.array([joint_keyframes[interval_indicator -1][0] + joint_keyframes[interval_indicator -1][2][2], joint_keyframes[interval_indicator -1][2][1]/ interval_etime])
                point2 = np.array([joint_keyframes[interval_indicator][0] + joint_keyframes[interval_indicator][1][2], (joint_keyframes[interval_indicator][1][1] + etime - stime)/interval_etime])
                point3 = np.array([joint_keyframes[interval_indicator][0], 1.0])
            
                interval_time = (((t +0.01) % 10) - stime) / interval_etime
                target_joints[joint_name] = ((1 - interval_time) ** 3 * point0 + 3 * (1 - interval_time) ** 2 * interval_time * point1 + 3 * (1 - interval_time) * interval_time ** 2 * point2 + interval_time ** 3 * point3)[0]
            
        for i in ['HeadYaw','HeadPitch','LShoulderPitch','LShoulderRoll','LElbowYaw','LElbowRoll','LHipYawPitch','LHipRoll','LHipPitch','LKneePitch','LAnklePitch','LAnkleRoll','RShoulderPitch','RShoulderRoll','RElbowYaw','RElbowRoll','RHipYawPitch','RHipRoll','RHipPitch','RKneePitch','RAnklePitch','RAnkleRoll']:
            try:
                tester = target_joints[i]
            except KeyError:
                target_joints[i] = 0.0
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
