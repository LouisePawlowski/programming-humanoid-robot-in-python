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
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(os.path.dirname(__file__))), 'joint_control'))

from numpy.matlib import matrix, identity, cos, sin, sqrt, dot
from keyframes import hello

from recognize_posture import PostureRecognitionAgent


class ForwardKinematicsAgent(PostureRecognitionAgent):
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
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll','LWristYaw'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll','RWristYaw'],
                       }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE
        R = identity(3)
        t = [0, 0, 0]
        c = cos(joint_angle)
        s = sin(joint_angle)

        if joint_name in ['HeadYaw', 'RShoulderRoll', 'RElbowRoll', 'LShoulderRoll', 'LElbowRoll']:
            # rotation around z
            R = [[c, -s, 0],
                 [s, c, 0],
                 [0, 0, 1]]
        elif joint_name in ['RElbowYaw', 'LElbowYaw', 'LHipRoll', 'LAnkleRoll', 'RHipRoll', 'RAnkleRoll']:
            # rotation around x
            R = [[1, 0, 0],
                 [0, c, -s],
                 [0, s, c]]
        elif joint_name in ['HeadPitch', 'RShoulderPitch', 'LShoulderPitch', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'RHipPitch', 'RKneePitch', 'RAnklePitch']:
            # rotation around y
            R = [[c, 0, s],
                 [0, 1, 0],
                 [-s, 0, c]]
        elif joint_name in ['LHipYawPitch', 'RHipYawPitch']:
            # rotation around y-z, 45deg
            R = [[c, sqrt(2)/2*s, sqrt(2)/2*s],
                 [-sqrt(2)/2*s, (c+1)/2, (c-1)/2],
                 [-sqrt(2)/2*s, (c-1)/2, (c+1)/2]]
        if joint_name == 'HeadYaw':
            t = [0, 0, 0.12650]
        elif joint_name == 'LshoulderPitch':
            t = [0, 0.098, 0.100]
        elif joint_name == 'LElbowYaw':
            t = [0.105, 0.015, 0]
        elif joint_name in ['LWristYaw', 'RWristYaw']:
            t = [0.05595, 0, 0]
        elif joint_name == 'RshoulderPitch':
            t = [0, -0.098, 0.100]
        elif joint_name == 'RElbowYaw':
            t = [0.105, -0.015, 0]
        elif joint_name == 'LHipYawPitch':
            t = [0, 0.050, -0.085]
        elif joint_name == 'RHipYawPitch':
            t = [0, -0.050, -0.085]
        elif joint_name in ['LKneePitch', 'RKneePitch']:
            t = [0, 0, -0.100]
        elif joint_name in ['LAnklePitch', 'RAnklePitch']:
            t = [0, 0, -0.1029]
        T = [R[0] + [t[0]],
             R[1]+[t[1]],
             R[2]+[t[2]],
             [0, 0, 0, 1]]
             
        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                if joint in joints:
                    angle = joints[joint]
                    Tl = self.local_trans(joint, angle)
                    # YOUR CODE HERE
                    T = dot(T, Tl)
                    self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.keyframes = hello()
    agent.run()
