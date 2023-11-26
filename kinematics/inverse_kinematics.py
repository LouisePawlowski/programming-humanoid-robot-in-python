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
from numpy import dot, pi, sin, cos, arccos, linalg, arctan2, sqrt, arcsin

def trans_x(x):
    T = [[1, 0, 0, x],
         [0, 1, 0, 0],
         [0, 0, 1, 0],
         [0, 0, 0, 1]]
    return T

def trans_y(y):
    T = [[1, 0, 0, 0],
         [0, 1, 0, y],
         [0, 0, 1, 0],
         [0, 0, 0, 1]]
    return T

def trans_z(z):
    T = [[1, 0, 0, 0],
         [0, 1, 0, 0],
         [0, 0, 1, z],
         [0, 0, 0, 1]]
    return T

def rot_x(theta):
    s = sin(theta)
    c = cos(theta)
    T = [[1, 0, 0, 0],
         [0, c, -s, 0],
         [0, s, c, 0],
         [0, 0, 0, 1]]
    return T

def rot_y(theta):
    s = sin(theta)
    c = cos(theta)
    T = [[c, 0, s, 0],
         [0, 1, 0, 0],
         [-s, 0, c, 0],
         [0, 0, 0, 1]]
    return T

def rot_z(theta):
    s = sin(theta)
    c = cos(theta)
    T = [[c, -s, 0, 0],
         [s, c, 0, 0],
         [0, 0, 1, 0],
         [0, 0, 0, 1]]
    return T


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        l_dist = 0.100
        l_upperleg = 0.100
        l_lowerleg = 0.1029
        
        # YOUR CODE HERE
        if effector_name == "RLeg":
            Foot2HipOrthogonal = dot(rot_x(pi/4), dot(trans_y(l_dist/2), transform))
        else:
            Foot2HipOrthogonal = dot(rot_x(pi/4), dot(trans_y(-l_dist/2), transform))
        x = Foot2HipOrthogonal[0,3]
        y = Foot2HipOrthogonal[1,3]
        z = Foot2HipOrthogonal[2,3]
        l_trans = sqrt(x**2 + y**2 + z**2)
        HipOrthogonal2Foot = linalg.inv(Foot2HipOrthogonal)
        gamma = arccos((l_upperleg**2 + l_lowerleg**2 - l_trans**2)/(2*l_upperleg*l_lowerleg))
        delta_knee = pi - gamma
        delta_footpitch1 = arccos((l_lowerleg**2 + l_trans**2 - l_upperleg**2)/(2*l_lowerleg*l_trans))
        delta_footpitch2 = arctan2(x, sqrt(y**2 + z**2))
        delta_footroll = arctan2(y, z)
        delta_footpitch = delta_footpitch1 + delta_footpitch2

        Tigh2Foot = dot(dot(dot(rot_x(delta_footroll), rot_y(delta_footpitch)), dot(trans_z(l_lowerleg), rot_y(delta_knee))), trans_z(l_upperleg))
        HipOrthogonal2Tigh = dot(linalg.inv(Tigh2Foot), HipOrthogonal2Foot)
        delta_x = arcsin(HipOrthogonal2Tigh[2,1])
        delta_hipyaw = arctan2(-HipOrthogonal2Tigh[0,1], HipOrthogonal2Tigh[1,1])
        delta_hippitch = arctan2(-HipOrthogonal2Tigh[2,0], HipOrthogonal2Tigh[2,2])
        delta_hiproll = delta_x - pi/4

        
        joint_angles = [delta_hipyaw, delta_hiproll, delta_hippitch-delta_hipyaw, delta_knee, delta_footpitch, delta_footroll]
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        joint_angles = self.inverse_kinematics(effector_name, transform)
    
        LLeg = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll']
        RLeg = ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
        names = list()
        times = list()
        keys = list()
        if effector_name == 'LLeg':
            for i in range(len(LLeg)):
                names.append(LLeg[i])
                times.append([1.0, 5.0])
                keys.append([[0, [3, 0, 0], [3, 0, 0]], [joint_angles[i], [3, 0, 0], [3, 0, 0]]])
        else:
            for i in range(len(RLeg)):
                names.append(RLeg[i])
                times.append([1.0, 5.0])
                keys.append([[self.perception.joint[RLeg[i]], [3, 0, 0], [3, 0, 0]], [joint_angles[i], [3, 0, 0], [3, 0, 0]]])
        print("names = ", names)
        print("times = ", times)
        print("keys = ", keys)
        self.keyframes = (names, times, keys)  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[0, -1] = -0.05
    T[1, -1] = 0.05
    T[2, -1] = -0.17
    print("T = ", T)
    agent.set_transforms('LLeg', T)
    agent.run()
