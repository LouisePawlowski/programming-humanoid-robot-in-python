'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
import xmlrpc.client
import numpy as np
#from keyframes import hello

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)
        self.proxy = xmlrpc.client.ServerProxy("http://localhost:9000/RPC2")
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.proxy.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.proxy.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.proxy.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.proxy.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        T_list = self.proxy.get_transform(name)
        T = np.zeros((4, 4))
        for i in range(4):
            for j in range(4):
                T[i, j] = T_list[4*i + j]
        return T

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        trans_list = []
        for i in range(4):
            for j in range(4):
                trans_list += [float(transform[i, j])]        
        self.proxy.set_transform(effector_name, trans_list)


if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE
    #print(agent.get_angle("LKneePitch")) # ----> ok
    #agent.set_angle("LKneePitch", 2.0) # ----> ok
    #print(agent.get_posture()) # ---->
    #agent.execute_keyframes(hello()) # ----> ok
    #print(agent.get_transform("LAnklePitch")) # ----> ok
    T = np.identity(4)
    T[0, -1] = -0.05
    T[1, -1] = 0.05
    T[2, -1] = -0.17
    print("T = ", T)
    agent.set_transform('LLeg', T)


