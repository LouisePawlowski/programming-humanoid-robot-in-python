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
import numpy as np
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(os.path.dirname(__file__))), 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent

class RequestHandler(xmlrpc.server.SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        
        super(InverseKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.perception.joint[joint_name]


    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.target_joints[joint_name] = angle
        return True


    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        now = self.perception.time
        finale_position = max(max(keyframes[1]))
        self.target_joints = self.angle_interpolation(keyframes, self.perception)
        while self.perception.time < now + 1.5*finale_position:
            self.target_joints = self.angle_interpolation(keyframes, self.perception)
        return True 
        

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        self.forward_kinematics(self.perception.joint)
        T = self.transforms[name]
        trans_list = []
        for i in range(4):
            for j in range(4):
                trans_list += [float(T[i, j])]
        return trans_list

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        T = np.zeros((4, 4))
        for i in range(4):
            for j in range(4):
                T[i, j] = transform[4*i + j]
        self.set_transforms(effector_name, T)
        return True
    
    

class ServerThread(threading.Thread):
    def __init__(self, serverAgent):
        threading.Thread.__init__(self)
        self.localServer = xmlrpc.server.SimpleXMLRPCServer(("localhost",9000), requestHandler=RequestHandler)
        self.localServer.register_function(serverAgent.get_angle)
        self.localServer.register_function(serverAgent.set_angle)
        self.localServer.register_function(serverAgent.get_posture)
        self.localServer.register_function(serverAgent.execute_keyframes)
        self.localServer.register_function(serverAgent.get_transform)
        self.localServer.register_function(serverAgent.set_transform)
        
        

    def run(self):
        print('Serving XML-RPC on localhost port 8000')
        try:
            self.localServer.serve_forever()
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received, exiting.")
            sys.exit(0)

if __name__ == '__main__':
    agent = ServerAgent()
    server = ServerThread(agent)
    server.start()
    agent.run()
    

