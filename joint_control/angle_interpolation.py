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
import numpy as np
from keyframes import hello
from keyframes import wipe_forehead
from keyframes import leftBackToStand
from keyframes import leftBellyToStand
from keyframes import rightBackToStand
from keyframes import rightBellyToStand


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
        if 'LHipYawPitch' in target_joints:
            target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        names, times, keys = keyframes
        if len(times) == 0:
            return target_joints
        max_time = max(max(times))

        current_time = perception.time
        

        while current_time > max_time:
            current_time -= max_time

        for i in range(len(names)):
            j = -1
            while current_time > times[i][j+1] and j + 2 < len(times[i]):
                j += 1
            if j+1 < len(times[i]) and j > -1:
                P0 = keys[i][j][0]
                P1 = keys[i][j][0] + keys[i][j][2][2]
                P2 = keys[i][j+1][0] + keys[i][j+1][1][2]
                P3 = keys[i][j+1][0]

                t = (current_time - times[i][j]) / (times[i][j+1] - times[i][j])
                B_t = (1-t)**3 * P0 + 3 * (1-t)**2 * t * P1 + 3*(1-t) * t**2 * P2 + t**3 * P3
                target_joints[names[i]] = B_t
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello() # CHANGE DIFFERENT KEYFRAMES
    agent.run()
