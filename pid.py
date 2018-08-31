import numpy as np
import sys

'''
3-axis PID controller

 - requires [kp,ki,kd] for each of 3 axes (x,y,z)
 - accepts translation matrix to apply pid output to motor speeds [fore, starboard, aft, port]
 - default translation matrix assumes controller is applied to angular velocity
 - return value from update() is a vector of motor speed adjustments [fore, starboard, aft, port]
'''

class PidController:
    
    t_angular = [[-1,0,1,0],[0,-1,0,1],[-1,1,-1,1]]    # translation matrix to apply angular values to motors
    t_linear =  [[1,0,-1,0],[0,1,0,-1],[-1,-1,-1,-1]]  # translation matrix to apply linear values to motors
    
    P = 0
    I = 1
    D = 2
    
    def __init__(self, k_3_3, f_state, target, t, lower=[0,0,0,0], upper=[1,1,1,1], i=0.1):
        self.k = k_3_3              # [kp,ki,kd] for each of 3 axes (x,y,z)
        self.f_state = f_state      # function that returns current state (x,y,z)
        self.target = target        # target setpoints for 3 axes (x,y,z)
        self.p = [None,None,None]   # p for 3 axes (x,y,z)
        self.i = [0.0,0.0,0.0]      # i for 3 axes (x,y,z)
        self.d = [0.0,0.0,0.0]      # d for 3 axes (x,y,z)
        self.pid = [0.0,0.0,0.0]    # resulting pid values for 3 axes (x,y,z)
        self.t = t                  # translation matrix to apply pid to motor speeds
        self.lower = lower          # lower limit for throttle adjustment
        self.upper = upper          # upper limit for throttle adjustment
        self.i_limit = i            # limit for i term
        self.count = 0
        self.dt = 0.0
        self.throttle_adj = [0.0,0.0,0.0,0.0]
        self.t_a_max = [0.0,0.0,0.0,0.0]
        self.t_a_min = [999.9,999.9,999.9,999.9]
    
    def update(self):
        self.count += 1
        xyz, xyz_dot, dt = self.f_state()
        self.dt += dt
        self.throttle_adj = [0.0,0.0,0.0,0.0]
        for n in range(3):
            error = xyz[n] - self.target[n]
            self.p[n] = error
            self.i[n] += error * dt
            self.i[n] = min(self.i[n], self.i_limit)
            self.i[n] = max(self.i[n], -self.i_limit)
            self.d[n] = -xyz_dot[n]
            self.pid[n] =  self.k[n][PidController.P] * self.p[n]
            self.pid[n] += self.k[n][PidController.I] * self.i[n] 
            self.pid[n] += self.k[n][PidController.D] * self.d[n]
            self.throttle_adj = np.add(self.throttle_adj, np.multiply(self.pid[n], self.t[n]))
            self.throttle_adj = np.maximum(self.throttle_adj, self.lower)
            self.throttle_adj = np.minimum(self.throttle_adj, self.upper)
        self.t_a_max = np.maximum(self.t_a_max, self.throttle_adj)
        self.t_a_min = np.minimum(self.t_a_min, self.throttle_adj)
        return self.throttle_adj
    
    def set_target(self, k_3):
        self.target = k_3
        
    def __str__(self):
        vals = np.array([self.p,self.i,self.d,self.pid])
        out =  '---' + str(self.count) + '-----------------------------------\n'
        out += str(vals) + '\n'
        out += str(self.throttle_adj) + '\n'
        out += 'max: ' + str(self.t_a_max) + '\n'
        out += 'min: ' + str(self.t_a_min) + '\n'
        if (self.dt > 0):
            out += 'count= ' + str(self.count) + ', time=' + str(self.dt) + ', Hz=' + str(self.count/self.dt)
        return out
        
        
