import time
import rcpy 
import rcpy.mpu9250 as mpu9250
import rcpy.servo as servo
import rcpy.clock as clock
from pid import PidController
from esc import ESCs
from ahrs import AHRS
import numpy as np

class Quad:
    
    PID_ANGULAR = [[0.10,0.0,0.05],[0,0,0],[0,0,0]]
    RCPY_STATES = ['IDLE','RUNNING','PAUSED','EXITING']
    def __init__(self):
        self.ahrs = AHRS()
        self.pid = PidController(Quad.PID_ANGULAR, self.ahrs.get_angular_position, [0,0,0], PidController.t_angular)
        self.escs = ESCs()
        self.mpu_time = None
        self.tb = None
        self.tb_dot = None
        
    def start(self):
        print('starting / rcpy initial state =', Quad.RCPY_STATES[rcpy.get_state()])
        rcpy.set_state(rcpy.RUNNING)
        self.ahrs.start()
        self.escs.start()
        print('starting / rcpy final state =', Quad.RCPY_STATES[rcpy.get_state()])
     
    def stop(self):
        rcpy.exit()
        
    def pid_test(self):
        for n in range(1000):
            self.pid.update()
        print(self.ahrs)
        print(self.pid)
            
q = Quad()
q.start()
#q.pid_test()
q.escs.spin_test()
q.stop()
            