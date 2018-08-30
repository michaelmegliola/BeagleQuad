import time
import rcpy 
import rcpy.mpu9250 as mpu9250
import rcpy.servo as servo
import rcpy.clock as clock
from pid import PidController
from esc import ESCs
from ahrs import AHRS
from altimeter import Altimeter
from esc import ESCs
import numpy as np

class Quad:
    
    PID_ANGULAR =  [[0.10,0.0,0.05],[0,0,0],[0,0,0]]
    PID_ALTITUDE = [[0.0,0.0,0.0],[0.0,0.0,0.0],[0.10,0.0,0.05]]
    RCPY_STATES = ['IDLE','RUNNING','PAUSED','EXITING']
    def __init__(self):
        self.altimeter = Altimeter()
        self.ahrs = AHRS()
        self.pid_angular = PidController(Quad.PID_ANGULAR, self.ahrs.get_angular_position, [0,0,0], PidController.t_angular)
        self.pid_altitude = PidController(Quad.PID_ALTITUDE, self.altimeter.get_altitude, [0,0,1], PidController.t_linear)
        self.escs = ESCs()
        self.escs.arm()
        self.mpu_time = None
        self.tb = None
        self.tb_dot = None
        
    def start(self):
        print('starting / rcpy initial state =', Quad.RCPY_STATES[rcpy.get_state()])
        rcpy.set_state(rcpy.RUNNING)
        self.ahrs.start()
        self.escs.start()
        self.altimeter.start()
        print('starting / rcpy final state =', Quad.RCPY_STATES[rcpy.get_state()])
     
    def stop(self):
        self.escs.stop()
        self.altimeter.stop()
        rcpy.exit()
        
    def run(self):
        print("running...")
        ts = np.zeros((10,4))
        t0 = time.time()
        t_alt = 0
        throttle = [0.0,0.0,0.0,0.0]
        for n in range(10):
            t1 = time.time()
            throttle = np.add(throttle, self.pid_angular.update())
            if t1 > t_alt:
                throttle = np.add(throttle, self.pid_altitude.update())
                t_alt = t1 + 0.5
            throttle = np.minimum(throttle, 0.40)
            throttle = np.maximum(throttle, 0.00)
            t0=t1
            ts[n,0] = throttle[0]
            ts[n,1] = throttle[1]
            ts[n,2] = throttle[2]
            ts[n,3] = throttle[3]
            
        print(ts)
        #print(self.pid_angular)
        #print(self.pid_altitude)
            
q = Quad()
q.start()
q.run()
q.stop()
            