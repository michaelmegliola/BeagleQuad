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
        self.pid_altitude = PidController(Quad.PID_ALTITUDE, self.altimeter.get_altitude, [0,0,2], PidController.t_linear, lower=[.25,.25,.25,.25], upper=[.4,.4,.4,.4])
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
        
    def spin_test(self):
        print('apply power')
        self.escs.set_throttle([0.0,0.0,0.0,0.0])
        for n in range(10):
            print(n)
            time.sleep(1)
        print('spin test')
        self.escs.set_throttle([0.25,0.25,0.25,0.25])
        time.sleep(2)
        self.escs.set_throttle([0.0,0.0,0.0,0.0])
        
    def run(self):
        print('getting ready...')
        throttle = [0.0,0.0,0.0,0.0]
        time.sleep(2)
        print("running...")
        t0 = time.time()
        t_stop = t0 + 1.5  # duration of flight
        
        while time.time() < t_stop:
            throttle = [0.0,0.0,0.0,0.0]
            throttle = np.add(throttle, self.pid_angular.update())
            throttle = np.add(throttle, self.pid_altitude.update())
            self.escs.set_throttle(throttle)
            print(self.pid_altitude)
            
        print(self.pid_angular)
        
            
q = Quad()
q.start()
#q.spin_test()
q.run()
q.stop()
            