import time
from pid import PidController
from esc import ESCs
from ahrs import AHRS
from altimeter import Altimeter
from esc import ESCs
import numpy as np
import rcpy

class Quad:
    
    Platform = 'BeagleBoneBlue'
    
    PID_ANGULAR =  [[0.0090,0.0,0.0035],[0.0090,0,0.0035],[0,0,0]]
    PID_ALTITUDE = [[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]]
    RCPY_STATES = ['IDLE','RUNNING','PAUSED','EXITING']
    def __init__(self):
        #self.altimeter = Altimeter()
        self.ahrs = AHRS()
        self.pid_angular = PidController(Quad.PID_ANGULAR, self.ahrs.get_angular_position, [0,0,0], PidController.t_angular)
        #self.pid_altitude = PidController(Quad.PID_ALTITUDE, self.altimeter.get_altitude, [0,0,0], PidController.t_linear, lower=[0.0,0.0,0.0,0.0], upper=[.4,.4,.4,.4])
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
        #self.altimeter.start()
        print('starting / rcpy final state =', Quad.RCPY_STATES[rcpy.get_state()])
     
    def stop(self):
        self.escs.stop()
        #self.altimeter.stop()
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
        
    def ahrs_test(self):
        print('AHRS test')
        for n in range(1000):
            print(self.ahrs.get_angular_position())

    
    def f(self):
        return [1,0,0],[0.0,0,0],0.01
        
    def run(self):
        print('getting ready...')
        throttle = [0.0,0.0,0.0,0.0]
        time.sleep(2)
        print("running...")
        t0 = time.time()
        t_stop = t0 + 10.0 # duration of flight
        
        while time.time() < t_stop:
            throttle = [0.0,0.0,0.0,0.0]
            throttle = np.add(throttle, self.pid_angular.update())
            self.escs.set_throttle(throttle)
        
        print(self.pid_angular)
        print('============THROTTLE=========================================')
        print(self.escs)
        
            
q = Quad()
q.start()
#q.spin_test()
#q.ahrs_test()
q.run()
q.stop()
            