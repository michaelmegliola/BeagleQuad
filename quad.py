import time
import getopt, sys
import rcpy 
import rcpy.mpu9250 as mpu9250
import rcpy.servo as servo
import rcpy.clock as clock
from pid import PidController
from esc import ESCs
import numpy as np

class Quad:
    
    PID_ANGULAR = [[0.10,0.0,0.05],[0,0,0],[0,0,0]]
    
    def __init__(self):
        self.pid = PidController(Quad.PID_ANGULAR, self.get_angular_position, [0,0,0], PidController.t_angular)
        self.escs = ESCs()
        self.mpu_time = None
        
    def start(self):
        rcpy.set_state(rcpy.RUNNING)
        mpu9250.initialize(enable_dmp = True, dmp_sample_rate = 200, enable_fusion = True, enable_magnetometer = False)
        print('warming up MPU...')
        xyz_0 = mpu9250.read()['tb']
        t0 = time.time()
        while True:
            xyz_1 = mpu9250.read()['tb']
            delta = np.sum(np.absolute(np.subtract(xyz_1,xyz_0)))
            xyz_0 = xyz_1
            if delta < 0.00005:
                break;
            if time.time() > t0 + 15.0:
                raise ValueError("Timeout: MPU did not settle down.")
        print('..MPU is OK to go; settle time (sec) =', time.time()-t0)

    def get_angular_position(self):
        data = mpu9250.read()
        t1 = time.time()
        dt = t1 - self.mpu_time if self.mpu_time != None else 0.0
        self.mpu_time = t1
        return data['tb'], data['gyro'], dt
        
    def pid_test(self):
        for n in range(100):
            self.pid.update()
        print(self.pid)
            
q = Quad()
q.start()
q.pid_test()
            