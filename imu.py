import time
import rcpy 
import rcpy.mpu9250 as mpu9250
import numpy as np

class IMU:
    def __init__(self):
        self.t0 = None
        self.tb = None
        self.tb_dot = None
        self.dt = 0.0
        
    def start(self):
        if rcpy.get_state() != rcpy.RUNNING:
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
        print('...MPU is OK to go; settle time (sec) =', time.time()-t0)

    def get_angular_position(self):
        data = mpu9250.read()
        t1 = time.time()
        tb = data['tb']
        if self.t0 == None:
            self.dt = 0.0
            self.tb = tb
            self.tb_dot = [0.0,0.0,0.0]
        else:
            self.dt = t1 - self.t0
            self.tb_dot = np.subtract(tb, self.tb)
            self.tb_dot = np.divide(self.tb_dot, self.dt)
        self.t0 = t1
        self.tb = tb
        return self.tb, self.tb_dot, self.dt
        
    def __str__(self):
        return 'xyz=' + str(self.tb) + ', xyz_dot=' + str(self.tb_dot) + ', dt=' + str(self.dt)
        
