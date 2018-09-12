import numpy as np
import time
import rcpy 
import rcpy.mpu9250 as mpu9250
import rcpy.servo as servo
import rcpy.clock as clock
from nxp_imu import IMU

class AHRS_BNO055:
    def __init__(self):
        self.imu = BNO055.BNO055(serial_port='/dev/serial0', rst=18) # connected to UART @ 115200 baud, not I2C bus
        self.xyz = (None, None, None)
        self.i = 0
        
    def get(self):
        self.i += 1
        y,r,p = self.imu.read_euler()  # TODO: docs say to use quaternions instead of euler angles
        return [r,p,y], time.time()
        
    def start(self):
        if not self.imu.begin():
            raise Exception("IMU failed to initialize")
        
    def __str__(self):
        return 'AHRS_BNO055 i=' + str(self.i)


class GYRO_FXAS21002:

    def __init__(self):
        self.gyro = IMU(dps=250, gyro_bw=800, verbose=False).gyro
        self.calibrated = False
        self.drift = [0.0,0.0,0.0]
        self.xyz = [0.0,0.0,0.0]
        self.xyz_dot = [0.0,0.0,0.0]
        self.i = 0

    def get(self):
        self.i += 1
        x_dot,y_dot,z_dot = self.gyro.get()
        return np.subtract((x_dot,y_dot,z_dot), self.drift), time.time()
        
    def start(self):
        self.calibrate()
        print('sample reading from GYRO_FXAS21002', self.gyro.get())
        
    def calibrate(self):
        print('Calibrating GYRO_FXAS21002...')

        for i in range(10):
            self.gyro.get()

        for i in range(400):
            x, y, z = self.gyro.get()
            v = (x,y,z)
            self.drift = np.add(v, self.drift)

        self.drift = np.divide(self.drift, 400)
        print('Gyro calibration:', self.drift)
        
    def __str__(self):
        return 'GYRO_FXAS21002 i=' + str(self.i)

class MPU_9250:
    def __init__(self):
        if rcpy.get_state() != rcpy.RUNNING:
            rcpy.set_state(rcpy.RUNNING)
        mpu9250.initialize(enable_dmp = True, dmp_sample_rate = 100, enable_fusion = True, enable_magnetometer = True)
        self.i = 0
        
    def get(self):
        self.i += 1
        xyz = np.multiply(mpu9250.read()['tb'], 57.29578) # radians to degrees
        return xyz, time.time()
        
    def start(self):
        print('warming up MPU...')
        xyz = mpu9250.read()['tb']
        while np.sum(xyz) == 0:
            print('MPU is not ready')
            time.sleep(0.10)
            
        for n in range(3):
            print('sample reading from MPU_9250', mpu9250.read()['tb'])
        
    def __str__(self):
        return 'MPU_9250 i=' + str(self.i)
        
class AHRS():
    
    Platform = 'BeagleBoneBlue'
    
    def __init__(self):
        self.gyro_sensor = GYRO_FXAS21002()
        if AHRS.Platform == 'BeagleBoneBlue':
            self.ahrs_sensor = MPU_9250()
        else:
            self.ahrs_sensor = AHRS_BNO055()
        self.i = 0
        self.xyz = [0.0,0.0,0.0]
        self.xyz_dot = [0.0,0.0,0.0]
        self.xyz_clipped = [0.0,0.0,0.0]
        self.t0 = None
        self.t_ahrs = None
        self.dt = None
        
    def start(self):
        self.gyro_sensor.start()
        self.ahrs_sensor.start()
        
    def Hz(x):
        if x == None or x == 0:
            return 0
        else:
            return 1.0/x
        
    def get_angular_position(self):

        if self.t0 == None:
            self.xyz, t1 = self.ahrs_sensor.get()
            self.t_ahrs = t1
            self.dt = 0.0
            self.xyz_dot[0] = 0.0
            self.xyz_dot[1] = 0.0
            self.xyz_dot[2] = 0.0   
        elif time.time() > self.t_ahrs + 1.00: # reset to absolute readings @ 1Hz
            xyz = self.xyz
            self.xyz, t1 = self.ahrs_sensor.get()
            self.t_ahrs = t1
            self.dt = t1 - self.t0
            self.xyz_dot[0] = (self.xyz[0] - xyz[0]) / self.dt
            self.xyz_dot[1] = (self.xyz[1] - xyz[1]) / self.dt
            self.xyz_dot[2] = (self.xyz[2] - xyz[2]) / self.dt
        else:
            self.xyz_dot, t1 = self.gyro_sensor.get()
            self.dt = t1 - self.t0
            ##############################################
            #                                            #
            # DANGER DANGER. Sensors are not aligned     #
            #                                            #
            ##############################################
            self.xyz[0] += self.xyz_dot[1] * self.dt  # between absolute readings, integrate
            self.xyz[1] -= self.xyz_dot[0] * self.dt  # gyro reading over time to advance
            self.xyz[2] += self.xyz_dot[2] * self.dt  # estimate of absolute euler angles.
            
        self.t0 = t1
        self.i += 1
        self.stop_time = time.time()

        self.xyz_clipped = np.minimum(self.xyz,[ 30.0, 30.0, 360.0])
        self.xyz_clipped = np.maximum(self.xyz_clipped,[-30.0,-30.0,-360.0])

        return self.xyz_clipped, self.xyz_dot, self.dt
    
    def __str__(self):   
        return 'AHRS i=' + str(self.i) + ',' + str(self.ahrs.i) + ',' + str(self.gyro.i) + ' ' + str(self.xyz) + str(self.xyz_dot) + ' Hz=' + str(AHRS.Hz(self.dt))