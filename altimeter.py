import time
import VL53L1X

class Altimeter():
    
    SHORT_RANGE = 1
    MEDIUM_RANGE = 2
    LONG_RANGE = 3
    
    Z = 2
    Hz = 4
    time_horizon = 1.0 / Hz
    
    def __init__(self):
        self.tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
        self.t0 = None
        self.dt = None
        self.xyz = None
        self.xyz_dot = None

    def start(self):
        self.tof.open() # Initialise the i2c bus and configure the sensor
        self.tof.start_ranging(Altimeter.SHORT_RANGE)
        print('altimeter started; altitude (m) =', self.tof.get_distance(),'timing budget (us)=', self.tof.get_timing())

    def get_altitude(self):
        if self.t0 == None:
            d = self.tof.get_distance() / 1000.0  # units are meters
            self.t0 = time.time()
            self.dt = 0.0
            self.xyz = [0.0,0.0,d]
            self.xyz_dot = [0.0,0.0,0.0]
        elif time.time() > self.t0 + Altimeter.time_horizon:
            d = self.tof.get_distance() / 1000.0  # units are meters
            t1 = time.time()
            self.xyz_dot[Altimeter.Z] = d - self.xyz[Altimeter.Z]
            self.xyz[Altimeter.Z] = d
            self.dt = t1 - self.t0
            self.t0 = t1
        return self.xyz, self.xyz_dot, self.dt
            
    def stop(self):
        self.tof.stop_ranging() # Stop ranging
        
    def __str__(self):
        return str(self.xyz) + str(self.xyz_dot) + str(self.dt)
