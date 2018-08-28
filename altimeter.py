import VL53L1X

class Altimeter():
    
    SHORT_RANGE = 1
    MEDIUM_RANGE = 2
    LONG_RANGE = 3
    
    def __init__(self):
        self.tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)

    def start(self):
        self.tof.open() # Initialise the i2c bus and configure the sensor
        self.tof.start_ranging(Altimeter.SHORT_RANGE)
        print('altimeter started; altitude (m) =', self.get(),'timing budget (us)=', self.tof.get_timing())

    def get(self):
        return self.tof.get_distance() / 1000.0  # units are meters

    def stop(self):
        self.tof.stop_ranging() # Stop ranging
