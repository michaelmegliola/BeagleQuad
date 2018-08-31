import rcpy
import rcpy.servo as servo
import rcpy.clock as clock
import numpy as np
import time

ESC_FORE = 0
ESC_STARBOARD = 1
ESC_AFT = 2
ESC_PORT = 3

MAX_THROTTLE = 0.2   # bench test @ 20% max throttle
MIN_THROTTLE = 0.0

PULSE_FREQUENCY_SECS = 0.02

class ESCs:
    def __init__(self):
        servo.disable()  # turn off 6v servo power rail
        self.throttle = [None,None,None,None]
        self.escs = [servo.ESC(ESC_FORE+1), servo.ESC(ESC_STARBOARD+1), servo.ESC(ESC_AFT+1), servo.ESC(ESC_PORT+1)]
        self.clks = [clock.Clock(esc, PULSE_FREQUENCY_SECS) for esc in self.escs]
        self.armed = False
        

    def arm(self):
        '''
        print('arming ESCs')
        for esc in self.escs:
            esc.pulse(0.0)
        time.sleep(1.0)
        for esc in self.escs:
            esc.pulse(-0.1)
            esc.set(-0.1)
        '''
        self.armed = True

    def start(self):
        if self.armed:
            for clk in self.clks:
                clk.start()
            for esc in self.escs:
                esc.set(0.0)
        else:
            raise ValueError('Cannot start: ESCs are not armed')

    def stop(self):
        print('idling motors')
        for esc, clk in zip(self.escs, self.clks):
            esc.set(-0.1)
            #clk.stop()

    # vector order is: FORE, STARBOARD, AFT, PORT
    def set_throttle(self,t):
        self.throttle = t
        self.throttle = np.minimum(self.throttle, MAX_THROTTLE)
        self.throttle = np.maximum(self.throttle, MIN_THROTTLE)
        for setting, esc in zip(self.throttle, self.escs):
            esc.set(setting)
            
    def __str__(self):
        return str(self.throttle)
