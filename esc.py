import rcpy
import rcpy.servo as servo
import rcpy.clock as clock

class ESCs:
    
    FORE = 0
    STARBOARD = 1
    AFT = 2
    PORT = 3
    
    PULSE_FREQUENCY_SECS = 0.02
    
    def __init__(self):
        self.escs = [servo.ESC(ESCs.FORE+1), servo.ESC(ESCs.STARBOARD+1), servo.ESC(ESCs.AFT+1), servo.ESC(ESCs.PORT+1)]
        self.clks = [clock.Clock(esc, ESCs.PULSE_FREQUENCY_SECS) for esc in self.escs]
        self.armed = False
        servo.disable()  # turn of 6v servo power rail as a precaution (can damage ESCs)

    def arm(self):
        self.armed = True

    def start(self):
        print('Arming ESCs...')
        for esc in self.escs:
            esc.pulse(-0.1)
            esc.set(-0.1)
 
        for clk in self.clks:
            clk.start()
            
        self.armed = True
        print('...ESCs are armed.')

    def stop(self):
        self.armed = False
        for esc, clk in zip(self.escs, self.clks):
            esc.set(0.0)
            clk.stop()

    # vector order is: FORE, STARBOARD, AFT, PORT
    def set(self, throttle_vector):
        if self.armed:
            throttle_vector = np.minimum(throttle_vector, [1.0,1.0,1.0,1.0])
            throttle_vector = np.maximum(throttle_vector, [0.0,0.0,0.0,0.0])
            for setting, esc in zip(throttle_vector, self.escs):
                esc.set(setting)
        else:
            self.stop()