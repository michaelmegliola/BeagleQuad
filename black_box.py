import numpy as np
import time

# flight data recorder

class BlackBox:
    def __init__(self, size = (1000,3)):
        self.data = np.zeros(size)
        self.i = 0

    def record(self, v):
        for n in range(len(v)):
            self.data[self.i][n] = v[n]
        self.i += 1
        self.i %= len(self.data)

    def write(self):
        f = open('data-' + str(time.time()) + '.csv','w')
        for m in range(len(self.data)):
            for n in range(len(self.data[m])):
                f.write(str(self.data[m][n]))
                if n < len(self.data[m]) - 1:
                    f.write(',')
            f.write('\n')
        f.write('i=' + str(self.i))
        f.close()

