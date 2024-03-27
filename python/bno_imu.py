## This is the python interface to read IMU data from the BNU005 IMU. 

import serial
from threading import Thread
import struct

class BnoImu:
    def __init__(self, port = '/dev/ttyACM0'):
        self.port = port
        self.serial = serial.Serial(port, baudrate=115200, timeout=1)
        self.running = True
        self.state = None
        self.thread = Thread(target=self.update)
        self.thread.start()

    def update(self):
        while self.running:
            data = self.serial.read_until(b'abc\n')
            data = struct.unpack('16f',data[:-4])
            self.state = {'q':data[0:4],
                          'acce':data[4:7],
                          'gyro':data[7:10],
                          'mag':data[10:13],
                          'gravity':data[13:16]}
            
    def read(self):
        return self.state
    
    def close(self):
        self.running=False
        self.serial.close()