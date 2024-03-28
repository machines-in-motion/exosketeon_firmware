import socket
import struct
import numpy as np
import time
import threading

MOTION_CMD = 0x01
MOTION_STATE = 0x01

class ExoSkeletonUDPInterface():
    def __init__(self, leg_ip='192.168.123.10', leg_port=5000, 
                       host_ip='192.168.123.1', host_port=5000,
                       num_actuators=1):
        self.num_actuators= num_actuators
        self.leg_ip = leg_ip
        self.leg_port = leg_port
        self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.RX_RUNNING = True
        self.transmit_counter = 0
        self.socket.bind((host_ip, host_port))
        self.rx_thread=threading.Thread(target=self.receivingThread)
        self.rx_thread.start()
        self.state = None
        self.latest_state_stamp = time.time()

    def setCommand(self,q_des, dq_des, kp, kd, tau_ff, q_offsets):
        assert self.num_actuators == len(q_des) == len(dq_des) == len(kp) == len(kd) == len(kd) == len(tau_ff), 'there should be one entry in all inputs per actuator'
        msg_format=f'I{self.num_actuators*6}f'
        stamp = time.time()
        data = np.vstack([q_des, dq_des, kp, kd, tau_ff, q_offsets]).T.reshape(-1)
        arguments =  [msg_format] + [MOTION_CMD] + \
                      data.tolist()
        data=struct.pack(*arguments)
        self.socket.sendto(data, (self.leg_ip, self.leg_port))

    def getState(self):
        if self.state is not None and (time.time()-self.latest_state_stamp) < 0.1:
            return self.state
        else:
            return None
        

    def receivingThread(self):
        while self.RX_RUNNING:
            data = self.socket.recvmsg(4096)
            # breakpoint()
            if data[0][0] == MOTION_STATE:
                msg_format=f'{self.num_actuators*6}f'
                data = struct.unpack(msg_format, data[0][1:])
                state = np.array(data).reshape(self.num_actuators, 6)  
                self.state={'q': state[:,0],
                            'dq': state[:,1],
                            'current': state[:,2],
                            'temp': state[:,3],
                            'motor_q': state[:,4],
                            'motor_dq': state[:,5]}
                self.latest_state_stamp = time.time()
            else:
                print("invalid motion response")

    def terminate(self):
        self.RX_RUNNING = False
        self.socket.close()
        self.rx_thread.join()