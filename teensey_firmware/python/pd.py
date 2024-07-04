import numpy as np
import time
from interface import ExoSkeletonUDPInterface
import matplotlib.pyplot as plt

interface = ExoSkeletonUDPInterface()

P = 2.0
D = 0.05
T = 5000
DT = 0.001
q_des = -0.0
time_init = time.time()

counter = 0
interface.setCommand([0.], [0.], [0.], [0.0], [0.0], [0.])
time.sleep(0.01)
state = interface.getState()
if state is not None:
    data = interface.getState()
    q_init = data['q']

recorded = np.zeros((T,2))
flag = 1
t = 0
F = 0.002
while flag == 1:
    time.sleep(0.001)
    state = interface.getState()
    if state is not None:
        data = interface.getState()
        q_motor = data['motor_q']
        dq_motor  = data['motor_dq']
        q_des = 0*np.sin(F*t)
        dq_des = F*0*np.cos(F*t)
        # reset
        torque = -2.0 #P*(q_des - q_motor) + D * (dq_des - dq_motor)
        # torque = 0.0
        print(q_motor, data["motor_q"])
        # print(dq_motor, data["motor_dq"])
        interface.setCommand([0], [0.], [0], [0], [torque], [0.])
        t += 1
    else:
        print('No response received from the leg. Check the network.')

interface.setCommand([0], [0.], [0], [0], [0.0], [0.])
print("finished calibrating...")
