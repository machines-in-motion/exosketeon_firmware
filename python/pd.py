import numpy as np
import time
from interface import ExoSkeletonUDPInterface
import matplotlib.pyplot as plt

interface = ExoSkeletonUDPInterface()

P = 0.5
D = 0.01
T = 5000
DT = 0.001
q_des = -2.5
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
while flag == 1:
    time.sleep(0.001)
    state = interface.getState()
    if state is not None:
        data = interface.getState()
        q_motor = data['q']
        dq_motor  = data['dq']
        # reset
        torque = 1.0  #+ 10*D * (0.5 - dq_motor)

        if (dq_motor < 0.005):
            counter += 1
        # if counter > 1000:
            # torque = 0.0
            q_offset = q_motor
            # flag = 0
        interface.setCommand([0], [0.], [0], [0], [torque], [0.])

    else:
        print('No response received from the leg. Check the network.')

interface.setCommand([0], [0.], [0], [0], [0.0], [0.])
print("finished calibrating...")

# while True:
#     time.sleep(0.001)
#     state = interface.getState()
#     if state is not None:
#         data = interface.getState()
#         q_motor = data['q'] - q_offset
#         dq_motor  = data['dq']
#         torque = max(P * (min(q_des, 0) - q_motor) + D * (-dq_motor), 0) 
#         # print(q_motor, torque)
#         interface.setCommand([0], [0.], [0], [0], [torque], [0.])
#     else:
#         print('No response received from the leg. Check the network.')

# interface.setCommand([0.], [0.], [0.], [0.0], [0.0], [0.])

# plt.plot(recorded[:,0])
# plt.plot(recorded[:,1])
# plt.legend()
# plt.show()