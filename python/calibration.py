import numpy as np
import time
from interface import ExoSkeletonUDPInterface
import matplotlib.pyplot as plt

interface = ExoSkeletonUDPInterface()
counter = 0
interface.calibrate()
while True:
    interface.setCommand([0], [0.], [0], [0], [0.0])
    time.sleep(0.001)
    state = interface.getState()
    print(state["motor_q"], state["q"])

flag = 1
time_check = 0
wait_time = 1.0

# while flag == 1:
#     time.sleep(0.001)
#     state = interface.getState()
#     if state is not None:
#         data = interface.getState()
#         q_motor = data['motor_q']
#         dq_motor  = data['motor_dq']
#         # reset
#         torque = 1.0
#         print(q_motor, data["motor_q"])
#         # print(dq_motor, data["motor_dq"])
#         interface.setCommand([0], [0.], [0], [0], [torque], [0.])
#         if dq_motor < 1e-2:
#             time_check += 0.001
#         if time_check > wait_time:
#             flag = 0
#     else:
#         print('No response received from the leg. Check the network.')


# interface.setCommand([0], [0.], [0], [0], [0.0], [0.])
# print("finished calibrating...")