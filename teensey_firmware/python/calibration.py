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
    # print(state["motor_q"], state["q"])
    print(state["wrist_ori"], state["shoulder_ori"], state["base_ori"])

flag = 1
time_check = 0
wait_time = 1.0

