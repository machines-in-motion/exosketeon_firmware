import numpy as np
import time
from interface import ExoSkeletonUDPInterface
import matplotlib.pyplot as plt

import imu_core.imu_core_cpp as IMU

imu_base = IMU.Imu3DM_GX3_45("/dev/ttyACM2", True)
imu_base.initialize()

iRb = (imu_base.get_rotation_matrix()).copy().T
print(iRb)