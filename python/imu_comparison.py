import numpy as np
import imu_core.imu_core_cpp as IMU

import meshcat
import pinocchio as pin
import time
from scipy.spatial.transform import Rotation
from bno_imu import BnoImu

vis = meshcat.Visualizer().open()

def add_frame(name, vis):
    xbox = meshcat.geometry.Box([0.1, 0.01, 0.01])
    vis["xbox_" + name].set_object(xbox, meshcat.geometry.MeshLambertMaterial(
                                    color=0xFF0000))
    ybox = meshcat.geometry.Box([0.01, 0.1, 0.01])
    vis["ybox_" + name].set_object(ybox, meshcat.geometry.MeshLambertMaterial(
                                    color=0x00FF00))
    zbox = meshcat.geometry.Box([0.01, 0.01, 0.1])
    vis["zbox_" + name].set_object(zbox, meshcat.geometry.MeshLambertMaterial(
                                    color=0x0000FF))

def update_frame(name, vis, R, offset = np.zeros(3)):
    X_TG = np.eye(4)
    X_TG[0,3] = 0.05
    Y_TG = np.eye(4)
    Y_TG[1,3] = 0.05
    Z_TG = np.eye(4)
    Z_TG[2,3] = 0.05

    offset_TG = np.eye(4)
    offset_TG[0:3,3] = offset


    T = np.eye(4)
    T[0:3,0:3] = R
    vis["xbox_" + name].set_transform( offset_TG @ T @ X_TG )
    vis["ybox_" + name].set_transform( offset_TG @ T @ Y_TG )
    vis["zbox_" + name].set_transform( offset_TG @ T @ Z_TG )



# micro_imu1 = IMU.Imu3DM_GX3_45("/dev/ttyACM1", True)
# micro_imu1.initialize()
# add_frame("1", vis)
# micro_imu2 = IMU.Imu3DM_GX3_45("/dev/ttyACM6", True)
# micro_imu2.initialize()
# add_frame("2", vis)

rooh_imu1 = BnoImu("/dev/ttyACM0")
add_frame("3", vis)
rooh_imu2 = BnoImu("/dev/ttyACM1")
# add_frame("4", vis)


while True:
    # iRb1 = (micro_imu1.get_rotation_matrix()).copy()
    # update_frame("1", vis, iRb1, np.array([0.5, 0, 0]))
    # iRb2 = (micro_imu2.get_rotation_matrix()).copy().T
    # update_frame("2", vis, iRb2, np.array([0.5, 0, 0]))

    time.sleep(0.01)
    try:
        iRbr1 = Rotation.from_quat(rooh_imu1.read()["q"]).as_matrix()
        iRbr2 = Rotation.from_quat(rooh_imu2.read()["q"]).as_matrix()
        update_frame("3", vis, iRbr2.T @ iRbr1)
        # update_frame("4", vis, iRbr2)
    except:
        print("missed signal")
    
    