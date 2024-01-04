import vrep
import threading
import time
import numpy as np
import sys


class VREP():
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.linear_velocity = 1
        self.speed = 0.026781272888183594  # m/s
        try:
            vrep.simxFinish(-1)  # 关掉之前连接
            self.clientID = vrep.simxStart(
                '127.0.0.1', 19997, True, True, 5000, 5)  # Connect to CoppeliaSim
            if self.clientID != -1:
                print('connect successfully')
            else:
                # Terminar este script
                sys.exit("[Error]:  no se puede conectar")
        except:
            print('[Error]: Check if CoppeliaSim is open')
        self.running = False
        # self.start_sim()

    def __del__(self):
        # if self.clientID:
        # 断开与V-REP仿真的连接
        self.running = False
        vrep.simxStopSimulation(
            self.clientID, vrep.simx_opmode_blocking)  # 关闭仿真
        vrep.simxFinish(self.clientID)
        time.sleep(1)  # 仿真开启延时5s
        vrep.simxFinish(-1)  # 关闭连接

    def start_sim(self):
        with self.lock:
            res = vrep.simxStartSimulation(
                self.clientID, vrep.simx_opmode_oneshot_wait)
            if res == vrep.simx_return_ok:
                self.running = True
                print("仿真环境已启动")
            else:
                print("[Error]: 仿真环境启动失败")

    def finish_sim(self):
        with self.lock:
            vrep.simxStopSimulation(
                self.clientID, vrep.simx_opmode_blocking)  # 关闭仿真
            self.running = False

    def get_handle(self, name):
        with self.lock:
            e, handle = vrep.simxGetObjectHandle(
                self.clientID, name, vrep.simx_opmode_oneshot_wait)
            if e != 0:
                print("[Error]: get {} handle error !!!".format(name))
            return handle

    def stop_sim(self):
        with self.lock:
            res = vrep.simxStopSimulation(
                self.clientID, vrep.simx_opmode_oneshot)
            if res == vrep.simx_return_ok:
                print("仿真环境已关闭")
            else:
                print("[Error]: 仿真环境关闭失败")

    def get_pose(self, handle):
        """
        agv_name: handle
        return : agv pose [x, y, z]
        """
        with self.lock:
            e, res = vrep.simxGetObjectPosition(
                self.clientID, handle, -1, vrep.simx_opmode_oneshot_wait)
            if e != 0:
                return [None, None, None]
            else:
                return res

    def get_ori(self, handle):
        with self.lock:
            e, res = vrep.simxGetObjectOrientation(
                self.clientID, handle, -1, vrep.simx_opmode_blocking)
            if e != 0:
                return [None, None, None]
            else:
                return np.rad2deg(res)

    def get_joint(self, handle):
        with self.lock:
            e, res = vrep.simxGetJointPosition(
                self.clientID, handle, vrep.simx_opmode_oneshot_wait)
            if e != 0:
                return [None, None, None]
            else:
                return res

    def get_obj_matrix(self, obj_name):
        with self.lock:
            _, obj_handle = vrep.simxGetObjectHandle(
                self.clientID, obj_name, vrep.simx_opmode_oneshot_wait)
            return vrep.getObjectMatrix(obj_handle)

    def set_joint_velocity(self, handle, speed):
        with self.lock:
            res = vrep.simxSetJointTargetVelocity(
                self.clientID, handle, speed, vrep.simx_opmode_oneshot)
