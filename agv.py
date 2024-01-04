import numpy as np
from vrep_client import VREP
import time
import math

from scipy.spatial.transform import Rotation as R

from pid_control import PIDControl


class SimAGV(VREP):
    def __init__(self, num) -> None:
        super().__init__()
        self.agvs = [self.creat_agv(i) for i in range(num)]
        print("[AGV]: creat AGV--",self.agvs)
        self.wheel_base = 0.3309983015060425
        self.wheel_diameter = 0.19411166926398632  # 轮子直径 m
        self.speel_l = 2
        
    def creat_agv(self, i):
        """
        return agv = (agv1_handle,leftMotor1_handle, rightMotor1_handle)
        """
        _agv_name = "Pioneer_p3dx#"+str(i)
        _left_motor = "Pioneer_p3dx_leftMotor#"+str(i)
        _right_motor = "Pioneer_p3dx_rightMotor#"+str(i)

        agv = (self.get_handle(_agv_name),
               self.get_handle(_left_motor), self.get_handle(_right_motor))
        return agv

    def move_l(self, agv_num, tar_pose, s=5, e=0.1, rate=0.01):
        pid = PIDControl(0.05, 0.02, 0.0, 5,-5)
        print(f"[Dispath]:agv-{agv_num} runing new potint--target:{tar_pose}")
        time_start = time.time()
        distance = 100
        agv_handle = self.agvs[agv_num][0]
        car_pose = self.get_pose(agv_handle)
        car_angle = self.get_ori(agv_handle)[2]
        car_radians = np.radians(car_angle)
        rotation_matrix = np.array([[np.cos(car_radians), -np.sin(car_radians), 0],
                                    [np.sin(car_radians), np.cos(car_radians), 0],
                                    [0, 0, 1]])
        vector_ = np.array(
            [tar_pose[0] - car_pose[0], tar_pose[1] - car_pose[1], 0])
        # 将向量差转换到B坐标系中
        transformed_vector = np.dot(rotation_matrix.T, vector_)
        dif_angle = np.degrees(np.arctan2(
                    transformed_vector[1], transformed_vector[0]))
        while distance > e:
            time.sleep(rate)
            car_pose = self.get_pose(agv_handle)
            car_angle = self.get_ori(agv_handle)[2]
            car_radians = np.radians(car_angle)
            rotation_matrix = np.array([[np.cos(car_radians), -np.sin(car_radians), 0],
                                        [np.sin(car_radians), np.cos(car_radians), 0],
                                        [0, 0, 1]])
            vector_ = np.array(
                [tar_pose[0] - car_pose[0], tar_pose[1] - car_pose[1], 0])
            # 将向量差转换到B坐标系中
            transformed_vector = np.dot(rotation_matrix.T, vector_)
            # 计算旋转角度（方位角度）
            dif_angle = np.degrees(np.arctan2(
                    transformed_vector[1], transformed_vector[0]))
            # TODO use pid
            distance = np.linalg.norm(vector_)
            p = pid.update(0, dif_angle, rate)
            print(f"[AGV] movel====>> pid: {p},tar_angle: {0}, dif_angle: {dif_angle}, distanc:{distance} \n")
            #   减速
            if distance < 0.5:
                print(f"[AGV] movel==== 0.5 distance:{distance}")
                self.run(agv_num, (s+p)/2, (s-p)/2)
            elif distance < 0.1:
                print(f"[AGV] movel==== 0.5 distance:{distance}")
                self.run(agv_num, (s+p)/3, (s-p)/3)
            else:
                self.run(agv_num, s+p, s-p)
        self.run(agv_num, 0, 0)
        # time.sleep(1)
        return time.time() - time_start

    def move_l_back(self, agv_num, tar_pose, s=5, e=0.1, rate=0.01):
        pid = PIDControl(0.05, 0.02, 0.0, 5,-5)
        print(f"[Dispath]:agv-{agv_num} runing new potint--target:{tar_pose}")
        time_start = time.time()
        distance = 100
        agv_handle = self.agvs[agv_num][0]
        car_pose = self.get_pose(agv_handle)
        car_angle = self.get_ori(agv_handle)[2]
        car_radians = np.radians(car_angle)
        rotation_matrix = np.array([[np.cos(car_radians), -np.sin(car_radians), 0],
                                    [np.sin(car_radians), np.cos(car_radians), 0],
                                    [0, 0, 1]])
        turn_matrix = R.from_euler('xyz', [0,0,180], degrees=True).as_matrix()
        rotation_matrix = np.dot(rotation_matrix, turn_matrix)
        vector_ = np.array(
            [tar_pose[0] - car_pose[0], tar_pose[1] - car_pose[1], 0])
        # 将向量差转换到B坐标系中
        transformed_vector = np.dot(rotation_matrix.T, vector_)
        dif_angle = np.degrees(np.arctan2(
                    transformed_vector[1], transformed_vector[0]))
        while distance > e:
            time.sleep(rate)
            car_pose = self.get_pose(agv_handle)
            car_angle = self.get_ori(agv_handle)[2]
            car_radians = np.radians(car_angle)
            rotation_matrix = np.array([[np.cos(car_radians), -np.sin(car_radians), 0],
                                        [np.sin(car_radians), np.cos(car_radians), 0],
                                        [0, 0, 1]])
            turn_matrix = R.from_euler('xyz', [0,0,180], degrees=True).as_matrix()
            rotation_matrix = np.dot(rotation_matrix, turn_matrix)
            vector_ = np.array(
                [tar_pose[0] - car_pose[0], tar_pose[1] - car_pose[1], 0])
            # 将向量差转换到B坐标系中
            transformed_vector = np.dot(rotation_matrix.T, vector_)
            # 计算旋转角度（方位角度）
            dif_angle = np.degrees(np.arctan2(
                    transformed_vector[1], transformed_vector[0]))
            # TODO use pid
            distance = np.linalg.norm(vector_)
            p = pid.update(0, dif_angle, rate)
            print(f"[AGV] movel====>> pid: {p},tar_angle: {0}, dif_angle: {dif_angle}, distanc:{distance} \n")
            #   减速
            if distance < 0.5:
                print(f"[AGV] movel back==== 0.5 distance:{distance}")
                self.run(agv_num, -(s+p)/2, -(s-p)/2)
            elif distance < 0.1:
                print(f"[AGV] movel back==== 0.5 distance:{distance}")
                self.run(agv_num, -(s+p)/3, -(s-p)/3)
            else:
                self.run(agv_num, -(s+p), -(s-p))
        self.run(agv_num, 0, 0)
        # time.sleep(1)
        return time.time() - time_start

    def move_c(self, agv_num, tar_angle, e=0.25, rate=0.01):
        pid = PIDControl(Kp=0.05, Ki=0.0, Kd=0.0005)
        time_start = time.time()
        cur_angle = self.get_ori(self.agvs[agv_num][0])[2]
        tar_matrix = R.from_euler('xyz', [0,0,tar_angle], degrees=True).as_matrix()
        while abs(tar_angle - cur_angle) > e:
            time.sleep(rate)
            cur_angle = self.get_ori(self.agvs[agv_num][0])[2]
            cur_rotation_matrix = R.from_euler('xyz', [0,0,cur_angle], degrees=True).as_matrix()
            C = np.dot(np.linalg.inv(tar_matrix), cur_rotation_matrix)
            yaw = R.from_matrix(C).as_euler('xyz')  
            yaw = np.degrees(yaw)[-1]
            vl = pid.update(0, yaw, rate)
            print("[AGV] movec vl:{:.3f}, 误差角:{:.3f}, tar_angle:{:.3f}, cur_angle:{:.3f}\n"\
                .format(vl, yaw,tar_angle,cur_angle))
            self.run(agv_num, -vl, vl)
        self.run(agv_num, 0, 0)
        pid.clear()
        return time.time() - time_start

    def move_offset_c(self, agv_num, angle, e=0.5, rate=0.01):
        pid = PIDControl(Kp=0.05, Ki=0.0, Kd=0.0005)
        time_start = time.time()
        cur_angle = self.get_ori(self.agvs[agv_num][0])[2]
        cur_rotation_matrix = R.from_euler('xyz', [0,0,cur_angle], degrees=True).as_matrix()
        angle_matrix = R.from_euler('xyz', [0,0,angle], degrees=True).as_matrix()
        tar_matrix = np.dot(cur_rotation_matrix,angle_matrix)
        tar_angle = R.from_matrix(tar_matrix).as_euler('xyz')[-1]
        yaw = 10
        while abs(yaw) > e:
            time.sleep(rate)
            cur_angle = self.get_ori(self.agvs[agv_num][0])[2]
            cur_rotation_matrix = R.from_euler('xyz', [0,0,cur_angle], degrees=True).as_matrix()
            C = np.dot(np.linalg.inv(tar_matrix), cur_rotation_matrix)
            yaw = R.from_matrix(C).as_euler('xyz')  
            yaw = np.degrees(yaw)[-1]
            vl = pid.update(0, yaw, rate)
            print("[AGV] movec vl:{:.3f}, 误差角:{:.3f}, tar_angle:{:.3f}, cur_angle:{:.3f}\n"\
                .format(vl, yaw,tar_angle,cur_angle))
            self.run(agv_num, -vl, vl)
        self.run(agv_num, 0, 0)
        pid.clear()
        return time.time() - time_start

    def run(self, agv_num, lspeed, rspeed):
        """
        speed:     0.029628699166434153 m/s
                或 0.3070062009269009 弧度/s  xuan
                或 17.59015959745676 角度/s 

        轮子直径： 0.19411166926398632                    
        """
        left_motor_handle = self.agvs[agv_num][1]
        right_motor_handle = self.agvs[agv_num][2]
        r_res = self.set_joint_velocity(right_motor_handle, rspeed)
        l_res = self.set_joint_velocity(left_motor_handle, lspeed)

    def get_agvs(self):
        return self.agvs

