# -*- coding:UTF-8 -*-
import time
import numpy as np
import threading
from queue import Queue
# import signal
# signal.signal(signal.SIGINT, signal.SIG_DFL)
from agv import SimAGV
from vrep_map import Map



class Planner():
    def __init__(self, map=Map, agvs=SimAGV) -> None:
        self.agvs = agvs
        self.map = map
        self.agvs.start_sim()

        self.queue = Queue()

    def multithreaded(func):
        def wrapper(*args, **kwargs):
            print("[Dispath]: agv excute routine task")
            return func(*args, **kwargs)
            t = threading.Thread(target=func, args=args, kwargs=kwargs)
            t.daemon = True
            t.start()
            return t
        return wrapper
        
    @multithreaded
    def run_agv0(self):
        agv_num = 0
        np.set_printoptions(suppress=True)
        t = self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane7_13"))
        t = self.agvs.move_c(agv_num, -90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_13"))
        self.agvs.move_c(agv_num, -180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_2"))
        self.agvs.move_c(agv_num, 90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane4_2"))
        self.agvs.move_c(agv_num, 0)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane4_3"))
        time.sleep(1)

    @multithreaded
    def run_agv1(self):
        agv_num = 1
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane8_13"))
        self.agvs.move_c(agv_num, -90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_13"))
        self.agvs.move_c(agv_num, -180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_2"))
        self.agvs.move_c(agv_num, 90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane3_2"))
        self.agvs.move_c(agv_num, 0)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane3_3"))
        time.sleep(1)

    @multithreaded
    def run_agv2(self):
        agv_num = 2
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane9_13"))
        self.agvs.move_c(agv_num, -90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_13"))
        self.agvs.move_c(agv_num, -180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_2"))
        self.agvs.move_c(agv_num, 90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane2_2"))
        self.agvs.move_c(agv_num, 0)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane2_4"))
        time.sleep(1)

    @multithreaded
    def run_agv3(self):
        agv_num = 3
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane10_13"))
        self.agvs.move_c(agv_num, -90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_13"))
        self.agvs.move_c(agv_num, -180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_7"))
        self.agvs.move_c(agv_num, 90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane4_7"))
        self.agvs.move_c(agv_num, 180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane4_6"))
        time.sleep(1)

    @multithreaded
    def run_agv4(self):
        agv_num = 4
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane11_13"))
        self.agvs.move_c(agv_num, -90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_13"))
        self.agvs.move_c(agv_num, -180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_7"))
        self.agvs.move_c(agv_num, 90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane3_7"))
        self.agvs.move_c(agv_num, 180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane3_6"))
        time.sleep(1)

    @multithreaded
    def run_agv5(self):
        agv_num = 5
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane12_13"))
        self.agvs.move_c(agv_num, -90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_13"))
        self.agvs.move_c(agv_num, -180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_7"))
        self.agvs.move_c(agv_num, 90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane2_7"))
        self.agvs.move_c(agv_num, 180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane2_5"))
        time.sleep(1)

    @multithreaded
    def run_agv6(self):
        agv_num = 6
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane7_13"))
        self.agvs.move_c(agv_num, -90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_13"))
        self.agvs.move_c(agv_num, -180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_2"))
        self.agvs.move_c(agv_num, 90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane4_2"))
        self.agvs.move_c(agv_num, 180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane4_1"))
        time.sleep(1)

    @multithreaded
    def run_agv7(self):
        agv_num = 7
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane8_13"))
        self.agvs.move_c(agv_num, -90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_13"))
        self.agvs.move_c(agv_num, -180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_2"))
        self.agvs.move_c(agv_num, 90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane3_2"))
        self.agvs.move_c(agv_num, 180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane3_1"))
        time.sleep(1)

    @multithreaded
    def run_agv8(self):
        agv_num = 8
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane9_13"))
        self.agvs.move_c(agv_num, -90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_13"))
        self.agvs.move_c(agv_num, -180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_2"))
        self.agvs.move_c(agv_num, 90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane2_2"))
        self.agvs.move_c(agv_num, 180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane2_1"))
        time.sleep(1)

    @multithreaded
    def run_agv9(self):
        agv_num = 9
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane10_13"))
        self.agvs.move_c(agv_num, -90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_13"))
        self.agvs.move_c(agv_num, -180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_2"))
        self.agvs.move_c(agv_num, 90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane1_2"))
        self.agvs.move_c(agv_num, 180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane1_1"))
        time.sleep(1)

    @multithreaded
    def run_agv10(self):
        agv_num = 10
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane11_13"))
        self.agvs.move_c(agv_num, -90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_13"))
        self.agvs.move_c(agv_num, -180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_3"))
        self.agvs.move_c(agv_num, 90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane1_3"))
        time.sleep(1)

    @multithreaded
    def run_agv11(self):
        agv_num = 11
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane12_13"))
        self.agvs.move_c(agv_num, -90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_13"))
        self.agvs.move_c(agv_num, -180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_4"))
        self.agvs.move_c(agv_num, 90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane1_4"))
        time.sleep(1)

    @multithreaded
    def run_agv12(self):
        agv_num = 12
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane7_13"))
        self.agvs.move_c(agv_num, -90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_13"))
        self.agvs.move_c(agv_num, -180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_5"))
        self.agvs.move_c(agv_num, 90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane1_5"))
        time.sleep(1)

    @multithreaded
    def run_agv13(self):
        agv_num = 13
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane8_13"))
        self.agvs.move_c(agv_num, -90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_13"))
        self.agvs.move_c(agv_num, -180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_6"))
        self.agvs.move_c(agv_num, 90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane1_6"))
        time.sleep(1)

    @multithreaded
    def run_agv14(self):
        agv_num = 14
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane9_13"))
        self.agvs.move_c(agv_num, -90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_13"))
        self.agvs.move_c(agv_num, -180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_7"))
        self.agvs.move_c(agv_num, 90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane4_7"))
        self.agvs.move_c(agv_num, 0)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane4_8"))
        time.sleep(1)

    @multithreaded
    def run_agv15(self):
        agv_num = 15
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane10_13"))
        self.agvs.move_c(agv_num, -90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_13"))
        self.agvs.move_c(agv_num, -180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_7"))
        self.agvs.move_c(agv_num, 90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane3_7"))
        self.agvs.move_c(agv_num, 0)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane3_8"))
        time.sleep(1)

    @multithreaded
    def run_agv16(self):
        agv_num = 16
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane11_13"))
        self.agvs.move_c(agv_num, -90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_13"))
        self.agvs.move_c(agv_num, -180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_7"))
        self.agvs.move_c(agv_num, 90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane2_7"))
        self.agvs.move_c(agv_num, 0)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane2_8"))
        time.sleep(1)

    @multithreaded
    def run_agv17(self):
        agv_num = 17
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane12_13"))
        self.agvs.move_c(agv_num, -90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_13"))
        self.agvs.move_c(agv_num, -180)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane0_7"))
        self.agvs.move_c(agv_num, 90)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane1_7"))
        self.agvs.move_c(agv_num, 0)
        self.agvs.move_l(agv_num, self.map.get_plane_pose("Plane1_8"))
        time.sleep(1)

    def test(self):
        agv_num = 0
        agv = self.agvs.get_agvs()
        h = agv[agv_num][0]
        target = self.agvs.get_pose(h)
        target[0] +=1.8
        # target[1] += 0.5
        print("target== x:{:.3f}, y: {:.3f}".format(target[0], target[1]))
        t = self.agvs.move_l_back(agv_num, target)
        input("<<<")
        t = self.agvs.move_offset_c(agv_num, -90)
        target[1] +=1.8
        t = self.agvs.move_l_back(agv_num, target)
        target[1] +=1.8
        input("<<<")
        # t = self.agvs.move_l_back(agv_num, target)
        # input("<<<")
        # t = self.agvs.move_offset_c(agv_num, -90)
        # input("<<<")
        # t = self.agvs.move_offset_c(agv_num, -90)
        # input("<<<")
        # t = self.agvs.move_offset_c(agv_num, -90)
        # input("<<<")
        # t = self.agvs.move_offset_c(agv_num, 90)
        # input("<<<")
        # t = self.agvs.move_offset_c(agv_num, 90)
        # input("<<<")
        # t = self.agvs.move_offset_c(agv_num, 90)
        # input("<<<")
        # t = self.agvs.move_offset_c(agv_num, 90)
        # print("t:",t)
        
if __name__ == "__main__":
    # m = Map().save_plane_matrix()
    m = Map()
    t = 2
    agv = SimAGV(12)
    planner = Planner(m, agv)
    input("<<<")
    # planner.test()
    planner.run_agv0()
    time.sleep(t)
    planner.run_agv1()
    time.sleep(t)
    planner.run_agv2()
    time.sleep(t)
    planner.run_agv3()
    time.sleep(t)
    planner.run_agv4()
    time.sleep(t)
    planner.run_agv5()
    time.sleep(t)
    planner.run_agv6()
    time.sleep(t)
    planner.run_agv7()
    time.sleep(t)
    planner.run_agv8()
    time.sleep(t)
    planner.run_agv9()
    time.sleep(t)
    planner.run_agv10()
    time.sleep(t)
    planner.run_agv11()
    time.sleep(t)
    planner.run_agv12()
    time.sleep(t)
    planner.run_agv13()
    time.sleep(t)
    planner.run_agv14()
    time.sleep(t)
    planner.run_agv15()
    time.sleep(t)
    planner.run_agv16()
    time.sleep(t)
    planner.run_agv17()
    time.sleep(t)
    input("<<<")
