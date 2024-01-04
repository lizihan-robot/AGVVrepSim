import numpy as np
import json

from vrep_client import VREP



class Map():
    mark_name = "Plane"
    def __init__(self) -> None:
        super().__init__()
        _mark_matrix = self._load_plane_matrix()
        self.mark_matrix = np.array(_mark_matrix)

    @classmethod
    def save_plane_matrix(cls, name, path="D:\\pyproject\\vrep\\plane_matrix.json", row=14, col=14):
        print("creat mark matrix")
        v = VREP()
        matrix = np.zeros((row, col), dtype=object)
        for i in range(0, row):
            for j in range(0, col):
                dict_key = cls.mark_name + str(i) + "_" + str(j)
                h = v.get_handle(dict_key)
                res = v.get_pose(h)
                matrix[i, j] = res
                # matrix[i, j] =  {dict_key:res}

                # 转换成4*4矩阵
                # _matrix = np.eye(4)
                # np.set_printoptions(precision=3, floatmode='fixed')
                # _matrix[0:3, 3] = np.array(res)
                # matrix[i, j] =  {dict_key:_matrix}
        m = Map()
        js = {"plane_matrix": matrix.tolist(),"name":name}
        with open(path, "w") as f:
            json.dump(js, f)
        return m

    def _load_plane_matrix(self, path="D:\\pyproject\\vrep\\plane_matrix.json"):
        with open(path, "r") as f:
            data = json.load(f)
        return data["plane_matrix"]

    def get_plane_pose(self, mark_name):
        print("[MAP]: get mark pose-- ",mark_name)
        i = int(mark_name[mark_name.index('e')+1:mark_name.index('_')])
        j = int(mark_name[mark_name.index('_')+1:])
        return self.mark_matrix[i, j]


if __name__=="__main__":
    Map().save_plane_matrix("new_agv")