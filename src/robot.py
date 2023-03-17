# coding=utf-8
from typing import Optional, List, Tuple
import numpy as np


class Robot:
    ROBOT_MOVING=
    def __init__(self, ID: int, loc: Tuple):
        self.ID = ID
        self.loc = loc
        self.speed = 0.0  # 线速度
        self.turn = 0.0  # 角速度
        self.workstation_ID = -1  # 所处工作台ID代表没有
        self.item_type = 0  # 携带物品类型
        self.time_value = 0.0  # 时间价值系数
        self.clash_value = 0.0  # 碰撞价值系数
        self.status: int = 0  # 0 空闲, 1 购买途中, 2 等待购买, 3 出售途中, 4 等待出售 # get set
        self.target = None  # 当前机器人的目标工作台的坐标 None代表无目标 *** get set
        self.target_theta = np.nan

    def get_loc(self):
        return
    def update(self, str):
        # 根据判题器的输入更新机器人状态, 记得更新status
        pass


