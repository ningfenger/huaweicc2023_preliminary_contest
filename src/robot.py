# coding=utf-8
from typing import Optional, List, Tuple


class Robot:
    def __init__(self, ID: int, loc: Tuple):
        self.ID = ID
        self.loc = loc
        self.speed = 0.0  # 线速度
        self.turn = 0.0  # 角速度
        self.workstation_ID = 0  # 所处工作台ID
        self.item_type = 0  # 携带物品类型
        self.time_value = 0.0  # 时间价值系数
        self.clash_value = 0.0  # 碰撞价值系数
        self.status: int = 0  # 0 空闲, 1 购买途中, 2 等待购买, 3 出售途中, 4 等待出售

    def update(self, str):
        # 根据判题器的输入更新机器人状态, 记得更新status
        pass

    # 四个动作
    def forward(self):
        pass

    def rotate(self):
        pass

    def buy(self):
        pass

    def destroy(self):
        pass
