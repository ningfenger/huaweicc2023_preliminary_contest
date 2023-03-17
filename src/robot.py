# coding=utf-8
from typing import Optional, List, Tuple


class Robot:
    def __init__(self, ID: int, loc: Tuple):
        self.ID = ID
        self.loc = loc
        self.speed = 0.0  # 线速度
        self.turn = 0.0  # 角速度
        self.workstation_ID = -1  # 所处工作台ID代表没有
        self.item_type = 0  # 携带物品类型
        self.time_value = 0.0  # 时间价值系数
        self.clash_value = 0.0  # 碰撞价值系数
        self.status: int = 0  # 0 空闲, 1 购买途中, 2 等待购买, 3 出售途中, 4 等待出售
        self.target = -1 # 当前机器人的目标控制台 -1代表无目标

    def update(self, str):
        # 根据判题器的输入更新机器人状态, 记得更新status
        pass

    # 四个动作
    def forward(self, speed):
        '''
        设置前进速度，单位为米/秒。
        正数表示前进。 
        负数表示后退。
        '''
        pass

    def rotate(self, turn):
        '''
        设置旋转速度，单位为弧度/秒。
        负数表示顺时针旋转。
        正数表示逆时针旋转。
        '''
        pass

    def buy(self):
        '''
        购买当前工作台的物品，以输入数据的身处工作台 ID 为准。
        '''
        pass

    def sell(self):
        '''
        出售物品给当前工作台，以输入数据的身处工作台 ID 为准。
        '''
        pass
    
    def destroy(self):
        '''
        销毁物品。
        '''
        pass
