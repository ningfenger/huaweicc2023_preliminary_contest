#!/bin/bash
# coding=utf-8
import sys
import logging
import numpy as np

DISMAP = None  # 初始化时更新，记录任意两个工作台间的测算距离/帧数
ITEMS_BUY = [0, 3000, 4400, 5800, 15400, 17200, 19200, 76000]  # 每个物品的购买价
ITEMS_SELL = [0, 6000, 7600, 9200, 22500, 25000, 27500, 105000]
ITEMS_NEED = [] + [[] for _ in range(7)]  # 记录收购每个商品的工作台编号
WORKSTAND_IN = {1: [], 2: [], 3: [], 4: [1, 2], 5: [1, 3], 6: [2, 3], 7: [4, 5, 6], 8: [7], 9: list(range(1, 8))}
WORKSTAND_OUT = {i: i for i in range(1, 8)}
WORKSTAND_OUT[8] = None
WORKSTAND_OUT[9] = None


class Controller:
    def __init__(self, robots, workstands):
        self.robots = robots
        self.workstands = workstands

    def move(self):
        pass

    def get_time(self, idx_robot, idx_workstand):  # 机器人到工作台的时间预估
        pass

    def get_distime(self, idx_workstand, idx_workstand2): # 两个工作台间的时间预估
        pass


class Map:
    def __init__(self):
        # 工作台信息 array
        self._platform = np.array([]).reshape(0, 8)

        # self.material_pro = 0  # 原料格预定状态, 防止有多个机器人将其作为出售目标 get set
        # self.product_pro = 0  # 防止有多个机器人将其作为购买目标 get set

    def add_platform(self, num_type, x, y):
        # 第一次读地图添加工作台
        # 类型 x y 剩余生产时间 原材料格状态 产品格状态
        new_info = np.array([num_type, x, y, 0.0, 0.0, 0.0] + [np.nan] * 2).reshape(1, 8)
        self._platform = np.concatenate([self._platform, new_info], axis=0)

    def update_platform(self, id_platform, state_str):
        # 后面读地图 更新工作台状态
        # 输入：字符串

        # 字符串解析为变量
        str_list = state_str.split()
        float_list = [float(str_item) for str_item in str_list]
        # num_type, x, y, waiting_time, raw_state, product_state = float_list
        logging.info(str(self._platform.shape))
        logging.info(str(np.array(float_list).shape))
        self._platform[id_platform, :6] = np.array(float_list)

    def set_material_pro(self, id_platform, value):
        # 预售接口 如果是89特殊处理
        self._platform[id_platform, -2] = value

    def set_product_pro(self, id_platform, value):
        # 预购接口, 如果是123特殊处理
        self._platform[id_platform, -1] = value

    def get_material_pro(self, id_platform):
        # 原料格预定状态, 防止有多个机器人将其作为出售目标
        return self._platform[id_platform, -2]

    def get_product_pro(self, id_platform):
        # 防止有多个机器人将其作为购买目标
        return self._platform[id_platform, -1]

    # self.tpye = type  # 类型
    # self.product_time = 0  # 剩余生产时间
    # self.material = 0  # 原材料格状态
    # self.product_status = 0  # 产品格状态
    # 给我get


class RobotGroup:
    def __init__(self):
        self.group_info = np.zeros((4, 10))

    def add_init_location(self, id_robot, x, y):
        self.group_info[id_robot, :] = np.array([np.nan, x, y] + [np.nan] * 7)

    def update_robot(self, id_robot, state_str):
        # 后面读地图 更新工作台状态
        # 输入：字符串

        # 字符串解析为变量
        logging.info(state_str)
        str_list = state_str.split()
        float_list = [float(str_item) for str_item in str_list]
        # platform_id, materials, time_value_coef, collision_value_coef, ang_velo, line_velo, theta, x, y = float_list
        self.group_info[id_robot, :10] = np.array(float_list)

    # 四个动作
    def forward(self, id_robot, speed):
        '''
        设置前进速度，单位为米/秒。
        正数表示前进。
        负数表示后退。
        '''
        pass

    def rotate(self, id_robot, turn):
        '''
        设置旋转速度，单位为弧度/秒。
        负数表示顺时针旋转。
        正数表示逆时针旋转。
        '''
        pass

    def buy(self, id_robot):
        '''
        购买当前工作台的物品，以输入数据的身处工作台 ID 为准。
        '''
        pass

    def sell(self, id_robot):
        '''
        出售物品给当前工作台，以输入数据的身处工作台 ID 为准。
        '''
        pass

    def destroy(self, id_robot):
        '''
        销毁物品。
        '''
        pass


def read_map(map_in, robot_group_in):
    # logging.info("========map init start========")
    num_robot = 0
    num_line = 0
    while True:
        line_read = input()
        if line_read == "OK":
            return
        else:
            for idx, point_str in enumerate(line_read):
                if point_str.isdigit() and 1 <= int(point_str) <= 9:
                    # 工作台
                    x = num_line * 0.5 + 0.25
                    y = idx * 0.5 + 0.25
                    map_in.add_platform(int(point_str), x, y)
                    # logging.info('platform:' + str([int(point_str), x, y]))
                elif point_str == 'A':
                    # 机器人
                    x = num_line * 0.5 + 0.25
                    y = idx * 0.5 + 0.25
                    robot_group_in.add_init_location(num_robot, x, y)
                    # logging.info('robot:' + str([num_robot, x, y]))
                    num_robot += 1
        num_line += 1


def get_info(map_in, robot_group):
    line_read = input()
    frame_id, money = line_read.split(' ')

    line_read = input()
    num_robot = 4
    num_platform = int(line_read)

    # 更新工作台
    for id_platform in range(num_platform):
        line_read = input()
        map_in.update_platform(id_platform, line_read)

    # 更新机器人
    for id_robot in range(num_robot):
        line_read = input()
        robot_group.update_robot(id_robot, line_read)

    line_read = input()
    if line_read == 'OK':
        return frame_id, money
    else:
        pass


def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()


if __name__ == '__main__':
    map_obj = Map()
    robot_group_obj = RobotGroup()

    read_map(map_obj, robot_group_obj)
    finish()
    while True:
        frame_id, money = get_info(map_obj, robot_group_obj)

        print(frame_id)
        line_speed, angle_speed = 3, 1.5
        for robot_id in range(4):
            sys.stdout.write('forward %d %d\n' % (robot_id, line_speed))
            sys.stdout.write('rotate %d %f\n' % (robot_id, angle_speed))
        finish()
