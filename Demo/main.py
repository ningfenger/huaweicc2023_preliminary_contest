#!/bin/bash
import sys
import logging
import numpy as np

class Map():
    def __init__(self):
        # 工作台信息 array
        self._platform = np.array([]).reshape(0, 6)

    def add_platform(self, num_type, x, y):
        # 第一次读地图添加工作台
        # 类型 x y 剩余生产时间 原材料格状态 产品格状态
        new_info = np.array([num_type, x, y, 0.0, 0.0, 0.0]).reshape(1, 6)
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
        self._platform[id_platform, :] = np.array(float_list)


class RobotGroup():
    def __init__(self):
        self.group_info = np.zeros((4, 10))

    def add_init_location(self, ID, x, y):
        self.group_info[ID, :] = np.array([np.nan, x, y, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan])

    def update_robot(self, id_robot, state_str):
        # 后面读地图 更新工作台状态
        # 输入：字符串

        # 字符串解析为变量
        logging.info(state_str)
        str_list = state_str.split()
        float_list = [float(str_item) for str_item in str_list]
        # platform_id, materials, time_value_coef, collision_value_coef, ang_velo, line_velo, theta, x, y = float_list
        self.group_info[id_robot, :] = np.array(float_list)


def read_map(map_in, robot_group_in):
    logging.info("========map init start========")
    num_robot = 0
    num_line = 0
    while True:
        line_read = input()
        if line_read == "OK":
            logging.info('========map init finish========')
            return
        else:
            for idx, point_str in enumerate(line_read):
                if point_str.isdigit() and 1 <= int(point_str) <= 9:
                    # 工作台
                    x = num_line * 0.5 + 0.25
                    y = idx * 0.5 + 0.25
                    map_in.add_platform(int(point_str), x, y)
                    logging.info('platform:' + str([int(point_str), x, y]))
                elif point_str == 'A':
                    # 机器人
                    x = num_line * 0.5 + 0.25
                    y = idx * 0.5 + 0.25
                    robot_group_in.add_init_location(num_robot, x, y)
                    logging.info('robot:' + str([num_robot, x, y]))
                    num_robot += 1
        num_line += 1


def get_info(map_in, robot_group):
    logging.info("========frame start========")
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
        logging.info("========frame finish========")
        return frame_id, money
    else:
        logging.info("========frame error========")


def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()


if __name__ == '__main__':
    logging.basicConfig(filename='LOG.log', encoding='utf-8', level=logging.DEBUG)
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
