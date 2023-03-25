# coding=utf-8
from workstand import Map
from robot import RobotGroup
from controller import Controller
from network import Network
import sys
import os
try:
    
    os.chdir('./src')
except:
    pass

import argparse


parser = argparse.ArgumentParser(description='华为软件精英挑战赛2023 调参版')
parser.add_argument('--move_speed', default=1 / 3 *
                    50,  type=float, help='估算移动速度')
parser.add_argument('--max_wait', default=2*50, type=float, help='最大等待帧数')
parser.add_argument('--sell_weight', default=1.4, type=float, help='优先生产权重')
parser.add_argument('--sell_debuff', default=0.6, type=float, help='优先生产权重')
parser.add_argument('--train', action="store_true",help='是否是训练模式')

def read_map(map_in: Map, robot_group_in: RobotGroup):
    num_robot = 0
    num_line = 0
    X = []
    while True:
        line_read = input()
        if line_read == "OK":
            return X
        else:
            for idx, point_str in enumerate(line_read):
                if point_str.isdigit() and 1 <= int(point_str) <= 9:
                    # 工作台
                    x = num_line * 0.5 + 0.25
                    y = idx * 0.5 + 0.25
                    map_in.add_workstand(int(point_str), x, y)
                    X.extend([num_line, idx, int(point_str)])
                elif point_str == 'A':
                    # 机器人
                    x = num_line * 0.5 + 0.25
                    y = idx * 0.5 + 0.25
                    robot_group_in.add_init_location(num_robot, x, y)
                    num_robot += 1
                    X.extend([num_line, idx, 10])

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
        # 到这就出错了
        pass


def finish():
    # logging.info('OK\n')
    sys.stdout.write('OK\n')
    sys.stdout.flush()


if __name__ == '__main__':
    # logging.basicConfig(filename='log/log.log', level=# logging.DEBUG)
    # logging.info('===================')
    args = parser.parse_args()
    robot_group_obj = RobotGroup()
    map_obj = Map()
    # time.sleep(20)
    X = read_map(map_obj, robot_group_obj)
    controller = Controller(robot_group_obj, map_obj)
    controller.init_ITEMS_NEED()
    # time.sleep(20)
    # 只需计算一次
    controller.cal_dis_workstand2workstand()
    network = Network()
    network.weight_loader()
    if args.train:
        controller.set_control_parameters(args.move_speed,  int(args.max_wait), args.sell_weight, args.sell_debuff)
    else:
        # pass
        MOVE_SPEED, MAX_WAIT, SELL_WEIGHT, SELL_DEBUFF = network.get_params(X)
        controller.set_control_parameters(MOVE_SPEED,  MAX_WAIT, SELL_WEIGHT, SELL_DEBUFF)
    finish()
    while True:
        try:
            frame_id, money = get_info(map_obj, robot_group_obj)
        except:
            break
        controller.cal_dis_robot2workstand()
        controller.cal_dis_robot2robot()

        # logging.info(frame_id)
        print(frame_id)
        controller.control(int(frame_id))

        finish()
