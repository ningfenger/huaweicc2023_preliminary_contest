# coding=utf-8
import copy
import time
import numpy as np

# _r结尾 机器人特征
feature_workstand_id_r = 0
feature_materials_r = 1
feature_time_value_r = 2
feature_collision_r = 3
feature_ang_velo_r = 4
feature_line_velo_x_r = 5
feature_line_velo_y_r = 6
feature_theta_r = 7
feature_x_r = 8
feature_y_r = 9
feature_status_r = 10  # 0 空闲, 1 购买途中, 2 等待购买, 3 出售途中, 4 等待出售
feature_target_r = 11
feature_target_theta_r = 12
feature_target_buy_r = 13
feature_target_sell_r = 14


class RobotGroup:
    # 0 空闲, 1 购买途中, 2 等待购买, 3 出售途中, 4 等待出售
    FREE_STATUS = 0
    MOVE_TO_BUY_STATUS = 1
    WAIT_TO_BUY_STATUS = 2
    MOVE_TO_SELL_STATUS = 3
    WAIT_TO_SELL_STATUS = 4

    def __init__(self):
        self.group_info = np.zeros((4, 15))
        self.robots_plan = [[-1, -1] for _ in range(4)]  # 记录每个机器人的买和卖目标

    def add_init_location(self, idx_robot, x, y):
        # 判题器获取
        # 0-platform_id, 1-materials, 2-time_value_coef, 3-collision_value_coef,
        # 4-ang_velo, 5-line_velo_x, 6-line_velo_x, 7-theta, 8-x, 9-y

        # 自定义
        # 10-status, 11-target, 12-target_theta
        self.group_info[idx_robot, :] = np.array(
            [-1, x, y] + [0] * 12)

    def update_robot(self, id_robot, state_str):
        # 后面读地图 更新工作台状态
        # 输入：字符串

        # 字符串解析为变量
        str_list = state_str.split()
        float_list = [float(str_item) for str_item in str_list]
        self.group_info[id_robot, :10] = np.array(float_list)

    # 不可set的变量[0-9]
    def get_loc(self, idx_robot):
        if idx_robot == -1:
            return copy.deepcopy(self.group_info[:, [8, 9]])
        else:
            return copy.deepcopy(self.group_info[idx_robot, [8, 9]])

    # 自定义变量【10-14】
    def get_status(self, feature_id, idx_robot):
        # 获取指定机器人状态
        # idx_robot为-1表示获取所有机器人状态
        if idx_robot == -1:
            return copy.deepcopy(self.group_info[:, feature_id])
        else:
            return copy.deepcopy(self.group_info[idx_robot, feature_id])

    # 自定义变量【10-14】
    def set_status_item(self, feature_id, idx_robot, value):
        # 设定指定机器人状态
        self.group_info[idx_robot, feature_id] = value

    # 四个动作
    def forward(self, idx_robot, speed):
        '''
        设置前进速度，单位为米/秒。
        正数表示前进。
        负数表示后退。
        '''

        # # logging.info(f"forward {idx_robot} {speed}")
        print("forward", idx_robot, speed)

    def rotate(self, idx_robot, turn):
        '''
        设置旋转速度，单位为弧度/秒。
        负数表示顺时针旋转。
        正数表示逆时针旋转。
        '''
        # logging.info(f"rotate {idx_robot} {turn}")
        print("rotate", idx_robot, turn)

    def buy(self, idx_robot):
        '''
        购买当前工作台的物品，以输入数据的身处工作台 ID 为准。
        '''
        if self.get_status(feature_workstand_id_r, idx_robot) == self.get_status(feature_target_r, idx_robot):

            # logging.info(f"buy {idx_robot}")
            print("buy", idx_robot)
            return True
        else:
            return False

    def sell(self, idx_robot):
        '''
        出售物品给当前工作台，以输入数据的身处工作台 ID 为准。
        '''
        if self.get_status(feature_workstand_id_r, idx_robot) == self.get_status(feature_target_r, idx_robot):

            # logging.info(f"sell {idx_robot}")
            print("sell", idx_robot)
            return True
        else:
            return False

    def plan2target_buy(self, idx_robot):
        # 把规划的目标now变成现实的目标
        self.set_status_item(feature_target_r, idx_robot, self.get_status(feature_target_buy_r, idx_robot))

    def destroy(self, idx_robot):
        '''
        销毁物品。
        '''
        # logging.info("destroy", idx_robot)
        print("destroy", idx_robot)
